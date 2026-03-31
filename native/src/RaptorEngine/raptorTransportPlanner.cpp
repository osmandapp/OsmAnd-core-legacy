#ifndef _OSMAND_RAPTOR_ENGINE_TRANSPORT_PLANNER_CPP
#define _OSMAND_RAPTOR_ENGINE_TRANSPORT_PLANNER_CPP

#include "RaptorEngine/raptorTransportPlanner.h"
#include "RaptorEngine/TransportContextBridge.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "Logging.h"
#include "routeCalculationProgress.h"
#include "transportRouteResult.h"
#include "transportRouteResultSegment.h"
#include "transportRouteSegment.h"
#include "transportRoutingConfiguration.h"
#include "transportRoutingContext.h"
#include "transportRoutingObjects.h"

#include "RaptorEngine/RaptorEngine.h"

namespace
{
	constexpr int32_t kWalkRouteTypeIndex = 0;
	constexpr double kInfinity = std::numeric_limits<double>::infinity();

	struct InputStopRecord
	{
		SHARED_PTR<TransportStop> stop;
		int32_t x31 = 0;
		int32_t y31 = 0;
	};

	struct PackedInputRoute
	{
		SHARED_PTR<TransportRoute> route;
		int32_t routeTypeIndex = kWalkRouteTypeIndex;
		double defaultHeadwaySeconds = 0.0;
		double boardingCostSeconds = 0.0;
		std::vector<int32_t> stopIndices;
		std::vector<int32_t> originalStopPositions;
		std::vector<double> cumulativeTravelDistancesMeters;
		std::vector<double> cumulativeCompatTravelSeconds;
		std::vector<int32_t> cumulativeScheduleOffsetsDeciSeconds;
		std::vector<int32_t> scheduleTripStartsDeciSeconds;
	};

	struct FootpathEdge
	{
		int32_t targetStopIndex = -1;
		double walkTimeSeconds = 0.0;
		double walkDistanceMeters = 0.0;
	};

	struct ReachableRouteSeed
	{
		SHARED_PTR<TransportRoute> route;
		int32_t depth = std::numeric_limits<int32_t>::max();
		int32_t minBoardStop = std::numeric_limits<int32_t>::max();
	};

	struct PackedBuildState
	{
		OsmAnd::RaptorEngine::PackedTransportSnapshot snapshot;
		OsmAnd::RaptorEngine::RaptorQueryOptions queryOptions;
		std::vector<SHARED_PTR<TransportRoute>> routesByIndex;
		std::vector<std::vector<int32_t>> routeOriginalStopPositionsByIndex;
		std::vector<InputStopRecord> stopsByIndex;
	};

	static bool isCancelled(const unique_ptr<TransportRoutingContext>& ctx)
	{
		return ctx->calculationProgress != nullptr && ctx->calculationProgress->isCancelled();
	}

	static void throwIfCancelled(const unique_ptr<TransportRoutingContext>& ctx, const char* stage)
	{
		if (isCancelled(ctx))
			throw std::runtime_error(std::string("RaptorTransportPlanner cancelled during ") + stage);
	}

	static std::string normalizeRouteTypeKey(const std::string& routeType)
	{
		std::string normalized = routeType;
		std::transform(normalized.begin(), normalized.end(), normalized.begin(), [](unsigned char ch) {
			return static_cast<char>(std::tolower(ch));
		});
		return normalized.empty() ? std::string("default") : normalized;
	}

	static uint64_t makeStopKey(const SHARED_PTR<TransportStop>& stop)
	{
		if (!stop)
			return 0;
		if (stop->id != 0)
			return static_cast<uint64_t>(stop->id);
		return static_cast<uint64_t>(reinterpret_cast<uintptr_t>(stop.get()));
	}

	static uint64_t makeBucketKey(const int32_t x, const int32_t y)
	{
		return (static_cast<uint64_t>(static_cast<uint32_t>(x)) << 32) |
		       static_cast<uint32_t>(y);
	}

	static int32_t clampAreaCoordinate(const int64_t value)
	{
		return static_cast<int32_t>(std::max<int64_t>(0, std::min<int64_t>(INT32_MAX, value)));
	}

	static int32_t metersTo31Units(const double meters)
	{
		if (!(meters > 0.0))
			return 0;

		const auto metersPer31 = TransportRoutingContext::getTileDistanceWidth(31);
		if (!(metersPer31 > 0.0))
			return 0;

		return static_cast<int32_t>(std::ceil(meters / metersPer31));
	}

	static double calculateCorridorPaddingMeters(const unique_ptr<TransportRoutingContext>& ctx)
	{
		const auto directDistanceMeters = getDistance(ctx->startLat, ctx->startLon, ctx->endLat, ctx->endLon);
		const auto maxRounds = std::max(1, ctx->cfg->maxNumberOfChanges + 1);
		const auto changePaddingMeters = static_cast<double>(std::max(0, ctx->cfg->walkChangeRadius)) * maxRounds * 2.0;
		const auto endpointPaddingMeters =
			static_cast<double>(std::max(ctx->cfg->walkRadius, ctx->cfg->walkChangeRadius)) * 2.0;
		const auto detourPaddingMeters = std::max(12000.0, directDistanceMeters * 0.25);
		const auto interchangePaddingMeters = std::min(30000.0, 4000.0 * maxRounds);
		return std::max(std::max(endpointPaddingMeters, changePaddingMeters),
		                std::max(detourPaddingMeters, interchangePaddingMeters));
	}

	static double calculateComfortTimeWindowSeconds(const double bestTimeSeconds)
	{
		if (!(bestTimeSeconds > 0.0) || !std::isfinite(bestTimeSeconds))
			return 600.0;
		return std::min(1800.0, std::max(600.0, bestTimeSeconds * 0.15));
	}

	static bool isAlmostEqual(const double left, const double right)
	{
		return std::abs(left - right) <= 1e-9;
	}

	static double calculateRankingTimeBandSeconds(const double bestTimeSeconds)
	{
		if (!(bestTimeSeconds > 0.0) || !std::isfinite(bestTimeSeconds))
			return 300.0;
		return std::min(900.0, std::max(300.0, bestTimeSeconds * 0.08));
	}

	static int32_t calculateTimeBandIndex(const double totalTimeSeconds,
	                                      const double bestTimeSeconds,
	                                      const double timeBandSeconds)
	{
		if (!std::isfinite(totalTimeSeconds))
			return std::numeric_limits<int32_t>::max();

		const auto timeDeltaSeconds = std::max(0.0, totalTimeSeconds - bestTimeSeconds);
		if (timeDeltaSeconds <= timeBandSeconds + 1e-9)
			return 0;

		return 1 + static_cast<int32_t>(
			std::floor((timeDeltaSeconds - timeBandSeconds - 1e-9) / timeBandSeconds));
	}

	static double calculateComfortScoreSeconds(const SHARED_PTR<TransportRouteResult>& route)
	{
		if (route == nullptr)
			return kInfinity;

		const auto transfers = std::max(0, static_cast<int32_t>(route->segments.size()) - 1);
		const auto segments = static_cast<int32_t>(route->segments.size());
		return route->routeTime +
		       static_cast<double>(transfers) * 480.0 +
		       static_cast<double>(segments) * 45.0 +
		       route->getWalkDist() * 0.04;
	}

	static std::vector<SHARED_PTR<TransportStop>> loadCorridorStops(unique_ptr<TransportRoutingContext>& ctx)
	{
		const auto padding31 = static_cast<int64_t>(metersTo31Units(calculateCorridorPaddingMeters(ctx)));
		return OsmAnd::RaptorEngine::TransportContextBridge::loadMergedTransportStops(
			*ctx,
			clampAreaCoordinate(static_cast<int64_t>(std::min(ctx->startX, ctx->targetX)) - padding31),
			clampAreaCoordinate(static_cast<int64_t>(std::max(ctx->startX, ctx->targetX)) + padding31),
			clampAreaCoordinate(static_cast<int64_t>(std::min(ctx->startY, ctx->targetY)) - padding31),
			clampAreaCoordinate(static_cast<int64_t>(std::max(ctx->startY, ctx->targetY)) + padding31));
	}

	static int32_t ensureStopIndex(const SHARED_PTR<TransportStop>& stop,
	                               PackedBuildState& state,
	                               std::unordered_map<uint64_t, int32_t>& stopIndexById)
	{
		const auto stopKey = makeStopKey(stop);
		const auto existing = stopIndexById.find(stopKey);
		if (existing != stopIndexById.end())
			return existing->second;

		const auto stopIndex = static_cast<int32_t>(state.snapshot.stopIds.size());
		state.snapshot.stopIds.push_back(static_cast<int64_t>(stop->id));
		state.snapshot.stopX31.push_back(stop->x31);
		state.snapshot.stopY31.push_back(stop->y31);
		state.stopsByIndex.push_back({stop, stop->x31, stop->y31});
		stopIndexById.emplace(stopKey, stopIndex);
		return stopIndex;
	}

	static int32_t ensureRouteTypeIndex(const std::string& routeType,
	                                    std::unordered_map<std::string, int32_t>& routeTypeIndexByKey,
	                                    std::vector<std::string>& routeTypeKeys,
	                                    std::vector<double>& routeTypeBoardingSeconds,
	                                    const SHARED_PTR<TransportRoutingConfiguration>& cfg)
	{
		const auto typeKey = normalizeRouteTypeKey(routeType);
		const auto existing = routeTypeIndexByKey.find(typeKey);
		if (existing != routeTypeIndexByKey.end())
			return existing->second;

		const auto routeTypeIndex = static_cast<int32_t>(routeTypeKeys.size());
		routeTypeIndexByKey.emplace(typeKey, routeTypeIndex);
		routeTypeKeys.push_back(typeKey);
		routeTypeBoardingSeconds.push_back(cfg->useSchedule
			                                   ? 0.0
			                                   : static_cast<double>(cfg->getBoardingTime(routeType)));
		return routeTypeIndex;
	}

	static PackedInputRoute buildPackedRoute(const SHARED_PTR<TransportRoute>& route,
	                                          PackedBuildState& state,
	                                          std::unordered_map<uint64_t, int32_t>& stopIndexById,
	                                          std::unordered_map<std::string, int32_t>& routeTypeIndexByKey,
	                                          std::vector<std::string>& routeTypeKeys,
	                                          std::vector<double>& routeTypeBoardingSeconds,
	                                          const SHARED_PTR<TransportRoutingConfiguration>& cfg)
	{
		PackedInputRoute packedRoute;
		packedRoute.route = route;
		packedRoute.routeTypeIndex =
			ensureRouteTypeIndex(route->getType(), routeTypeIndexByKey, routeTypeKeys, routeTypeBoardingSeconds, cfg);
		packedRoute.defaultHeadwaySeconds = static_cast<double>(route->calcIntervalInSeconds());
		packedRoute.boardingCostSeconds =
			cfg->useSchedule
				? 0.0
				: packedRoute.defaultHeadwaySeconds > 0.0
				? packedRoute.defaultHeadwaySeconds * 0.5
				: static_cast<double>(cfg->getBoardingTime(route->getType()));

		const auto routeTravelSpeed = static_cast<double>(cfg->getSpeedByRouteType(route->getType()));
		if (!(routeTravelSpeed > 0.0) && !cfg->useSchedule)
			return packedRoute;

		const auto stopTimeSeconds = static_cast<double>(cfg->getStopTime(route->getType()));
		packedRoute.stopIndices.reserve(route->forwardStops.size());
		packedRoute.originalStopPositions.reserve(route->forwardStops.size());

		for (int32_t originalStopPosition = 0; originalStopPosition < static_cast<int32_t>(route->forwardStops.size());
		     ++originalStopPosition)
		{
			const auto& stop = route->forwardStops[originalStopPosition];
			if (!stop)
				continue;

			const auto stopIndex = ensureStopIndex(stop, state, stopIndexById);
			if (!packedRoute.stopIndices.empty() && packedRoute.stopIndices.back() == stopIndex)
				continue;
			packedRoute.stopIndices.push_back(stopIndex);
			packedRoute.originalStopPositions.push_back(originalStopPosition);
		}

		if (packedRoute.stopIndices.size() < 2)
		{
			packedRoute.stopIndices.clear();
			packedRoute.originalStopPositions.clear();
			return packedRoute;
		}

		packedRoute.cumulativeTravelDistancesMeters.resize(packedRoute.stopIndices.size(), 0.0);
		packedRoute.cumulativeCompatTravelSeconds.resize(packedRoute.stopIndices.size(), 0.0);
		packedRoute.cumulativeScheduleOffsetsDeciSeconds.resize(packedRoute.stopIndices.size(), 0);
		for (size_t stopPos = 1; stopPos < packedRoute.stopIndices.size(); ++stopPos)
		{
			const auto& prevStop = state.stopsByIndex[packedRoute.stopIndices[stopPos - 1]].stop;
			const auto& stop = state.stopsByIndex[packedRoute.stopIndices[stopPos]].stop;
			const auto segmentDistanceMeters = getDistance(prevStop->lat, prevStop->lon, stop->lat, stop->lon);
			const auto previousOriginalStopPosition = packedRoute.originalStopPositions[stopPos - 1];
			const auto currentOriginalStopPosition = packedRoute.originalStopPositions[stopPos];

			packedRoute.cumulativeTravelDistancesMeters[stopPos] =
				packedRoute.cumulativeTravelDistancesMeters[stopPos - 1] + segmentDistanceMeters;
			packedRoute.cumulativeCompatTravelSeconds[stopPos] =
				packedRoute.cumulativeCompatTravelSeconds[stopPos - 1] +
				(cfg->useSchedule && !(routeTravelSpeed > 0.0)
					 ? 0.0
					 : stopTimeSeconds + segmentDistanceMeters / routeTravelSpeed);

			int32_t scheduleIncrement = 0;
			for (int32_t intervalIndex = previousOriginalStopPosition; intervalIndex < currentOriginalStopPosition; ++intervalIndex)
			{
				if (intervalIndex >= 0 &&
				    intervalIndex < static_cast<int32_t>(route->schedule.avgStopIntervals.size()))
				{
					scheduleIncrement += route->schedule.avgStopIntervals[intervalIndex];
				}
			}
			packedRoute.cumulativeScheduleOffsetsDeciSeconds[stopPos] =
				packedRoute.cumulativeScheduleOffsetsDeciSeconds[stopPos - 1] + scheduleIncrement;
			if (cfg->useSchedule && !(routeTravelSpeed > 0.0))
			{
				packedRoute.cumulativeCompatTravelSeconds[stopPos] =
					packedRoute.cumulativeCompatTravelSeconds[stopPos - 1] + scheduleIncrement * 10.0;
			}
		}

		if (cfg->useSchedule)
		{
			int32_t tripStartDeciSeconds = 0;
			const auto scheduleWindowStart = cfg->scheduleTimeOfDay;
			const auto scheduleWindowEnd = cfg->scheduleTimeOfDay + cfg->scheduleMaxTime;
			for (const auto tripInterval : route->schedule.tripIntervals)
			{
				tripStartDeciSeconds += tripInterval;
				bool insideWindow = false;
				for (const auto cumulativeOffset : packedRoute.cumulativeScheduleOffsetsDeciSeconds)
				{
					const auto departureAtStop = tripStartDeciSeconds + cumulativeOffset;
					if (departureAtStop >= scheduleWindowStart && departureAtStop <= scheduleWindowEnd)
					{
						insideWindow = true;
						break;
					}
				}
				if (insideWindow)
					packedRoute.scheduleTripStartsDeciSeconds.push_back(tripStartDeciSeconds - cfg->scheduleTimeOfDay);
			}

			if (packedRoute.scheduleTripStartsDeciSeconds.empty())
			{
				packedRoute.stopIndices.clear();
				packedRoute.originalStopPositions.clear();
			}
		}

		return packedRoute;
	}

	static void fillReachability(const int32_t pointX31,
	                             const int32_t pointY31,
	                             const int32_t walkRadius,
	                             const double walkSpeed,
	                             const PackedBuildState& state,
	                             std::vector<double>& walkSeconds,
	                             std::vector<double>& walkDistancesMeters)
	{
		walkSeconds.assign(state.snapshot.stopCount, kInfinity);
		walkDistancesMeters.assign(state.snapshot.stopCount, 0.0);

		if (!(walkSpeed > 0.0) || walkRadius <= 0)
			return;

		for (int32_t stopIndex = 0; stopIndex < state.snapshot.stopCount; ++stopIndex)
		{
			const auto& stop = state.stopsByIndex[stopIndex].stop;
			const auto walkDistanceMeters =
				getDistance(get31LatitudeY(pointY31), get31LongitudeX(pointX31), stop->lat, stop->lon);
			if (walkDistanceMeters > walkRadius)
				continue;

			walkDistancesMeters[stopIndex] = walkDistanceMeters;
			walkSeconds[stopIndex] = walkDistanceMeters / walkSpeed;
		}
	}

	static void buildFootpaths(const unique_ptr<TransportRoutingContext>& ctx,
	                           PackedBuildState& state)
	{
		auto& snapshot = state.snapshot;
		snapshot.transitData.footpathOffsets.clear();
		snapshot.transitData.footpathTargetIndices.clear();
		snapshot.transitData.footpathTimesSeconds.clear();
		snapshot.transitData.footpathDistancesMeters.clear();
		snapshot.transitData.footpathOffsets.push_back(0);

		if (snapshot.stopCount == 0 || ctx->cfg->walkChangeRadius <= 0 || !(ctx->cfg->walkSpeed > 0.0f))
		{
			snapshot.transitData.footpathOffsets.assign(static_cast<size_t>(snapshot.stopCount) + 1, 0);
			return;
		}

		const auto bucketSize31 = std::max(1, ctx->walkChangeRadiusIn31);
		std::unordered_map<uint64_t, std::vector<int32_t>> buckets;
		buckets.reserve(static_cast<size_t>(snapshot.stopCount) * 2u);
		for (int32_t stopIndex = 0; stopIndex < snapshot.stopCount; ++stopIndex)
		{
			const auto bucketX = snapshot.stopX31[stopIndex] / bucketSize31;
			const auto bucketY = snapshot.stopY31[stopIndex] / bucketSize31;
			buckets[makeBucketKey(bucketX, bucketY)].push_back(stopIndex);
		}

		std::vector<std::vector<FootpathEdge>> footpathsByStop(static_cast<size_t>(snapshot.stopCount));
		for (int32_t stopIndex = 0; stopIndex < snapshot.stopCount; ++stopIndex)
		{
			const auto sourceBucketX = snapshot.stopX31[stopIndex] / bucketSize31;
			const auto sourceBucketY = snapshot.stopY31[stopIndex] / bucketSize31;
			const auto& sourceStop = state.stopsByIndex[stopIndex].stop;

			for (int32_t deltaX = -1; deltaX <= 1; ++deltaX)
			{
				for (int32_t deltaY = -1; deltaY <= 1; ++deltaY)
				{
					const auto bucketIt = buckets.find(makeBucketKey(sourceBucketX + deltaX, sourceBucketY + deltaY));
					if (bucketIt == buckets.end())
						continue;

					for (const auto targetStopIndex : bucketIt->second)
					{
						if (targetStopIndex <= stopIndex)
							continue;

						const auto& targetStop = state.stopsByIndex[targetStopIndex].stop;
						const auto walkDistanceMeters =
							getDistance(sourceStop->lat, sourceStop->lon, targetStop->lat, targetStop->lon);
						if (walkDistanceMeters > ctx->cfg->walkChangeRadius)
							continue;

						const auto walkTimeSeconds = walkDistanceMeters / ctx->cfg->walkSpeed;
						footpathsByStop[stopIndex].push_back({targetStopIndex, walkTimeSeconds, walkDistanceMeters});
						footpathsByStop[targetStopIndex].push_back({stopIndex, walkTimeSeconds, walkDistanceMeters});
					}
				}
			}
		}

		snapshot.transitData.footpathOffsets.clear();
		snapshot.transitData.footpathOffsets.reserve(static_cast<size_t>(snapshot.stopCount) + 1);
		snapshot.transitData.footpathOffsets.push_back(0);
		for (auto& stopFootpaths : footpathsByStop)
		{
			std::sort(stopFootpaths.begin(), stopFootpaths.end(), [](const FootpathEdge& left, const FootpathEdge& right) {
				return left.targetStopIndex < right.targetStopIndex;
			});
			for (const auto& edge : stopFootpaths)
			{
				snapshot.transitData.footpathTargetIndices.push_back(edge.targetStopIndex);
				snapshot.transitData.footpathTimesSeconds.push_back(edge.walkTimeSeconds);
				snapshot.transitData.footpathDistancesMeters.push_back(edge.walkDistanceMeters);
			}
			snapshot.transitData.footpathOffsets.push_back(
				static_cast<int32_t>(snapshot.transitData.footpathTargetIndices.size()));
		}
	}

	static void buildChangeMatrix(const SHARED_PTR<TransportRoutingConfiguration>& cfg,
	                              const std::vector<std::string>& routeTypeKeys,
	                              OsmAnd::RaptorEngine::RaptorQueryOptions& queryOptions)
	{
		const auto routeTypeCount = static_cast<int32_t>(routeTypeKeys.size());
		queryOptions.changeSecondsMatrix.assign(static_cast<size_t>(routeTypeCount) * routeTypeCount, 0.0);
		for (int32_t fromType = 0; fromType < routeTypeCount; ++fromType)
		{
			for (int32_t toType = 0; toType < routeTypeCount; ++toType)
			{
				if (fromType == kWalkRouteTypeIndex || toType == kWalkRouteTypeIndex)
					continue;
				queryOptions.changeSecondsMatrix[static_cast<size_t>(fromType) * routeTypeCount + toType] =
					static_cast<double>(cfg->getChangeTime(routeTypeKeys[fromType], routeTypeKeys[toType]));
			}
		}
	}

	static double calculateRouteLimitSeconds(const unique_ptr<TransportRoutingContext>& ctx)
	{
		double finishTime = static_cast<double>(ctx->cfg->maxRouteTime);
		const auto totalDistance = getDistance(ctx->startLat, ctx->startLon, ctx->endLat, ctx->endLon);
		if (totalDistance > ctx->cfg->maxRouteDistance && ctx->cfg->maxRouteIncreaseSpeed > 0)
		{
			const auto increaseTime =
				(totalDistance - ctx->cfg->maxRouteDistance) * 3.6 / ctx->cfg->maxRouteIncreaseSpeed;
			finishTime += increaseTime;
		}

		double walkOnlyTime = kInfinity;
		if (ctx->cfg->walkSpeed > 0.0f)
			walkOnlyTime = totalDistance / ctx->cfg->walkSpeed;

		const auto alternativeLimit = finishTime * ctx->cfg->increaseForAlternativesRoutes;
		return std::min(alternativeLimit, walkOnlyTime);
	}

	static int32_t calculateJourneyCandidateLimit(const unique_ptr<TransportRoutingContext>& ctx)
	{
		const auto limitByNumber = std::max(1, ctx->cfg->ptLimitResultsByNumber);
		const auto maxChanges = std::max(1, ctx->cfg->maxNumberOfChanges);
		const auto optimalLimit = 100 * limitByNumber * maxChanges;
		return std::min(std::max(4000, optimalLimit), 20000);
	}

	static std::vector<ReachableRouteSeed> collectReachableRoutes(unique_ptr<TransportRoutingContext>& ctx)
	{
		std::vector<ReachableRouteSeed> seeds;
		std::unordered_map<int64_t, int32_t> seedIndexByRouteId;
		const auto corridorStops = loadCorridorStops(ctx);
		seeds.reserve(corridorStops.size());

		for (const auto& stop : corridorStops)
		{
			throwIfCancelled(ctx, "corridor route extraction");
			if (!stop || stop->isDeleted())
				continue;

			for (const auto& route : stop->routes)
			{
				if (!route || route->forwardStops.size() < 2)
					continue;

				const auto inserted = seedIndexByRouteId.emplace(route->id, static_cast<int32_t>(seeds.size()));
				if (!inserted.second)
					continue;

				ReachableRouteSeed seed;
				seed.route = route;
				seed.depth = 1;
				seed.minBoardStop = 0;
				seeds.push_back(std::move(seed));
			}
		}

		std::sort(seeds.begin(), seeds.end(), [](const ReachableRouteSeed& left, const ReachableRouteSeed& right) {
			if (!left.route || !right.route)
				return static_cast<bool>(left.route);
			return left.route->id < right.route->id;
		});
		return seeds;
	}

	static PackedBuildState buildPackedState(unique_ptr<TransportRoutingContext>& ctx)
	{
		PackedBuildState state;
		auto& snapshot = state.snapshot;
		auto& queryOptions = state.queryOptions;

		std::unordered_map<uint64_t, int32_t> stopIndexById;
		std::unordered_map<std::string, int32_t> routeTypeIndexByKey;
		std::vector<std::string> routeTypeKeys;
		std::vector<double> routeTypeBoardingSeconds;
		std::vector<PackedInputRoute> packedRoutes;

		routeTypeIndexByKey.emplace("walk", kWalkRouteTypeIndex);
		routeTypeKeys.push_back("walk");
		routeTypeBoardingSeconds.push_back(0.0);

		const auto reachableRoutes = collectReachableRoutes(ctx);
		packedRoutes.reserve(reachableRoutes.size());
		state.routesByIndex.reserve(reachableRoutes.size());
		state.routeOriginalStopPositionsByIndex.reserve(reachableRoutes.size());
		snapshot.routeIds.reserve(reachableRoutes.size());

		for (const auto& routeSeed : reachableRoutes)
		{
			throwIfCancelled(ctx, "snapshot packing");
			if (!routeSeed.route)
				continue;

			auto packedRoute = buildPackedRoute(routeSeed.route,
			                                   state,
			                                   stopIndexById,
			                                   routeTypeIndexByKey,
			                                   routeTypeKeys,
			                                   routeTypeBoardingSeconds,
			                                   ctx->cfg);
			if (packedRoute.stopIndices.size() < 2)
				continue;

			packedRoutes.push_back(std::move(packedRoute));
		}

		snapshot.stopCount = static_cast<int32_t>(snapshot.stopIds.size());
		snapshot.routeCount = static_cast<int32_t>(packedRoutes.size());
		snapshot.routeTypeCount = static_cast<int32_t>(routeTypeKeys.size());

		snapshot.transitData.routeStopOffsets.reserve(packedRoutes.size() + 1);
		snapshot.transitData.routeStopOffsets.push_back(0);
		if (ctx->cfg->useSchedule)
			snapshot.transitData.scheduleTripOffsets.push_back(0);
		std::vector<std::vector<std::pair<int32_t, int32_t>>> stopRouteRefs(static_cast<size_t>(snapshot.stopCount));
		for (int32_t routeIndex = 0; routeIndex < snapshot.routeCount; ++routeIndex)
		{
			const auto& route = packedRoutes[routeIndex];
			state.routesByIndex.push_back(route.route);
			state.routeOriginalStopPositionsByIndex.push_back(route.originalStopPositions);
			snapshot.routeIds.push_back(route.route->id);
			snapshot.transitData.routeTypeIndices.push_back(route.routeTypeIndex);
			snapshot.transitData.routeDefaultHeadwaySeconds.push_back(route.defaultHeadwaySeconds);

			for (size_t stopPosition = 0; stopPosition < route.stopIndices.size(); ++stopPosition)
			{
				const auto stopIndex = route.stopIndices[stopPosition];
				snapshot.transitData.routeStopIndices.push_back(stopIndex);
				snapshot.transitData.boardingCostSeconds.push_back(route.boardingCostSeconds);
				snapshot.transitData.cumulativeCompatTravelSeconds.push_back(route.cumulativeCompatTravelSeconds[stopPosition]);
				snapshot.transitData.cumulativeTravelDistancesMeters.push_back(route.cumulativeTravelDistancesMeters[stopPosition]);
				if (ctx->cfg->useSchedule)
				{
					snapshot.transitData.cumulativeScheduleOffsetsDeciSeconds.push_back(
						route.cumulativeScheduleOffsetsDeciSeconds[stopPosition]);
				}
				stopRouteRefs[stopIndex].push_back(std::make_pair(routeIndex, static_cast<int32_t>(stopPosition)));
			}

			if (ctx->cfg->useSchedule)
			{
				snapshot.transitData.scheduleTripStartsDeciSeconds.insert(
					snapshot.transitData.scheduleTripStartsDeciSeconds.end(),
					route.scheduleTripStartsDeciSeconds.begin(),
					route.scheduleTripStartsDeciSeconds.end());
				snapshot.transitData.scheduleTripOffsets.push_back(
					static_cast<int32_t>(snapshot.transitData.scheduleTripStartsDeciSeconds.size()));
			}
			snapshot.transitData.routeStopOffsets.push_back(
				static_cast<int32_t>(snapshot.transitData.routeStopIndices.size()));
		}

		snapshot.transitData.stopRouteOffsets.reserve(static_cast<size_t>(snapshot.stopCount) + 1);
		snapshot.transitData.stopRouteOffsets.push_back(0);
		for (auto& refsForStop : stopRouteRefs)
		{
			std::sort(refsForStop.begin(), refsForStop.end());
			for (const auto& ref : refsForStop)
			{
				snapshot.transitData.stopRouteRouteIndices.push_back(ref.first);
				snapshot.transitData.stopRouteStopPositions.push_back(ref.second);
			}
			snapshot.transitData.stopRouteOffsets.push_back(
				static_cast<int32_t>(snapshot.transitData.stopRouteRouteIndices.size()));
		}

		fillReachability(ctx->startX,
		                 ctx->startY,
		                 ctx->cfg->walkRadius,
		                 ctx->cfg->walkSpeed,
		                 state,
		                 snapshot.startWalkSeconds,
		                 snapshot.startWalkDistancesMeters);
		fillReachability(ctx->targetX,
		                 ctx->targetY,
		                 ctx->cfg->walkRadius,
		                 ctx->cfg->walkSpeed,
		                 state,
		                 snapshot.endWalkSeconds,
		                 snapshot.endWalkDistancesMeters);
		buildFootpaths(ctx, state);

		queryOptions.maxRounds = std::max(1, ctx->cfg->maxNumberOfChanges + 1);
		queryOptions.maxJourneys = calculateJourneyCandidateLimit(ctx);
		queryOptions.useTimetable = ctx->cfg->useSchedule;
		queryOptions.schedulelessMode = OsmAnd::RaptorEngine::SchedulelessMode::Compat;
		queryOptions.maxRouteTimeSeconds = calculateRouteLimitSeconds(ctx);
		queryOptions.walkRouteTypeIndex = kWalkRouteTypeIndex;
		queryOptions.boardingSecondsByRouteType = routeTypeBoardingSeconds;
		buildChangeMatrix(ctx->cfg, routeTypeKeys, queryOptions);

		std::string validationError;
		if (!snapshot.validate(&validationError))
			throw std::runtime_error(validationError);
		if (!queryOptions.validate(snapshot.routeTypeCount, &validationError))
			throw std::runtime_error(validationError);

		return state;
	}

	static SHARED_PTR<TransportRouteResult> convertJourney(const OsmAnd::RaptorEngine::RaptorJourney& journey,
	                                                       const PackedBuildState& state,
	                                                       SHARED_PTR<TransportRoutingConfiguration> cfg)
	{
		SHARED_PTR<TransportRouteResult> routeResult = std::make_shared<TransportRouteResult>(cfg);
		routeResult->routeTime = journey.totalTimeSeconds;
		routeResult->finishWalkDist = journey.finishWalkDistanceMeters;

		for (const auto& routeSegment : journey.segments)
		{
			if (routeSegment.routeIndex < 0 ||
			    routeSegment.routeIndex >= static_cast<int32_t>(state.routesByIndex.size()))
			{
				continue;
			}

			SHARED_PTR<TransportRouteResultSegment> resultSegment =
				std::make_shared<TransportRouteResultSegment>();
			resultSegment->route = state.routesByIndex[routeSegment.routeIndex];
			const auto& originalStopPositions = state.routeOriginalStopPositionsByIndex[routeSegment.routeIndex];
			if (routeSegment.boardStopPosition < 0 ||
			    routeSegment.alightStopPosition < routeSegment.boardStopPosition ||
			    routeSegment.alightStopPosition >= static_cast<int32_t>(originalStopPositions.size()))
			{
				continue;
			}
			resultSegment->start = originalStopPositions[routeSegment.boardStopPosition];
			resultSegment->end = originalStopPositions[routeSegment.alightStopPosition];
			resultSegment->walkDist = routeSegment.walkDistanceMeters;
			resultSegment->walkTime = cfg->walkSpeed > 0.0f
				? resultSegment->walkDist / cfg->walkSpeed
				: 0.0;
			resultSegment->travelDistApproximate = routeSegment.travelDistanceMeters;
			resultSegment->travelTime = routeSegment.travelTimeSeconds;
			resultSegment->depTime =
				cfg->useSchedule && routeSegment.departureTimeDeciSeconds >= 0
					? routeSegment.departureTimeDeciSeconds + cfg->scheduleTimeOfDay
					: routeSegment.departureTimeDeciSeconds;
			routeResult->segments.push_back(resultSegment);
		}

		return routeResult;
	}

	static bool sameRouteWithExtraSegments(const SHARED_PTR<TransportRouteResult>& fastRoute,
	                                       const SHARED_PTR<TransportRouteResult>& testRoute)
	{
		if (testRoute->segments.size() < fastRoute->segments.size())
			return false;

		int32_t testIndex = 0;
		for (int32_t fastIndex = 0; fastIndex < static_cast<int32_t>(fastRoute->segments.size()); ++fastIndex, ++testIndex)
		{
			const auto& fastSegment = fastRoute->segments[fastIndex];
			while (testIndex < static_cast<int32_t>(testRoute->segments.size()))
			{
				const auto& testSegment = testRoute->segments[testIndex];
				if (fastSegment->route->id != testSegment->route->id)
					testIndex += 1;
				else
					break;
			}
			if (testIndex >= static_cast<int32_t>(testRoute->segments.size()))
				return false;
		}
		return true;
	}

	static bool excludeRoute(const unique_ptr<TransportRoutingContext>& ctx,
	                         const SHARED_PTR<TransportRouteResult>& fastRoute,
	                         const SHARED_PTR<TransportRouteResult>& testRoute)
	{
		if (sameRouteWithExtraSegments(fastRoute, testRoute))
			return true;

		const auto fastRouteWalkDist = std::max(
			fastRoute->getWalkDist(),
			ctx->cfg->combineAltRoutesDiffStops * ctx->cfg->increaseForAltRoutesWalking);
		if (fastRouteWalkDist * ctx->cfg->increaseForAltRoutesWalking < testRoute->getWalkDist())
			return true;

		for (const auto& alternativeRoute : fastRoute->alternativeRoutes)
		{
			if (sameRouteWithExtraSegments(alternativeRoute, testRoute))
				return true;
		}
		return false;
	}

	static bool checkAlternative(const unique_ptr<TransportRoutingContext>& ctx,
	                             const SHARED_PTR<TransportRouteResult>& fastRoute,
	                             const SHARED_PTR<TransportRouteResult>& testRoute)
	{
		if (testRoute->segments.size() != fastRoute->segments.size())
			return false;

		const auto perSegmentDiffLimit = std::max<double>(ctx->cfg->combineAltRoutesDiffStops, 220.0);
		const auto sumDiffLimit = std::max<double>(
			ctx->cfg->combineAltRoutesSumDiffStops,
			perSegmentDiffLimit * std::max<size_t>(1, fastRoute->segments.size()) * 1.75);

		double sumDiffs = 0.0;
		for (int32_t segmentIndex = 0; segmentIndex < static_cast<int32_t>(fastRoute->segments.size()); ++segmentIndex)
		{
			const auto& fastSegment = fastRoute->segments[segmentIndex];
			const auto& testSegment = testRoute->segments[segmentIndex];
			const auto startDiff = measuredDist31(fastSegment->getStart().x31,
			                                      fastSegment->getStart().y31,
			                                      testSegment->getStart().x31,
			                                      testSegment->getStart().y31);
			const auto endDiff = measuredDist31(fastSegment->getEnd().x31,
			                                    fastSegment->getEnd().y31,
			                                    testSegment->getEnd().x31,
			                                    testSegment->getEnd().y31);
			sumDiffs += startDiff + endDiff;
			if (startDiff > perSegmentDiffLimit ||
			    endDiff > perSegmentDiffLimit)
			{
				return false;
			}
		}

		if (sumDiffs >= sumDiffLimit)
			return false;

		fastRoute->alternativeRoutes.push_back(testRoute);
		return true;
	}

	static SHARED_PTR<TransportRouteResultSegment> cloneSegmentWithoutAlternatives(
		const SHARED_PTR<TransportRouteResultSegment>& segment)
	{
		if (segment == nullptr)
			return nullptr;

		auto segmentCopy = std::make_shared<TransportRouteResultSegment>();
		segmentCopy->route = segment->route;
		segmentCopy->walkTime = segment->walkTime;
		segmentCopy->travelDistApproximate = segment->travelDistApproximate;
		segmentCopy->travelTime = segment->travelTime;
		segmentCopy->start = segment->start;
		segmentCopy->end = segment->end;
		segmentCopy->walkDist = segment->walkDist;
		segmentCopy->depTime = segment->depTime;
		return segmentCopy;
	}

	static SHARED_PTR<TransportRouteResultSegment> createSegmentAlternative(
		const SHARED_PTR<TransportRoute>& route,
		const int32_t startIndex,
		const int32_t endIndex,
		const SHARED_PTR<TransportRouteResultSegment>& mainSegment)
	{
		if (route == nullptr || mainSegment == nullptr)
			return nullptr;
		if (startIndex < 0 || endIndex <= startIndex ||
		    endIndex >= static_cast<int32_t>(route->forwardStops.size()))
		{
			return nullptr;
		}

		auto alternativeSegment = std::make_shared<TransportRouteResultSegment>();
		alternativeSegment->route = route;
		alternativeSegment->start = startIndex;
		alternativeSegment->end = endIndex;
		alternativeSegment->walkDist = mainSegment->walkDist;
		alternativeSegment->walkTime = mainSegment->walkTime;
		alternativeSegment->depTime = mainSegment->depTime;

		double distanceMeters = 0.0;
		for (int32_t stopIndex = startIndex + 1; stopIndex <= endIndex; ++stopIndex)
		{
			const auto& previousStop = route->forwardStops[stopIndex - 1];
			const auto& currentStop = route->forwardStops[stopIndex];
			distanceMeters += getDistance(previousStop->lat,
			                              previousStop->lon,
			                              currentStop->lat,
			                              currentStop->lon);
		}
		alternativeSegment->travelDistApproximate = distanceMeters;
		if (mainSegment->travelDistApproximate > 0.0 &&
		    mainSegment->travelTime > 0.0 &&
		    distanceMeters > 0.0)
		{
			alternativeSegment->travelTime =
				mainSegment->travelTime * distanceMeters / mainSegment->travelDistApproximate;
		}
		else
		{
			alternativeSegment->travelTime = mainSegment->travelTime;
		}
		return alternativeSegment;
	}

	static bool haveSameRouteType(const SHARED_PTR<TransportRouteResultSegment>& firstSegment,
	                              const SHARED_PTR<TransportRouteResultSegment>& secondSegment)
	{
		if (firstSegment == nullptr || secondSegment == nullptr ||
		    firstSegment->route == nullptr || secondSegment->route == nullptr)
		{
			return false;
		}

		return normalizeRouteTypeKey(firstSegment->route->getType()) ==
		       normalizeRouteTypeKey(secondSegment->route->getType());
	}

	static double calculateSegmentDirectionSimilarity(const SHARED_PTR<TransportRouteResultSegment>& firstSegment,
	                                                  const SHARED_PTR<TransportRouteResultSegment>& secondSegment)
	{
		if (firstSegment == nullptr || secondSegment == nullptr)
			return -1.0;

		const auto& firstStart = firstSegment->getStart();
		const auto& firstEnd = firstSegment->getEnd();
		const auto& secondStart = secondSegment->getStart();
		const auto& secondEnd = secondSegment->getEnd();

		const auto firstDx = static_cast<double>(firstEnd.x31) - firstStart.x31;
		const auto firstDy = static_cast<double>(firstEnd.y31) - firstStart.y31;
		const auto secondDx = static_cast<double>(secondEnd.x31) - secondStart.x31;
		const auto secondDy = static_cast<double>(secondEnd.y31) - secondStart.y31;
		const auto firstLengthSquared = firstDx * firstDx + firstDy * firstDy;
		const auto secondLengthSquared = secondDx * secondDx + secondDy * secondDy;
		if (!(firstLengthSquared > 0.0) || !(secondLengthSquared > 0.0))
			return 1.0;

		return (firstDx * secondDx + firstDy * secondDy) /
		       std::sqrt(firstLengthSquared * secondLengthSquared);
	}

	static double calculateDirectionSimilarityForStops(const TransportStop& firstStart,
	                                                   const TransportStop& firstEnd,
	                                                   const TransportStop& secondStart,
	                                                   const TransportStop& secondEnd)
	{
		const auto firstDx = static_cast<double>(firstEnd.x31) - firstStart.x31;
		const auto firstDy = static_cast<double>(firstEnd.y31) - firstStart.y31;
		const auto secondDx = static_cast<double>(secondEnd.x31) - secondStart.x31;
		const auto secondDy = static_cast<double>(secondEnd.y31) - secondStart.y31;
		const auto firstLengthSquared = firstDx * firstDx + firstDy * firstDy;
		const auto secondLengthSquared = secondDx * secondDx + secondDy * secondDy;
		if (!(firstLengthSquared > 0.0) || !(secondLengthSquared > 0.0))
			return 1.0;

		return (firstDx * secondDx + firstDy * secondDy) /
		       std::sqrt(firstLengthSquared * secondLengthSquared);
	}

	static bool areEquivalentLegEndpoints(const TransportStop& mainStop,
	                                      const SHARED_PTR<TransportStop>& candidateStop,
	                                      const double matchLimitMeters)
	{
		if (candidateStop == nullptr)
			return false;
		if (mainStop.id != 0 && candidateStop->id != 0 && mainStop.id == candidateStop->id)
			return true;

		const auto endpointTolerance = std::max(45.0, matchLimitMeters * 0.30);
		return measuredDist31(mainStop.x31, mainStop.y31, candidateStop->x31, candidateStop->y31) <= endpointTolerance;
	}

	static int32_t countOrderedInteriorStopMatches(const std::vector<SHARED_PTR<TransportStop>>& mainStops,
	                                               const SHARED_PTR<TransportRoute>& candidateRoute,
	                                               const int32_t candidateStartIndex,
	                                               const int32_t candidateEndIndex,
	                                               const double matchLimitMeters)
	{
		if (candidateRoute == nullptr)
			return 0;
		if (mainStops.size() <= 2 || candidateEndIndex - candidateStartIndex <= 1)
			return 0;

		const auto limit = std::max(120.0, matchLimitMeters);
		int32_t nextCandidateIndex = candidateStartIndex + 1;
		int32_t matches = 0;
		for (int32_t mainIndex = 1; mainIndex + 1 < static_cast<int32_t>(mainStops.size()) &&
		                              nextCandidateIndex < candidateEndIndex;
		     ++mainIndex)
		{
			const auto& mainStop = mainStops[mainIndex];
			if (mainStop == nullptr)
				continue;

			double bestDistance = limit;
			int32_t bestCandidateIndex = -1;
			for (int32_t candidateIndex = nextCandidateIndex; candidateIndex < candidateEndIndex; ++candidateIndex)
			{
				const auto& candidateStop = candidateRoute->forwardStops[candidateIndex];
				if (candidateStop == nullptr)
					continue;

				const auto distance = measuredDist31(mainStop->x31,
				                                     mainStop->y31,
				                                     candidateStop->x31,
				                                     candidateStop->y31);
				if (distance > bestDistance)
					continue;

				bestDistance = distance;
				bestCandidateIndex = candidateIndex;
				if (distance <= limit * 0.35)
					break;
			}

			if (bestCandidateIndex < 0)
				continue;

			++matches;
			nextCandidateIndex = bestCandidateIndex + 1;
		}

		return matches;
	}

	static bool matchesSameLeg(const SHARED_PTR<TransportRouteResultSegment>& mainSegment,
	                           const SHARED_PTR<TransportRouteResultSegment>& candidateSegment,
	                           const SHARED_PTR<TransportRoutingConfiguration>& cfg)
	{
		if (mainSegment == nullptr || candidateSegment == nullptr ||
		    mainSegment->route == nullptr || candidateSegment->route == nullptr)
		{
			return false;
		}

		if (!haveSameRouteType(mainSegment, candidateSegment))
			return false;

		if (mainSegment->route->id == candidateSegment->route->id &&
		    mainSegment->start == candidateSegment->start &&
		    mainSegment->end == candidateSegment->end)
		{
			return false;
		}

		const auto perStopMatchLimit = std::max<double>(
			cfg != nullptr ? cfg->combineAltRoutesDiffStops : 0.0,
			220.0);
		const auto totalStopMatchLimit = std::max<double>(
			cfg != nullptr ? cfg->combineAltRoutesSumDiffStops : 0.0,
			perStopMatchLimit * 2.25);
		const auto startDiff = measuredDist31(mainSegment->getStart().x31,
		                                      mainSegment->getStart().y31,
		                                      candidateSegment->getStart().x31,
		                                      candidateSegment->getStart().y31);
		const auto endDiff = measuredDist31(mainSegment->getEnd().x31,
		                                    mainSegment->getEnd().y31,
		                                    candidateSegment->getEnd().x31,
		                                    candidateSegment->getEnd().y31);
		if (startDiff > perStopMatchLimit || endDiff > perStopMatchLimit)
			return false;
		if (startDiff + endDiff > totalStopMatchLimit)
			return false;

		const auto minimumDirectionSimilarity =
			startDiff + endDiff <= perStopMatchLimit * 0.6 ? 0.35 : 0.55;
		return calculateSegmentDirectionSimilarity(mainSegment, candidateSegment) >= minimumDirectionSimilarity;
	}

	struct AnalogousSegmentCandidate
	{
		SHARED_PTR<TransportRouteResultSegment> segment;
		double score = kInfinity;
		double startDiff = kInfinity;
		double endDiff = kInfinity;
		int32_t exactEndpointMatches = 0;
		int32_t interiorMatches = 0;
	};

	static std::vector<SHARED_PTR<TransportRouteResultSegment>> collectAnalogousSegmentsOnRoute(
		const SHARED_PTR<TransportRouteResultSegment>& mainSegment,
		const SHARED_PTR<TransportRoute>& candidateRoute,
		const SHARED_PTR<TransportRoutingConfiguration>& cfg)
	{
		if (mainSegment == nullptr || candidateRoute == nullptr || candidateRoute->forwardStops.size() < 2)
			return {};
		if (mainSegment->route == nullptr)
			return {};
		if (normalizeRouteTypeKey(mainSegment->route->getType()) != normalizeRouteTypeKey(candidateRoute->getType()))
			return {};

		const auto startMatchLimit = std::max<double>(
			cfg != nullptr ? cfg->combineAltRoutesDiffStops : 0.0,
			320.0);
		const auto endMatchLimit = startMatchLimit;
		const auto totalMatchLimit = std::max<double>(
			cfg != nullptr ? cfg->combineAltRoutesSumDiffStops : 0.0,
			startMatchLimit * 3.0);

		const auto& mainStart = mainSegment->getStart();
		const auto& mainEnd = mainSegment->getEnd();
		const auto mainTravelStops = mainSegment->getTravelStops();
		std::vector<std::pair<int32_t, double>> startCandidates;
		std::vector<std::pair<int32_t, double>> endCandidates;
		startCandidates.reserve(8);
		endCandidates.reserve(8);

		for (int32_t stopIndex = 0; stopIndex < static_cast<int32_t>(candidateRoute->forwardStops.size()); ++stopIndex)
		{
			const auto& stop = candidateRoute->forwardStops[stopIndex];
			const auto startDiff = measuredDist31(mainStart.x31, mainStart.y31, stop->x31, stop->y31);
			if (startDiff <= startMatchLimit)
				startCandidates.emplace_back(stopIndex, startDiff);

			const auto endDiff = measuredDist31(mainEnd.x31, mainEnd.y31, stop->x31, stop->y31);
			if (endDiff <= endMatchLimit)
				endCandidates.emplace_back(stopIndex, endDiff);
		}

		std::vector<AnalogousSegmentCandidate> rankedCandidates;
		std::set<std::pair<int32_t, int32_t>> seenEndpointPairs;
		for (const auto& startCandidate : startCandidates)
		{
			for (const auto& endCandidate : endCandidates)
			{
				if (endCandidate.first <= startCandidate.first)
					continue;

				const auto totalDiff = startCandidate.second + endCandidate.second;
				if (totalDiff > totalMatchLimit)
					continue;

				const auto& candidateStart = *candidateRoute->forwardStops[startCandidate.first];
				const auto& candidateEnd = *candidateRoute->forwardStops[endCandidate.first];
				const auto directionSimilarity =
					calculateDirectionSimilarityForStops(mainStart, mainEnd, candidateStart, candidateEnd);
				const auto minimumDirectionSimilarity =
					totalDiff <= startMatchLimit * 0.75 ? 0.20 : 0.45;
				if (directionSimilarity < minimumDirectionSimilarity)
					continue;
				if (!seenEndpointPairs.emplace(startCandidate.first, endCandidate.first).second)
					continue;

				const auto candidateDistance =
					measuredDist31(candidateStart.x31, candidateStart.y31, candidateEnd.x31, candidateEnd.y31);
				const auto mainDistance =
					measuredDist31(mainStart.x31, mainStart.y31, mainEnd.x31, mainEnd.y31);
				double distancePenalty = 0.0;
				if (mainDistance > 0.0 && candidateDistance > 0.0)
				{
					const auto ratio = candidateDistance / mainDistance;
					if (ratio < 0.50 || ratio > 2.50)
						continue;
					distancePenalty = std::abs(std::log(ratio)) * 60.0;
				}

				const auto mainInteriorCount = std::max<int32_t>(0, static_cast<int32_t>(mainTravelStops.size()) - 2);
				const auto candidateInteriorCount = std::max<int32_t>(0, endCandidate.first - startCandidate.first - 1);
				const auto minInteriorCount = std::min(mainInteriorCount, candidateInteriorCount);
				int32_t matchedInteriorStops = 0;
				if (minInteriorCount > 0)
				{
					matchedInteriorStops =
						countOrderedInteriorStopMatches(mainTravelStops,
						                               candidateRoute,
						                               startCandidate.first,
						                               endCandidate.first,
						                               startMatchLimit * 0.75);
				}

				int32_t exactEndpointMatches = 0;
				if (areEquivalentLegEndpoints(mainStart, candidateRoute->forwardStops[startCandidate.first], startMatchLimit))
					++exactEndpointMatches;
				if (areEquivalentLegEndpoints(mainEnd, candidateRoute->forwardStops[endCandidate.first], endMatchLimit))
					++exactEndpointMatches;

				const auto endpointBonus = static_cast<double>(exactEndpointMatches) * startMatchLimit * 0.70;
				const auto coverageBonus = static_cast<double>(matchedInteriorStops) * 12.0;
				const auto score = totalDiff + distancePenalty - endpointBonus - coverageBonus;

				auto alternativeSegment =
					createSegmentAlternative(candidateRoute, startCandidate.first, endCandidate.first, mainSegment);
				if (alternativeSegment == nullptr)
					continue;

				rankedCandidates.push_back({alternativeSegment,
				                           score,
				                           startCandidate.second,
				                           endCandidate.second,
				                           exactEndpointMatches,
				                           matchedInteriorStops});
			}
		}

		std::sort(rankedCandidates.begin(),
		          rankedCandidates.end(),
		          [](const AnalogousSegmentCandidate& left, const AnalogousSegmentCandidate& right)
		          {
			          if (left.exactEndpointMatches != right.exactEndpointMatches)
				          return left.exactEndpointMatches > right.exactEndpointMatches;
			          if (left.score != right.score)
				          return left.score < right.score;
			          if (left.interiorMatches != right.interiorMatches)
				          return left.interiorMatches > right.interiorMatches;
			          if (left.startDiff + left.endDiff != right.startDiff + right.endDiff)
				          return left.startDiff + left.endDiff < right.startDiff + right.endDiff;
			          return left.segment->travelDistApproximate < right.segment->travelDistApproximate;
		          });

		std::vector<SHARED_PTR<TransportRouteResultSegment>> alternatives;
		const auto maxAlternativesPerRoute = static_cast<size_t>(4);
		alternatives.reserve(std::min(maxAlternativesPerRoute, rankedCandidates.size()));
		for (size_t i = 0; i < rankedCandidates.size() && i < maxAlternativesPerRoute; ++i)
			alternatives.push_back(rankedCandidates[i].segment);
		return alternatives;
	}

	static void attachSegmentAlternatives(vector<SHARED_PTR<TransportRouteResult>>& routes,
	                                      const SHARED_PTR<TransportRoutingConfiguration>& cfg,
	                                      const std::vector<SHARED_PTR<TransportRoute>>& routeUniverse)
	{
		const auto alternativeKey = [](const SHARED_PTR<TransportRouteResultSegment>& segment) -> std::string
		{
			if (segment == nullptr || segment->route == nullptr)
				return std::string();
			const auto adjustedRef = segment->route->getAdjustedRouteRef(true);
			if (!adjustedRef.empty())
				return adjustedRef;
			if (!segment->route->ref.empty())
				return segment->route->ref;
			return std::to_string(segment->route->id);
		};

		for (const auto& route : routes)
		{
			for (int32_t segmentIndex = 0; segmentIndex < static_cast<int32_t>(route->segments.size()); ++segmentIndex)
			{
				const auto maxAlternativesPerSegment = static_cast<size_t>(24);
				std::set<std::string> altRefsAlreadyAdded;
				std::vector<SHARED_PTR<TransportRouteResultSegment>> alternatives;
				const auto& mainSegment = route->segments[segmentIndex];
				const auto mainRef = alternativeKey(mainSegment);
				const auto tryAddAlternative =
					[&altRefsAlreadyAdded, &alternatives, &alternativeKey, &mainRef, maxAlternativesPerSegment](
						const SHARED_PTR<TransportRouteResultSegment>& sourceSegment)
					{
						if (alternatives.size() >= maxAlternativesPerSegment)
							return;
						const auto altRef = alternativeKey(sourceSegment);
						if (altRef.empty() || altRef == mainRef || altRefsAlreadyAdded.count(altRef) > 0)
							return;

						auto alternativeCopy = cloneSegmentWithoutAlternatives(sourceSegment);
						if (alternativeCopy == nullptr)
							return;

						altRefsAlreadyAdded.insert(altRef);
						alternatives.push_back(alternativeCopy);
					};

				for (const auto& alternativeRoute : route->alternativeRoutes)
				{
					if (segmentIndex >= static_cast<int32_t>(alternativeRoute->segments.size()))
						continue;

					tryAddAlternative(alternativeRoute->segments[segmentIndex]);
				}

				for (const auto& selectedRoute : routes)
				{
					if (selectedRoute == nullptr || selectedRoute.get() == route.get())
						continue;

					for (const auto& candidateSegment : selectedRoute->segments)
					{
						if (!matchesSameLeg(mainSegment, candidateSegment, cfg))
							continue;

						tryAddAlternative(candidateSegment);
					}
				}

				for (const auto& candidateRoute : routeUniverse)
				{
					if (alternatives.size() >= maxAlternativesPerSegment)
						break;
					if (candidateRoute == nullptr)
						continue;

					const auto analogousSegments = collectAnalogousSegmentsOnRoute(mainSegment, candidateRoute, cfg);
					for (const auto& analogousSegment : analogousSegments)
					{
						if (alternatives.size() >= maxAlternativesPerSegment)
							break;
						tryAddAlternative(analogousSegment);
					}
				}

				auto& segmentAlternatives = route->segments[segmentIndex]->alternatives;
				segmentAlternatives.clear();
				segmentAlternatives.insert(segmentAlternatives.end(), alternatives.begin(), alternatives.end());
			}
		}
	}

	static void collectResultRoutes(unique_ptr<TransportRoutingContext>& ctx,
	                                vector<SHARED_PTR<TransportRouteResult>>& candidateRoutes,
	                                vector<SHARED_PTR<TransportRouteResult>>& routes,
	                                const std::vector<SHARED_PTR<TransportRoute>>& routeUniverse)
	{
		if (candidateRoutes.empty())
			return;

		const auto transferCount = [](const SHARED_PTR<TransportRouteResult>& route) -> int32_t
		{
			return route == nullptr ? std::numeric_limits<int32_t>::max() : std::max(0, static_cast<int32_t>(route->segments.size()) - 1);
		};
		const auto segmentCount = [](const SHARED_PTR<TransportRouteResult>& route) -> int32_t
		{
			return route == nullptr ? std::numeric_limits<int32_t>::max() : static_cast<int32_t>(route->segments.size());
		};
		const auto walkDistance = [](const SHARED_PTR<TransportRouteResult>& route) -> double
		{
			return route == nullptr ? kInfinity : route->getWalkDist();
		};
		std::sort(candidateRoutes.begin(),
		          candidateRoutes.end(),
		          [&transferCount, &segmentCount, &walkDistance](const SHARED_PTR<TransportRouteResult>& left,
		                                                         const SHARED_PTR<TransportRouteResult>& right)
		          {
			          if (left->routeTime != right->routeTime)
				          return left->routeTime < right->routeTime;
			          if (transferCount(left) != transferCount(right))
				          return transferCount(left) < transferCount(right);
			          if (segmentCount(left) != segmentCount(right))
				          return segmentCount(left) < segmentCount(right);
			          if (walkDistance(left) != walkDistance(right))
				          return walkDistance(left) < walkDistance(right);
			          return left->getTravelDist() < right->getTravelDist();
		          });

		const auto bestTimeSeconds = candidateRoutes.front()->routeTime;
		const auto nearBestTimeWindowSeconds = calculateComfortTimeWindowSeconds(bestTimeSeconds);
		const auto rankingTimeBandSeconds = calculateRankingTimeBandSeconds(bestTimeSeconds);
		std::stable_sort(candidateRoutes.begin(),
		                 candidateRoutes.end(),
		                 [bestTimeSeconds,
		                  nearBestTimeWindowSeconds,
		                  rankingTimeBandSeconds,
		                  &transferCount,
		                  &segmentCount,
		                  &walkDistance](const SHARED_PTR<TransportRouteResult>& left,
		                                 const SHARED_PTR<TransportRouteResult>& right)
		                 {
			                 const auto leftNearBest =
				                 left->routeTime <= bestTimeSeconds + nearBestTimeWindowSeconds;
			                 const auto rightNearBest =
				                 right->routeTime <= bestTimeSeconds + nearBestTimeWindowSeconds;
			                 if (leftNearBest != rightNearBest)
				                 return leftNearBest;

			                 const auto leftTimeBand =
				                 calculateTimeBandIndex(left->routeTime, bestTimeSeconds, rankingTimeBandSeconds);
			                 const auto rightTimeBand =
				                 calculateTimeBandIndex(right->routeTime, bestTimeSeconds, rankingTimeBandSeconds);
			                 if (leftTimeBand != rightTimeBand)
				                 return leftTimeBand < rightTimeBand;

			                 if (transferCount(left) != transferCount(right))
				                 return transferCount(left) < transferCount(right);
			                 if (left->routeTime != right->routeTime)
				                 return left->routeTime < right->routeTime;
			                 if (!isAlmostEqual(walkDistance(left), walkDistance(right)))
				                 return walkDistance(left) < walkDistance(right);
			                 if (segmentCount(left) != segmentCount(right))
				                 return segmentCount(left) < segmentCount(right);
			                 return left->getTravelDist() < right->getTravelDist();
		                 });

		const auto routeSignature = [](const SHARED_PTR<TransportRouteResult>& route) -> std::string
		{
			std::ostringstream out;
			if (route == nullptr)
				return out.str();
			for (const auto& segment : route->segments)
			{
				if (segment == nullptr || segment->route == nullptr)
					continue;
				out << segment->route->id << ':' << segment->start << ':' << segment->end << ';';
			}
			return out.str();
		};
		const auto routeChainSignature = [](const SHARED_PTR<TransportRouteResult>& route) -> std::string
		{
			std::ostringstream out;
			if (route == nullptr)
				return out.str();
			for (const auto& segment : route->segments)
			{
				if (segment == nullptr || segment->route == nullptr)
					continue;
				out << segment->route->id << ';';
			}
			return out.str();
		};

		std::vector<int32_t> preferredIndices;
		preferredIndices.reserve(candidateRoutes.size());
		std::unordered_set<std::string> selectedExactSignatures;
		selectedExactSignatures.reserve(candidateRoutes.size());
		std::unordered_set<std::string> selectedRouteChains;
		selectedRouteChains.reserve(candidateRoutes.size());
		const auto trySelectCandidate =
			[&candidateRoutes, &preferredIndices, &selectedExactSignatures, &selectedRouteChains, &routeSignature, &routeChainSignature](
				const int32_t index,
				const bool allowUsedRouteChain) -> bool
		{
			if (index < 0 || index >= static_cast<int32_t>(candidateRoutes.size()))
				return false;

			const auto exactSignature = routeSignature(candidateRoutes[index]);
			if (!selectedExactSignatures.emplace(exactSignature).second)
				return false;

			const auto chainSignature = routeChainSignature(candidateRoutes[index]);
			if (!allowUsedRouteChain && selectedRouteChains.count(chainSignature) > 0)
			{
				selectedExactSignatures.erase(exactSignature);
				return false;
			}

			preferredIndices.push_back(index);
			selectedRouteChains.emplace(std::move(chainSignature));
			return true;
		};

		trySelectCandidate(0, true);

		int32_t leastTransfersIndex = -1;
		int32_t leastWalkIndex = -1;
		int32_t comfortIndex = -1;
		int32_t leastSegmentsIndex = -1;
		const auto preferenceTimeWindowSeconds =
			std::max(rankingTimeBandSeconds, std::min(nearBestTimeWindowSeconds, rankingTimeBandSeconds * 2.0));
		for (int32_t i = 0; i < static_cast<int32_t>(candidateRoutes.size()); ++i)
		{
			const auto& route = candidateRoutes[i];
			if (route->routeTime > bestTimeSeconds + preferenceTimeWindowSeconds)
				break;

			if (leastTransfersIndex < 0 ||
			    transferCount(route) < transferCount(candidateRoutes[leastTransfersIndex]) ||
			    (transferCount(route) == transferCount(candidateRoutes[leastTransfersIndex]) &&
			     route->routeTime < candidateRoutes[leastTransfersIndex]->routeTime))
			{
				leastTransfersIndex = i;
			}

			if (leastSegmentsIndex < 0 ||
			    segmentCount(route) < segmentCount(candidateRoutes[leastSegmentsIndex]) ||
			    (segmentCount(route) == segmentCount(candidateRoutes[leastSegmentsIndex]) &&
			     route->routeTime < candidateRoutes[leastSegmentsIndex]->routeTime))
			{
				leastSegmentsIndex = i;
			}

			if (leastWalkIndex < 0 ||
			    walkDistance(route) < walkDistance(candidateRoutes[leastWalkIndex]) ||
			    (walkDistance(route) == walkDistance(candidateRoutes[leastWalkIndex]) &&
			     route->routeTime < candidateRoutes[leastWalkIndex]->routeTime))
			{
				leastWalkIndex = i;
			}

			if (comfortIndex < 0 ||
			    calculateComfortScoreSeconds(route) < calculateComfortScoreSeconds(candidateRoutes[comfortIndex]))
			{
				comfortIndex = i;
			}
		}

		trySelectCandidate(leastTransfersIndex, false);
		trySelectCandidate(leastSegmentsIndex, false);
		trySelectCandidate(leastWalkIndex, false);
		trySelectCandidate(comfortIndex, false);

		for (int32_t pass = 0; pass < 2; ++pass)
		{
			const auto allowUsedRouteChain = pass > 0;
			for (int32_t i = 0; i < static_cast<int32_t>(candidateRoutes.size()); ++i)
				trySelectCandidate(i, allowUsedRouteChain);
		}

		if (!preferredIndices.empty())
		{
			std::sort(preferredIndices.begin(), preferredIndices.end());
			std::vector<char> used(candidateRoutes.size(), 0);
			std::vector<SHARED_PTR<TransportRouteResult>> orderedCandidates;
			orderedCandidates.reserve(candidateRoutes.size());
			for (const auto index : preferredIndices)
			{
				used[index] = 1;
				orderedCandidates.push_back(candidateRoutes[index]);
			}
			for (int32_t i = 0; i < static_cast<int32_t>(candidateRoutes.size()); ++i)
			{
				if (!used[i])
					orderedCandidates.push_back(candidateRoutes[i]);
			}
			candidateRoutes.swap(orderedCandidates);
		}

		for (const auto& route : candidateRoutes)
		{
			bool exclude = false;
			for (const auto& existingRoute : routes)
			{
				if (excludeRoute(ctx, existingRoute, route))
				{
					exclude = true;
					break;
				}
			}

			if (!exclude)
			{
				for (const auto& existingRoute : routes)
				{
					if (checkAlternative(ctx, existingRoute, route))
					{
						exclude = true;
						break;
					}
				}
			}

			if (exclude)
				continue;

			const auto limitByNumber = std::max(10, ctx->cfg->ptLimitResultsByNumber);
			if (limitByNumber > 0 && static_cast<int32_t>(routes.size()) >= limitByNumber)
				break;

			routes.push_back(route);
		}

		attachSegmentAlternatives(routes, ctx->cfg, routeUniverse);
	}
}

RaptorTransportPlanner::RaptorTransportPlanner() {
}

RaptorTransportPlanner::~RaptorTransportPlanner() {
}

bool RaptorTransportPlanner::buildTransportRoute(unique_ptr<TransportRoutingContext>& ctx,
                                                 vector<SHARED_PTR<TransportRouteResult>>& res) {
	if (!ctx || !ctx->cfg)
		return false;

	try
	{
		ctx->calcLatLons();
		auto packedState = buildPackedState(ctx);
		OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info,
		                  "RaptorTransportPlanner snapshot built: %d routes, %d stops, %zu tiles",
		                  packedState.snapshot.routeCount,
		                  packedState.snapshot.stopCount,
		                  ctx->quadTree.size());

		const auto queryResult = OsmAnd::RaptorEngine::Engine::run(
			packedState.snapshot, packedState.queryOptions, false);
		if (!queryResult.found)
		{
			OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "RaptorTransportPlanner found no PT route");
			return true;
		}

		vector<SHARED_PTR<TransportRouteResult>> candidateRoutes;
		candidateRoutes.reserve(queryResult.journeys.size());
		for (const auto& journey : queryResult.journeys)
		{
			if (!journey.found)
				continue;
			candidateRoutes.push_back(convertJourney(journey, packedState, ctx->cfg));
		}
		if (candidateRoutes.empty())
		{
			OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "RaptorTransportPlanner produced no convertible PT journeys");
			return true;
		}

		collectResultRoutes(ctx, candidateRoutes, res, packedState.routesByIndex);
		return true;
	}
	catch (const std::exception& e)
	{
		const std::string errorMessage = e.what();
		if (errorMessage.find("cancelled") != std::string::npos)
		{
			OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info,
			                  "RaptorTransportPlanner cancelled: %s", errorMessage.c_str());
			return true;
		}

		OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Warning,
		                  "RaptorTransportPlanner failed: %s", errorMessage.c_str());
		return false;
	}
}

#endif // _OSMAND_RAPTOR_ENGINE_TRANSPORT_PLANNER_CPP
