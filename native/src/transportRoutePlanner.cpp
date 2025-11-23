#include <inttypes.h>
#ifndef _OSMAND_TRANSPORT_ROUTE_PLANNER_CPP
#define _OSMAND_TRANSPORT_ROUTE_PLANNER_CPP
#include "transportRoutePlanner.h"

#include "Logging.h"
#include "transportRouteResult.h"
#include "transportRouteResultSegment.h"
#include "transportRouteSegment.h"
#include "transportRoutingConfiguration.h"
#include "transportRoutingContext.h"
#include "transportRoutingObjects.h"

#define TRACE_ONBOARD_ID 0 // 19728216
#define TRACE_CHANGE_ID 0 // 19728216

struct TransportSegmentsComparator {
	TransportSegmentsComparator() = default;

	template <typename T>
	static int cmp(const T& a, const T& b)
	{
		if (a < b) return -1;
		if (a > b) return +1;
		return 0;
	}

	bool operator()(const SHARED_PTR<TransportRouteSegment>& o1, const SHARED_PTR<TransportRouteSegment>& o2) const
	{
		int cmpDist = cmp(o1->distFromStart, o2->distFromStart);
		return cmpDist == 0 ? cmp(o1->getId() + o1->nonce, o2->getId() + o2->nonce) > 0 : cmpDist > 0;
	}
};

TransportRoutePlanner::TransportRoutePlanner() {
}

TransportRoutePlanner::~TransportRoutePlanner() {
}

bool TransportRoutePlanner::includeRoute(SHARED_PTR<TransportRouteResult>& fastRoute,
										 SHARED_PTR<TransportRouteResult>& testRoute) {
	if (testRoute->segments.size() < fastRoute->segments.size()) {
		return false;
	}
	int32_t j = 0;
	for (int32_t i = 0; i < fastRoute->segments.size(); i++, j++) {
		SHARED_PTR<TransportRouteResultSegment>& fs = fastRoute->segments.at(i);
		while (j < testRoute->segments.size()) {
			SHARED_PTR<TransportRouteResultSegment>& ts = testRoute->segments[j];
			if (fs->route->id != ts->route->id) {
				j++;
			} else {
				break;
			}
		}
		if (j >= testRoute->segments.size()) {
			return false;
		}
	}
	return true;
}

void TransportRoutePlanner::prepareResults(unique_ptr<TransportRoutingContext>& ctx,
										   vector<SHARED_PTR<TransportRouteSegment>>& results,
										   vector<SHARED_PTR<TransportRouteResult>>& routes) {
	sort(results.begin(), results.end(),
		 [](const SHARED_PTR<TransportRouteSegment>& lhs, const SHARED_PTR<TransportRouteSegment>& rhs) {
			 return lhs->distFromStart < rhs->distFromStart;
		 });

	OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info,
					  "Found %zu results, visited %d routes / %d stops, loaded "
					  "%zu tiles, loaded ways %d (%d wrong)",
					  results.size(), ctx->visitedRoutesCount, ctx->visitedStops, ctx->quadTree.size(), ctx->loadedWays,
					  ctx->wrongLoadedWays);

	for (SHARED_PTR<TransportRouteSegment>& res : results) {
		if (ctx->calculationProgress.get() && ctx->calculationProgress->isCancelled()) {
			return;
		}
		SHARED_PTR<TransportRouteResult> route(new TransportRouteResult(ctx->cfg));
		route->routeTime = res->distFromStart;
		route->finishWalkDist = res->walkDist;
		SHARED_PTR<TransportRouteSegment> p = res;
		while (p != nullptr) {
			if (ctx->calculationProgress != nullptr && ctx->calculationProgress->isCancelled()) {
				return;
			}
			if (p->hasParentRoute) {
				unique_ptr<TransportRouteResultSegment> sg(new TransportRouteResultSegment());
				sg->route = p->parentRoute->road;
				sg->start = p->parentRoute->segStart;
				sg->end = p->parentStop;
				sg->walkDist = p->parentRoute->walkDist;
				sg->walkTime = sg->walkDist / ctx->cfg->walkSpeed;
				sg->depTime = p->departureTime;
				sg->travelDistApproximate = p->parentTravelDist;
				sg->travelTime = p->parentTravelTime;
				route->segments.insert(route->segments.begin(), std::move(sg));
			}
			p = p->parentRoute;
		}
		// test if faster routes fully included
		bool include = false; // TODO next 105
		for (SHARED_PTR<TransportRouteResult>& s : routes) {
			if (ctx->calculationProgress.get() && ctx->calculationProgress->isCancelled()) {
				return;
			}
			if (includeRoute(s, route)) {
				include = true;
				break;
			}
		}
		if (!include) {
			route->toString();
			routes.push_back(std::move(route));
			// System.out.println(route.toString());
		}
	}
}

void TransportRoutePlanner::buildTransportRoute(unique_ptr<TransportRoutingContext>& ctx,
												vector<SHARED_PTR<TransportRouteResult>>& res) {
	long nonce = 0;
	OsmAnd::ElapsedTimer pt_timer;
	pt_timer.Start();
	ctx->loadTime.Enable();
	ctx->searchTransportIndexTime.Enable();
	ctx->readTime.Enable();

	TransportSegmentsComparator trSegmComp;
	TRANSPORT_SEGMENTS_QUEUE queue(trSegmComp);
	vector<SHARED_PTR<TransportRouteSegment>> startStops;
	vector<SHARED_PTR<TransportRouteSegment>> endStops;
	UNORDERED(map)<int64_t, SHARED_PTR<TransportRouteSegment>> endSegments;
	vector<SHARED_PTR<TransportRouteSegment>> results;

	ctx->getTransportStops(ctx->startX, ctx->startY, false, startStops);
	ctx->getTransportStops(ctx->targetX, ctx->targetY, false, endStops);
	ctx->calcLatLons();

	for (SHARED_PTR<TransportRouteSegment>& s : endStops) {
		endSegments.insert({s->getId(), s});
	}

	if (startStops.size() == 0) {
		return;
	}

	for (SHARED_PTR<TransportRouteSegment>& r : startStops) {
		r->walkDist = getDistance(r->getLocationLat(), r->getLocationLon(), ctx->startLat, ctx->startLon);
		r->distFromStart = r->walkDist / ctx->cfg->walkSpeed;
		if (TRACE_ONBOARD_ID != 0)
		{
			ObfConstants::osmand_id_t id = ObfConstants::getOsmIdFromBinaryMapObjectId(r->road->id);
			if (id == TRACE_ONBOARD_ID)
			{
				LogPrintf(OsmAnd::LogSeverityLevel::Debug, "TRACE_ONBOARD_ID %f %" PRId64 " %s",
				          r->distFromStart, id, r->to_string().c_str());
			}
		}
		r->nonce = nonce++;
		queue.push(r);
	}

	double totalDistance = getDistance(ctx->startLat, ctx->startLon, ctx->endLat, ctx->endLon);
	double finishTime = ctx->cfg->maxRouteTime;
	if (totalDistance > ctx->cfg->maxRouteDistance && ctx->cfg->maxRouteIncreaseSpeed > 0) {
		int increaseTime = (int)((totalDistance - ctx->cfg->maxRouteDistance) * 3.6 / ctx->cfg->maxRouteIncreaseSpeed);
		finishTime += increaseTime;
	}

	double maxTravelTimeCmpToWalk = totalDistance / ctx->cfg->walkSpeed;

	while (queue.size() > 0) {
		if (ctx->calculationProgress != nullptr && ctx->calculationProgress->isCancelled()) {
			ctx->calculationProgress->setSegmentNotFound(0);
			return;
		}

		SHARED_PTR<TransportRouteSegment> segment = queue.top();
		queue.pop();
		SHARED_PTR<TransportRouteSegment> ex;

		int64_t segIdWithParent = segmentWithParentId(segment, segment->parentRoute);

		if (ctx->visitedSegments.find(segIdWithParent) != ctx->visitedSegments.end()) {
			ex = ctx->visitedSegments.find(segIdWithParent)->second;
			if (ex->distFromStart > segment->distFromStart) {
				OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "%.1f (%s) > %.1f (%s)", ex->distFromStart,
								  ex->to_string().c_str(), segment->distFromStart, segment->to_string().c_str());
			}
			continue;
		}
		ctx->visitedRoutesCount++;
		ctx->visitedSegments.insert({segIdWithParent, segment});

		if (segment->distFromStart > finishTime * ctx->cfg->increaseForAlternativesRoutes ||
			segment->distFromStart > maxTravelTimeCmpToWalk) {
			break;
		}

		// int64_t segmentId = segment->getId(); // TODO remove
		SHARED_PTR<TransportRouteSegment> finish = nullptr;
		double minDist = 0;
		double travelDist = 0;

		int onboardTime = ctx->cfg->getBoardingTime(segment->road->getType());
		int intervalTime = parseNumberSilently(segment->road->getInterval(), 0); // TODO check both functions
		if (intervalTime > 0) {
			onboardTime = intervalTime * 60 / 2;
		}
		double travelTime = onboardTime;

		const float routeTravelSpeed = ctx->cfg->getSpeedByRouteType(segment->road->type);

		if (routeTravelSpeed == 0) {
			continue;
		}
		SHARED_PTR<TransportStop> prevStop = segment->getStop(segment->segStart);
		vector<SHARED_PTR<TransportRouteSegment>> sgms;

		if (TRACE_ONBOARD_ID != 0) {
			ObfConstants::osmand_id_t id = ObfConstants::getOsmIdFromBinaryMapObjectId(segment->road->id);
			if (id == TRACE_ONBOARD_ID) {
				LogPrintf(OsmAnd::LogSeverityLevel::Debug, "TRACE_ONBOARD_ID -> %" PRId64 " (%d) %.2f (parent %s)",
				          id, segment->segStart, segment->distFromStart, segment->parentRoute
					                                                         ? segment->parentRoute->to_string().c_str()
					                                                         : "null");
			}
		}

		for (int32_t ind = 1 + segment->segStart; ind < segment->getLength(); ind++) {
			if (ctx->calculationProgress != nullptr && ctx->calculationProgress->isCancelled()) {
				return;
			}
			segIdWithParent++;
			ctx->visitedSegments.insert({segIdWithParent, segment});
			SHARED_PTR<TransportStop> stop = segment->getStop(ind);
			double segmentDist = getDistance(prevStop->lat, prevStop->lon, stop->lat, stop->lon);
			travelDist += segmentDist;

			if (ctx->cfg->useSchedule) {
				// TransportSchedule& sc = segment->road->schedule;
				int interval = segment->road->schedule.avgStopIntervals.at(ind - 1);
				travelTime += interval * 10;
			} else {
				int stopTime = ctx->cfg->getStopTime(segment->road->getType());
				travelTime += stopTime + segmentDist / routeTravelSpeed;
			}
			if (segment->distFromStart + travelTime > finishTime * ctx->cfg->increaseForAlternativesRoutes) {
				break;
			}
			sgms.clear();
			if (segment->getDepth() < ctx->cfg->maxNumberOfChanges + 1) {
				ctx->getTransportStops(stop->x31, stop->y31, true, sgms);
				ctx->visitedStops++;
				for (SHARED_PTR<TransportRouteSegment>& sgm : sgms) {
					if (ctx->calculationProgress != nullptr && ctx->calculationProgress->isCancelled()) {
						return;
					}
					if (segment->wasVisited(sgm)) {
						continue;
					}
					if (ctx->visitedSegments.find(segmentWithParentId(sgm, segment)) != ctx->visitedSegments.end()) {
						continue;
					}
					SHARED_PTR<TransportRouteSegment> nextSegment = make_shared<TransportRouteSegment>(sgm);
					nextSegment->parentRoute = segment;
					nextSegment->hasParentRoute = true;
					nextSegment->parentStop = ind;
					nextSegment->walkDist =
					getDistance(nextSegment->getLocationLat(), nextSegment->getLocationLon(), stop->lat, stop->lon);
					nextSegment->parentTravelTime = travelTime;
					nextSegment->parentTravelDist = travelDist;
					double walkTime = nextSegment->walkDist / ctx->cfg->walkSpeed +
						ctx->cfg->getChangeTime(segment->road->getType(), sgm->road->getType());
					nextSegment->distFromStart = segment->distFromStart + travelTime + walkTime;
					nextSegment->nonce = nonce++;
					if (ctx->cfg->useSchedule) {
						int tm = (sgm->departureTime - ctx->cfg->scheduleTimeOfDay) * 10;
						if (tm >= nextSegment->distFromStart) {
							nextSegment->distFromStart = tm;
							queue.push(nextSegment);
						}
					} else {
						queue.push(nextSegment);
					}
					if (TRACE_CHANGE_ID != 0) {
						ObfConstants::osmand_id_t from = ObfConstants::getOsmIdFromBinaryMapObjectId(segment->road->id);
						ObfConstants::osmand_id_t to = ObfConstants::getOsmIdFromBinaryMapObjectId(sgm->road->id);
						LogPrintf(OsmAnd::LogSeverityLevel::Debug,
						          "TRACE_CHANGE_ID ? Change %" PRId64 " (%d) -> %" PRId64" (%d) %.3f",
						          from, ind, to, sgm->segStart, sgm->distFromStart);
					}
				}
			}
			SHARED_PTR<TransportRouteSegment> finalSegment = nullptr;

			int64_t finalSegmentId = segment->getId() + ind - segment->segStart;
			if (endSegments.find(finalSegmentId) != endSegments.end()) {
				finalSegment = endSegments[finalSegmentId];
			}
			double distToEnd = getDistance(stop->lat, stop->lon, ctx->endLat, ctx->endLon);

			if (finalSegment != nullptr && distToEnd < ctx->cfg->walkRadius) {
				if (finish == nullptr || minDist > distToEnd) {
					minDist = distToEnd;
					finish = make_shared<TransportRouteSegment>(finalSegment);
					finish->parentRoute = segment;
					finish->hasParentRoute = true;
					finish->parentStop = ind;
					finish->walkDist = distToEnd;
					finish->parentTravelTime = travelTime;
					finish->parentTravelDist = travelDist;

					double walkTime = distToEnd / ctx->cfg->walkSpeed;
					finish->distFromStart = segment->distFromStart + travelTime + walkTime;
				}
			}
			prevStop = stop;
		}
		if (finish != nullptr) {
			if (finishTime > finish->distFromStart) {
				finishTime = finish->distFromStart;
			}
			if (finish->distFromStart < finishTime * ctx->cfg->increaseForAlternativesRoutes &&
				(finish->distFromStart < maxTravelTimeCmpToWalk || results.size() == 0)) {
				results.push_back(finish);
			}
		}

		if (ctx->calculationProgress != nullptr && ctx->calculationProgress->isCancelled()) {
			OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "Route calculation interrupted");
			return;
		}

		// updateCalculationProgress(ctx, queue);
	}
	pt_timer.Pause();
	OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info,
					  "[NATIVE] PT calculation took %.3f s, loading tiles "
					  "(overall): %.3f s, readTime : %.3f s",
					  (double)pt_timer.GetElapsedMs() / 1000.0, (double)ctx->loadTime.GetElapsedMs() / 1000.0,
					  (double)ctx->readTime.GetElapsedMs() / 1000.0);

	prepareResults(ctx, results, res);
}

void TransportRoutePlanner::updateCalculationProgress(unique_ptr<TransportRoutingContext>& ctx,
													  priority_queue<SHARED_PTR<TransportRouteSegment>>& queue) {
	if (ctx->calculationProgress.get()) {
		ctx->calculationProgress->directSegmentQueueSize = queue.size();
		if (queue.size() > 0) {
			SHARED_PTR<TransportRouteSegment> peek = queue.top();
			ctx->calculationProgress->distanceFromBegin =
				(int64_t)fmax(peek->distFromStart, ctx->calculationProgress->distanceFromBegin);
		}
	}
}

int64_t TransportRoutePlanner::segmentWithParentId(const SHARED_PTR<TransportRouteSegment>& segment,
                                                   const SHARED_PTR<TransportRouteSegment>& parent)
{
	// TODO add bool hasParentRoute (but first check if it possible to use std::shared_ptr == nullptr)
	// TODO check both callers for parent == nullptr and/or hasParentRoute !? -- might be a difference with Java
	return ((parent ? ObfConstants::getOsmIdFromBinaryMapObjectId(parent->road->id) : 0) << 30l) + segment->getId();
}

#endif	//_OSMAND_TRANSPORT_ROUTE_PLANNER_CPP
