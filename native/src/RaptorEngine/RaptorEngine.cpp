#include "RaptorEngine/RaptorEngine.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <unordered_set>
#include <vector>

namespace OsmAnd
{
    namespace RaptorEngine
    {
        namespace
        {
            constexpr double kInf = std::numeric_limits<double>::infinity();
            constexpr double kTickToSeconds = 10.0;

            static bool validateOffsets(const std::vector<int32_t>& offsets,
                                        size_t ownerCount,
                                        size_t payloadCount,
                                        const char* fieldName,
                                        std::string* errorMessage)
            {
                if (offsets.size() != ownerCount + 1)
                {
                    if (errorMessage)
                    {
                        std::ostringstream out;
                        out << fieldName << " must contain " << (ownerCount + 1) << " entries";
                        *errorMessage = out.str();
                    }
                    return false;
                }
                if (offsets.empty() || offsets.front() != 0)
                {
                    if (errorMessage)
                        *errorMessage = std::string(fieldName) + " must start with zero";
                    return false;
                }
                for (size_t i = 1; i < offsets.size(); ++i)
                {
                    if (offsets[i] < offsets[i - 1])
                    {
                        if (errorMessage)
                            *errorMessage = std::string(fieldName) + " must be monotonic";
                        return false;
                    }
                }
                if (static_cast<size_t>(offsets.back()) != payloadCount)
                {
                    if (errorMessage)
                    {
                        std::ostringstream out;
                        out << fieldName << " tail offset " << offsets.back()
                            << " does not match payload size " << payloadCount;
                        *errorMessage = out.str();
                    }
                    return false;
                }
                return true;
            }

            static bool isFinite(const double value)
            {
                return std::isfinite(value);
            }

            static bool isAlmostEqual(const double left, const double right)
            {
                return std::abs(left - right) <= 1e-9;
            }

            static double calculateComfortTimeWindowSeconds(const double bestTimeSeconds)
            {
                if (!isFinite(bestTimeSeconds) || !(bestTimeSeconds > 0.0))
                    return 600.0;
                return std::min(1800.0, std::max(600.0, bestTimeSeconds * 0.15));
            }

            static double calculateRankingTimeBandSeconds(const double bestTimeSeconds)
            {
                if (!isFinite(bestTimeSeconds) || !(bestTimeSeconds > 0.0))
                    return 300.0;
                return std::min(900.0, std::max(300.0, bestTimeSeconds * 0.08));
            }

            static int32_t calculateTimeBandIndex(const double totalTimeSeconds,
                                                  const double bestTimeSeconds,
                                                  const double timeBandSeconds)
            {
                if (!isFinite(totalTimeSeconds))
                    return std::numeric_limits<int32_t>::max();

                const auto timeDeltaSeconds = std::max(0.0, totalTimeSeconds - bestTimeSeconds);
                if (timeDeltaSeconds <= timeBandSeconds + 1e-9)
                    return 0;

                return 1 + static_cast<int32_t>(
                               std::floor((timeDeltaSeconds - timeBandSeconds - 1e-9) / timeBandSeconds));
            }

            static double calculateTotalWalkDistanceMeters(const RaptorJourney& journey)
            {
                double totalWalkDistanceMeters = std::max(0.0, journey.finishWalkDistanceMeters);
                for (const auto& segment : journey.segments)
                    totalWalkDistanceMeters += std::max(0.0, segment.walkDistanceMeters);
                return totalWalkDistanceMeters;
            }

            static double calculateComfortScoreSeconds(const RaptorJourney& journey)
            {
                const auto transfers =
                    journey.segments.empty() ? 0.0 : static_cast<double>(journey.segments.size() - 1);
                const auto segments = static_cast<double>(journey.segments.size());
                const auto totalWalkDistanceMeters = calculateTotalWalkDistanceMeters(journey);
                return journey.totalTimeSeconds +
                       transfers * 480.0 +
                       segments * 45.0 +
                       totalWalkDistanceMeters * 0.04;
            }

            static std::string buildJourneySignature(const RaptorJourney& journey)
            {
                std::ostringstream out;
                for (const auto& segment : journey.segments)
                {
                    out << segment.routeIndex << ':'
                        << segment.boardStopPosition << ':'
                        << segment.alightStopPosition << ';';
                }
                return out.str();
            }

            static std::string buildJourneyRouteChainSignature(const RaptorJourney& journey)
            {
                std::ostringstream out;
                for (const auto& segment : journey.segments)
                {
                    out << segment.routeIndex << ';';
                }
                return out.str();
            }

            constexpr int32_t kMiniBagSize = 4;
            constexpr int32_t kTimetableDepartureCandidates = 3;
            constexpr double kTimetableDepartureWindowSeconds = 1800.0;

            struct ActiveRideState
            {
                int32_t previousLabel = -1;
                int32_t boardStopPos = -1;
                int32_t currentStopPos = -1;
                int32_t departureTime = -1;
                double currentArrivalSeconds = kInf;
                double currentTravelSeconds = 0.0;
            };

            struct QueryState
            {
                const PackedTransportSnapshot& snapshot;
                const RaptorQueryOptions& options;
                const int32_t stopCount;
                const int32_t routeCount;
                const int32_t routeTypeCount;
                const int32_t rounds;
                const int32_t ownerCountPerRound;
                const int32_t maxBagSize;
                const int32_t timetableDepartureCandidates;
                const double timetableDepartureWindowSeconds;

                std::vector<std::vector<int32_t>> ownerBags;

                std::vector<int32_t> labelRounds;
                std::vector<int32_t> labelRouteTypes;
                std::vector<int32_t> labelStops;
                std::vector<double> arrivals;
                std::vector<int32_t> segmentCounts;
                std::vector<double> totalWalkDistances;
                std::vector<uint8_t> predecessorKind;
                std::vector<int32_t> predecessorLabel;
                std::vector<int32_t> predecessorRoute;
                std::vector<int32_t> predecessorBoardPos;
                std::vector<int32_t> predecessorAlightPos;
                std::vector<int32_t> predecessorDepartureTime;
                std::vector<double> predecessorWalkDistance;
                std::vector<double> predecessorTravelTime;

                std::vector<int> routeMarkedAtRound;
                std::vector<double> finalJourneyTimes;
                std::vector<double> finalJourneyEndWalkDistances;
                std::vector<char> finalJourneySeen;
                std::vector<int32_t> finalJourneyLabels;
                std::vector<RaptorTraceEntry> trace;

                int32_t bestFinalLabel = -1;
                double bestFinalTimeSeconds = kInf;
                double bestFinalEndWalkDistance = 0.0;

                QueryState(const PackedTransportSnapshot& snapshot_,
                           const RaptorQueryOptions& options_,
                           const bool collectTrace)
                    : snapshot(snapshot_)
                    , options(options_)
                    , stopCount(snapshot_.stopCount)
                    , routeCount(snapshot_.routeCount)
                    , routeTypeCount(snapshot_.routeTypeCount)
                    , rounds(std::max(0, options_.maxRounds) + 1)
                    , ownerCountPerRound(snapshot_.stopCount * snapshot_.routeTypeCount)
                    , maxBagSize(kMiniBagSize)
                    , timetableDepartureCandidates(kTimetableDepartureCandidates)
                    , timetableDepartureWindowSeconds(kTimetableDepartureWindowSeconds)
                    , ownerBags(static_cast<size_t>(rounds) * ownerCountPerRound)
                    , routeMarkedAtRound(snapshot_.routeCount, -1)
                {
                    if (!collectTrace)
                        trace.clear();
                }

                int32_t ownerIndex(const int32_t round, const int32_t routeType, const int32_t stop) const
                {
                    return round * ownerCountPerRound + routeType * stopCount + stop;
                }

                std::vector<int32_t>& bag(const int32_t round, const int32_t routeType, const int32_t stop)
                {
                    return ownerBags[ownerIndex(round, routeType, stop)];
                }

                const std::vector<int32_t>& bag(const int32_t round,
                                                const int32_t routeType,
                                                const int32_t stop) const
                {
                    return ownerBags[ownerIndex(round, routeType, stop)];
                }

                void decodeLabel(const int32_t labelIndex, int32_t& round, int32_t& routeType, int32_t& stop) const
                {
                    round = labelRounds[labelIndex];
                    routeType = labelRouteTypes[labelIndex];
                    stop = labelStops[labelIndex];
                }

                double boardingSeconds(const int32_t routeType) const
                {
                    if (routeType < 0 || routeType >= static_cast<int32_t>(options.boardingSecondsByRouteType.size()))
                        return 0.0;
                    return options.boardingSecondsByRouteType[routeType];
                }

                double changeSeconds(const int32_t fromRouteType, const int32_t toRouteType) const
                {
                    const auto expected = routeTypeCount * routeTypeCount;
                    if (static_cast<int32_t>(options.changeSecondsMatrix.size()) != expected)
                        return 0.0;
                    if (fromRouteType < 0 || toRouteType < 0 ||
                        fromRouteType >= routeTypeCount || toRouteType >= routeTypeCount)
                        return 0.0;
                    return options.changeSecondsMatrix[fromRouteType * routeTypeCount + toRouteType];
                }

                bool routeHasSchedule(const int32_t routeIndex) const
                {
                    const auto& tripOffsets = snapshot.transitData.scheduleTripOffsets;
                    if (tripOffsets.size() != static_cast<size_t>(routeCount + 1))
                        return false;
                    return tripOffsets[routeIndex + 1] > tripOffsets[routeIndex];
                }

                double compatTravelSeconds(const int32_t routeIndex, const int32_t fromPos, const int32_t toPos) const
                {
                    const auto base = snapshot.transitData.routeStopOffsets[routeIndex];
                    return snapshot.transitData.cumulativeCompatTravelSeconds[base + toPos] -
                           snapshot.transitData.cumulativeCompatTravelSeconds[base + fromPos];
                }

                double travelDistanceMeters(const int32_t routeIndex, const int32_t fromPos, const int32_t toPos) const
                {
                    const auto base = snapshot.transitData.routeStopOffsets[routeIndex];
                    return snapshot.transitData.cumulativeTravelDistancesMeters[base + toPos] -
                           snapshot.transitData.cumulativeTravelDistancesMeters[base + fromPos];
                }

                double scheduleTravelSeconds(const int32_t routeIndex, const int32_t fromPos, const int32_t toPos) const
                {
                    const auto base = snapshot.transitData.routeStopOffsets[routeIndex];
                    return static_cast<double>(
                               snapshot.transitData.cumulativeScheduleOffsetsDeciSeconds[base + toPos] -
                               snapshot.transitData.cumulativeScheduleOffsetsDeciSeconds[base + fromPos]) *
                           kTickToSeconds;
                }

                double headwayPenaltySeconds(const int32_t routeIndex, const int32_t stopPos) const
                {
                    if (options.schedulelessMode != SchedulelessMode::Headway)
                        return 0.0;

                    const auto metricIndex = snapshot.transitData.routeStopOffsets[routeIndex] + stopPos;
                    if (metricIndex >= 0 &&
                        metricIndex < static_cast<int32_t>(snapshot.transitData.expectedWaitSeconds.size()))
                    {
                        const auto explicitWait = snapshot.transitData.expectedWaitSeconds[metricIndex];
                        if (explicitWait > 0.0)
                            return explicitWait;
                    }

                    if (routeIndex >= 0 &&
                        routeIndex < static_cast<int32_t>(snapshot.transitData.routeDefaultHeadwaySeconds.size()))
                    {
                        const auto headway = snapshot.transitData.routeDefaultHeadwaySeconds[routeIndex];
                        if (headway > 0.0)
                            return headway * 0.5;
                    }

                    return 0.0;
                }

                double schedulelessBoardingSeconds(const int32_t routeIndex,
                                                   const int32_t stopPos,
                                                   const int32_t routeType) const
                {
                    const auto metricIndex = snapshot.transitData.routeStopOffsets[routeIndex] + stopPos;
                    if (metricIndex >= 0 &&
                        metricIndex < static_cast<int32_t>(snapshot.transitData.boardingCostSeconds.size()))
                    {
                        const auto explicitBoarding = snapshot.transitData.boardingCostSeconds[metricIndex];
                        if (std::isfinite(explicitBoarding) && explicitBoarding >= 0.0)
                            return explicitBoarding;
                    }

                    return boardingSeconds(routeType) + headwayPenaltySeconds(routeIndex, stopPos);
                }

                int32_t earliestTripDeparture(const int32_t routeIndex,
                                              const int32_t stopPos,
                                              const double earliestDepartureSeconds) const
                {
                    if (!routeHasSchedule(routeIndex))
                        return -1;

                    const auto routeBase = snapshot.transitData.routeStopOffsets[routeIndex];
                    const auto scheduleOffset =
                        snapshot.transitData.cumulativeScheduleOffsetsDeciSeconds[routeBase + stopPos];
                    const auto targetTick =
                        static_cast<int32_t>(std::ceil(earliestDepartureSeconds / kTickToSeconds));

                    auto begin = snapshot.transitData.scheduleTripStartsDeciSeconds.begin() +
                                 snapshot.transitData.scheduleTripOffsets[routeIndex];
                    auto end = snapshot.transitData.scheduleTripStartsDeciSeconds.begin() +
                               snapshot.transitData.scheduleTripOffsets[routeIndex + 1];
                    auto it = std::lower_bound(begin, end, targetTick - scheduleOffset);
                    if (it == end)
                        return -1;
                    return (*it) + scheduleOffset;
                }

                void collectTripDepartures(const int32_t routeIndex,
                                           const int32_t stopPos,
                                           const double earliestDepartureSeconds,
                                           std::vector<int32_t>& departures) const
                {
                    departures.clear();
                    if (!routeHasSchedule(routeIndex))
                        return;

                    const auto routeBase = snapshot.transitData.routeStopOffsets[routeIndex];
                    const auto scheduleOffset =
                        snapshot.transitData.cumulativeScheduleOffsetsDeciSeconds[routeBase + stopPos];
                    const auto targetTick =
                        static_cast<int32_t>(std::ceil(earliestDepartureSeconds / kTickToSeconds));

                    auto begin = snapshot.transitData.scheduleTripStartsDeciSeconds.begin() +
                                 snapshot.transitData.scheduleTripOffsets[routeIndex];
                    auto end = snapshot.transitData.scheduleTripStartsDeciSeconds.begin() +
                               snapshot.transitData.scheduleTripOffsets[routeIndex + 1];
                    auto it = std::lower_bound(begin, end, targetTick - scheduleOffset);
                    if (it == end)
                        return;

                    const auto firstDeparture = (*it) + scheduleOffset;
                    const auto maxDeparture =
                        firstDeparture + static_cast<int32_t>(std::ceil(timetableDepartureWindowSeconds / kTickToSeconds));
                    for (; it != end && static_cast<int32_t>(departures.size()) < timetableDepartureCandidates; ++it)
                    {
                        const auto departure = (*it) + scheduleOffset;
                        if (departure > maxDeparture)
                            break;
                        departures.push_back(departure);
                    }
                }

                bool labelDominates(const int32_t existingLabel,
                                    const double arrivalSeconds,
                                    const int32_t candidateSegmentCount,
                                    const double candidateWalkDistance) const
                {
                    const auto existingArrival = arrivals[existingLabel];
                    const auto existingSegments = segmentCounts[existingLabel];
                    const auto existingWalk = totalWalkDistances[existingLabel];
                    const auto notWorse =
                        existingArrival <= arrivalSeconds &&
                        existingSegments <= candidateSegmentCount &&
                        existingWalk <= candidateWalkDistance;
                    const auto strictlyBetter =
                        existingArrival < arrivalSeconds ||
                        existingSegments < candidateSegmentCount ||
                        existingWalk < candidateWalkDistance;
                    return notWorse && strictlyBetter;
                }

                bool candidateDominatesLabel(const double arrivalSeconds,
                                             const int32_t candidateSegmentCount,
                                             const double candidateWalkDistance,
                                             const int32_t existingLabel) const
                {
                    const auto existingArrival = arrivals[existingLabel];
                    const auto existingSegments = segmentCounts[existingLabel];
                    const auto existingWalk = totalWalkDistances[existingLabel];
                    const auto notWorse =
                        arrivalSeconds <= existingArrival &&
                        candidateSegmentCount <= existingSegments &&
                        candidateWalkDistance <= existingWalk;
                    const auto strictlyBetter =
                        arrivalSeconds < existingArrival ||
                        candidateSegmentCount < existingSegments ||
                        candidateWalkDistance < existingWalk;
                    return notWorse && strictlyBetter;
                }

                bool sameLabelState(const int32_t existingLabel,
                                    const double arrivalSeconds,
                                    const int32_t candidateSegmentCount,
                                    const double candidateWalkDistance,
                                    const RaptorPredecessorKind kind,
                                    const int32_t previousLabel,
                                    const int32_t routeIndex,
                                    const int32_t boardPos,
                                    const int32_t alightPos,
                                    const int32_t departureTime) const
                {
                    return isAlmostEqual(arrivals[existingLabel], arrivalSeconds) &&
                           segmentCounts[existingLabel] == candidateSegmentCount &&
                           isAlmostEqual(totalWalkDistances[existingLabel], candidateWalkDistance) &&
                           predecessorKind[existingLabel] == static_cast<uint8_t>(kind) &&
                           predecessorLabel[existingLabel] == previousLabel &&
                           predecessorRoute[existingLabel] == routeIndex &&
                           predecessorBoardPos[existingLabel] == boardPos &&
                           predecessorAlightPos[existingLabel] == alightPos &&
                           predecessorDepartureTime[existingLabel] == departureTime;
                }

                bool betterThanLabel(const double arrivalSeconds,
                                     const int32_t candidateSegmentCount,
                                     const double candidateWalkDistance,
                                     const int32_t existingLabel) const
                {
                    if (arrivalSeconds != arrivals[existingLabel])
                        return arrivalSeconds < arrivals[existingLabel];
                    if (candidateSegmentCount != segmentCounts[existingLabel])
                        return candidateSegmentCount < segmentCounts[existingLabel];
                    if (candidateWalkDistance != totalWalkDistances[existingLabel])
                        return candidateWalkDistance < totalWalkDistances[existingLabel];
                    return false;
                }

                bool betterLabel(const int32_t leftLabel, const int32_t rightLabel) const
                {
                    if (arrivals[leftLabel] != arrivals[rightLabel])
                        return arrivals[leftLabel] < arrivals[rightLabel];
                    if (segmentCounts[leftLabel] != segmentCounts[rightLabel])
                        return segmentCounts[leftLabel] < segmentCounts[rightLabel];
                    if (totalWalkDistances[leftLabel] != totalWalkDistances[rightLabel])
                        return totalWalkDistances[leftLabel] < totalWalkDistances[rightLabel];
                    return leftLabel < rightLabel;
                }

                int32_t createLabel(const int32_t round,
                                    const int32_t routeType,
                                    const int32_t stop,
                                    const double arrivalSeconds,
                                    const int32_t candidateSegmentCount,
                                    const double candidateWalkDistance,
                                    const RaptorPredecessorKind kind,
                                    const int32_t previousLabel,
                                    const int32_t routeIndex,
                                    const int32_t boardPos,
                                    const int32_t alightPos,
                                    const int32_t departureTime,
                                    const double walkDistanceMeters,
                                    const double travelTimeSeconds,
                                    const bool collectTrace)
                {
                    const auto label = static_cast<int32_t>(arrivals.size());
                    labelRounds.push_back(round);
                    labelRouteTypes.push_back(routeType);
                    labelStops.push_back(stop);
                    arrivals.push_back(arrivalSeconds);
                    segmentCounts.push_back(candidateSegmentCount);
                    totalWalkDistances.push_back(candidateWalkDistance);
                    predecessorKind.push_back(static_cast<uint8_t>(kind));
                    predecessorLabel.push_back(previousLabel);
                    predecessorRoute.push_back(routeIndex);
                    predecessorBoardPos.push_back(boardPos);
                    predecessorAlightPos.push_back(alightPos);
                    predecessorDepartureTime.push_back(departureTime);
                    predecessorWalkDistance.push_back(walkDistanceMeters);
                    predecessorTravelTime.push_back(travelTimeSeconds);
                    finalJourneyTimes.push_back(kInf);
                    finalJourneyEndWalkDistances.push_back(0.0);
                    finalJourneySeen.push_back(0);

                    if (collectTrace)
                    {
                        RaptorTraceEntry entry;
                        entry.round = round;
                        entry.stopIndex = stop;
                        entry.lastRouteTypeIndex = routeType;
                        entry.arrivalSeconds = arrivalSeconds;
                        entry.predecessorKind = kind;
                        entry.previousLabelIndex = previousLabel;
                        entry.routeIndex = routeIndex;
                        entry.boardStopPosition = boardPos;
                        entry.alightStopPosition = alightPos;
                        entry.departureTimeDeciSeconds = departureTime;
                        entry.walkDistanceMeters = walkDistanceMeters;
                        entry.travelTimeSeconds = travelTimeSeconds;
                        trace.push_back(entry);
                    }

                    return label;
                }

                int32_t improveLabel(const int32_t round,
                                     const int32_t routeType,
                                     const int32_t stop,
                                     const double arrivalSeconds,
                                  const RaptorPredecessorKind kind,
                                  const int32_t previousLabel,
                                  const int32_t routeIndex,
                                  const int32_t boardPos,
                                  const int32_t alightPos,
                                  const int32_t departureTime,
                                  const double walkDistanceMeters,
                                  const double travelTimeSeconds,
                                  const bool collectTrace)
                {
                    int32_t candidateSegmentCount = 0;
                    if (kind == RaptorPredecessorKind::Ride)
                    {
                        const auto previousSegments =
                            previousLabel >= 0 ? segmentCounts[previousLabel] : 0;
                        candidateSegmentCount =
                            previousSegments == std::numeric_limits<int32_t>::max() ? 1 : previousSegments + 1;
                    }
                    else if (kind == RaptorPredecessorKind::Walk)
                    {
                        candidateSegmentCount =
                            previousLabel >= 0 ? segmentCounts[previousLabel] : 0;
                    }

                    double candidateWalkDistance = 0.0;
                    if (kind == RaptorPredecessorKind::Start)
                        candidateWalkDistance = walkDistanceMeters;
                    else if (kind == RaptorPredecessorKind::Walk)
                        candidateWalkDistance =
                            (previousLabel >= 0 ? totalWalkDistances[previousLabel] : 0.0) + walkDistanceMeters;
                    else
                        candidateWalkDistance = previousLabel >= 0 ? totalWalkDistances[previousLabel] : 0.0;

                    if (arrivalSeconds > options.maxRouteTimeSeconds)
                        return -1;

                    auto filteredBag = bag(round, routeType, stop);
                    std::vector<int32_t> keptLabels;
                    keptLabels.reserve(filteredBag.size() + 1);
                    for (const auto existingLabel : filteredBag)
                    {
                        if (sameLabelState(existingLabel,
                                           arrivalSeconds,
                                           candidateSegmentCount,
                                           candidateWalkDistance,
                                           kind,
                                           previousLabel,
                                           routeIndex,
                                           boardPos,
                                           alightPos,
                                           departureTime) ||
                            labelDominates(existingLabel,
                                           arrivalSeconds,
                                           candidateSegmentCount,
                                           candidateWalkDistance))
                        {
                            return -1;
                        }

                        if (!candidateDominatesLabel(arrivalSeconds,
                                                     candidateSegmentCount,
                                                     candidateWalkDistance,
                                                     existingLabel))
                        {
                            keptLabels.push_back(existingLabel);
                        }
                    }

                    while (static_cast<int32_t>(keptLabels.size()) >= maxBagSize)
                    {
                        auto worstIt = keptLabels.begin();
                        for (auto it = keptLabels.begin() + 1; it != keptLabels.end(); ++it)
                        {
                            if (betterLabel(*worstIt, *it))
                                worstIt = it;
                        }
                        if (!betterThanLabel(arrivalSeconds,
                                             candidateSegmentCount,
                                             candidateWalkDistance,
                                             *worstIt))
                        {
                            return -1;
                        }
                        keptLabels.erase(worstIt);
                    }

                    const auto label = createLabel(round,
                                                   routeType,
                                                   stop,
                                                   arrivalSeconds,
                                                   candidateSegmentCount,
                                                   candidateWalkDistance,
                                                   kind,
                                                   previousLabel,
                                                   routeIndex,
                                                   boardPos,
                                                   alightPos,
                                                   departureTime,
                                                   walkDistanceMeters,
                                                   travelTimeSeconds,
                                                   collectTrace);
                    keptLabels.push_back(label);
                    std::sort(keptLabels.begin(),
                              keptLabels.end(),
                              [this](const int32_t leftLabel, const int32_t rightLabel)
                              {
                                  return betterLabel(leftLabel, rightLabel);
                              });
                    bag(round, routeType, stop) = std::move(keptLabels);
                    return label;
                }
            };

            static void pushImprovedStop(const int32_t stop,
                                         std::vector<char>& stopSeen,
                                         std::vector<int32_t>& stopList)
            {
                if (!stopSeen[stop])
                {
                    stopSeen[stop] = 1;
                    stopList.push_back(stop);
                }
            }

            static int32_t activeRideSegmentCount(const QueryState& state, const ActiveRideState& ride)
            {
                if (ride.previousLabel < 0)
                    return 1;
                const auto previousSegments = state.segmentCounts[ride.previousLabel];
                return previousSegments == std::numeric_limits<int32_t>::max() ? 1 : previousSegments + 1;
            }

            static double activeRideWalkDistance(const QueryState& state, const ActiveRideState& ride)
            {
                if (ride.previousLabel < 0)
                    return 0.0;
                return state.totalWalkDistances[ride.previousLabel];
            }

            static bool activeRideDominates(const QueryState& state,
                                            const ActiveRideState& left,
                                            const ActiveRideState& right)
            {
                const auto leftSegments = activeRideSegmentCount(state, left);
                const auto rightSegments = activeRideSegmentCount(state, right);
                const auto leftWalkDistance = activeRideWalkDistance(state, left);
                const auto rightWalkDistance = activeRideWalkDistance(state, right);
                const auto notWorse =
                    left.currentArrivalSeconds <= right.currentArrivalSeconds &&
                    leftSegments <= rightSegments &&
                    leftWalkDistance <= rightWalkDistance;
                const auto strictlyBetter =
                    left.currentArrivalSeconds < right.currentArrivalSeconds ||
                    leftSegments < rightSegments ||
                    leftWalkDistance < rightWalkDistance;
                return notWorse && strictlyBetter;
            }

            static void pruneActiveRides(const QueryState& state, std::vector<ActiveRideState>& activeRides)
            {
                std::sort(activeRides.begin(),
                          activeRides.end(),
                          [&state](const ActiveRideState& left, const ActiveRideState& right)
                          {
                              if (left.currentArrivalSeconds != right.currentArrivalSeconds)
                                  return left.currentArrivalSeconds < right.currentArrivalSeconds;

                              const auto leftSegments = activeRideSegmentCount(state, left);
                              const auto rightSegments = activeRideSegmentCount(state, right);
                              if (leftSegments != rightSegments)
                                  return leftSegments < rightSegments;

                              const auto leftWalkDistance = activeRideWalkDistance(state, left);
                              const auto rightWalkDistance = activeRideWalkDistance(state, right);
                              if (leftWalkDistance != rightWalkDistance)
                                  return leftWalkDistance < rightWalkDistance;

                              return left.previousLabel < right.previousLabel;
                          });

                std::vector<ActiveRideState> keptRides;
                keptRides.reserve(activeRides.size());
                for (const auto& candidateRide : activeRides)
                {
                    bool dominated = false;
                    for (const auto& keptRide : keptRides)
                    {
                        if (activeRideDominates(state, keptRide, candidateRide))
                        {
                            dominated = true;
                            break;
                        }
                    }
                    if (dominated)
                        continue;

                    keptRides.push_back(candidateRide);
                    if (static_cast<int32_t>(keptRides.size()) >= state.maxBagSize * 2)
                        break;
                }
                activeRides.swap(keptRides);
            }

            static void tryFinish(QueryState& state, const int32_t label)
            {
                int32_t round = -1;
                int32_t routeType = -1;
                int32_t stop = -1;
                state.decodeLabel(label, round, routeType, stop);
                if (round <= 0 || stop < 0 || stop >= state.stopCount)
                    return;
                if (stop >= static_cast<int32_t>(state.snapshot.endWalkSeconds.size()))
                    return;

                const auto endWalkSeconds = state.snapshot.endWalkSeconds[stop];
                if (!isFinite(endWalkSeconds))
                    return;

                const auto total = state.arrivals[label] + endWalkSeconds;
                if (total < state.finalJourneyTimes[label])
                {
                    state.finalJourneyTimes[label] = total;
                    state.finalJourneyEndWalkDistances[label] =
                        stop < static_cast<int32_t>(state.snapshot.endWalkDistancesMeters.size())
                            ? state.snapshot.endWalkDistancesMeters[stop]
                            : 0.0;
                    if (!state.finalJourneySeen[label])
                    {
                        state.finalJourneySeen[label] = 1;
                        state.finalJourneyLabels.push_back(label);
                    }
                }

                if (total < state.bestFinalTimeSeconds)
                {
                    state.bestFinalTimeSeconds = total;
                    state.bestFinalLabel = label;
                    state.bestFinalEndWalkDistance = state.finalJourneyEndWalkDistances[label];
                }
            }

            static void relaxFootpathsForRound(QueryState& state,
                                               const int32_t round,
                                               const std::vector<int32_t>& seedLabels,
                                               std::vector<int32_t>& improvedStops,
                                               const bool collectTrace)
            {
                std::vector<char> stopSeen(state.stopCount, 0);
                for (const auto stop : improvedStops)
                    stopSeen[stop] = 1;

                for (const auto label : seedLabels)
                {
                    int32_t decodedRound = -1;
                    int32_t routeType = -1;
                    int32_t stop = -1;
                    state.decodeLabel(label, decodedRound, routeType, stop);
                    if (decodedRound != round || stop < 0 || stop >= state.stopCount)
                        continue;

                    for (int32_t edge = state.snapshot.transitData.footpathOffsets[stop];
                         edge < state.snapshot.transitData.footpathOffsets[stop + 1];
                         ++edge)
                    {
                        const auto targetStop = state.snapshot.transitData.footpathTargetIndices[edge];
                        const auto candidateArrival =
                            state.arrivals[label] + state.snapshot.transitData.footpathTimesSeconds[edge];
                        const auto targetLabel = state.improveLabel(round,
                                                                    routeType,
                                                                    targetStop,
                                                                    candidateArrival,
                                                                    RaptorPredecessorKind::Walk,
                                                                    label,
                                                                    -1,
                                                                    -1,
                                                                    -1,
                                                                    -1,
                                                                    state.snapshot.transitData.footpathDistancesMeters[edge],
                                                                    state.snapshot.transitData.footpathTimesSeconds[edge],
                                                                    collectTrace);
                        if (targetLabel >= 0)
                        {
                            pushImprovedStop(targetStop, stopSeen, improvedStops);
                            tryFinish(state, targetLabel);
                        }
                    }
                }
            }

            static std::vector<int32_t> reconstructLabels(const QueryState& state, const int32_t finalLabel)
            {
                std::vector<int32_t> labels;
                auto cursor = finalLabel;
                const auto guardLimit = static_cast<int32_t>(state.arrivals.size()) + 1;
                while (cursor >= 0 && static_cast<int32_t>(labels.size()) <= guardLimit)
                {
                    labels.push_back(cursor);
                    const auto kind = static_cast<RaptorPredecessorKind>(state.predecessorKind[cursor]);
                    if (kind == RaptorPredecessorKind::Start)
                        break;
                    cursor = state.predecessorLabel[cursor];
                }
                std::reverse(labels.begin(), labels.end());
                return labels;
            }

            static RaptorJourney buildJourney(const QueryState& state,
                                              const int32_t finalLabel,
                                              const double totalTimeSeconds,
                                              const double finishWalkDistanceMeters)
            {
                RaptorJourney journey;
                if (finalLabel < 0 || !isFinite(totalTimeSeconds))
                    return journey;

                const auto labels = reconstructLabels(state, finalLabel);
                double pendingWalkDistance = 0.0;
                for (const auto label : labels)
                {
                    const auto kind = static_cast<RaptorPredecessorKind>(state.predecessorKind[label]);
                    if (kind == RaptorPredecessorKind::Start || kind == RaptorPredecessorKind::Walk)
                    {
                        pendingWalkDistance += state.predecessorWalkDistance[label];
                        continue;
                    }
                    if (kind != RaptorPredecessorKind::Ride)
                        continue;

                    RaptorRouteSegment segment;
                    segment.routeIndex = state.predecessorRoute[label];
                    segment.boardStopPosition = state.predecessorBoardPos[label];
                    segment.alightStopPosition = state.predecessorAlightPos[label];
                    segment.departureTimeDeciSeconds = state.predecessorDepartureTime[label];
                    segment.travelTimeSeconds = state.predecessorTravelTime[label];
                    segment.walkDistanceMeters = pendingWalkDistance;
                    int32_t round = -1;
                    int32_t routeType = -1;
                    int32_t stop = -1;
                    state.decodeLabel(label, round, routeType, stop);
                    segment.round = round;
                    if (segment.routeIndex >= 0 &&
                        segment.boardStopPosition >= 0 &&
                        segment.alightStopPosition >= segment.boardStopPosition)
                    {
                        segment.travelDistanceMeters =
                            state.travelDistanceMeters(segment.routeIndex,
                                                       segment.boardStopPosition,
                                                       segment.alightStopPosition);
                    }
                    journey.segments.push_back(segment);
                    pendingWalkDistance = 0.0;
                }

                journey.found = !journey.segments.empty();
                if (!journey.found)
                    return journey;

                journey.totalTimeSeconds = totalTimeSeconds;
                journey.finishWalkDistanceMeters = pendingWalkDistance + finishWalkDistanceMeters;
                state.decodeLabel(finalLabel, journey.finalRound, journey.finalRouteTypeIndex, journey.finalStopIndex);
                return journey;
            }

            static RaptorQueryResult buildResult(const QueryState& state)
            {
                RaptorQueryResult result;
                result.trace = state.trace;
                if (state.finalJourneyLabels.empty())
                    return result;

                auto finalLabels = state.finalJourneyLabels;
                std::sort(finalLabels.begin(),
                          finalLabels.end(),
                          [&state](const int32_t left, const int32_t right)
                          {
                              const auto leftTime = state.finalJourneyTimes[left];
                              const auto rightTime = state.finalJourneyTimes[right];
                              if (leftTime != rightTime)
                                  return leftTime < rightTime;
                              return left < right;
                          });

                std::vector<RaptorJourney> candidateJourneys;
                candidateJourneys.reserve(finalLabels.size());
                for (const auto finalLabel : finalLabels)
                {
                    auto journey = buildJourney(state,
                                                finalLabel,
                                                state.finalJourneyTimes[finalLabel],
                                                state.finalJourneyEndWalkDistances[finalLabel]);
                    if (!journey.found)
                        continue;
                    candidateJourneys.push_back(std::move(journey));
                }

                if (candidateJourneys.empty())
                    return result;

                const auto journeyTransferCount = [](const RaptorJourney& journey) -> int32_t
                {
                    return journey.segments.empty()
                        ? 0
                        : static_cast<int32_t>(journey.segments.size()) - 1;
                };
                std::sort(candidateJourneys.begin(),
                          candidateJourneys.end(),
                          [&journeyTransferCount](const RaptorJourney& left, const RaptorJourney& right)
                          {
                              if (left.totalTimeSeconds != right.totalTimeSeconds)
                                  return left.totalTimeSeconds < right.totalTimeSeconds;

                              const auto leftTransfers = journeyTransferCount(left);
                              const auto rightTransfers = journeyTransferCount(right);
                              if (leftTransfers != rightTransfers)
                                  return leftTransfers < rightTransfers;

                              if (left.segments.size() != right.segments.size())
                                  return left.segments.size() < right.segments.size();

                              if (left.finishWalkDistanceMeters != right.finishWalkDistanceMeters)
                                  return left.finishWalkDistanceMeters < right.finishWalkDistanceMeters;

                              return left.finalStopIndex < right.finalStopIndex;
                          });
                std::vector<RaptorJourney> rankedJourneys;
                rankedJourneys.reserve(candidateJourneys.size());
                std::unordered_set<std::string> seenSignatures;
                seenSignatures.reserve(candidateJourneys.size());
                const auto journeyPoolLimit =
                    std::max<int32_t>(512, std::min<int32_t>(state.options.maxJourneys * 16, 8192));
                for (auto& candidate : candidateJourneys)
                {
                    auto signature = buildJourneySignature(candidate);
                    if (!seenSignatures.emplace(std::move(signature)).second)
                        continue;
                    rankedJourneys.push_back(std::move(candidate));
                    if (static_cast<int32_t>(rankedJourneys.size()) >= journeyPoolLimit)
                        break;
                }

                if (rankedJourneys.empty())
                    return result;

                const auto bestTimeSeconds = rankedJourneys.front().totalTimeSeconds;
                const auto nearBestTimeWindowSeconds = calculateComfortTimeWindowSeconds(bestTimeSeconds);
                const auto rankingTimeBandSeconds = calculateRankingTimeBandSeconds(bestTimeSeconds);
                std::stable_sort(rankedJourneys.begin(),
                                 rankedJourneys.end(),
                                 [bestTimeSeconds, nearBestTimeWindowSeconds, rankingTimeBandSeconds, &journeyTransferCount](const RaptorJourney& left,
                                                                                                                           const RaptorJourney& right)
                                 {
                                      const auto leftNearBest =
                                          left.totalTimeSeconds <= bestTimeSeconds + nearBestTimeWindowSeconds;
                                      const auto rightNearBest =
                                          right.totalTimeSeconds <= bestTimeSeconds + nearBestTimeWindowSeconds;
                                      if (leftNearBest != rightNearBest)
                                          return leftNearBest;

                                      const auto leftTimeBand =
                                          calculateTimeBandIndex(left.totalTimeSeconds, bestTimeSeconds, rankingTimeBandSeconds);
                                      const auto rightTimeBand =
                                          calculateTimeBandIndex(right.totalTimeSeconds, bestTimeSeconds, rankingTimeBandSeconds);
                                      if (leftTimeBand != rightTimeBand)
                                          return leftTimeBand < rightTimeBand;

                                      const auto leftTransfers = journeyTransferCount(left);
                                      const auto rightTransfers = journeyTransferCount(right);
                                      if (leftTransfers != rightTransfers)
                                          return leftTransfers < rightTransfers;

                                      if (left.totalTimeSeconds != right.totalTimeSeconds)
                                          return left.totalTimeSeconds < right.totalTimeSeconds;

                                      const auto leftWalkDistanceMeters = calculateTotalWalkDistanceMeters(left);
                                      const auto rightWalkDistanceMeters = calculateTotalWalkDistanceMeters(right);
                                      if (!isAlmostEqual(leftWalkDistanceMeters, rightWalkDistanceMeters))
                                          return leftWalkDistanceMeters < rightWalkDistanceMeters;

                                      if (left.segments.size() != right.segments.size())
                                          return left.segments.size() < right.segments.size();

                                      return left.finalStopIndex < right.finalStopIndex;
                                 });

                const auto availableJourneys = static_cast<int32_t>(rankedJourneys.size());
                const auto requestedJourneys = std::max<int32_t>(10, state.options.maxJourneys);
                const auto maxJourneys =
                    std::max<int32_t>(1, std::min<int32_t>(requestedJourneys, availableJourneys));

                std::vector<int32_t> selectedIndices;
                selectedIndices.reserve(maxJourneys);
                std::unordered_set<std::string> selectedPathSignatures;
                selectedPathSignatures.reserve(maxJourneys * 2);
                std::unordered_set<std::string> selectedRouteChains;
                selectedRouteChains.reserve(maxJourneys * 2);

                const auto trySelectJourney =
                    [&rankedJourneys, &selectedIndices, &selectedPathSignatures, &selectedRouteChains](
                        const int32_t index,
                        const bool allowUsedRouteChain) -> bool
                {
                    if (index < 0 || index >= static_cast<int32_t>(rankedJourneys.size()))
                        return false;

                    const auto pathSignature = buildJourneySignature(rankedJourneys[index]);
                    if (!selectedPathSignatures.emplace(pathSignature).second)
                        return false;

                    const auto routeChainSignature = buildJourneyRouteChainSignature(rankedJourneys[index]);
                    if (!allowUsedRouteChain && selectedRouteChains.count(routeChainSignature) > 0)
                    {
                        selectedPathSignatures.erase(pathSignature);
                        return false;
                    }

                    selectedIndices.push_back(index);
                    selectedRouteChains.emplace(std::move(routeChainSignature));
                    return true;
                };

                trySelectJourney(0, true);

                int32_t leastTransfersIndex = -1;
                int32_t leastWalkIndex = -1;
                int32_t comfortIndex = -1;
                int32_t leastSegmentsIndex = -1;
                const auto preferenceTimeWindowSeconds =
                    std::max(rankingTimeBandSeconds, std::min(nearBestTimeWindowSeconds, rankingTimeBandSeconds * 2.0));
                for (int32_t i = 0; i < availableJourneys; ++i)
                {
                    const auto& journey = rankedJourneys[i];
                    if (journey.totalTimeSeconds > bestTimeSeconds + preferenceTimeWindowSeconds)
                        break;

                    if (leastTransfersIndex < 0 ||
                        journeyTransferCount(journey) < journeyTransferCount(rankedJourneys[leastTransfersIndex]) ||
                        (journeyTransferCount(journey) == journeyTransferCount(rankedJourneys[leastTransfersIndex]) &&
                         journey.totalTimeSeconds < rankedJourneys[leastTransfersIndex].totalTimeSeconds))
                    {
                        leastTransfersIndex = i;
                    }

                    if (leastSegmentsIndex < 0 ||
                        journey.segments.size() < rankedJourneys[leastSegmentsIndex].segments.size() ||
                        (journey.segments.size() == rankedJourneys[leastSegmentsIndex].segments.size() &&
                         journey.totalTimeSeconds < rankedJourneys[leastSegmentsIndex].totalTimeSeconds))
                    {
                        leastSegmentsIndex = i;
                    }

                    if (leastWalkIndex < 0 ||
                        calculateTotalWalkDistanceMeters(journey) <
                            calculateTotalWalkDistanceMeters(rankedJourneys[leastWalkIndex]) ||
                        (isAlmostEqual(calculateTotalWalkDistanceMeters(journey),
                                       calculateTotalWalkDistanceMeters(rankedJourneys[leastWalkIndex])) &&
                         journey.totalTimeSeconds < rankedJourneys[leastWalkIndex].totalTimeSeconds))
                    {
                        leastWalkIndex = i;
                    }

                    if (comfortIndex < 0 ||
                        calculateComfortScoreSeconds(journey) <
                            calculateComfortScoreSeconds(rankedJourneys[comfortIndex]))
                    {
                        comfortIndex = i;
                    }
                }

                trySelectJourney(leastTransfersIndex, false);
                trySelectJourney(leastSegmentsIndex, false);
                trySelectJourney(leastWalkIndex, false);
                trySelectJourney(comfortIndex, false);

                for (int32_t pass = 0; pass < 2 && static_cast<int32_t>(selectedIndices.size()) < maxJourneys; ++pass)
                {
                    const auto allowUsedRouteChain = pass > 0;
                    for (int32_t i = 0; i < availableJourneys; ++i)
                    {
                        if (static_cast<int32_t>(selectedIndices.size()) >= maxJourneys)
                            break;
                        trySelectJourney(i, allowUsedRouteChain);
                    }
                }

                std::sort(selectedIndices.begin(), selectedIndices.end());
                result.journeys.reserve(selectedIndices.size());
                for (const auto index : selectedIndices)
                    result.journeys.push_back(std::move(rankedJourneys[index]));

                const auto& bestJourney = result.journeys.front();
                result.found = bestJourney.found;
                result.totalTimeSeconds = bestJourney.totalTimeSeconds;
                result.finishWalkDistanceMeters = bestJourney.finishWalkDistanceMeters;
                result.finalStopIndex = bestJourney.finalStopIndex;
                result.finalRound = bestJourney.finalRound;
                result.finalRouteTypeIndex = bestJourney.finalRouteTypeIndex;
                result.segments = bestJourney.segments;
                return result;
            }
        } // namespace

        bool PackedTransportSnapshot::validate(std::string* errorMessage) const
        {
            if (stopCount < 0 || routeCount < 0 || routeTypeCount <= 0)
            {
                if (errorMessage)
                    *errorMessage = "snapshot counts must be non-negative and routeTypeCount must be positive";
                return false;
            }

            if (stopIds.size() != static_cast<size_t>(stopCount) ||
                routeIds.size() != static_cast<size_t>(routeCount) ||
                stopX31.size() != static_cast<size_t>(stopCount) ||
                stopY31.size() != static_cast<size_t>(stopCount) ||
                startWalkSeconds.size() != static_cast<size_t>(stopCount) ||
                startWalkDistancesMeters.size() != static_cast<size_t>(stopCount) ||
                endWalkSeconds.size() != static_cast<size_t>(stopCount) ||
                endWalkDistancesMeters.size() != static_cast<size_t>(stopCount))
            {
                if (errorMessage)
                    *errorMessage = "snapshot metadata vectors must match stop/route counts";
                return false;
            }

            const auto totalStopRouteRefs = transitData.stopRouteRouteIndices.size();
            if (!validateOffsets(transitData.stopRouteOffsets,
                                 static_cast<size_t>(stopCount),
                                 totalStopRouteRefs,
                                 "stopRouteOffsets",
                                 errorMessage))
            {
                return false;
            }
            if (transitData.stopRouteStopPositions.size() != totalStopRouteRefs)
            {
                if (errorMessage)
                    *errorMessage = "stopRouteStopPositions must match stopRouteRouteIndices";
                return false;
            }

            const auto totalRouteStops = transitData.routeStopIndices.size();
            if (!validateOffsets(transitData.routeStopOffsets,
                                 static_cast<size_t>(routeCount),
                                 totalRouteStops,
                                 "routeStopOffsets",
                                 errorMessage))
            {
                return false;
            }

            if (!validateOffsets(transitData.footpathOffsets,
                                 static_cast<size_t>(stopCount),
                                 transitData.footpathTargetIndices.size(),
                                 "footpathOffsets",
                                 errorMessage))
            {
                return false;
            }
            if (transitData.footpathTimesSeconds.size() != transitData.footpathTargetIndices.size() ||
                transitData.footpathDistancesMeters.size() != transitData.footpathTargetIndices.size())
            {
                if (errorMessage)
                    *errorMessage = "footpath payload arrays must match footpathTargetIndices";
                return false;
            }

            if (transitData.routeTypeIndices.size() != static_cast<size_t>(routeCount))
            {
                if (errorMessage)
                    *errorMessage = "routeTypeIndices must match routeCount";
                return false;
            }
            if (!transitData.routeDefaultHeadwaySeconds.empty() &&
                transitData.routeDefaultHeadwaySeconds.size() != static_cast<size_t>(routeCount))
            {
                if (errorMessage)
                    *errorMessage = "routeDefaultHeadwaySeconds must match routeCount";
                return false;
            }
            if (!transitData.boardingCostSeconds.empty() &&
                transitData.boardingCostSeconds.size() != totalRouteStops)
            {
                if (errorMessage)
                    *errorMessage = "boardingCostSeconds must match total route stops";
                return false;
            }

            if (transitData.cumulativeCompatTravelSeconds.size() != totalRouteStops ||
                transitData.cumulativeTravelDistancesMeters.size() != totalRouteStops)
            {
                if (errorMessage)
                    *errorMessage = "route cumulative metrics must match total route stops";
                return false;
            }
            if (!transitData.cumulativeScheduleOffsetsDeciSeconds.empty() &&
                transitData.cumulativeScheduleOffsetsDeciSeconds.size() != totalRouteStops)
            {
                if (errorMessage)
                    *errorMessage = "schedule offsets must match total route stops";
                return false;
            }
            if (!transitData.expectedWaitSeconds.empty() &&
                transitData.expectedWaitSeconds.size() != totalRouteStops)
            {
                if (errorMessage)
                    *errorMessage = "expectedWaitSeconds must match total route stops";
                return false;
            }

            if (!transitData.scheduleTripOffsets.empty() &&
                !validateOffsets(transitData.scheduleTripOffsets,
                                 static_cast<size_t>(routeCount),
                                 transitData.scheduleTripStartsDeciSeconds.size(),
                                 "scheduleTripOffsets",
                                 errorMessage))
            {
                return false;
            }

            for (const auto routeType : transitData.routeTypeIndices)
            {
                if (routeType < 0 || routeType >= routeTypeCount)
                {
                    if (errorMessage)
                        *errorMessage = "routeTypeIndices contains an out-of-range type index";
                    return false;
                }
            }

            for (const auto routeStopIndex : transitData.routeStopIndices)
            {
                if (routeStopIndex < 0 || routeStopIndex >= stopCount)
                {
                    if (errorMessage)
                        *errorMessage = "routeStopIndices contains an out-of-range stop index";
                    return false;
                }
            }

            return true;
        }

        bool RaptorQueryOptions::validate(const int32_t routeTypeCount, std::string* errorMessage) const
        {
            if (maxRounds <= 0)
            {
                if (errorMessage)
                    *errorMessage = "maxRounds must be positive";
                return false;
            }
            if (maxJourneys <= 0)
            {
                if (errorMessage)
                    *errorMessage = "maxJourneys must be positive";
                return false;
            }
            if (walkRouteTypeIndex < 0 || walkRouteTypeIndex >= routeTypeCount)
            {
                if (errorMessage)
                    *errorMessage = "walkRouteTypeIndex must be inside route type range";
                return false;
            }
            if (!boardingSecondsByRouteType.empty() &&
                boardingSecondsByRouteType.size() != static_cast<size_t>(routeTypeCount))
            {
                if (errorMessage)
                    *errorMessage = "boardingSecondsByRouteType must match routeTypeCount";
                return false;
            }
            if (!changeSecondsMatrix.empty() &&
                changeSecondsMatrix.size() != static_cast<size_t>(routeTypeCount * routeTypeCount))
            {
                if (errorMessage)
                    *errorMessage = "changeSecondsMatrix must be routeTypeCount * routeTypeCount";
                return false;
            }
            return true;
        }

        RaptorQueryResult Engine::run(const PackedTransportSnapshot& snapshot,
                                      const RaptorQueryOptions& options,
                                      const bool collectTrace)
        {
            std::string errorMessage;
            if (!snapshot.validate(&errorMessage))
                throw std::invalid_argument(errorMessage);
            if (!options.validate(snapshot.routeTypeCount, &errorMessage))
                throw std::invalid_argument(errorMessage);

            QueryState state(snapshot, options, collectTrace);

            std::vector<int32_t> markedStops;
            std::vector<int32_t> startLabels;
            std::vector<char> startStopSeen(snapshot.stopCount, 0);
            for (int32_t stop = 0; stop < snapshot.stopCount; ++stop)
            {
                const auto startSeconds = snapshot.startWalkSeconds[stop];
                if (!isFinite(startSeconds))
                    continue;

                const auto label = state.improveLabel(0,
                                                      options.walkRouteTypeIndex,
                                                      stop,
                                                      startSeconds,
                                                      RaptorPredecessorKind::Start,
                                                      -1,
                                                      -1,
                                                      -1,
                                                      -1,
                                                      -1,
                                                      snapshot.startWalkDistancesMeters[stop],
                                                      startSeconds,
                                                      collectTrace);
                if (label >= 0)
                {
                    startLabels.push_back(label);
                    pushImprovedStop(stop, startStopSeen, markedStops);
                }
            }

            relaxFootpathsForRound(state, 0, startLabels, markedStops, collectTrace);

            for (int32_t round = 1; round <= options.maxRounds && !markedStops.empty(); ++round)
            {
                std::vector<int32_t> markedRoutes;
                for (const auto stop : markedStops)
                {
                    for (int32_t ref = snapshot.transitData.stopRouteOffsets[stop];
                         ref < snapshot.transitData.stopRouteOffsets[stop + 1];
                         ++ref)
                    {
                        const auto routeIndex = snapshot.transitData.stopRouteRouteIndices[ref];
                        if (state.routeMarkedAtRound[routeIndex] == round)
                            continue;
                        state.routeMarkedAtRound[routeIndex] = round;
                        markedRoutes.push_back(routeIndex);
                    }
                }

                std::vector<int32_t> rideSeedLabels;
                std::vector<int32_t> nextMarkedStops;
                std::vector<char> nextStopSeen(snapshot.stopCount, 0);

                for (const auto routeIndex : markedRoutes)
                {
                    const auto routeType = snapshot.transitData.routeTypeIndices[routeIndex];
                    const auto routeBase = snapshot.transitData.routeStopOffsets[routeIndex];
                    const auto routeSize =
                        snapshot.transitData.routeStopOffsets[routeIndex + 1] - routeBase;
                    const bool useTimetableForRoute =
                        options.useTimetable && state.routeHasSchedule(routeIndex);
                    std::vector<ActiveRideState> activeRides;
                    std::vector<int32_t> departures;

                    for (int32_t stopPos = 0; stopPos < routeSize; ++stopPos)
                    {
                        const auto stopIndex = snapshot.transitData.routeStopIndices[routeBase + stopPos];
                        std::vector<ActiveRideState> boardingCandidates;

                        for (int32_t previousType = 0; previousType < snapshot.routeTypeCount; ++previousType)
                        {
                            const auto& previousBag = state.bag(round - 1, previousType, stopIndex);
                            for (const auto previousLabel : previousBag)
                            {
                                const auto previousArrival = state.arrivals[previousLabel];
                                if (!isFinite(previousArrival))
                                    continue;

                                const auto transferSeconds =
                                    previousType == options.walkRouteTypeIndex
                                        ? 0.0
                                        : state.changeSeconds(previousType, routeType);
                                const auto boardBase = previousArrival + transferSeconds;
                                const auto boardingSeconds = state.boardingSeconds(routeType);

                                if (useTimetableForRoute)
                                {
                                    state.collectTripDepartures(routeIndex,
                                                                stopPos,
                                                                boardBase + boardingSeconds,
                                                                departures);
                                    for (const auto departure : departures)
                                    {
                                        ActiveRideState ride;
                                        ride.previousLabel = previousLabel;
                                        ride.boardStopPos = stopPos;
                                        ride.currentStopPos = stopPos;
                                        ride.departureTime = departure;
                                        ride.currentArrivalSeconds =
                                            static_cast<double>(departure) * kTickToSeconds;
                                        ride.currentTravelSeconds = ride.currentArrivalSeconds - boardBase;
                                        boardingCandidates.push_back(ride);
                                    }
                                }
                                else
                                {
                                    ActiveRideState ride;
                                    ride.previousLabel = previousLabel;
                                    ride.boardStopPos = stopPos;
                                    ride.currentStopPos = stopPos;
                                    ride.departureTime = -1;
                                    ride.currentTravelSeconds =
                                        state.schedulelessBoardingSeconds(routeIndex, stopPos, routeType);
                                    ride.currentArrivalSeconds = boardBase + ride.currentTravelSeconds;
                                    boardingCandidates.push_back(ride);
                                }
                            }
                        }

                        for (auto& ride : activeRides)
                        {
                            if (ride.currentStopPos < stopPos)
                            {
                                const auto deltaSeconds =
                                    useTimetableForRoute
                                        ? state.scheduleTravelSeconds(routeIndex, ride.currentStopPos, stopPos)
                                        : state.compatTravelSeconds(routeIndex, ride.currentStopPos, stopPos);
                                ride.currentArrivalSeconds += deltaSeconds;
                                ride.currentTravelSeconds += deltaSeconds;
                                ride.currentStopPos = stopPos;
                            }

                            if (stopPos > ride.boardStopPos)
                            {
                                const auto label = state.improveLabel(round,
                                                                      routeType,
                                                                      stopIndex,
                                                                      ride.currentArrivalSeconds,
                                                                      RaptorPredecessorKind::Ride,
                                                                      ride.previousLabel,
                                                                      routeIndex,
                                                                      ride.boardStopPos,
                                                                      stopPos,
                                                                      ride.departureTime,
                                                                      0.0,
                                                                      ride.currentTravelSeconds,
                                                                      collectTrace);
                                if (label >= 0)
                                {
                                    rideSeedLabels.push_back(label);
                                    pushImprovedStop(stopIndex, nextStopSeen, nextMarkedStops);
                                    tryFinish(state, label);
                                }
                            }
                        }

                        activeRides.insert(activeRides.end(), boardingCandidates.begin(), boardingCandidates.end());
                        pruneActiveRides(state, activeRides);
                    }
                }

                relaxFootpathsForRound(state, round, rideSeedLabels, nextMarkedStops, collectTrace);
                markedStops.swap(nextMarkedStops);
            }

            return buildResult(state);
        }
    } // namespace RaptorEngine
} // namespace OsmAnd
