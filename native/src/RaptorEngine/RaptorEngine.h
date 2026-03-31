#ifndef _OSMAND_CORE_RAPTOR_ENGINE_H_
#define _OSMAND_CORE_RAPTOR_ENGINE_H_

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace OsmAnd
{
    namespace RaptorEngine
    {
        enum class SchedulelessMode : uint8_t
        {
            Compat = 0,
            Headway = 1
        };

        enum class RaptorPredecessorKind : uint8_t
        {
            None = 0,
            Start = 1,
            Walk = 2,
            Ride = 3
        };

        struct PackedTransitData
        {
            std::vector<int32_t> stopRouteOffsets;
            std::vector<int32_t> stopRouteRouteIndices;
            std::vector<int32_t> stopRouteStopPositions;

            std::vector<int32_t> routeStopOffsets;
            std::vector<int32_t> routeStopIndices;

            std::vector<int32_t> footpathOffsets;
            std::vector<int32_t> footpathTargetIndices;
            std::vector<double> footpathTimesSeconds;
            std::vector<double> footpathDistancesMeters;

            std::vector<int32_t> routeTypeIndices;
            std::vector<double> routeDefaultHeadwaySeconds;
            std::vector<double> boardingCostSeconds;

            std::vector<double> cumulativeCompatTravelSeconds;
            std::vector<double> cumulativeTravelDistancesMeters;
            std::vector<int32_t> cumulativeScheduleOffsetsDeciSeconds;
            std::vector<double> expectedWaitSeconds;

            std::vector<int32_t> scheduleTripOffsets;
            std::vector<int32_t> scheduleTripStartsDeciSeconds;
        };

        struct PackedTransportSnapshot
        {
            PackedTransitData transitData;

            int32_t stopCount = 0;
            int32_t routeCount = 0;
            int32_t routeTypeCount = 0;

            std::vector<int64_t> stopIds;
            std::vector<int64_t> routeIds;
            std::vector<int32_t> stopX31;
            std::vector<int32_t> stopY31;

            std::vector<double> startWalkSeconds;
            std::vector<double> startWalkDistancesMeters;
            std::vector<double> endWalkSeconds;
            std::vector<double> endWalkDistancesMeters;

            bool validate(std::string* errorMessage = nullptr) const;
        };

        struct RaptorQueryOptions
        {
            int32_t maxRounds = 3;
            int32_t maxJourneys = 1;
            bool useTimetable = false;
            SchedulelessMode schedulelessMode = SchedulelessMode::Compat;
            double maxRouteTimeSeconds = std::numeric_limits<double>::infinity();
            int32_t walkRouteTypeIndex = 0;

            std::vector<double> boardingSecondsByRouteType;
            std::vector<double> changeSecondsMatrix;

            bool validate(int32_t routeTypeCount, std::string* errorMessage = nullptr) const;
        };

        struct RaptorRouteSegment
        {
            int32_t routeIndex = -1;
            int32_t boardStopPosition = -1;
            int32_t alightStopPosition = -1;
            int32_t departureTimeDeciSeconds = -1;
            int32_t round = -1;
            double walkDistanceMeters = 0.0;
            double travelTimeSeconds = 0.0;
            double travelDistanceMeters = 0.0;
        };

        struct RaptorJourney
        {
            bool found = false;
            double totalTimeSeconds = std::numeric_limits<double>::infinity();
            double finishWalkDistanceMeters = 0.0;
            int32_t finalStopIndex = -1;
            int32_t finalRound = -1;
            int32_t finalRouteTypeIndex = -1;

            std::vector<RaptorRouteSegment> segments;
        };

        struct RaptorTraceEntry
        {
            int32_t round = -1;
            int32_t stopIndex = -1;
            int32_t lastRouteTypeIndex = -1;
            double arrivalSeconds = std::numeric_limits<double>::infinity();
            RaptorPredecessorKind predecessorKind = RaptorPredecessorKind::None;
            int32_t previousLabelIndex = -1;
            int32_t routeIndex = -1;
            int32_t boardStopPosition = -1;
            int32_t alightStopPosition = -1;
            int32_t departureTimeDeciSeconds = -1;
            double walkDistanceMeters = 0.0;
            double travelTimeSeconds = 0.0;
        };

        struct RaptorQueryResult
        {
            bool found = false;
            double totalTimeSeconds = std::numeric_limits<double>::infinity();
            double finishWalkDistanceMeters = 0.0;
            int32_t finalStopIndex = -1;
            int32_t finalRound = -1;
            int32_t finalRouteTypeIndex = -1;

            std::vector<RaptorRouteSegment> segments;
            std::vector<RaptorJourney> journeys;
            std::vector<RaptorTraceEntry> trace;
        };

        class Engine
        {
        public:
            static RaptorQueryResult run(const PackedTransportSnapshot& snapshot,
                                         const RaptorQueryOptions& options,
                                         bool collectTrace = false);
        };
    }
}

#endif // !defined(_OSMAND_CORE_RAPTOR_ENGINE_H_)
