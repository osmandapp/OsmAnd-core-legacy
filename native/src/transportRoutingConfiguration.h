#ifndef _OSMAND_TRANSPORT_ROUTING_CONFIGURATION_H
#define _OSMAND_TRANSPORT_ROUTING_CONFIGURATION_H
#include "CommonCollections.h"
#include "boost/dynamic_bitset.hpp"
#include "commonOsmAndCore.h"
// #include <algorithm>

class GeneralRouter;
typedef boost::dynamic_bitset<> dynbitset;

struct TransportRoutingConfiguration {
	const string KEY = "public_transport";
	int32_t zoomToLoadTiles = 15;
	int32_t walkRadius = 1500;
	int32_t walkChangeRadius = 300;
	int32_t maxNumberOfChanges = 3;
	int32_t finishTimeSeconds = 1200; // TODO comment

	double increaseForAlternativesRoutes = 2.0;
	double increaseForAltRoutesWalking = 3.0;

	int combineAltRoutesDiffStops = 120;
	int combineAltRoutesSumDiffStops = 300;

	int32_t maxRouteTime = 60 * 60 * 10;
	int32_t maxRouteDistance = 0;
	int32_t maxRouteIncreaseSpeed = 30;
	SHARED_PTR<GeneralRouter> router;

	float walkSpeed = (float)(3.6 / 3.6);
	float defaultTravelSpeed = (60 / 3.6);
	// int32_t stopTime = 30; // TODO remove
	// int32_t changeTime = 180; // TODO remove
	// int32_t boardingTime = 180; // TODO remove

	int32_t defaultStopTime = 0;
	UNORDERED(map)<string, int32_t> stopTimes;

	int32_t defaultBoardingTime = 0;
	UNORDERED(map)<string, int32_t> boardingTimes;

	int32_t defaultChangeTime = 0;
	UNORDERED(map)<string, int32_t> changingTimes;

	bool useSchedule = false;

	int32_t scheduleTimeOfDay = 12 * 60 * 6;
	int32_t scheduleMaxTime = 50 * 6;
	//    int32_t scheduleDayNumber; Unused
	MAP_STR_FLOAT speed;
	UNORDERED(map)<string, float> rawTypes;

	TransportRoutingConfiguration();

	TransportRoutingConfiguration(SHARED_PTR<GeneralRouter> prouter,
								  MAP_STR_STR params);

	float getSpeedByRouteType(std::string routeType);
	dynbitset getRawBitset(std::string tg, std::string vl);
	uint getRawType(string &tg, string &vl);
	// int32_t getChangeTime(); // TODO remove
	// int32_t getBoardingTime(); // TODO remove

	int32_t getStopTime(const std::string &routeType);
	int32_t getBoardingTime(const std::string &routeType);
	int32_t getChangeTime(const std::string &fromRouteType, const std::string &toRouteType);
};

#endif	//_OSMAND_TRANSPORT_ROUTING_CONFIGURATION_H
