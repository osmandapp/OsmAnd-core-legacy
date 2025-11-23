#ifndef _OSMAND_TRANSPORT_ROUTE_CONFIGUERATION_CPP
#define _OSMAND_TRANSPORT_ROUTE_CONFIGUERATION_CPP
#include "transportRoutingConfiguration.h"

#include <stdlib.h>

#include "generalRouter.h"

TransportRoutingConfiguration::TransportRoutingConfiguration()
	: router(new GeneralRouter()) {}

TransportRoutingConfiguration::TransportRoutingConfiguration(
	SHARED_PTR<GeneralRouter> prouter, MAP_STR_STR params) {
	if (prouter != nullptr) {
		this->router = prouter->build(params);
		walkRadius = router->getIntAttribute("walkRadius", walkRadius);
		walkChangeRadius = 
			router->getIntAttribute("walkChangeRadius", walkChangeRadius);
		zoomToLoadTiles =
			router->getIntAttribute("zoomToLoadTiles", zoomToLoadTiles);
		maxNumberOfChanges =
			router->getIntAttribute("maxNumberOfChanges", maxNumberOfChanges);
		maxRouteTime = router->getIntAttribute("maxRouteTime", maxRouteTime);
		finishTimeSeconds = router->getIntAttribute( // TODO remove
			"delayForAlternativesRoutes", finishTimeSeconds);

		increaseForAlternativesRoutes = router->
			getFloatAttribute("increaseForAlternativesRoutes", static_cast<float>(increaseForAlternativesRoutes));
		increaseForAltRoutesWalking = router->
			getFloatAttribute("increaseForAltRoutesWalking", static_cast<float>(increaseForAltRoutesWalking));

		combineAltRoutesDiffStops = router->
			getIntAttribute("combineAltRoutesDiffStops", combineAltRoutesDiffStops);
		combineAltRoutesSumDiffStops = router->
			getIntAttribute("combineAltRoutesSumDiffStops", combineAltRoutesSumDiffStops);

		string mn = router->getAttribute("max_num_changes");
		try {
			maxNumberOfChanges = std::stoi(mn);
		} catch (...) {
			// Ignore
		}

		walkSpeed =
			router->getFloatAttribute("minDefaultSpeed", walkSpeed * 3.6f) /
			3.6f;
		defaultTravelSpeed = router->getFloatAttribute(
								 "maxDefaultSpeed", defaultTravelSpeed * 3.6f) /
							 3.6f;
		maxRouteIncreaseSpeed = router->getIntAttribute("maxRouteIncreaseSpeed", maxRouteIncreaseSpeed);
		maxRouteDistance =  router->getIntAttribute("maxRouteDistance", maxRouteDistance);

		// TODO remove lines 45-52
		RouteAttributeContext &obstacles =
			router->getObjContext(RouteDataObjectAttribute::ROUTING_OBSTACLES);
		dynbitset bs = getRawBitset("time", "stop");
		stopTime = obstacles.evaluateInt(bs, stopTime);
		bs = getRawBitset("time", "change");
		changeTime = obstacles.evaluateInt(bs, changeTime);
		bs = getRawBitset("time", "boarding");
		boardingTime = obstacles.evaluateInt(bs, boardingTime);

		RouteAttributeContext &spds =
			router->getObjContext(RouteDataObjectAttribute::ROAD_SPEED);
		bs = getRawBitset("route", "walk");
		walkSpeed = spds.evaluateFloat(bs, walkSpeed);
	}
}

float TransportRoutingConfiguration::getSpeedByRouteType(
	std::string routeType) {
	const auto it = speed.find(routeType);
	float sl = defaultTravelSpeed;
	if (it == speed.end()) {
		dynbitset bs = getRawBitset("route", routeType);
		sl = router->getObjContext(RouteDataObjectAttribute::ROAD_SPEED)
				 .evaluateFloat(bs, defaultTravelSpeed);
		speed[routeType] = sl;
	} else {
		sl = it->second;
	}
	return sl;
}

dynbitset TransportRoutingConfiguration::getRawBitset(std::string tg,
													  std::string vl) {
	uint id = getRawType(tg, vl);
	dynbitset bs(router->getBitSetSize());
	bs.set(id);
	return bs;
}

uint TransportRoutingConfiguration::getRawType(string &tg, string &vl) {
	string key = tg + "$" + vl;
	if (rawTypes.find(key) == rawTypes.end()) {
		uint at = router->registerTagValueAttribute(tag_value(tg, vl));
		rawTypes[key] = at;
	}
	return rawTypes[key];
}

int32_t TransportRoutingConfiguration::getChangeTime() { // TODO remove
	return useSchedule ? 0 : changeTime;
};

int32_t TransportRoutingConfiguration::getBoardingTime() { // TODO remove
	return boardingTime;
};

int32_t TransportRoutingConfiguration::getStopTime(const std::string &routeType) {
	if (stopTimes.count(routeType)) {
		const int32_t time = stopTimes.at(routeType);
		if (time > 0) return time;
	}

	RouteAttributeContext &obstacles =
		router->getObjContext(RouteDataObjectAttribute::ROUTING_OBSTACLES);

	dynbitset bs = getRawBitset("stop", routeType);
	const int32_t time = obstacles.evaluateInt(bs, 0);
	stopTimes[routeType] = time;

	if (time > 0) return time;

	if (defaultStopTime == 0) {
		dynbitset bs2 = getRawBitset("stop", "");
		defaultStopTime = obstacles.evaluateInt(bs2, 30);
	}
	return defaultStopTime;
}

int32_t TransportRoutingConfiguration::getBoardingTime(const std::string &routeType) {
	if (boardingTimes.count(routeType)) {
		const int32_t time = boardingTimes.at(routeType);
		if (time > 0) return time;
	}

	RouteAttributeContext &obstacles =
		router->getObjContext(RouteDataObjectAttribute::ROUTING_OBSTACLES);

	dynbitset bs = getRawBitset("boarding", routeType);
	const int32_t time = obstacles.evaluateInt(bs, 0);
	boardingTimes[routeType] = time;

	if (time > 0) return time;

	if (defaultBoardingTime == 0) {
		dynbitset bs2 = getRawBitset("boarding", "");
		defaultBoardingTime = obstacles.evaluateInt(bs2, 150);
	}
	return defaultBoardingTime;
}

int32_t TransportRoutingConfiguration::getChangeTime(const std::string &fromRouteType, const std::string &toRouteType) {
	const std::string key = fromRouteType + "_" + toRouteType;

	if (changingTimes.count(key)) {
		const int32_t time = changingTimes.at(key);
		if (time > 0) return time;
	}

	RouteAttributeContext &obstacles =
		router->getObjContext(RouteDataObjectAttribute::ROUTING_OBSTACLES);

	dynbitset bs = getRawBitset("change", key);
	const int32_t time = obstacles.evaluateInt(bs, 0);
	changingTimes[key] = time;

	if (time > 0) return time;

	if (defaultChangeTime == 0) {
		dynbitset bs2 = getRawBitset("change", "");
		defaultChangeTime = obstacles.evaluateInt(bs2, 240);
	}
	return defaultChangeTime;
}

#endif
