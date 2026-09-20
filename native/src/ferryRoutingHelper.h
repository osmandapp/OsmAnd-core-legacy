#ifndef _OSMAND_FERRY_ROUTING_HELPER_H
#define _OSMAND_FERRY_ROUTING_HELPER_H

#include "CommonCollections.h"
#include "binaryRead.h"
#include "commonOsmAndCore.h"
#include "generalRouter.h"
#include "routeSegment.h"
#include "routeSegmentResult.h"
#include "routingContext.h"
#include "transportRoutingObjects.h"

// Ferry crossing time, same for all routing profiles, route search and estimated time (FerryRoutingHelper.java):
// waiting for a ferry (half of its interval, otherwise ferryBoardingTime), getting on it (ferryTerminalTime),
// sailing (a part of the duration tag, otherwise with the ferry speed and ferryTerminalTime at each terminal
// on the way) and getting off it (half of ferryTerminalTime). Getting on and off is paid only for a whole crossing:
// a route starting or ending on a ferry (at its terminal or on board) only sails.
struct FerryRoutingHelper {
	static constexpr const char* FERRY = "ferry";
	static constexpr const char* DURATION_TAG = "duration";
	// routing.xml attributes ferryBoardingTime and ferryTerminalTime (seconds) are read by RoutingConfiguration

	static bool isFerry(const SHARED_PTR<RouteDataObject>& road) {
		return road->containsType(road->region->ferry);
	}

	// waiting for a ferry (half of its interval if known) and getting on it while it stands at the terminal
	static double getBoardingTime(int ferryBoardingTime, int ferryTerminalTime, int interval) {
		return (interval > 0 ? interval / 2.0 : ferryBoardingTime) + ferryTerminalTime;
	}

	// getting off doesn't wait until the ferry finishes its stop
	static double getAlightingTime(int ferryTerminalTime) {
		return ferryTerminalTime / 2.0;
	}

	// time from a duration tag, 0 if the tag is absent or unrealistic for the distance (meters)
	static int parseDuration(const string& duration, double distance) {
		int seconds = TransportRoute::parseIntervalTagToSeconds(duration);
		double speed = seconds > 0 ? distance / seconds * 3.6 : 0;
		return speed >= MIN_DURATION_SPEED && speed <= MAX_DURATION_SPEED ? seconds : 0;
	}

	// route search: a ferry with a duration tag moves with the speed from it
	static float getRoutingSpeed(const SHARED_PTR<GeneralRouter>& router, const SHARED_PTR<RouteDataObject>& road,
								 float speed) {
		double durationSpeed = speed > 0 && isPassenger(router) && isFerry(road) ? getDurationSpeed(road) : 0;
		return durationSpeed > 0 ? (float)durationSpeed : speed;
	}

	// route search: getting on or off a ferry at a turn from one road to another
	static double getTransitionTime(RoutingContext* ctx, const SHARED_PTR<RouteSegment>& from,
									const SHARED_PTR<RouteSegment>& to) {
		bool toFerry = isFerry(to->getRoad());
		if (isFerry(from->getRoad()) == toFerry || !isPassenger(ctx->config->router)) {
			return 0;
		}
		const auto& config = ctx->config;
		return toFerry ? getBoardingTime(config->ferryBoardingTime, config->ferryTerminalTime, getInterval(to->getRoad()))
					   : getAlightingTime(config->ferryTerminalTime);
	}

	// route search: the ferry stops at the point between the segment and its parent (both directions of search)
	static double getStopTime(RoutingContext* ctx, const SHARED_PTR<RouteSegment>& segment) {
		SHARED_PTR<RouteSegment> parent = segment->getParentRoute();
		if (!parent || !isFerry(segment->getRoad()) || !isFerry(parent->getRoad()) || !isPassenger(ctx->config->router)) {
			return 0;
		}
		return getStopTime(ctx->config->ferryTerminalTime, segment->getRoad(), segment->getSegmentStart(),
						   parent->getRoad(), parent->getSegmentEnd());
	}

	// estimated time of the ferry segments, calculated again for the whole segment
	static void updateSegmentTimes(RoutingContext* ctx, vector<SHARED_PTR<RouteSegmentResult>>& result) {
		const auto& router = ctx->config->router;
		if (!isPassenger(router)) {
			return;
		}
		const auto& config = ctx->config;
		for (int i = 0; i < (int)result.size(); i++) {
			auto& rr = result[i];
			const auto& road = rr->object;
			if (!isFerry(road)) {
				continue;
			}
			double speed = getDurationSpeed(road);
			if (speed <= 0) {
				speed = router->defineVehicleSpeed(road, rr->isForwardDirection());
			}
			if (speed <= 0) {
				speed = router->getDefaultSpeed();
			}
			double time = rr->distance / speed;
			int start = std::min(rr->getStartPointIndex(), rr->getEndPointIndex());
			int end = std::max(rr->getStartPointIndex(), rr->getEndPointIndex());
			for (int point = start + 1; point < end; point++) {
				time += getStopTime(config->ferryTerminalTime, road, point, road, point);
			}
			bool crossing = isCrossing(result, i);
			if (crossing && !isFerry(result[i - 1]->object)) {
				time += getBoardingTime(config->ferryBoardingTime, config->ferryTerminalTime, getInterval(road));
			}
			SHARED_PTR<RouteSegmentResult> next = i + 1 < (int)result.size() ? result[i + 1] : nullptr;
			if (next && isFerry(next->object)) {
				time += getStopTime(config->ferryTerminalTime, road, rr->getEndPointIndex(), next->object,
									next->getStartPointIndex());
			} else if (crossing) {
				time += getAlightingTime(config->ferryTerminalTime);
			}
			if (time > 0) {
				rr->segmentTime = (float)time;
				rr->segmentSpeed = (float)(rr->distance / time);  // navigation calculates time left with the speed
			}
		}
	}

   private:
	// speeds (km/h) that make a duration tag believable for its distance
	static constexpr double MIN_DURATION_SPEED = 1;
	static constexpr double MAX_DURATION_SPEED = 100;

	// a boat sails along a ferry line by itself
	static bool isPassenger(const SHARED_PTR<GeneralRouter>& router) {
		return router->getProfile() != GeneralRouterProfile::BOAT;
	}

	// the ferry segments around this one have roads before and after them
	static bool isCrossing(vector<SHARED_PTR<RouteSegmentResult>>& result, int ferrySegment) {
		int first = ferrySegment;
		int last = ferrySegment;
		while (first > 0 && isFerry(result[first - 1]->object)) {
			first--;
		}
		while (last < (int)result.size() - 1 && isFerry(result[last + 1]->object)) {
			last++;
		}
		return first > 0 && last < (int)result.size() - 1;
	}

	static int getInterval(const SHARED_PTR<RouteDataObject>& road) {
		return TransportRoute::parseIntervalTagToSeconds(road->getValue(TransportRoute::INTERVAL_KEY));
	}

	// the same point of two ferry ways (or of one way): the ferry stops at a terminal
	// unless a duration of a way includes this stop
	static double getStopTime(int ferryTerminalTime, const SHARED_PTR<RouteDataObject>& road, int point,
							  const SHARED_PTR<RouteDataObject>& other, int otherPoint) {
		bool terminal = road->getValue(point, "amenity") == "ferry_terminal";
		return terminal && !isInDuration(road, point) && !isInDuration(other, otherPoint) ? ferryTerminalTime : 0;
	}

	static bool isInDuration(const SHARED_PTR<RouteDataObject>& road, int point) {
		return point > 0 && point < road->getPointsLength() - 1 && getDurationSpeed(road) > 0;
	}

	// meters per second, 0 without a duration tag
	static double getDurationSpeed(const SHARED_PTR<RouteDataObject>& road) {
		string duration = road->getValue(DURATION_TAG);
		if (duration.empty()) {
			return 0;
		}
		double length = 0;
		for (int i = 1; i < road->getPointsLength(); i++) {
			length += measuredDist31(road->getPoint31XTile(i - 1), road->getPoint31YTile(i - 1),
									 road->getPoint31XTile(i), road->getPoint31YTile(i));
		}
		int seconds = parseDuration(duration, length);
		return seconds > 0 ? length / seconds : 0;
	}
};

#endif	// _OSMAND_FERRY_ROUTING_HELPER_H
