#ifndef _OSMAND_TRANSPORT_FERRY_HELPER_H
#define _OSMAND_TRANSPORT_FERRY_HELPER_H

#include "CommonCollections.h"
#include "commonOsmAndCore.h"
#include "ferryRoutingHelper.h"
#include "transportRoutingConfiguration.h"
#include "transportRoutingObjects.h"

// Public transport ferries (TransportFerryHelper.java): routes built by the map creator from route=ferry ways
// without a route relation and ferry crossings of other routes. Stop flags are stored as route tags with indexes
// of the route stops. The name tag a generated stop carries itself is read by the map layers only -
// TransportStopsLayer on Android, TransportStop::isSynthetic of OsmAndCore on iOS.
// transportRouteResultSegment.h can't be included here: its MISSING_STOP_SEARCH_RADIUS clashes with
// the one of transportRouteStopsReader.h, which is included together with this header
struct TransportRouteResultSegment;

struct TransportFerryHelper {
	// stops generated at ferry way ends (not present in OSM), "j" marks a junction of ferry ways
	// in the water (only a change to the next ferry way at the same stop): "0,3:j,5"
	static constexpr const char* FERRY_STOPS_TAG = "osmand_ferry_stops";
	static constexpr const char* JUNCTION_VALUE = "j";
	// non-ferry route goes over a ferry before these stops:
	// "stop index:ferry interval:ferry duration:ferry length" (seconds and meters, 0 - unknown)
	static constexpr const char* CROSSINGS_TAG = "osmand_ferry_crossings";

	static bool isFerry(const SHARED_PTR<TransportRoute>& route) {
		return route->type == FerryRoutingHelper::FERRY;
	}

	// the route lists the indexes of its generated stops
	static bool isSyntheticStop(const SHARED_PTR<TransportRoute>& route, int stop) {
		string value;
		return getStopValue(route, FERRY_STOPS_TAG, stop, value);
	}

	static bool isJunctionStop(const SHARED_PTR<TransportRoute>& route, int stop) {
		string value;
		return getStopValue(route, FERRY_STOPS_TAG, stop, value) && value == JUNCTION_VALUE;
	}

	// ferry with a duration tag moves with the speed from it
	static double getTravelSpeed(const SHARED_PTR<TransportRoute>& route, double defaultSpeed) {
		int duration = getDuration(route);
		return duration > 0 ? (double)route->getDist() / duration : defaultSpeed;
	}

	// staying on board: the ferry stops unless its duration includes the stops
	// (a junction isn't a stop, the ferry doesn't stop again at another berth of the same terminal)
	static int getStopTime(const SHARED_PTR<TransportRoutingConfiguration>& cfg, const SHARED_PTR<TransportRoute>& route,
						   int stop) {
		return isFerry(route) && !isJunctionStop(route, stop) && getDuration(route) == 0 &&
					   !isSameTerminal(route, stop - 1, stop) && !isSameTerminal(route, stop, stop + 1)
				   ? cfg->ferryTerminalTime
				   : 0;
	}

	// ferry stops between these ones (inclusive) are berths of one terminal: stops with the same name
	static bool isSameTerminal(const SHARED_PTR<TransportRoute>& route, int from, int to) {
		const auto& stops = route->forwardStops;
		if (!isFerry(route) || from < 0 || to >= (int)stops.size() || stops[from]->name.empty()) {
			return false;
		}
		for (int i = from + 1; i <= to; i++) {
			if (stops[from]->name != stops[i]->name) {
				return false;
			}
		}
		return true;
	}

	// getting off a ferry at the stop (at a junction the ride continues on the next ferry way)
	static double getAlightingTime(const SHARED_PTR<TransportRoutingConfiguration>& cfg,
								   const SHARED_PTR<TransportRoute>& route, int stop) {
		return isFerry(route) && !isJunctionStop(route, stop) ? FerryRoutingHelper::getAlightingTime(cfg->ferryTerminalTime)
															  : 0;
	}

	// ferry on the way to the stop, same as the ferry route itself: getting on, sailing (its duration
	// or its length with the ferry speed) and getting off
	static double getCrossingTime(const SHARED_PTR<TransportRoutingConfiguration>& cfg,
								  const SHARED_PTR<TransportRoute>& route, int stop) {
		int crossing[3] = {0, 0, 0};  // interval, duration, length
		if (!getCrossing(route, stop, crossing)) {
			return 0;
		}
		float speed = cfg->getSpeedByRouteType(FerryRoutingHelper::FERRY);
		double sailingTime = crossing[1] > 0 ? crossing[1] : speed > 0 ? crossing[2] / speed : 0;
		return cfg->getBoardingTime(FerryRoutingHelper::FERRY, crossing[0]) + sailingTime +
			   FerryRoutingHelper::getAlightingTime(cfg->ferryTerminalTime);
	}

	// ferry ways joined by a junction stop in the water are one ferry ride
	// (defined in transportRouteResultSegment.cpp, see the forward declaration above)
	static void mergeJunctionSegments(vector<SHARED_PTR<TransportRouteResultSegment>>& segments);

   private:
	static bool getCrossing(const SHARED_PTR<TransportRoute>& route, int stop, int crossing[3]) {
		string value;
		if (!getStopValue(route, CROSSINGS_TAG, stop, value)) {
			return false;
		}
		vector<string> values = split_string(value, ":");
		for (int i = 0; i < (int)values.size() && i < 3; i++) {
			crossing[i] = OsmAndAlgorithms::parseNumberSilently<int>(values[i], 0);
		}
		return true;
	}

	// route distance is a sum of distances between stops
	static int getDuration(const SHARED_PTR<TransportRoute>& route) {
		if (!isFerry(route)) {
			return 0;
		}
		auto it = route->tags.find(FerryRoutingHelper::DURATION_TAG);
		return it == route->tags.end() ? 0 : FerryRoutingHelper::parseDuration(it->second, route->getDist());
	}

	static bool getStopValue(const SHARED_PTR<TransportRoute>& route, const char* tag, int stop, string& result) {
		auto it = route->tags.find(tag);
		if (it == route->tags.end()) {
			return false;
		}
		for (const string& v : split_string(it->second, ",")) {
			string::size_type sep = v.find(':');
			string index = sep == string::npos ? v : v.substr(0, sep);
			if (OsmAndAlgorithms::parseNumberSilently<int>(index, -1) == stop) {
				result = sep == string::npos ? "" : v.substr(sep + 1);
				return true;
			}
		}
		return false;
	}
};

#endif	// _OSMAND_TRANSPORT_FERRY_HELPER_H
