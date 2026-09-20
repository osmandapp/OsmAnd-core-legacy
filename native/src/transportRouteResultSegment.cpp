#ifndef _OSMAND_TRANSPORT_ROUTE_RESULT_SEGMENT_CPP
#define _OSMAND_TRANSPORT_ROUTE_RESULT_SEGMENT_CPP
#include "transportRouteResultSegment.h"

#include "transportFerryHelper.h"
#include "transportRoutingObjects.h"

TransportRouteResultSegment::TransportRouteResultSegment() {
}

int TransportRouteResultSegment::getArrivalTime() {
	if (depTime != -1) {
		int32_t tm = depTime;
		std::vector<int32_t> intervals = route->schedule.avgStopIntervals;
		for (int i = start; i <= end; i++) {
			if (i == end) {
				return tm;
			}
			if (intervals.size() > 1) {
				tm += intervals.at(i);
			} else {
				break;
			}
		}
	}
	return -1;
}

double TransportRouteResultSegment::getTravelDist() {
	double d = 0;
	for (int32_t k = start; k < end; k++) {
		const auto& stop = route->forwardStops[k];
		const auto& nextStop = route->forwardStops[k + 1];
		d += getDistance(stop->lat, stop->lon, nextStop->lat, nextStop->lon);
	}
	return d;
}

void TransportRouteResultSegment::getGeometry(vector<shared_ptr<Way>>& list) {
	route->mergeForwardWays();
	if (DISPLAY_FULL_SEGMENT_ROUTE) {
		if (route->forwardWays.size() > DISPLAY_SEGMENT_IND && DISPLAY_SEGMENT_IND != -1) {
			list.push_back(route->forwardWays[DISPLAY_SEGMENT_IND]);
			return;
		}
		list.insert(list.end(), route->forwardWays.begin(), route->forwardWays.end());
		return;
	}
	vector<shared_ptr<Way>> ways = route->forwardWays;

	const double startLat = getStart().lat;
	const double startLon = getStart().lon;
	const double endLat = getEnd().lat;
	const double endLon = getEnd().lon;

	SearchNodeInd startInd;
	SearchNodeInd endInd;

	vector<Node> res;
	for (int i = 0; i < ways.size(); i++) {
		// for (auto it = ways.begin(); it != ways.end(); ++it) {
		vector<Node> nodes = ways[i]->nodes;
		// for (auto nodesIt = nodes.begin(); nodesIt != nodes.end(); ++nodesIt) {
		for (int j = 0; j < nodes.size(); j++) {
			const auto n = nodes[j];
			double startDist = getDistance(startLat, startLon, n.lat, n.lon);
			if (startDist < startInd.dist) {
				startInd.dist = startDist;
				startInd.ind = j;
				startInd.way = ways[i];
			}
			double endDist = getDistance(endLat, endLon, n.lat, n.lon);
			if (endDist < endInd.dist) {
				endInd.dist = endDist;
				endInd.ind = j;
				endInd.way = ways[i];
			}
		}
	}
	// parallel ways of one route (ferry berths) are merged into a way going there and back,
	// so the part between the stops can be in any direction
	bool validOneWay = startInd.way != nullptr && startInd.way == endInd.way;
	if (validOneWay) {
		shared_ptr<Way> way = make_shared<Way>(GEOMETRY_WAY_ID);
		int step = startInd.ind <= endInd.ind ? 1 : -1;
		for (int k = startInd.ind; k != endInd.ind + step; k += step) {
			way->addNode(startInd.way->nodes[k]);
		}
		list.push_back(way);
		return;
	}
	bool validContinuation = startInd.way != nullptr && endInd.way != nullptr && startInd.way != endInd.way;
	if (validContinuation) {
		Node ln = startInd.way->getLastNode();
		Node fn = endInd.way->getFirstNode();
		// HERE we need to check other ways for continuation
		if (getDistance(ln.lat, ln.lon, fn.lat, fn.lon) < MISSING_STOP_SEARCH_RADIUS) {
			validContinuation = true;
		} else {
			validContinuation = false;
		}
	}
	if (validContinuation) {
		SHARED_PTR<Way> way = make_shared<Way>(GEOMETRY_WAY_ID);
		for (int k = startInd.ind; k < startInd.way->nodes.size(); k++) {
			way->addNode(startInd.way->nodes[k]);
		}
		list.push_back(way);
		way = make_shared<Way>(GEOMETRY_WAY_ID);
		for (int k = 0; k <= endInd.ind; k++) {
			way->addNode(endInd.way->nodes[k]);
		}
		list.push_back(way);
		return;
	}

	SHARED_PTR<Way> way = make_shared<Way>(STOPS_WAY_ID);
	for (int i = start; i <= end; i++) {
		double lLat = getStop(i).lat;
		double lLon = getStop(i).lon;
		Node n(lLat, lLon);
		way->addNode(n);
	}
	list.push_back(way);
}

const TransportStop& TransportRouteResultSegment::getStart() {
	return *route->forwardStops.at(start).get();
}

const TransportStop& TransportRouteResultSegment::getEnd() {
	return *route->forwardStops.at(end).get();
}

vector<SHARED_PTR<TransportStop>> TransportRouteResultSegment::getTravelStops() {
	return vector<SHARED_PTR<TransportStop>>(route->forwardStops.begin() + start,
											 route->forwardStops.begin() + end + 1);
}

const TransportStop& TransportRouteResultSegment::getStop(int32_t i) {
	return *route->forwardStops.at(i).get();
}

// the junction stop itself is dropped: the two ways become one ride between its own start and end
static SHARED_PTR<TransportRouteResultSegment> merge(const SHARED_PTR<TransportRouteResultSegment>& s,
													 const SHARED_PTR<TransportRouteResultSegment>& next) {
	vector<SHARED_PTR<TransportStop>> travelStops = s->getTravelStops();
	vector<SHARED_PTR<TransportStop>> nextStops = next->getTravelStops();
	vector<SHARED_PTR<TransportStop>> stops(travelStops.begin(), travelStops.end() - 1);
	stops.insert(stops.end(), nextStops.begin() + 1, nextStops.end());
	vector<SHARED_PTR<Way>> ways;
	for (const auto& w : s->route->forwardWays) {
		ways.push_back(make_shared<Way>(*w));
	}
	for (const auto& w : next->route->forwardWays) {
		ways.push_back(make_shared<Way>(*w));
	}
	SHARED_PTR<TransportRouteResultSegment> res = make_shared<TransportRouteResultSegment>();
	res->route = make_shared<TransportRoute>(s->route, stops, ways);
	res->start = 0;
	res->end = (int32_t)stops.size() - 1;
	res->walkDist = s->walkDist;
	res->walkTime = s->walkTime;
	res->depTime = s->depTime;
	res->travelTime = s->travelTime + next->travelTime;
	res->travelDistApproximate = s->travelDistApproximate + next->travelDistApproximate;
	return res;
}

void TransportFerryHelper::mergeJunctionSegments(vector<SHARED_PTR<TransportRouteResultSegment>>& segments) {
	for (int i = (int)segments.size() - 1; i > 0; i--) {
		SHARED_PTR<TransportRouteResultSegment> s = segments[i - 1];
		if (isJunctionStop(s->route, s->end)) {
			SHARED_PTR<TransportRouteResultSegment> next = segments[i];
			segments.erase(segments.begin() + i);
			segments[i - 1] = merge(s, next);
		}
	}
}

#endif /*_OSMAND_TRANSPORT_ROUTE_RESULT_SEGMENT_CPP*/
