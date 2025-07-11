#ifndef _OSMAND_ROUTE_SEGMENT_H
#define _OSMAND_ROUTE_SEGMENT_H

#include "CommonCollections.h"
#include "commonOsmAndCore.h"
#include "routeSegmentResult.h"

#ifdef _IOS_BUILD
#include <OsmAndCore/Logging.h>
#else
#include "Logging.h"
#endif

struct RouteSegment {
	// Route segment represents part (segment) of the road.
	// In our current data it's always length of 1: [X, X + 1] or [X - 1, X]

	// # Final fields that store objects
	uint16_t segmentStart;
	uint16_t segmentEnd;
	SHARED_PTR<RouteDataObject> road;

	// Segments only allowed for Navigation connected to the same end point
	// Removed due to leaks. It was replaced for std::vector<SHARED_PTR<RouteSegment>> segmentsResult in
	// loadRouteSegment from routingContext SHARED_PTR<RouteSegment> next;

	// # Represents cheap-storage of LinkedList connected segments
	// All the road segments from map data connected to the same end point
	// Removed due to leaks. It was replaced for std::vector<SHARED_PTR<RouteSegment>> segmentsResult in routes from
	// routingContext SHARED_PTR<RouteSegment> nextLoaded;

	// # Caches of similar segments to speed up routing calculation
	// Segment of opposite direction i.e. for [4 -> 5], opposite [5 -> 4]
	weak_ptr<RouteSegment> oppositeDirection;

	// Same Road/ same Segment but used for opposite A* search (important to have different cause #parentRoute is
	// different) Note: if we use 1-direction A* then this is field is not needed
	weak_ptr<RouteSegment> reverseSearch;

	// # Important for A*-search to distinguish whether segment was visited or not
	// Initially all segments null and startSegment/endSegment.parentRoute = RouteSegment.NULL;
	// After iteration stores previous segment i.e. how it was reached from startSegment
	SHARED_PTR<RouteSegment> parentRoute;

	// final route segment
	int8_t reverseWaySearch;
	SHARED_PTR<RouteSegment> opposite;

	// # A* routing - Distance measured in time (seconds)
	// There is a small (important!!!) difference how it's calculated for visited (parentRoute != null) and non-visited
	// NON-VISITED: time from Start [End for reverse A*] to @segStart of @this, including turn time from previous
	// segment (@parentRoute) VISITED: time from Start [End for reverse A*] to @segEnd of @this,
	//          including turn time from previous segment (@parentRoute) and obstacle / distance time between
	//          @segStart-@segEnd on @this
	float distanceFromStart;

	// NON-VISITED: Approximated (h(x)) time from @segStart of @this route segment to End [Start for reverse A*]
	// VISITED: Approximated (h(x)) time from @segEnd of @this route segment to End [Start for reverse A*]
	float distanceToEnd;

	bool isFinalSegment;

	inline bool isReverseWaySearch() { return reverseWaySearch == 1; }

	inline uint16_t getSegmentStart() { return segmentStart; }

	inline uint16_t getSegmentEnd() { return segmentEnd; }

    inline int getStartPointX() { return road->getPoint31XTile(segmentStart); }

    inline int getStartPointY() { return road->getPoint31YTile(segmentStart); }

    inline int getEndPointY() { return road->getPoint31YTile(segmentEnd); }

    inline int getEndPointX() { return road->getPoint31XTile(segmentEnd); }

    float getDistanceFromStart() { return distanceFromStart; }

	inline bool isPositive() { return segmentEnd > segmentStart; }

	inline SHARED_PTR<RouteDataObject>& getRoad() { return road; }

	bool isSegmentAttachedToStart() { return parentRoute != nullptr; }

	inline SHARED_PTR<RouteSegment> getParentRoute();
	inline bool isNull();

	static SHARED_PTR<RouteSegment> initRouteSegment(const SHARED_PTR<RouteSegment>& th, bool positiveDirection) {
		if (th->segmentStart == 0 && !positiveDirection) {
			return nullptr;
		}
		if (th->segmentStart == th->road->getPointsLength() - 1 && positiveDirection) {
			return nullptr;
		}

		if (th->segmentStart == th->segmentEnd) {
			throw std::invalid_argument("segmentStart == segmentEnd");
		} else {
			if (positiveDirection == (th->segmentEnd > th->segmentStart)) {
				return th;
			} else {
				if (th->oppositeDirection.expired()) {
					auto seg = std::make_shared<RouteSegment>(
						th->road, th->segmentStart,
						th->segmentEnd > th->segmentStart ? (th->segmentStart - 1) : (th->segmentStart + 1));
					seg->oppositeDirection = th;
					th->oppositeDirection = seg;
					return seg;
				}
				return th->oppositeDirection.lock();
			}
		}
		return nullptr; // actually unreached
	}

	RouteSegment()
		: segmentStart(0),
		  road(nullptr),
		  oppositeDirection(),
		  parentRoute(),
		  reverseWaySearch(0),
		  opposite(),
		  distanceFromStart(0),
		  distanceToEnd(0),
		  isFinalSegment(false) {}

	RouteSegment(const SHARED_PTR<RouteDataObject>& road, int segmentStart, int segmentEnd)
		: segmentStart(segmentStart),
		  segmentEnd(segmentEnd),
		  road(road),
		  oppositeDirection(),
		  parentRoute(),
		  reverseWaySearch(0),
		  opposite(),
		  distanceFromStart(0),
		  distanceToEnd(0),
		  isFinalSegment(false) {}

	RouteSegment(const SHARED_PTR<RouteDataObject>& road, int segmentStart)
		: segmentStart(segmentStart),
		  segmentEnd(segmentStart < road->getPointsLength() - 1 ? segmentStart + 1 : segmentStart - 1),
		  road(road),
		  oppositeDirection(),
		  parentRoute(),
		  reverseWaySearch(0),
		  opposite(),
		  distanceFromStart(0),
		  distanceToEnd(0),
		  isFinalSegment(false) {}

	virtual ~RouteSegment() = default;

	virtual std::string toString() {
		std::string dst;
		if (road != nullptr) {
			int x = road->getPoint31XTile(segmentStart);
			int y = road->getPoint31YTile(segmentStart);
			int xe = road->getPoint31XTile(segmentEnd);
			int ye = road->getPoint31YTile(segmentEnd);
			dst = std::to_string(((int)(squareRootDist31(x, y, xe, ye) * 10)) / 10.0f) + " m";
		}
		if (distanceFromStart != 0) {
			dst = "dstStart=" + std::to_string((int)(distanceFromStart * 10) / 10.0f);
		}
		std::string rd;
		if (road != nullptr) {
			rd = road->toString();
		}
		std::string ss = "[" + std::to_string(segmentStart) + "-" + std::to_string(segmentEnd) + "]";
		return rd + " " + ss + " " + dst;
	}
};

inline bool RouteSegment::isNull() {
	return parentRoute != nullptr && parentRoute->segmentStart == 0 && parentRoute->segmentEnd == 1 &&
		   parentRoute->road == nullptr;
}

inline SHARED_PTR<RouteSegment> RouteSegment::getParentRoute() { return isNull() ? nullptr : parentRoute; };

struct RouteSegmentPoint : RouteSegment {
	RouteSegmentPoint(const SHARED_PTR<RouteDataObject>& road, int segmentStart, double distSquare)
		: RouteSegment(road, segmentStart), dist(distSquare) {
		this->preciseX = road->getPoint31XTile(segmentStart);
		this->preciseY = road->getPoint31YTile(segmentStart);
	}

	RouteSegmentPoint(const SHARED_PTR<RouteDataObject>& road, int segmentStart, int segmentEnd, double distSquare)
		: RouteSegment(road, segmentStart, segmentEnd), dist(distSquare) {
		this->preciseX = road->getPoint31XTile(segmentStart, segmentEnd);
		this->preciseY = road->getPoint31YTile(segmentStart, segmentEnd);
	}

	RouteSegmentPoint(SHARED_PTR<RouteSegmentPoint>& pnt)
		: RouteSegment(pnt->road, pnt->segmentStart, pnt->segmentEnd),
		  dist(pnt->dist), preciseX(pnt->preciseX), preciseY(pnt->preciseY)
		  /*, distSquare(pnt->distSquare)*/ { }

	double dist;
	int preciseX;
	int preciseY;
	vector<SHARED_PTR<RouteSegmentPoint>> others;

	LatLon getPreciseLatLon() { return LatLon(get31LatitudeY(preciseY), get31LongitudeX(preciseX)); }

	std::string toString() override {
		std::stringstream ss;
		ss
			<< segmentStart << " "
			<< "(" << getPreciseLatLon().lat << "," << getPreciseLatLon().lon << ")"
			<< ": Road(" << road->getId() / 64 << ")";
		return ss.str(); // String.format("%d (%s): %s", segStart, getPreciseLatLon(), road);
	}
};

struct FinalRouteSegment : RouteSegment {
	FinalRouteSegment(const SHARED_PTR<RouteDataObject>& road, int segmentStart, int segmentEnd)
		: RouteSegment(road, segmentStart, segmentEnd) {}
	bool reverseWaySearch;
	SHARED_PTR<RouteSegment> opposite;
};

struct GpxPoint {
	int32_t ind;
	double lat;
	double lon;
	int x31, y31;

	double cumDist;
	SHARED_PTR<RouteSegmentPoint> pnt;
	vector<SHARED_PTR<RouteSegmentResult>> routeToTarget;
	vector<SHARED_PTR<RouteSegmentResult>> stepBackRoute;
	int targetInd = -1;
	bool straightLine = false;
	SHARED_PTR<RouteDataObject> object;

	GpxPoint(int32_t ind, double lat, double lon, double cumDist) : ind(ind), lat(lat), lon(lon), cumDist(cumDist){};

	GpxPoint(const SHARED_PTR<GpxPoint>& p) : ind(p->ind), lat(p->lat), lon(p->lon), cumDist(p->cumDist), object(p->object) {};

	SHARED_PTR<RouteSegmentResult> getFirstRouteRes() { return routeToTarget.empty() ? nullptr : routeToTarget.at(0); }
	SHARED_PTR<RouteSegmentResult> getLastRouteRes() { return routeToTarget.empty() ? nullptr : routeToTarget.at(routeToTarget.size() - 1); }
};

#endif /*_OSMAND_ROUTE_SEGMENT_H*/
