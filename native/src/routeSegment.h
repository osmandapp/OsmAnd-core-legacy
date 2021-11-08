#ifndef _OSMAND_ROUTE_SEGMENT_H
#define _OSMAND_ROUTE_SEGMENT_H
#include "CommonCollections.h"
#include "commonOsmAndCore.h"
#include "routeSegmentResult.h"
#include "Logging.h"

struct RouteSegmentNULL;

static const SHARED_PTR<RouteSegmentNULL> routeSegmentNULL;

struct RouteSegment {
    
    uint16_t segmentStart;
    uint16_t segmentEnd;
    SHARED_PTR<RouteDataObject> road;
    // needed to store intersection of routes
    SHARED_PTR<RouteSegment> next;
    SHARED_PTR<RouteSegment> nextLoaded;
    SHARED_PTR<RouteSegment> oppositeDirection;
    SHARED_PTR<RouteSegment> reverseSearch;
    
    // search context (needed for searching route)
    // Initially it should be null (!) because it checks was it segment visited before
    std::weak_ptr<RouteSegment> parentRoute;
    //uint16_t parentSegmentEnd;
    
    
    // 1 - positive , -1 - negative, 0 not assigned
   // int8_t directionAssgn;
    
    // final route segment
    int8_t reverseWaySearch;
    SHARED_PTR<RouteSegment> opposite;
    
    // distance measured in time (seconds)
    // doesn't include distance from @segStart to @segStart + @directionAssgn
    // TODO explain difference for visited & non visited segment
    float distanceFromStart;
    float distanceToEnd;
    
    inline bool isFinal() {
        return reverseWaySearch != 0;
    }
    
    inline bool isReverseWaySearch() {
        return reverseWaySearch == 1;
    }
    
    inline uint16_t getSegmentStart() {
        return segmentStart;
    }

    inline uint16_t getSegmentEnd() {
        return segmentEnd;
    }
    
    inline bool isPositive() {
        return segmentEnd > segmentStart;
    }
    
    inline SHARED_PTR<RouteDataObject>& getRoad() {
        return road;
    }
    
    inline std::weak_ptr<RouteSegment> getParentRoute() {
        return parentRoute.lock() == routeSegmentNULL ? std::weak_ptr<RouteSegment>() : parentRoute;
    };
    
    bool isSegmentAttachedToStart() {
        if (!parentRoute.lock()) {
            return true;
        }
        return false;
    }
    
    static SHARED_PTR<RouteSegment> initRouteSegment(SHARED_PTR<RouteSegment>& th, bool positiveDirection) {
        if(th->segmentStart == 0 && !positiveDirection) {
            return SHARED_PTR<RouteSegment>();
        }
        if(th->segmentStart == th->road->getPointsLength() - 1 && positiveDirection) {
            return SHARED_PTR<RouteSegment>();
        }

        if (th->segmentStart == th->segmentEnd) {
            throw std::invalid_argument("segmentStart or segmentEnd");
        } else {
            if (positiveDirection == (th->segmentEnd > th->segmentStart)) {
                return th;
            } else {
                if (th->oppositeDirection.get() == nullptr) {
                    th->oppositeDirection = std::make_shared<RouteSegment>(th->road, th->segmentStart,
                                                         th->segmentEnd > th->segmentStart ? (th->segmentStart - 1) : (th->segmentStart + 1));
                    th->oppositeDirection->oppositeDirection = th;
                }
                return th->oppositeDirection;
            }
        }
        return nullptr;
    }
    
    RouteSegment()
        : segmentStart(0)
        , road(nullptr)
        , next()
        , oppositeDirection()
        , parentRoute()
        , reverseWaySearch(0)
        , opposite()
        , distanceFromStart(0)
        , distanceToEnd(0) {
    }
    
    RouteSegment(SHARED_PTR<RouteDataObject> road, int segmentStart, int segmentEnd)
        : segmentStart(segmentStart)
        , segmentEnd(segmentEnd)
        , road(road)
        , next()
        , oppositeDirection()
        , parentRoute()
        , reverseWaySearch(0)
        , opposite()
        , distanceFromStart(0)
        , distanceToEnd(0) {
    }

    RouteSegment(SHARED_PTR<RouteDataObject>& road, int segmentStart)
        : segmentStart(segmentStart)
        , segmentEnd(segmentStart < road->getPointsLength() - 1 ? segmentStart + 1 : segmentStart - 1)
        , road(road)
        , next()
        , oppositeDirection()
        , parentRoute()
        , reverseWaySearch(0)
        , opposite()
        , distanceFromStart(0)
        , distanceToEnd(0) {
    }
};

struct RouteSegmentNULL : RouteSegment {
    RouteSegmentNULL(SHARED_PTR<RouteDataObject> road = nullptr, int segmentStart = 0, int segmentEnd = 1) : RouteSegment(road, segmentStart, segmentEnd) {}
};

struct RouteSegmentPoint : RouteSegment {
	RouteSegmentPoint(SHARED_PTR<RouteDataObject>& road, int segmentStart) : RouteSegment(road, segmentStart) {}
	RouteSegmentPoint(SHARED_PTR<RouteSegmentPoint>& pnt) : RouteSegment(pnt->road, pnt->segmentStart, pnt->segmentEnd),
		/*distSquare(pnt->distSquare),*/ preciseX(pnt->preciseX), preciseY(pnt->preciseY) {}
	double dist;
	int preciseX;
	int preciseY;
	vector<SHARED_PTR<RouteSegmentPoint>> others;

	LatLon getPreciseLatLon() { return LatLon(get31LatitudeY(preciseY), get31LongitudeX(preciseX)); }
};

struct FinalRouteSegment : RouteSegment {
	FinalRouteSegment(SHARED_PTR<RouteDataObject>& road, int segmentStart, int segmentEnd) : RouteSegment(road, segmentStart, segmentEnd) {}
	bool reverseWaySearch;
	SHARED_PTR<RouteSegment> opposite;
};

struct GpxPoint {
	int32_t ind;
	double lat;
	double lon;
	double cumDist;
	SHARED_PTR<RouteSegmentPoint> pnt;
	vector<SHARED_PTR<RouteSegmentResult>> routeToTarget;
	vector<SHARED_PTR<RouteSegmentResult>> stepBackRoute;
	int targetInd = -1;
	bool straightLine = false;

	GpxPoint(int32_t ind, double lat, double lon, double cumDist)
		: ind(ind), lat(lat), lon(lon), cumDist(cumDist){};
	
	GpxPoint(const SHARED_PTR<GpxPoint>& p)
		: ind(p->ind), lat(p->lat), lon(p->lon), cumDist(p->cumDist){};
};

#endif /*_OSMAND_ROUTE_SEGMENT_H*/
