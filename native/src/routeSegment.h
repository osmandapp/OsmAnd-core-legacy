#ifndef _OSMAND_ROUTE_SEGMENT_H
#define _OSMAND_ROUTE_SEGMENT_H
#include "CommonCollections.h"
#include "commonOsmAndCore.h"
#include "routeSegmentResult.h"
#include "Logging.h"

struct RouteSegment {
    // Route segment represents part (segment) of the road. 
	// In our current data it's always length of 1: [X, X + 1] or [X - 1, X] 
    
    // # Final fields that store objects
    uint16_t segmentStart;
    uint16_t segmentEnd;
    SHARED_PTR<RouteDataObject> road;
    // Segments only allowed for Navigation connected to the same end point
    SHARED_PTR<RouteSegment> next;
    // # Represents cheap-storage of LinkedList connected segments
	// All the road segments from map data connected to the same end point
    // removed due to leaks
    //SHARED_PTR<RouteSegment> nextLoaded;
    weak_ptr<RouteSegment> oppositeDirection;
    // Same Road/ same Segment but used for opposite A* search (important to have different cause #parentRoute is different)
	// Note: if we use 1-direction A* then this is field is not needed
    SHARED_PTR<RouteSegment> reverseSearch;
    
    // # Important for A*-search to distinguish whether segment was visited or not
	// Initially all segments null and startSegment/endSegment.parentRoute = RouteSegment.NULL;
	// After iteration stores previous segment i.e. how it was reached from startSegment
    SHARED_PTR<RouteSegment> parentRoute;
    
    // final route segment
    int8_t reverseWaySearch;
    // # Caches of similar segments to speed up routing calculation 
	// Segment of opposite direction i.e. for [4 -> 5], opposite [5 -> 4]
    SHARED_PTR<RouteSegment> opposite;
    
    // # A* routing - Distance measured in time (seconds)
	// There is a small (important!!!) difference how it's calculated for visited (parentRoute != null) and non-visited
	// NON-VISITED: time from Start [End for reverse A*] to @segStart of @this, including turn time from previous segment (@parentRoute)
	// VISITED: time from Start [End for reverse A*] to @segEnd of @this, 
	//          including turn time from previous segment (@parentRoute) and obstacle / distance time between @segStart-@segEnd on @this 
    float distanceFromStart;
    // NON-VISITED: Approximated (h(x)) time from @segStart of @this route segment to End [Start for reverse A*] 
	// VISITED: Approximated (h(x)) time from @segEnd of @this route segment to End [Start for reverse A*]
    float distanceToEnd;
    bool isFinalSegment;
    
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
    
    bool isSegmentAttachedToStart() {
		return parentRoute != nullptr;
    }
    
    inline SHARED_PTR<RouteSegment> getParentRoute();
    inline bool isNull();
    
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
                if (th->oppositeDirection.expired()) {
                    auto seg  = std::make_shared<RouteSegment>(th->road, th->segmentStart,
                                                               th->segmentEnd > th->segmentStart ? (th->segmentStart - 1) : (th->segmentStart + 1));
                    seg->oppositeDirection = th;
                    th->oppositeDirection = seg;
                    return seg;
                }
                return th->oppositeDirection.lock();
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
        , distanceToEnd(0)
        , isFinalSegment(false) {
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
        , distanceToEnd(0)
        , isFinalSegment(false) {
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
        , distanceToEnd(0)
        , isFinalSegment(false) {
    }
};

inline bool RouteSegment::isNull() {
    return parentRoute != nullptr && parentRoute->segmentStart == 0 && parentRoute->segmentEnd == 1 && parentRoute->road == nullptr;
}

inline SHARED_PTR<RouteSegment> RouteSegment::getParentRoute() {
    return isNull() ? nullptr : parentRoute;
};

struct RouteSegmentPoint : RouteSegment {
	RouteSegmentPoint(SHARED_PTR<RouteDataObject>& road, int segmentStart) : RouteSegment(road, segmentStart) {}
	RouteSegmentPoint(SHARED_PTR<RouteSegmentPoint>& pnt) : RouteSegment(pnt->road, pnt->segmentStart),
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
