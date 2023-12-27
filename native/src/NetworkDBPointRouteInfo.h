#ifndef _OSMAND_NETWORK_DB_POINT_ROUTE_INFO_H
#define _OSMAND_NETWORK_DB_POINT_ROUTE_INFO_H

#include "commonOsmAndCore.h"
#include "routeSegment.h"

struct NetworkDBPoint;

struct NetworkDBPointRouteInfo {
    SHARED_PTR<NetworkDBPoint> rtRouteToPoint;
    bool rtVisited;
    double rtDistanceFromStart;
    int rtDepth = -1; // possibly not needed (used 1)
    double rtDistanceToEnd; // possibly not needed (used 1)
    double rtCost;
    FinalRouteSegment rtDetailedRoute;
    
    NetworkDBPointRouteInfo(): rtDetailedRoute(nullptr, 0, 0) {
    }
    
    int getDepth(bool rev);
    
    //TODO shared_ptr ??
    void setDetailedParentRt(FinalRouteSegment & r);
    
    void setCostParentRt(bool rev, double cost, SHARED_PTR<NetworkDBPoint> point, double segmentDist);
};

#endif
