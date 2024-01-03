#ifndef _OSMAND_NETWORK_DB_POINT_ROUTE_INFO_H
#define _OSMAND_NETWORK_DB_POINT_ROUTE_INFO_H

#include "commonOsmAndCore.h"
#include "routeSegment.h"

struct NetworkDBPoint;

struct NetworkDBPointRouteInfo {
    NetworkDBPoint * rtRouteToPoint;
    bool rtVisited;
    double rtDistanceFromStart;
    int rtDepth = -1; // possibly not needed (used 1)
    double rtDistanceToEnd; // possibly not needed (used 1)
    double rtCost;
    SHARED_PTR<RouteSegment> rtDetailedRoute;
    
    NetworkDBPointRouteInfo(): rtDetailedRoute() {
    }
    
    int getDepth(bool rev);
    
    //TODO shared_ptr ??
    void setDetailedParentRt(SHARED_PTR<RouteSegment> r);
    
    void setCostParentRt(bool rev, double cost, NetworkDBPoint * point, double segmentDist);
};

#endif
