#include "NetworkDBPointRouteInfo.h"
#include "hhRouteDataStructure.h"

int NetworkDBPointRouteInfo::getDepth(bool rev) {
    if (rtDepth > 0) {
        return rtDepth;
    }
    if (rtRouteToPoint) {
        rtDepth = rtRouteToPoint->rt(rev)->getDepth(rev) + 1;
        return rtDepth ;
    }
    return 0;
}

void NetworkDBPointRouteInfo::setDetailedParentRt(FinalRouteSegment & r) {
    double segmentDist = r.getDistanceFromStart();
    rtRouteToPoint = nullptr;
    rtCost = rtDistanceToEnd + segmentDist;
    rtDetailedRoute = r;
    rtDistanceFromStart = segmentDist;
}

void NetworkDBPointRouteInfo::setCostParentRt(bool rev, double cost, SHARED_PTR<NetworkDBPoint> point, double segmentDist) {
    rtCost = cost;
    //TODO check memory leak
    rtRouteToPoint = point;
    rtDistanceFromStart = (point == nullptr ? 0 : point->rt(rev)->rtDistanceFromStart) + segmentDist;
}

