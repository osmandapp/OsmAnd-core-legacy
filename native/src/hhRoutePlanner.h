#ifndef _OSMAND_HH_ROUTE_PLANNER_H
#define _OSMAND_HH_ROUTE_PLANNER_H

#include "CommonCollections.h"
#include "hhRouteDataStructure.h"
#include "routingContext.h"

typedef UNORDERED_map<int64_t, std::vector<NetworkDBPoint *>> MAP_VECTORS_NETWORK_DB_POINTS;

class HHRoutePlanner {
        
public:
    HHRoutePlanner(SHARED_PTR<RoutingContext> ctx);
    
    int DEBUG_VERBOSE_LEVEL = 0;
    int DEBUG_ALT_ROUTE_SELECTION = -1;
    bool ASSERT_COST_INCREASING = false;
    bool ASSERT_AND_CORRECT_DIST_SMALLER = true;
    
    constexpr static const double MINIMAL_COST = 0.01;
    static const int PNT_SHORT_ROUTE_START_END = -1000;
    static const int MAX_POINTS_CLUSTER_ROUTING = 150000;
    static const int ROUTE_POINTS = 11;
    
    SHARED_PTR<HHRoutingConfig> prepareDefaultRoutingConfig(SHARED_PTR<HHRoutingConfig> c);
    HHNetworkRouteRes runRouting(int startX, int startY, int endX, int endY, SHARED_PTR<HHRoutingConfig> config);
    
private:
    double distanceToEnd(SHARED_PTR<HHRoutingContext> hctx, bool reverse, NetworkDBPoint * nextPoint);
    int64_t calcRPId(SHARED_PTR<RouteSegmentPoint> p, int pntId, int nextPntId);
    int64_t calculateRoutePointInternalId(int64_t id, int32_t pntId, int32_t nextPntId);
    int64_t calculateRoutePointInternalId(SHARED_PTR<RouteDataObject> road, int32_t pntId, int32_t nextPntId);
    int64_t calcUniDirRoutePointInternalId(SHARED_PTR<RouteSegmentPoint> segm);
    std::string toString(GeneralRouterProfile grp);
    SHARED_PTR<HHRoutingContext> cacheHctx;
    SHARED_PTR<HHRoutingContext> initHCtx(SHARED_PTR<HHRoutingConfig> c, int startX, int startY, int endX, int endY);
    SHARED_PTR<HHRoutingContext> initNewContext(SHARED_PTR<RoutingContext> ctx, std::vector<SHARED_PTR<HHRouteRegionPointsCtx>> regions);
    SHARED_PTR<HHRoutingContext> selectBestRoutingFiles(int startX, int startY, int endX, int endY, SHARED_PTR<HHRoutingContext> hctx);
    SHARED_PTR<HHRouteRegionsGroup> hhRouteRegionGroup;
    void findFirstLastSegments(SHARED_PTR<HHRoutingContext> hctx, int startX, int startY, int endX, int endY,
                               UNORDERED_map<int64_t, NetworkDBPoint *> stPoints,
                               UNORDERED_map<int64_t, NetworkDBPoint *> endPoints);
    MAP_VECTORS_NETWORK_DB_POINTS groupByClusters(UNORDERED_map<int64_t, NetworkDBPoint *> pointsById, bool out);
    UNORDERED_map<int64_t, NetworkDBPoint *> initStart(SHARED_PTR<HHRoutingContext> hctx, SHARED_PTR<RouteSegmentPoint> s,
                                            bool reverse, UNORDERED_map<int64_t, NetworkDBPoint *> & pnts);
    HHNetworkRouteRes * createRouteSegmentFromFinalPoint(SHARED_PTR<HHRoutingContext> hctx, NetworkDBPoint * pnt);

};

#endif /*_OSMAND_HH_ROUTE_PLANNER_H*/
