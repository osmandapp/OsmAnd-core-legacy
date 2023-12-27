#ifndef _OSMAND_HH_ROUTE_PLANNER_H
#define _OSMAND_HH_ROUTE_PLANNER_H

#include "CommonCollections.h"
#include "hhRouteDataStructure.h"
#include "routingContext.h"

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
    
    SHARED_PTR<HHRoutingConfig> prepareDefaultRoutingConfig(SHARED_PTR<HHRoutingConfig> c);
    HHNetworkRouteRes runRouting(int startX, int startY, int endX, int endY, SHARED_PTR<HHRoutingConfig> config);
    
private:
    SHARED_PTR<HHRoutingContext> cacheHctx;
    SHARED_PTR<HHRoutingContext> initHCtx(SHARED_PTR<HHRoutingConfig> c, int startX, int startY, int endX, int endY);
    SHARED_PTR<HHRoutingContext> initNewContext(SHARED_PTR<RoutingContext> ctx, std::vector<SHARED_PTR<HHRouteRegionPointsCtx>> regions);
    SHARED_PTR<HHRoutingContext> selectBestRoutingFiles(int startX, int startY, int endX, int endY, SHARED_PTR<HHRoutingContext> hctx);
    std::string toString(GeneralRouterProfile grp);
    SHARED_PTR<HHRouteRegionsGroup> hhRouteRegionGroup;
};

#endif /*_OSMAND_HH_ROUTE_PLANNER_H*/
