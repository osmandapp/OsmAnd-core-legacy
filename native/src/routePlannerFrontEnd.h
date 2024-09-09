#ifndef _OSMAND_ROUTE_PLANNER_FRONT_END_H
#define _OSMAND_ROUTE_PLANNER_FRONT_END_H

#include "CommonCollections.h"
#include "routingContext.h"
#include "hhRouteDataStructure.h"
#include "hhRoutePlanner.h"

struct RouteSegmentResult;
struct RouteSegmentPoint;
struct PrecalculatedRouteDirection;
struct RouteSegment;
struct RoutingConfiguration;
struct GpxRouteApproximation;

typedef std::function<bool(const std::shared_ptr<GpxRouteApproximation>& approximation)> GpxRouteApproximationCallback;

class RoutePlannerFrontEnd {
    bool useSmartRouteRecalculation = true;
    HHRoutingConfig * HH_ROUTING_CONFIG = nullptr;
    
public:
    bool USE_ONLY_HH_ROUTING = false;
    bool CALCULATE_MISSING_MAPS = true;
    bool useGeometryBasedApproximation = false;

    constexpr static int GPX_OSM_POINTS_MATCH_ALGORITHM = 1;
    constexpr static int GPX_OSM_MULTISEGMENT_SCAN_ALGORITHM = 2;
    int GPX_SEGMENT_ALGORITHM = GPX_OSM_MULTISEGMENT_SCAN_ALGORITHM; // TODO should be switchable from JNI

    RoutePlannerFrontEnd();
    RoutePlannerFrontEnd(HHRoutingConfig * hhConfig);

    vector<SHARED_PTR<RouteSegmentResult> > searchRoute(RoutingContext* ctx,
                                           vector<SHARED_PTR<RouteSegmentPoint>>& points,
                                           SHARED_PTR<PrecalculatedRouteDirection> routeDirection);
    
    vector<SHARED_PTR<RouteSegmentResult> > searchRouteInternalPrepare(RoutingContext* ctx, SHARED_PTR<RouteSegmentPoint> start, SHARED_PTR<RouteSegmentPoint> end, SHARED_PTR<PrecalculatedRouteDirection> routeDirection);

    void setUseFastRecalculation(bool use);
    void setUseGeometryBasedApproximation(bool enabled);

    SHARED_PTR<RoutingContext> buildRoutingContext(SHARED_PTR<RoutingConfiguration> config, RouteCalculationMode rm = RouteCalculationMode::NORMAL);

    SHARED_PTR<RouteSegmentPoint> getRecalculationEnd(RoutingContext* ctx);

    vector<SHARED_PTR<RouteSegmentResult> > searchRoute(SHARED_PTR<RoutingContext> ctx,
                                           int startX, int startY,
                                           int endX, int endY,
                                           vector<int>& intermediatesX, vector<int>& intermediatesY,
                                           SHARED_PTR<PrecalculatedRouteDirection> routeDirection = nullptr);
    vector<SHARED_PTR<RouteSegmentResult>> searchHHRoute(RoutingContext * ctx);
    bool needRequestPrivateAccessRouting(RoutingContext* ctx, vector<int>& targetsX, vector<int>& targetsY);
    
    static SHARED_PTR<RouteSegmentResult> generateStraightLineSegment(float averageSpeed, vector<pair<double, double>> points);

    vector<SHARED_PTR<GpxPoint>> generateGpxPoints(SHARED_PTR<GpxRouteApproximation>& gctx, const vector<pair<double, double>>& locationsHolder);
    void searchGpxRoute(SHARED_PTR<GpxRouteApproximation>& gctx, vector<SHARED_PTR<GpxPoint>>& gpxPoints, GpxRouteApproximationCallback acceptor = nullptr);
    static bool hasSegment(vector<SHARED_PTR<RouteSegmentResult>>& result, SHARED_PTR<RouteSegment>& current);
    HHRoutingConfig* setDefaultRoutingConfig();

    void makeStartEndPointsPrecise(RoutingContext* ctx, vector<SHARED_PTR<RouteSegmentResult>>& res, int startX, int startY, int endX, int endY);
    void makeSegmentPointPrecise(RoutingContext* ctx, SHARED_PTR<RouteSegmentResult>& routeSegmentResult, int px, int py, bool st);
    void makeSegmentPointPrecise(RoutingContext* ctx, SHARED_PTR<RouteSegmentResult>& routeSegmentResult, double lat, double lon, bool st);

private:
    HHNetworkRouteRes* calculateHHRoute(HHRoutePlanner & routePlanner, RoutingContext* ctx, int startX, int startY, int endX, int endY, double dir);
};

#endif /*_OSMAND_ROUTE_PLANNER_FRONT_END_H*/
