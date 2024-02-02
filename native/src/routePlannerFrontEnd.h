#ifndef _OSMAND_ROUTE_PLANNER_FRONT_END_H
#define _OSMAND_ROUTE_PLANNER_FRONT_END_H
#include "CommonCollections.h"
#include "commonOsmAndCore.h"
#include "routingContext.h"
#include "hhRouteDataStructure.h"
#include "hhRoutePlanner.h"

struct RouteSegmentResult;
struct RouteSegmentPoint;
struct PrecalculatedRouteDirection;
struct RouteSegment;
struct RoutingConfiguration;

struct GpxRouteApproximation {
	
	RoutingContext* ctx;
	int routeCalculations = 0;
	int routePointsSearched = 0;
	int routeDistCalculations = 0;
	vector<SHARED_PTR<GpxPoint>> finalPoints;
	vector<SHARED_PTR<RouteSegmentResult>> result;
	int routeDistance;
	int routeGapDistance;
	int routeDistanceUnmatched;

	GpxRouteApproximation(RoutingContext* rctx) { ctx = rctx; }

	GpxRouteApproximation(const GpxRouteApproximation& gctx) {
		ctx = gctx.ctx;
		routeDistance = gctx.routeDistance;
	}

	double distFromLastPoint(double lat, double lon);

	LatLon getLastPoint();
};

typedef std::function<bool(const std::shared_ptr<GpxRouteApproximation>& approximation)> GpxRouteApproximationCallback;

class RoutePlannerFrontEnd {
    
    bool useSmartRouteRecalculation;
    
    bool TRACE_ROUTING = false;
    bool USE_HH_ROUTING = false;
    bool USE_ONLY_HH_ROUTING = false;
    HHRoutingConfig * HH_ROUTING_CONFIG = nullptr;
    
public:
    
    RoutePlannerFrontEnd() : useSmartRouteRecalculation(true) {
    }
    
    RoutePlannerFrontEnd(HHRoutingConfig * hhConfig) : useSmartRouteRecalculation(true) {
        if (HH_ROUTING_CONFIG != nullptr) {
            delete HH_ROUTING_CONFIG;
        }
        HH_ROUTING_CONFIG = hhConfig;
    }
    
    vector<SHARED_PTR<RouteSegmentResult> > searchRoute(RoutingContext* ctx,
                                           vector<SHARED_PTR<RouteSegmentPoint>>& points,
                                           SHARED_PTR<PrecalculatedRouteDirection> routeDirection);
    
    vector<SHARED_PTR<RouteSegmentResult> > searchRouteInternalPrepare(RoutingContext* ctx, SHARED_PTR<RouteSegmentPoint> start, SHARED_PTR<RouteSegmentPoint> end, SHARED_PTR<PrecalculatedRouteDirection> routeDirection);
    
    void setUseFastRecalculation(bool use) {
        useSmartRouteRecalculation = use;
    }

    SHARED_PTR<RoutingContext> buildRoutingContext(SHARED_PTR<RoutingConfiguration> config, RouteCalculationMode rm = RouteCalculationMode::NORMAL);

	SHARED_PTR<RouteSegmentPoint> getRecalculationEnd(RoutingContext* ctx);

	vector<SHARED_PTR<RouteSegmentResult> > searchRoute(SHARED_PTR<RoutingContext> ctx,
                                           int startX, int startY,
                                           int endX, int endY,
                                           vector<int>& intermediatesX, vector<int>& intermediatesY,
                                           SHARED_PTR<PrecalculatedRouteDirection> routeDirection = nullptr);
    vector<SHARED_PTR<RouteSegmentResult>> searchHHRoute(RoutingContext * ctx);
    bool needRequestPrivateAccessRouting(SHARED_PTR<RoutingContext> ctx, vector<int>& targetsX, vector<int>& targetsY);
    
    static SHARED_PTR<RouteSegmentResult> generateStraightLineSegment(float averageSpeed, vector<pair<double, double>> points);
	
	vector<SHARED_PTR<GpxPoint>> generateGpxPoints(SHARED_PTR<GpxRouteApproximation>& gctx, const vector<pair<double, double>>& locationsHolder);
	
	void searchGpxRoute(SHARED_PTR<GpxRouteApproximation>& gctx, vector<SHARED_PTR<GpxPoint>>& gpxPoints, GpxRouteApproximationCallback acceptor = nullptr);
	static bool hasSegment(vector<SHARED_PTR<RouteSegmentResult>>& result, SHARED_PTR<RouteSegment>& current);
    HHRoutingConfig * setDefaultRoutingConfig();
	
private:
	
	SHARED_PTR<GpxPoint> findNextGpxPointWithin(vector<SHARED_PTR<GpxPoint>>& gpxPoints, SHARED_PTR<GpxPoint>& start,
												double dist);
	bool findGpxRouteSegment(SHARED_PTR<GpxRouteApproximation>& gctx, vector<SHARED_PTR<GpxPoint>>& gpxPoints,
							 SHARED_PTR<GpxPoint>& start, SHARED_PTR<GpxPoint>& target, bool prevRouteCalculated);
	bool initRoutingPoint(SHARED_PTR<GpxPoint>& start, SHARED_PTR<GpxRouteApproximation>& gctx, double distThreshold);
	bool stepBackAndFindPrevPointInRoute(SHARED_PTR<GpxRouteApproximation>& gctx, vector<SHARED_PTR<GpxPoint>>& gpxPoints,
										 SHARED_PTR<GpxPoint>& start, SHARED_PTR<GpxPoint>& next);
	void calculateGpxRoute(SHARED_PTR<GpxRouteApproximation>& gctx, vector<SHARED_PTR<GpxPoint>>& gpxPoints);
	void addStraightLine(const SHARED_PTR<GpxRouteApproximation>& gctx, vector<LatLon>& lastStraightLine, const SHARED_PTR<GpxPoint>& strPnt,
                         const SHARED_PTR<RoutingIndex>& reg);
	void cleanupResultAndAddTurns(SHARED_PTR<GpxRouteApproximation>& gctx);
	void simplifyDouglasPeucker(vector<LatLon>& l, double eps, int start, int end, std::vector<bool>& include);
	bool isRouteCloseToGpxPoints(SHARED_PTR<GpxRouteApproximation>& gctx, vector<SHARED_PTR<GpxPoint>>& gpxPoints,
	                         SHARED_PTR<GpxPoint>& start, SHARED_PTR<GpxPoint>& next);
	bool pointCloseEnough(SHARED_PTR<GpxRouteApproximation>& gctx, LatLon point,
	                      SHARED_PTR<GpxPoint>& gpxPoint, SHARED_PTR<GpxPoint>& gpxPointNext);
	bool pointCloseEnough(SHARED_PTR<GpxRouteApproximation>& gctx, SHARED_PTR<GpxPoint>& ipoint,
	                      vector<SHARED_PTR<RouteSegmentResult>>& res);
	void makeSegmentPointPrecise(SHARED_PTR<RouteSegmentResult>& routeSegmentResult, double lat, double lon, bool st);
    HHNetworkRouteRes * calculateHHRoute(HHRoutePlanner & routePlanner, int startX, int startY, int endX, int endY, double dir);
};

#endif /*_OSMAND_ROUTE_PLANNER_FRONT_END_H*/
