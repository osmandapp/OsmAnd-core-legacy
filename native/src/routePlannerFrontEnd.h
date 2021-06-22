#ifndef _OSMAND_ROUTE_PLANNER_FRONT_END_H
#define _OSMAND_ROUTE_PLANNER_FRONT_END_H
#include "CommonCollections.h"
#include "commonOsmAndCore.h"
#include "routingContext.h"

struct RouteSegmentResult;
struct RouteSegmentPoint;
struct PrecalculatedRouteDirection;
struct RouteSegment;
struct RoutingConfiguration;

struct GpxRouteApproximation {
	// ! MAIN parameter to approximate (35m good for custom recorded tracks)
	double MINIMUM_POINT_APPROXIMATION = 50; // 35 m good for small deviations
	// This parameter could speed up or slow down evaluation (better to make bigger for long routes and smaller for short)
	double MAXIMUM_STEP_APPROXIMATION = 3000;
	// don't search subsegments shorter than specified distance (also used to step back for car turns)
	double MINIMUM_STEP_APPROXIMATION = 100;
	// Parameter to smoother the track itself (could be 0 if it's not recorded track)
	double SMOOTHEN_POINTS_NO_ROUTE = 5;
	
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
    
public:
    
    RoutePlannerFrontEnd() : useSmartRouteRecalculation(true) {
    }
    
    vector<SHARED_PTR<RouteSegmentResult> > searchRoute(RoutingContext* ctx,
                                           vector<SHARED_PTR<RouteSegmentPoint>>& points,
                                           SHARED_PTR<PrecalculatedRouteDirection> routeDirection);
    
    vector<SHARED_PTR<RouteSegmentResult> > searchRouteInternalPrepare(RoutingContext* ctx, SHARED_PTR<RouteSegmentPoint> start, SHARED_PTR<RouteSegmentPoint> end, SHARED_PTR<PrecalculatedRouteDirection> routeDirection);
    
    void setUseFastRecalculation(bool use) {
        useSmartRouteRecalculation = use;
    }

    SHARED_PTR<RoutingContext> buildRoutingContext(SHARED_PTR<RoutingConfiguration> config, RouteCalculationMode rm = RouteCalculationMode::NORMAL);
    
    SHARED_PTR<RouteSegment> getRecalculationEnd(RoutingContext* ctx);

    vector<SHARED_PTR<RouteSegmentResult> > searchRoute(SHARED_PTR<RoutingContext> ctx,
                                           int startX, int startY,
                                           int endX, int endY,
                                           vector<int>& intermediatesX, vector<int>& intermediatesY,
                                           SHARED_PTR<PrecalculatedRouteDirection> routeDirection = nullptr);
    
    static SHARED_PTR<RouteSegmentResult> generateStraightLineSegment(float averageSpeed, vector<pair<double, double>> points);
	
	vector<SHARED_PTR<GpxPoint>> generateGpxPoints(SHARED_PTR<GpxRouteApproximation>& gctx, const vector<pair<double, double>>& locationsHolder);
	
	void searchGpxRoute(SHARED_PTR<GpxRouteApproximation>& gctx, vector<SHARED_PTR<GpxPoint>>& gpxPoints, GpxRouteApproximationCallback acceptor = nullptr);
	
private:
	
	SHARED_PTR<GpxPoint> findNextGpxPointWithin(vector<SHARED_PTR<GpxPoint>>& gpxPoints, SHARED_PTR<GpxPoint>& start,
												double dist);
	bool findGpxRouteSegment(SHARED_PTR<GpxRouteApproximation>& gctx, vector<SHARED_PTR<GpxPoint>>& gpxPoints,
							 SHARED_PTR<GpxPoint>& start, SHARED_PTR<GpxPoint>& target, bool prevRouteCalculated);
	bool initRoutingPoint(SHARED_PTR<GpxPoint>& start, SHARED_PTR<GpxRouteApproximation>& gctx, double distThreshold);
	bool stepBackAndFindPrevPointInRoute(SHARED_PTR<GpxRouteApproximation>& gctx, vector<SHARED_PTR<GpxPoint>>& gpxPoints,
										 SHARED_PTR<GpxPoint>& start, SHARED_PTR<GpxPoint>& next);
	void calculateGpxRoute(SHARED_PTR<GpxRouteApproximation>& gctx, vector<SHARED_PTR<GpxPoint>>& gpxPoints);
	void addStraightLine(SHARED_PTR<GpxRouteApproximation>& gctx, vector<LatLon>& lastStraightLine, SHARED_PTR<GpxPoint>& strPnt,
						 RoutingIndex* reg);
	void cleanupResultAndAddTurns(SHARED_PTR<GpxRouteApproximation>& gctx);
	void simplifyDouglasPeucker(vector<LatLon>& l, double eps, int start, int end);
	bool pointCloseEnough(SHARED_PTR<GpxRouteApproximation>& gctx, SHARED_PTR<GpxPoint>& ipoint,
						  vector<SHARED_PTR<RouteSegmentResult>>& res);
	void makeSegmentPointPrecise(SHARED_PTR<RouteSegmentResult>& routeSegmentResult, double lat, double lon, bool st);
};

#endif /*_OSMAND_ROUTE_PLANNER_FRONT_END_H*/
