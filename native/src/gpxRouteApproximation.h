#ifndef _OSMAND_GPX_ROUTE_APPROXIMATION_H
#define _OSMAND_GPX_ROUTE_APPROXIMATION_H

#include "CommonCollections.h"
#include "commonOsmAndCore.h"

struct RoutingIndex;
struct RouteSegmentResult;
struct GpxPoint;
struct RoutingContext;
class RoutePlannerFrontEnd;

struct GpxRouteApproximation {
	RoutingContext* ctx = nullptr;
	RoutePlannerFrontEnd* router = nullptr;
	int routeCalculations = 0;
	int routePointsSearched = 0;
	int routeDistCalculations = 0;
	vector<SHARED_PTR<GpxPoint>> finalPoints;
	vector<SHARED_PTR<RouteSegmentResult>> fullRoute;
	int routeDistance;
	int routeGapDistance;
	int routeDistanceUnmatched;

	void setRouter(RoutePlannerFrontEnd* router);
	GpxRouteApproximation(RoutingContext* rctx);
	GpxRouteApproximation(const GpxRouteApproximation& gctx);

	void searchGpxRouteByRouting(SHARED_PTR<GpxRouteApproximation>& gctx, vector<SHARED_PTR<GpxPoint>>& gpxPoints);

	double distFromLastPoint(double lat, double lon);
	void reconstructFinalPointsFromFullRoute();
	LatLon getLastPoint();

	bool pointCloseEnough(float minPointApproximation, LatLon point,
	                      SHARED_PTR<GpxPoint>& gpxPoint, SHARED_PTR<GpxPoint>& gpxPointNext);
	bool pointCloseEnough(SHARED_PTR<GpxRouteApproximation>& gctx, SHARED_PTR<GpxPoint>& ipoint,
	                      vector<SHARED_PTR<RouteSegmentResult>>& res);
	bool isRouteCloseToGpxPoints(float minPointApproximation, vector<SHARED_PTR<GpxPoint>>& gpxPoints,
	                         SHARED_PTR<GpxPoint>& start, SHARED_PTR<GpxPoint>& next);
	bool initRoutingPoint(SHARED_PTR<GpxPoint>& start, SHARED_PTR<GpxRouteApproximation>& gctx, double distThreshold);
	SHARED_PTR<GpxPoint> findNextGpxPointWithin(vector<SHARED_PTR<GpxPoint>>& gpxPoints, SHARED_PTR<GpxPoint>& start,
												double dist);
	bool findGpxRouteSegment(SHARED_PTR<GpxRouteApproximation>& gctx, vector<SHARED_PTR<GpxPoint>>& gpxPoints,
							 SHARED_PTR<GpxPoint>& start, SHARED_PTR<GpxPoint>& target, bool prevRouteCalculated);
	bool stepBackAndFindPrevPointInRoute(SHARED_PTR<GpxRouteApproximation>& gctx, vector<SHARED_PTR<GpxPoint>>& gpxPoints,
										 SHARED_PTR<GpxPoint>& start, SHARED_PTR<GpxPoint>& next);
	void calculateGpxRoute(SHARED_PTR<GpxRouteApproximation>& gctx, vector<SHARED_PTR<GpxPoint>>& gpxPoints);
	void addStraightLine(const SHARED_PTR<GpxRouteApproximation>& gctx, vector<LatLon>& lastStraightLine, const SHARED_PTR<GpxPoint>& strPnt,
                         const SHARED_PTR<RoutingIndex>& reg);
	void simplifyDouglasPeucker(vector<LatLon>& l, double eps, int start, int end, std::vector<bool>& include);
	void cleanupResultAndAddTurns(SHARED_PTR<GpxRouteApproximation>& gctx);
};

#endif
