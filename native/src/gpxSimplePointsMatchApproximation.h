#ifndef _OSMAND_GPX_SIMPLE_POINTS_MATCH_APPROXIMATION_H
#define _OSMAND_GPX_SIMPLE_POINTS_MATCH_APPROXIMATION_H

class GpxSimplePointsMatchApproximation {
    constexpr static int LOOKUP_AHEAD = 10;
    constexpr static double DILUTE_BY_SEGMENT_DISTANCE = 0.001; // add a fraction of seg dist to pnt-to-gpx dist (0.001)

public:
    static void gpxApproximation(RoutePlannerFrontEnd* frontEnd, SHARED_PTR<GpxRouteApproximation>& gctx,
                                     std::vector<SHARED_PTR<GpxPoint>>& gpxPoints);

private:
    static double minDistResult(SHARED_PTR<RouteSegmentResult>& res, double minDistSqr,
                                const SHARED_PTR<RouteSegmentPoint>& pnt,
                                const SHARED_PTR<GpxPoint>& loc);

    static double sumPntDistanceSqr(const SHARED_PTR<RouteSegmentPoint>& pnt, int start, int end);

    static SHARED_PTR<GpxPoint> findNextRoutablePoint(RoutePlannerFrontEnd* frontEnd,
                                                      SHARED_PTR<GpxRouteApproximation>& gctx,
                                                      double distThreshold,
                                                      std::vector<SHARED_PTR<GpxPoint>>& gpxPoints,
                                                      int searchStart);

    static void initGpxPointsXY31(std::vector<SHARED_PTR<GpxPoint>>& gpxPoints);
};

#endif
