#ifndef _OSMAND_GPX_MULTI_SEGMENTS_APPROXIMATION_H
#define _OSMAND_GPX_MULTI_SEGMENTS_APPROXIMATION_H

#include <functional>
#include <queue>

#include "CommonCollections.h"
#include "commonOsmAndCore.h"

struct RouteSegmentPoint;
struct RouteSegment;
struct GpxPoint;
struct GpxRouteApproximation;
class RoutePlannerFrontEnd;

class GpxMultiSegmentsApproximation {
    // sync GpxMultiSegmentsApproximation.java
    static constexpr int MAX_DEPTH_ROLLBACK = 500;
    static constexpr double MIN_BRANCHING_DIST = 10;
    static constexpr bool EAGER_ALGORITHM = false;
    static constexpr bool PRIORITY_ALGORITHM = !EAGER_ALGORITHM;
    static constexpr int ROUTE_POINTS = 12;
    static constexpr int GPX_MAX = 30;
    static constexpr bool TEST_SHIFT_GPX_POINTS = false;
    static constexpr bool DEBUG = false;

public:
    GpxMultiSegmentsApproximation(const RoutePlannerFrontEnd* frontEnd,
                                  const SHARED_PTR<GpxRouteApproximation>& gctx,
                                  const std::vector<SHARED_PTR<GpxPoint>>& gpxPoints);
    void gpxApproximation();

private:
    const RoutePlannerFrontEnd* const frontEnd;
    const SHARED_PTR<GpxRouteApproximation>& gctx;
    const std::vector<SHARED_PTR<GpxPoint>>& gpxPoints;
    const float minPointApproximation;
    const float initDist;

    class RouteSegmentAppr {
    private:
        const SHARED_PTR<RouteSegment> segment;
        const SHARED_PTR<RouteSegmentAppr> parent;
        const int gpxStart;
        int gpxLen = 0;
        double maxDistToGpx = 0;
        RouteSegmentAppr(int start, const SHARED_PTR<RouteSegmentPoint>& pnt);
        RouteSegmentAppr(const SHARED_PTR<RouteSegmentAppr>& parent, const SHARED_PTR<RouteSegment>& segment);
        int gpxNext() const;

    public:
        double metric() const;
    };

    const std::function<
        bool (const SHARED_PTR<RouteSegmentAppr>& o1, const SHARED_PTR<RouteSegmentAppr>& o2)
    > METRICS_COMPARATOR;

    const std::priority_queue<
        SHARED_PTR<RouteSegmentAppr>,
        std::vector<SHARED_PTR<RouteSegmentAppr>>,
        decltype(METRICS_COMPARATOR)
    > queue;

    std::unordered_set<int64_t> visited;

    // public void loadConnections(RouteSegmentAppr last, List<RouteSegmentAppr> connected) {
    // private void addSegmentInternal(RouteSegmentAppr last, RouteSegment sg, List<RouteSegmentAppr> connected) {
    // private void addSegment(RouteSegmentAppr last, RouteSegment sg, List<RouteSegmentAppr> connected) {
    // public void visit(RouteSegmentAppr r) {
    // public boolean isVisited(RouteSegmentAppr r) {
    // private RouteSegmentAppr peakMinFromQueue(RouteSegmentAppr bestRoute, RouteSegmentAppr bestNext) {
    // private void debugln(String string) {
    // private double gpxDist(int gpxL1, int gpxL2) {
    // private boolean approximateSegment(RouteSegmentAppr parent, RouteSegment sg, List<RouteSegmentAppr> connected) {
    // private boolean addConnected(RouteSegmentAppr parent, RouteSegmentAppr c, List<RouteSegmentAppr> connected) {
    // public GpxPoint findNextRoutablePoint(int searchStart) throws IOException {
    // private boolean initRoutingPoint(GpxPoint start, double distThreshold) throws IOException {
    // private RouteSegmentPoint initStartPoint(GpxPoint start, double gpxDir, RouteSegmentPoint rsp) {
    // private void initGpxPointsXY31(List<GpxPoint> gpxPoints) {
    // private void wrapupRoute(List<GpxPoint> gpxPoints, RouteSegmentAppr bestRoute) {
    // private static long calculateRoutePointId(RouteSegmentAppr segm) {
};

#endif
