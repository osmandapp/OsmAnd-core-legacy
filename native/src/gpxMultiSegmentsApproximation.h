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
    static constexpr bool DEBUG = false;

public:
    GpxMultiSegmentsApproximation(const SHARED_PTR<GpxRouteApproximation>& gctx,
                                  const std::vector<SHARED_PTR<GpxPoint>>& gpxPoints);
    ~GpxMultiSegmentsApproximation();
    void gpxApproximation();

private:
    struct RouteSegmentAppr {
        const SHARED_PTR<RouteSegment> segment;
        const SHARED_PTR<RouteSegmentAppr> parent;
        const int gpxStart;
        int gpxLen = 0;
        double maxDistToGpx = 0;

        RouteSegmentAppr(int start, const SHARED_PTR<RouteSegmentPoint>& pnt);
        RouteSegmentAppr(const SHARED_PTR<RouteSegmentAppr>& parent, const SHARED_PTR<RouteSegment>& segment);

        int gpxNext() const;
        double metric() const;
        std::string toString() const;
    };

private:
    const SHARED_PTR<GpxRouteApproximation>& gctx;
    const std::vector<SHARED_PTR<GpxPoint>>& gpxPoints;
    const float minPointApproximation;
    const float initDist;

    const std::function<
        bool (const SHARED_PTR<RouteSegmentAppr>& o1, const SHARED_PTR<RouteSegmentAppr>& o2)
    > METRICS_COMPARATOR;

    std::priority_queue<
        SHARED_PTR<RouteSegmentAppr>,
        std::vector<SHARED_PTR<RouteSegmentAppr>>,
        decltype(METRICS_COMPARATOR)
    > queue; // initialized in constructor

    std::unordered_set<int64_t> visited;

    std::vector<std::weak_ptr<RouteSegmentAppr>> garbageValidationList;

private:
    void loadConnections(const SHARED_PTR<RouteSegmentAppr>& last,
                         std::vector<SHARED_PTR<RouteSegmentAppr>>& connected);
    void addSegmentInternal(const SHARED_PTR<RouteSegmentAppr>& last, const SHARED_PTR<RouteSegment>& sg,
                            std::vector<SHARED_PTR<RouteSegmentAppr>>& connected);
    void addSegment(const SHARED_PTR<RouteSegmentAppr>& last, const SHARED_PTR<RouteSegment>& sg,
                    std::vector<SHARED_PTR<RouteSegmentAppr>>& connected);
    bool approximateSegment(const SHARED_PTR<RouteSegmentAppr>& parent, const SHARED_PTR<RouteSegment>& sg,
                            std::vector<SHARED_PTR<RouteSegmentAppr>>& connected);
    bool addConnected(const SHARED_PTR<RouteSegmentAppr>& parent, const SHARED_PTR<RouteSegmentAppr>& c,
                      std::vector<SHARED_PTR<RouteSegmentAppr>>& connected);
    void visit(const SHARED_PTR<RouteSegmentAppr>& r);
    bool isVisited(const SHARED_PTR<RouteSegmentAppr>& r);
    static int64_t calculateRoutePointId(const SHARED_PTR<RouteSegmentAppr>& segm);
    static void initGpxPointsXY31(const std::vector<SHARED_PTR<GpxPoint>>& gpxPoints);
    SHARED_PTR<GpxPoint> findNextRoutablePoint(int searchStart) const;
    bool initRoutingPoint(const SHARED_PTR<GpxPoint>& start, float distThreshold) const;
    static SHARED_PTR<RouteSegmentPoint> initStartPoint(const SHARED_PTR<GpxPoint>& start, double gpxDir,
                                                        const SHARED_PTR<RouteSegmentPoint>& rsp);
    static void debugln(const std::string& line);
    SHARED_PTR<RouteSegmentAppr>& peakMinFromQueue(const SHARED_PTR<RouteSegmentAppr>& bestRoute,
                                                   SHARED_PTR<RouteSegmentAppr>& bestNextMutableRef); // ref -> ref
    double gpxDist(int gpxL1, int gpxL2) const;
    static void wrapupRoute(const std::vector<SHARED_PTR<GpxPoint>>& gpxPoints,
                            const SHARED_PTR<RouteSegmentAppr>& bestRouteConst);
};

#endif
