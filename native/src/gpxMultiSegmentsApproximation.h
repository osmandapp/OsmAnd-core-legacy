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
    static constexpr bool VERBOSE = false; // DEBUG

public:
    GpxMultiSegmentsApproximation(const SHARED_PTR<GpxRouteApproximation>& gctx,
                                  const std::vector<SHARED_PTR<GpxPoint>>& gpxPoints);
    ~GpxMultiSegmentsApproximation();
    void gpxApproximation();

private:
    struct RouteSegmentAppr {
        const SHARED_PTR<RouteSegment> segment;
        const RouteSegmentAppr* parent;
        const int gpxStart;
        int gpxLen = 0;
        double maxDistToGpx = 0;

        RouteSegmentAppr(int start, const SHARED_PTR<RouteSegmentPoint>& pnt);
        RouteSegmentAppr(const RouteSegmentAppr* parent, const SHARED_PTR<RouteSegment>& segment);

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
        bool (const RouteSegmentAppr* o1, const RouteSegmentAppr* o2)
    > METRICS_COMPARATOR;

    std::priority_queue<
        RouteSegmentAppr*,
        std::vector<RouteSegmentAppr*>,
        decltype(METRICS_COMPARATOR)
    > queue; // initialized in constructor

    std::vector<RouteSegmentAppr*> garbage;

    std::unordered_set<int64_t> visited;

private:
    void loadConnections(const RouteSegmentAppr* last, std::vector<RouteSegmentAppr*>& connected);
    void addSegmentInternal(const RouteSegmentAppr* last, const SHARED_PTR<RouteSegment>& sg,
                            std::vector<RouteSegmentAppr*>& connected);
    void addSegment(const RouteSegmentAppr* last, const SHARED_PTR<RouteSegment>& sg,
                    std::vector<RouteSegmentAppr*>& connected);
    bool approximateSegment(const RouteSegmentAppr* parent, const SHARED_PTR<RouteSegment>& sg,
                            std::vector<RouteSegmentAppr*>& connected);
    bool addConnected(const RouteSegmentAppr* parent, RouteSegmentAppr* c, std::vector<RouteSegmentAppr*>& connected);
    void visit(const RouteSegmentAppr* r);
    bool isVisited(const RouteSegmentAppr* r);
    static int64_t calculateRoutePointId(const RouteSegmentAppr* segm);
    static void initGpxPointsXY31(const std::vector<SHARED_PTR<GpxPoint>>& gpxPoints);
    SHARED_PTR<GpxPoint> findNextRoutablePoint(int searchStart) const;
    bool initRoutingPoint(const SHARED_PTR<GpxPoint>& start, float distThreshold) const;
    static SHARED_PTR<RouteSegmentPoint> initStartPoint(const SHARED_PTR<GpxPoint>& start, double gpxDir,
                                                        const SHARED_PTR<RouteSegmentPoint>& rsp);
    static void debugln(const std::string& line);
    RouteSegmentAppr* peakMinFromQueue(const RouteSegmentAppr* bestRoute, RouteSegmentAppr* bestNext);
    double gpxDist(int gpxL1, int gpxL2) const;
    static void wrapupRoute(const std::vector<SHARED_PTR<GpxPoint>>& gpxPoints, const RouteSegmentAppr* bestRoute);
};

#endif
