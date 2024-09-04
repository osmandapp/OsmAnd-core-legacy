#include "gpxMultiSegmentsApproximation.h"

#include "gpxRouteApproximation.h"
#include "routePlannerFrontEnd.h"

// GpxMultiSegmentsApproximation (port)

// TODO: switch between methods (pass from Java)
// TODO: port GpxMultiSegmentsApproximation class
// TODO: port GpxPoint.object and generateGpxPoints()
// TODO: review Java commits to calculateGpxRoute() method

// DONE: separate GpxRouteApproximation from routePlannetFrontend

GpxMultiSegmentsApproximation::GpxMultiSegmentsApproximation(const RoutePlannerFrontEnd* frontEnd,
                                                             const SHARED_PTR<GpxRouteApproximation>& gctx,
                                                             const std::vector<SHARED_PTR<GpxPoint>>& gpxPoints)
    :
    frontEnd{frontEnd},
    gctx{gctx},
    gpxPoints{gpxPoints},
    minPointApproximation{gctx->ctx->config->minPointApproximation},
    initDist{gctx->ctx->config->minPointApproximation / 2},
    METRICS_COMPARATOR{
        [](const SHARED_PTR<RouteSegmentAppr>& o1, const SHARED_PTR<RouteSegmentAppr>& o2) {
            return o1->metric() > o2->metric();
        }
    },
    queue{METRICS_COMPARATOR} {
}

GpxMultiSegmentsApproximation::RouteSegmentAppr::RouteSegmentAppr(int start, const SHARED_PTR<RouteSegmentPoint>& pnt)
    :
    segment{pnt},
    parent{nullptr},
    gpxStart{start} {
}

GpxMultiSegmentsApproximation::RouteSegmentAppr::RouteSegmentAppr(const SHARED_PTR<RouteSegmentAppr>& parent,
                                                                  const SHARED_PTR<RouteSegment>& segment)
    :
    segment{segment},
    parent{parent},
    gpxStart{parent->gpxStart + parent->gpxLen} {
}

int GpxMultiSegmentsApproximation::RouteSegmentAppr::gpxNext() const {
    return gpxStart + gpxLen + 1;
}

double GpxMultiSegmentsApproximation::RouteSegmentAppr::metric() const {
    return maxDistToGpx / sqrt(gpxLen + 1); // heuristics for eager algorithm
    // return maxDistToGpx;
}

void GpxMultiSegmentsApproximation::gpxApproximation() {
}
