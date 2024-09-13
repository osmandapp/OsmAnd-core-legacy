#include "gpxSimplePointsMatchApproximation.h"
#include "gpxRouteApproximation.h"

#include "routePlannerFrontEnd.h"

void GpxSimplePointsMatchApproximation::gpxApproximation(RoutePlannerFrontEnd* frontEnd,
                                                    SHARED_PTR<GpxRouteApproximation>& gctx,
                                                    std::vector<SHARED_PTR<GpxPoint>>& gpxPoints) {
    OsmAnd::ElapsedTimer timer;
    timer.Start();

    initGpxPointsXY31(gpxPoints);

    float minPointApproximation = gctx->ctx->config->minPointApproximation;
    SHARED_PTR<GpxPoint> currentPoint = findNextRoutablePoint(frontEnd, gctx, minPointApproximation, gpxPoints, 0);

    while (currentPoint != nullptr && currentPoint->pnt != nullptr) {
        double minDistSqrSegment = 0;
        SHARED_PTR<RouteSegmentResult> fres = nullptr;
        int minNextInd = -1;
        int start = currentPoint->ind + 1;
        int end = std::min(currentPoint->ind + LOOKUP_AHEAD, static_cast<int>(gpxPoints.size()));
        for (int j = start; j < end; j++) {
            SHARED_PTR<RouteSegmentResult> res = nullptr;
            double minDistSqr = std::numeric_limits<double>::infinity();
            const auto& ps = gpxPoints.at(j);
            minDistSqr = minDistResult(res, minDistSqr, currentPoint->pnt, ps);
            if (!currentPoint->pnt->others.empty() /* != null */) {
                for (const auto& oth : currentPoint->pnt->others) {
                    minDistSqr = minDistResult(res, minDistSqr, oth, ps);
                }
            }
            if (fres == nullptr || minDistSqr <= minDistSqrSegment) {
                fres = std::move(res);
                minDistSqrSegment = minDistSqr;
                minNextInd = j;
            }
            if (getDistance(currentPoint->lat, currentPoint->lon,
                            gpxPoints.at(j)->lat, gpxPoints.at(j)->lon) > minPointApproximation) {
                break; // avoid shortcutting of loops
            }
        }
        if (minNextInd < 0) {
            break;
        }
        if (minDistSqrSegment > minPointApproximation * minPointApproximation) {
            const int nextIndex = currentPoint->ind + 1;
            currentPoint = findNextRoutablePoint(frontEnd, gctx, minPointApproximation, gpxPoints, nextIndex);
            continue;
        }
        fres->setGpxPointIndex(currentPoint->ind);
        currentPoint->routeToTarget.clear(); // = new ArrayList<RouteSegmentResult>()
        currentPoint->routeToTarget.push_back(fres);
        currentPoint->targetInd = minNextInd;

        currentPoint = gpxPoints.at(minNextInd); // next point

        auto sgVectorSharedRouteSegment = gctx->ctx->loadRouteSegment(fres->getEndPointX(), fres->getEndPointY());

        for (const auto& sg : sgVectorSharedRouteSegment) {
            if (sg->getRoad()->getId() != fres->object->getId() || sg->getSegmentEnd() != fres->getEndPointIndex()) {
                auto p = std::make_shared<RouteSegmentPoint>(sg->getRoad(), sg->getSegmentStart(), sg->getSegmentEnd(),
                                                             0);
                if (currentPoint->pnt == nullptr) {
                    currentPoint->pnt = p;
                }
                else {
                    // if (currentPoint.pnt.others == null) {
                    //     currentPoint.pnt.others = new ArrayList<>();
                    // }
                    currentPoint->pnt->others.push_back(p);
                }
            }
        }
    }

    OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info,
                      "Native Geo-Based Approximation took %.2f seconds (%d route points searched)",
                      static_cast<double>(timer.GetElapsedMs()) / 1000.0, gctx->routePointsSearched);
}

double GpxSimplePointsMatchApproximation::minDistResult(SHARED_PTR<RouteSegmentResult>& res, double minDistSqr,
                                               const SHARED_PTR<RouteSegmentPoint>& pnt,
                                               const SHARED_PTR<GpxPoint>& loc) {
    int segmentEnd = -1;
    double dist = 0;
    int start = std::max(0, pnt->getSegmentStart() - LOOKUP_AHEAD);
    int end = std::min(pnt->getRoad()->getPointsLength(), pnt->getSegmentStart() + LOOKUP_AHEAD);
    for (int i = start; i < end; i++) {
        if (i == pnt->getSegmentStart()) {
            continue;
        }
        double d = squareDist31TileMetric(loc->x31, loc->y31,
                                          pnt->getRoad()->getPoint31XTile(i), pnt->getRoad()->getPoint31YTile(i));
        if (segmentEnd < 0 || d < dist) {
            segmentEnd = i;
            dist = d;
        }
    }
    dist += pnt->dist; // distToProj > 0 is only for pnt(s) after findRouteSegment

    // Sometimes, more than 1 segment from (pnt+others) to next-gpx-point might have the same distance.
    // To make difference, a small fraction (1/1000) of real-segment-distance is added as "dilution" value.
    // Such a small dilution prevents from interfering with main searching of minimal distance to gpx-point.
    // https://test.osmand.net/map/?start=52.481439,13.386036&end=52.483094,13.386060&profile=rescuetrack&params=rescuetrack,geoapproximation#18/52.48234/13.38672
    dist += sumPntDistanceSqr(pnt, pnt->getSegmentStart(), segmentEnd) * DILUTE_BY_SEGMENT_DISTANCE;

    if ((res == nullptr || dist < minDistSqr) && segmentEnd >= 0) {
        minDistSqr = dist;
        res = std::make_shared<RouteSegmentResult>(pnt->getRoad(), pnt->getSegmentStart(), segmentEnd);
    }
    return minDistSqr;
}

double GpxSimplePointsMatchApproximation::sumPntDistanceSqr(const SHARED_PTR<RouteSegmentPoint>& pnt, int start, int end) {
    if (start == end) {
        return 0;
    }
    if (start > end) {
        int swap = start;
        start = end;
        end = swap;
    }
    double dist = 0;
    for (int i = start; i < end; i++) {
        dist += squareRootDist31(
            pnt->getRoad()->getPoint31XTile(i), pnt->getRoad()->getPoint31YTile(i),
            pnt->getRoad()->getPoint31XTile(i + 1), pnt->getRoad()->getPoint31YTile(i + 1));
    }
    return dist * dist;
}

SHARED_PTR<GpxPoint> GpxSimplePointsMatchApproximation::findNextRoutablePoint(RoutePlannerFrontEnd* frontEnd,
                                                                     SHARED_PTR<GpxRouteApproximation>& gctx,
                                                                     double distThreshold,
                                                                     std::vector<SHARED_PTR<GpxPoint>>& gpxPoints,
                                                                     int searchStart) {
    for (int i = searchStart; i < gpxPoints.size(); i++) {
        if (gctx->initRoutingPoint(gpxPoints.at(i), gctx, distThreshold)) {
            return gpxPoints.at(i);
        }
    }

    return nullptr;
}

void GpxSimplePointsMatchApproximation::initGpxPointsXY31(std::vector<SHARED_PTR<GpxPoint>>& gpxPoints) {
    for (auto& p : gpxPoints) {
        p->x31 = get31TileNumberX(p->lon);
        p->y31 = get31TileNumberY(p->lat);
    }
}
