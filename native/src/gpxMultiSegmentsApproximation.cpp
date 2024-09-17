#include "gpxMultiSegmentsApproximation.h"

#include "gpxRouteApproximation.h"
#include "routePlannerFrontEnd.h"
#include "binaryRoutePlanner.h"

// GpxMultiSegmentsApproximation (Java to C++ port)

GpxMultiSegmentsApproximation::GpxMultiSegmentsApproximation(const SHARED_PTR<GpxRouteApproximation>& gctx,
                                                             const std::vector<SHARED_PTR<GpxPoint>>& gpxPoints)
    :
    gctx{gctx},
    gpxPoints{gpxPoints},
    minPointApproximation{gctx->ctx->config->minPointApproximation},
    initDist{gctx->ctx->config->minPointApproximation / 2},
    METRICS_COMPARATOR{
        [](const RouteSegmentAppr* o1, const RouteSegmentAppr* o2) {
            return o1->metric() > o2->metric();
        }
    },
    queue{METRICS_COMPARATOR} {
    garbage.reserve(gpxPoints.size()); // at minimum
}

GpxMultiSegmentsApproximation::~GpxMultiSegmentsApproximation() {
    for (RouteSegmentAppr* k : garbage) {
        delete k;
    }
}

void GpxMultiSegmentsApproximation::debugln(const std::string& line) {
    if (VERBOSE) {
        OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "%s", line.c_str());
    }
}

GpxMultiSegmentsApproximation::RouteSegmentAppr::RouteSegmentAppr(int start, const SHARED_PTR<RouteSegmentPoint>& pnt)
    :
    segment{pnt},
    parent{nullptr},
    gpxStart{start} {
}

GpxMultiSegmentsApproximation::RouteSegmentAppr::RouteSegmentAppr(const RouteSegmentAppr* parent,
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

std::string GpxMultiSegmentsApproximation::RouteSegmentAppr::toString() const {
    // return String.format("%d -> %d  ( %s ) %.2f", gpxStart, gpxNext(), segment, maxDistToGpx);
    return std::to_string(gpxStart) + " -> " + std::to_string(gpxNext()) +
        " ( " + segment->toString() + " ) " + std::to_string(std::round(maxDistToGpx * 100) / 100.0f);
}

void GpxMultiSegmentsApproximation::loadConnections(const RouteSegmentAppr* last,
                                                    std::vector<RouteSegmentAppr*>& connected) {
    connected.clear();
    if (last->parent == nullptr) {
        const SHARED_PTR<RouteSegmentPoint> pnt = std::dynamic_pointer_cast<RouteSegmentPoint>(last->segment);
        if (pnt == nullptr) {
            throw std::bad_cast();
        }
        addSegmentInternal(last, pnt, connected);
        if (!pnt->others.empty() /* pnt.others != null */) {
            for (const SHARED_PTR<RouteSegmentPoint>& o : pnt->others) {
                addSegmentInternal(last, o, connected);
            }
        }
    }
    else {
        auto sgVectorSharedRouteSegment = gctx->ctx->loadRouteSegment(last->segment->getEndPointX(),
                                                                      last->segment->getEndPointY());
        for (const auto& sg : sgVectorSharedRouteSegment) {
            addSegment(last, sg->initRouteSegment(sg, !sg->isPositive()), connected);
            addSegment(last, sg, connected);
        }
    }
}

void GpxMultiSegmentsApproximation::addSegmentInternal(const RouteSegmentAppr* last, const SHARED_PTR<RouteSegment>& sg,
                                                       std::vector<RouteSegmentAppr*>& connected) {
    bool accept = approximateSegment(last, sg, connected);
    if (!accept && VERBOSE) {
        debugln("** " + sg->toString() + " - not accepted");
    }
}

void GpxMultiSegmentsApproximation::addSegment(const RouteSegmentAppr* last, const SHARED_PTR<RouteSegment>& sg,
                                               std::vector<RouteSegmentAppr*>& connected) {
    if (sg == nullptr) {
        return;
    }

    int oneway = gctx->ctx->config->router->isOneWay(sg->getRoad());
    if ((sg->isPositive() && oneway < 0) || (!sg->isPositive() && oneway > 0)) {
        // don't allow passing wrong way
        return;
    }

    if (sg->getRoad()->getId() != last->segment->road->getId()
        || sg->getSegmentStart() != last->segment->getSegmentStart()
    ) {
        addSegmentInternal(last, sg, connected);
    }
}

bool GpxMultiSegmentsApproximation::approximateSegment(const RouteSegmentAppr* parent,
                                                       const SHARED_PTR<RouteSegment>& sg,
                                                       std::vector<RouteSegmentAppr*>& connected) {
    auto* c = allocateRouteSegmentAppr(parent, sg);
    bool added = false;
    for (int pointInd = c->gpxStart + 1; pointInd < gpxPoints.size(); pointInd++) {
        const SHARED_PTR<GpxPoint>& p = gpxPoints.at(pointInd);
        if (p->x31 == c->segment->getEndPointX() && p->y31 == c->segment->getEndPointY()) {
            c->gpxLen++;
            continue;
        }
        std::pair<int, int> pp = getProjectionPoint(p->x31, p->y31,
                                                    c->segment->getStartPointX(), c->segment->getStartPointY(),
                                                    c->segment->getEndPointX(), c->segment->getEndPointY());
        int pp_x = pp.first, pp_y = pp.second;
        bool beforeStart = (pp_x == c->segment->getStartPointX() && pp_y == c->segment->getStartPointY());
        bool farEnd = (pp_x == c->segment->getEndPointX() && pp_y == c->segment->getEndPointY());
        if (farEnd) {
            break;
        }
        double dist = squareRootDist31((int)pp_x, (int)pp_y, p->x31, p->y31);
        if (dist > minPointApproximation) {
            if (beforeStart || c->gpxLen > 0) {
                break;
            }
            return added;
        }
        if (dist > MIN_BRANCHING_DIST && dist > c->maxDistToGpx && c->gpxLen > 0) {
            auto* altShortBranch = allocateRouteSegmentAppr(parent, sg);
            altShortBranch->maxDistToGpx = c->maxDistToGpx;
            altShortBranch->gpxLen = c->gpxLen;
            added |= addConnected(parent, altShortBranch, connected);
        }
        c->maxDistToGpx = std::max(c->maxDistToGpx, dist);
        c->gpxLen++;
    }
    added |= addConnected(parent, c, connected);
    return added;
}

bool GpxMultiSegmentsApproximation::addConnected(const RouteSegmentAppr* parent, RouteSegmentAppr* c,
                                                 std::vector<RouteSegmentAppr*>& connected) {
    // Note: RouteSegmentAppr* parent is not used here (the same as in Java)
    if (isVisited(c)) {
        return false;
    }
    int pointInd = c->gpxNext();
    // calculate dist for last segment (end point is exactly in between prev gpx / next gpx)
    // because next gpx point doesn't project onto segment
    if (pointInd < gpxPoints.size()) {
        std::pair<int, int> pp = getProjectionPoint(c->segment->getEndPointX(), c->segment->getEndPointY(),
                                                    gpxPoints.at(pointInd - 1)->x31, gpxPoints.at(pointInd - 1)->y31,
                                                    gpxPoints.at(pointInd)->x31, gpxPoints.at(pointInd)->y31);
        int pp_x = pp.first, pp_y = pp.second;
        double dist = squareRootDist31((int)pp_x, (int)pp_y, c->segment->getEndPointX(), c->segment->getEndPointY());
        c->maxDistToGpx = std::max(c->maxDistToGpx, dist);
        if (dist > minPointApproximation) {
            if (VERBOSE) {
                debugln("** " + c->toString() + " - ignore " + std::to_string(dist));
            }
            return false;
        }
    }
    connected.push_back(c);
    if (VERBOSE) {
        debugln("** " + c->toString() + " - accept");
    }
    return true;
}

void GpxMultiSegmentsApproximation::visit(const RouteSegmentAppr* r) {
    visited.insert(calculateRoutePointId(r));
}

bool GpxMultiSegmentsApproximation::isVisited(const RouteSegmentAppr* r) {
    return visited.find(calculateRoutePointId(r)) != visited.end();
}

int64_t GpxMultiSegmentsApproximation::calculateRoutePointId(const RouteSegmentAppr* segm) {
    int64_t segId = 0;
    if (segm->parent != nullptr) {
        bool positive = segm->segment->isPositive();
        segId = (segm->segment->getRoad()->getId() << ROUTE_POINTS) + (segm->segment->getSegmentStart() << 1)
            + (positive ? 1 : 0);
    }
    return (segId << GPX_MAX) + (segm->gpxStart + segm->gpxLen);
}

void GpxMultiSegmentsApproximation::initGpxPointsXY31(const std::vector<SHARED_PTR<GpxPoint>>& gpxPoints) {
    for (auto& p : gpxPoints) {
        p->x31 = get31TileNumberX(p->lon);
        p->y31 = get31TileNumberY(p->lat);
    }
}

SHARED_PTR<GpxPoint> GpxMultiSegmentsApproximation::findNextRoutablePoint(int searchStart) const {
    for (int i = searchStart; i < gpxPoints.size(); i++) {
        if (initRoutingPoint(gpxPoints.at(i), initDist)) {
            return gpxPoints.at(i);
        }
    }
    return nullptr;
}

SHARED_PTR<RouteSegmentPoint> GpxMultiSegmentsApproximation::initStartPoint(const SHARED_PTR<GpxPoint>& start,
                                                                            double gpxDir,
                                                                            const SHARED_PTR<RouteSegmentPoint>& rsp) {
    double dirc = rsp->road->directionRoute(rsp->getSegmentStart(), rsp->isPositive());
    bool direct = std::abs(alignAngleDifference(gpxDir - dirc)) < M_PI / 2;
    return std::make_shared<RouteSegmentPoint>(rsp->getRoad(),
                                               direct ? rsp->getSegmentStart() : rsp->getSegmentEnd(),
                                               direct ? rsp->getSegmentEnd() : rsp->getSegmentStart(),
                                               rsp->dist); // rst.distToProj ?
}

bool GpxMultiSegmentsApproximation::initRoutingPoint(const SHARED_PTR<GpxPoint>& start, float distThreshold) const {
    if (start != nullptr && start->pnt == nullptr) {
        gctx->routePointsSearched++;
        double gpxDir = start->object->directionRoute(start->ind, true);
        SHARED_PTR<RouteSegmentPoint> rsp = findRouteSegment(start->x31, start->y31, gctx->ctx);
        if (rsp == nullptr || getDistance(get31LatitudeY(rsp->preciseY), get31LongitudeX(rsp->preciseX),
                                          start->lat, start->lon) > distThreshold) {
            return false;
        }
        start->pnt = initStartPoint(start, gpxDir, rsp);
        if (!rsp->others.empty()) {
            for (const SHARED_PTR<RouteSegmentPoint>& o : rsp->others) {
                if (getDistance(get31LatitudeY(o->preciseY), get31LongitudeX(o->preciseX),
                                start->lat, start->lon) < distThreshold) {
                    start->pnt->others.push_back(initStartPoint(start, gpxDir, o));
                }
            }
        }
        return true;
    }
    return false;
}

double GpxMultiSegmentsApproximation::gpxDist(int gpxL1, int gpxL2) const {
    return gpxPoints.at(std::min(gpxL1, static_cast<int>(gpxPoints.size()) - 1))->cumDist -
        gpxPoints.at(std::min(gpxL2, static_cast<int>(gpxPoints.size()) - 1))->cumDist;
}

GpxMultiSegmentsApproximation::RouteSegmentAppr* GpxMultiSegmentsApproximation::peakMinFromQueue(
    const RouteSegmentAppr* bestRoute, RouteSegmentAppr* bestNext) {
    while (!queue.empty() && bestNext == nullptr) {
        bestNext = queue.top();
        queue.pop();
        if ((bestRoute != nullptr && gpxDist(bestRoute->gpxNext(), bestNext->gpxNext()) >
            MAX_DEPTH_ROLLBACK)) {
            bestNext = nullptr;
        }
    }
    return bestNext;
}

void GpxMultiSegmentsApproximation::wrapupRoute(const std::vector<SHARED_PTR<GpxPoint>>& gpxPoints,
                                                const RouteSegmentAppr* bestRoute) {
    if (bestRoute->parent == nullptr) {
        return;
    }
    std::vector<SHARED_PTR<RouteSegmentResult>> res;
    int startInd = 0;
    int last = std::min(bestRoute->gpxNext(), (int)(gpxPoints.size() - 1));
    // combining segments doesn't seem to have any effect on tests
    SHARED_PTR<RouteSegmentResult> lastRes = nullptr;
    while (bestRoute != nullptr && bestRoute->parent != nullptr) {
        startInd = bestRoute->gpxStart;
        int end = bestRoute->segment->getSegmentEnd();
        if (lastRes != nullptr && bestRoute->segment->getRoad()->getId() == lastRes->object->getId()) {
            if (lastRes->getStartPointIndex() == bestRoute->segment->getSegmentEnd()
                && lastRes->isForwardDirection() == bestRoute->segment->isPositive()) {
                end = lastRes->getEndPointIndex();
                res.pop_back();
            }
        }
        SHARED_PTR<RouteSegmentResult> routeRes = std::make_shared<RouteSegmentResult>(bestRoute->segment->road,
            bestRoute->segment->getSegmentStart(), end);
        res.push_back(routeRes);
        bestRoute = bestRoute->parent;
        lastRes = routeRes;
    }
    std::reverse(res.begin(), res.end());
    if (VERBOSE) {
        debugln("ROUTE " + std::to_string(startInd) + " -> " + std::to_string(last) + ":");
        for (const SHARED_PTR<RouteSegmentResult>& r : res) {
            debugln(" " + r->toString());
        }
    }
    for (auto& r : res) {
        r->setGpxPointIndex(startInd); // required for reconstructFinalPointsFromFullRoute()
    }
    gpxPoints.at(startInd)->routeToTarget = res;
    gpxPoints.at(startInd)->targetInd = last;
}

void GpxMultiSegmentsApproximation::gpxApproximation() {
    OsmAnd::ElapsedTimer timer;
    timer.Start();

    initGpxPointsXY31(gpxPoints);

    SHARED_PTR<GpxPoint> currentPoint = findNextRoutablePoint(0);

    if (currentPoint == nullptr) {
        return;
    }

    auto* last = allocateRouteSegmentAppr(0, currentPoint->pnt);
    std::vector<RouteSegmentAppr*> connected;
    RouteSegmentAppr* bestRoute = nullptr;

    while (last->gpxNext() < gpxPoints.size()) {
        RouteSegmentAppr* bestNext = nullptr;
        if (!isVisited(last)) {
            visit(last);
            loadConnections(last, connected);
            if (!connected.empty()) {
                if (EAGER_ALGORITHM) {
                    std::sort(connected.begin(), connected.end(), METRICS_COMPARATOR);
                    bestNext = connected.at(0);
                    for (int i = 1; i < connected.size(); i++) {
                        queue.push(connected.at(i)); // queue.addAll(connected.subList(1, connected.size()));
                    }
                }
                else if (PRIORITY_ALGORITHM) {
                    for (const auto& r : connected) {
                        queue.push(r); // queue.addAll(connected);
                    }
                }
            }
        }
        bestNext = peakMinFromQueue(bestRoute, bestNext); // try to revert to MAX_DEPTH_ROLLBACK if bestNext null
        if (bestNext != nullptr) {
            if (VERBOSE) {
                int ind = bestNext->gpxStart + bestNext->gpxLen;
                std::string ll = std::to_string(gpxPoints.at(ind)->lat) + "," + std::to_string(gpxPoints.at(ind)->lon);
                debugln(bestNext->toString() + " " + ll);
            }
            if (bestRoute == nullptr || bestRoute->gpxNext() < last->gpxNext()) {
                bestRoute = last;
            }
            last = bestNext;
        }
        else {
            if (bestRoute != nullptr) {
                wrapupRoute(gpxPoints, bestRoute);
            }
            SHARED_PTR<GpxPoint> pnt = findNextRoutablePoint(
                bestRoute != nullptr ? bestRoute->gpxNext() : last->gpxNext());
            visited.clear(); // visited = new TLongHashSet();
            if (pnt == nullptr) {
                debugln("------------------");
                break;
            }
            else {
                debugln(
                    "\n!!! " + std::to_string(pnt->ind) + " " + std::to_string(pnt->lat) + "," +
                    std::to_string(pnt->lon) + " " + pnt->pnt->toString());
                last = allocateRouteSegmentAppr(pnt->ind, pnt->pnt);
                bestRoute = nullptr;
            }
        }
    }
    if (bestRoute == nullptr || bestRoute->gpxNext() < last->gpxNext()) {
        bestRoute = last; // prefer the farthest end-of-the-route
    }
    if (bestRoute != nullptr) {
        wrapupRoute(gpxPoints, bestRoute);
    }

    OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info,
                      "Native Multi-Segments Approximation took %.2f seconds (%d route points searched)",
                      static_cast<double>(timer.GetElapsedMs()) / 1000.0, gctx->routePointsSearched);
}

GpxMultiSegmentsApproximation::RouteSegmentAppr* GpxMultiSegmentsApproximation::allocateRouteSegmentAppr(
    int start, const SHARED_PTR<RouteSegmentPoint>& pnt) {
    auto* p = new RouteSegmentAppr(start, pnt);
    garbage.push_back(p);
    return p;
}

GpxMultiSegmentsApproximation::RouteSegmentAppr* GpxMultiSegmentsApproximation::allocateRouteSegmentAppr(
    const RouteSegmentAppr* parent, const SHARED_PTR<RouteSegment>& segment) {
    auto* p = new RouteSegmentAppr(parent, segment);
    garbage.push_back(p);
    return p;
}
