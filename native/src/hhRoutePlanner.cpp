#ifndef _OSMAND_HH_ROUTE_PLANNER_CPP
#define _OSMAND_HH_ROUTE_PLANNER_CPP

#include <ctime>
#include "hhRoutePlanner.h"
#include "CommonCollections.h"
#include "Logging.h"
#include "routePlannerFrontEnd.h"
#include "routeResultPreparation.h"
#include "binaryRoutePlanner.h"

const int HHRoutePlanner::PNT_SHORT_ROUTE_START_END = -1000;
const int HHRoutePlanner::MAX_POINTS_CLUSTER_ROUTING = 150000;
const int HHRoutePlanner::ROUTE_POINTS = 11;

HHRoutePlanner::HHRoutePlanner(RoutingContext * ctx) {
    std::vector<SHARED_PTR<HHRouteRegionPointsCtx>> regions;
    initNewContext(ctx, regions);
    hhRouteRegionGroup = std::make_shared<HHRouteRegionsGroup>();
}

HHRoutingConfig * HHRoutePlanner::prepareDefaultRoutingConfig(HHRoutingConfig * c) {
    if (c == nullptr) {
        c = HHRoutingConfig::astar(0);
        // test data for debug swap
        // c = HHRoutingConfig::dijkstra(0);
        // c = HHRoutingConfig::ch();
        // c->preloadSegments();
        c->ROUTE_LAST_MILE = true;
        c->calcDetailed(2);
        // c->calcAlternative();
        // c->gc();
        // c->DEBUG_VERBOSE_LEVEL = 2;
        // c->DEBUG_ALT_ROUTE_SELECTION++;
        c->ALT_EXCLUDE_RAD_MULT_IN = 1;
        c->ALT_EXCLUDE_RAD_MULT = 0.05;
        // c->INITIAL_DIRECTION = 30 / 180.0 * Math.PI;
    }
    return c;
}

SHARED_PTR<HHRoutingContext> HHRoutePlanner::initNewContext(RoutingContext * ctx, std::vector<SHARED_PTR<HHRouteRegionPointsCtx>> & regions) {
    currentCtx = std::make_shared<HHRoutingContext>();
    currentCtx->rctx = ctx;
    if (regions.size() > 0) {
        currentCtx->regions.insert(currentCtx->regions.end(), regions.begin(), regions.end());
    }
    return currentCtx;
}

SHARED_PTR<HHRoutingContext> HHRoutePlanner::selectBestRoutingFiles(int startX, int startY, int endX, int endY, const SHARED_PTR<HHRoutingContext> & hctx) {
    std::vector<SHARED_PTR<HHRouteRegionsGroup>> groups;
    SHARED_PTR<GeneralRouter> router = hctx->rctx->config->router;
    string profile = profileToString(router->getProfile()); // use base profile
    std::vector<string> ls = router->serializeParameterValues(router->getParameterValues());

    SkRect qr = SkRect::MakeLTRB(std::min(startX, endX), std::min(startY, endY), std::max(startX, endX), std::max(startY, endY));

    const std::vector<BinaryMapFile*> & openFiles = getOpenMapFiles();
    for (BinaryMapFile * r : openFiles) {
        for (SHARED_PTR<HHRouteIndex> & hhRegion : r->hhIndexes) {
            SkRect * hhRegionRect = hhRegion->getSkRect();
            if (hhRegion->profile == profile && SkRect::Intersects(qr, *hhRegionRect)) {
                double intersect = hhRegion->intersectionArea(qr);
                hhRouteRegionGroup->appendToGroups(hhRegion, r, groups, intersect);
            }
        }
    }
    for (auto & g : groups) {
        g->containsStartEnd = g->contains(startX, startY, hctx) && g->contains(endX, endY, hctx);
        vector<string> params = split_string(g->profileParams, ",");
        for (string & p : params) {
            if (trim(p).length() == 0) {
                continue;
            }
            if (std::find(ls.begin(), ls.end(), p) == ls.end()) {
                g->extraParam++;
            } else {
                g->matchParam++;
            }
        }
    }
    std::sort(groups.begin(), groups.end(), [](const SHARED_PTR<HHRouteRegionsGroup> o1, const SHARED_PTR<HHRouteRegionsGroup> o2) {
        if (o1->containsStartEnd != o2->containsStartEnd) {
            return !o1->containsStartEnd;
        } else if (o1->extraParam != o2->extraParam) {
            return o1->extraParam > o2->extraParam;
        } else if (o1->matchParam != o2->matchParam) {
            return o1->matchParam < o2->matchParam;
        }
        return o1->sumIntersects < o2->sumIntersects; // higher is better
    });
    if (groups.size() == 0) {
        return nullptr;
    }
    
    auto & bestGroup = groups[0];
    vector<SHARED_PTR<HHRouteRegionPointsCtx>> regions;
    for(short mapId = 0; mapId < bestGroup->regions.size(); mapId++) {
        auto & params = bestGroup->regions.at(mapId)->profileParams;
        auto it = find(params.begin(), params.end(), bestGroup->profileParams);
        if (it != params.end()) {
            int rProf = (int) (it - params.begin());
            SHARED_PTR<HHRouteRegionPointsCtx> reg = std::make_shared<HHRouteRegionPointsCtx>(mapId, bestGroup->regions.at(mapId), bestGroup->readers.at(mapId), rProf);
            regions.push_back(reg);
        }
    }
    
    bool allMatched = true;
    for (auto & r : regions) {
        bool match = false;
        for (auto & p : currentCtx->regions) {
            if (p->file == r->file && p->fileRegion == r->fileRegion && p->routingProfile == r->routingProfile) {
                match = true;
                break;
            }
        }
        if (!match) {
            allMatched = false;
            break;
        }
    }
    if (allMatched) {
        return currentCtx;
    }
    return initNewContext(hctx->rctx, regions);
}

MAP_VECTORS_NETWORK_DB_POINTS HHRoutePlanner::groupByClusters( UNORDERED_map<int64_t, NetworkDBPoint *> & pointsById, bool out) {
    MAP_VECTORS_NETWORK_DB_POINTS res;
    UNORDERED_map<int64_t, NetworkDBPoint *>::iterator it;
    for (it = pointsById.begin(); it != pointsById.end(); it++) {
        NetworkDBPoint * p = it->second;
        int cid = out ? p->clusterId : p->dualPoint->clusterId;
        if (res.find(cid) == res.end()) {
            std::vector<NetworkDBPoint *> v;
            res.insert(std::pair<int64_t, std::vector<NetworkDBPoint *>>(cid, v));
        }
        res.find(cid)->second.push_back(p);
    }
    
    for (auto iterator = res.begin(); iterator != res.end(); ++iterator) {
        auto & l = iterator->second;
        std::sort(l.begin(), l.end(), [](const NetworkDBPoint * lhs, const NetworkDBPoint * rhs) {
            return lhs->index < rhs->index;
        });
    }
    return res;
}

int64_t HHRoutePlanner::calculateRoutePointInternalId(int64_t id, int32_t pntId, int32_t nextPntId) const {
    int32_t positive = nextPntId - pntId;
    return (id << ROUTE_POINTS) + (pntId << 1) + (positive > 0 ? 1 : 0);
}

int64_t HHRoutePlanner::calculateRoutePointInternalId(const SHARED_PTR<RouteDataObject> & road, int32_t pntId, int32_t nextPntId) const {
    int32_t positive = nextPntId - pntId;
    int pntLen = road->getPointsLength();
    if (positive < 0) {
        OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "Check only positive segments are in calculation (hhRoutePlanner) [Native]");
        return -1;
    }
    if (pntId < 0 || nextPntId < 0 || pntId >= pntLen || nextPntId >= pntLen ||
        (positive != -1 && positive != 1) || pntLen > (1 << ROUTE_POINTS)) {
        OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "Assert failed");
        return -1;
    }
    return (road->getId() << ROUTE_POINTS) + (pntId << 1) + (positive > 0 ? 1 : 0);
}

SHARED_PTR<HHRoutingContext> HHRoutePlanner::initHCtx(HHRoutingConfig * c, int startX, int startY, int endX, int endY) {
    SHARED_PTR<HHRoutingContext> hctx = currentCtx;
    SHARED_PTR<RouteCalculationProgress> progress = hctx->rctx->progress;
    progress->hhIteration(RouteCalculationProgress::HHIteration::SELECT_REGIONS);
    hctx = selectBestRoutingFiles(startX, startY, endX, endY, hctx);
    
    if (hctx == nullptr) {
        OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "No files found for routing");
        return hctx;
    }
    if (c->STATS_VERBOSE_LEVEL > 0) {
        OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "Selected files: %s", (hctx == nullptr ? " NULL " : hctx->getRoutingInfo().c_str()));
    }
    
    hctx->stats = RoutingStats();
    hctx->config = c;
    hctx->startX = startX;
    hctx->startY = startY;
    hctx->endX = endX;
    hctx->endY = endY;

    hctx->clearVisited();
    if (hctx->initialized) {
        return hctx;
    }
            
    OsmAnd::ElapsedTimer timer;
    timer.Start();
    progress->hhIteration(RouteCalculationProgress::HHIteration::LOAD_POINTS);
    if (c->STATS_VERBOSE_LEVEL > 0) {
        OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "Loading points... ");
    }
    hctx->pointsById = hctx->loadNetworkPoints();
    hctx->stats.loadPointsTime = timer.GetElapsedMs();
    timer.Start();
    if (c->STATS_VERBOSE_LEVEL > 0) {
        OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, " %zu - %.2fms\n", hctx->pointsById.size(), hctx->stats.loadPointsTime);
    }
    UNORDERED_map<int64_t, NetworkDBPoint *>::iterator it;
    for (it = hctx->pointsById.begin(); it != hctx->pointsById.end(); it++) {
        it->second->markSegmentsNotLoaded();
    }
    hctx->clusterOutPoints = groupByClusters(hctx->pointsById, true);
    hctx->clusterInPoints  = groupByClusters(hctx->pointsById, false);
    for (it = hctx->pointsById.begin(); it != hctx->pointsById.end(); it++) {
        NetworkDBPoint * pnt = it->second;
        int64_t pos = calculateRoutePointInternalId(pnt->roadId, pnt->start, pnt->end);
        LatLon latlon = pnt->getPoint();
        hctx->pointsRect.registerObject(latlon.lat, latlon.lon, pnt);
        hctx->boundaries.insert(std::pair<int64_t, SHARED_PTR<RouteSegment>>(pos, nullptr));
        hctx->pointsByGeo.insert(std::pair<int64_t, NetworkDBPoint *>(pos, pnt));
        hctx->regions[pnt->mapId]->pntsByFileId.insert(std::pair<int64_t, NetworkDBPoint *>(pnt->fileId, pnt));
    }
    if (DEBUG_VERBOSE_LEVEL > 0) {
        hctx->pointsRect.printStatsDistribution("Points distributed");
    }
    hctx->initialized = true;
    hctx->stats.loadPointsTime = timer.GetElapsedMs();
    if (c->STATS_VERBOSE_LEVEL > 0) {
        OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, " %zu - %.2f ms\n", hctx->pointsById.size(), hctx->stats.loadPointsTime);
    }
    return hctx;
}

HHNetworkRouteRes * HHRoutePlanner::cancelledStatus() const {
    return new HHNetworkRouteRes("Routing was cancelled.");
}

HHNetworkRouteRes * HHRoutePlanner::runRouting(int startX, int startY, int endX, int endY, HHRoutingConfig * config) {
    auto & progress = currentCtx->rctx->progress;
    OsmAnd::ElapsedTimer timer;
    OsmAnd::ElapsedTimer overallTimer;
    overallTimer.Start();
    int SL = config->STATS_VERBOSE_LEVEL;
    config = prepareDefaultRoutingConfig(config);
    SHARED_PTR<HHRoutingContext> hctx = initHCtx(config, startX, startY, endX, endY);
    if (hctx == nullptr) {
        HHNetworkRouteRes * res = new HHNetworkRouteRes("Files for hh routing were not initialized. Route couldn't be calculated.");
        return res;
    }
    filterPointsBasedOnConfiguration(hctx);
    
    OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "Routing %.5f %.5f -> %.5f %.5f (HC %d, dir %d)",
                      get31LatitudeY(startY), get31LongitudeX(startX),
                      get31LatitudeY(endY), get31LongitudeX(endX),
                      (int) config->HEURISTIC_COEFFICIENT, (int) config->DIJKSTRA_DIRECTION);
    UNORDERED_map<int64_t, NetworkDBPoint *> stPoints;
    UNORDERED_map<int64_t, NetworkDBPoint *> endPoints;
    // avoid overwriting progress
    if (hctx->rctx->progress == nullptr) {
        hctx->rctx->progress = std::make_shared<RouteCalculationProgress>();
    }
    progress->hhIteration(RouteCalculationProgress::HHIteration::START_END_POINT);
    findFirstLastSegments(hctx, startX, startY, endX, endY, stPoints, endPoints);
    HHNetworkRouteRes * route = nullptr;
    bool recalc = false;
    double firstIterationTime = 0;
    int iteration = 0;
    do {
        timer.Reset();
        timer.Start();
        progress->hhIteration(RouteCalculationProgress::HHIteration::ROUTING);
        iteration++;
        if (recalc && firstIterationTime == 0) {
            if (DEBUG_VERBOSE_LEVEL > 0) {
                OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "Recalculating route due to route structure changes...");
            }
            firstIterationTime = hctx->stats.routingTime;
        }
        if (!recalc || DEBUG_VERBOSE_LEVEL > 0) {
            OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "Routing...");
        }
        NetworkDBPoint * finalPnt = runRoutingPointsToPoints(hctx, stPoints, endPoints);
        if (progress->isCancelled()) {
            return cancelledStatus();
        }
        route = createRouteSegmentFromFinalPoint(hctx, finalPnt);
        double time = timer.GetElapsedMs();
        if (!recalc || DEBUG_VERBOSE_LEVEL > 0) {
            OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "%zu segments, cost %.2f, %.2f ms\n", route->segments.size(), route->getHHRoutingTime(), time);
        }
        progress->hhIteration(RouteCalculationProgress::HHIteration::DETAILED);
        if (!recalc || DEBUG_VERBOSE_LEVEL > 0) {
            OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "Parse detailed route segments...");
        }
        recalc = retrieveSegmentsGeometry(hctx, route, hctx->config->ROUTE_ALL_SEGMENTS, progress);
        if (progress->isCancelled()) {
            return cancelledStatus();
        }
        time = timer.GetElapsedMs();
        if (firstIterationTime == 0 || DEBUG_VERBOSE_LEVEL > 0) {
            OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "%.2f ms\n", time);
        }
        hctx->stats.routingTime += time;
        if (recalc) {
            if (hctx->stats.prepTime + hctx->stats.routingTime > hctx->config->MAX_TIME_REITERATION_MS) {
                HHNetworkRouteRes * res = new HHNetworkRouteRes("Too many route recalculations (maps are outdated).");
                return res;
            }
            hctx->clearVisited(stPoints, endPoints);
            delete route;
            route = nullptr;
        }
    } while (route == nullptr);
    
    if (firstIterationTime > 0 && DEBUG_VERBOSE_LEVEL == 0 && SL > 0) {
        OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "%d iterations, %.2f ms\n", iteration, hctx->stats.routingTime - firstIterationTime);
    }
    
    double altRoutes = 0;
    if (hctx->config->CALC_ALTERNATIVES) {
        progress->hhIteration(RouteCalculationProgress::HHIteration::ALTERNATIVES);
        OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "Alternative routes...");
        timer.Start();
        calcAlternativeRoute(hctx, route, stPoints, endPoints, progress);
        hctx->stats.altRoutingTime += timer.GetElapsedMs();
        hctx->stats.routingTime += hctx->stats.altRoutingTime;
        OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "%zu %.2f ms\n", route->altRoutes.size(), hctx->stats.altRoutingTime);
        timer.Start();
        for (HHNetworkRouteRes * alt : route->altRoutes) {
            retrieveSegmentsGeometry(hctx, alt, hctx->config->ROUTE_ALL_ALT_SEGMENTS, progress);
            if (progress->isCancelled()) {
                return cancelledStatus();
            }
        }
        altRoutes = timer.GetElapsedMs();
        OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "%.2f ms\n", altRoutes);
    }
    
    if (hctx->config->USE_GC_MORE_OFTEN) {
        hctx->unloadAllConnections();
    }
    
    timer.Start();
    OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info,"Prepare results (turns, alt routes)...");
    prepareRouteResults(hctx, route, startX, startY, endX, endY);
    hctx->stats.prepTime += timer.GetElapsedMs();
    OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "prepTime %.2f ms\n", hctx->stats.prepTime);
    OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info,
                      "Found final route - cost %.2f (detailed %.2f, %.1f%%), %zu depth ( first met %d, visited %d (%d unique) of %d added vertices )",
                      route->getHHRoutingTime(), route->getHHRoutingDetailed(), 100 * (1 - route->getHHRoutingDetailed() / route->getHHRoutingTime()),
                      route->segments.size(), hctx->stats.firstRouteVisitedVertices, hctx->stats.visitedVertices, hctx->stats.uniqueVisitedVertices,
                      hctx->stats.addedVertices);
    timer.Start();
    OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "Routing %.5f %.5f -> %.5f %.5f (HC %d, dir %d)",
                      get31LatitudeY(startY), get31LongitudeX(startX),
                      get31LatitudeY(endY), get31LongitudeX(endX),
                      (int) hctx->config->HEURISTIC_COEFFICIENT, (int) hctx->config->DIJKSTRA_DIRECTION);
    hctx->stats.prepTime += altRoutes;
    
    if (progress->isCancelled()) {
        return cancelledStatus();
    }

    OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "%llu ms\n", timer.GetElapsedMs());
    printResults(hctx->rctx, startX, startY, endX, endY, route->detailed);
    OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info,
                      "Routing finished all %llu ms: last mile %.f ms, load data %.f ms (%d edges), routing %.f ms (queue - %.f add ms + %.f poll ms), prep result %.f ms\n",
                      overallTimer.GetElapsedMs(), hctx->stats.searchPointsTime, (hctx->stats.loadEdgesTime + hctx->stats.loadPointsTime),
                      hctx->stats.loadEdgesCnt, hctx->stats.routingTime, hctx->stats.addQueueTime, hctx->stats.pollQueueTime, hctx->stats.prepTime);
    return route;
}

HHNetworkRouteRes * HHRoutePlanner::prepareRouteResults(const SHARED_PTR<HHRoutingContext> & hctx, HHNetworkRouteRes * route, int startX, int startY, int endX, int endY) {
    route->stats = hctx->stats;
    SHARED_PTR<RouteSegmentResult> straightLine = nullptr;
    for(int routeSegmentInd = 0; routeSegmentInd < route->segments.size(); routeSegmentInd++ ) {
        HHNetworkSegmentRes & routeSegment = route->segments[routeSegmentInd];
        NetworkDBSegment * s = routeSegment.segment;
        if (routeSegment.list.size() > 0) {
            if (straightLine != nullptr) {
                route->detailed.push_back(straightLine);
                straightLine = nullptr;
            }
            if (routeSegmentInd > 0) {
                SHARED_PTR<RouteSegmentResult> p = routeSegment.list.at(0);
                if (abs(p->getStartPointIndex() - p->getEndPointIndex()) <= 1) {
                    routeSegment.list.erase(routeSegment.list.begin() + 0);
                } else {
                    p->setStartPointIndex(p->getStartPointIndex() + (p->isForwardDirection() ? +1 : -1));
                }
            }
            route->detailed.insert(route->detailed.end(), routeSegment.list.begin(), routeSegment.list.end());
        } else {
            auto reg = std::make_shared<RoutingIndex>();
            reg->initRouteEncodingRule(0, "highway", "unmatched");
            auto rdo = make_shared<RouteDataObject>(reg);
            rdo->types = { 0 };
            rdo->pointsX = { s->start->startX, s->end->startX };
            rdo->pointsY = { s->start->startY, s->end->startY };
            auto sh = make_shared<RouteDataObject>(reg);
            sh->types = { 0 };
            sh->pointsX = { s->end->startX, s->end->endX };
            sh->pointsY = { s->end->startY, s->end->endY };
            straightLine = make_shared<RouteSegmentResult>(sh, 0, 1);
            route->detailed.push_back(make_shared<RouteSegmentResult>(rdo, 0, 1));
        }
        
        hctx->rctx->progress->routingCalculatedTime += routeSegment.rtTimeDetailed;

        if (DEBUG_VERBOSE_LEVEL >= 1) {
            int segments = (int) routeSegment.list.size();
            if (s == nullptr) {
                if (hctx->config->STATS_VERBOSE_LEVEL > 0) {
                    OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info,
                                      "First / last segment - %d segments, %.2fs \n",
                                      segments, routeSegment.rtTimeDetailed);
                }
            } else {
                if (hctx->config->STATS_VERBOSE_LEVEL > 0) {
                    OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info,
                                      "\nRoute %lld [%d] -> %lld [%d] %s - hh dist %.2f s, detail %.2f s (%.1f%%) segments %d ( end %.5f/%.5f - %lld ) ",
                                      s->start->index, s->start->chInd(), s->end->index, s->end->chInd(), s->shortcut ? "sh" : "bs",
                                      s->dist, routeSegment.rtTimeDetailed, 100 * (1 - routeSegment.rtTimeDetailed / s->dist),
                                      segments, get31LatitudeY(s->end->startY), get31LongitudeX(s->end->startX), s->end->roadId / 64);
                }
            }
        }
    }
    return route;
}

void HHRoutePlanner::findFirstLastSegments(const SHARED_PTR<HHRoutingContext> & hctx, int startX, int startY, int endX, int endY,
                                           UNORDERED_map<int64_t, NetworkDBPoint *> & stPoints,
                                           UNORDERED_map<int64_t, NetworkDBPoint *> & endPoints) {
    OsmAnd::ElapsedTimer timer;
    timer.Start();
    if (hctx->config->STATS_VERBOSE_LEVEL > 0) {
        OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "Finding first / last segments...");
    }
    // auto planner = std::shared_ptr<RoutePlannerFrontEnd>();
    int startReiterate = -1, endReiterate = -1;
    bool found = false;
    hctx->rctx->progress->hhIterationProgress(0.00); // %
    SHARED_PTR<RouteSegmentPoint> startPnt = findRouteSegment(startX, startY, hctx->rctx, false);
    if (startPnt == nullptr) {
        OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Warning, "Start point was not found (hhRoutePlanner) [Native]");
        return;
    }
    hctx->rctx->progress->hhIterationProgress(0.25); // %
    SHARED_PTR<RouteSegmentPoint> endPnt = findRouteSegment(endX, endY, hctx->rctx, false);
    if (endPnt == nullptr) {
        OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Warning, "End point was not found (hhRoutePlanner) [Native]");
        return;
    }
    vector<SHARED_PTR<RouteSegmentPoint>> stOthers = startPnt->others;//copy
    vector<SHARED_PTR<RouteSegmentPoint>> endOthers = endPnt->others;
    do {
        if (startReiterate + endReiterate >= hctx->config->MAX_START_END_REITERATIONS) {
            break;
        }
        UNORDERED_map<int64_t, NetworkDBPoint *>::iterator it;
        for (it = stPoints.begin(); it != stPoints.end(); it++) {
            it->second->clearRouting();
        }
        stPoints.clear();
        for (it = endPoints.begin(); it != endPoints.end(); it++) {
            it->second->clearRouting();
        }
        endPoints.clear();
        auto startP = startPnt;//copy
        if (startReiterate >= 0) {
            if (startReiterate < stOthers.size()) {
                startP = stOthers.at(startReiterate);
            } else {
                break;
            }
        }
        auto endP = endPnt;//copy
        if (endReiterate >= 0) {
            //Java if (endOthers != null ...)
            if (endReiterate < endOthers.size()) {
                endP = endOthers.at(endReiterate);
            } else {
                break;
            }
        }
        double prev = hctx->rctx->config->initialDirection;
        hctx->rctx->config->initialDirection = hctx->config->INITIAL_DIRECTION;
        hctx->boundaries.insert(std::pair<int64_t, SHARED_PTR<RouteSegment>>(calcRPId(endP, endP->getSegmentEnd(), endP->getSegmentStart()), nullptr));
        hctx->boundaries.insert(std::pair<int64_t, SHARED_PTR<RouteSegment>>(calcRPId(endP, endP->getSegmentStart(), endP->getSegmentEnd()), nullptr));
        hctx->rctx->progress->hhIterationProgress(0.50); // %
        initStart(hctx, startP, false, stPoints);
        hctx->rctx->config->initialDirection = prev;
        if (stPoints.empty()) {
            LatLon l = startP->getPreciseLatLon();
            auto & r = startP->road;
            if (hctx->config->STATS_VERBOSE_LEVEL > 0) {
                OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "Reiterate with next start point: %d (%.5f %.5f): %lld %s",
                                  startP->segmentStart, l.lat, l.lon, r->getId() / 64, r->getName().c_str());
            }
            startReiterate++;
            found = false;
            continue;
        }
        hctx->boundaries.erase(calcRPId(endP, endP->getSegmentEnd(), endP->getSegmentStart()));
        hctx->boundaries.erase(calcRPId(endP, endP->getSegmentStart(), endP->getSegmentEnd()));
        it = stPoints.find(PNT_SHORT_ROUTE_START_END);
        if (it != stPoints.end()) {
            endPoints.insert(std::pair<int64_t, NetworkDBPoint *>(PNT_SHORT_ROUTE_START_END, it->second));
        }
        hctx->rctx->progress->hhIterationProgress(0.75); // %
        initStart(hctx, endP, true, endPoints);
        if (endPoints.empty()) {
            LatLon l = endP->getPreciseLatLon();
            auto & r = endP->road;
            if (hctx->config->STATS_VERBOSE_LEVEL > 0) {
                OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "Reiterate with next end point: %d (%.5f %.5f): %lld %s",
                                  endP->segmentStart, l.lat, l.lon, r->getId() / 64, r->getName().c_str());
            }
            endReiterate++;
            found = false;
            continue;
        }
        found = true;
    } while (!found);
    hctx->stats.searchPointsTime = timer.GetElapsedMs();
    if (hctx->config->STATS_VERBOSE_LEVEL > 0) {
        OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "Finding first (%zu) / last (%zu) segments...%.2f ms\n", stPoints.size(), endPoints.size(), hctx->stats.searchPointsTime);
    }
    timer.Disable();
}

int64_t HHRoutePlanner::calcRPId(const SHARED_PTR<RouteSegmentPoint> & p, int pntId, int nextPntId) const {
    return calculateRoutePointInternalId(p->getRoad()->getId(), pntId, nextPntId);
}

int64_t HHRoutePlanner::calcRPId(const SHARED_PTR<RouteSegment> & p, int pntId, int nextPntId) const {
    return calculateRoutePointInternalId(p->getRoad()->getId(), pntId, nextPntId);
}

UNORDERED_map<int64_t, NetworkDBPoint *> HHRoutePlanner::initStart(const SHARED_PTR<HHRoutingContext> & hctx, const SHARED_PTR<RouteSegmentPoint> & s,
                                                                   bool reverse, UNORDERED_map<int64_t, NetworkDBPoint *> & pnts) {
    if (!hctx->config->ROUTE_LAST_MILE) {
        // simple method to calculate without detailed maps
        double startLat = get31LatitudeY(!reverse? hctx->startY : hctx->endY);
        double startLon = get31LongitudeX(!reverse? hctx->startX : hctx->endX);
        double rad = 10000;
        float spd = hctx->rctx->config->router->getMinSpeed();
        while (rad < 300000 && pnts.empty()) {
            rad = rad * 2;
            std::vector<NetworkDBPoint *> pntSelect = hctx->pointsRect.getClosestObjects(startLat, startLon, rad);
            // limit by cluster
            int cid = pntSelect.at(0)->clusterId;
            for (auto & pSelect : pntSelect) {
                if (pSelect->clusterId != cid) {
                    continue;
                }
                auto & pnt = reverse ? pSelect->dualPoint : pSelect;
                auto l = pnt->getPoint();
                double cost = getDistance(l.lat, l.lon, startLat, startLon) / spd;
                pnt->setCostParentRt(reverse, cost + hctx->distanceToEnd(reverse, pnt), nullptr, cost);
                pnts.insert(std::pair<int64_t, NetworkDBPoint *>(pnt->index, pnt));
            }
        }
        return pnts;
    }
    
    if (s == nullptr) {
        return pnts;
    }
    
    int64_t uniDirRouteIntId = calcUniDirRoutePointInternalId(s);
    if (uniDirRouteIntId == -1) {
        return pnts;
    }
    auto findIt = hctx->pointsByGeo.find(uniDirRouteIntId);
    if (findIt != hctx->pointsByGeo.end()) {
        NetworkDBPoint * finitePnt = findIt->second;
        // start / end point is directly on a network point
        double plusCost = 0, negCost = 0;
        if (hctx->rctx->config->initialDirection != NO_DIRECTION) {
            double diff = s->getRoad()->directionRoute(s->getSegmentStart(), s->isPositive()) - hctx->rctx->config->initialDirection;
            if (std::abs(alignAngleDifference(diff - M_PI)) <= M_PI / 3) {
                plusCost += hctx->rctx->config->penaltyForReverseDirection;
            }
            diff = s->getRoad()->directionRoute(s->getSegmentEnd(), !s->isPositive()) - hctx->rctx->config->initialDirection;
            if (std::abs(alignAngleDifference(diff - M_PI)) <= M_PI / 3) {
                negCost += hctx->rctx->config->penaltyForReverseDirection;
            }
        }
        finitePnt->setDistanceToEnd(reverse, hctx->distanceToEnd(reverse, finitePnt));
        finitePnt->setCostParentRt(reverse, plusCost, nullptr, plusCost);
        pnts.insert(std::pair<int64_t, NetworkDBPoint *>(finitePnt->index, finitePnt));
        NetworkDBPoint * dualPoint =  finitePnt->dualPoint;
        dualPoint->setDistanceToEnd(reverse, hctx->distanceToEnd(reverse, dualPoint));
        dualPoint->setCostParentRt(reverse, negCost, nullptr, negCost);
        pnts.insert(std::pair<int64_t, NetworkDBPoint *>(dualPoint->index, dualPoint));
        return pnts;
    }
    hctx->rctx->config->MAX_VISITED = MAX_POINTS_CLUSTER_ROUTING;
    hctx->rctx->config->planRoadDirection = reverse ? -1 : 1;
    hctx->rctx->config->heurCoefficient = 0; // dijkstra
    hctx->rctx->unloadAllData(); // needed for proper multidijsktra work
    std::vector<SHARED_PTR<RouteSegment>> frs = searchRouteInternal(hctx->rctx, reverse ? nullptr : s, reverse ? s : nullptr, hctx->boundaries, {});
    hctx->rctx->config->MAX_VISITED = -1;
    if (!frs.empty()) {
        std::set<int64_t> set;
        for (auto & o : frs) {
            // duplicates are possible as alternative routes
            int64_t pntId = calculateRoutePointInternalId(o->getRoad()->getId(),
                            reverse ? o->getSegmentEnd() : o->getSegmentStart(),
                            reverse ? o->getSegmentStart() : o->getSegmentEnd());
            if (set.insert(pntId).second) {
                NetworkDBPoint * pnt;
                auto it = hctx->pointsByGeo.find(pntId);
                if (it == hctx->pointsByGeo.end()) {
                    it = pnts.find(PNT_SHORT_ROUTE_START_END);
                    if (it != pnts.end()) {
                        continue;
                    }
                    pnt = hctx->createNetworkDBPoint();
                    pnt->index = PNT_SHORT_ROUTE_START_END;
                    pnt->roadId = o->getRoad()->getId();
                    pnt->start = o->getSegmentStart();
                    pnt->end = o->getSegmentEnd();
                    pnt->startX = o->getStartPointX();
                    pnt->endX = o->getEndPointX();
                    pnt->startY = o->getStartPointY();
                    pnt->endY = o->getEndPointY();
                    int preciseY = reverse? hctx->startY : hctx->endY;
                    int preciseX = reverse? hctx->startX : hctx->endX;
                    o->distanceFromStart += calculatePreciseStartTime(hctx->rctx, preciseX, preciseY, o);
                } else {
                    pnt = it->second;
                    o->distanceFromStart += calcRoutingSegmentTimeOnlyDist(hctx->rctx->config->router, o) / 2;
                }
                if (pnt->rt(reverse)->rtCost != 0) {
                    OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "Illegal state exception (hhRoutePlanner) [Native]");
                    return pnts;
                }
                pnt->setDistanceToEnd(reverse, hctx->distanceToEnd(reverse, pnt));
                pnt->setDetailedParentRt(reverse, o);
                pnts.insert(std::pair<int64_t, NetworkDBPoint *>(pnt->index, pnt));
            }
        }
    }
    if (hctx->config->USE_GC_MORE_OFTEN) {
        hctx->rctx->unloadAllData();
    }
    return pnts;
}

int64_t HHRoutePlanner::calcUniDirRoutePointInternalId(const SHARED_PTR<RouteSegmentPoint> & segm) const {
    if (segm->getSegmentStart() < segm->getSegmentEnd()) {
        return calculateRoutePointInternalId(segm->getRoad(), segm->getSegmentStart(), segm->getSegmentEnd());
    } else {
        return calculateRoutePointInternalId(segm->getRoad(), segm->getSegmentEnd(), segm->getSegmentStart());
    }
}

HHNetworkRouteRes * HHRoutePlanner::createRouteSegmentFromFinalPoint(const SHARED_PTR<HHRoutingContext> & hctx, NetworkDBPoint * pnt) {
    HHNetworkRouteRes * route = new HHNetworkRouteRes();
    if (pnt != nullptr) {
        NetworkDBPoint * itPnt = pnt;
        route->uniquePoints.insert(itPnt->index);
        while (itPnt->rt(true)->rtRouteToPoint != nullptr) {
            NetworkDBPoint * nextPnt = itPnt->rt(true)->rtRouteToPoint;
            NetworkDBSegment * segment = nextPnt->getSegment(itPnt, false);
            HHNetworkSegmentRes res(segment);
            res.rtTimeDetailed = res.rtTimeHHSegments = segment->dist;
            route->segments.push_back(res);
            itPnt = nextPnt;
            route->uniquePoints.insert(itPnt->index);
        }
        
        if (itPnt->rt(true)->rtDetailedRoute != nullptr) {
            HHNetworkSegmentRes res(nullptr);
            res.list = convertFinalSegmentToResults(hctx->rctx, itPnt->rt(true)->rtDetailedRoute);
            res.rtTimeDetailed = res.rtTimeHHSegments = itPnt->rt(true)->rtDetailedRoute->distanceFromStart;
            route->segments.push_back(res);
        }
        std::reverse(route->segments.begin(), route->segments.end());
        itPnt = pnt;
        while (itPnt->rt(false)->rtRouteToPoint != nullptr) {
            NetworkDBPoint * nextPnt = itPnt->rt(false)->rtRouteToPoint;
            NetworkDBSegment * segment = nextPnt->getSegment(itPnt, true);
            HHNetworkSegmentRes res(segment);
            res.rtTimeDetailed = res.rtTimeHHSegments = segment->dist;
            route->segments.push_back(res);
            itPnt = nextPnt;
            route->uniquePoints.insert(itPnt->index);
        }
        if (itPnt->rt(false)->rtDetailedRoute != nullptr) {
            HHNetworkSegmentRes res(nullptr);
            res.list = convertFinalSegmentToResults(hctx->rctx, itPnt->rt(false)->rtDetailedRoute);
            res.rtTimeDetailed = res.rtTimeHHSegments = itPnt->rt(false)->rtDetailedRoute->distanceFromStart;
            route->segments.push_back(res);
        }
        std::reverse(route->segments.begin(), route->segments.end());
    }
    return route;
}

void HHRoutePlanner::recalculateNetworkCluster(const SHARED_PTR<HHRoutingContext> & hctx, NetworkDBPoint * start) {
    hctx->rctx->config->planRoadDirection = 1;
    hctx->rctx->config->heurCoefficient = 0;
    // SPEEDUP: Speed up by just clearing visited
    hctx->rctx->unloadAllData(); // needed for proper multidijsktra work
    SHARED_PTR<RouteSegmentPoint> s = loadPoint(hctx->rctx, start);
    //hctx->rctx->progress = std::make_shared<RouteCalculationProgress>(); // we should reuse same progress for cancellation
    hctx->rctx->config->MAX_VISITED = MAX_POINTS_CLUSTER_ROUTING * 2;
    int64_t ps = calcRPId(s, s->getSegmentStart(), s->getSegmentEnd());
    int64_t ps2 = calcRPId(s, s->getSegmentEnd(), s->getSegmentStart());
    std::vector<int64_t> excludedKeys = {ps, ps2};
    auto frs = searchRouteInternal(hctx->rctx, s, nullptr, hctx->boundaries, excludedKeys);
    hctx->rctx->config->MAX_VISITED = -1;
    UNORDERED_map<int64_t, SHARED_PTR<RouteSegment>> resUnique;
    if (frs.size() > 0) {
        for (auto & o : frs) {
            int64_t pntId = calculateRoutePointInternalId(o->getRoad()->getId(), o->getSegmentStart(), o->getSegmentEnd());
            auto it = resUnique.find(pntId);
            if (it != resUnique.end()) {
                if (it->second->getDistanceFromStart() > o->getDistanceFromStart()) {
                    //TODO write expand log
                    //OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, resUnique.get(pntId) + " > " + o + " - " + s);
                }
            } else {
                resUnique.insert(std::pair<int64_t, SHARED_PTR<RouteSegment>>(pntId, o));
                auto it = hctx->pointsByGeo.find(calcRPId(o, o->getSegmentStart(), o->getSegmentEnd()));
                //NetworkDBPoint * p = hctx->pointsByGeo.get();
                if (it == hctx->pointsByGeo.end()) {
                    OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "Error calculations new final boundary not found");
                    continue;
                }
                NetworkDBPoint * p = it->second;
                float routeTime = o->getDistanceFromStart() + calcRoutingSegmentTimeOnlyDist(hctx->rctx->config->router, o) / 2 + 1;
                NetworkDBSegment * c = start->getSegment(p, true);
                if (c != nullptr) {
                    c->dist = routeTime;
                } else {
                    start->connected.push_back(hctx->createNetworkDBSegment(start, p, routeTime, true, false));
                }
                NetworkDBSegment * co = p->getSegment(start, false);
                if (co != nullptr) {
                    co->dist = routeTime;
                } else {
                    p->connectedReverse.push_back(hctx->createNetworkDBSegment(start, p, routeTime, false, false));
                }
            }
        }
    }
    
    for (NetworkDBSegment * c : start->connected) {
        auto it = resUnique.find(calculateRoutePointInternalId(c->end->roadId, c->end->start, c->end->end));
        if (it == resUnique.end()) {
            c->dist = -1; // disable as not found
            NetworkDBSegment * co = c->end->getSegment(start, false);
            if (co != nullptr) {
                co->dist = -1;
            }
        }
    }
}


bool HHRoutePlanner::retrieveSegmentsGeometry(const SHARED_PTR<HHRoutingContext> & hctx, HHNetworkRouteRes * route,
                                              bool routeSegments, SHARED_PTR<RouteCalculationProgress> progress) {
    for (int i = 0; i < route->segments.size(); i++) {
        progress->hhIterationProgress((double) i / route->segments.size());
        HHNetworkSegmentRes & s = route->segments[i];
        if (s.segment == nullptr) {
            // start / end points
            if(i > 0 && i < route->segments.size() -1 ) {
                OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "Segment ind %d is null.", i);
                return false;
            }
            continue;
        }
        
        if (routeSegments) {
            if (progress->isCancelled()) {
                return false;
            }
            std::vector<SHARED_PTR<RouteSegment>> f = runDetailedRouting(hctx, s.segment->start, s.segment->end, true);
            if (progress->isCancelled()) {
                OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Warning, "runDetailedRouting() f.size()=%zu (cancel)", f.size());
                return false;
            }
            if (f.size() == 0) {
                bool full = hctx->config->FULL_DIJKSTRA_NETWORK_RECALC-- > 0;
                OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info,
                                  "Route not found (%srecalc) %lld -> %lld",
                                  full ? "dijkstra+" : "", s.segment->start->index, s.segment->end->index);
                if (full) {
                    recalculateNetworkCluster(hctx, s.segment->start);
                }
                s.segment->dist = -1;
                return true;
            }
            if (f.size() > 1) {
                OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "More than one final route segment (hhRoutePlanner) [Native]");
                return false;
            }
            float distanceFromStart = f.at(0)->distanceFromStart;
            if ((distanceFromStart + MAX_INC_COST_CORR) > (s.segment->dist + MAX_INC_COST_CORR) * hctx->config->MAX_INC_COST_CF) {
                if (DEBUG_VERBOSE_LEVEL > 0) {
                    OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info,
                                      "Route cost increased (%.2f > %.2f) between %lld -> %lld: recalculate route\n",
                                      distanceFromStart, s.segment->dist, s.segment->start->index, s.segment->end->index);
                }
                s.segment->dist = distanceFromStart;
                return true;
            }
            s.rtTimeDetailed = distanceFromStart;
            s.list = convertFinalSegmentToResults(hctx->rctx, f.at(0));
        }
    }
    return false;
}

SHARED_PTR<RouteSegmentPoint> HHRoutePlanner::loadPoint(RoutingContext * ctx, const NetworkDBPoint * pnt) {
    std::vector<SHARED_PTR<RouteSegment>> segments = ctx->loadRouteSegment(pnt->startX, pnt->startY);
    SHARED_PTR<RouteSegment> seg = nullptr;
    for (auto & s : segments) {
        if (s) {
            seg = s;
            if (s->getRoad()->getId() == pnt->roadId && s->getSegmentStart() == pnt->start) {
                if (s->getSegmentEnd() != pnt->end) {
                    seg = s->initRouteSegment(s, !s->isPositive());
                }
                break;
            }
        }
    }
    if (seg == nullptr || seg->getSegmentStart() != pnt->start || seg->getSegmentEnd() != pnt->end || seg->getRoad()->getId() != pnt->roadId) {
        // throw new IllegalStateException("Error on segment " + pnt.roadId / 64);
        return nullptr;
    }
    SHARED_PTR<RouteSegmentPoint> rsp = std::make_shared<RouteSegmentPoint>(seg->getRoad(), seg->getSegmentStart(), seg->getSegmentEnd(), 0);
    return rsp;
}

std::vector<SHARED_PTR<RouteSegment>> HHRoutePlanner::runDetailedRouting(const SHARED_PTR<HHRoutingContext> & hctx, const NetworkDBPoint * startS, const NetworkDBPoint * endS, bool useBoundaries) {
    std::vector<SHARED_PTR<RouteSegment>> f;
    hctx->rctx->config->planRoadDirection = 0; // A* bidirectional
    hctx->rctx->config->heurCoefficient = 1;
    // SPEEDUP: Speed up by just clearing visited
    hctx->rctx->unloadAllData(); // needed for proper multidijsktra work
    SHARED_PTR<RouteSegmentPoint> start = loadPoint(hctx->rctx, startS);
    SHARED_PTR<RouteSegmentPoint> end = loadPoint(hctx->rctx, endS);
    if (start == nullptr) {
        return f; // no logging it's same as end of previos segment
    } else if (end == nullptr) {
        OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info,
                          "End point is not present in detailed maps: Point %lld (%lld %d-%d)",
                          endS->index, endS->roadId/64, endS->start, endS->end);
        return f;
    }
    double oldP = hctx->rctx->config->penaltyForReverseDirection;
    hctx->rctx->config->penaltyForReverseDirection *= 4;
    hctx->rctx->config->initialDirection = start->getRoad()->directionRoute(start->getSegmentStart(), start->isPositive());
    hctx->rctx->config->targetDirection = end->getRoad()->directionRoute(end->getSegmentEnd(), !end->isPositive());
    hctx->rctx->config->MAX_VISITED = useBoundaries ? -1 : MAX_POINTS_CLUSTER_ROUTING * 2;
    // boundaries help to reduce max visited (helpful for long ferries)
    if (useBoundaries) {
        int64_t ps = calcRPId(start, start->getSegmentEnd(), start->getSegmentStart());
        int64_t pe = calcRPId(end, end->getSegmentStart(), end->getSegmentEnd());
        std::vector<int64_t> excludedKeys = {ps, pe};
        f = searchRouteInternal(hctx->rctx, start, end, hctx->boundaries, excludedKeys);
    } else {
        f = searchRouteInternal(hctx->rctx, start, end, {}, {});
    }
    if (f.size() == 0) {
        OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info,
                          "No route found between %d %.5f %.5f Road (%d) name ('%s') -> %d %.5f %.5f Road (%d) name ('%s')",
                          (int) start->segmentStart, (float) get31LatitudeY(start->preciseY), (float) get31LongitudeX(start->preciseX),
                          (int) (start->getRoad()->getId() / 64), start->getRoad()->getName().c_str(),
                          (int) end->segmentStart, (float) get31LatitudeY(end->preciseY), (float) get31LongitudeX(end->preciseX),
                          (int) (end->getRoad()->getId() / 64), end->getRoad()->getName().c_str());
    }
    hctx->rctx->config->MAX_VISITED = -1;
    // clean up
    hctx->rctx->config->initialDirection = NO_DIRECTION;
    hctx->rctx->config->targetDirection = NO_DIRECTION;
    hctx->rctx->config->penaltyForReverseDirection = oldP;
    return f;
}

NetworkDBPoint * HHRoutePlanner::scanFinalPoint(NetworkDBPoint * finalPoint, std::vector<NetworkDBPoint *> & lt) {
    for (NetworkDBPoint * p : lt) {
        if (p->rt(true)->rtDistanceFromStart == 0 || p->rt(false)->rtDistanceFromStart == 0) {
            continue;
        }
        if (p->rt(true)->rtDistanceFromStart + p->rt(false)->rtDistanceFromStart <
            finalPoint->rt(true)->rtDistanceFromStart + finalPoint->rt(false)->rtDistanceFromStart) {
            finalPoint = p;
        }
    }
    return finalPoint;
}

double HHRoutePlanner::smallestSegmentCost(const SHARED_PTR<HHRoutingContext> & hctx, NetworkDBPoint * st, NetworkDBPoint * end) const {
    double dist = squareRootDist31(st->midX(), st->midY(), end->midX(), end->midY());
    return dist / hctx->rctx->config->router->getMaxSpeed();
}

void HHRoutePlanner::addPointToQueue(const SHARED_PTR<HHRoutingContext> & hctx, SHARED_PTR<HH_QUEUE> queue, bool reverse,
                                     NetworkDBPoint * point, NetworkDBPoint * parent, double segmentDist, double cost) {
    OsmAnd::ElapsedTimer timer;
    timer.Start();
    if (DEBUG_VERBOSE_LEVEL > 2) {
        OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info,
                          "Add  Point %lld to visit - cost %.2f (%.2f prev, %.2f dist) > prev cost %.2f \n", point->index,
                          cost, parent == nullptr ? 0 : parent->rt(reverse)->rtDistanceFromStart, segmentDist, point->rt(reverse)->rtCost);
    }
    if (point->rt(reverse)->rtVisited) {
        OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error,
                          "Point %lld (%lld %d-%d) visited - cost %.2f > prev cost %.2f",
                          point->index, point->roadId / 64, point->start, point->end, cost, point->rt(reverse)->rtCost);
        return;
    }
    point->setCostParentRt(reverse, cost, parent, segmentDist);
    hctx->queueAdded.push_back(point);
    //OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "Point added:%d %.2f %s", point->index, cost, reverse ? "rev" : "pos");
    queue->push(std::make_shared<NetworkDBPointCost>(point, cost, reverse)); // we need to add new object to not  remove / rebalance priority queue
    hctx->stats.addQueueTime += timer.GetElapsedMs();
    hctx->stats.addedVertices++;
    timer.Disable();
}

void HHRoutePlanner::addConnectedToQueue(const SHARED_PTR<HHRoutingContext> & hctx, SHARED_PTR<HH_QUEUE> & queue, NetworkDBPoint * point, bool reverse) {
    int depth = hctx->config->USE_MIDPOINT || hctx->config->MAX_DEPTH > 0 ? point->rt(reverse)->getDepth(reverse) : 0;
    if (hctx->config->MAX_DEPTH > 0 && depth >= hctx->config->MAX_DEPTH) {
        return;
    }
    OsmAnd::ElapsedTimer timer;
    timer.Start();
    int cnt = hctx->loadNetworkSegmentPoint(point, reverse);
    hctx->stats.loadEdgesCnt += cnt;
    hctx->stats.loadEdgesTime += timer.GetElapsedMs();
    for (NetworkDBSegment * connected : point->conn(reverse)) {
        NetworkDBPoint * nextPoint = reverse ? connected->start : connected->end;
        if (!hctx->config->USE_CH && !hctx->config->USE_CH_SHORTCUTS && connected->shortcut) {
            continue;
        }
        if (nextPoint->rtExclude) {
            continue;
        }
        // modify CH to not compute all top points
        if (hctx->config->USE_CH && (nextPoint->chInd() > 0 && nextPoint->chInd() < point->chInd())) {
            continue;
        }
        if (hctx->config->USE_MIDPOINT && std::min(depth, hctx->config->MIDPOINT_MAX_DEPTH) > nextPoint->midPntDepth() + hctx->config->MIDPOINT_ERROR) {
            continue;
        }
        if (connected->dist < 0) {
            // disabled segment
            continue;
        }
        if (ASSERT_AND_CORRECT_DIST_SMALLER && hctx->config->HEURISTIC_COEFFICIENT > 0
            && smallestSegmentCost(hctx, point, nextPoint) - connected->dist >  1) {
            double sSegmentCost = smallestSegmentCost(hctx, point, nextPoint);
            OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info,
                              "Incorrect distance Point %lld -> Point %lld: db = %.2f > fastest %.2f",
                              point->index, nextPoint->index, connected->dist, sSegmentCost);
            connected->dist = sSegmentCost;
        }
        double cost = point->rt(reverse)->rtDistanceFromStart  + connected->dist + hctx->distanceToEnd(reverse, nextPoint);
        if (ASSERT_COST_INCREASING && point->rt(reverse)->rtCost - cost > 1) {
            OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error,
                              "Point %lld (%lld %d-%d) (cost %.2f) -> %lld (cost %.2f) st=%.2f-> + %.2f, toend=%.2f->%.2f: ",
                              point->index, point->roadId / 64, point->start, point->end, point->rt(reverse)->rtCost,
                              nextPoint->index, cost, point->rt(reverse)->rtDistanceFromStart, connected->dist,
                              point->rt(reverse)->rtDistanceToEnd, hctx->distanceToEnd(reverse, nextPoint));
            return;
        }
        double exCost = nextPoint->rt(reverse)->rtCost;
        if ((exCost == 0 && !nextPoint->rt(reverse)->rtVisited) || cost < exCost) {
            addPointToQueue(hctx, queue, reverse, nextPoint, point, connected->dist, cost);
        }
    }
}

NetworkDBPoint * HHRoutePlanner::runRoutingPointsToPoints(const SHARED_PTR<HHRoutingContext> & hctx,
                                                          UNORDERED_map<int64_t, NetworkDBPoint *> & stPoints,
                                                          UNORDERED_map<int64_t, NetworkDBPoint *> & endPoints) {
    UNORDERED_map<int64_t, NetworkDBPoint *>::iterator it;
    for (it = stPoints.begin(); it != stPoints.end(); it++) {
        NetworkDBPoint * start = it->second;
        if (start->rtExclude) {
            continue;
        }
        double cost = start->rt(false)->rtCost;
        addPointToQueue(hctx, hctx->queue(false), false, start, nullptr, start->rt(false)->rtDistanceFromStart,
                        cost <= 0 ? MINIMAL_COST : cost);
    }
    for (it = endPoints.begin(); it != endPoints.end(); it++) {
        NetworkDBPoint * end = it->second;
        if (end->rtExclude) {
            continue;
        }
        double cost = end->rt(true)->rtCost;
        addPointToQueue(hctx, hctx->queue(true), true, end, nullptr, end->rt(true)->rtDistanceFromStart,
                        cost <= 0 ? MINIMAL_COST : cost);
    }
    NetworkDBPoint * t = runRoutingWithInitQueue(hctx);
    return t;
}

NetworkDBPoint * HHRoutePlanner::runRoutingWithInitQueue(const SHARED_PTR<HHRoutingContext> & hctx) {
    float DIR_CONFIG = hctx->config->DIJKSTRA_DIRECTION;
    SHARED_PTR<RouteCalculationProgress> progress = hctx->rctx == nullptr ? nullptr : hctx->rctx->progress;
    double straightStartEndCost = squareRootDist31(hctx->startX, hctx->startY, hctx->endX, hctx->endY) /
            hctx->rctx->config->router->getMaxSpeed();
    while (true) {
        SHARED_PTR<HH_QUEUE> queue;
        if (hctx->USE_GLOBAL_QUEUE) {
            queue = hctx->queue(false);
            if (queue->empty()) {
                break;
            }
        } else {
            SHARED_PTR<HH_QUEUE> pos = hctx->queue(false);
            SHARED_PTR<HH_QUEUE> rev = hctx->queue(true);
            if (hctx->config->DIJKSTRA_DIRECTION == 0 || (!rev->empty() && !pos->empty())) {
                if (rev->empty() || pos->empty()) {
                    break;
                }
                queue = pos->top()->cost < rev->top()->cost ? pos : rev;
            } else {
                queue = hctx->config->DIJKSTRA_DIRECTION > 0 ? pos : rev;
                if (queue->empty()) {
                    break;
                }
            }
        }
        if (progress != nullptr && progress->isCancelled()) {
            return nullptr;
        }
        OsmAnd::ElapsedTimer timer;
        timer.Start();
        SHARED_PTR<NetworkDBPointCost> pointCost = queue->top();
        queue->pop();
        NetworkDBPoint * point = pointCost->point;
        bool rev = pointCost->rev;
        hctx->stats.pollQueueTime += timer.GetElapsedMs();
        hctx->stats.visitedVertices++;
        if (point->rt(!rev)->rtVisited) {
            if (hctx->stats.firstRouteVisitedVertices == 0) {
                hctx->stats.firstRouteVisitedVertices = hctx->stats.visitedVertices;
                if (DIR_CONFIG == 0 && hctx->config->HEURISTIC_COEFFICIENT != 0) {
                    // focus on 1 direction as it slightly faster
                    DIR_CONFIG = rev ? -1 : 1;
                }
            }
            if (hctx->config->HEURISTIC_COEFFICIENT == 0 && hctx->config->DIJKSTRA_DIRECTION == 0) {
                // Valid only HC=0, Dijkstra as we run Many-to-Many - Test( Lat 49.12691 Lon 9.213685 -> Lat 49.155483 Lon 9.2140045)
                NetworkDBPoint * finalPoint = point;
                finalPoint = scanFinalPoint(finalPoint, hctx->visited);
                finalPoint = scanFinalPoint(finalPoint, hctx->visitedRev);
                return finalPoint;
            } else {
                double rcost = point->rt(true)->rtDistanceFromStart + point->rt(false)->rtDistanceFromStart;
                if (rcost <= pointCost->cost) {
                    // Universal condition to stop: works for any algorithm - cost equals to route length
                    return point;
                } else {
                    queue->push(std::make_shared<NetworkDBPointCost>(point, rcost, rev));
                    point->markVisited(rev);
                    continue;
                }
            }
        }
        if (point->rt(rev)->rtVisited) {
            continue;
        }
        hctx->stats.uniqueVisitedVertices++;
        point->markVisited(rev);
        hctx->visited.push_back(point);
        (rev ? hctx->visited : hctx->visitedRev).push_back(point);
        printPoint(point, rev);
        if (progress != nullptr && straightStartEndCost > 0) {
            const double STRAIGHT_TO_ROUTE_COST = 1.25; // approximate, tested on car/bike
            // correlation between straight-cost and route-cost (enough for the progress bar)
            double k = (pointCost->cost - straightStartEndCost) / straightStartEndCost * STRAIGHT_TO_ROUTE_COST;
            progress->hhIterationProgress(k);
        }
        if (hctx->config->MAX_COST > 0 && pointCost->cost > hctx->config->MAX_COST) {
            break;
        }
        if (hctx->config->MAX_SETTLE_POINTS > 0 && (rev ? hctx->visitedRev : hctx->visited).size() > hctx->config->MAX_SETTLE_POINTS) {
            break;
        }
        
        bool directionAllowed = (DIR_CONFIG <= 0 && rev) || (DIR_CONFIG >= 0 && !rev);
        if (directionAllowed) {
            addConnectedToQueue(hctx, queue, point, rev);
        }
    }
    return nullptr;
}

void HHRoutePlanner::printPoint(NetworkDBPoint * p, bool rev) const {
    if (DEBUG_VERBOSE_LEVEL > 1) {
        int64_t pind = 0;
        int64_t pchInd = 0;
        if (p->rt(rev)->rtRouteToPoint != nullptr) {
            pind = p->rt(rev)->rtRouteToPoint->index;
            pchInd = p->rt(rev)->rtRouteToPoint->chInd();
        }
        OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "Visit Point %s %lld [%d] (from %lld [%lld]) (cost %.1f s) %.5f/%.5f - %lld",
                          rev ? "<-" : "->", p->index, p->chInd(), pind, pchInd, p->rt(rev)->rtCost,
                          get31LatitudeY(p->startY), get31LongitudeX(p->startX), p->roadId / 64);
    }
}

void HHRoutePlanner::calcAlternativeRoute(const SHARED_PTR<HHRoutingContext> & hctx, HHNetworkRouteRes * route,
                                          UNORDERED_map<int64_t, NetworkDBPoint *> & stPoints, UNORDERED_map<int64_t,
                                          NetworkDBPoint *> & endPoints, SHARED_PTR<RouteCalculationProgress> & progress) {
    std::vector<NetworkDBPoint *> exclude;
    HHNetworkRouteRes * rt = route;
    // distances between all points and start/end
    std::vector<NetworkDBPoint *> points;
    for (int i = 0; i < route->segments.size(); i++) {
        NetworkDBSegment * s = route->segments.at(i).segment;
        if (s == nullptr) {
            continue;
        }
        if(points.size() == 0) {
            points.push_back(s->start);
        }
        points.push_back(s->end);
    }
    double * distances = new double[points.size()];
    NetworkDBPoint * prev = nullptr;
    for (int i = 0; i < sizeof(distances); i++) {
        NetworkDBPoint * pnt = points.at(i);
        if (i == 0) {
            distances[i] = squareRootDist31(hctx->startX, hctx->startY, pnt->midX(), pnt->midY());
        } else if (i == sizeof(distances) - 1) {
            distances[i] = squareRootDist31(hctx->endX, hctx->endY, pnt->midX(), pnt->midY());
        } else {
            distances[i] = squareRootDist31(prev->midX(), prev->midY(), pnt->midX(), pnt->midY());
        }
        prev = pnt;
    }
    
    // calculate min(cumPos, cumNeg) distance
    double * cdistPos = new double[sizeof(distances)];
    double * cdistNeg = new double[sizeof(distances)];
    for (int i = 0; i < sizeof(distances); i++) {
        if(i == 0) {
            cdistPos[0] = distances[i];
            cdistNeg[sizeof(distances) - 1] = distances[sizeof(distances)- 1];
        } else {
            cdistPos[i] = cdistPos[i - 1] + distances[i];
            cdistNeg[sizeof(distances) - i - 1] = cdistNeg[sizeof(distances) - i] + distances[sizeof(distances) - i - 1];
        }
    }
    double * minDistance = new double[sizeof(distances)];
    bool * useToSkip = new bool[sizeof(distances)];
    int altPoints = 0;
    for (int i = 0; i < sizeof(distances); i++) {
        minDistance[i] = std::min(cdistNeg[i], cdistPos[i]) * hctx->config->ALT_EXCLUDE_RAD_MULT;
        bool coveredByPrevious = false;
        for (int j = 0; j < i; j++) {
            if (useToSkip[j] && cdistPos[i] - cdistPos[j] < minDistance[j] * hctx->config->ALT_EXCLUDE_RAD_MULT_IN) {
                coveredByPrevious = true;
                break;
            }
        }
        if(!coveredByPrevious) {
            useToSkip[i] = true;
            altPoints ++;
        } else {
            minDistance[i] = 0; // for printing purpose
        }
    }
    if (DEBUG_VERBOSE_LEVEL >= 1) {
        std::string s = "[";
        for (int i = 0; i < sizeof(minDistance); i++) {
            if (i > 0) {
                s += ", ";
            }
            s += to_string(minDistance[i]);
        }
        s += "]";
        OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "Selected %d points for alternatives %s\n", altPoints, s.c_str());
    }
    if (progress->isCancelled()) {
        return;
    }
    for (int i = 0; i < sizeof(distances); i++) {
        if (!useToSkip[i]) {
            continue;
        }
        hctx->clearVisited(stPoints, endPoints);
        for (NetworkDBPoint * pnt : exclude) {
            pnt->rtExclude = false;
        }
        exclude.clear();
        
        LatLon pnt = points.at(i)->getPoint();
        //std::vector<NetworkDBPoint *> getClosestObjects(double latitude, double longitude, double radius);
        std::vector<NetworkDBPoint *> objs = hctx->pointsRect.getClosestObjects(pnt.lat, pnt.lon, minDistance[i]);
        for (NetworkDBPoint * p : objs) {
            LatLon pCoords = p->getPoint();
            if (getDistance(pCoords.lat, pCoords.lon, pnt.lat, pnt.lon) <= minDistance[i]) {
                exclude.push_back(p);
                p->rtExclude = true;
            }
        }
        
        NetworkDBPoint * finalPnt = runRoutingPointsToPoints(hctx, stPoints, endPoints);
        if (progress->isCancelled()) {
            return;
        }
        if (finalPnt != nullptr) {
            double cost = (finalPnt->rt(false)->rtDistanceFromStart + finalPnt->rt(true)->rtDistanceFromStart);
            if (DEBUG_VERBOSE_LEVEL == 1) {
                OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "Alternative route cost: %.2f", cost);
            }
            rt = createRouteSegmentFromFinalPoint(hctx, finalPnt);
            route->altRoutes.push_back(rt);
        } else {
            break;
        }
    }
    
    std::sort(route->altRoutes.begin(), route->altRoutes.end(), [](HHNetworkRouteRes * o1, HHNetworkRouteRes * o2) {
        return o1->getHHRoutingTime() > o2->getHHRoutingTime();
    });
    
    int size = (int) route->altRoutes.size();
    if (size > 0) {
        for(int k = 0; k < route->altRoutes.size(); ) {
            HHNetworkRouteRes * altR = route->altRoutes.at(k);
            bool unique = true;
            for (int j = 0; j <= k; j++) {
                HHNetworkRouteRes * cmp = j == k ? route : route->altRoutes.at(j);
                std::set<int64_t> cp = altR->uniquePoints;
                retainAll(cp, cmp->uniquePoints);                
                if (cp.size() >= hctx->config->ALT_NON_UNIQUENESS * altR->uniquePoints.size()) {
                    unique = false;
                    break;
                }
            }
            if (unique) {
                k++;
            } else {
                route->altRoutes.erase(route->altRoutes.begin() + k);
            }
        }
        if (route->altRoutes.size() > 0 && hctx->config->STATS_VERBOSE_LEVEL > 0) {
            OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info,
                              "Cost %.2f - %.2f [%zu unique / %d]...",
                              route->altRoutes.at(0)->getHHRoutingTime(),
                              route->altRoutes.at(route->altRoutes.size() - 1)->getHHRoutingTime(),
                              route->altRoutes.size(), size);
        }
        int ind = DEBUG_ALT_ROUTE_SELECTION % (route->altRoutes.size() + 1);
        if (ind > 0) {
            HHNetworkRouteRes * rts = route->altRoutes.at(ind - 1);
            if (hctx->config->STATS_VERBOSE_LEVEL > 0) {
                OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "%d select %.2f ", DEBUG_ALT_ROUTE_SELECTION, rts->getHHRoutingTime());
            }
            route->detailed = rts->detailed;
            route->segments = rts->segments;
            std::vector<HHNetworkRouteRes *> r;
            r.push_back(rts);
            route->altRoutes = r;
        }
    }
    
    for (NetworkDBPoint * pnt : exclude) {
        pnt->rtExclude = false;
    }
}

bool HHRoutePlanner::retainAll(std::set<int64_t> & source, const std::set<int64_t> & other) const {
    if (source == other) {
        return false;
    } else {
        bool modified = false;
        std::set<int64_t>::iterator iter = source.begin();
        while (iter != source.end()) {
            int64_t value = *iter;
            auto it = other.find(value);
            if (it == other.end()) {
                source.erase(iter);
                modified = true;
            }
            iter++;
        }
        return modified;
    }
}

void HHRoutePlanner::filterPointsBasedOnConfiguration(const SHARED_PTR<HHRoutingContext> & hctx) {
    SHARED_PTR<GeneralRouter>  & vr = hctx->rctx->config->router;
    UNORDERED_map<string, string> tm;
    if (vr->hhNativeFilter.size() > 0) {
        tm = vr->hhNativeFilter;
    } else {
        UNORDERED_map<string, RoutingParameter> & parameters = vr->getParameters();
        for (const auto & es : vr->getParameterValues()) {
            string paramId = es.first;
            // These parameters don't affect the routing filters
            // This assumption probably shouldn't be used in general and only for popular parameters)
            if (parameters.find(paramId) != parameters.end() && paramId != GeneralRouterConstants::USE_SHORTEST_WAY
                && paramId != GeneralRouterConstants::USE_HEIGHT_OBSTACLES
                && !startsWith(paramId, GeneralRouterConstants::GROUP_RELIEF_SMOOTHNESS_FACTOR)) {
                tm.insert(std::make_pair(paramId, es.second));
            }
        }
    }
    if (hctx->filterRoutingParameters.size() == tm.size()) {
        bool eq = true;
        for (const auto & t : tm) {
            if (hctx->filterRoutingParameters.find(t.first) == hctx->filterRoutingParameters.end()) {
                eq = false;
                break;
            }
        }
        if (eq) {
            return;
        }
    }
    for (auto & it : hctx->pointsById) {
        it.second->rtExclude = false;
    }
    if (tm.size() == 0) {
        // no parameters
        hctx->filterRoutingParameters = tm;
        return;
    }
    if (hctx->config->STATS_VERBOSE_LEVEL > 0) {
        OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, " Filter points based on parameters...");
    }
    OsmAnd::ElapsedTimer timer;
    timer.Start();
    const SHARED_PTR<RoutingIndex> regR = std::make_shared<RoutingIndex>();
    SHARED_PTR<RouteDataObject> rdo = std::make_shared<RouteDataObject>(regR);
    for (auto & it : hctx->pointsById) {
        NetworkDBPoint * pnt = it.second;
        for (TagValuePair & tp : pnt->tagValues) {
            tp.additionalAttribute = -1;
        }
    }
    int filtered = 0;
    for (auto & it : hctx->pointsById) {
        NetworkDBPoint * pnt = it.second;
        if (pnt->tagValues.size() > 0) {
            rdo->types.clear();
            for (TagValuePair & tp : pnt->tagValues) {
                // reuse additionalAttribute to cache values
                if (tp.additionalAttribute < 0) {
                    tp.additionalAttribute = regR->searchRouteEncodingRule(tp.tag, tp.value);
                }
                if (tp.additionalAttribute < 0) {
                    tp.additionalAttribute = (int32_t)regR->routeEncodingRules.size();
                    regR->initRouteEncodingRule(tp.additionalAttribute, tp.tag, tp.value);
                }
                rdo->types.push_back(tp.additionalAttribute);
            }
            pnt->rtExclude = !currentCtx->rctx->config->router->acceptLine(rdo);
            if (!pnt->rtExclude) {
                // constant should be reduced if route is not found
                pnt->rtExclude = currentCtx->rctx->config->router->defineSpeedPriority(rdo, pnt->end > pnt->start) < EXCLUDE_PRIORITY_CONSTANT;
            }
            if (pnt->rtExclude) {
                filtered++;
            }
            pnt->tagValues = {};
        }
    }
    hctx->filterRoutingParameters = tm;
    if (hctx->config->STATS_VERBOSE_LEVEL > 0) {
        OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "%d excluded from %zu, %.2llu ms\n", filtered, hctx->pointsById.size(), timer.GetElapsedMs());
    }
}

#endif /*_OSMAND_HH_ROUTE_PLANNER_CPP*/
