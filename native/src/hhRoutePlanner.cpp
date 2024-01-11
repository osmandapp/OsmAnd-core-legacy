#ifndef _OSMAND_HH_ROUTE_PLANNER_CPP
#define _OSMAND_HH_ROUTE_PLANNER_CPP

#include <ctime>
#include "hhRoutePlanner.h"
#include "CommonCollections.h"
#include "Logging.h"
#include "routePlannerFrontEnd.cpp"

HHRoutePlanner::HHRoutePlanner(SHARED_PTR<RoutingContext> ctx) {
    std::vector<SHARED_PTR<HHRouteRegionPointsCtx>> regions;
    initNewContext(ctx, regions);
    hhRouteRegionGroup = std::make_shared<HHRouteRegionsGroup>();
}

SHARED_PTR<HHRoutingConfig> HHRoutePlanner::prepareDefaultRoutingConfig(SHARED_PTR<HHRoutingConfig> c) {
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

SHARED_PTR<HHRoutingContext> HHRoutePlanner::initNewContext(SHARED_PTR<RoutingContext> ctx, std::vector<SHARED_PTR<HHRouteRegionPointsCtx>> regions) {
    //TODO change to pointer
    cacheHctx = std::make_shared<HHRoutingContext>();
    cacheHctx->rctx = ctx;
    if (regions.size() > 0) {
        cacheHctx->regions.insert(cacheHctx->regions.end(), regions.begin(), regions.end());
    }
    return cacheHctx;
}

SHARED_PTR<HHRoutingContext> HHRoutePlanner::selectBestRoutingFiles(int startX, int startY, int endX, int endY, SHARED_PTR<HHRoutingContext> hctx) {
    std::vector<SHARED_PTR<HHRouteRegionsGroup>> groups;
    SHARED_PTR<GeneralRouter> router = hctx->rctx->config->router;
    string profile = profileToString(router->getProfile()); // use base profile
    std::vector<string> ls = router->serializeParameterValues(router->getParameterValues());
    
    SkRect qr = SkRect::MakeLTRB(std::min(startX, endX), std::max(startY, endY), std::max(startX, endX), std::min(startY, endY));
    
    const std::vector<BinaryMapFile*> & openFiles = getOpenMapFiles();
    //TODo const ???
    for (BinaryMapFile* r : openFiles) {
        for (SHARED_PTR<HHRouteIndex> & hhRegion : r->hhIndexes) {
            hhRouteRegionGroup->appendToGroups(hhRegion, r, groups);
        }
    }
    
    SHARED_PTR<HHRouteRegionsGroup> bestGroup = nullptr;
    for (auto & g : groups) {
        g->containsStartEnd = g->contains(startX, startY) && g->contains(endX, endY);
        if (bestGroup == nullptr) {
            bestGroup = g;
        }
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
        if (g->containsStartEnd != bestGroup->containsStartEnd) {
            if (g->containsStartEnd) {
                bestGroup = g;
            }
        } else if (g->extraParam != bestGroup->extraParam) {
            if (g->extraParam < bestGroup->extraParam) {
                bestGroup = g;
            }
        } else if (g->matchParam != bestGroup->matchParam) {
            if (g->matchParam > bestGroup->matchParam) {
                bestGroup = g;
            }
        }
    }
    if (bestGroup == nullptr) {
        return nullptr;
    }
    
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
    
    if (cacheHctx != nullptr) {
       //TODO cacheHctx needs to remove (Victor)
    }
    return initNewContext(hctx->rctx, regions);
}

MAP_VECTORS_NETWORK_DB_POINTS HHRoutePlanner::groupByClusters( UNORDERED_map<int64_t, NetworkDBPoint *> pointsById, bool out) {
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
    
    MAP_VECTORS_NETWORK_DB_POINTS::iterator it2;
    for (it2 = res.begin(); it2 != res.end(); it++) {
        auto & l = it2->second;
        //TODO check Integer.compare
        std::sort(l.begin(), l.end(), [](const NetworkDBPoint * lhs, const NetworkDBPoint * rhs) {
            return lhs->index > rhs->index;
        });
    }
    return res;
}

int64_t HHRoutePlanner::calculateRoutePointInternalId(int64_t id, int32_t pntId, int32_t nextPntId) {
    int32_t positive = nextPntId - pntId;
    return (id << ROUTE_POINTS) + (pntId << 1) + (positive > 0 ? 1 : 0);
}

int64_t HHRoutePlanner::calculateRoutePointInternalId(SHARED_PTR<RouteDataObject> road, int32_t pntId, int32_t nextPntId) {
    int32_t positive = nextPntId - pntId;
    int pntLen = road->getPointsLength();
    if (positive < 0) {
        OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "Check only positive segments are in calculation");
        return -1;
    }
    if (pntId < 0 || nextPntId < 0 || pntId >= pntLen || nextPntId >= pntLen ||
        (positive != -1 && positive != 1) || pntLen > (1 << ROUTE_POINTS)) {
        OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "Assert failed");
        return -1;
    }
    return (road->getId() << ROUTE_POINTS) + (pntId << 1) + (positive > 0 ? 1 : 0);
}

SHARED_PTR<HHRoutingContext> HHRoutePlanner::initHCtx(SHARED_PTR<HHRoutingConfig> c, int startX, int startY, int endX, int endY) {
    SHARED_PTR<HHRoutingContext> hctx = cacheHctx;
    if (hctx->regions.size() != 1) {
        hctx = selectBestRoutingFiles(startX, startY, endX, endY, hctx);
    }
    
    OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "Selected files: %s", (hctx == nullptr ? " NULL " : hctx->getRoutingInfo().c_str()));
    if (hctx == nullptr) {
        return hctx;
    }
    hctx->config = c;
    hctx->startX = startX;
    hctx->startY = startY;
    hctx->endX = endX;
    hctx->endX = endY;

    hctx->clearVisited();
    if (hctx->initialized) {
        return hctx;
    }
            
    OsmAnd::ElapsedTimer timer;
    timer.Start();
    OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "Loading points... ");
    hctx->pointsById = hctx->loadNetworkPoints();
    hctx->stats.loadPointsTime = timer.GetElapsedMs();
    OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, " %d - %.2fms\n", hctx->pointsById.size(), hctx->stats.loadPointsTime);
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
    hctx->pointsRect.printStatsDistribution("Points distributed");
    hctx->initialized = true;
    return hctx;
}

//TODO how to store HHNetworkRouteRes shared_ptr or pointer ?
HHNetworkRouteRes HHRoutePlanner::runRouting(int startX, int startY, int endX, int endY, SHARED_PTR<HHRoutingConfig> config) {
    OsmAnd::ElapsedTimer timer;
    timer.Start();
    config = prepareDefaultRoutingConfig(config);
    SHARED_PTR<HHRoutingContext> hctx = initHCtx(config, startX, startY, endX, endY);
    if (hctx == nullptr) {
        HHNetworkRouteRes res("Files for hh routing were not initialized. Route couldn't be calculated.");
        return res;
    }
    if (hctx->config->USE_GC_MORE_OFTEN) {
        //printGCInformation();
    }
    OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "Routing %.5f %.5f -> %.5f %.5f (HC %d, dir %d)",
                      get31LatitudeY(startY), get31LongitudeX(startX),
                      get31LatitudeY(endY), get31LongitudeX(endX),
                      (int) config->HEURISTIC_COEFFICIENT, (int) config->DIJKSTRA_DIRECTION);
    UNORDERED_map<int64_t, NetworkDBPoint *> stPoints;
    UNORDERED_map<int64_t, NetworkDBPoint *> endPoints;
    findFirstLastSegments(hctx, startX, startY, endX, endY, stPoints, endPoints);
    //RouteResultPreparation rrp;
    HHNetworkRouteRes * route = nullptr;
    while (route == nullptr) {
        OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "Routing...");
        long time = timer.GetElapsedMs();
        NetworkDBPoint * finalPnt = runRoutingPointsToPoints(hctx, stPoints, endPoints);
        route = createRouteSegmentFromFinalPoint(hctx, finalPnt);
        time = (timer.GetElapsedMs() - time);
        OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "%d segments, cost %.2f, %.2f ms\n", route->segments.size(), route->getHHRoutingTime(), time);
        hctx->stats.routingTime+= time;
        OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "Parse detailed route segments...");
        time = timer.GetElapsedMs();
        bool recalc = retrieveSegmentsGeometry(hctx, route, hctx->config->ROUTE_ALL_SEGMENTS);
        time = (timer.GetElapsedMs() - time);
        OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "%.2f ms\n", time);
        hctx->stats.routingTime += time;
        if (recalc) {
            if (hctx->stats.prepTime + hctx->stats.routingTime > hctx->config->MAX_TIME_REITERATION_MS) {
                HHNetworkRouteRes res("Too many route recalculations (maps are outdated).");
                return res;
            }
            hctx->clearVisited(stPoints, endPoints);
            delete route;
            route = nullptr;
        }
    }
    
    if (hctx->config->CALC_ALTERNATIVES) {
        OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "Alternative routes...");
        long time = timer.GetElapsedMs();
        calcAlternativeRoute(hctx, route, stPoints, endPoints);
        hctx->stats.altRoutingTime += timer.GetElapsedMs() - time;
        hctx->stats.routingTime += hctx->stats.altRoutingTime;
        OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "%d %.2f ms\n", route->altRoutes.size(), hctx->stats.altRoutingTime);
        time = timer.GetElapsedMs();
        for (HHNetworkRouteRes * alt : route->altRoutes) {
            retrieveSegmentsGeometry(hctx, alt, hctx->config->ROUTE_ALL_ALT_SEGMENTS);
        }
        hctx->stats.prepTime += timer.GetElapsedMs() - time;
    }
    
    if (hctx->config->USE_GC_MORE_OFTEN) {
        //hctx->unloadAllConnections();
        //printGCInformation();
    }
    
    long time = timer.GetElapsedMs();
    //prepareRouteResults(hctx, route, start, end, rrp);
    hctx->stats.prepTime += timer.GetElapsedMs() - time;
    OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "%.2f ms\n", hctx->stats.prepTime);
    if (DEBUG_VERBOSE_LEVEL >= 1) {
        //OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "Detailed progress: " + hctx->rctx->calculationProgress.getInfo(null));
    }
    OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info,
                      "Found final route - cost %.2f (detailed %.2f, %.1f%%), %d depth ( first met %,d, visited %,d (%,d unique) of %,d added vertices )",
                      route->getHHRoutingTime(), route->getHHRoutingDetailed(), 100 * (1 - route->getHHRoutingDetailed() / route->getHHRoutingTime()),
                      route->segments.size(), hctx->stats.firstRouteVisitedVertices, hctx->stats.visitedVertices, hctx->stats.uniqueVisitedVertices,
                      hctx->stats.addedVertices);
    time = timer.GetElapsedMs();
    OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "Routing %.5f %.5f -> %.5f %.5f (HC %d, dir %d)",
                      get31LatitudeY(startY), get31LongitudeX(startX),
                      get31LatitudeY(endY), get31LongitudeX(endX),
                      (int) hctx->config->HEURISTIC_COEFFICIENT, (int) hctx->config->DIJKSTRA_DIRECTION);
    OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "Calculate turns...");
    
    if (hctx->config->ROUTE_ALL_SEGMENTS/* && route.detailed != null*/) {
        route->detailed = prepareResult(hctx->rctx.get(), route->detailed);
    }
    OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "%.2f ms\n", timer.GetElapsedMs() - time);
    printResults(hctx->rctx.get(), startX, startY, endX, endY, route->detailed);
    OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info,
                      "Routing finished all %.1f ms: last mile %.1f ms, load data %.1f ms (%,d edges), routing %.1f ms (queue  - %.1f add ms + %.1f poll ms), prep result %.1f ms\n",
                      timer.GetElapsedMs(), hctx->stats.searchPointsTime, hctx->stats.loadEdgesTime + hctx->stats.loadPointsTime,
                      hctx->stats.loadEdgesCnt, hctx->stats.routingTime, hctx->stats.addQueueTime, hctx->stats.pollQueueTime, hctx->stats.prepTime);
    //printGCInformation();
    return *route;
}

void HHRoutePlanner::findFirstLastSegments(SHARED_PTR<HHRoutingContext> hctx, int startX, int startY, int endX, int endY,
                                           UNORDERED_map<int64_t, NetworkDBPoint *> stPoints,
                                           UNORDERED_map<int64_t, NetworkDBPoint *> endPoints) {
    OsmAnd::ElapsedTimer timer;
    timer.Start();
    OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "Finding first / last segments...");
    auto planner = std::shared_ptr<RoutePlannerFrontEnd>();
    int startReiterate = -1, endReiterate = -1;
    bool found = false;
    SHARED_PTR<RouteSegmentPoint> startPnt = findRouteSegment(startX, startY, hctx->rctx.get(), false);
    SHARED_PTR<RouteSegmentPoint> endPnt = findRouteSegment(endX, endY, hctx->rctx.get(), false);
    auto & stOthers = startPnt->others;
    auto & endOthers = endPnt->others;
    while (!found) {
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
        //TODO need copy here???
        auto & startP = startPnt;
        if (startReiterate >= 0) {
            if (startReiterate < stOthers.size()) {
                startP = stOthers.at(startReiterate);
            } else {
                break;
            }
        }
        auto & endP = endPnt;
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
        initStart(hctx, startP, false, stPoints);
        hctx->rctx->config->initialDirection = prev;
        if (stPoints.empty()) {
            LatLon l = startP->getPreciseLatLon();
            auto & r = startP->road;
            OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "Reiterate with next start point: %d (%.5f %.5f): %d %s",
                              startP->segmentStart, l.lat, l.lon, r->getId() / 64, r->getName().c_str());
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
        initStart(hctx, endP, true, endPoints);
        if (endPoints.empty()) {
            LatLon l = endP->getPreciseLatLon();
            auto & r = endP->road;
            OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "Reiterate with next end point: %d (%.5f %.5f): %d %s",
                              endP->segmentStart, l.lat, l.lon, r->getId() / 64, r->getName().c_str());
            endReiterate++;
            found = false;
            continue;
        }
        found = true;
    }
    hctx->stats.searchPointsTime = timer.GetElapsedMs();
    OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "Finding first / last segments...%.2f ms\n", hctx->stats.searchPointsTime);
    timer.Disable();
}

int64_t HHRoutePlanner::calcRPId(SHARED_PTR<RouteSegmentPoint> p, int pntId, int nextPntId) {
    return calculateRoutePointInternalId(p->getRoad()->getId(), pntId, nextPntId);
}

int64_t HHRoutePlanner::calcRPId(SHARED_PTR<RouteSegment> p, int pntId, int nextPntId) {
    return calculateRoutePointInternalId(p->getRoad()->getId(), pntId, nextPntId);
}

UNORDERED_map<int64_t, NetworkDBPoint *> HHRoutePlanner::initStart(SHARED_PTR<HHRoutingContext> hctx, SHARED_PTR<RouteSegmentPoint> s,
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
                pnt->setCostParentRt(reverse, cost + distanceToEnd(hctx, reverse, pnt), nullptr, cost);
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
    NetworkDBPoint * finitePnt = hctx->pointsByGeo.at(uniDirRouteIntId);
    if (finitePnt != nullptr) {
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
        finitePnt->setDistanceToEnd(reverse, distanceToEnd(hctx, reverse, finitePnt));
        finitePnt->setCostParentRt(reverse, plusCost, nullptr, plusCost);
        pnts.insert(std::pair<int64_t, NetworkDBPoint *>(finitePnt->index, finitePnt));
        NetworkDBPoint * dualPoint =  finitePnt->dualPoint;
        dualPoint->setDistanceToEnd(reverse, distanceToEnd(hctx, reverse, dualPoint));
        dualPoint->setCostParentRt(reverse, negCost, nullptr, negCost);
        pnts.insert(std::pair<int64_t, NetworkDBPoint *>(dualPoint->index, dualPoint));
        return pnts;
    }
    hctx->rctx->config->MAX_VISITED = MAX_POINTS_CLUSTER_ROUTING;
    hctx->rctx->config->planRoadDirection = reverse ? -1 : 1;
    hctx->rctx->config->heurCoefficient = 0; // dijkstra
    hctx->rctx->unloadAllData(); // needed for proper multidijsktra work
    //hctx->rctx->calculationProgress = new RouteCalculationProgress();
    const SHARED_PTR<VISITED_MAP> boundaries = std::make_shared<VISITED_MAP>(hctx->boundaries);
    std::vector<SHARED_PTR<RouteSegment>> frs = searchRouteInternal(hctx->rctx.get(), reverse ? nullptr : s, reverse ? s : nullptr, boundaries);
    hctx->rctx->config->MAX_VISITED = -1;
    //OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, hctx.rctx.calculationProgress.getInfo(null));
    if (!frs.empty()) {
        std::set<int64_t> set;
        for (auto & o : frs) {
            // duplicates are possible as alternative routes
            int64_t pntId = calculateRoutePointInternalId(o->getRoad()->getId(),
                            reverse ? o->getSegmentEnd() : o->getSegmentStart(),
                            reverse ? o->getSegmentStart() : o->getSegmentEnd());
            if (set.insert(pntId).second) {
                NetworkDBPoint * pnt = hctx->createNetworkDBPoint();
                auto it = hctx->pointsByGeo.find(pntId);
                if (it == hctx->pointsByGeo.end()) {
                    it = pnts.find(PNT_SHORT_ROUTE_START_END);
                    if (it != pnts.end()) {
                        continue;
                    }
                    
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
                    o->distanceFromStart += calculatePreciseStartTime(hctx->rctx.get(), preciseX, preciseY, o);
                } else {
                    o->distanceFromStart += calcRoutingSegmentTimeOnlyDist(hctx->rctx->config->router, o) / 2;
                }
                if (pnt->rt(reverse)->rtCost != 0) {
                    OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "Illegal state exception");
                    return pnts;
                }
                pnt->setDistanceToEnd(reverse, distanceToEnd(hctx, reverse, pnt));
                pnt->setDetailedParentRt(reverse, o);
                pnts.insert(std::pair<int64_t, NetworkDBPoint *>(pnt->index, pnt));
            }
        }
    }
    if (hctx->config->USE_GC_MORE_OFTEN) {
        hctx->rctx->unloadAllData();
        //printGCInformation();
    }
    return pnts;
}

double HHRoutePlanner::distanceToEnd(SHARED_PTR<HHRoutingContext> hctx, bool reverse, NetworkDBPoint * nextPoint) {
    if (hctx->config->HEURISTIC_COEFFICIENT > 0) {
        double distanceToEnd = nextPoint->rt(reverse)->rtDistanceToEnd;
        if (distanceToEnd == 0) {
            double dist = squareRootDist31(reverse ? hctx->startX : hctx->endX, reverse ? hctx->startY : hctx->endY,
                                           nextPoint->midX(), nextPoint->midY());
            distanceToEnd = hctx->config->HEURISTIC_COEFFICIENT * dist / hctx->rctx->config->router->getMaxSpeed();
            nextPoint->setDistanceToEnd(reverse, distanceToEnd);
        }
        return distanceToEnd;
    }
    return 0;
}

int64_t HHRoutePlanner::calcUniDirRoutePointInternalId(SHARED_PTR<RouteSegmentPoint> segm) {
    if (segm->getSegmentStart() < segm->getSegmentEnd()) {
        return calculateRoutePointInternalId(segm->getRoad(), segm->getSegmentStart(), segm->getSegmentEnd());
    } else {
        return calculateRoutePointInternalId(segm->getRoad(), segm->getSegmentEnd(), segm->getSegmentStart());
    }
}

HHNetworkRouteRes * HHRoutePlanner::createRouteSegmentFromFinalPoint(SHARED_PTR<HHRoutingContext> hctx, NetworkDBPoint * pnt) {
    HHNetworkRouteRes * route = new HHNetworkRouteRes();
    if (pnt != nullptr) {
        NetworkDBPoint * itPnt = pnt;
        route->uniquePoints.insert(itPnt->index);
        while (itPnt->rt(true)->rtRouteToPoint != nullptr) {
            NetworkDBPoint * nextPnt = itPnt->rt(true)->rtRouteToPoint;
            NetworkDBSegment * segment = nextPnt->getSegment(itPnt, false);
            HHNetworkSegmentRes res(segment);
            route->segments.push_back(res);
            res.rtTimeDetailed = res.rtTimeHHSegments = segment->dist;
            itPnt = nextPnt;
            route->uniquePoints.insert(itPnt->index);
            //TODO check deletion of pointer NetworkDBSegment * segment in the end of calculation
        }
        
        if (itPnt->rt(true)->rtDetailedRoute != nullptr) {
            HHNetworkSegmentRes res(nullptr);
            res.list = convertFinalSegmentToResults(hctx->rctx.get(), itPnt->rt(true)->rtDetailedRoute);
            res.rtTimeDetailed = res.rtTimeHHSegments = itPnt->rt(true)->rtDetailedRoute->distanceFromStart;
            route->segments.push_back(res);
        }
        std::reverse(route->segments.begin(), route->segments.end());
        itPnt = pnt;
        while (itPnt->rt(false)->rtRouteToPoint != nullptr) {
            NetworkDBPoint * nextPnt = itPnt->rt(false)->rtRouteToPoint;
            NetworkDBSegment * segment = nextPnt->getSegment(itPnt, true);
            HHNetworkSegmentRes res(segment);
            route->segments.push_back(res);
            res.rtTimeDetailed = res.rtTimeHHSegments = segment->dist;
            itPnt = nextPnt;
            route->uniquePoints.insert(itPnt->index);
            //TODO check deletion of pointer NetworkDBSegment * segment in the end of calculation
        }
        if (itPnt->rt(false)->rtDetailedRoute != nullptr) {
            HHNetworkSegmentRes res(nullptr);
            res.list = convertFinalSegmentToResults(hctx->rctx.get(), itPnt->rt(false)->rtDetailedRoute);
            res.rtTimeDetailed = res.rtTimeHHSegments = itPnt->rt(false)->rtDetailedRoute->distanceFromStart;
            route->segments.push_back(res);
        }
        std::reverse(route->segments.begin(), route->segments.end());
    }
    return route;
}

void HHRoutePlanner::recalculateNetworkCluster(SHARED_PTR<HHRoutingContext> hctx, NetworkDBPoint * start) {
    //BinaryRoutePlanner plan = new BinaryRoutePlanner();
    hctx->rctx->config->planRoadDirection = 1;
    hctx->rctx->config->heurCoefficient = 0;
    // SPEEDUP: Speed up by just clearing visited
    hctx->rctx->unloadAllData(); // needed for proper multidijsktra work
    SHARED_PTR<RouteSegmentPoint> s = loadPoint(hctx->rctx, start);
    //hctx->rctx->calculationProgress = new RouteCalculationProgress();
    hctx->rctx->config->MAX_VISITED = MAX_POINTS_CLUSTER_ROUTING * 2;
    int64_t ps = calcRPId(s, s->getSegmentStart(), s->getSegmentEnd());
    //TODO ExcludeTLongObjectMap
    SHARED_PTR<VISITED_MAP> bounds = nullptr;
    //ExcludeTLongObjectMap<RouteSegment> bounds = new ExcludeTLongObjectMap<>(hctx.boundaries, ps);
    auto frs = searchRouteInternal(hctx->rctx.get(), s, nullptr, bounds);
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
                    // System.out.printf("Correct dist %.2f -> %.2f\n", c.dist, routeTime);
                    c->dist = routeTime;
                } else {
                    //TODO check deletion of pointer
                    start->connected.push_back(hctx->createNetworkDBSegment(start, p, routeTime, true, false));
                }
                NetworkDBSegment * co = p->getSegment(start, false);
                if (co != nullptr) {
                    co->dist = routeTime;
                } else if (p->connectedReverse.size() > 0) {
                    //TODO check deletion of pointer
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


bool HHRoutePlanner::retrieveSegmentsGeometry(SHARED_PTR<HHRoutingContext> hctx, HHNetworkRouteRes * route, bool routeSegments) {
    for (int i = 0; i < route->segments.size(); i++) {
        HHNetworkSegmentRes s = route->segments.at(i);
        if (s.segment == nullptr) {
            // start / end points
            if(i > 0 && i < route->segments.size() -1 ) {
                OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "Segment ind %d is null.", i);
                return false;
            }
            continue;
        }
        
        if (routeSegments) {
            std::vector<SHARED_PTR<RouteSegment>> f = runDetailedRouting(hctx, s.segment->start, s.segment->end, true);
            if (f.size() == 0) {
                bool full = hctx->config->FULL_DIJKSTRA_NETWORK_RECALC-- > 0;
                OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info,
                                  "Route not found (%srecalc) %s -> %s\n", full ? "dijkstra+" : "",
                                  s.segment->start, s.segment->end);
                if (full) {
                    recalculateNetworkCluster(hctx, s.segment->start);
                }
                s.segment->dist = -1;
                return true;
            }
            //TODO what is distanceFromStart in really if we return vector
            //if ((f.distanceFromStart + MAX_INC_COST_CORR) > (s.segment.dist + MAX_INC_COST_CORR) * hctx.config.MAX_INC_COST_CF) {
            float distanceFromStart = f.at(0)->distanceFromStart;
            if ((distanceFromStart + MAX_INC_COST_CORR) > (s.segment->dist + MAX_INC_COST_CORR) * hctx->config->MAX_INC_COST_CF) {
                OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info,
                                  "Route cost increased (%.2f > %.2f) between %s -> %s: recalculate route\n",
                                  distanceFromStart, s.segment->dist, s.segment->start, s.segment->end);
                s.segment->dist = distanceFromStart;
                return true;
            }
            s.rtTimeDetailed = distanceFromStart;
            //TODO how to work if size > 1
            s.list = convertFinalSegmentToResults(hctx->rctx.get(), f.at(0));
        } else {
            // load segment geometry from db
            /*if (!hctx->loadGeometry(s.segment, false)) {
                s.segment->getGeometry().clear();
                s.segment->getGeometry().add(s.segment.start.getPoint());
                s.segment->getGeometry().add(s.segment.end.getPoint());
            }*/
        }
    }
    return false;
}

SHARED_PTR<RouteSegmentPoint> HHRoutePlanner::loadPoint(SHARED_PTR<RoutingContext> ctx, const NetworkDBPoint * pnt) {
    std::vector<SHARED_PTR<RouteSegment>> segments = ctx->loadRouteSegment(pnt->startX, pnt->startY);
    SHARED_PTR<RouteSegment> seg = nullptr;
    for (auto & s : segments) {
        if (s) {
            if (s->getRoad()->getId() == pnt->roadId && s->getSegmentStart() == pnt->start) {
                if (s->getSegmentEnd() != pnt->end) {
                    s = s->initRouteSegment(s, !s->isPositive());
                }
                break;
            }
            seg = s;
        }
    }
    if (seg == nullptr || seg->getSegmentStart() != pnt->start || seg->getSegmentEnd() != pnt->end || seg->getRoad()->getId() != pnt->roadId) {
        // throw new IllegalStateException("Error on segment " + pnt.roadId / 64);
        return nullptr;
    }
    SHARED_PTR<RouteSegmentPoint> rsp = std::make_shared<RouteSegmentPoint>(seg->getRoad(), seg->getSegmentStart(), seg->getSegmentEnd(), 0);
    return rsp;
}

std::vector<SHARED_PTR<RouteSegment>> HHRoutePlanner::runDetailedRouting(SHARED_PTR<HHRoutingContext> hctx, const NetworkDBPoint * startS, const NetworkDBPoint * endS, bool useBoundaries) {
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
                          "End point is not present in detailed maps: Point %d (%d %d-%d)",
                          endS->index, endS->roadId, endS->roadId/64, endS->start, endS->end);
        return f;
    }
    double oldP = hctx->rctx->config->penaltyForReverseDirection;
    hctx->rctx->config->penaltyForReverseDirection *= 4;
    hctx->rctx->config->initialDirection = start->getRoad()->directionRoute(start->getSegmentStart(), start->isPositive());
    hctx->rctx->config->targetDirection = end->getRoad()->directionRoute(end->getSegmentEnd(), !end->isPositive());
    hctx->rctx->config->MAX_VISITED = useBoundaries ? -1 : MAX_POINTS_CLUSTER_ROUTING * 2;
    // boundaries help to reduce max visited (helpful for long ferries)
    SHARED_PTR<VISITED_MAP> bounds = nullptr;
    if (useBoundaries) {
        int64_t ps = calcRPId(start, start->getSegmentEnd(), start->getSegmentStart());
        int64_t pe = calcRPId(end, end->getSegmentStart(), end->getSegmentEnd());
        //TODO separate class ExcludeBounds !!!
        //bounds = new ExcludeTLongObjectMap<>(hctx.boundaries, ps, pe);
    }
    f = searchRouteInternal(hctx->rctx.get(), start, end, bounds);
    if (f.size() == 0) {
        OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info,
                          "No route found between %d %.5f %.5f Road (%d) name ('%s') -> %d %.5f %.5f Road (%d) name ('%s') \n",
                          start->segmentStart, get31LatitudeY(start->preciseY), get31LongitudeX(start->preciseY),
                          start->getRoad()->getId() / 64, start->getRoad()->getName().c_str(),
                          end->segmentStart, get31LatitudeY(end->preciseY), get31LongitudeX(end->preciseY),
                          end->getRoad()->getId() / 64, end->getRoad()->getName().c_str());
    }
    hctx->rctx->config->MAX_VISITED = -1;
    // clean up
    hctx->rctx->config->initialDirection = NO_DIRECTION;
    hctx->rctx->config->targetDirection = NO_DIRECTION;
    hctx->rctx->config->penaltyForReverseDirection = oldP;
    return f;
}

NetworkDBPoint * HHRoutePlanner::scanFinalPoint(NetworkDBPoint * finalPoint, std::vector<NetworkDBPoint *> lt) {
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

double HHRoutePlanner::smallestSegmentCost(SHARED_PTR<HHRoutingContext> hctx, NetworkDBPoint * st, NetworkDBPoint * end) {
    double dist = squareRootDist31(st->midX(), st->midY(), end->midX(), end->midY());
    return dist / hctx->rctx->config->router->getMaxSpeed();
}

//TODO perhaps SHARED_PTR<HH_QUEUE> without shared_ptr and only link
void HHRoutePlanner::addPointToQueue(SHARED_PTR<HHRoutingContext> hctx, SHARED_PTR<HH_QUEUE> queue, bool reverse,
                                     NetworkDBPoint * point, NetworkDBPoint * parent, double segmentDist, double cost) {
    OsmAnd::ElapsedTimer timer;
    timer.Start();
    if (DEBUG_VERBOSE_LEVEL > 2) {
        OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info,
                          "Add  %s to visit - cost %.2f (%.2f prev, %.2f dist) > prev cost %.2f \n", point,
                          cost, parent == nullptr ? 0 : parent->rt(reverse)->rtDistanceFromStart, segmentDist, point->rt(reverse)->rtCost);
    }
    if (point->rt(reverse)->rtVisited) {
        OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error,
                          "Point %d (%d %d-%d) visited - cost %.2f > prev cost %.2f",
                          point->index, point->roadId / 64, point->start, point->end, cost, point->rt(reverse)->rtCost);
        return;
    }
    point->setCostParentRt(reverse, cost, parent, segmentDist);
    hctx->queueAdded.push_back(point);
    queue->push(std::make_shared<NetworkDBPointCost>(point, cost, reverse)); // we need to add new object to not  remove / rebalance priority queue
    hctx->stats.addQueueTime += timer.GetElapsedMs();
    hctx->stats.addedVertices++;
    timer.Disable();
}

void HHRoutePlanner::addConnectedToQueue(SHARED_PTR<HHRoutingContext> hctx, SHARED_PTR<HH_QUEUE> queue, NetworkDBPoint * point, bool reverse) {
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
        //TODO chInd() is always 0 !!!
        if (hctx->config->USE_CH && (nextPoint->chInd() > 0 && nextPoint->chInd() < point->chInd())) {
            continue;
        }
        //TODO midPntDepth() is always 0 !!!
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
            // TODO lots of incorrect distance in db
            OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info,
                              "Incorrect distance %s -> %s: db = %.2f > fastest %.2f \n",
                              point, nextPoint, connected->dist, sSegmentCost);
            connected->dist = sSegmentCost;
        }
        double cost = point->rt(reverse)->rtDistanceFromStart  + connected->dist + distanceToEnd(hctx, reverse, nextPoint);
        if (ASSERT_COST_INCREASING && point->rt(reverse)->rtCost - cost > 1) {
            OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error,
                              "Point %d (%d %d-%d) (cost %.2f) -> %s (cost %.2f) st=%.2f-> + %.2f, toend=%.2f->%.2f: ",
                              point->index, point->roadId / 64, point->start, point->end, point->rt(reverse)->rtCost,
                              nextPoint, cost, point->rt(reverse)->rtDistanceFromStart, connected->dist,
                              point->rt(reverse)->rtDistanceToEnd, distanceToEnd(hctx, reverse, nextPoint));
            return;
        }
        double exCost = nextPoint->rt(reverse)->rtCost;
        if ((exCost == 0 && !nextPoint->rt(reverse)->rtVisited) || cost < exCost) {
            addPointToQueue(hctx, queue, reverse, nextPoint, point, connected->dist, cost);
        }
    }
}

NetworkDBPoint * HHRoutePlanner::runRoutingPointsToPoints(SHARED_PTR<HHRoutingContext> hctx,
                                                          UNORDERED_map<int64_t, NetworkDBPoint *> stPoints,
                                                          UNORDERED_map<int64_t, NetworkDBPoint *> endPoints) {
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

NetworkDBPoint * HHRoutePlanner::runRoutingWithInitQueue(SHARED_PTR<HHRoutingContext> hctx) {
    float DIR_CONFIG = hctx->config->DIJKSTRA_DIRECTION;
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

void HHRoutePlanner::printPoint(NetworkDBPoint * p, bool rev) {
    if (DEBUG_VERBOSE_LEVEL > 1) {
        int64_t pind = 0;
        int64_t pchInd = 0;
        if (p->rt(rev)->rtRouteToPoint != nullptr) {
            pind = p->rt(rev)->rtRouteToPoint->index;
            pchInd = p->rt(rev)->rtRouteToPoint->chInd();
        }
        OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info,
                          "Visit Point %s %d [%d] (from %d [%d]) (cost %.1f s) %.5f/%.5f - %d\n",
                          rev ? "<-" : "->", p->index, p->chInd(), pind, pchInd, p->rt(rev)->rtCost,
                          get31LatitudeY(p->startY), get31LongitudeX(p->startX), p->roadId / 64);
    }
}

void HHRoutePlanner::calcAlternativeRoute(SHARED_PTR<HHRoutingContext> hctx, HHNetworkRouteRes * route,
                                          UNORDERED_map<int64_t, NetworkDBPoint *> stPoints, UNORDERED_map<int64_t, NetworkDBPoint *> endPoints) {
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
        if (route->altRoutes.size() > 0) {
            OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info,
                              "Cost %.2f - %.2f [%d unique / %d]...",
                              route->altRoutes.at(0)->getHHRoutingTime(),
                              route->altRoutes.at(route->altRoutes.size() - 1)->getHHRoutingTime(),
                              route->altRoutes.size(), size);
        }
        int ind = DEBUG_ALT_ROUTE_SELECTION % (route->altRoutes.size() + 1);
        if (ind > 0) {
            HHNetworkRouteRes * rts = route->altRoutes.at(ind - 1);
            OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "%d select %.2f ", DEBUG_ALT_ROUTE_SELECTION, rts->getHHRoutingTime());
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

bool HHRoutePlanner::retainAll(std::set<int64_t> & source, const std::set<int64_t> & other) {
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

#endif /*_OSMAND_HH_ROUTE_PLANNER_CPP*/
