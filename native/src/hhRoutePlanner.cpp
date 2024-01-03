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
        NetworkDBPoint * finalPnt = nullptr;//runRoutingPointsToPoints(hctx, stPoints, endPoints);
        route = createRouteSegmentFromFinalPoint(hctx, finalPnt);
        time = (timer.GetElapsedMs() - time);
        OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "%d segments, cost %.2f, %.2f ms\n", route->segments.size(), route->getHHRoutingTime(), time);
        hctx->stats.routingTime+= time;
        OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "Parse detailed route segments...");
        time = timer.GetElapsedMs();
        bool recalc = false;//retrieveSegmentsGeometry(hctx, rrp, route, hctx->config->ROUTE_ALL_SEGMENTS);
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
        //calcAlternativeRoute(hctx, route, stPoints, endPoints);
        hctx->stats.altRoutingTime += timer.GetElapsedMs() - time;
        hctx->stats.routingTime += hctx->stats.altRoutingTime;
        OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "%d %.2f ms\n", route->altRoutes.size(), hctx->stats.altRoutingTime);
        time = timer.GetElapsedMs();
        for (SHARED_PTR<HHNetworkRouteRes> & alt : route->altRoutes) {
            //retrieveSegmentsGeometry(hctx, rrp, alt, hctx->config->ROUTE_ALL_ALT_SEGMENTS);
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
        //route->detailed = rrp.prepareResult(hctx->rctx, route->detailed).detailed;
    }
    OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "%.2f ms\n", timer.GetElapsedMs() - time);
    //RouteResultPreparation.printResults(hctx.rctx, start, end, route.detailed);
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
                plusCost += hctx->rctx->config->PENALTY_FOR_REVERSE_DIRECTION;
            }
            diff = s->getRoad()->directionRoute(s->getSegmentEnd(), !s->isPositive()) - hctx->rctx->config->initialDirection;
            if (std::abs(alignAngleDifference(diff - M_PI)) <= M_PI / 3) {
                negCost += hctx->rctx->config->PENALTY_FOR_REVERSE_DIRECTION;
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
                NetworkDBPoint * pnt = new NetworkDBPoint();
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

#endif /*_OSMAND_HH_ROUTE_PLANNER_CPP*/
