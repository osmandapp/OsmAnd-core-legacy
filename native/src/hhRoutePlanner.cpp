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

UNORDERED_map<int64_t, std::vector<NetworkDBPoint *>> HHRoutePlanner::groupByClusters( UNORDERED_map<int64_t, NetworkDBPoint *> pointsById, bool out) {
    UNORDERED_map<int64_t, std::vector<NetworkDBPoint *>> res;
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
    
    UNORDERED_map<int64_t, std::vector<NetworkDBPoint *>>::iterator it2;
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
    //timer.GetElapsedMs();
    config = prepareDefaultRoutingConfig(config);
    SHARED_PTR<HHRoutingContext> hctx = initHCtx(config, startX, startY, endX, endY);
    if (hctx == nullptr) {
        HHNetworkRouteRes res("Files for hh routing were not initialized. Route couldn't be calculated.");
        return res;
    }
    OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "Routing %.5f %.5f -> %.5f %.5f (HC %d, dir %d)",
                      get31LatitudeY(startY), get31LongitudeX(startX),
                      get31LatitudeY(endY), get31LongitudeX(endX),
                      (int) config->HEURISTIC_COEFFICIENT, (int) config->DIJKSTRA_DIRECTION);
    //TLongObjectHashMap<T> stPoints = new TLongObjectHashMap<>(), endPoints = new TLongObjectHashMap<>();
    //findFirstLastSegments(hctx, start, end, stPoints, endPoints);
    
    
    /*
     
            TLongObjectHashMap<T> stPoints = new TLongObjectHashMap<>(), endPoints = new TLongObjectHashMap<>();
            findFirstLastSegments(hctx, start, end, stPoints, endPoints);

            RouteResultPreparation rrp = new RouteResultPreparation();*/
            HHNetworkRouteRes route;
            /*while (route == null) {
                System.out.printf("Routing...");
                long time = System.nanoTime();
                NetworkDBPoint finalPnt = runRoutingPointsToPoints(hctx, stPoints, endPoints);
                route = createRouteSegmentFromFinalPoint(hctx, finalPnt);
                time = (System.nanoTime() - time) ;
                System.out.printf("%d segments, cost %.2f, %.2f ms\n", route.segments.size(), route.getHHRoutingTime(), time / 1e6);
                hctx.stats.routingTime+= time/ 1e6;

                System.out.printf("Parse detailed route segments...");
                time = System.nanoTime();
                boolean recalc = retrieveSegmentsGeometry(hctx, rrp, route, hctx.config.ROUTE_ALL_SEGMENTS);
                time = (System.nanoTime() - time);
                System.out.printf("%.2f ms\n", time / 1e6);
                hctx.stats.routingTime += time / 1e6;
                if (recalc) {
                    if (hctx.stats.prepTime + hctx.stats.routingTime > hctx.config.MAX_TIME_REITERATION_MS) {
                        return new HHNetworkRouteRes("Too many route recalculations (maps are outdated).");
                    }
                    hctx.clearVisited(stPoints, endPoints);
                    route = null;
                }
            }
            
            if (hctx.config.CALC_ALTERNATIVES) {
                System.out.printf("Alternative routes...");
                long time = System.nanoTime();
                calcAlternativeRoute(hctx, route, stPoints, endPoints);
                hctx.stats.altRoutingTime += (System.nanoTime() - time) / 1e6;
                hctx.stats.routingTime += hctx.stats.altRoutingTime;
                System.out.printf("%d %.2f ms\n", route.altRoutes.size(), hctx.stats.altRoutingTime);

                time = System.nanoTime();
                for (HHNetworkRouteRes alt : route.altRoutes) {
                    retrieveSegmentsGeometry(hctx, rrp, alt, hctx.config.ROUTE_ALL_ALT_SEGMENTS);
                }
                hctx.stats.prepTime += (System.nanoTime() - time) / 1e6;
            }

            if (hctx.config.USE_GC_MORE_OFTEN) {
                hctx.unloadAllConnections();
                printGCInformation();
            }
            
            long time = System.nanoTime();
            prepareRouteResults(hctx, route, start, end, rrp);
            hctx.stats.prepTime += (System.nanoTime() - time) / 1e6;
            
            System.out.printf("%.2f ms\n", hctx.stats.prepTime);
            if (DEBUG_VERBOSE_LEVEL >= 1) {
                System.out.println("Detailed progress: " + hctx.rctx.calculationProgress.getInfo(null));
            }
            
            System.out.println(String.format("Found final route - cost %.2f (detailed %.2f, %.1f%%), %d depth ( first met %,d, visited %,d (%,d unique) of %,d added vertices )",
                    route.getHHRoutingTime(), route.getHHRoutingDetailed(), 100 * (1 - route.getHHRoutingDetailed() / route.getHHRoutingTime()),
                    route.segments.size(), hctx.stats.firstRouteVisitedVertices, hctx.stats.visitedVertices, hctx.stats.uniqueVisitedVertices, hctx.stats.addedVertices));
            
            time = System.nanoTime();
            System.out.println(hctx.config.toString(start, end));
            System.out.printf("Calculate turns...");
            
            if (hctx.config.ROUTE_ALL_SEGMENTS && route.detailed != null) {
                route.detailed = rrp.prepareResult(hctx.rctx, route.detailed).detailed;
            }
            System.out.printf("%.2f ms\n", (System.nanoTime() - time) / 1e6);
            RouteResultPreparation.printResults(hctx.rctx, start, end, route.detailed);
            
            System.out.printf("Routing finished all %.1f ms: last mile %.1f ms, load data %.1f ms (%,d edges), routing %.1f ms (queue  - %.1f add ms + %.1f poll ms), prep result %.1f ms\n",
                    (System.nanoTime() - startTime) / 1e6, hctx.stats.searchPointsTime,
                    hctx.stats.loadEdgesTime + hctx.stats.loadPointsTime, hctx.stats.loadEdgesCnt, hctx.stats.routingTime,
                    hctx.stats.addQueueTime, hctx.stats.pollQueueTime, hctx.stats.prepTime);
            printGCInformation();*/
    return route;
}

void HHRoutePlanner::findFirstLastSegments(SHARED_PTR<HHRoutingContext> hctx, int startX, int startY, int endX, int endY,
                                           UNORDERED_map<int64_t, std::vector<NetworkDBPoint *>> stPoints,
                                           UNORDERED_map<int64_t, std::vector<NetworkDBPoint *>> endPoints) {
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
        UNORDERED_map<int64_t, std::vector<NetworkDBPoint *>>::iterator it;
        for (it = stPoints.begin(); it != stPoints.end(); it++) {
            for (auto & p : it->second) {
                p->clearRouting();
            }
        }
        stPoints.clear();
        for (it = endPoints.begin(); it != endPoints.end(); it++) {
            for (auto & p : it->second) {
                p->clearRouting();
            }
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
        //initStart(hctx, startP, false, stPoints);
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

//        hctx->boundaries.remove(calcRPId(endP, endP.getSegmentEnd(), endP.getSegmentStart()));
//        hctx->boundaries.remove(calcRPId(endP, endP.getSegmentStart(), endP.getSegmentEnd()));
//        if (stPoints.containsKey(PNT_SHORT_ROUTE_START_END)) {
//            endPoints.put(PNT_SHORT_ROUTE_START_END, stPoints.get(PNT_SHORT_ROUTE_START_END));
//        }
//        initStart(hctx, endP, true, endPoints);
//        if (endPoints.isEmpty()) {
//            System.out.println("Reiterate with next end point: " + endP);
//            endReiterate++;
//            found = false;
//            continue;
//        }
//        found = true;
        
    }
    //hctx.stats.searchPointsTime = (System.nanoTime() - time) / 1e6;
    //System.out.printf("Finding first / last segments...%.2f ms\n", hctx.stats.searchPointsTime);
}

int64_t HHRoutePlanner::calcRPId(SHARED_PTR<RouteSegmentPoint> p, int pntId, int nextPntId) {
    return calculateRoutePointInternalId(p->getRoad()->getId(), pntId, nextPntId);
}

#endif /*_OSMAND_HH_ROUTE_PLANNER_CPP*/
