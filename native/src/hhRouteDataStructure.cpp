#ifndef _OSMAND_HH_ROUTE_DATA_STRUCTURE_CPP
#define _OSMAND_HH_ROUTE_DATA_STRUCTURE_CPP

#include "commonOsmAndCore.h"
#include "hhRouteDataStructure.h"

std::vector<NetworkDBPoint *> DataTileManager::getClosestObjects(double latitude, double longitude, double radius) {
    std::vector<NetworkDBPoint *> res;
    if (isEmpty()) {
        return res;
    }
    double tileDist = radius / getTileDistanceWidth(latitude, zoom);
    int tileDistInt = (int) std::ceil(tileDist);
    double px = getTileNumberX(zoom, longitude);
    double py = getTileNumberY(zoom, latitude);
    int stTileX = (int) px;
    int stTileY = (int) py;
    std::map<int64_t, double> tiles;
    for (int xTile = -tileDistInt; xTile <= tileDistInt; xTile++) {
        for (int yTile = -tileDistInt; yTile <= tileDistInt; yTile++) {
            double dx = xTile + 0.5 - (px - stTileX);
            double dy = yTile + 0.5 - (py - stTileY);
            double dist = std::sqrt(dx * dx + dy * dy);
            if (dist <= tileDist) {
                tiles.insert(std::pair<int64_t, double>(evTile(stTileX + xTile, stTileY + yTile), dist));
            }
        }
    }
    
    std::vector<int64_t> keys;
    for(std::map<int64_t, double>::iterator it = tiles.begin(); it != tiles.end(); ++it) {
      keys.push_back(it->first);
    }
    
    std::sort(keys.begin(), keys.end(), [tiles](const int64_t o1, const int64_t o2) {
        return tiles.at(o1) < tiles.at(o2);
    });
    for (int64_t key : keys) {
        putObjects(key, res);
    }
    return res;
}

NetworkDBSegment * NetworkDBPoint::getSegment(const NetworkDBPoint * target, bool dir) const {
    std::vector<NetworkDBSegment *> l = (dir ? connected : connectedReverse);
    for (NetworkDBSegment * s : l) {
        if (dir && s->end == target) {
            return s;
        } else if (!dir && s->start == target) {
            return s;
        }
    }
    return nullptr;
}

void HHRoutingContext::clearVisited(UNORDERED_map<int64_t, NetworkDBPoint *> & stPoints, UNORDERED_map<int64_t, NetworkDBPoint *> & endPoints) {
    resetAllQueues();
    for (NetworkDBPoint * p : queueAdded) {
        SHARED_PTR<RouteSegment> pos = std::move(p->rt(false)->rtDetailedRoute);
        SHARED_PTR<RouteSegment> rev = std::move(p->rt(true)->rtDetailedRoute);
        p->clearRouting();
        auto itS = stPoints.find(p->index);
        auto itE = endPoints.find(p->index);
        if (itS != stPoints.end() && pos) {
            p->setDistanceToEnd(false, distanceToEnd(false, p));
            p->setDetailedParentRt(false, pos);
        } else if (itE != endPoints.end() && rev) {
            p->setDistanceToEnd(true, distanceToEnd(true, p));
            p->setDetailedParentRt(true, rev);
        }
    }
    queueAdded.clear();
    visited.clear();
    visitedRev.clear();
}

double HHRoutingContext::distanceToEnd(bool reverse, NetworkDBPoint * nextPoint) {
    if (config->HEURISTIC_COEFFICIENT > 0) {
        double distanceToEnd = nextPoint->rt(reverse)->rtDistanceToEnd;
        if (distanceToEnd == 0) {
            double dist = squareRootDist31(reverse ? startX : endX, reverse ? startY : endY,
                                           nextPoint->midX(), nextPoint->midY());
            distanceToEnd = config->HEURISTIC_COEFFICIENT * dist / rctx->config->router->getMaxSpeed();
            nextPoint->setDistanceToEnd(reverse, distanceToEnd);
        }
        return distanceToEnd;
    }
    return 0;
}

#endif /*_OSMAND_HH_ROUTE_DATA_STRUCTURE_CPP*/
