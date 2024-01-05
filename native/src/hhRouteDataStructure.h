#ifndef _OSMAND_HH_ROUTE_DATA_STRUCTURE_H
#define _OSMAND_HH_ROUTE_DATA_STRUCTURE_H

#include "CommonCollections.h"
#include "routeCalcResult.h"
#include "routingContext.h"
#include "NetworkDBPointRouteInfo.h"
#include <set>
#include <queue>

struct HHRoutingConfig
{
	float HEURISTIC_COEFFICIENT = 0; // A* - 1, Dijkstra - 0
	float DIJKSTRA_DIRECTION = 0; // 0 - 2 directions, 1 - positive, -1 - reverse
			
	double INITIAL_DIRECTION;
	static const int CALCULATE_ALL_DETAILED = 3;
    
    int FULL_DIJKSTRA_NETWORK_RECALC = 10;
    double MAX_INC_COST_CF = 1.25;
    int MAX_START_END_REITERATIONS = 50;
			
	bool ROUTE_LAST_MILE = false;
	bool ROUTE_ALL_SEGMENTS = false;
	bool ROUTE_ALL_ALT_SEGMENTS = false;
	bool PRELOAD_SEGMENTS = false;
			
	bool CALC_ALTERNATIVES = false;
	bool USE_GC_MORE_OFTEN = false;
	// TODO 3.1 HHRoutePlanner Alternative routes - could use distributions like 50% route (2 alt), 25%/75% route (1 alt)
	double ALT_EXCLUDE_RAD_MULT = 0.3; // radius multiplier to exclude points
	double ALT_EXCLUDE_RAD_MULT_IN = 3; // skip some points to speed up calculation
	double ALT_NON_UNIQUENESS = 0.7; // 0.7 - 30% of points must be unique
			
	double MAX_COST;
	int MAX_DEPTH = -1; // max depth to go to
	int MAX_SETTLE_POINTS = -1; // max points to settle
			
	bool USE_CH;
	bool USE_CH_SHORTCUTS;

	bool USE_MIDPOINT;
	int MIDPOINT_ERROR = 3;
	int MIDPOINT_MAX_DEPTH = 20 + MIDPOINT_ERROR;
    double MAX_TIME_REITERATION_MS = 60000;
	
	HHRoutingConfig() {}
	
	static SHARED_PTR<HHRoutingConfig> dijkstra(int direction) {
		auto df = std::make_shared<HHRoutingConfig>();
		df->HEURISTIC_COEFFICIENT = 0;
		df->DIJKSTRA_DIRECTION = direction;
		return df;
	}
			
	static SHARED_PTR<HHRoutingConfig> astar(int direction) {
		auto df = std::make_shared<HHRoutingConfig>();
		df->HEURISTIC_COEFFICIENT = 1;
		df->DIJKSTRA_DIRECTION = direction;
		return df;
	}
			
	static SHARED_PTR<HHRoutingConfig> ch() {
		auto df = std::make_shared<HHRoutingConfig>();
		df->HEURISTIC_COEFFICIENT = 0;
		df->USE_CH = true;
		df->USE_CH_SHORTCUTS = true;
		df->DIJKSTRA_DIRECTION = 0;
		return df;
	}
			
	static SHARED_PTR<HHRoutingConfig> midPoints(bool astar, int dir) {
		auto df = std::make_shared<HHRoutingConfig>();
		df->HEURISTIC_COEFFICIENT = astar ? 1 : 0;
		df->USE_MIDPOINT = true;
		df->DIJKSTRA_DIRECTION = dir;
		return df;
	}
	
	void preloadSegments() {
		PRELOAD_SEGMENTS = true;
	}
			
	void calcAlternative() {
        CALC_ALTERNATIVES = true;
	}
			
	void calcDetailed(int segments) {
        ROUTE_LAST_MILE = true;
        ROUTE_ALL_SEGMENTS = segments >= 1;
		ROUTE_ALL_ALT_SEGMENTS = segments >= 2;
    }
			
	void useShortcuts() {
        USE_CH_SHORTCUTS = true;
    }
			
	void gc() {
        USE_GC_MORE_OFTEN = true;
    }
			
    void maxCost(double cost) {
		MAX_COST = cost;
    }
			
    void maxDepth(int depth) {
		MAX_DEPTH = depth;
    }
			
    void maxSettlePoints(int maxPoints) {
        MAX_SETTLE_POINTS = maxPoints;
    }
};

struct NetworkDBSegment;
struct NetworkDBPoint {
    NetworkDBPoint * dualPoint;
    int64_t index;
    int clusterId;
    int fileId;
    short mapId;
            
    int64_t roadId;
    short start;
    short end;
    int startX;
    int startY;
    int endX;
    int endY;
    bool rtExclude;
    
    SHARED_PTR<NetworkDBPointRouteInfo> rtRev;
    SHARED_PTR<NetworkDBPointRouteInfo> rtPos;
    std::vector<NetworkDBSegment *> connected;
    std::vector<NetworkDBSegment *> connectedReverse;
    
    void clearRouting() {
        rtExclude = false;
        rtPos = nullptr;
        rtRev = nullptr;
    }
    
    SHARED_PTR<NetworkDBPointRouteInfo> rt(bool rev) {
        if (rev) {
            if (rtRev == nullptr) {
                rtRev = std::make_shared<NetworkDBPointRouteInfo>();
            }
            return rtRev;
        } else {
            if (rtPos == nullptr) {
                rtPos = std::make_shared<NetworkDBPointRouteInfo>();
            }
            return rtPos;
        }
    }
    
    void markSegmentsNotLoaded() {
        connected.clear();
        connectedReverse.clear();
    }
    
    LatLon getPoint() {
        LatLon l(get31LatitudeY(startY / 2 + endY / 2), get31LongitudeX(startX / 2 + endX / 2));
        return l;
    }
    
    void setCostParentRt(bool reverse, double cost, NetworkDBPoint * point, double segmentDist) {
        rt(reverse)->setCostParentRt(reverse, cost, point, segmentDist);
    }
    
    int midX() {
        return startX / 2 + endX / 2;
    }
            
    int midY() {
        return startY / 2 + endY/ 2;
    }
    
    void setDistanceToEnd(bool rev, double segmentDist) {
        rt(rev)->rtDistanceToEnd = segmentDist;
    }
    
    void setDetailedParentRt(bool rev, SHARED_PTR<RouteSegment> r) {
        rt(rev)->setDetailedParentRt(r);
    }
    
    NetworkDBSegment * getSegment(const NetworkDBPoint * target, bool dir) const;
    
    std::vector<NetworkDBSegment *> conn(bool rev) {
        return rev ? connectedReverse : connected;
    }
    
    void connectedSet(bool rev, std::vector<NetworkDBSegment *> l) {
        if (rev) {
            connectedReverse = l;
            } else {
                connected = l;
            }
    }
    
    int chInd() {
        return 0;
    }
    
    int midPntDepth() {
        return 0;
    }
    
    bool operator == (const NetworkDBPoint & that) const {
        if (index != that.index || clusterId != that.clusterId || fileId != that.fileId || mapId != that.mapId ||
            roadId != that.roadId || start != that.start || end != that.end || startX != that.startX ||
            endX != that.endX || endY != that.endY || rtExclude != that.rtExclude) {
            return false;
        }
        if (dualPoint != that.dualPoint) {
            return false;
        }
        return true;
    }
};

struct NetworkDBSegment {
    const bool direction;
    NetworkDBPoint * start;
    NetworkDBPoint * end;
    const bool shortcut;
    double dist;
    //List<LatLon> geom;
    
    NetworkDBSegment(NetworkDBPoint * start, NetworkDBPoint * end, double dist, bool direction, bool shortcut):
        direction(direction), start(start), end(end), shortcut(shortcut), dist(dist) {
    }
    
    /*public List<LatLon> getGeometry() {
        if (geom == null) {
            geom = new ArrayList<LatLon>();
        }
        return geom;
    }
    
    @Override
    public String toString() {
        return String.format("Segment %s -> %s [%.2f] %s", start, end, dist, shortcut ? "sh" : "bs");
    }*/
};

struct HHNetworkSegmentRes {
    NetworkDBSegment * segment;
    std::vector<SHARED_PTR<RouteSegmentResult>> list;
    double rtTimeDetailed;
    double rtTimeHHSegments;
    
    HHNetworkSegmentRes(NetworkDBSegment * s): segment(s) {
    }
};

struct RoutingStats {
    int firstRouteVisitedVertices = 0;
    int visitedVertices = 0;
    int uniqueVisitedVertices = 0;
    int addedVertices = 0;

    double loadPointsTime = 0;
    int loadEdgesCnt;
    double loadEdgesTime = 0;
    double altRoutingTime;
    double routingTime = 0;
    double searchPointsTime = 0;
    double addQueueTime = 0;
    double pollQueueTime = 0;
    double prepTime = 0;
};

struct HHNetworkRouteRes : public RouteCalcResult {
    RoutingStats stats;
    std::vector<HHNetworkSegmentRes> segments;
    std::vector<HHNetworkRouteRes *> altRoutes;
    std::set<int64_t> uniquePoints;
    
    HHNetworkRouteRes(): RouteCalcResult() {        
    }
    
    HHNetworkRouteRes(std::string error): RouteCalcResult(error) {
    }
    
    double getHHRoutingTime() {
        double d = 0;
        for (auto & r : segments) {
            d += r.rtTimeHHSegments;
        }
        return d;
    }
    
    double getHHRoutingDetailed() {
        double d = 0;
        for (auto & r : segments) {
            d += r.rtTimeDetailed;
        }
        return d;
    }

    void append(SHARED_PTR<HHNetworkRouteRes> res) {
        if (!res || !res->error.empty()) {
            error = "Can't build a route with intermediate point";
        } else {
            detailed.insert(detailed.end(), res->detailed.begin(), res->detailed.end());
            segments.insert(segments.end(), res->segments.begin(), res->segments.end());
            altRoutes.clear();
            uniquePoints.clear();
        }
    }    
};

struct HHRouteRegionPointsCtx {
    short id;
    SHARED_PTR<HHRouteIndex> fileRegion;
    BinaryMapFile* file;
    int32_t routingProfile = 0;
    UNORDERED_map<int64_t, NetworkDBPoint *> pntsByFileId;
    
    HHRouteRegionPointsCtx(short id): id(id), fileRegion(nullptr), file(nullptr) {
    }
    
    HHRouteRegionPointsCtx(short id, SHARED_PTR<HHRouteIndex> fileRegion, BinaryMapFile* file, int rProf) {
        this->id = id;
        this->fileRegion = fileRegion;
        this->file = file;
        if (routingProfile >= 0) {
            routingProfile = rProf;
        }
    }
    
    int32_t getRoutingProfile() {
        return routingProfile;
    }
    
    SHARED_PTR<HHRouteIndex> getFileRegion() {
        return fileRegion;
    }
    
    
    NetworkDBPoint * getPoint(int pntFileId) {
        auto pos = pntsByFileId.find(pntFileId);
        if (pos != pntsByFileId.end()) {
            return pos->second;
        }
        return nullptr;
    }
};

struct NetworkDBPointCost {
    NetworkDBPoint * point;
    const double cost;
    const bool rev;
    
    NetworkDBPointCost(NetworkDBPoint * p, double cost, bool rev): point(p), cost(cost), rev(rev) {
    }
    
};

//TODO check Double.compare
struct HHPointComparator : public std::function<bool(SHARED_PTR<NetworkDBPointCost>&, SHARED_PTR<NetworkDBPointCost>&)> {
    bool operator()(const SHARED_PTR<NetworkDBPointCost>& o1, const SHARED_PTR<NetworkDBPointCost>& o2) {
        if (o1->cost > o2->cost) {
            return true;
        }
        return false;
    }
};

struct DataTileManager {
    const int zoom;
    UNORDERED_map<int64_t, std::vector<NetworkDBPoint *>> objects;
    
    DataTileManager(): zoom(15) {
    }
    
    DataTileManager(int z): zoom(z) {
    }
    
    int64_t registerObject(double latitude, double longitude, NetworkDBPoint * object) {
        int64_t tile = evaluateTile(latitude, longitude);
        return addObject(object, tile);
    }
    
    void printStatsDistribution(std::string name) {
        int min = -1, max = -1, total = 0;
        UNORDERED_map<int64_t, std::vector<NetworkDBPoint *>>::iterator it;
        for (it = objects.begin(); it != objects.end(); it++) {
            auto & l = it->second;
            if (min == -1) {
                max = min = (int) l.size();
            } else {
                min = std::min(min, (int) l.size());
                max = std::max(max, (int) l.size());
            }
            total += l.size();
        }
        OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "%s tiles stores %d in %d tiles. Tile size min %d, max %d, avg %.2f.\n ",
                          name.c_str(), total, objects.size(), min, max, total / (objects.size() + 0.1));
    }
    
    std::vector<NetworkDBPoint *> getClosestObjects(double latitude, double longitude, double radius);
    
private:
    int64_t evTile(int32_t tileX, int32_t tileY) {
        return ((int64_t) (tileX) << zoom) + tileY;
    }
    
    int64_t evaluateTile(double latitude, double longitude) {
        int tileX = (int) getTileNumberX(zoom, longitude);
        int tileY = (int) getTileNumberY(zoom, latitude);
        return evTile(tileX, tileY);
    }
    
    int64_t addObject(NetworkDBPoint * object, int64_t tile) {
        auto it = objects.find(tile);
        if (it == objects.end()) {
            std::vector<NetworkDBPoint *> v;
            objects.insert(std::pair<int64_t, std::vector<NetworkDBPoint *>>(tile, v));
        }
        objects[tile].push_back(object);
        return tile;
    }
    
    bool isEmpty() {
        return getObjectsCount() == 0;
    }
    
    int getObjectsCount() {
        int x = 0;
        UNORDERED_map<int64_t, std::vector<NetworkDBPoint *>>::iterator it;
        for (it = objects.begin(); it != objects.end(); it++) {
            x += it->second.size();
        }
        return x;
    }
    
    void putObjects(int64_t t, std::vector<NetworkDBPoint *> & r) {
        auto it = objects.find(t);
        if (it != objects.end()) {
            r.insert(r.end(), it->second.begin(), it->second.end());
        }
    }
};

typedef priority_queue<SHARED_PTR<NetworkDBPointCost>, vector<SHARED_PTR<NetworkDBPointCost>>, HHPointComparator> HH_QUEUE;

struct HHRoutingContext {
    bool USE_GLOBAL_QUEUE = false;
    
    SHARED_PTR<RoutingContext> rctx;
    std::vector<SHARED_PTR<HHRouteRegionPointsCtx>> regions;
    RoutingStats stats;
    SHARED_PTR<HHRoutingConfig> config;
    int32_t startX;
    int32_t startY;
    int32_t endY;
    int32_t endX;
    bool initialized;
    
    std::vector<NetworkDBPoint *> queueAdded;
    std::vector<NetworkDBPoint *> visited;
    std::vector<NetworkDBPoint *> visitedRev;
    
    UNORDERED_map<int64_t, NetworkDBPoint *> pointsById;
    UNORDERED_map<int64_t, NetworkDBPoint *> pointsByGeo;
    UNORDERED_map<int64_t, SHARED_PTR<RouteSegment>> boundaries;
    UNORDERED_map<int64_t, std::vector<NetworkDBPoint *>> clusterInPoints;
    UNORDERED_map<int64_t, std::vector<NetworkDBPoint *>> clusterOutPoints;
    
    DataTileManager pointsRect;
    
    HHRoutingContext(): pointsRect(11) {
        queueGlobal = createQueue();
        queuePos = createQueue();
        queueRev = createQueue();
        initialized = false;
    }
    
    //TODO add destructor
    
    SHARED_PTR<HH_QUEUE> createQueue() {
        HHPointComparator comparator;
        SHARED_PTR<HH_QUEUE> queue = std::make_shared<HH_QUEUE>(comparator);
        return queue;
    }
    
    SHARED_PTR<HH_QUEUE> queue(bool rev) {
        return USE_GLOBAL_QUEUE ? queueGlobal : (rev ? queueRev : queuePos);
    }
    
    void clearVisited() {
        queue(true).reset();
        queue(false).reset();
        for (auto & p : queueAdded) {
            p->clearRouting();
        }
        queueAdded.clear();
        visited.clear();
        visitedRev.clear();
    }
    
    void clearVisited(UNORDERED_map<int64_t, NetworkDBPoint *> & stPoints, UNORDERED_map<int64_t, NetworkDBPoint *> & endPoints);
    
    UNORDERED_map<int64_t, NetworkDBPoint *> loadNetworkPoints() {
        UNORDERED_map<int64_t, NetworkDBPoint *> points;
        for (auto & r : regions) {
            if (r->file != nullptr) {
                initHHPoints(r->file, r->fileRegion, r->id, points);
            }
        }
        return points;
    }
    
    std::string getRoutingInfo() {
        std::string b;
        for (auto & r : regions) {
            if(b.length() > 0) {
                b.append(", ");
            }
            if (r->fileRegion != nullptr) {
                b.append(/*r->file.getFile().getName() + " " + */r->fileRegion->profile + " [" +
                        r->fileRegion->profileParams.at(r->routingProfile) + "]");
            } else {
                b.append("unknown");
            }
        }
        return b;
    }
    
    std::vector<NetworkDBPoint *> getIncomingPoints(NetworkDBPoint * point) {
        auto it = clusterInPoints.find(point->clusterId);
        return it->second;
    }
    
    std::vector<NetworkDBPoint *> getOutgoingPoints(NetworkDBPoint * point) {
        auto it = clusterOutPoints.find(point->clusterId);
        return it->second;
    }
    
    static bool checkId(int id, HHRouteBlockSegments * s) {
        return s->idRangeStart <= id && s->idRangeStart + s->idRangeLength > id;
    }
    
    int32_t loadNetworkSegmentPoint(NetworkDBPoint * point, bool reverse) {
        if (point->conn(reverse).size() > 0) {
            return 0;
        }
        short mapId = point->mapId;
        if (mapId < regions.size()) {
            SHARED_PTR<HHRouteRegionPointsCtx> & r = regions[mapId];
            SHARED_PTR<HHRouteIndex> & fileRegion = r->fileRegion;
            for (auto * s : fileRegion->segments) {
                if (s->profileId == r->getRoutingProfile() && checkId(point->fileId, s)) {
                    return ::loadNetworkSegmentPoint(this, r, s, point->fileId);
                }
            }
        }
        return 0;
    }
    
private:
    SHARED_PTR<HH_QUEUE> queueGlobal;
    SHARED_PTR<HH_QUEUE> queuePos;
    SHARED_PTR<HH_QUEUE> queueRev;
};

struct HHRouteRegionsGroup {
    std::vector<SHARED_PTR<HHRouteIndex>> regions;
    std::vector<BinaryMapFile*> readers;
    const long edition;
    const std::string profileParams;
    
    int extraParam = 0;
    int matchParam = 0;
    bool containsStartEnd;
    
    HHRouteRegionsGroup(): edition(-1), profileParams() {
    }
    
    HHRouteRegionsGroup(long edition, std::string params): edition(edition), profileParams(params) {
    }
    
   void appendToGroups(SHARED_PTR<HHRouteIndex> r, BinaryMapFile* rdr, std::vector<SHARED_PTR<HHRouteRegionsGroup>> groups) {
        for (std::string & params : r->profileParams) {
            SHARED_PTR<HHRouteRegionsGroup> matchGroup = nullptr;
            for (auto & g : groups) {
                if (g->edition == r->edition && params == g->profileParams) {
                    matchGroup = g;
                    break;
                }
            }
            if (matchGroup == nullptr) {
                matchGroup = std::make_shared<HHRouteRegionsGroup>(r->edition, params);
                groups.push_back(matchGroup);
            }
            matchGroup->regions.push_back(r);
            matchGroup->readers.push_back(rdr);
        }
    }

    bool contains(int x, int y) {
        bool contains = false;
        for (auto & r : regions) {
            if (r->top->contains(x, y)) {
                contains = true;
                break;
            }
        }
        return contains;
    }
};

#endif /*_OSMAND_HH_ROUTE_DATA_STRUCTURE_H*/
