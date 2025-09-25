#ifndef _OSMAND_HH_ROUTE_PLANNER_H
#define _OSMAND_HH_ROUTE_PLANNER_H

#include "CommonCollections.h"
#include "hhRouteDataStructure.h"
#include "routingContext.h"

typedef UNORDERED_map<int64_t, std::vector<NetworkDBPoint *>> MAP_VECTORS_NETWORK_DB_POINTS;

class HHRoutePlanner {
		
public:
	HHRoutePlanner(RoutingContext * ctx);
	
	int DEBUG_VERBOSE_LEVEL = 0;
	int DEBUG_ALT_ROUTE_SELECTION = -1;
	bool ASSERT_COST_INCREASING = false;
	bool ASSERT_AND_CORRECT_DIST_SMALLER = true;
	
	constexpr static const double MINIMAL_COST = 0.01;
	static const int PNT_SHORT_ROUTE_START_END;
	static const int MAX_POINTS_CLUSTER_ROUTING;
	static const int ROUTE_POINTS;
	constexpr static const double MAX_INC_COST_CORR = 10.0;
	// this constant should dynamically change if route is not found
	constexpr static const double EXCLUDE_PRIORITY_CONSTANT = 0.0; // see comments below
	
	HHRoutingConfig * prepareDefaultRoutingConfig(HHRoutingConfig * c);
	HHNetworkRouteRes * runRouting(int startX, int startY, int endX, int endY, HHRoutingConfig * config);
	
private:
	double smallestSegmentCost(const SHARED_PTR<HHRoutingContext> & hctx, NetworkDBPoint * st, NetworkDBPoint * end) const;
	bool retrieveSegmentsGeometry(const SHARED_PTR<HHRoutingContext> & hctx, HHNetworkRouteRes * route, bool routeSegments, SHARED_PTR<RouteCalculationProgress> progress);
	bool retainAll(std::set<int64_t> & d, const std::set<int64_t> & s) const;
	int64_t calcRPId(const SHARED_PTR<RouteSegmentPoint> & p, int pntId, int nextPntId) const;
	int64_t calcRPId(const SHARED_PTR<RouteSegment> & p, int pntId, int nextPntId) const;
	int64_t calculateRoutePointInternalId(int64_t id, int32_t pntId, int32_t nextPntId) const;
	int64_t calculateRoutePointInternalId(const SHARED_PTR<RouteDataObject> & road, int32_t pntId, int32_t nextPntId) const;
	int64_t calcUniDirRoutePointInternalId(const SHARED_PTR<RouteSegmentPoint> & segm) const;
	std::string toString(GeneralRouterProfile grp);
	SHARED_PTR<HHRoutingContext> currentCtx;
	SHARED_PTR<HHRoutingContext> initHCtx(HHRoutingConfig * c, int startX, int startY, int endX, int endY);
	SHARED_PTR<HHRoutingContext> initNewContext(RoutingContext * ctx, std::vector<SHARED_PTR<HHRouteRegionPointsCtx>> & regions);
	SHARED_PTR<HHRoutingContext> selectBestRoutingFiles(int startX, int startY, int endX, int endY, const SHARED_PTR<HHRoutingContext> & hctx, bool strictBestGroupMaps);
	SHARED_PTR<RouteSegmentPoint> loadPoint(RoutingContext * ctx, const NetworkDBPoint * pnt);
	SHARED_PTR<HHRouteRegionsGroup> hhRouteRegionGroup;
	void findFirstLastSegments(const SHARED_PTR<HHRoutingContext> & hctx, int startX, int startY, int endX, int endY,
							   UNORDERED_map<int64_t, NetworkDBPoint *> & stPoints,
							   UNORDERED_map<int64_t, NetworkDBPoint *> & endPoints);
	void recalculateNetworkCluster(const SHARED_PTR<HHRoutingContext> & hctx, NetworkDBPoint * start);
	MAP_VECTORS_NETWORK_DB_POINTS groupByClusters(UNORDERED_map<int64_t, NetworkDBPoint *> & pointsById, bool out);
	UNORDERED_map<int64_t, NetworkDBPoint *> initStart(const SHARED_PTR<HHRoutingContext> & hctx, const SHARED_PTR<RouteSegmentPoint> & s,
											bool reverse, UNORDERED_map<int64_t, NetworkDBPoint *> & pnts);
	HHNetworkRouteRes * createRouteSegmentFromFinalPoint(const SHARED_PTR<HHRoutingContext> & hctx, NetworkDBPoint * pnt);
	HHNetworkRouteRes * prepareRouteResults(const SHARED_PTR<HHRoutingContext> & hctx, HHNetworkRouteRes * route, int startX, int startY, int endX, int endY);
	std::vector<SHARED_PTR<RouteSegment>> runDetailedRouting(const SHARED_PTR<HHRoutingContext> & hctx, const NetworkDBPoint * startS, const NetworkDBPoint * endS, bool useBoundaries);
	
	NetworkDBPoint * runRoutingWithInitQueue(const SHARED_PTR<HHRoutingContext> & hctx);
	NetworkDBPoint * scanFinalPoint(NetworkDBPoint * finalPoint, std::vector<NetworkDBPoint *> & lt);
	NetworkDBPoint * runRoutingPointsToPoints(const SHARED_PTR<HHRoutingContext> & hctx,
											  UNORDERED_map<int64_t, NetworkDBPoint *> & stPoints,
											  UNORDERED_map<int64_t, NetworkDBPoint *> & endPoints);
	void addPointToQueue(const SHARED_PTR<HHRoutingContext> & hctx, SHARED_PTR<HH_QUEUE> queue, bool reverse,
						 NetworkDBPoint * point, NetworkDBPoint * parent, double segmentDist, double cost);
	void addConnectedToQueue(const SHARED_PTR<HHRoutingContext> & hctx, SHARED_PTR<HH_QUEUE> & queue, NetworkDBPoint * point, bool reverse);
	void printPoint(NetworkDBPoint * p, bool rev) const;
	void calcAlternativeRoute(const SHARED_PTR<HHRoutingContext> & hctx, HHNetworkRouteRes * route,
							  UNORDERED_map<int64_t, NetworkDBPoint *> & stPoints, UNORDERED_map<int64_t,
							  NetworkDBPoint *> & endPoints, SHARED_PTR<RouteCalculationProgress> & progress);
	HHNetworkRouteRes * cancelledStatus() const;
	void filterPointsBasedOnConfiguration(const SHARED_PTR<HHRoutingContext> & hctx);
	
};

#endif /*_OSMAND_HH_ROUTE_PLANNER_H*/
