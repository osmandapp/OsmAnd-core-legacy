#ifndef _OSMAND_BINARY_ROUTE_PLANNER_H
#define _OSMAND_BINARY_ROUTE_PLANNER_H
#include "CommonCollections.h"
#include "commonOsmAndCore.h"

static const int REVERSE_WAY_RESTRICTION_ONLY = 1024;

static const int ROUTE_POINTS = 11;
// static const float TURN_DEGREE_MIN = 45;
static const short RESTRICTION_NO_RIGHT_TURN = 1;
static const short RESTRICTION_NO_LEFT_TURN = 2;
static const short RESTRICTION_NO_U_TURN = 3;
static const short RESTRICTION_NO_STRAIGHT_ON = 4;
static const short RESTRICTION_ONLY_RIGHT_TURN = 5;
static const short RESTRICTION_ONLY_LEFT_TURN = 6;
static const short RESTRICTION_ONLY_STRAIGHT_ON = 7;

struct RoutingContext;
struct RouteSegmentResult;
struct RouteSegmentPoint;
struct RouteSegment;

typedef UNORDERED_map<int64_t, SHARED_PTR<RouteSegment> > VISITED_MAP;

// typedef std::pair<int, std::pair<string, string> > ROUTE_TRIPLE;

SHARED_PTR<RouteSegmentPoint> findRouteSegment(int px, int py, RoutingContext* ctx, bool transportStop = false,
											   int64_t roadId = -1, int segmentInd = 0);

vector<SHARED_PTR<RouteSegmentResult> > searchRouteInternal(RoutingContext* ctx, bool leftSideNavigation);
vector<SHARED_PTR<RouteSegment>> searchRouteInternal(RoutingContext* ctx, SHARED_PTR<RouteSegmentPoint> start,
													 SHARED_PTR<RouteSegmentPoint> end, const VISITED_MAP & boundaries, std::vector<int64_t> excludedKeys);

bool checkMovementAllowed(RoutingContext* ctx, bool reverseWaySearch, const SHARED_PTR<RouteSegment>& segment);
bool containsKey(VISITED_MAP& visited, int64_t routePointId);
SHARED_PTR<RouteSegment> createNull();
float calculatePreciseStartTime(const RoutingContext* ctx, int projX, int projY, const SHARED_PTR<RouteSegment>& seg);
class GeneralRouter;
float calcRoutingSegmentTimeOnlyDist(const SHARED_PTR<GeneralRouter>& router, const SHARED_PTR<RouteSegment>& segment);
vector<SHARED_PTR<RouteSegmentResult>> convertFinalSegmentToResults(RoutingContext* ctx, const SHARED_PTR<RouteSegment>& finalSegment);
void attachConnectedRoads(RoutingContext* ctx, vector<SHARED_PTR<RouteSegmentResult>>& res);

#endif /*_OSMAND_BINARY_ROUTE_PLANNER_H*/
