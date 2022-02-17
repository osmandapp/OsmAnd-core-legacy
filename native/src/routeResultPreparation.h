//  git revision ab36d0ce1a089ab1cf8dd8c582ac9e85cd77bcff

#ifndef _OSMAND_ROUTE_RESULT_PREPARATION_H
#define _OSMAND_ROUTE_RESULT_PREPARATION_H
#include "CommonCollections.h"
#include "commonOsmAndCore.h"

const string UNMATCHED_HIGHWAY_TYPE = "unmatched";

struct RouteSegmentResult;
struct FinalRouteSegment;
struct CombineAreaRoutePoint;

bool segmentLineBelongsToPolygon(CombineAreaRoutePoint& p, CombineAreaRoutePoint& n,
                                 vector<CombineAreaRoutePoint>& originalWay);
void simplifyAreaRouteWay(vector<CombineAreaRoutePoint>& routeWay, vector<CombineAreaRoutePoint>& originalWay);
void combineWayPointsForAreaRouting(RoutingContext* ctx, vector<SHARED_PTR<RouteSegmentResult> >& result);

void printResults(RoutingContext* ctx, int startX, int startY, int endX, int endY, vector<SHARED_PTR<RouteSegmentResult> >& result);
vector<SHARED_PTR<RouteSegmentResult> > prepareResult(RoutingContext* ctx, SHARED_PTR<FinalRouteSegment> finalSegment);
vector<SHARED_PTR<RouteSegmentResult> > prepareResult(RoutingContext* ctx, vector<SHARED_PTR<RouteSegmentResult> >& result);

vector<int> parseTurnLanes(const SHARED_PTR<RouteDataObject>& ro, double dirToNorthEastPi);
vector<int> parseLanes(const SHARED_PTR<RouteDataObject>& ro, double dirToNorthEastPi);
void prepareTurnResults(RoutingContext* ctx, vector<SHARED_PTR<RouteSegmentResult> >& result);
SHARED_PTR<TurnType> getTurnByCurrentTurns(std::vector<std::vector<int>> lanesInfo,
										   SHARED_PTR<RouteSegmentResult>& currentSegm, vector<int>& rawLanes,
										   int keepTurnType, bool leftSide);
void checkTotalRoutingTime(vector<SHARED_PTR<RouteSegmentResult> > result, float cmp);
bool containsAll(set<int> a, set<int> b);
bool hasTU(string turnLanesPrevSegm, bool attachedOnTheRight);
bool foundTUturn(vector<int> turnList);
void printRouteInfoSegments(vector<SHARED_PTR<RouteSegmentResult> >& result);
string buildRouteMessagesFromInfo(UNORDERED(map) < string, UNORDERED(map) < string, string >> info, vector<string>& routeMessages);
#endif /*_OSMAND_ROUTE_RESULT_PREPARATION_H*/
