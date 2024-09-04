//  git revision ab36d0ce1a089ab1cf8dd8c582ac9e85cd77bcff

#ifndef _OSMAND_ROUTE_RESULT_PREPARATION_H
#define _OSMAND_ROUTE_RESULT_PREPARATION_H
#include "CommonCollections.h"
#include "commonOsmAndCore.h"

const string UNMATCHED_HIGHWAY_TYPE = "unmatched";

struct RouteSegmentResult;
struct FinalRouteSegment;
struct CombineAreaRoutePoint;
struct RouteDataObject;
struct TurnType;
struct AttachedRoadInfo;

bool segmentLineBelongsToPolygon(CombineAreaRoutePoint& p, CombineAreaRoutePoint& n,
                                 vector<CombineAreaRoutePoint>& originalWay);
void simplifyAreaRouteWay(vector<CombineAreaRoutePoint>& routeWay, vector<CombineAreaRoutePoint>& originalWay);
void combineWayPointsForAreaRouting(RoutingContext* ctx, vector<SHARED_PTR<RouteSegmentResult> >& result);

void printResults(RoutingContext* ctx, int startX, int startY, int endX, int endY, vector<SHARED_PTR<RouteSegmentResult> >& result);
vector<SHARED_PTR<RouteSegmentResult> > prepareResult(RoutingContext* ctx, SHARED_PTR<FinalRouteSegment> finalSegment);
vector<SHARED_PTR<RouteSegmentResult> > prepareResult(RoutingContext* ctx, vector<SHARED_PTR<RouteSegmentResult> >& result);

vector<int> parseTurnLanes(const SHARED_PTR<RouteDataObject>& ro, double dirToNorthEastPi);
vector<int> parseLanes(const SHARED_PTR<RouteDataObject>& ro, double dirToNorthEastPi);
void avoidKeepForThroughMoving(vector<SHARED_PTR<RouteSegmentResult> >& result);
void muteAndRemoveTurns(vector<SHARED_PTR<RouteSegmentResult> >& result);
void prepareTurnResults(RoutingContext* ctx, vector<SHARED_PTR<RouteSegmentResult> >& result);
SHARED_PTR<TurnType> getTurnByCurrentTurns(std::vector<SHARED_PTR<AttachedRoadInfo>> & lanesInfo, vector<int>& rawLanes,
										   int keepTurnType, bool leftSide);
void checkTotalRoutingTime(vector<SHARED_PTR<RouteSegmentResult> > result, float cmp);
bool containsAll(vector<int> a, vector<int> b);
bool hasTU(string turnLanesPrevSegm, bool attachedOnTheRight);
bool foundTUturn(vector<int> turnList);
void printRouteInfoSegments(vector<SHARED_PTR<RouteSegmentResult> >& result);
string buildRouteMessagesFromInfo(UNORDERED(map) < string, UNORDERED(map) < string, string >> info, vector<string>& routeMessages);
void calculateTimeSpeed(RoutingContext* ctx, vector<SHARED_PTR<RouteSegmentResult> >& result);
void calculateTimeSpeed(RoutingContext* ctx, SHARED_PTR<RouteSegmentResult>& result);
vector<int> getUniqTurnTypes(const string& turnLanes);
bool hasTurn(const string& turnLanes, int turnType);
bool hasSharpOrReverseTurnLane(const string& turnLanes);
bool hasSameTurnLanes(SHARED_PTR<RouteSegmentResult>& prevSegm, SHARED_PTR<RouteSegmentResult>& currentSegm);
SHARED_PTR<TurnType> getActiveTurnType(const vector<int>& lanes, bool leftSide, SHARED_PTR<TurnType> oldTurnType);
bool hasAllowedLanes(int mainTurnType, vector<int>& lanesArray, int startActiveIndex, int endActiveIndex);
int countOccurrences(const string& haystack, char needle);
int highwaySpeakPriority(const string& highway);
int getTurnByAngle(double angle);
bool isSwitchToLink(SHARED_PTR<RouteSegmentResult>& curr, SHARED_PTR<RouteSegmentResult>& prev);
bool twiceRoadPresent(vector<SHARED_PTR<RouteSegmentResult> >& result, int i);
void setActiveTurns(vector<int> & rawLanes, int activeBeginIndex, int activeEndIndex);
bool isKeepTurn(SHARED_PTR<TurnType> t);
bool isHighSpeakPriority(SHARED_PTR<RouteSegmentResult>& curr);
bool isForkByLanes(SHARED_PTR<RouteSegmentResult>&  curr, SHARED_PTR<RouteSegmentResult>&  prev);
#endif /*_OSMAND_ROUTE_RESULT_PREPARATION_H*/
