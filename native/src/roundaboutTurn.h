#ifndef _OSMAND_ROUNDABOUT_TURN_H
#define _OSMAND_ROUNDABOUT_TURN_H

#include <algorithm>
#include "CommonCollections.h"
#include "turnType.h"

struct RoadSplitStructure;
struct RouteSegmentResult;

class RoundaboutTurn {
public:
    RoundaboutTurn(vector<SHARED_PTR<RouteSegmentResult>>& routeSegmentResults, int i, bool leftSide);
    bool isRoundaboutExist();
    SHARED_PTR<TurnType> getRoundaboutType();
private:
    vector<SHARED_PTR<RouteSegmentResult>> routeSegmentResults;
    SHARED_PTR<RouteSegmentResult> current;
    SHARED_PTR<RouteSegmentResult> prev;
    int iteration;
    bool roundabout;
    bool miniRoundabout;
    bool prevRoundabout;
    bool leftSide;
    bool isMiniRoundabout(SHARED_PTR<RouteSegmentResult> prev, SHARED_PTR<RouteSegmentResult> current);
    SHARED_PTR<TurnType> processRoundaboutTurn();
    SHARED_PTR<TurnType> processMiniRoundaboutTurn();
    SHARED_PTR<RoadSplitStructure> calculateSimpleRoadSplitStructure(const vector<SHARED_PTR<RouteSegmentResult>>& attachedRoutes);
    float calculateRoundaboutTurnAngle(const SHARED_PTR<RouteSegmentResult>& last, const SHARED_PTR<RouteSegmentResult>& firstRoundabout, const SHARED_PTR<RouteSegmentResult>& lastRoundabout, int ind);
};


#endif /*_OSMAND_ROUNDABOUT_TURN_H*/
