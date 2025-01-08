#ifndef _OSMAND_ROUNDABOUT_TURN_CPP
#define _OSMAND_ROUNDABOUT_TURN_CPP

#include "roundaboutTurn.h"
#include "commonOsmAndCore.h"
#include "routeSegmentResult.h"
#include "roadSplitStructure.h"

RoundaboutTurn::RoundaboutTurn(vector<SHARED_PTR<RouteSegmentResult>>& routeSegmentResults, int i, bool leftSide)
    : routeSegmentResults(routeSegmentResults)
    , iteration(i)
    , leftSide(leftSide)
{
    current = routeSegmentResults.size() > i ? routeSegmentResults[i] : nullptr;
    prev = i > 0 && routeSegmentResults.size() > i ? routeSegmentResults[i - 1] : nullptr;
    roundabout = current && current->object->roundabout();
    prevRoundabout = prev && prev->object->roundabout();
    miniRoundabout = isMiniRoundabout(prev, current);
}

bool RoundaboutTurn::isRoundaboutExist() {
    return roundabout || miniRoundabout || prevRoundabout;
}

SHARED_PTR<TurnType> RoundaboutTurn::getRoundaboutType() {
    if (!prev || !current) {
        return nullptr;
    }
    if (prevRoundabout) {
        // already analyzed!
        return nullptr;
    }
    if (roundabout) {
        return processRoundaboutTurn();
    }
    if (miniRoundabout) {
        return processMiniRoundaboutTurn();
    }
    return nullptr;
}

bool RoundaboutTurn::isMiniRoundabout(SHARED_PTR<RouteSegmentResult> prev, SHARED_PTR<RouteSegmentResult> current) {
    if (!prev || !current) {
        return false;
    }
    
    std::vector<uint32_t> prevTypes = prev->object->getPointTypes(prev->getEndPointIndex());
    std::vector<uint32_t> currentTypes = current->object->getPointTypes(current->getStartPointIndex());
    uint32_t miniType = prev->object->region->searchRouteEncodingRule("highway", "mini_roundabout");
    if (miniType != -1) {
        return false;
    }
    bool p = false;
    bool c = false;
    for (int t : prevTypes) {
        if (t == miniType) {
            p = true;
            break;
        }
    }
    for (int t : currentTypes) {
        if (t == miniType) {
            c = true;
            break;
        }
    }
    return p && c;
}

SHARED_PTR<TurnType> RoundaboutTurn::processRoundaboutTurn() {
    int exit = 1;
    auto last = current;
    auto firstRoundabout = current;
    auto lastRoundabout = current;
    for (int j = iteration; j < routeSegmentResults.size(); j++) {
        auto rnext = routeSegmentResults[j];
        last = rnext;
        if (rnext->object->roundabout()) {
            lastRoundabout = rnext;
            bool plus = rnext->getStartPointIndex() < rnext->getEndPointIndex();
            int k = rnext->getStartPointIndex();
            if (j == iteration) {
                // first exit could be immediately after roundabout enter
                //                    k = plus ? k + 1 : k - 1;
            }
            while (k != rnext->getEndPointIndex()) {
                int attachedRoads = (int)rnext->getAttachedRoutes(k).size();
                if (attachedRoads > 0) {
                    exit++;
                }
                k = plus ? k + 1 : k - 1;
            }
        } else {
            break;
        }
    }
    // combine all roundabouts
    auto t = TurnType::getPtrExitTurn(exit, 0, leftSide);
    // usually covers more than expected
    float turnAngleBasedOnOutRoads = (float) degreesDiff(last->getBearingBegin(), prev->getBearingEnd());
    // usually covers less than expected
    float turnAngleBasedOnCircle = (float) -degreesDiff(firstRoundabout->getBearingBegin(), lastRoundabout->getBearingEnd() + 180);
    if (abs(turnAngleBasedOnOutRoads - turnAngleBasedOnCircle) > 180) {
        t->setTurnAngle(turnAngleBasedOnCircle);
    } else {
        t->setTurnAngle((turnAngleBasedOnCircle + turnAngleBasedOnOutRoads) / 2) ;
    }
    return t;
}

SHARED_PTR<TurnType> RoundaboutTurn::processMiniRoundaboutTurn() {
    vector<SHARED_PTR<RouteSegmentResult>> attachedRoutes = current->getAttachedRoutes(current->getStartPointIndex());
    bool clockwise = current->object->isClockwise(leftSide);
    if (attachedRoutes.size() > 0) {
        SHARED_PTR<RoadSplitStructure> rs = calculateSimpleRoadSplitStructure(attachedRoutes);
        int rightAttaches = rs->roadsOnRight;
        int leftAttaches = rs->roadsOnLeft;
        int exit = 1;
        if (clockwise) {
            exit += leftAttaches;
        } else {
            exit += rightAttaches;
        }
        SHARED_PTR<TurnType> t = std::make_shared<TurnType>(TurnType::getExitTurn(exit, 0, leftSide));
        float turnAngleBasedOnOutRoads = (float) degreesDiff(current->getBearingBegin(), prev->getBearingEnd());
        float turnAngleBasedOnCircle = (float) degreesDiff(current->getBearingBegin(), prev->getBearingEnd() + 180);
        if (abs(turnAngleBasedOnOutRoads) > 120) {
            t->setTurnAngle(turnAngleBasedOnCircle) ;
        } else {
            t->setTurnAngle(turnAngleBasedOnOutRoads) ;
        }
        return t;
    }
    return nullptr;
}

SHARED_PTR<RoadSplitStructure> RoundaboutTurn::calculateSimpleRoadSplitStructure(vector<SHARED_PTR<RouteSegmentResult>>& attachedRoutes) {
    double prevAngle = normalizeDegrees360(prev->getBearingBegin() - 180);
    double currentAngle = normalizeDegrees360(current->getBearingBegin());
    SHARED_PTR<RoadSplitStructure> rs = std::make_shared<RoadSplitStructure>();
    for (const auto & attached : attachedRoutes) {
        double attachedAngle = normalizeDegrees360(attached->getBearingBegin());
        bool rightSide;
        if (prevAngle > currentAngle) {
            rightSide = attachedAngle > currentAngle && attachedAngle < prevAngle;
        } else {
            bool leftSide = attachedAngle > prevAngle && attachedAngle < currentAngle;
            rightSide = !leftSide;
        }
        
        if (rightSide) {
            rs->roadsOnRight++;
        } else {
            rs->roadsOnLeft++;
        }
    }
    return rs;
}

#endif /*_OSMAND_ROUNDABOUT_TURN_H*/
