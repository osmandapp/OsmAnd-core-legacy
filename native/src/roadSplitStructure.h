#ifndef _OSMAND_ROAD_SPLIT_STRUCTURE_H
#define _OSMAND_ROAD_SPLIT_STRUCTURE_H

#include <algorithm>
#include "CommonCollections.h"
#include "turnType.h"

const float TURN_SLIGHT_DEGREE = 5;

struct AttachedRoadInfo {
    vector<int> parsedLanes;
    double attachedAngle;
    int lanes;
    int speakPriority;
    bool attachedOnTheRight;
    int turnType;
};

struct RoadSplitStructure {
    bool keepLeft = false;
    bool keepRight = false;
    bool speak = false;
    vector<SHARED_PTR<AttachedRoadInfo>> leftLanesInfo;
    int leftLanes = 0;
    vector<SHARED_PTR<AttachedRoadInfo>> rightLanesInfo;
    int rightLanes = 0;
    int roadsOnLeft = 0;
    int addRoadsOnLeft = 0;
    int roadsOnRight = 0;
    int addRoadsOnRight = 0;
    int leftMaxPrio = 0;
    int rightMaxPrio = 0;
    
    bool allAreStraight() {
        for (const SHARED_PTR<AttachedRoadInfo> & angle : leftLanesInfo) {
            if (abs(angle->attachedAngle) > TURN_SLIGHT_DEGREE) {
                return false;
            }
        }
        for (const SHARED_PTR<AttachedRoadInfo> & angle : rightLanesInfo) {
            if (abs(angle->attachedAngle) > TURN_SLIGHT_DEGREE) {
                return false;
            }
        }
        return true;
    }
};

#endif /*_OSMAND_ROAD_SPLIT_STRUCTURE_H*/
