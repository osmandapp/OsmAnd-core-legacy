#ifndef _OSMAND_HH_BINARY_READ_H
#define _OSMAND_HH_BINARY_READ_H

#include "binaryRead.h"
#include "hhRouteDataStructure.h"

struct HHRoutePointsBox {
    int32_t length;
    int32_t filePointer;
    int32_t left, right, bottom, top;
    bool init;
    
    HHRoutePointsBox(): length(0), filePointer(0), left(0), right(0), bottom(0), top(0), init(false) {
    }

    /*QuadRect getLatLonBox() {
        QuadRect q = new QuadRect();
        q.left = MapUtils.get31LongitudeX(left);
        q.right = MapUtils.get31LongitudeX(right);
        q.top = MapUtils.get31LatitudeY(top);
        q.bottom = MapUtils.get31LatitudeY(bottom);
        return q;
    }*/

    bool contains(int x, int y) {
        return x >= left && x <= right && y >= top && y <= bottom;
    }
};

//HHRouteRegion
struct HHRouteIndex : BinaryPartIndex {
    uint64_t edition;
    std::string profile;
    std::vector<std::string> profileParams;
    HHRoutePointsBox top;
    
    HHRouteIndex() : BinaryPartIndex(HH_INDEX), edition(0), profile("") {
    }
    
    // not stored in cache
    std::vector<HHRouteBlockSegments> segments;

        /*std::string getPartName() {
            return "Highway routing";
        }

        int getFieldNumber() {
            return OsmandOdb.OsmAndStructure.HHROUTINGINDEX_FIELD_NUMBER;
        }

        QuadRect getLatLonBbox() {
            if(top == null) {
                return new QuadRect();
            }
            return top.getLatLonBox();
        }*/
};

std::vector<NetworkDBPoint> initHHPoints(BinaryMapFile* file, SHARED_PTR<HHRouteIndex> reg, short mapId);

#endif