#ifndef _OSMAND_ROUTE_DATA_RESOURCES_H
#define _OSMAND_ROUTE_DATA_RESOURCES_H
#include "CommonCollections.h"
#include "commonOsmAndCore.h"
#include "routeTypeRule.h"
#include "binaryRead.h"

struct Location {
  public:
	long time;
	double latitude;
	double longitude;
	double altitude;
	double speed;
	double bearing;
	double accuracy;
	double verticalAccuracy;
	
	Location();
	Location(double latitude, double longitude);
	
	bool isInitialized();
};

struct RouteDataResources {
private:
public:
    int currentSegmentStartLocationIndex;
    UNORDERED_map<RouteTypeRule, uint32_t> rules;
    vector<RouteTypeRule> insertOrder;
    vector<Location> locations;
    UNORDERED_map<SHARED_PTR<RouteDataObject>, vector<vector<uint32_t>>> pointNamesMap;
    vector<int> routePointIndexes;
    
    RouteDataResources();
    RouteDataResources(vector<Location> locations, vector<int>& routePointIndexes);
    
    Location getCurrentSegmentLocation(int offset);
    int getCurrentSegmentStartLocationIndex();
    void updateNextSegmentStartLocation(int currentSegmentLength);
};

#endif /*_OSMAND_ROUTE_DATA_RESOURCES_H*/
