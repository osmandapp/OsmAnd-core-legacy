#ifndef _OSMAND_ROUTING_CONFIGURATION_H
#define _OSMAND_ROUTING_CONFIGURATION_H
#include "CommonCollections.h"
#include "commonOsmAndCore.h"
#include "generalRouter.h"
#include <algorithm>

struct RoutingRule {
    string tagName;
    string t;
    string v;
    string param;
    string value1;
    string value2;
    string type;
};

const std::string DirectionPoint_TAG = "osmand_dp";
const std::string DirectionPoint_DELETE_TYPE = "osmand_delete_point";
const std::string DirectionPoint_CREATE_TYPE = "osmand_add_point";
struct DirectionPoint {
    double distance = -1.0;
    int32_t pointIndex;
    int32_t x31;
    int32_t y31;
    SHARED_PTR<RouteDataObject> connected;
    std::vector<uint32_t> types;
    std::vector<std::pair<std::string, std::string>> tags;
    int connectedx;
    int connectedy;
};

struct RoutingConfiguration {

    const static int DEFAULT_MEMORY_LIMIT = 100;
    const static int DEVIATION_RADIUS = 3000;
    MAP_STR_STR attributes;
    quad_tree<SHARED_PTR<DirectionPoint>> directionPoints;
    double directionPointsRadius = 30.0; // 30 m

    SHARED_PTR<GeneralRouter> router;

	long memoryLimitation;
	float initialDirection;

	int zoomToLoad;
	float heurCoefficient;
	int planRoadDirection;
	string routerName;
	
    // 1.5 Recalculate distance help
    float recalculateDistance;
    time_t routeCalculationTime = 0;

    RoutingConfiguration(float initDirection = -360, int memLimit = DEFAULT_MEMORY_LIMIT) : router(new GeneralRouter()), memoryLimitation(memLimit), initialDirection(initDirection), zoomToLoad(16), heurCoefficient(1), planRoadDirection(0), routerName(""), recalculateDistance(20000.0f) {
    }

    string getAttribute(SHARED_PTR<GeneralRouter> router, string propertyName) {
        if (router->containsAttribute(propertyName)) {
            return router->getAttribute(propertyName);
        }
        return attributes[propertyName];
    }

    void initParams() {
		planRoadDirection = (int) parseFloat(getAttribute(router, "planRoadDirection"), 0);
		heurCoefficient = parseFloat(getAttribute(router, "heuristicCoefficient"), 1);
        recalculateDistance = parseFloat(getAttribute(router, "recalculateDistanceHelp"), 20000);
		// don't use file limitations?
		memoryLimitation = (int)parseFloat(getAttribute(router, "nativeMemoryLimitInMB"), memoryLimitation);
		zoomToLoad = (int)parseFloat(getAttribute(router, "zoomToLoadTiles"), 16);
		//routerName = parseString(getAttribute(router, "name"), "default");
	}
};

class RoutingConfigurationBuilder {
private:
    MAP_STR_STR attributes;
    UNORDERED(map)<int64_t, int_pair> impassableRoadLocations;    

public:
    UNORDERED(map)<string, SHARED_PTR<GeneralRouter> > routers;
    string defaultRouter;
    std::vector<DirectionPoint> directionPointsBuilder;

    RoutingConfigurationBuilder() : defaultRouter("") {
    }
    
    SHARED_PTR<RoutingConfiguration> build(string router, int memoryLimitMB, const MAP_STR_STR& params = MAP_STR_STR()) {
        return build(router, -360, memoryLimitMB, params);
    }
    
    SHARED_PTR<RoutingConfiguration> build(string router, float direction, long memoryLimitMB, const MAP_STR_STR& params = MAP_STR_STR()) {
        if (routers.find(router) == routers.end()) {
            router = defaultRouter;
        }
        SHARED_PTR<RoutingConfiguration> i = std::make_shared<RoutingConfiguration>();
        if (routers.find(router) != routers.end()) {
            i->router = routers[router]->build(params);
            i->routerName = router;
        }
        attributes["routerName"] = router;
        i->attributes.insert(attributes.begin(), attributes.end());
        i->initialDirection = direction;
        i->memoryLimitation = memoryLimitMB;
        i->initParams();
        
        auto it = impassableRoadLocations.begin();
        for(;it != impassableRoadLocations.end(); it++) {
            i->router->impassableRoadIds.insert(it->first);
        }
        if (directionPointsBuilder.size() > 0) {
            SkIRect rect = SkIRect::MakeLTRB(0, 0, 0x7FFFFFFF, 0x7FFFFFFF);
            i->directionPoints = quad_tree<SHARED_PTR<DirectionPoint>>(rect, 14, 0.5);
            for (int j = 0; j < directionPointsBuilder.size(); j++) {
                SHARED_PTR<DirectionPoint> dp = std::make_shared<DirectionPoint>(directionPointsBuilder[j]);
                SkIRect rectDp = SkIRect::MakeLTRB(dp->x31, dp->y31, dp->x31, dp->y31);
                i->directionPoints.insert(dp, rectDp);
            }
        }
        return i;
    }
    
    UNORDERED(map)<int64_t, int_pair>& getImpassableRoadLocations() {
        return impassableRoadLocations;
    }
    
    bool addImpassableRoad(int64_t routeId, int x31, int y31) {
        if (impassableRoadLocations.find(routeId) == impassableRoadLocations.end()) {
            impassableRoadLocations[routeId] = int_pair(x31, y31);
            return true;
        }
        return false;
    }
    
    SHARED_PTR<GeneralRouter> getRouter(string applicationMode) {
        return routers[applicationMode];
    }
    
    void addRouter(string name, SHARED_PTR<GeneralRouter> router) {
        routers[name] = router;
    }

    void addAttribute(string name, string value) {
        attributes[name] = value;
    }

    void removeImpassableRoad(int64_t routeId) {
        impassableRoadLocations.erase(routeId);
    }
};

SHARED_PTR<RoutingConfigurationBuilder> parseRoutingConfigurationFromXml(const char* filePath, const char* fileName);

#endif /*_OSMAND_ROUTING_CONFIGURATION_H*/
