#ifndef _OSMAND_ROUTING_CONFIGURATION_H
#define _OSMAND_ROUTING_CONFIGURATION_H
#include <algorithm>

#include "CommonCollections.h"
#include "commonOsmAndCore.h"
#include "generalRouter.h"
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
const std::string DirectionPoint_ANGLE_TAG = "apply_direction_angle";
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
    static constexpr double MAX_ANGLE_DIFF = 45; //in degrees
    static constexpr double NO_ANGLE = 9999;

    // get angle in degrees or NO_ANGLE if empty
    double getAngle() {
        std::string angle = "";
        for (std::pair<std::string, std::string>& tag : tags) {
            if (tag.first == DirectionPoint_ANGLE_TAG) {
                angle = tag.second;
                break;
            }
        }
        if (!angle.empty()) {
            const char* angle_c = angle.c_str();
            char c = angle_c[0];
            double a = atof(angle_c);
            if (c != '0' && a == 0.0) {
                return NO_ANGLE;
            }
            return a;
        }
        return NO_ANGLE;
    }
};

struct RoutingConfiguration {

    const static int DEFAULT_MEMORY_LIMIT = 256;
    const static int DEVIATION_RADIUS = 3000;
    constexpr const static double DEFAULT_PENALTY_FOR_REVERSE_DIRECTION = 500;
    MAP_STR_STR attributes;
    quad_tree<SHARED_PTR<DirectionPoint>> directionPoints;
    double directionPointsRadius = 30.0; // 30 m

    SHARED_PTR<GeneralRouter> router;

    long memoryLimitation;
    float initialDirection;
    double targetDirection; // TODO pass it in getRoutingContext java_wrap.cpp (for HH)
    double PENALTY_FOR_REVERSE_DIRECTION = DEFAULT_PENALTY_FOR_REVERSE_DIRECTION; // TODO pass it for HH

    int zoomToLoad;
    float heurCoefficient;
    int planRoadDirection;
    string routerName;

    // ! MAIN parameter to approximate (35m good for custom recorded tracks)
    float minPointApproximation = 50;

    // don't search subsegments shorter than specified distance (also used to step back for car turns)
    float minStepApproximation = 100;

    // This parameter could speed up or slow down evaluation (better to make bigger for long routes and smaller for short)
    float maxStepApproximation = 3000;

    // Parameter to smoother the track itself (could be 0 if it's not recorded track)
    float smoothenPointsNoRoute = 5;

    // 1.5 Recalculate distance help
    float recalculateDistance;
    time_t routeCalculationTime = 0;

    RoutingConfiguration(float initDirection = -2 * M_PI, int memLimit = DEFAULT_MEMORY_LIMIT) : router(new GeneralRouter()), memoryLimitation(memLimit), initialDirection(initDirection), zoomToLoad(16), heurCoefficient(1), planRoadDirection(0), routerName(""), recalculateDistance(20000.0f) {
    }

    string getAttribute(SHARED_PTR<GeneralRouter> router, string propertyName) {
        if (router->containsAttribute(propertyName)) {
            return router->getAttribute(propertyName);
        }
        return attributes[propertyName];
    }

    void initParams() {
        planRoadDirection = (int) parseFloat(getAttribute(router, "planRoadDirection"), 0);
        recalculateDistance = parseFloat(getAttribute(router, "recalculateDistanceHelp"), 20000);
        heurCoefficient = parseFloat(getAttribute(router, "heuristicCoefficient"), 1);
        minPointApproximation = parseFloat(getAttribute(router, "minPointApproximation"), 50);
        minStepApproximation = parseFloat(getAttribute(router, "minStepApproximation"), 100);
        maxStepApproximation = parseFloat(getAttribute(router, "maxStepApproximation"), 3000);
        smoothenPointsNoRoute = parseFloat(getAttribute(router, "smoothenPointsNoRoute"), 5);
        // don't use file limitations?
        memoryLimitation = (int)parseFloat(getAttribute(router, "nativeMemoryLimitInMB"), memoryLimitation);
        zoomToLoad = (int)parseFloat(getAttribute(router, "zoomToLoadTiles"), 16);
        //routerName = parseString(getAttribute(router, "name"), "default");
    }
};

class RoutingConfigurationBuilder {
private:
    UNORDERED(map)<int64_t, int_pair> impassableRoadLocations;
public:
    MAP_STR_STR attributes;
    UNORDERED(map)<string, SHARED_PTR<GeneralRouter> > routers;
    string defaultRouter;
    std::vector<DirectionPoint> directionPointsBuilder;

    RoutingConfigurationBuilder() : defaultRouter("") {
    }
    
    SHARED_PTR<RoutingConfiguration> build(string router, int memoryLimitMB, MAP_STR_STR& params) {
        return build(router, -2 * M_PI, memoryLimitMB, params);
    }
    
    SHARED_PTR<RoutingConfiguration> build(string router, float direction, long memoryLimitMB, MAP_STR_STR& params) {
        string derivedProfile;
        if (routers.find(router) == routers.end()) {
            for (const auto& r : routers) {
                string derivedProfiles = r.second->getAttribute("derivedProfiles");
                if (!derivedProfiles.empty() && derivedProfiles.find(router) != std::string::npos) {
                    derivedProfile = router;
                    router = r.first;
                    break;
                }
            }
            if (derivedProfile.empty()) {
                router = defaultRouter;
            }
        }
        if (!derivedProfile.empty()) {
            params["profile_" + derivedProfile] = "true";
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
        const auto it = routers.find(applicationMode);
        if (it == routers.end())
            return nullptr;
        return it->second;
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
