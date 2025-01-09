#ifndef _OSMAND_ROUTE_RESULT_PREPARATION_CPP
#define _OSMAND_ROUTE_RESULT_PREPARATION_CPP
#include "routingContext.h"
#include "routeSegment.h"
#include "routeSegmentResult.h"
#include "routeResultPreparation.h"
#include "turnType.h"
#include "binaryRoutePlanner.h"
#include "multipolygons.h"
#include "roadSplitStructure.h"
#include "roundaboutTurn.h"

const int MAX_SPEAK_PRIORITY = 5;
const float TURN_DEGREE_MIN = 45;
const float UNMATCHED_TURN_DEGREE_MINIMUM = 45;
const float SPLIT_TURN_DEGREE_NOT_STRAIGHT = 100;

const int TurnType::TURNS_ORDER[9] = {TU, TSHL, TL, TSLL, C, TSLR, TR, TSHR, TRU};

struct MergeTurnLaneTurn {
    SHARED_PTR<TurnType> turn;
    vector<int> originalLanes;
    vector<int> disabledLanes;
    int activeStartIndex;
    int activeEndIndex;
    int activeLen;
    
    MergeTurnLaneTurn(SHARED_PTR<RouteSegmentResult>& segment) : activeStartIndex(-1), activeEndIndex(-1), activeLen(0) {
        turn = segment->turnType;
        if (turn) {
            originalLanes = turn->getLanes();
        }
        if (!originalLanes.empty()) {
            disabledLanes.clear();
            disabledLanes.resize(originalLanes.size());
            for (int i = 0; i < originalLanes.size(); i++) {
                int ln = originalLanes[i];
                disabledLanes[i] = ln & ~1;
                if ((ln & 1) > 0) {
                    if (activeStartIndex == -1) {
                        activeStartIndex = i;
                    }
                    activeEndIndex = i;
                    activeLen++;
                }
            }
        }
    }
    
    inline bool isActiveTurnMostLeft() {
        return activeStartIndex == 0;
    }
    inline bool isActiveTurnMostRight() {
        return activeEndIndex == originalLanes.size() - 1;
    }
};

struct CombineAreaRoutePoint {
    int x31;
    int y31;
    int originalIndex;
};

vector<int> getPossibleTurns(vector<int>& oLanes, bool onlyPrimary, bool uniqueFromActive);

int64_t getPoint(const SHARED_PTR<RouteDataObject>& road, int pointInd) {
    return (((int64_t) road->pointsX[pointInd]) << 31) + (int64_t) road->pointsY[pointInd];
}

bool isMotorway(const SHARED_PTR<RouteSegmentResult>& s) {
    string h = s->object->getHighway();
    return "motorway" == h || "motorway_link" == h  || "trunk" == h || "trunk_link" == h;
}

bool isPrimary(const SHARED_PTR<RouteSegmentResult>& s) {
    string h = s->object->getHighway();
    return "primary" == h || "primary_link" == h;
}

void ignorePrecedingStraightsOnSameIntersection(bool leftside, vector<SHARED_PTR<RouteSegmentResult> >& result) {
    //Issue 2571: Ignore TurnType::C if immediately followed by another turn in non-motorway cases, as these likely belong to the very same intersection
    SHARED_PTR<RouteSegmentResult> nextSegment = nullptr;
    double distanceToNextTurn = 999999;
    for (int i = (int)result.size() - 1; i >= 0; i--) {
        // Mark next "real" turn
        if (nextSegment && nextSegment->turnType &&
            nextSegment->turnType->getValue() != TurnType::C && !isMotorway(nextSegment)) {
            if (distanceToNextTurn == 999999) {
                distanceToNextTurn = 0;
            }
        }
        auto currentSegment = result[i];
        // Identify preceding goStraights within distance limit and suppress
        if (currentSegment) {
            distanceToNextTurn += currentSegment->distance;
            if (currentSegment->turnType &&
                currentSegment->turnType->getValue() == TurnType::C && distanceToNextTurn <= 200) {
                currentSegment->turnType->setSkipToSpeak(true);
            } else {
                nextSegment = currentSegment;
                if (currentSegment->turnType) {
                    distanceToNextTurn = 999999;
                }
            }
        }
    }
}

void validateAllPointsConnected(vector<SHARED_PTR<RouteSegmentResult> >& result) {
    for (int i = 1; i < result.size(); i++) {
        auto rr = result[i];
        auto pr = result[i - 1];
        double d = measuredDist31(pr->object->pointsX[pr->getEndPointIndex()], pr->object->pointsY[pr->getEndPointIndex()], rr->object->pointsX[rr->getStartPointIndex()], rr->object->pointsY[rr->getStartPointIndex()]);
        if (d > 0) {
            OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Warning, "Points are not connected: %p(%d) -> %p(%d) %f meters", pr->object.get(), pr->getEndPointIndex(), rr->object.get(), rr->getStartPointIndex(), d);
        }
    }
}

// try to attach all segments except with current id
void attachSegments(RoutingContext* ctx, const SHARED_PTR<RouteSegment>& routeSegment, const SHARED_PTR<RouteDataObject>& road, const SHARED_PTR<RouteSegmentResult>& rr, int64_t previousRoadId, int pointInd, int64_t prevL, int64_t nextL) {
    if (routeSegment->road->getId() != road->getId() && routeSegment->road->getId() != previousRoadId) {
        auto addRoad = routeSegment->road;
        //checkAndInitRouteRegion(ctx, addRoad); ? TODO
        // Future: restrictions can be considered as well
        int oneWay = ctx->config->router->isOneWay(addRoad);
        if (oneWay >= 0 && routeSegment->getSegmentStart() < addRoad->getPointsLength() - 1) {
            int64_t pointL = getPoint(addRoad, routeSegment->getSegmentStart() + 1);
            if (pointL != nextL && pointL != prevL) {
                // if way contains same segment (nodes) as different way (do not attach it)
                auto rsr = std::make_shared<RouteSegmentResult>(addRoad, routeSegment->getSegmentStart(), addRoad->getPointsLength() - 1);
                rr->attachRoute(pointInd, rsr);
            }
        }
        if (oneWay <= 0 && routeSegment->getSegmentStart() > 0) {
            int64_t pointL = getPoint(addRoad, routeSegment->getSegmentStart() - 1);
            // if way contains same segment (nodes) as different way (do not attach it)
            if (pointL != nextL && pointL != prevL) {
                auto rsr = std::make_shared<RouteSegmentResult>(addRoad, routeSegment->getSegmentStart(), 0);
                rr->attachRoute(pointInd, rsr);
            }
        }
    }
}

void attachRoadSegments(RoutingContext* ctx, vector<SHARED_PTR<RouteSegmentResult> >& result, int routeInd, int pointInd, bool plus) {
    auto rr = result[routeInd];
    auto road = rr->object;
    int64_t nextL = pointInd < road->getPointsLength() - 1 ? getPoint(road, pointInd + 1) : 0;
    int64_t prevL = pointInd > 0 ? getPoint(road, pointInd - 1) : 0;
    
    // attach additional roads to represent more information about the route
    SHARED_PTR<RouteSegmentResult> previousResult = nullptr;

    // by default make same as this road id
    int64_t previousRoadId = road->getId();
    if (pointInd == rr->getStartPointIndex() && routeInd > 0) {
        previousResult = result[routeInd - 1];
        previousRoadId = previousResult->object->getId();
        if (previousRoadId != road->getId()) {
            if (previousResult->getStartPointIndex() < previousResult->getEndPointIndex() && previousResult->getEndPointIndex() < previousResult->object->getPointsLength() - 1) {
                auto segResult = std::make_shared<RouteSegmentResult>(previousResult->object, previousResult->getEndPointIndex(), previousResult->object->getPointsLength() - 1);
                rr->attachRoute(pointInd, segResult);
            } else if (previousResult->getStartPointIndex() > previousResult->getEndPointIndex() && previousResult->getEndPointIndex() > 0) {
                auto segResult = std::make_shared<RouteSegmentResult>(previousResult->object, previousResult->getEndPointIndex(), 0);
                rr->attachRoute(pointInd, segResult);
            }
        }
    }
    if (!rr->getPreAttachedRoutes(pointInd).empty()) {
        const auto& list = rr->getPreAttachedRoutes(pointInd);
        for (auto r : list) {
            auto rs = std::make_shared<RouteSegment>(r->object, r->getStartPointIndex(), r->getEndPointIndex());
            attachSegments(ctx, rs, road, rr, previousRoadId, pointInd, prevL, nextL);
        }
    } else {
        auto segments = ctx->loadRouteSegment(road->pointsX[pointInd], road->pointsY[pointInd]);
		for (auto& segment : segments) {
			attachSegments(ctx, segment, road, rr, previousRoadId, pointInd, prevL, nextL);
		}
	}
}

void splitRoadsAndAttachRoadSegments(RoutingContext* ctx, vector<SHARED_PTR<RouteSegmentResult> >& result) {
    for (int i = 0; i < result.size(); i++) {
        //ctx->unloadUnusedTiles(ctx->config->memoryLimitation);
        
        auto rr = result[i];
        auto road = rr->object;
        //checkAndInitRouteRegion(ctx, road); ? TODO
        bool plus = rr->getStartPointIndex() < rr->getEndPointIndex();
        int next;
        bool unmatched = UNMATCHED_HIGHWAY_TYPE == rr->object->getHighway();
        for (int j = rr->getStartPointIndex(); j != rr->getEndPointIndex(); j = next) {
            next = plus ? j + 1 : j - 1;
            if (j == rr->getStartPointIndex()) {
                attachRoadSegments(ctx, result, i, j, plus);
            }
            if (next != rr->getEndPointIndex()) {
                attachRoadSegments(ctx, result, i, next, plus);
            }
            const auto& attachedRoutes = rr->getAttachedRoutes(next);
            bool tryToSplit = next != rr->getEndPointIndex() && !rr->object->roundabout();
            if (rr->getDistance(next, plus ) == 0) {
                // same point will be processed next step
                tryToSplit = false;
            }
            if (tryToSplit) {
                float distBearing = unmatched ? DIST_BEARING_DETECT_UNMATCHED : DIST_BEARING_DETECT;
                // avoid small zigzags
                float before = rr->getBearingEnd(next, distBearing);
                float after = rr->getBearingBegin(next, distBearing);
                if(rr->getDistance(next, plus) < distBearing) {
                    after = before;
                } else if(rr->getDistance(next, !plus) < distBearing) {
                    before = after;
                }
                double contAngle = abs(degreesDiff(before, after));
                bool straight = contAngle < TURN_DEGREE_MIN;
                bool isSplit = false;
                
                if (unmatched && abs(contAngle) >= UNMATCHED_TURN_DEGREE_MINIMUM) {
                    isSplit = true;
                }
                // split if needed
                for (auto rs : attachedRoutes) {
                    double diff = degreesDiff(before, rs->getBearingBegin());
                    if (abs(diff) <= TURN_DEGREE_MIN) {
                        isSplit = true;
                    } else if (!straight && abs(diff) < SPLIT_TURN_DEGREE_NOT_STRAIGHT) {
                        isSplit = true;
                    }
                }
                if (isSplit) {
                    int endPointIndex = rr->getEndPointIndex();
                    auto split = std::make_shared<RouteSegmentResult>(rr->object, next, endPointIndex);
                    split->copyPreattachedRoutes(rr, abs(next - rr->getStartPointIndex()));
                    rr->setEndPointIndex(next);
                    result.insert(result.begin() + i + 1, split);
                    i++;
                    // switch current segment to the splitted
                    rr = split;
                }
            }
        }
    }
}

void calculateTimeSpeed(RoutingContext* ctx, vector<SHARED_PTR<RouteSegmentResult>>& result) {
    for (int i = 0; i < result.size(); i++) {
        calculateTimeSpeed(ctx, result[i]);
    }
}

void calculateTimeSpeed(RoutingContext* ctx, SHARED_PTR<RouteSegmentResult>& rr) {
    // Naismith's/Scarf rules are used to clarify time on uphills
    bool useNaismithRule = false;
    double scarfSeconds = 0; // Additional time as per Naismith/Scarf
    if (ctx->config->router->getProfile() == GeneralRouterProfile::PEDESTRIAN) {
        // PEDESTRIAN 1:7.92 based on https://en.wikipedia.org/wiki/Naismith%27s_rule (Scarf rule)
        scarfSeconds = 7.92f / ctx->config->router->getDefaultSpeed();
        useNaismithRule = true;
    } else if (ctx->config->router->getProfile() == GeneralRouterProfile::BICYCLE) {
        // BICYCLE 1:8.2 based on https://pubmed.ncbi.nlm.nih.gov/17454539/ (Scarf's article)
        scarfSeconds = 8.2f / ctx->config->router->getDefaultSpeed();
        useNaismithRule = true;
    }

    {
        auto road = rr->object;
        double distOnRoadToPass = 0;
        double speed = ctx->config->router->defineVehicleSpeed(road, rr->isForwardDirection());
        if (speed == 0) {
            speed = ctx->config->router->getDefaultSpeed();
        } else {
            if (speed > 15) {
                // decrease speed proportionally from 15ms (50kmh)
                // reference speed 30ms (108kmh) - 2ms (7kmh)
                speed = speed - (speed / 15.f - 1) * 2.f;
            }
        }
        bool plus = rr->getStartPointIndex() < rr->getEndPointIndex();
        int next;
        double distance = 0;
        
        //for Naismith/Scarf
        vector<double> heightDistanceArray;
        if (useNaismithRule) {
            road->calculateHeightArray();
            heightDistanceArray = road->heightDistanceArray;
        }
        
        for (int j = rr->getStartPointIndex(); j != rr->getEndPointIndex(); j = next) {
            next = plus ? j + 1 : j - 1;
            double d = measuredDist31(road->pointsX[j], road->pointsY[j], road->pointsX[next], road->pointsY[next]);
            distance += d;
            double obstacle = ctx->config->router->defineObstacle(road, j, rr->getStartPointIndex() < rr->getEndPointIndex());
            if (obstacle < 0) {
                obstacle = 0;
            }
            distOnRoadToPass += d / speed + obstacle;

            //for Naismith/Scarf
            if (useNaismithRule) {
                int heightIndex = 2 * j + 1;
                int nextHeightIndex = 2 * next + 1;
                if (!heightDistanceArray.empty() && heightIndex < heightDistanceArray.size() && nextHeightIndex < heightDistanceArray.size()) {
                    float heightDiff = heightDistanceArray[nextHeightIndex] - heightDistanceArray[heightIndex];
                    if (heightDiff > 0) { // ascent only
                        // Naismith/Scarf rule: An ascent adds 7.92 times the hiking time its vertical elevation gain takes to cover horizontally
                        //   (- Naismith original: Add 1 hour per vertical 2000ft (600m) at assumed horizontal speed 3mph)
                        //   (- Swiss Alpine Club: Uses conservative 1 hour per 400m at 4km/h)
                        distOnRoadToPass += heightDiff * scarfSeconds;
                    }
                }
            }
        }
        // last point turn time can be added
        // if(i + 1 < result.size()) { distOnRoadToPass += ctx.getRouter().calculateTurnTime(); }
        rr->segmentTime = (float) distOnRoadToPass;
        rr->distance = (float) distance;
        if (distOnRoadToPass != 0) {
            rr->segmentSpeed = (float) (distance / distOnRoadToPass);//effective segment speed incl. obstacle and height effects
        } else {
            rr->segmentSpeed = (float) speed;
        }
    }
}

string getTurnLanesString(SHARED_PTR<RouteSegmentResult>& segment) {
    if (segment->object->getOneway() == 0) {
        if (segment->isForwardDirection()) {
            return segment->object->getValue("turn:lanes:forward");
        } else {
            return segment->object->getValue("turn:lanes:backward");
        }
    } else {
        return segment->object->getValue("turn:lanes");
    }
}

string getTurnString(SHARED_PTR<RouteSegmentResult>& segment) {
    return segment->object->getValue("turn");
}

vector<int> calculateRawTurnLanes(string turnLanes, int calcTurnType) {
    vector<string> splitLaneOptions = split_string(turnLanes, "|");
    vector<int> lanes(splitLaneOptions.size());
    for (int i = 0; i < splitLaneOptions.size(); i++) {
        vector<string> laneOptions = split_string(splitLaneOptions[i], ";");
        for (int j = 0; j < laneOptions.size(); j++) {
            int turn = TurnType::convertType(laneOptions[j]);
            int primary = TurnType::getPrimaryTurn(lanes[i]);
            if (primary == 0) {
                TurnType::setPrimaryTurnAndReset(lanes, i, turn);
            } else {
                if (turn == calcTurnType ||
                    (TurnType::isRightTurn(calcTurnType) && TurnType::isRightTurn(turn)) ||
                    (TurnType::isLeftTurn(calcTurnType) && TurnType::isLeftTurn(turn))
                    ) {
                    TurnType::setPrimaryTurnShiftOthers(lanes, i, turn);
                } else if (TurnType::getSecondaryTurn(lanes[i]) == 0) {
                    TurnType::setSecondaryTurn(lanes, i, turn);
                } else if (TurnType::getTertiaryTurn(lanes[i]) == 0) {
                    TurnType::setTertiaryTurn(lanes, i, turn);
                } else {
                    // ignore
                }
            }
        }
    }
    return lanes;
}

bool setAllowedLanes(int mainTurnType, vector<int>& lanesArray) {
    bool turnSet = false;
    for (int i = 0; i < lanesArray.size(); i++) {
        if (TurnType::getPrimaryTurn(lanesArray[i]) == mainTurnType) {
            lanesArray[i] |= 1;
            turnSet = true;
        }
    }
    return turnSet;
}

int countLanesMinOne(SHARED_PTR<RouteSegmentResult>& attached) {
    bool oneway = attached->object->getOneway() != 0;
    int lns = attached->object->getLanes();
    if (lns == 0) {
        string tls = getTurnLanesString(attached);
        if (tls != "") {
            return max(1, countOccurrences(tls, '|'));
        }
    }
    if (oneway) {
        return max(1, lns);
    }
    if (attached->isForwardDirection() && attached->object->getValue("lanes:forward") != "") {
        int val = -1;
        if (sscanf(attached->object->getValue("lanes:forward").c_str(), "%d", &val) != EOF) {
            return val;
        }
    } else if (!attached->isForwardDirection() && attached->object->getValue("lanes:backward") != "") {
        int val = -1;
        if (sscanf(attached->object->getValue("lanes:backward").c_str(), "%d", &val) != EOF) {
            return val;
        }
    }
    return max(1, (lns + 1) / 2);
}

RoadSplitStructure calculateRoadSplitStructure(SHARED_PTR<RouteSegmentResult>& prevSegm,
                                               SHARED_PTR<RouteSegmentResult>& currentSegm,
                                               vector<SHARED_PTR<RouteSegmentResult>>& attachedRoutes,
                                               string turnLanesPrevSegm) {
    RoadSplitStructure rs;
    int speakPriority = max(highwaySpeakPriority(prevSegm->object->getHighway()),
                            highwaySpeakPriority(currentSegm->object->getHighway()));
    double currentAngle = normalizeDegrees360(currentSegm->getBearingBegin());
    double prevAngle =  normalizeDegrees360(prevSegm->getBearingBegin() - 180);
    bool hasSharpOrReverseLane = hasSharpOrReverseTurnLane(turnLanesPrevSegm);
    bool isSameTurnLanes = hasSameTurnLanes(prevSegm, currentSegm);
    for (auto & attached : attachedRoutes) {
        bool restricted = false;
        for (int k = 0; k < prevSegm->object->getRestrictionLength(); k++) {
            if (prevSegm->object->getRestrictionId(k) == attached->object->getId() &&
                prevSegm->object->getRestrictionType(k) <= RESTRICTION_NO_STRAIGHT_ON) {
                restricted = true;
                break;
            }
        }
        if (restricted) {
            continue;
        }
        double ex = degreesDiff(attached->getBearingBegin(), currentSegm->getBearingBegin());
        double deviation = degreesDiff(prevSegm->getBearingEnd(), attached->getBearingBegin());
        double mpi = abs(deviation);
        int lanes = countLanesMinOne(attached);
        bool smallStraightVariation = mpi < TURN_DEGREE_MIN;
        bool smallTargetVariation = abs(ex) < TURN_DEGREE_MIN;
        bool verySharpTurn = abs(ex) > 150;
        SHARED_PTR<AttachedRoadInfo> ai = std::make_shared<AttachedRoadInfo>();
        ai->speakPriority = highwaySpeakPriority(attached->object->getHighway());
        ai->attachedOnTheRight = ex >= 0;
        ai->attachedAngle = deviation;
        ai->parsedLanes = parseTurnLanes(attached->object, attached->getBearingBegin() * M_PI / 180);;
        ai->lanes = lanes;

        if (!verySharpTurn || hasSharpOrReverseLane) {
            double attachedAngle = normalizeDegrees360(attached->getBearingBegin());
            bool rightSide;
            if (prevAngle > currentAngle) {
                // left side angle range contains 0 degree transition
                rightSide = attachedAngle > currentAngle && attachedAngle < prevAngle;
            } else {
                // right side angle range contains 0 degree transition
                bool leftSide = attachedAngle > prevAngle && attachedAngle < currentAngle;
                rightSide = !leftSide;
            }
            
            //check if need ignore right or left attached road
            if (isSameTurnLanes && !smallTargetVariation && !smallStraightVariation) {
                if (rightSide && !hasTurn(turnLanesPrevSegm, TurnType::TR)) {
                    //restricted
                    continue;
                } else if (!hasTurn(turnLanesPrevSegm, TurnType::TL)) {
                    //restricted
                    continue;
                }
            }
            
            if (rightSide) {
                rs.roadsOnRight++;
            } else {
                rs.roadsOnLeft++;
            }
        }
        
        if (!turnLanesPrevSegm.empty() || ai->speakPriority != MAX_SPEAK_PRIORITY ||
            speakPriority == MAX_SPEAK_PRIORITY) {
            if (smallTargetVariation || smallStraightVariation) {
                if (ai->attachedOnTheRight) {
                    rs.keepLeft = true;
                    rs.rightLanes += lanes;
                    rs.rightMaxPrio = std::max(rs.rightMaxPrio, highwaySpeakPriority(attached->object->getHighway()));
                    rs.rightLanesInfo.push_back(ai);
                } else {
                    rs.keepRight = true;
                    rs.leftLanes += lanes;
                    rs.leftMaxPrio = std::max(rs.leftMaxPrio, highwaySpeakPriority(attached->object->getHighway()));
                    rs.leftLanesInfo.push_back(ai);
                }
                rs.speak = rs.speak || ai->speakPriority <= speakPriority;
            }
        }
    }
    return rs;
}

std::array<int,3> findActiveIndex(SHARED_PTR<RouteSegmentResult> prevSegm, SHARED_PTR<RouteSegmentResult> currentSegm, vector<int> rawLanes, SHARED_PTR<RoadSplitStructure> rs, string turnLanes) {
    std::array<int,3> pair = { -1, -1, 0 };
    if (turnLanes.empty()) {
        return pair;
    }
    if (!rs) {
        vector<SHARED_PTR<RouteSegmentResult>> attachedRoutes = currentSegm->getAttachedRoutes(currentSegm->getStartPointIndex());
        if(attachedRoutes.size() > 0) {
            rs = std::make_shared<RoadSplitStructure>(calculateRoadSplitStructure(prevSegm, currentSegm, attachedRoutes, turnLanes));
        }
    }
    if (!rs) {
        return pair;
    }
    vector<int> directions = getUniqTurnTypes(turnLanes);
    if (rs->roadsOnLeft + rs->roadsOnRight < directions.size()) {
        int startDirection = directions[rs->roadsOnLeft];
        int endDirection = directions[directions.size() - rs->roadsOnRight - 1];
        for (int i = 0; i < rawLanes.size(); i++) {
            int p = TurnType::getPrimaryTurn(rawLanes[i]);
            int s = TurnType::getSecondaryTurn(rawLanes[i]);
            int t = TurnType::getTertiaryTurn(rawLanes[i]);
            if (p == startDirection || s == startDirection || t == startDirection) {
                pair[0] = i;
                pair[2] = startDirection;
                break;
            }
        }
        for (int i = (int)rawLanes.size() - 1; i >= 0; i--) {
            int p = TurnType::getPrimaryTurn(rawLanes[i]);
            int s = TurnType::getSecondaryTurn(rawLanes[i]);
            int t = TurnType::getTertiaryTurn(rawLanes[i]);
            if (p == endDirection || s == endDirection || t == endDirection) {
                pair[1] = i;
                break;
            }
        }
    }
    return pair;
}

vector<int> getTurnLanesInfo(SHARED_PTR<RouteSegmentResult>& prevSegm, SHARED_PTR<RouteSegmentResult>& currentSegm, int mainTurnType) {
    string turnLanes = getTurnLanesString(prevSegm);
    vector<int> lanesArray;
    if (turnLanes.empty()) {
        if(prevSegm->turnType && !prevSegm->turnType->getLanes().empty() && prevSegm->distance < 60) {
            const auto& lns = prevSegm->turnType->getLanes();
            vector<int> lst;
            for(int i = 0; i < lns.size(); i++) {
                if (lns[i] % 2 == 1) {
                    lst.push_back((lns[i] >> 1) << 1);
                }
            }
            if (lst.empty()) {
                return lanesArray;
            }
            lanesArray = lst;
        } else {
            return lanesArray;
        }
    } else {
        lanesArray = calculateRawTurnLanes(turnLanes, mainTurnType);
    }
    bool isSet = false;
    std::array<int, 3> act = findActiveIndex(prevSegm, currentSegm, lanesArray, nullptr, turnLanes);
    int startIndex = act[0];
    int endIndex = act[1];
    if (startIndex != -1 && endIndex != -1) {
        if (hasAllowedLanes(mainTurnType, lanesArray, startIndex, endIndex)) {
            for (int k = startIndex; k <= endIndex; k++) {
                vector<int> oneActiveLane;
                oneActiveLane.push_back(lanesArray[k]);
                if (hasAllowedLanes(mainTurnType, oneActiveLane, 0, 0)) {
                    lanesArray[k] |= 1;
                }
            }
            isSet = true;
        }
    }
    if (!isSet) {
        // Manually set the allowed lanes.
        isSet = setAllowedLanes(mainTurnType, lanesArray);
    }
    return lanesArray;
}

int highwaySpeakPriority(const string& highway) {
    if (highway == "" || endsWith(highway, "track") || endsWith(highway, "services") || endsWith(highway, "service")
       || endsWith(highway, "path")) {
        return MAX_SPEAK_PRIORITY;
    }
    if (endsWith(highway, "_link")  || endsWith(highway, "unclassified") || endsWith(highway, "road")
        || endsWith(highway, "living_street") || endsWith(highway, "residential"))  {
        return 3;
    }
    if (endsWith(highway, "tertiary")) {
        return 2;
    }
    if (endsWith(highway, "secondary")) {
        return 1;
    }
    return 0;
}

void filterMinorStops(SHARED_PTR<RouteSegmentResult> seg) {
	std::vector<uint32_t> stops;
	bool plus = seg->getStartPointIndex() < seg->getEndPointIndex();
	int next;

	for (int i = seg->getStartPointIndex(); i != seg->getEndPointIndex(); i = next) {
		next = plus ? i + 1 : i - 1;
		std::vector<uint32_t> pointTypes = seg->object->getPointTypes(i);
		if (pointTypes.size() > 0) {
			for (auto it = pointTypes.begin(); it != pointTypes.end(); ++it) {
				if (*it == seg->object->region->stopMinor) {
					stops.push_back(i);
				}
			}
		}
	}

	for (uint32_t stop : stops) {
		std::vector<SHARED_PTR<RouteSegmentResult>> attachedRoutes = seg->getAttachedRoutes(stop);
		for (auto it = attachedRoutes.begin(); it != attachedRoutes.end(); ++it)
		{
			const string highway = (*it)->object->getHighway();
			const string segHighway = seg->object->getHighway();
			int attStopPriority = highwaySpeakPriority(highway);
			int segStopPriority = highwaySpeakPriority(segHighway);
			if (segStopPriority < attStopPriority) {
				seg->object->removePointType(stop, seg->object->region->stopSign);
				break;
			}
		}
	}
}

int countOccurrences(const string& haystack, char needle) {
    int count = 0;
    for (int i = 0; i < haystack.size(); i++) {
        if (haystack[i] == needle) {
            count++;
        }
    }
    return count;
}

vector<int> parseTurnLanes(const SHARED_PTR<RouteDataObject>& ro, double dirToNorthEastPi) {
    string turnLanes;
    if (ro->getOneway() == 0) {
        // we should get direction to detect forward or backward
        double cmp = ro->directionRoute(0, true);
        if (abs(alignAngleDifference(dirToNorthEastPi -cmp)) < M_PI / 2) {
            turnLanes = ro->getValue("turn:lanes:forward");
        } else {
            turnLanes = ro->getValue("turn:lanes:backward");
        }
    } else {
        turnLanes = ro->getValue("turn:lanes");
    }
    if (turnLanes.empty()) {
        return vector<int>();
    }
    return calculateRawTurnLanes(turnLanes, 0);
}

vector<int> parseLanes(const SHARED_PTR<RouteDataObject>& ro, double dirToNorthEastPi) {
    int lns = 0;
    if (ro->getOneway() == 0) {
        // we should get direction to detect forward or backward
        double cmp = ro->directionRoute(0, true);
        
        if (abs(alignAngleDifference(dirToNorthEastPi -cmp)) < M_PI / 2) {
            if (!ro->getValue("lanes:forward").empty()) {
                lns = atoi(ro->getValue("lanes:forward").c_str());
            }
        } else {
            if (!ro->getValue("lanes:backward").empty()) {
                lns = atoi(ro->getValue("lanes:backward").c_str());
            }
        }
        if (lns == 0 && !ro->getValue("lanes").empty()) {
            lns = atoi(ro->getValue("lanes").c_str()) / 2;
        }
    } else {
        lns = atoi(ro->getValue("lanes").c_str());
    }
    if (lns > 0 ) {
        return vector<int>(lns);
    }
    return vector<int>();
}

bool hasTU(string turnLanesPrevSegm, bool attachedOnTheRight) {
	if (!turnLanesPrevSegm.empty()) {
		vector<int> turns = calculateRawTurnLanes(turnLanesPrevSegm, TurnType::C);
		int lane = attachedOnTheRight ? turns[turns.size() - 1] : turns[0];
		vector<int> turnList;
		turnList.push_back(TurnType::getPrimaryTurn(lane));
		turnList.push_back(TurnType::getSecondaryTurn(lane));
		turnList.push_back(TurnType::getTertiaryTurn(lane));
		if (attachedOnTheRight) {
			reverse(turnList.begin(), turnList.end());
		}
		return foundTUturn(turnList);
	}
	return false;
}

bool foundTUturn(vector<int> turnList) {
	for (int t : turnList) {
		if (t != 0) {
			return t == TurnType::TU;
		}
	}
	return false;
}

vector<int> createCombinedTurnTypeForSingleLane(RoadSplitStructure & rs, double currentDeviation) {
    vector<double> attachedAngles;
    attachedAngles.push_back(currentDeviation);
    for (const SHARED_PTR<AttachedRoadInfo> & l : rs.leftLanesInfo) {
        attachedAngles.push_back(l->attachedAngle);
    }
    for (const SHARED_PTR<AttachedRoadInfo> & l : rs.rightLanesInfo) {
        attachedAngles.push_back(l->attachedAngle);
    }
    std::sort(attachedAngles.begin(), attachedAngles.end(), std::greater<double>{});
    int size = (int) attachedAngles.size();
    bool allStraight = rs.allAreStraight();
    vector<int> lanes(1);
    int extraLanes = 0;
    double prevAngle = NAN;
    //iterate from left to right turns
    int prevTurn = 0;
    for (int i = 0; i < size; i++) {
        double angle = attachedAngles[i];
        if (!isnan(prevAngle) && angle == prevAngle) {
            continue;
        }
        prevAngle = angle;
        int turn;
        if (allStraight) {
            //create fork intersection
            if (i == 0) {
                turn = TurnType::KL;
            } else if (i == size - 1) {
                turn = TurnType::KR;
            } else {
                turn = TurnType::C;
            }
        } else {
            turn = getTurnByAngle(angle);
            if (prevTurn > 0 && prevTurn == turn) {
                turn = TurnType::getNext(turn);
            }
        }
        prevTurn = turn;
        if (angle == currentDeviation) {
            TurnType::setPrimaryTurn(lanes, 0, turn);
        } else {
            if (extraLanes++ == 0) {
                TurnType::setSecondaryTurn(lanes, 0, turn);
            } else {
                TurnType::setTertiaryTurn(lanes, 0, turn);
            }
        }
    }
    lanes[0] |= 1;
    return lanes;
}

void synteticAssignTurnTypes(RoadSplitStructure & rs, int mainLaneType, vector<SHARED_PTR<AttachedRoadInfo>> & roads, bool left) {
    auto comparatorByAngle = [left](const SHARED_PTR<AttachedRoadInfo>& o1, const SHARED_PTR<AttachedRoadInfo>& o2) {
            return (left ? 1 : -1) * (o1->attachedAngle < o2->attachedAngle ? -1 : (o1->attachedAngle > o2->attachedAngle ? 1 : 0));
    };
    vector<SHARED_PTR<AttachedRoadInfo>> & col = left ? rs.leftLanesInfo : rs.rightLanesInfo;
    std::sort(col.begin(), col.end(), comparatorByAngle);
    int type = mainLaneType;
    for (int i = (int)col.size() - 1; i >= 0; i--) {
        SHARED_PTR<AttachedRoadInfo> & info = col[i];
        int turnByAngle = getTurnByAngle(info->attachedAngle);
        if (left && turnByAngle >= type) {
            type = TurnType::getPrev(type);
        } else if (!left && turnByAngle <= type) {
            type = TurnType::getNext(type);
        } else {
            type = turnByAngle;
        }
        info->turnType = type;
        roads.push_back(info);
    }
}

SHARED_PTR<TurnType> createSimpleKeepLeftRightTurn(bool leftSide, SHARED_PTR<RouteSegmentResult>& prevSegm,
                                                 SHARED_PTR<RouteSegmentResult>& currentSegm, RoadSplitStructure& rs) {
    double deviation = degreesDiff(prevSegm->getBearingEnd(), currentSegm->getBearingBegin());
    bool makeSlightTurn = abs(deviation) > TURN_SLIGHT_DEGREE;
    SHARED_PTR<TurnType> t = nullptr;
    int mainLaneType = TurnType::C;
    if (rs.keepLeft || rs.keepRight) {
        if (deviation < -TURN_SLIGHT_DEGREE && makeSlightTurn) {
            t = TurnType::ptrValueOf(TurnType::TSLR, leftSide);
            mainLaneType = TurnType::TSLR;
        } else if (deviation > TURN_SLIGHT_DEGREE && makeSlightTurn) {
            t = TurnType::ptrValueOf(TurnType::TSLL, leftSide);
            mainLaneType = TurnType::TSLL;
        } else if (rs.keepLeft && rs.keepRight) {
            t = TurnType::ptrValueOf(TurnType::C, leftSide);
        } else {
            t = TurnType::ptrValueOf(rs.keepLeft ? TurnType::KL : TurnType::KR, leftSide);
        }
    } else {
        return nullptr;
    }
    int currentLanesCount = countLanesMinOne(currentSegm);
    int prevLanesCount = countLanesMinOne(prevSegm);
    bool oneLane = currentLanesCount == 1 && prevLanesCount == 1;
    vector<int> lanes(prevLanesCount);
    if (oneLane) {
        lanes = createCombinedTurnTypeForSingleLane(rs, deviation);
        t->setLanes(lanes);
        int active = t->getActiveCommonLaneTurn();
        if (active > 0 && (!TurnType::isKeepDirectionTurn(active) || !TurnType::isKeepDirectionTurn(t->getValue()))) {
            t = TurnType::ptrValueOf(active, leftSide);
        }
    } else {
        bool ltr = rs.leftLanes < rs.rightLanes;
        vector<SHARED_PTR<AttachedRoadInfo>> roads;
        SHARED_PTR<AttachedRoadInfo> mainType = std::make_shared<AttachedRoadInfo>();
        mainType->lanes = currentLanesCount;
        mainType->speakPriority = highwaySpeakPriority(currentSegm->object->getHighway());
        mainType->turnType = mainLaneType;
        roads.push_back(mainType);
        
        synteticAssignTurnTypes(rs, mainLaneType, roads, true);
        synteticAssignTurnTypes(rs, mainLaneType, roads, false);
        // sort important last
        std::sort(roads.begin(), roads.end(), [](const SHARED_PTR<AttachedRoadInfo>& o1, const SHARED_PTR<AttachedRoadInfo>& o2) {
            if (o1->speakPriority == o2->speakPriority) {
                return o1->lanes > o2->lanes;  // Descending order for lanes
            }
            return o1->speakPriority > o2->speakPriority;  // Descending order for speakPriority
        });
        for (SHARED_PTR<AttachedRoadInfo> & i : roads) {
            int sumLanes = 0;
            for (const SHARED_PTR<AttachedRoadInfo> & l : roads) {
                sumLanes += l->lanes;
            }
            if (sumLanes < 2 * lanes.size()) {
                // max 2 attached per lane is enough
                break;
            }
            i->lanes = 1; // if not enough reset to 1 lane
        }
        
        // active lanes
        int startActive = std::max(0, ltr ? 0 : (int)lanes.size() - mainType->lanes);
        int endActive = std::min((int)lanes.size(), startActive + mainType->lanes) - 1;
        for (int i = startActive; i <= endActive; i++) {
            lanes[i] = (mainType->turnType << 1) + 1;
        }
        int ind = 0;
        for (const SHARED_PTR<AttachedRoadInfo> & i : rs.leftLanesInfo) {
            for (int k = 0; k < i->lanes && ind <= startActive; k++, ind++) {
                if (lanes[ind] == 0) {
                    lanes[ind] = i->turnType << 1;
                } else if (TurnType::getSecondaryTurn(lanes[ind]) == 0) {
                    TurnType::setSecondaryTurn(lanes, ind, i->turnType);
                } else {
                    TurnType::setTertiaryTurn(lanes, ind, i->turnType);
                }
            }
        }
        ind = (int)lanes.size() - 1;
        for (const SHARED_PTR<AttachedRoadInfo> & i : rs.rightLanesInfo) {
            for (int k = 0; k < i->lanes && ind >= endActive; k++, ind--) {
                if (lanes[ind] == 0) {
                    lanes[ind] = i->turnType << 1;
                } else if (TurnType::getSecondaryTurn(lanes[ind]) == 0) {
                    TurnType::setSecondaryTurn(lanes, ind, i->turnType);
                } else {
                    TurnType::setTertiaryTurn(lanes, ind, i->turnType);
                }
            }
        }

        // Fill All left empty slots with inactive C
        for (int i = 0; i < prevLanesCount; i++) {
            if (lanes[i] == 0) {
                lanes[i] = TurnType::C << 1;
            }
        }
    }
    t->setSkipToSpeak(!rs.speak);
    t->setLanes(lanes);
    return t;
}

int inferSlightTurnFromActiveLanes(vector<int>& oLanes, bool mostLeft, bool mostRight) {
    const auto possibleTurns = getPossibleTurns(oLanes, false, false);
    if (possibleTurns.size() == 0) {
        // No common turns, so can't determine anything.
        return 0;
    }
    int infer = 0;
    if (possibleTurns.size() == 1) {
        infer = possibleTurns[0];
    } else if (possibleTurns.size() == 2) {
        // this method could be adapted for 3+ turns
        if (mostLeft && !mostRight) {
            infer = possibleTurns[0];
        } else if (mostRight && !mostLeft) {
            infer = possibleTurns[possibleTurns.size() - 1];
        } else {
            infer = possibleTurns[1];
        }
    }
    return infer;
}

void setActiveLanesRange(vector<int> & rawLanes, int activeBeginIndex, int activeEndIndex, int activeTurn) {
    for (int k = activeBeginIndex; k < rawLanes.size() && k <= activeEndIndex; k++) {
        rawLanes[k] |= 1;
    }
    for (int k = activeBeginIndex; k < rawLanes.size() && k <= activeEndIndex; k++) {
        if(TurnType::getPrimaryTurn(rawLanes[k]) != activeTurn) {
            if(TurnType::getSecondaryTurn(rawLanes[k]) == activeTurn) {
                TurnType::setSecondaryToPrimary(rawLanes, k);
            } else if(TurnType::getTertiaryTurn(rawLanes[k]) == activeTurn) {
                TurnType::setTertiaryToPrimary(rawLanes, k);
            }
        }
    }
}

SHARED_PTR<TurnType> createKeepLeftRightTurnBasedOnTurnTypes(RoadSplitStructure& rs, SHARED_PTR<RouteSegmentResult>& prevSegm,
                                                           SHARED_PTR<RouteSegmentResult>& currentSegm, string turnLanes, bool leftSide) {
    // Maybe going straight at a 90-degree intersection
    auto t = TurnType::ptrValueOf(TurnType::C, leftSide);
    auto rawLanes = calculateRawTurnLanes(turnLanes, TurnType::C);
    bool possiblyLeftTurn = rs.roadsOnLeft == 0;
    bool possiblyRightTurn = rs.roadsOnRight == 0;
    for (int k = 0; k < rawLanes.size(); k++) {
        int turn = TurnType::getPrimaryTurn(rawLanes[k]);
        int sturn = TurnType::getSecondaryTurn(rawLanes[k]);
        int tturn = TurnType::getTertiaryTurn(rawLanes[k]);
        if (turn == TurnType::TU || sturn == TurnType::TU || tturn == TurnType::TU) {
            possiblyLeftTurn = true;
        }
        if (turn == TurnType::TRU || sturn == TurnType::TRU || tturn == TurnType::TRU) {
            possiblyRightTurn = true;
        }
    }
    
    std::array<int, 3> act = findActiveIndex(prevSegm, currentSegm, rawLanes, std::make_shared<RoadSplitStructure>(rs), turnLanes);
    int activeBeginIndex = act[0];
    int activeEndIndex = act[1];
    int activeTurn = act[2];
    bool leftOrRightKeep = (rs.keepLeft && !rs.keepRight) || (!rs.keepLeft && rs.keepRight);
    if (activeBeginIndex == -1 || activeEndIndex == -1 || activeBeginIndex > activeEndIndex) {
        // something went wrong
        return createSimpleKeepLeftRightTurn(leftSide, prevSegm, currentSegm, rs);
    }
    if (leftOrRightKeep) {
        setActiveLanesRange(rawLanes, activeBeginIndex, activeEndIndex, activeTurn);
        int tp = inferSlightTurnFromActiveLanes(rawLanes, rs.keepLeft, rs.keepRight);
        // Checking to see that there is only one unique turn
        if (tp != 0) {
            // add extra lanes with same turn
            for(int i = 0; i < rawLanes.size(); i++) {
                if(TurnType::getSecondaryTurn(rawLanes[i]) == tp) {
                    TurnType::setSecondaryToPrimary(rawLanes, i);
                    rawLanes[i] |= 1;
                } else if(TurnType::getPrimaryTurn(rawLanes[i]) == tp) {
                    rawLanes[i] |= 1;
                }
            }
        }
        if (tp != t->getValue() && tp != 0) {
            t = TurnType::ptrValueOf(tp, leftSide);
        } else {
            // use keepRight and keepLeft turns when attached road doesn't have lanes
            // or prev segment has more then 1 turn to the active lane
            if (rs.keepRight && !rs.keepLeft) {
                t = getTurnByCurrentTurns(rs.leftLanesInfo, rawLanes, TurnType::KR, leftSide);
            } else if (rs.keepLeft && !rs.keepRight) {
                t = getTurnByCurrentTurns(rs.rightLanesInfo, rawLanes, TurnType::KL, leftSide);
            }
        }
    } else {
        if (activeBeginIndex != -1 && activeEndIndex != -1) {
            setActiveLanesRange(rawLanes, activeBeginIndex, activeEndIndex, activeTurn);
            t = getActiveTurnType(rawLanes, leftSide, t);
        }
    }
    t->setLanes(rawLanes);
    t->setPossibleLeftTurn(possiblyLeftTurn);
    t->setPossibleRightTurn(possiblyRightTurn);
    return t;
}

SHARED_PTR<TurnType> getTurnByCurrentTurns(std::vector<SHARED_PTR<AttachedRoadInfo>> & otherSideLanesInfo, vector<int>& rawLanes,
										   int keepTurnType, bool leftSide) {
    vector<int> otherSideTurns;
	if (!otherSideLanesInfo.empty()) {
		for (SHARED_PTR<AttachedRoadInfo> & li : otherSideLanesInfo) {
			for (int i : li->parsedLanes) {
				TurnType::collectTurnTypes(i, otherSideTurns);
			}
		}
	}
    vector<int> currentTurns;
	for (auto const& ln : rawLanes) {
		TurnType::collectTurnTypes(ln, currentTurns);
	}

	// Here we detect single case when turn lane continues on 1 road / single sign and all other lane turns continue on
	// the other side roads
    vector<int> analyzedList(currentTurns.begin(), currentTurns.end());
    if (analyzedList.size() > 1) {
        if (keepTurnType == TurnType::KL) {
            // no need analyze turns in left side (current direction)
            analyzedList.erase(analyzedList.begin());
        } else if (keepTurnType == TurnType::KR) {
            // no need analyze turns in right side (current direction)
            analyzedList.erase(analyzedList.end() - 1);
        }
        if (containsAll(analyzedList, otherSideTurns)) {
            // currentTurns.removeAll(otherSideTurns);
            for (int i : otherSideTurns) {
                auto it = find(currentTurns.begin(), currentTurns.end(), i);
                if (it != currentTurns.end()) {
                    currentTurns.erase(it);
                }
            }
            if (currentTurns.size() == 1) {
                return TurnType::ptrValueOf(*currentTurns.begin(), leftSide);
            }
        } else {
            // Avoid "keep" instruction if active side contains only "through" moving
            if ((keepTurnType == TurnType::KL && currentTurns[0] == TurnType::C)) {
                return TurnType::ptrValueOf(TurnType::C, leftSide);
            }
            if (keepTurnType == TurnType::KR && currentTurns[currentTurns.size() - 1] == TurnType::C) {
                return TurnType::ptrValueOf(TurnType::C, leftSide);
            }
        }
    }

	return TurnType::ptrValueOf(keepTurnType, leftSide);
}

bool containsAll(vector<int> a, vector<int> b) {
	auto notFound = a.end();
	for (auto element : b)
		if (std::find(a.begin(), a.end(), element) == notFound) return false;
	return true;
}

SHARED_PTR<TurnType> attachKeepLeftInfoAndLanes(bool leftSide, SHARED_PTR<RouteSegmentResult>& prevSegm, SHARED_PTR<RouteSegmentResult>& currentSegm, bool twiceRoadPresent) {
    auto attachedRoutes = currentSegm->getAttachedRoutes(currentSegm->getStartPointIndex());
    if (attachedRoutes.empty()) {
        return nullptr;
    }
    // keep left/right
    string turnLanesPrevSegm = twiceRoadPresent ? "" : getTurnLanesString(prevSegm);
    RoadSplitStructure rs = calculateRoadSplitStructure(prevSegm, currentSegm, attachedRoutes, turnLanesPrevSegm);
    if(rs.roadsOnLeft  + rs.roadsOnRight == 0) {
        return nullptr;
    }
    
    // turn lanes exist
    if (!turnLanesPrevSegm.empty()) {
        return createKeepLeftRightTurnBasedOnTurnTypes(rs, prevSegm, currentSegm, turnLanesPrevSegm, leftSide);
    }
    
    // turn lanes don't exist
    if (rs.keepLeft || rs.keepRight) {
        return createSimpleKeepLeftRightTurn(leftSide, prevSegm, currentSegm, rs);
    }
    return nullptr;
}

SHARED_PTR<TurnType> getTurnInfo(vector<SHARED_PTR<RouteSegmentResult> >& result, int i, bool leftSide) {
    if (i == 0) {
        return TurnType::ptrValueOf(TurnType::C, false);
    }
    RoundaboutTurn roundaboutTurn(result, i, leftSide);
    if (roundaboutTurn.isRoundaboutExist()) {
        return roundaboutTurn.getRoundaboutType();
    }
    auto prev = result[i - 1];
    auto rr = result[i];
    SHARED_PTR<TurnType> t = nullptr;
	if (prev != nullptr)
	{
		// avoid small zigzags is covered at (search for "zigzags")
		float bearingDist = DIST_BEARING_DETECT;
		if (UNMATCHED_HIGHWAY_TYPE == rr->object->getHighway()) {
			bearingDist = DIST_BEARING_DETECT_UNMATCHED;
		}
		double mpi = degreesDiff(prev->getBearingEnd(prev->getEndPointIndex(),std::min(prev->distance,bearingDist)),
		 rr->getBearingBegin(rr->getStartPointIndex(),std::min(rr->distance, bearingDist)));
        
        string turnTag = getTurnString(rr);
        bool twRoadPresent = twiceRoadPresent(result, i);
        if (!turnTag.empty()) {
            int fromTag = TurnType::convertType(turnTag);
            if (!TurnType::isSlightTurn(fromTag)) {
                t = TurnType::ptrValueOf(fromTag, leftSide);
                const vector<int>& lanes = getTurnLanesInfo(prev, rr, t->getValue());
                t = getActiveTurnType(lanes, leftSide, t);
                t->setLanes(lanes);
            } else if (fromTag != TurnType::C) {
                t = attachKeepLeftInfoAndLanes(leftSide, prev, rr, twRoadPresent);
                if (t) {
                    SHARED_PTR<TurnType> mainTurnType = TurnType::ptrValueOf(fromTag, leftSide);
                    const vector<int>& lanes = t->getLanes();
                    t = getActiveTurnType(t->getLanes(), leftSide, mainTurnType);
                    t->setLanes(lanes);
                }
            }
            if (t) {
                t->setTurnAngle((float) - mpi);
                return t;
            }
        }
        
		if (mpi >= TURN_DEGREE_MIN) {
			if (mpi < TURN_DEGREE_MIN) {
				// Slight turn detection here causes many false positives where drivers would expect a "normal" TL. Best use limit-angle=TURN_DEGREE_MIN, this reduces TSL to the turn-lanes cases.
				t = TurnType::ptrValueOf(TurnType::TSLL, leftSide);
			} else if (mpi < 120) {
				t = TurnType::ptrValueOf(TurnType::TL, leftSide);
			} else if (mpi < 150 || leftSide) {
				t = TurnType::ptrValueOf(TurnType::TSHL, leftSide);
			} else {
				t = TurnType::ptrValueOf(TurnType::TU, leftSide);
			}
			const auto& lanes = getTurnLanesInfo(prev, rr, t->getValue());
            t = getActiveTurnType(lanes, leftSide, t);
			t->setLanes(lanes);
		} else if (mpi < -TURN_DEGREE_MIN) {
			if (mpi > -TURN_DEGREE_MIN) {
				t = TurnType::ptrValueOf(TurnType::TSLR, leftSide);
			} else if (mpi > -120) {
				t = TurnType::ptrValueOf(TurnType::TR, leftSide);
			} else if (mpi > -150 || !leftSide) {
				t = TurnType::ptrValueOf(TurnType::TSHR, leftSide);
			} else {
				t = TurnType::ptrValueOf(TurnType::TRU, leftSide);
			}
			const auto& lanes = getTurnLanesInfo(prev, rr, t->getValue());
            t = getActiveTurnType(lanes, leftSide, t);
			t->setLanes(lanes);
		} else {
			t = attachKeepLeftInfoAndLanes(leftSide, prev, rr, twRoadPresent);
		}
		if (t) {
			t->setTurnAngle((float) -mpi);
		}
	}

    return t;
}

bool mergeTurnLanes(bool leftSide, SHARED_PTR<RouteSegmentResult>& currentSegment, SHARED_PTR<RouteSegmentResult>& nextSegment) {
    MergeTurnLaneTurn active(currentSegment);
    MergeTurnLaneTurn target(nextSegment);
    if (active.activeLen < 2) {
        return false;
    }
    if (target.activeStartIndex == -1) {
        return false;
    }
    bool changed = false;
    if (target.isActiveTurnMostLeft()) {
        // let only the most left lanes be enabled
        if (target.activeLen < active.activeLen) {
            active.activeEndIndex -= (active.activeLen - target.activeLen);
            changed = true;
        }
    } else if (target.isActiveTurnMostRight()) {
        // next turn is right
        // let only the most right lanes be enabled
        if (target.activeLen < active.activeLen ) {
            active.activeStartIndex += (active.activeLen - target.activeLen);
            changed = true;
        }
    } else {
        // next turn is get through (take out the left and the right turn)
        if (target.activeLen < active.activeLen) {
            if (target.originalLanes.size() == active.activeLen) {
                active.activeEndIndex = active.activeStartIndex + target.activeEndIndex;
                active.activeStartIndex = active.activeStartIndex + target.activeStartIndex;
                changed = true;
            } else {
                int straightActiveLen = 0;
                int straightActiveBegin = -1;
                for (int i = active.activeStartIndex; i <= active.activeEndIndex; i++) {
                    if (TurnType::hasAnyTurnLane(active.originalLanes[i], TurnType::C)) {
                        straightActiveLen++;
                        if(straightActiveBegin == -1) {
                            straightActiveBegin = i;
                        }
                    }
                }
                if (straightActiveBegin != -1 && straightActiveLen <= target.activeLen) {
                    active.activeStartIndex = straightActiveBegin;
                    active.activeEndIndex = straightActiveBegin + straightActiveLen - 1;
                    changed = true;
                } else {
                    // cause the next-turn goes forward exclude left most and right most lane
                    if (active.activeStartIndex == 0) {
                        active.activeStartIndex++;
                        active.activeLen--;
                    }
                    if (active.activeEndIndex == active.originalLanes.size() - 1) {
                        active.activeEndIndex--;
                        active.activeLen--;
                    }
                    float ratio = (active.activeLen - target.activeLen) / 2.f;
                    if (ratio > 0) {
                        active.activeEndIndex = (int) ceil(active.activeEndIndex - ratio);
                        active.activeStartIndex = (int) floor(active.activeStartIndex + ratio);
                    }
                    changed = true;
                }
            }
        }
    }
    if (!changed) {
        return false;
    }
    
    // set the allowed lane bit
    for (int i = 0; i < active.disabledLanes.size(); i++) {
        if (i >= active.activeStartIndex && i <= active.activeEndIndex && 
            active.originalLanes[i] % 2 == 1) {
            active.disabledLanes[i] |= 1;
        }
    }
    const auto& currentTurn = currentSegment->turnType;
    currentTurn->setLanes(active.disabledLanes);
    return true;
}

void inferCommonActiveLane(const SHARED_PTR<TurnType>& currentTurn, const SHARED_PTR<TurnType>& nextTurn) {
    auto& lanes = currentTurn->getLanes();
    set<int> turnSet;
    for (int i = 0; i < lanes.size(); i++) {
        if (lanes[i] % 2 == 1 ) {
            int singleTurn = TurnType::getPrimaryTurn(lanes[i]);
            turnSet.insert(singleTurn);
            if (TurnType::getSecondaryTurn(lanes[i]) != 0) {
                turnSet.insert(TurnType::getSecondaryTurn(lanes[i]));
            }
            if (TurnType::getTertiaryTurn(lanes[i]) != 0) {
                turnSet.insert(TurnType::getTertiaryTurn(lanes[i]));
            }
        }
    }
    int singleTurn = 0;
    if (turnSet.size() == 1) {
        singleTurn = *turnSet.begin();
    } else if ((currentTurn->goAhead() || currentTurn->keepLeft() || currentTurn->keepRight())
               && turnSet.find(nextTurn->getValue()) != turnSet.end()) {
        int nextTurnLane = nextTurn->getActiveCommonLaneTurn();
        if (currentTurn->isPossibleLeftTurn() && TurnType::isLeftTurn(nextTurn->getValue())) {
            singleTurn = nextTurn->getValue();
        } else if(currentTurn->isPossibleLeftTurn() && TurnType::isLeftTurn(nextTurnLane)) {
            singleTurn = nextTurnLane;
        } else if (currentTurn->isPossibleRightTurn() && TurnType::isRightTurn(nextTurn->getValue())) {
            singleTurn = nextTurn->getValue();
        } else if (currentTurn->isPossibleRightTurn() && TurnType::isRightTurn(nextTurnLane)) {
            singleTurn = nextTurnLane;
        } else if ((currentTurn->goAhead() || currentTurn->keepLeft() || currentTurn->keepRight())
                   && TurnType::isKeepDirectionTurn(nextTurnLane)) {
            singleTurn = nextTurnLane;
        }
    }
    if (singleTurn == 0) {
        singleTurn = currentTurn->getValue();
        if (singleTurn == TurnType::KL || singleTurn == TurnType::KR) {
            return;
        }
    }
    for (int i = 0; i < lanes.size(); i++) {
        if (lanes[i] % 2 == 1 && TurnType::getPrimaryTurn(lanes[i]) != singleTurn) {
            if (TurnType::getSecondaryTurn(lanes[i]) == singleTurn) {
                TurnType::setSecondaryTurn(lanes, i, TurnType::getPrimaryTurn(lanes[i]));
                TurnType::setPrimaryTurn(lanes, i, singleTurn);
            } else if (TurnType::getTertiaryTurn(lanes[i]) == singleTurn) {
                TurnType::setTertiaryTurn(lanes, i, TurnType::getPrimaryTurn(lanes[i]));
                TurnType::setPrimaryTurn(lanes, i, singleTurn);
            } else {
                if (lanes.size() == 1)
                {
                    return;
                }
                // disable lane
                lanes[i] = lanes[i] - 1;
            }
        }
    }
}

void inferActiveTurnLanesFromTurn(const SHARED_PTR<TurnType>& tt, int type) {
    bool found = false;
    auto& lanes = tt->getLanes();
    if (tt->getValue() == type && !lanes.empty()) {
        for (int it = 0; it < lanes.size(); it++) {
            int turn = lanes[it];
            if (TurnType::getPrimaryTurn(turn) == type ||
                TurnType::getSecondaryTurn(turn) == type ||
                TurnType::getTertiaryTurn(turn) == type) {
                found = true;
                break;
            }
        }
    }
    if (found) {
        for (int it = 0; it < lanes.size(); it++) {
            int turn = lanes[it];
            if (TurnType::getPrimaryTurn(turn) != type) {
                if (TurnType::getSecondaryTurn(turn) == type) {
                    int st = TurnType::getSecondaryTurn(turn);
                    TurnType::setSecondaryTurn(lanes, it, TurnType::getPrimaryTurn(turn));
                    TurnType::setPrimaryTurn(lanes, it, st);
                } else if (TurnType::getTertiaryTurn(turn) == type) {
                    int st = TurnType::getTertiaryTurn(turn);
                    TurnType::setTertiaryTurn(lanes, it, TurnType::getPrimaryTurn(turn));
                    TurnType::setPrimaryTurn(lanes, it, st);
                } else {
                    lanes[it] = turn & (~1);
                }
            } else {
                lanes[it] = turn | 1;
            }
        }
    }
}

bool sortArray(int o1, int o2) {
    return TurnType::orderFromLeftToRight(o1) < TurnType::orderFromLeftToRight(o2);
}

vector<int> getPossibleTurns(vector<int>& oLanes, bool onlyPrimary, bool uniqueFromActive) {
    set<int> possibleTurns;
    set<int> upossibleTurns;
    for (int i = 0; i < oLanes.size(); i++) {
        // Nothing is in the list to compare to, so add the first elements
        upossibleTurns.clear();
        upossibleTurns.insert(TurnType::getPrimaryTurn(oLanes[i]));
        if (!onlyPrimary && TurnType::getSecondaryTurn(oLanes[i]) != 0) {
            upossibleTurns.insert(TurnType::getSecondaryTurn(oLanes[i]));
        }
        if (!onlyPrimary && TurnType::getTertiaryTurn(oLanes[i]) != 0) {
            upossibleTurns.insert(TurnType::getTertiaryTurn(oLanes[i]));
        }
        if (!uniqueFromActive) {
            possibleTurns.insert(upossibleTurns.begin(), upossibleTurns.end());
            //                if (!possibleTurns.isEmpty()) {
            //                    possibleTurns.retainAll(upossibleTurns);
            //                    if(possibleTurns.isEmpty()) {
            //                        break;
            //                    }
            //                } else {
            //                    possibleTurns.addAll(upossibleTurns);
            //                }
        } else if ((oLanes[i] & 1) == 1) {
            if (possibleTurns.size() > 0) {
                auto it = possibleTurns.begin();
                while(it != possibleTurns.end()) {
                    auto current = it++;
                    if (upossibleTurns.find(*current) == upossibleTurns.end())
                        possibleTurns.erase(current);
                }
                if(possibleTurns.size() == 0) {
                    break;
                }
            } else {
                possibleTurns.insert(upossibleTurns.begin(), upossibleTurns.end());
            }
        }
    }
    // Remove all turns from lanes not selected...because those aren't it
    if (uniqueFromActive) {
        for (int i = 0; i < oLanes.size(); i++) {
            if ((oLanes[i] & 1) == 0) {
                possibleTurns.erase(TurnType::getPrimaryTurn(oLanes[i]));
                if (TurnType::getSecondaryTurn(oLanes[i]) != 0) {
                    possibleTurns.erase(TurnType::getSecondaryTurn(oLanes[i]));
                }
                if (TurnType::getTertiaryTurn(oLanes[i]) != 0) {
                    possibleTurns.erase(TurnType::getTertiaryTurn(oLanes[i]));
                }
            }
        }
    }
    vector<int> array(possibleTurns.begin(), possibleTurns.end());
    sort(array.begin(), array.end(), sortArray);
    return array;
}

vector<int> getPossibleTurnsFromActiveLanes(vector<int>& oLanes, bool onlyPrimary)
{
    return getPossibleTurns(oLanes, onlyPrimary, true);
}

void replaceConfusingKeepTurnsWithLaneTurn(SHARED_PTR<RouteSegmentResult> & currentSegment, bool leftSide) {
    if (!currentSegment->turnType) {
        return;
    }
    int currentTurn = currentSegment->turnType->getValue();
    int activeTurn = currentSegment->turnType->getActiveCommonLaneTurn();
    bool changeToActive = false;
    if (TurnType::isKeepDirectionTurn(currentTurn) && !TurnType::isKeepDirectionTurn(activeTurn)) {
        if (TurnType::isLeftTurn(currentTurn) && !TurnType::isLeftTurn(activeTurn)) {
            changeToActive = true;
        }
        if (TurnType::isRightTurn(currentTurn) && !TurnType::isRightTurn(activeTurn)) {
            changeToActive = true;
        }
    }
    if (changeToActive) {
        SHARED_PTR<TurnType> turn = TurnType::ptrValueOf(activeTurn, leftSide);
        turn->setLanes(currentSegment->turnType->getLanes());
        currentSegment->turnType = turn;
    }
}

void determineTurnsToMerge(bool leftside, vector<SHARED_PTR<RouteSegmentResult> >& result) {
    SHARED_PTR<RouteSegmentResult> nextSegment = nullptr;
    double dist = 0;
    for (int i = (int)result.size() - 1; i >= 0; i--) {
        auto currentSegment = result[i];
        const auto& currentTurn = currentSegment->turnType;
        dist += currentSegment->distance;
        if (!currentTurn || currentTurn->getLanes().empty()) {
            // skip
        } else {
            bool merged = false;
            if (nextSegment) {
                string hw = currentSegment->object->getHighway();
                double mergeDistance = 200;
                if (startsWith(hw, "trunk") || startsWith(hw, "motorway")) {
                    mergeDistance = 400;
                }
                if (dist < mergeDistance) {
                    mergeTurnLanes(leftside, currentSegment, nextSegment);
                    inferCommonActiveLane(currentSegment->turnType, nextSegment->turnType);
                    merged = true;
                    replaceConfusingKeepTurnsWithLaneTurn(currentSegment, leftside);
                }
            }
            if (!merged) {
                const auto& tt = currentSegment->turnType;
                inferActiveTurnLanesFromTurn(tt, tt->getValue());
            }
            nextSegment = currentSegment;
            dist = 0;
        }
    }
}

string getStreetName(vector<SHARED_PTR<RouteSegmentResult> >& result, int i, bool dir) {
    string nm = result[i]->object->getName();
    if (nm.empty()) {
        if (!dir) {
            if (i > 0) {
                nm = result[i - 1]->object->getName();
            }
        } else {
            if(i < result.size() - 1) {
                nm = result[i + 1]->object->getName();
            }
        }
    }
    return nm;
}

SHARED_PTR<TurnType> justifyUTurn(bool leftside, vector<SHARED_PTR<RouteSegmentResult> >& result, int i, const SHARED_PTR<TurnType>& t) {
    bool tl = TurnType::isLeftTurnNoUTurn(t->getValue());
    bool tr = TurnType::isRightTurnNoUTurn(t->getValue());
    if (tl || tr) {
        const auto& tnext = result[i + 1]->turnType;
        if (tnext && result[i]->distance < 50) {
            bool ut = true;
            if (i > 0) {
                double uTurn = degreesDiff(result[i - 1]->getBearingEnd(), result[i + 1]->getBearingBegin());
                if (abs(uTurn) < 120) {
                    ut = false;
                }
            }
            //				String highway = result->get(i).getObject().getHighway();
            //				if(highway == null || highway.endsWith("track") || highway.endsWith("services") || highway.endsWith("service")
            //						|| highway.endsWith("path")) {
            //					ut = false;
            //				}
            if (result[i - 1]->object->getOneway() == 0 || result[i + 1]->object->getOneway() == 0) {
                ut = false;
            }
            if (getStreetName(result, i - 1, false) != getStreetName(result, i + 1, true)) {
                ut = false;
            }
            if (ut) {
                tnext->setSkipToSpeak(true);
                if (tl && TurnType::isLeftTurnNoUTurn(tnext->getValue())) {
                    auto tt = TurnType::ptrValueOf(TurnType::TU, false);
                    tt->setLanes(t->getLanes());
                    return tt;
                } else if (tr && TurnType::isRightTurnNoUTurn(tnext->getValue())) {
                    auto tt = TurnType::ptrValueOf(TurnType::TU, true);
                    tt->setLanes(t->getLanes());
                    return tt;
                }
            }
        }
    }
    return nullptr;
}

void justifyUTurns(bool leftSide, vector<SHARED_PTR<RouteSegmentResult> >& result) {
    int next;
    for (int i = 1; i < ((int) result.size() - 1); i = next) {
        next = i + 1;
        const auto& t = result[i]->turnType;
        // justify turn
        if (t) {
            const auto& jt = justifyUTurn(leftSide, result, i, t);
            if (jt) {
                result[i]->turnType = jt;
                next = i + 2;
            }
        }
    }
}

void addTurnInfoDescriptions(vector<SHARED_PTR<RouteSegmentResult> >& result) {
    int prevSegment = -1;
    float dist = 0;
    char distStr[32];
    for (int i = 0; i <= result.size(); i++) {
        if (i == result.size() || result[i]->turnType) {
            if (prevSegment >= 0) {
                string turn = result[prevSegment]->turnType->toString();
                sprintf(distStr, "%.2f", dist);
                result[prevSegment]->description = turn + " and go " + distStr + " meters";
                if (result[prevSegment]->turnType->isSkipToSpeak()) {
                    result[prevSegment]->description = "-*" + result[prevSegment]->description;
                }
            }
            prevSegment = i;
            dist = 0;
        }
        if (i < result.size()) {
            dist += result[i]->distance;
        }
    }
}

bool combineTwoSegmentResult(SHARED_PTR<RouteSegmentResult>& toAdd, SHARED_PTR<RouteSegmentResult>& previous, bool reverse) {
    bool ld = previous->getEndPointIndex() > previous->getStartPointIndex();
    bool rd = toAdd->getEndPointIndex() > toAdd->getStartPointIndex();
    if (rd == ld) {
        if (toAdd->getStartPointIndex() == previous->getEndPointIndex() && !reverse) {
            previous->setEndPointIndex(toAdd->getEndPointIndex());
            previous->routingTime = previous->routingTime + toAdd->routingTime;
            return true;
        } else if (toAdd->getEndPointIndex() == previous->getStartPointIndex() && reverse) {
            previous->setStartPointIndex(toAdd->getStartPointIndex());
            previous->routingTime = previous->routingTime + toAdd->routingTime;
            return true;
        }
    }
    return false;
}

void addRouteSegmentToResult(RoutingContext* ctx, vector<SHARED_PTR<RouteSegmentResult> >& result, SHARED_PTR<RouteSegmentResult>& res, bool reverse) {
    if (res->getStartPointIndex() != res->getEndPointIndex()) {
        if (result.size() > 0) {
            auto last = result.back();
            if (ctx->calculationMode != RouteCalculationMode::BASE) {
                if (combineTwoSegmentResult(res, last, reverse)) {
                    return;
                }
            }
        }
        result.push_back(res);
    }
}

vector<SHARED_PTR<RouteSegmentResult> > convertFinalSegmentToResults(RoutingContext* ctx, const SHARED_PTR<FinalRouteSegment>& finalSegment) {
    vector<SHARED_PTR<RouteSegmentResult> > result;
    if (finalSegment) {
        if (ctx->progress)
            ctx->progress->routingCalculatedTime += finalSegment->distanceFromStart;

        OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "Routing calculated time distance %f", finalSegment->distanceFromStart);
        // Get results from opposite direction roads
        SHARED_PTR<RouteSegment> segment = finalSegment->reverseWaySearch ? finalSegment->getParentRoute() : finalSegment->opposite;
        while (segment.get() != NULL && segment->getRoad() != nullptr) {
            auto res = std::make_shared<RouteSegmentResult>(segment->road, segment->getSegmentEnd(), segment->getSegmentStart());
            float parentRoutingTime = segment->getParentRoute() != nullptr ? segment->getParentRoute()->distanceFromStart : 0;
            res->routingTime = segment->distanceFromStart - parentRoutingTime;
            segment = segment->getParentRoute();
            addRouteSegmentToResult(ctx, result, res, false);
        }
        // reverse it just to attach good direction roads
        std::reverse(result.begin(), result.end());

        segment = finalSegment->reverseWaySearch ? finalSegment->opposite : finalSegment->getParentRoute();
        while (segment.get() != NULL && segment->getRoad() != nullptr) {
            auto res = std::make_shared<RouteSegmentResult>(segment->road, segment->getSegmentStart(), segment->getSegmentEnd());
            float parentRoutingTime = segment->getParentRoute() != nullptr ? segment->getParentRoute()->distanceFromStart : 0;
            res->routingTime = segment->distanceFromStart - parentRoutingTime;
            segment = segment->getParentRoute();
            // happens in smart recalculation
            addRouteSegmentToResult(ctx, result, res, true);
        }
        std::reverse(result.begin(), result.end());
        checkTotalRoutingTime(result, finalSegment->distanceFromStart);
    }
    return result;
}

void checkTotalRoutingTime(vector<SHARED_PTR<RouteSegmentResult>> result, float cmp) {
	float totalRoutingTime = 0;
	for (auto r : result) {
		totalRoutingTime += r->routingTime;
	}
	if (abs(totalRoutingTime - cmp) > 1) {
		OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Debug, "Total sum routing time ! totalRoutingTime=%f  == cmp=%f",
						  totalRoutingTime, cmp);
	}
}

void printAdditionalPointInfo(SHARED_PTR<RouteSegmentResult>& res) {
    bool plus = res->getStartPointIndex() < res->getEndPointIndex();
    for(int k = res->getStartPointIndex(); k != res->getEndPointIndex(); ) {
        if (res->object->pointTypes.size() > k || res->object->pointNameTypes.size() > k) {
            string bld;
            bld.append("<point ").append(std::to_string(k));
            if (res->object->pointTypes.size() > k) {
                auto& tp = res->object->pointTypes[k];
                for (int t = 0; t < tp.size(); t++) {
                    auto& rr = res->object->region->quickGetEncodingRule(tp[t]);
                    bld.append(" ").append(rr.getTag()).append("=\"").append(rr.getValue()).append("\"");
                }
            }
            if (res->object->pointNameTypes.size() > k && res->object->pointNames.size() > k) {
                auto& pointNames = res->object->pointNames[k];
                auto& pointNameTypes = res->object->pointNameTypes[k];
                for (int t = 0; t < pointNameTypes.size(); t++) {
                    auto& rr = res->object->region->quickGetEncodingRule(pointNameTypes[t]);
                    bld.append(" ").append(rr.getTag()).append("=\"").append(pointNames[t]).append("\"");
                }
            }
            bld.append("/>");
            OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Debug, "\t%s", bld.c_str());
        }
        if (plus) {
            k++;
        } else {
            k--;
        }
    }
}

const static bool PRINT_TO_CONSOLE_ROUTE_INFORMATION_TO_TEST = false;

void printResults(RoutingContext* ctx, int startX, int startY, int endX, int endY,
				  vector<SHARED_PTR<RouteSegmentResult>>& result) {
	UNORDERED(map) < string, UNORDERED(map) < string, string >> info;
	UNORDERED(map)<string, string> route;
	route.insert({"routing_time", std::to_string(ctx->progress->routingCalculatedTime - ctx->calculationProgressFirstPhase->routingCalculatedTime)});
	route.insert({"vehicle", ctx->config->routerName});
	route.insert({"base", std::to_string(ctx->calculationMode == RouteCalculationMode::BASE)});
	route.insert({"start_lat", std::to_string(get31LatitudeY(startY))});
	route.insert({"start_lon", std::to_string(get31LongitudeX(startX))});
	route.insert({"target_lat", std::to_string(get31LatitudeY(endY))});
	route.insert({"target_lon", std::to_string(get31LongitudeX(endX))});
	if (!result.empty()) {
		float completeTime = 0;
		float completeDistance = 0;
		for (auto r : result) {
			completeTime += r->segmentTime;
			completeDistance += r->distance;
		}
		route.insert({"complete_distance", std::to_string(completeDistance)});
		route.insert({"complete_time", std::to_string(completeTime)});
	}
	info.insert({"route", route});
	route.insert({"native", std::to_string(true)});

	if (ctx->progress && (ctx->progress->timeToCalculate).GetElapsedMs() > 0) {
		UNORDERED(map) < string, UNORDERED(map) < string,
			string >> info2 = ctx->progress->getInfo(ctx->calculationProgressFirstPhase);
		info.insert(info2.begin(), info2.end());
	}

	string alerts;
	alerts.append("Alerts during routing: ")
		.append(std::to_string(ctx->alertFasterRoadToVisitedSegments))
		.append(" fastRoads,")
		.append(std::to_string(ctx->alertSlowerSegmentedWasVisitedEarlier))
		.append(" slowSegmentsEearlier");
	if (ctx->alertFasterRoadToVisitedSegments + ctx->alertSlowerSegmentedWasVisitedEarlier == 0) {
		alerts = "No alerts";
	}
	OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Debug, " ROUTE. %s", alerts.c_str());
	vector<string> routeInfo;
	string extraInfo = buildRouteMessagesFromInfo(info, routeInfo);
	if (PRINT_TO_CONSOLE_ROUTE_INFORMATION_TO_TEST && !result.empty()) {
		OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Debug, "<test %s>", extraInfo.c_str());
		printRouteInfoSegments(result);
		OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Debug, "</test>");
		// duplicate base info
		if (ctx->calculationProgressFirstPhase) {
			OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Debug, "<<<1st Phase>>>>");
			vector<string> baseRouteInfo;
			buildRouteMessagesFromInfo(ctx->calculationProgressFirstPhase->getInfo(nullptr), baseRouteInfo);
			if (!baseRouteInfo.empty()) {
				for (string msg : baseRouteInfo) {
					OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Debug, "%s", msg.c_str());
				}
			}
			OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Debug, "<<2nd Phase>>>>");
		}
	}
	for (string msg : routeInfo) {
		OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Debug, "%s", msg.c_str());
	}
}

string buildRouteMessagesFromInfo(UNORDERED(map) < string, UNORDERED(map) < string, string >> info,
								  vector<string>& routeMessages) {
	string extraInfo;

	auto it = info.begin();
	while (it != info.end()) {
		string key = it->first;
		UNORDERED(map)<string, string> mp = it->second;
		string msg = "Route <" + key + ">";
		int i = 0;
		auto it2 = mp.begin();
		while (it2 != mp.end()) {
			string mkey = it2->first;
			msg.append((i++ == 0) ? ": " : ", ");
			string valueString = it2->second;
			msg.append(mkey).append("=").append(valueString);
			extraInfo.append(" ").append(key + "_" + mkey).append("=\"").append(valueString).append("\"");
			it2++;
		}
		routeMessages.push_back(msg);
		it++;
	}
	return extraInfo;
}

void printRouteInfoSegments(vector<SHARED_PTR<RouteSegmentResult>>& result) {
	string xmlStr;
	xmlStr.append("\n<trk>\n");
	xmlStr.append("<trkseg>\n");

	double lastHeight = -180;
	for (auto& res : result) {
		string trkSeg;
		string name = res->object->getName();
		string lang = "";
		string ref = res->object->getRef(lang, false, res->isForwardDirection());
		if (!ref.empty()) {
			name += " (" + ref + ") ";
		}
		string additional;
		additional.append("time = \"").append(std::to_string((int)(res->segmentTime) * 100 / 100.0f)).append("\" ");
		if (res->routingTime > 0) {
			additional.append("rtime = \"")
				.append(std::to_string((int)(res->segmentTime) * 100 / 100.0f))
				.append("\" ");
		}
		additional.append("name = \"").append(name).append("\" ");
		//				float ms = res->getSegmentSpeed();
		float ms = res->object->getMaximumSpeed(res->isForwardDirection());
		if (ms > 0) {
			additional.append("maxspeed = \"").append(std::to_string(ms * 3.6f)).append("\" ");
		}
		additional.append("distance = \"").append(std::to_string((int)(res->segmentTime) * 100 / 100.0f)).append("\" ");
		additional.append(res->object->getHighway()).append(" ");
		if (res->turnType) {
			additional.append("turn = \"").append(res->turnType->toString()).append("\" ");
			additional.append("turn_angle = \"").append(std::to_string(res->turnType->getTurnAngle())).append("\" ");
			if (!res->turnType->getLanes().empty()) {
				additional.append("lanes = \"");
				additional.append("[");
				string lanes;
				for (auto l : res->turnType->getLanes()) {
					if (!lanes.empty()) {
						lanes.append(", ");
					}
					lanes.append(std::to_string(l));
				}
				additional.append(lanes).append("]").append("\" ");
				;
			}
		}
		additional.append("start_bearing = \"").append(std::to_string(res->getBearingBegin())).append("\" ");
		additional.append("end_bearing = \"").append(std::to_string(res->getBearingEnd())).append("\" ");
		additional.append("height = \"");
		additional.append("[");
		string hs;
		const auto& heights = res->getHeightValues();
		for (auto h : heights) {
			if (!hs.empty()) {
				hs.append(", ");
			}
			hs.append(std::to_string(h));
		}
		additional.append(hs).append("]").append("\" ");
		additional.append("description = \"").append(res->description).append("\" ");

		OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Debug,
						  "\t<segment id=\"%ld\" oid=\"%ld\" start=\"%d\" end=\"%d\" %s/>",
						  (res->object->getId() >> SHIFT_ID), res->object->getId(), res->getStartPointIndex(),
						  res->getEndPointIndex(), additional.c_str());
		int inc = res->getStartPointIndex() < res->getEndPointIndex() ? 1 : -1;
		int indexnext = res->getStartPointIndex();
		int prevX = 0;
		int prevY = 0;
		for (int index = res->getStartPointIndex(); index != res->getEndPointIndex();) {
			index = indexnext;
			indexnext += inc;

			int x = res->object->pointsX[index];
			int y = res->object->pointsY[index];
			double lat = get31LatitudeY(y);
			double lon = get31LongitudeX(x);

			trkSeg.append("<trkpt");
			trkSeg.append(" lat=\"").append(std::to_string(lat)).append("\"");
			trkSeg.append(" lon=\"").append(std::to_string(lon)).append("\"");
			trkSeg.append(">\n");
			auto& vls = res->object->heightDistanceArray;
			double dist = prevX == 0 ? 0 : measuredDist31(x, y, prevX, prevY);
			if (index * 2 + 1 < vls.size()) {
				auto h = vls[2 * index + 1];
				trkSeg.append("<ele>");
				trkSeg.append(std::to_string(h));
				trkSeg.append("</ele>\n");
				if (lastHeight != -180 && dist > 0) {
					trkSeg.append("<cmt>");
					trkSeg.append(std::to_string((float)((h - lastHeight) / dist * 100)))
						.append("%% degree ")
						.append(std::to_string((float)atan(((h - lastHeight) / dist)) / M_PI * 180))
						.append(" asc ")
						.append(std::to_string((float)(h - lastHeight)))
						.append(" dist ")
						.append(std::to_string((float)dist));
					trkSeg.append("</cmt>\n");
					trkSeg.append("<slope>");
					trkSeg.append(std::to_string((h - lastHeight) / dist * 100));
					trkSeg.append("</slope>\n");
				}
				trkSeg.append("<desc>");
				trkSeg.append(std::to_string(res->object->getId() >> SHIFT_ID))
					.append(" ")
					.append(std::to_string(index));
				trkSeg.append("</desc>\n");
				lastHeight = h;
			} else if (lastHeight != -180) {
				//								serializer.startTag("","ele");
				//								serializer.text(lastHeight +"");
				//								serializer.endTag("","ele");
			}
			trkSeg.append("</trkpt>\n");
			prevX = x;
			prevY = y;
		}
		xmlStr.append(trkSeg);
		printAdditionalPointInfo(res);
	}
	xmlStr.append("</trkseg>\n");
	xmlStr.append("</trk>\n");
	// OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Debug, xmlStr.c_str());
}

bool segmentLineBelongsToPolygon(CombineAreaRoutePoint& p, CombineAreaRoutePoint& n,
        vector<CombineAreaRoutePoint>& originalWay) {
    int intersections = 0;
    int mx = p.x31 / 2 + n.x31 / 2;
    int my = p.y31 / 2 + n.y31 / 2;
    for(int i = 1; i < originalWay.size(); i++) {
        CombineAreaRoutePoint& p2 = originalWay[i - 1];
        CombineAreaRoutePoint& n2 = originalWay[i];
        if(p.originalIndex != i && p.originalIndex != i - 1) {
            if(n.originalIndex != i && n.originalIndex != i - 1) {
                if(linesIntersect(p.x31, p.y31, n.x31, n.y31, p2.x31, p2.y31, n2.x31, n2.y31)) {
                    return false;
                }
			}
		}
        int fx = ray_intersect_x(p2.x31, p2.y31, n2.x31, n2.y31, my);
        if (INT_MIN != fx && mx >= fx) {
            intersections++;
        }
	}
	return intersections % 2 == 1;
}

void simplifyAreaRouteWay(vector<CombineAreaRoutePoint>& routeWay, vector<CombineAreaRoutePoint>& originalWay) {
    bool changed = true;
    while (changed) {
        changed = false;
        uint64_t connectStart = -1;
        uint64_t connectLen = 0;
        double dist = 0;
        int64_t length = routeWay.size() - 1;
        while (length > 0 && connectLen == 0) {
            for (int i = 0; i < routeWay.size() - length; i++) {
                CombineAreaRoutePoint& p = routeWay[i];
                CombineAreaRoutePoint& n = routeWay[i + length];
                if (segmentLineBelongsToPolygon(p, n, originalWay)) {
                    double ndist = squareRootDist31(p.x31, p.y31, n.x31, n.y31);
                    if (ndist > dist) {
                        ndist = dist;
                        connectStart = i;
                        connectLen = length;
                    }
                }
            }
            length--;
		}
		while (connectLen > 1) {
            routeWay.erase(routeWay.begin() + connectStart + 1);
            connectLen--;
            changed = true;
		}
	}
}

void combineWayPointsForAreaRouting(RoutingContext* ctx, vector<SHARED_PTR<RouteSegmentResult> >& result) {
    for(int i = 0; i < result.size(); i++) {
        const auto& rsr = result[i];
        auto& obj = rsr->object;
        bool area = false;
        if(obj->pointsX[0] == obj->pointsX[obj->getPointsLength() - 1] &&
                obj->pointsY[0] == obj->pointsY[obj->getPointsLength() - 1]) {
            area = true;
        }
        if(!area || !ctx->config->router->isArea(obj)) {
            continue;
		}
		vector<CombineAreaRoutePoint> originalWay;
        vector<CombineAreaRoutePoint> routeWay;
        for(int j = 0;  j < obj->getPointsLength(); j++) {
            CombineAreaRoutePoint pnt;
            pnt.x31 = obj->pointsX[j];
            pnt.y31 = obj->pointsY[j];
            pnt.originalIndex = j;

            originalWay.push_back(pnt);
            if(j >= rsr->getStartPointIndex() && j <= rsr->getEndPointIndex()) {
                routeWay.push_back(pnt);
            } else if(j <= rsr->getStartPointIndex() && j >= rsr->getEndPointIndex()) {
                routeWay.insert(routeWay.begin(), pnt);
            }
        }
        uint64_t originalSize = routeWay.size();
        simplifyAreaRouteWay(routeWay, originalWay);
        uint64_t newsize = routeWay.size();
        if (routeWay.size() != originalSize) {
            SHARED_PTR<RouteDataObject> nobj = std::make_shared<RouteDataObject>(obj);
            nobj->pointsX.clear();
            nobj->pointsY.clear();
            nobj->pointsX.resize(newsize);
            nobj->pointsY.resize(newsize);
            for (int k = 0; k < newsize; k++) {
                nobj->pointsX[k] = routeWay[k].x31;
                nobj->pointsY[k] = routeWay[k].y31;
            }
            // in future point names might be used
            nobj->restrictions.clear();
//            nobj->restrictionsVia.clear();
            nobj->pointTypes.clear();
            nobj->pointNames.clear();
            nobj->pointNameTypes.clear();
            auto nrsr = std::make_shared<RouteSegmentResult>(nobj, 0, newsize - 1);
            result[i] = nrsr;
		}
	}
}

void avoidKeepForThroughMoving(vector<SHARED_PTR<RouteSegmentResult> >& result) {
    for (int i = 1; i < result.size(); i++) {
        auto & curr = result[i];
        auto & turnType = curr->turnType;
        if (!turnType) {
            continue;
        }
        if (!turnType->keepLeft() && !turnType->keepRight()) {
            continue;
        }
        if (isSwitchToLink(curr, result[i - 1])) {
            continue;
        }
        if (isKeepTurn(turnType) && isHighSpeakPriority(curr)) {
            continue;
        }
        if (isForkByLanes(curr, result[i - 1])) {
            continue;
        }
        int tt = TurnType::C;
        int cnt = turnType->countTurnTypeDirections(tt, true);
        int cntAll = turnType->countTurnTypeDirections(tt, false);
        if(cnt > 0 && cnt == cntAll) {
            curr->turnType = std::make_shared<TurnType>(tt, turnType->getExitOut(), turnType->getTurnAngle(),
                                                          turnType->isSkipToSpeak(), turnType->getLanes(),
                                                          turnType->isPossibleLeftTurn(), turnType->isPossibleRightTurn());
        }
    }
}

void muteAndRemoveTurns(vector<SHARED_PTR<RouteSegmentResult> >& result) {
    for (int i = 0; i < result.size(); i++) {
        auto & curr = result[i];
        auto & turnType = curr->turnType;
        if (!turnType || turnType->getLanes().empty()) {
            continue;
        }
        int active = turnType->getActiveCommonLaneTurn();
        if (TurnType::isKeepDirectionTurn(active)) {
            if (i > 0 && isSwitchToLink(curr, result[i - 1])) {
                continue;
            }
            if (isKeepTurn(turnType) && isHighSpeakPriority(curr)) {
                continue;
            }
            turnType->setSkipToSpeak(true);
            if (turnType->goAhead()) {
                int uniqDirections = turnType->countDirections();
                if (uniqDirections >= 3) {
                    continue;
                }
                int cnt = turnType->countTurnTypeDirections(TurnType::C, true);
                int cntAll = turnType->countTurnTypeDirections(TurnType::C, false);
                if (cnt == cntAll && cnt >= 2 && (turnType->getLanes().size() - cnt) <= 1) {
                    curr->turnType = nullptr;
                }
            }
        }
    }
}

void prepareTurnResults(RoutingContext* ctx, vector<SHARED_PTR<RouteSegmentResult> >& result) {
    for (int i = 0; i < result.size(); i ++) {
        const auto& turnType = getTurnInfo(result, i, ctx->leftSideNavigation);
        result[i]->turnType = turnType;
    }
    
    determineTurnsToMerge(ctx->leftSideNavigation, result);
    ignorePrecedingStraightsOnSameIntersection(ctx->leftSideNavigation, result);
    justifyUTurns(ctx->leftSideNavigation, result);
    avoidKeepForThroughMoving(result);
    muteAndRemoveTurns(result);
    addTurnInfoDescriptions(result);
}

vector<SHARED_PTR<RouteSegmentResult> > prepareResult(RoutingContext* ctx, vector<SHARED_PTR<RouteSegmentResult> >& result) {
	combineWayPointsForAreaRouting(ctx, result);
	validateAllPointsConnected(result);
	splitRoadsAndAttachRoadSegments(ctx, result);
	for (auto it = result.begin(); it != result.end(); ++it) {
		filterMinorStops(*it);
	}
	calculateTimeSpeed(ctx, result);
	prepareTurnResults(ctx, result);
	return result;
}

vector<SHARED_PTR<RouteSegmentResult> > prepareResult(RoutingContext* ctx, const SHARED_PTR<FinalRouteSegment>& finalSegment) {
    auto result = convertFinalSegmentToResults(ctx, finalSegment);
    prepareResult(ctx, result);
    return result;
}

vector<int> getUniqTurnTypes(const string & turnLanes) {
    std::set<int> tSet;
    vector<int> uniq;
    vector<string> splitLaneOptions = split_string(turnLanes, "|");
    for (int i = 0; i < splitLaneOptions.size(); i++) {
        vector<string> laneOptions = split_string(splitLaneOptions[i], ";");
        for (int j = 0; j < laneOptions.size(); j++) {
            int turn = TurnType::convertType(laneOptions[j]);
            auto result = tSet.insert(turn);
            if (result.second) {
                uniq.push_back(turn);
            }
        }
    }
    return uniq;
}

bool hasTurn(const string & turnLanes, int turnType) {
    if (turnLanes.empty()) {
        return false;
    }
    vector<int> uniqTurnTypes = getUniqTurnTypes(turnLanes);
    for (int lane : uniqTurnTypes) {
        if (lane == turnType) {
            return true;
        }
    }
    return false;
}

bool hasSharpOrReverseTurnLane(const string & turnLanes) {
    if (turnLanes.empty()) {
        return false;
    }
    vector<int> uniqTurnTypes = getUniqTurnTypes(turnLanes);
    for (int lane : uniqTurnTypes) {
        if (TurnType::isSharpOrReverse(lane)) {
            return true;
        }
    }
    return false;
}

bool hasSameTurnLanes(SHARED_PTR<RouteSegmentResult> & prevSegm, SHARED_PTR<RouteSegmentResult> & currentSegm) {
    string turnLanesPrevSegm = getTurnLanesString(prevSegm);
    string turnLanesCurrSegm = getTurnLanesString(currentSegm);
    if (turnLanesPrevSegm.empty() || turnLanesCurrSegm.empty()) {
        return false;
    }
    vector<int> uniqPrev = getUniqTurnTypes(turnLanesPrevSegm);
    vector<int> uniqCurr = getUniqTurnTypes(turnLanesCurrSegm);
    if (uniqPrev.size() != uniqCurr.size()) {
        return false;
    }
    for (int i = 0; i < uniqCurr.size(); i++) {
        if (uniqPrev[i] != uniqCurr[i]) {
            return false;
        }
    }
    return true;
}

bool hasAllowedLanes(int mainTurnType, vector<int>& lanesArray, int startActiveIndex, int endActiveIndex) {
    if (lanesArray.size() == 0 || startActiveIndex > endActiveIndex) {
        return false;
    }
    vector<int> activeLines;
    for (int i = startActiveIndex; i <= endActiveIndex; i++) {
        activeLines.push_back(lanesArray[i]);
    }
    bool possibleSharpLeftOrUTurn = startActiveIndex == 0;
    bool possibleSharpRightOrUTurn = endActiveIndex == lanesArray.size() - 1;
    for (int i = 0; i < activeLines.size(); i++) {
        int turnType = TurnType::getPrimaryTurn(activeLines[i]);
        if (turnType == mainTurnType) {
            return true;
        }
        if (TurnType::isLeftTurnNoUTurn(mainTurnType) && TurnType::isLeftTurnNoUTurn(turnType)) {
            return true;
        }
        if (TurnType::isRightTurnNoUTurn(mainTurnType) && TurnType::isRightTurnNoUTurn(turnType)) {
            return true;
        }
        if (mainTurnType == TurnType::C && TurnType::isSlightTurn(turnType)) {
            return true;
        }
        if (possibleSharpLeftOrUTurn && TurnType::isSharpLeftOrUTurn(mainTurnType) && TurnType::isSharpLeftOrUTurn(turnType)) {
            return true;
        }
        if (possibleSharpRightOrUTurn && TurnType::isSharpRightOrUTurn(mainTurnType) && TurnType::isSharpRightOrUTurn(turnType)) {
            return true;
        }
    }
    return false;
}

SHARED_PTR<TurnType> getActiveTurnType(const vector<int>& lanes, bool leftSide, SHARED_PTR<TurnType> oldTurnType) {
    if (lanes.size() == 0) {
        return oldTurnType;
    }
    int tp = oldTurnType->getValue();
    int cnt = 0;
    for (int k = 0; k < lanes.size(); k++) {
        int ln = lanes[k];
        if ((ln & 1) > 0) {
            vector<int> oneActiveLane;
            oneActiveLane.push_back(lanes[k]);
            if (hasAllowedLanes(oldTurnType->getValue(), oneActiveLane, 0, 0)) {
                 tp = TurnType::getPrimaryTurn(lanes[k]);
            }
            cnt++;
        }
    }
    SHARED_PTR<TurnType> t = TurnType::ptrValueOf(tp, leftSide);
    // mute when most lanes have a straight/slight direction
    if (cnt >= 3 && TurnType::isSlightTurn(t->getValue())) {
        t->setSkipToSpeak(true);
    }
    return t;
}

int getTurnByAngle(double angle) {
    int turnType = TurnType::C;
    if (angle < -150) {
        turnType = TurnType::TRU;
    } else if (angle < -120) {
        turnType = TurnType::TSHR;
    } else if (angle < -TURN_DEGREE_MIN) {
        turnType = TurnType::TR;
    } else if (angle <= -TURN_SLIGHT_DEGREE) {
        turnType = TurnType::TSLR;
    } else if (angle > -TURN_SLIGHT_DEGREE && angle < TURN_SLIGHT_DEGREE) {
        turnType = TurnType::C;
    } else if (angle < TURN_DEGREE_MIN) {
        turnType = TurnType::TSLL;
    } else if (angle < 120) {
        turnType = TurnType::TL;
    } else if (angle < 150) {
        turnType = TurnType::TSHL;
    } else {
        turnType = TurnType::TU;
    }
    return turnType;
}

bool isSwitchToLink(SHARED_PTR<RouteSegmentResult>& curr, SHARED_PTR<RouteSegmentResult>& prev) {
    string c = curr->object->getHighway();
    string p = prev->object->getHighway();
    return c.find("_link") != std::string::npos && p.find("_link") != std::string::npos;
}

bool twiceRoadPresent(vector<SHARED_PTR<RouteSegmentResult> >& result, int i) {
    if (i > 0 && i < result.size() - 1) {
        SHARED_PTR<RouteSegmentResult> & prev = result.at(i - 1);
        string turnLanes = getTurnLanesString(prev);
        if (turnLanes.empty()) {
            return false;
        }
        SHARED_PTR<RouteSegmentResult> & curr = result.at(i);
        SHARED_PTR<RouteSegmentResult> & next = result.at(i + 1);
        if (prev->object->getId() == curr->object->getId()) {
            vector<SHARED_PTR<RouteSegmentResult>> attachedRoutes = next->getAttachedRoutes(next->getStartPointIndex());
            //check if turn lanes allowed for next segment
            return attachedRoutes.size() > 0;
        } else {
            vector<SHARED_PTR<RouteSegmentResult>> attachedRoutes = curr->getAttachedRoutes(curr->getStartPointIndex());
            for (SHARED_PTR<RouteSegmentResult> & attach : attachedRoutes) {
                if (attach->object->getId() == prev->object->getId()) {
                    //check if road the continue in attached roads
                    return true;
                }
            }
        }
    }
    return false;
}

bool isKeepTurn(SHARED_PTR<TurnType> t) {
    return t->keepLeft() || t->keepRight();
}

bool isHighSpeakPriority(SHARED_PTR<RouteSegmentResult>& curr) {
    vector<SHARED_PTR<RouteSegmentResult>> attachedRoutes = curr->getAttachedRoutes(curr->getStartPointIndex());
    string h = curr->object->getHighway();
    for (SHARED_PTR<RouteSegmentResult> & attach : attachedRoutes) {
        string c = attach->object->getHighway();
        if(highwaySpeakPriority(h) >= highwaySpeakPriority(c)) {
			return true;
		}
	}
	return false;
}

bool isForkByLanes(SHARED_PTR<RouteSegmentResult>& curr, SHARED_PTR<RouteSegmentResult>& prev) {
    //check for Y-intersections with many lanes
    if (countLanesMinOne(curr) < countLanesMinOne(prev)) {
        vector<SHARED_PTR<RouteSegmentResult>> attachedRoutes = curr->getAttachedRoutes(curr->getStartPointIndex());
        if (attachedRoutes.size() == 1) {
            return countLanesMinOne(attachedRoutes.at(0)) >= 2;
        }
    }
    return false;
}

#endif /*_OSMAND_ROUTE_RESULT_PREPARATION_CPP*/
