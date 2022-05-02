#include "binaryRoutePlanner.h"

#include <functional>
#include <iterator>
#include <queue>

#include "Logging.h"
#include "binaryRead.h"
#include "routingContext.h"

//	static bool PRINT_TO_CONSOLE_ROUTE_INFORMATION_TO_TEST = true;

static const bool TRACE_ROUTING = false;
static const bool PRINT_ROUTING_ALERTS = false;

// Check issue #8649
static const double GPS_POSSIBLE_ERROR = 7;

inline int roadPriorityComparator(float o1DistanceFromStart, float o1DistanceToEnd, float o2DistanceFromStart,
								  float o2DistanceToEnd, float heuristicCoefficient) {
	// f(x) = g(x) + h(x)  --- g(x) - distanceFromStart, h(x) - distanceToEnd (not exact)
	float f1 = o1DistanceFromStart + heuristicCoefficient * o1DistanceToEnd;
	float f2 = o2DistanceFromStart + heuristicCoefficient * o2DistanceToEnd;
	if (f1 == f2) {
		return 0;
	}
	return f1 < f2 ? -1 : 1;
}

void printRoad(const char* prefix, RouteSegment* segment) {
    const auto& parentRoute = segment->getParentRoute();
	OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Debug, "%s Road id=%lld ind=%d->%d ds=%f es=%f pend=%d parent=%lld",
					  prefix, segment->getRoad()->id / 64, segment->getSegmentStart(), segment->getSegmentEnd(),
					  segment->distanceFromStart, segment->distanceToEnd,
                      parentRoute != nullptr ? parentRoute->segmentEnd : 0,
                      parentRoute != nullptr && parentRoute->road != nullptr
						  ? parentRoute->getRoad()->id / 64
						  : 0);
}

void printRoad(const char* prefix, SHARED_PTR<RouteSegment>& segment) { printRoad(prefix, segment.get()); }

int64_t calculateRoutePointInternalId(SHARED_PTR<RouteDataObject>& road, int pntId, int nextPntId) {
	int positive = nextPntId - pntId;
	int pntLen = road->getPointsLength();
	if (pntId < 0 || nextPntId < 0 || pntId >= pntLen || nextPntId >= pntLen || (positive != -1 && positive != 1)) {
		OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Warning, "Assert failed route point (should never happen)");
	}
	return (road->id << ROUTE_POINTS) + (pntId << 1) + (positive > 0 ? 1 : 0);
}

int64_t calculateRoutePointId(SHARED_PTR<RouteSegment>& segm) {
	return calculateRoutePointInternalId(
		segm->getRoad(), segm->getSegmentStart(),
		segm->isPositive() ? segm->getSegmentStart() + 1 : segm->getSegmentStart() - 1);
	// return calculateRoutePointInternalId(segm.getRoad(), segm.getSegmentStart(), segm.getSegmentEnd());
}

static float estimatedDistance(RoutingContext* ctx, int targetEndX, int targetEndY, int startX, int startY) {
	double distance = squareRootDist31(startX, startY, targetEndX, targetEndY);
	return (float)(distance / ctx->config->router->getMaxSpeed());
}

static double h(RoutingContext* ctx, int begX, int begY, int endX, int endY) {
	double distToFinalPoint = squareRootDist31(begX, begY, endX, endY);
	double result = distToFinalPoint / ctx->config->router->getMaxSpeed();
	if (ctx->precalcRoute != nullptr) {
		float te = ctx->precalcRoute->timeEstimate(begX, begY, endX, endY);
		if (te > 0) return te;
	}
	return result;
}

struct SegmentsComparator : public std::binary_function<SHARED_PTR<RouteSegment>&, SHARED_PTR<RouteSegment>&, bool> {
	RoutingContext* ctx;
	SegmentsComparator(RoutingContext* c) : ctx(c) {}
	bool operator()(const SHARED_PTR<RouteSegment>& lhs, const SHARED_PTR<RouteSegment>& rhs) const {
		int cmp =
			roadPriorityComparator(lhs.get()->distanceFromStart, lhs.get()->distanceToEnd, rhs.get()->distanceFromStart,
								   rhs.get()->distanceToEnd, ctx->getHeuristicCoefficient());
		return cmp > 0;
	}
};
struct NonHeuristicSegmentsComparator
	: public std::binary_function<SHARED_PTR<RouteSegment>&, SHARED_PTR<RouteSegment>&, bool> {
	bool operator()(const SHARED_PTR<RouteSegment>& lhs, const SHARED_PTR<RouteSegment>& rhs) const {
		return roadPriorityComparator(lhs.get()->distanceFromStart, lhs.get()->distanceToEnd,
									  rhs.get()->distanceFromStart, rhs.get()->distanceToEnd, 0.5) > 0;
	}
};

typedef UNORDERED(map)<int64_t, SHARED_PTR<RouteSegment>> VISITED_MAP;
typedef priority_queue<SHARED_PTR<RouteSegment>, vector<SHARED_PTR<RouteSegment>>, SegmentsComparator> SEGMENTS_QUEUE;
void processRouteSegment(RoutingContext* ctx, bool reverseWaySearch, SEGMENTS_QUEUE& graphSegments,
						 VISITED_MAP& visitedSegments, SHARED_PTR<RouteSegment>& segment, VISITED_MAP& oppositeSegments,
						 bool direction);

SHARED_PTR<RouteSegment> processIntersections(RoutingContext* ctx, SEGMENTS_QUEUE& graphSegments,
											  VISITED_MAP& visitedSegments, SHARED_PTR<RouteSegment>& currentSegment,
											  bool reverseWaySearch, bool doNotAddIntersections);

bool processOneRoadIntersection(RoutingContext* ctx, bool reverseWaySearch, SEGMENTS_QUEUE& graphSegments,
								VISITED_MAP& visitedSegments, SHARED_PTR<RouteSegment>& segment,
								SHARED_PTR<RouteSegment>& next, bool nullGraph = false);

long calculateSizeOfSearchMaps(SEGMENTS_QUEUE& graphDirectSegments, SEGMENTS_QUEUE& graphReverseSegments,
							   VISITED_MAP& visitedDirectSegments, VISITED_MAP& visitedOppositeSegments) {
	long sz = visitedDirectSegments.size() * sizeof(pair<int64_t, SHARED_PTR<RouteSegment>>);
	sz += visitedOppositeSegments.size() * sizeof(pair<int64_t, SHARED_PTR<RouteSegment>>);
	sz += graphDirectSegments.size() * sizeof(SHARED_PTR<RouteSegment>);
	sz += graphReverseSegments.size() * sizeof(SHARED_PTR<RouteSegment>);
	return sz;
}

SHARED_PTR<RouteSegment> loadSameSegment(RoutingContext* ctx, SHARED_PTR<RouteSegment> segment, int ind,
										 bool reverseSearchWay) {
	int x31 = segment->getRoad()->pointsX[ind];
	int y31 = segment->getRoad()->pointsY[ind];

	auto segments = ctx->loadRouteSegment(x31, y31, reverseSearchWay);
	for (auto& seg : segments) {
		if (seg->getRoad()->getId() == segment->getRoad()->getId()) {
			segment = seg;
			break;
		}
	}
	return segment;
}

SHARED_PTR<RouteSegment> initRouteSegment(RoutingContext* ctx, SHARED_PTR<RouteSegment>& segment,
										  bool positiveDirection, bool reverseSearchWay) {
	if (segment->getSegmentStart() == 0 && !positiveDirection && segment->getRoad()->getPointsLength() > 0) {
		segment = loadSameSegment(ctx, segment, 1, reverseSearchWay);
		// } else if(segment->getSegmentStart() == segment->getRoad()->getPointsLength() -1 && positiveDirection &&
		// segment->getSegmentStart() > 0) { assymetric cause we calculate initial point differently (segmentStart means
		// that point is between ]segmentStart-1, segmentStart]
	} else if (segment->getSegmentStart() > 0 && positiveDirection) {
		segment = loadSameSegment(ctx, segment, segment->getSegmentStart() - 1, reverseSearchWay);
	}
	if (!segment) {
		return segment;
	}
	return RouteSegment::initRouteSegment(segment, positiveDirection);
}

SHARED_PTR<RouteSegment> createNull() { return std::make_shared<RouteSegment>(nullptr, 0, 1); }

void initQueuesWithStartEnd(RoutingContext* ctx, SHARED_PTR<RouteSegment> start, SHARED_PTR<RouteSegment> end,
							SEGMENTS_QUEUE& graphDirectSegments, SEGMENTS_QUEUE& graphReverseSegments) {
	SHARED_PTR<RouteSegment> startPos = initRouteSegment(ctx, start, true, false);
	SHARED_PTR<RouteSegment> startNeg = initRouteSegment(ctx, start, false, false);
	SHARED_PTR<RouteSegment> endPos = initRouteSegment(ctx, end, true, true);
	SHARED_PTR<RouteSegment> endNeg = initRouteSegment(ctx, end, false, true);
	if (TRACE_ROUTING) {
		printRoad("START+", startPos);
		printRoad("START-", startNeg);
		printRoad("END+", endPos);
		printRoad("END-", endNeg);
	}
    if (startPos) startPos->parentRoute = createNull();
    if (startNeg) startNeg->parentRoute = createNull();
    if (endPos) endPos->parentRoute = createNull();
    if (endNeg) endNeg->parentRoute = createNull();

	// for start : f(start) = g(start) + h(start) = 0 + h(start) = h(start)
	if (ctx->config->initialDirection > -180 && ctx->config->initialDirection < 180) {
		double plusDir = start->getRoad()->directionRoute(start->getSegmentStart(), true);
		double diff = plusDir - ctx->config->initialDirection;
		if (abs(alignAngleDifference(diff)) <= M_PI / 3) {
			if (startNeg.get() != NULL) {
				startNeg->distanceFromStart += 500;
			}
		} else if (abs(alignAngleDifference(diff - M_PI)) <= M_PI / 3) {
			if (startPos.get() != NULL) {
				startPos->distanceFromStart += 500;
			}
		}
	}

	float estimatedDist = estimatedDistance(ctx, ctx->targetX, ctx->targetY, ctx->startX, ctx->startY);
	if (startPos.get() != NULL && checkMovementAllowed(ctx, false, startPos)) {
		startPos->distanceToEnd = estimatedDist;
		graphDirectSegments.push(startPos);
	}
	if (startNeg.get() != NULL && checkMovementAllowed(ctx, false, startNeg)) {
		startNeg->distanceToEnd = estimatedDist;
		graphDirectSegments.push(startNeg);
	}
	if (endPos.get() != NULL && checkMovementAllowed(ctx, true, endPos)) {
		endPos->distanceToEnd = estimatedDist;
		graphReverseSegments.push(endPos);
	}
	if (endNeg.get() != NULL && checkMovementAllowed(ctx, true, endNeg)) {
		endNeg->distanceToEnd = estimatedDist;
		graphReverseSegments.push(endNeg);
	}
}

bool checkIfGraphIsEmpty(RoutingContext* ctx, bool allowDirection, bool reverseWaySearch, SEGMENTS_QUEUE& graphSegments,
						 SHARED_PTR<RouteSegmentPoint>& pnt, VISITED_MAP& visited, string msg) {
	if (!allowDirection) {
		return false;
	}
	if (allowDirection && graphSegments.empty()) {
		if (pnt->others.size() > 0) {
			vector<SHARED_PTR<RouteSegmentPoint>>::iterator pntIterator = pnt->others.begin();
			while (pntIterator != pnt->others.end()) {
				SHARED_PTR<RouteSegment> next = *pntIterator;
				pntIterator = pnt->others.erase(pntIterator);
				float estimatedDist = estimatedDistance(ctx, ctx->targetX, ctx->targetY, ctx->startX, ctx->startY);
				SHARED_PTR<RouteSegment> pos = RouteSegment::initRouteSegment(next, true);
				if (pos && !containsKey(visited, calculateRoutePointId(pos)) &&
					checkMovementAllowed(ctx, reverseWaySearch, pos)) {
					pos->parentRoute.reset();
					pos->distanceFromStart = 0;
					pos->distanceToEnd = estimatedDist;
					graphSegments.push(pos);
				}
				SHARED_PTR<RouteSegment> neg = RouteSegment::initRouteSegment(next, false);
				if (neg && !containsKey(visited, calculateRoutePointId(neg)) &&
					checkMovementAllowed(ctx, reverseWaySearch, neg)) {
					neg->parentRoute.reset();
					neg->distanceFromStart = 0;
					neg->distanceToEnd = estimatedDist;
					graphSegments.push(neg);
				}

				if (!graphSegments.empty()) {
					OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Debug, "Reiterate point with new start/destination ");
					printRoad("Reiterate point ", next);
					break;
				}
			}
			pnt->others.shrink_to_fit();
			if (graphSegments.empty()) {
				OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Warning, "Route is not found to selected target point.");
			}
		}
	}
	return graphSegments.empty();
}

bool containsKey(VISITED_MAP& visited, int64_t routePointId) { return visited.find(routePointId) != visited.end(); }

/**
 * Calculate route between start.segmentEnd and end.segmentStart (using A* algorithm)
 * return list of segments
 */
SHARED_PTR<RouteSegment> searchRouteInternal(RoutingContext* ctx, SHARED_PTR<RouteSegmentPoint>& start,
											 SHARED_PTR<RouteSegmentPoint>& end, bool leftSideNavigation) {
	// FIXME intermediate points
	// measure time

	int iterationsToUpdate = 0;
	if (ctx->progress.get()) {
		ctx->progress->timeToCalculate.Start();
	}

	SegmentsComparator sgmCmp(ctx);
	NonHeuristicSegmentsComparator nonHeuristicSegmentsComparator;
	SEGMENTS_QUEUE graphDirectSegments(sgmCmp);
	SEGMENTS_QUEUE graphReverseSegments(sgmCmp);

	// Set to not visit one segment twice (stores road.id << X + segmentStart)
	VISITED_MAP visitedDirectSegments;
	VISITED_MAP visitedOppositeSegments;

	initQueuesWithStartEnd(ctx, start, end, graphDirectSegments, graphReverseSegments);

	// Extract & analyze segment with min(f(x)) from queue while final segment is not found
	bool forwardSearch = true;

	SEGMENTS_QUEUE* graphSegments = &graphDirectSegments;
	bool onlyBackward = ctx->getPlanRoadDirection() < 0;
	bool onlyForward = ctx->getPlanRoadDirection() > 0;

	SHARED_PTR<RouteSegment> finalSegment;
	while (graphSegments->size() > 0) {
		SHARED_PTR<RouteSegment> segment = graphSegments->top();
		graphSegments->pop();
		// Memory management
		// ctx.memoryOverhead = calculateSizeOfSearchMaps(graphDirectSegments, graphReverseSegments,
		// visitedDirectSegments, visitedOppositeSegments);
		if (TRACE_ROUTING) {
			printRoad(forwardSearch ? "F>" : "B>", segment);
		}
		if (segment->isFinalSegment) {
			finalSegment = segment;
			ctx->finalRouteSegment = segment;
			if (TRACE_ROUTING) {
				OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Debug, "Final segment found");
			}
			break;
		}
		if (ctx->progress.get()) {
			ctx->progress->visitedSegments++;
		}
		if (forwardSearch) {
			bool doNotAddIntersections = onlyBackward;
			processRouteSegment(ctx, false, graphDirectSegments, visitedDirectSegments, segment,
								visitedOppositeSegments, doNotAddIntersections);
		} else {
			bool doNotAddIntersections = onlyForward;
			processRouteSegment(ctx, true, graphReverseSegments, visitedOppositeSegments, segment,
								visitedDirectSegments, doNotAddIntersections);
		}
		if (ctx->progress.get() && iterationsToUpdate-- < 0) {
			iterationsToUpdate = 100;
			ctx->progress->updateStatus(
				graphDirectSegments.empty() ? 0 : graphDirectSegments.top()->distanceFromStart,
				graphDirectSegments.size(),
				graphReverseSegments.empty() ? 0 : graphReverseSegments.top()->distanceFromStart,
				graphReverseSegments.size());
			if (ctx->progress->isCancelled()) {
				break;
			}
		}
		if (checkIfGraphIsEmpty(ctx, ctx->getPlanRoadDirection() <= 0, true, graphReverseSegments, end,
								visitedOppositeSegments, "Route is not found to selected target point.")) {
			return finalSegment;
		}
		if (checkIfGraphIsEmpty(ctx, ctx->getPlanRoadDirection() >= 0, false, graphDirectSegments, start,
								visitedDirectSegments, "Route is not found from selected start point.")) {
			return finalSegment;
		}
		if (ctx->planRouteIn2Directions()) {
			if (graphDirectSegments.empty() || graphReverseSegments.empty()) {
				// can't proceed - so no route
				break;
			} else {
				forwardSearch = !nonHeuristicSegmentsComparator(graphDirectSegments.top(), graphReverseSegments.top());
			}
			// if (graphDirectSegments.size() * 2 > graphReverseSegments.size()) {
			//	forwardSearch = false;
			//} else if (graphDirectSegments.size() < 2 * graphReverseSegments.size()) {
			//	forwardSearch = true;
			//}
		} else {
			// different strategy : use onedirectional graph
			forwardSearch = onlyForward;
			if (onlyBackward && graphDirectSegments.size() > 0) {
				forwardSearch = true;
			}
			if (onlyForward && graphReverseSegments.size() > 0) {
				forwardSearch = false;
			}
		}
		if (forwardSearch) {
			graphSegments = &graphDirectSegments;
		} else {
			graphSegments = &graphReverseSegments;
		}

		// check if interrupted
		if (ctx->isInterrupted()) {
			return finalSegment;
		}
	}
	if (ctx->progress.get()) {
		ctx->progress->timeToCalculate.Pause();
		ctx->progress->visitedDirectSegments += visitedDirectSegments.size();
		ctx->progress->visitedOppositeSegments += visitedOppositeSegments.size();
		ctx->progress->directQueueSize += graphDirectSegments.size();
		ctx->progress->oppositeQueueSize += graphReverseSegments.size();
	}
	return finalSegment;
}

bool checkMovementAllowed(RoutingContext* ctx, bool reverseWaySearch, SHARED_PTR<RouteSegment>& segment) {
	bool directionAllowed;
	int oneway = ctx->config->router->isOneWay(segment->getRoad());
	// use positive direction as agreed
	if (!reverseWaySearch) {
		if (segment->isPositive()) {
			directionAllowed = oneway >= 0;
		} else {
			directionAllowed = oneway <= 0;
		}
	} else {
		if (segment->isPositive()) {
			directionAllowed = oneway <= 0;
		} else {
			directionAllowed = oneway >= 0;
		}
	}
	return directionAllowed;
}

float calculateTimeWithObstacles(RoutingContext* ctx, SHARED_PTR<RouteDataObject>& road, float distOnRoadToPass,
								 float obstaclesTime) {
	// ctx->timeExtra.Start();
	float priority = ctx->config->router->defineSpeedPriority(road);
	float speed = ctx->config->router->defineRoutingSpeed(road) * priority;
	if (speed == 0) {
		speed = ctx->config->router->getDefaultSpeed() * priority;
	}
	// speed can not exceed max default speed according to A*
	if (speed > ctx->config->router->getMaxSpeed()) {
		speed = ctx->config->router->getMaxSpeed();
	}
	// ctx->timeExtra.Pause();
	return obstaclesTime + distOnRoadToPass / speed;
}

bool checkViaRestrictions(SHARED_PTR<RouteSegment>& from, SHARED_PTR<RouteSegment>& to) {
	if (from && to) {
		int64_t fid = to->getRoad()->getId();
		for (uint i = 0; i < from->getRoad()->restrictions.size(); i++) {
			int64_t id = from->getRoad()->restrictions[i].to;
			int tp = from->getRoad()->restrictions[i].type;
			if (fid == id) {
				if (tp == RESTRICTION_NO_LEFT_TURN || tp == RESTRICTION_NO_RIGHT_TURN ||
					tp == RESTRICTION_NO_STRAIGHT_ON || tp == RESTRICTION_NO_U_TURN) {
					return false;
				}
				break;
			}
			if (tp == RESTRICTION_ONLY_STRAIGHT_ON) {
				return false;
			}
		}
	}
	return true;
}

SHARED_PTR<RouteSegment> getParentDiffId(SHARED_PTR<RouteSegment> s) {
	auto res = s;
	while (res->getParentRoute() && res->getParentRoute()->getRoad() &&
		   res->getParentRoute()->getRoad()->id == res->getRoad()->id) {
		res = res->getParentRoute();
	}
	return res->getParentRoute();
}

bool checkIfOppositeSegmentWasVisited(bool reverseWaySearch, SEGMENTS_QUEUE& graphSegments,
									  SHARED_PTR<RouteSegment>& currentSegment, VISITED_MAP& oppositeSegments) {
	// check inverse direction for opposite
	int64_t currPoint = calculateRoutePointInternalId(currentSegment->getRoad(), currentSegment->getSegmentEnd(),
													  currentSegment->getSegmentStart());
	const auto opIt = oppositeSegments.find(currPoint);
	if (opIt != oppositeSegments.end() && opIt->second) {
		SHARED_PTR<RouteSegment> opposite = opIt->second;
		SHARED_PTR<RouteSegment> curParent = getParentDiffId(currentSegment);
		SHARED_PTR<RouteSegment> oppParent = getParentDiffId(opposite);
		SHARED_PTR<RouteSegment> to = reverseWaySearch ? curParent : oppParent;
		SHARED_PTR<RouteSegment> from = !reverseWaySearch ? curParent : oppParent;
		if (checkViaRestrictions(from, to)) {
			SHARED_PTR<RouteSegment> frs = std::make_shared<RouteSegment>(
				currentSegment->getRoad(), currentSegment->getSegmentStart(), currentSegment->getSegmentEnd());
			frs->parentRoute = currentSegment->getParentRoute();
			frs->reverseWaySearch = reverseWaySearch;
			frs->distanceFromStart = opposite->distanceFromStart + currentSegment->distanceFromStart;
			frs->distanceToEnd = 0;
			frs->opposite = opposite;
			frs->isFinalSegment = true;
			graphSegments.push(frs);
			if (TRACE_ROUTING) {
				printRoad("  >> Final segment : ", frs);
			}
			return true;
		}
	}
	return false;
}

double calculateRouteSegmentTime(RoutingContext* ctx, bool reverseWaySearch, SHARED_PTR<RouteSegment> segment) {
	SHARED_PTR<RouteDataObject> road = segment->getRoad();
	// store <segment> in order to not have unique <segment, direction> in visitedSegments
	short segmentInd = !reverseWaySearch ? segment->getSegmentStart() : segment->getSegmentEnd();
	short prevSegmentInd = reverseWaySearch ? segment->getSegmentStart() : segment->getSegmentEnd();
	const int x = road->pointsX[segmentInd];
	const int y = road->pointsY[segmentInd];
	const int prevX = road->pointsX[prevSegmentInd];
	const int prevY = road->pointsY[prevSegmentInd];

	// calculate point and try to load neighbor ways if they are not loaded
	double distOnRoadToPass = squareRootDist31(x, y, prevX, prevY);
	float priority = ctx->config->router->defineSpeedPriority(road);
	float speed = (ctx->config->router->defineRoutingSpeed(road) * priority);
	if (speed == 0) {
		speed = (ctx->config->router->getDefaultSpeed() * priority);
	}
	// speed can not exceed max default speed according to A*
	if (speed > ctx->config->router->getMaxSpeed()) {
		speed = ctx->config->router->getMaxSpeed();
	}

	// calculate possible obstacle plus time
	double obstacle = ctx->config->router->defineRoutingObstacle(road, segmentInd, prevSegmentInd > segmentInd);
	if (obstacle < 0) {
		return -1;
	}
	double heightObstacle = ctx->config->router->defineHeightObstacle(road, segmentInd, prevSegmentInd);
	if (heightObstacle < 0) {
		return -1;
	}
	return obstacle + heightObstacle + distOnRoadToPass / speed;
}

void processRouteSegment(RoutingContext* ctx, bool reverseWaySearch, SEGMENTS_QUEUE& graphSegments,
						 VISITED_MAP& visitedSegments, SHARED_PTR<RouteSegment>& startSegment,
						 VISITED_MAP& oppositeSegments, bool doNotAddIntersections) {
	SHARED_PTR<RouteDataObject> road = startSegment->getRoad();
	//	bool directionAllowed = true;
	// Go through all point of the way and find ways to continue
	// ! Actually there is small bug when there is restriction to move forward on the way (it doesn't take into account)
	// +/- diff from middle point
	SHARED_PTR<RouteSegment> nextCurrentSegment = startSegment;
	SHARED_PTR<RouteSegment> currentSegment;
	while (nextCurrentSegment) {
		currentSegment = nextCurrentSegment;
		nextCurrentSegment.reset();

		// 1. calculate obstacle for passing this segment
		float segmentAndObstaclesTime = (float)calculateRouteSegmentTime(ctx, reverseWaySearch, currentSegment);
		if (segmentAndObstaclesTime < 0) {
			// directionAllowed = false;
			break;
		}
		// calculate new start segment time as we're going to assign to put to visited segments
		float distFromStartPlusSegmentTime = currentSegment->distanceFromStart + segmentAndObstaclesTime;
		// 2. check if segment was already visited in opposite direction
		// We check before we calculate segmentTime (to not calculate it twice with opposite and calculate turns
		// onto each segment).
		bool alreadyVisited =
			checkIfOppositeSegmentWasVisited(reverseWaySearch, graphSegments, currentSegment, oppositeSegments);
		if (alreadyVisited) {
			// we don't stop here in order to allow improve found *potential* final segment - test case on short route
			// directionAllowed = false;
			// break;
		}

		// 3. upload segment itself to visited segments
		int64_t nextPntId = calculateRoutePointId(currentSegment);
		SHARED_PTR<RouteSegment> existingSegment;
		const auto existingSegIt = visitedSegments.find(nextPntId);
		if (existingSegIt != visitedSegments.end()) existingSegment = existingSegIt->second;
		visitedSegments[nextPntId] = currentSegment;
		if (existingSegment) {
			if (distFromStartPlusSegmentTime > existingSegment->distanceFromStart) {
				// insert back original segment (test case with large area way)
				visitedSegments[nextPntId] = existingSegment;
				// directionAllowed = false;
				if (TRACE_ROUTING) {
					OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "  >> Already visited");
				}
				break;
			} else {
				if (ctx->getHeuristicCoefficient() <= 1) {
					if (PRINT_ROUTING_ALERTS) {
						OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Debug,
										  "! ALERT slower segment was visited earlier %f > %f :",
										  distFromStartPlusSegmentTime, existingSegment->distanceFromStart);
						printRoad("CurrentSegment ", currentSegment);
						printRoad("ExistingSegment ", existingSegment);
					} else {
						ctx->alertSlowerSegmentedWasVisitedEarlier++;
					}
				}
			}
		} else {
			visitedSegments[nextPntId] = currentSegment;
		}

		// reassign @distanceFromStart to make it correct for visited segment
		currentSegment->distanceFromStart = distFromStartPlusSegmentTime;

		// 4. load road connections at the end of segment
		nextCurrentSegment = processIntersections(ctx, graphSegments, visitedSegments, currentSegment, reverseWaySearch,
												  doNotAddIntersections);
	}
}

void clearSegments(vector<SHARED_PTR<RouteSegment>>& segments) {
	segments.clear();
	segments.shrink_to_fit();
}

void processRestriction(RoutingContext* ctx, std::vector<SHARED_PTR<RouteSegment>>& inputNext, bool reverseWay,
						int64_t viaId, SHARED_PTR<RouteDataObject>& road) {
	bool via = viaId != 0;
	std::vector<SHARED_PTR<RouteSegment>> segments = inputNext;
	bool exclusiveRestriction = false;

	for (auto& segment : segments) {
		int type = -1;
		if (!reverseWay) {
			for (uint i = 0; i < road->restrictions.size(); i++) {
				if (road->restrictions[i].to == segment->getRoad()->id) {
					if (!via || road->restrictions[i].via == viaId) {
						type = road->restrictions[i].type;
						;
						break;
					}
				}
				if (road->restrictions[i].via == viaId && via &&
					road->restrictions[i].type == RESTRICTION_ONLY_STRAIGHT_ON) {
					type = RESTRICTION_NO_STRAIGHT_ON;
					break;
				}
			}
		} else {
			for (uint i = 0; i < segment->getRoad()->restrictions.size(); i++) {
				int rt = segment->getRoad()->restrictions[i].type;
				int64_t restrictedTo = segment->getRoad()->restrictions[i].to;
				if (restrictedTo == road->id) {
					if (!via || segment->getRoad()->restrictions[i].via == viaId) {
						type = rt;
						break;
					}
				}
				if (segment->getRoad()->restrictions[i].via == viaId && via && rt == RESTRICTION_ONLY_STRAIGHT_ON) {
					type = RESTRICTION_NO_STRAIGHT_ON;
					break;
				}

				// Check if there is restriction only to the other than current road
				if (rt == RESTRICTION_ONLY_RIGHT_TURN || rt == RESTRICTION_ONLY_LEFT_TURN ||
					rt == RESTRICTION_ONLY_STRAIGHT_ON) {
					// check if that restriction applies to considered junk
					std::vector<SHARED_PTR<RouteSegment>> segments = inputNext;
					bool isFound = false;
					for (auto& segment : segments) {
						if (segment->getRoad()->id == restrictedTo) {
							isFound = true;
							break;
						}
					}
					if (isFound) {
						type = REVERSE_WAY_RESTRICTION_ONLY;  // special constant
					}
				}
			}
		}
		if (type == REVERSE_WAY_RESTRICTION_ONLY) {
			// next = next.next; continue;
		} else if (type == -1 && exclusiveRestriction) {
			// next = next.next; continue;
		} else if (type == RESTRICTION_NO_LEFT_TURN || type == RESTRICTION_NO_RIGHT_TURN ||
				   type == RESTRICTION_NO_STRAIGHT_ON || type == RESTRICTION_NO_U_TURN) {
			// next = next.next; continue;
			if (via) {
				auto it = find(ctx->segmentsToVisitPrescripted.begin(), ctx->segmentsToVisitPrescripted.end(), segment);
				if (it != ctx->segmentsToVisitPrescripted.end()) {
					ctx->segmentsToVisitPrescripted.erase(it);
				}
			}
		} else if (type == -1) {
			// case no restriction
			ctx->segmentsToVisitNotForbidden.push_back(segment);
		} else {
			if (!via) {
				// case exclusive restriction (only_right, only_straight, ...)
				// 1. in case we are going backward we should not consider only_restriction
				// as exclusive because we have many "in" roads and one "out"
				// 2. in case we are going forward we have one "in" and many "out"
				if (!reverseWay) {
					exclusiveRestriction = true;
					clearSegments(ctx->segmentsToVisitNotForbidden);
					ctx->segmentsToVisitPrescripted.push_back(segment);
				} else {
					ctx->segmentsToVisitNotForbidden.push_back(segment);
				}
			}
		}
	}

	if (!via) {
		ctx->segmentsToVisitPrescripted.insert(ctx->segmentsToVisitPrescripted.end(),
											   ctx->segmentsToVisitNotForbidden.begin(),
											   ctx->segmentsToVisitNotForbidden.end());
	}
	ctx->segmentsToVisitPrescripted.shrink_to_fit();
}

bool proccessRestrictions(RoutingContext* ctx, SHARED_PTR<RouteSegment>& segment,
						  std::vector<SHARED_PTR<RouteSegment>>& inputNext, bool reverseWay) {
	if (!ctx->config->router->restrictionsAware()) {
		return false;
	}
	SHARED_PTR<RouteDataObject> road = segment->getRoad();
	SHARED_PTR<RouteSegment> parent = getParentDiffId(segment);

	if (!reverseWay && road->restrictions.size() == 0 && (!parent || parent->getRoad()->restrictions.size() == 0)) {
		return false;
	}
	clearSegments(ctx->segmentsToVisitPrescripted);
	clearSegments(ctx->segmentsToVisitNotForbidden);
	processRestriction(ctx, inputNext, reverseWay, 0, road);
	if (parent) {
		processRestriction(ctx, inputNext, reverseWay, segment->getRoad()->id, parent->road);
	}
	return true;
}

SHARED_PTR<RouteSegment> processIntersections(RoutingContext* ctx, SEGMENTS_QUEUE& graphSegments,
											  VISITED_MAP& visitedSegments, SHARED_PTR<RouteSegment>& currentSegment,
											  bool reverseWaySearch, bool doNotAddIntersections) {
	SHARED_PTR<RouteSegment> nextCurrentSegment;
	int targetEndX = reverseWaySearch ? ctx->startX : ctx->targetX;
	int targetEndY = reverseWaySearch ? ctx->startY : ctx->targetY;
	const int x = currentSegment->getRoad()->pointsX[currentSegment->getSegmentEnd()];
	const int y = currentSegment->getRoad()->pointsY[currentSegment->getSegmentEnd()];
	float distanceToEnd = h(ctx, x, y, targetEndX, targetEndY);
	// reassign @distanceToEnd to make it correct for visited segment
	currentSegment->distanceToEnd = distanceToEnd;
	auto connectedNextSegments = ctx->loadRouteSegment(x, y, reverseWaySearch);
	bool directionAllowed = true;
	bool singleRoad = true;
	for (auto& roadIter : connectedNextSegments) {
		if (currentSegment->getSegmentEnd() == roadIter->getSegmentStart() &&
			roadIter->getRoad()->getId() == currentSegment->getRoad()->getId()) {
			nextCurrentSegment = RouteSegment::initRouteSegment(roadIter, currentSegment->isPositive());
			if (!nextCurrentSegment) {
				directionAllowed = false;
			} else {
				if (nextCurrentSegment->isSegmentAttachedToStart()) {
					directionAllowed = processOneRoadIntersection(ctx, reverseWaySearch, graphSegments, visitedSegments,
																  currentSegment, nextCurrentSegment, true);
				} else {
					nextCurrentSegment->parentRoute = currentSegment;
					nextCurrentSegment->distanceFromStart = currentSegment->distanceFromStart;
					nextCurrentSegment->distanceToEnd = distanceToEnd;
					int nx = nextCurrentSegment->getRoad()->pointsX[nextCurrentSegment->getSegmentEnd()];
					int ny = nextCurrentSegment->getRoad()->pointsY[nextCurrentSegment->getSegmentEnd()];
					if (nx == x && ny == y) {
						// don't process other intersections (let process further segment)
						return nextCurrentSegment;
					}
				}
			}
		} else {
			singleRoad = false;
		}
	}

	if (singleRoad) {
		return nextCurrentSegment;
	}

	// find restrictions and iterator
	auto nextIterator = ctx->segmentsToVisitPrescripted.end();
	bool thereAreRestrictions = proccessRestrictions(ctx, currentSegment, connectedNextSegments, reverseWaySearch);
	if (thereAreRestrictions) {
		nextIterator = ctx->segmentsToVisitPrescripted.begin();
		if (TRACE_ROUTING) {
			OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Debug, "  >> There are restrictions");
		}
	}

	// Calculate possible turns to put into priority queue
	std::vector<SHARED_PTR<RouteSegment>> nextSegments = connectedNextSegments;
	if (!nextSegments.empty()) {
		bool hasNext = thereAreRestrictions ? nextIterator != ctx->segmentsToVisitPrescripted.end()
											: nextSegments.front() != nullptr;
		for (auto& segment : nextSegments) {
			if (segment != nextSegments.front() && !thereAreRestrictions) {
				hasNext = segment != nullptr;
			}
			while (hasNext) {
				if (thereAreRestrictions) {
					segment = *nextIterator;
				}
				if (segment->getSegmentStart() == currentSegment->getSegmentEnd() &&
					segment->getRoad()->getId() == currentSegment->getRoad()->getId()) {
					// skip itself
				} else if (!doNotAddIntersections) {
					SHARED_PTR<RouteSegment> nextPos = RouteSegment::initRouteSegment(segment, true);
					processOneRoadIntersection(ctx, reverseWaySearch, graphSegments, visitedSegments, currentSegment,
											   nextPos);
					SHARED_PTR<RouteSegment> nextNeg = RouteSegment::initRouteSegment(segment, false);
					processOneRoadIntersection(ctx, reverseWaySearch, graphSegments, visitedSegments, currentSegment,
											   nextNeg);
				}
				// iterate to next road
				if (!thereAreRestrictions) {
					break;
				} else {
					nextIterator++;
					hasNext = nextIterator != ctx->segmentsToVisitPrescripted.end();
				}
			}
		}
	}

	if (nextCurrentSegment == nullptr && directionAllowed) {
		if (ctx->calculationMode != RouteCalculationMode::BASE) {
			// std::cout << "!!!!!!!!!!" << thereAreRestrictions << std::endl;
			return nextCurrentSegment;
		} else {
			// TODO Issue #...: we know that bug in data (how we simplify base data and connect between regions), so we
			// workaround it
			int newEnd = currentSegment->getSegmentEnd() + (currentSegment->isPositive() ? +1 : -1);
			if (newEnd >= 0 && newEnd < currentSegment->getRoad()->getPointsLength() - 1) {
				nextCurrentSegment = make_shared<RouteSegment>(currentSegment->getRoad(),
															   (int)currentSegment->getSegmentEnd(), (int)newEnd);
				nextCurrentSegment->parentRoute = currentSegment;
				nextCurrentSegment->distanceFromStart = currentSegment->distanceFromStart;
				nextCurrentSegment->distanceToEnd = distanceToEnd;
			}
		}
	}
	return nextCurrentSegment;
}

bool sortRoutePoints(const SHARED_PTR<RouteSegmentPoint>& i, const SHARED_PTR<RouteSegmentPoint>& j) {
	return (i->dist < j->dist);
}

SHARED_PTR<RouteSegmentPoint> findRouteSegment(int px, int py, RoutingContext* ctx) {
	return findRouteSegment(px, py, ctx, false);
}

SHARED_PTR<RouteSegmentPoint> findRouteSegment(int px, int py, RoutingContext* ctx, bool transportStop, int64_t roadId,
											   int segmentInd) {
	if (ctx->progress) {
		ctx->progress->timeToFindInitialSegments.Start();
	}
	vector<SHARED_PTR<RouteDataObject>> dataObjects;
	ctx->loadTileData(px, py, 17, dataObjects);
	if (dataObjects.size() == 0) {
		ctx->loadTileData(px, py, 15, dataObjects);
	}
	if (dataObjects.size() == 0) {
		ctx->loadTileData(px, py, 14, dataObjects);
	}
	vector<SHARED_PTR<RouteSegmentPoint>> list;
	vector<SHARED_PTR<RouteDataObject>>::iterator it = dataObjects.begin();
	for (; it != dataObjects.end(); it++) {
		SHARED_PTR<RouteDataObject> r = *it;
		if (r->id == roadId && roadId > 0 && segmentInd < r->pointsX.size()) {
			SHARED_PTR<RouteSegmentPoint> road = std::make_shared<RouteSegmentPoint>(r, segmentInd, 0);
			road->preciseX = px;
			road->preciseY = py;
			list.push_back(road);
			break;
		}
	}
	if (list.empty()) {
		it = dataObjects.begin();
	}
	for (; it != dataObjects.end(); it++) {
		SHARED_PTR<RouteDataObject> r = *it;
		if (r->pointsX.size() > 1) {
			SHARED_PTR<RouteSegmentPoint> road;
			for (uint j = 1; j < r->pointsX.size(); j++) {
				std::pair<int, int> p =
					getProjectionPoint(px, py, r->pointsX[j - 1], r->pointsY[j - 1], r->pointsX[j], r->pointsY[j]);
				int prx = p.first;
				int pry = p.second;
				double currentsDist = squareDist31TileMetric(prx, pry, px, py);
				if (!road || currentsDist < road->dist) {
					road = std::make_shared<RouteSegmentPoint>(r, j, currentsDist);
					road->preciseX = prx;
					road->preciseY = pry;
				}
			}
			if (road) {
				if (!transportStop) {
					float prio = max(ctx->config->router->defineSpeedPriority(road->road), 0.3);
					if (prio > 0) {
						road->dist = (road->dist + GPS_POSSIBLE_ERROR * GPS_POSSIBLE_ERROR) / (prio * prio);
						list.push_back(road);
					}
				} else {
					list.push_back(road);
				}
			}
		}
	}
	sort(list.begin(), list.end(), sortRoutePoints);
	if (ctx->progress) {
		ctx->progress->timeToFindInitialSegments.Pause();
	}
	if (list.size() > 0) {
		SHARED_PTR<RouteSegmentPoint> ps;
		int i = 0;
		if (ctx->publicTransport) {
			vector<SHARED_PTR<RouteSegmentPoint>>::iterator it = list.begin();
			for (; it != list.end(); it++) {
				if (transportStop && (*it)->dist > 100) {
					break;
				}
				bool platform = (*it)->road->platform();
				if (transportStop && platform) {
					ps = *it;
					list.erase(it);
					break;
				}
				if (!transportStop && !platform) {
					ps = *it;
					list.erase(it);
					break;
				}
			}
			i++;
		}
		if (ps == nullptr) {
			ps = list[0];
			list.erase(list.begin());
		}
		ps->others = list;
		return ps;
	}
	return nullptr;
}

bool combineTwoSegmentResultPlanner(SHARED_PTR<RouteSegmentResult>& toAdd, SHARED_PTR<RouteSegmentResult>& previous,
									bool reverse) {
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

void addRouteSegmentToResult(vector<SHARED_PTR<RouteSegmentResult>>& result, SHARED_PTR<RouteSegmentResult>& res,
							 bool reverse) {
	if (res->getEndPointIndex() != res->getStartPointIndex()) {
		if (result.size() > 0) {
			auto last = result.back();
			if (last->object->id == res->object->id) {
				if (combineTwoSegmentResultPlanner(res, last, reverse)) {
					return;
				}
			}
		}
		result.push_back(res);
	}
}

void attachConnectedRoads(RoutingContext* ctx, vector<SHARED_PTR<RouteSegmentResult>>& res) {
	for (auto it : res) {
		bool plus = it->getStartPointIndex() < it->getEndPointIndex();
		int j = it->getStartPointIndex();
		do {
			std::vector<SHARED_PTR<RouteSegment>> segments =
				ctx->loadRouteSegment(it->object->pointsX[j], it->object->pointsY[j]);
			vector<SHARED_PTR<RouteSegmentResult>> r;
			for (auto& segment : segments) {
				auto res = std::make_shared<RouteSegmentResult>(segment->road, segment->getSegmentStart(),
																segment->getSegmentStart());
				r.push_back(res);
			}
			it->attachedRoutes.push_back(r);
			j = plus ? j + 1 : j - 1;

		} while (j != it->getEndPointIndex());
	}
}

bool processOneRoadIntersection(RoutingContext* ctx, bool reverseWaySearch, SEGMENTS_QUEUE& graphSegments,
								VISITED_MAP& visitedSegments, SHARED_PTR<RouteSegment>& segment,
								SHARED_PTR<RouteSegment>& next, bool isNullGraph) {
	if (next) {
		if (!checkMovementAllowed(ctx, reverseWaySearch, next)) {
			return false;
		}
		// Can lose 1 during cast double to float
		float obstaclesTime = (float)ctx->config->router->calculateTurnTime(
			next, next->isPositive() ? next->getRoad()->getPointsLength() - 1 : 0, segment, segment->getSegmentEnd());
		if (obstaclesTime < 0) {
			return false;
		}
		float distFromStart = obstaclesTime + segment->distanceFromStart;
		const auto visIt = visitedSegments.find(calculateRoutePointId(next));
		bool toAdd = true;
		if (visIt != visitedSegments.end() && visIt->second) {
			const auto& visitedSeg = visIt->second;
			if (TRACE_ROUTING) {
				printRoad("  >?", visitedSegments.at(calculateRoutePointId(next)));
			}
			toAdd = false;
			// The segment was already visited! We can try to follow new route if it's shorter.
			// That is very exceptional situation and almost exception, it can happen
			// 1. We underestimate distanceToEnd - wrong h() of A* (heuristic > 1)
			// 2. We don't process small segments 1 by 1 from the queue but the whole road,
			//  and it could be that deviation from the road is faster than following the whole road itself!
			if (distFromStart < visitedSeg->distanceFromStart) {
				double routeSegmentTime = calculateRouteSegmentTime(ctx, reverseWaySearch, visitedSeg);
				// we need to properly compare @distanceFromStart VISITED and NON-VISITED segment
				if (distFromStart + routeSegmentTime < visitedSeg->distanceFromStart) {
					// Here it's not very legitimate action cause in theory we need to go up to the final segment in the
					// queue & decrease final time But it's compensated by chain reaction cause next.distanceFromStart <
					// finalSegment.distanceFromStart and revisiting all segments

					// We don't check ```next.getParentRoute() == null``` cause segment could be unloaded
					// so we need to add segment back to the queue & reassign the parent (same as for
					// next.getParentRoute() == null)
					toAdd = true;
					if (ctx->getHeuristicCoefficient() <= 1) {
						if (PRINT_ROUTING_ALERTS) {
							OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Debug, "! ALERT new faster path to a visited segment: %f < %f",
											  (distFromStart + routeSegmentTime), visIt->second.get()->distanceFromStart);
							printRoad("next", next);
							printRoad("visIt", visitedSeg.get());
						} else {
							ctx->alertFasterRoadToVisitedSegments++;
						}
						
					}
					// ??? It's not clear whether this block is needed or not ???
					// All Test cases work with and without it
					// visIt.setParentRoute(segment);
					// visIt.distanceFromStart = (float) (distFromStart + routeSegmentTime);
					// visIt.distanceToEnd = segment.distanceToEnd;
					// ???
				}
			}
		}
		if (toAdd && (!next->isSegmentAttachedToStart() ||
					  roadPriorityComparator(next->distanceFromStart, next->distanceToEnd, distFromStart,
											 segment->distanceToEnd, ctx->getHeuristicCoefficient()) > 0)) {
			next->distanceFromStart = distFromStart;
			next->distanceToEnd = segment->distanceToEnd;
			if (TRACE_ROUTING) {
				printRoad("  >>", next);
			}
			// put additional information to recover whole route after
			next->parentRoute = segment;
			if (!isNullGraph) {
				graphSegments.push(next);
			}
			return true;
		}
	}
	return false;
}

vector<SHARED_PTR<RouteSegmentResult>> convertFinalSegmentToResults(RoutingContext* ctx,
																	SHARED_PTR<RouteSegment>& finalSegment) {
	vector<SHARED_PTR<RouteSegmentResult>> result;
	if (finalSegment) {
		SHARED_PTR<RouteSegment> segment =
			finalSegment->isReverseWaySearch() ? finalSegment->getParentRoute() : finalSegment->opposite;
		while (segment && segment->getRoad()) {
			auto res = std::make_shared<RouteSegmentResult>(segment->road, segment->getSegmentEnd(),
															segment->getSegmentStart());
			float parentRoutingTime =
				segment->getParentRoute() != nullptr ? segment->getParentRoute()->distanceFromStart : 0;
			res->routingTime = segment->distanceFromStart - parentRoutingTime;
			segment = segment->getParentRoute();
			addRouteSegmentToResult(result, res, false);
		}
		// reverse it just to attach good direction roads
		std::reverse(result.begin(), result.end());
		segment = finalSegment->isReverseWaySearch() ? finalSegment->opposite : finalSegment->getParentRoute();
		while (segment && segment->getRoad()) {
			auto res = std::make_shared<RouteSegmentResult>(segment->road, segment->getSegmentStart(),
															segment->getSegmentEnd());
			float parentRoutingTime =
				segment->getParentRoute() != nullptr ? segment->getParentRoute()->distanceFromStart : 0;
			res->routingTime = segment->distanceFromStart - parentRoutingTime;
			segment = segment->getParentRoute();
			// happens in smart recalculation
			addRouteSegmentToResult(result, res, true);
		}
		std::reverse(result.begin(), result.end());
	}
	return result;
}

vector<SHARED_PTR<RouteSegmentResult>> searchRouteInternal(RoutingContext* ctx, bool leftSideNavigation) {
	SHARED_PTR<RouteSegmentPoint> start =
		findRouteSegment(ctx->startX, ctx->startY, ctx, ctx->publicTransport && ctx->startTransportStop,
						 ctx->startRoadId, ctx->startSegmentInd);
	if (start.get() == NULL) {
		OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Warning, "Start point was not found [Native]");
		if (ctx->progress.get()) {
			ctx->progress->setSegmentNotFound(0);
		}
		return vector<SHARED_PTR<RouteSegmentResult>>();
	} else {
		// OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "Start point was found %lld [Native]", start->road->id /
		// 64);
	}
	SHARED_PTR<RouteSegmentPoint> end =
		findRouteSegment(ctx->targetX, ctx->targetY, ctx, ctx->publicTransport && ctx->targetTransportStop,
						 ctx->targetRoadId, ctx->targetSegmentInd);
	if (end.get() == NULL) {
		if (ctx->progress.get()) {
			ctx->progress->setSegmentNotFound(1);
		}
		OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Warning, "End point was not found [Native]");
		return vector<SHARED_PTR<RouteSegmentResult>>();
	} else {
		// OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "End point was found %lld [Native]", end->road->id / 64);
	}
	SHARED_PTR<RouteSegment> finalSegment = searchRouteInternal(ctx, start, end, leftSideNavigation);
	vector<SHARED_PTR<RouteSegmentResult>> res = convertFinalSegmentToResults(ctx, finalSegment);
	attachConnectedRoads(ctx, res);
	return res;
}
