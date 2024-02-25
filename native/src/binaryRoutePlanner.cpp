#include "binaryRoutePlanner.h"

#include <functional>
#include <iterator>
#include <queue>

#include "Logging.h"
#include "binaryRead.h"
#include "routingContext.h"

//	static bool PRINT_TO_CONSOLE_ROUTE_INFORMATION_TO_TEST = true;

static const bool TRACE_ROUTING = false;
static const bool ASSERT_CHECKS = true;
static const bool PRINT_ROUTING_ALERTS = false;
static const bool DEBUG_BREAK_EACH_SEGMENT = false;
static const bool DEBUG_PRECISE_DIST_MEASUREMENT = false;

// Check issue #8649
static const double GPS_POSSIBLE_ERROR = 7;

static double squareRootDist(int x1, int y1, int x2, int y2) {
		if (DEBUG_PRECISE_DIST_MEASUREMENT) {
			return measuredDist31(x1, y1, x2, y2);
		}
		return squareRootDist31(x1, y1, x2, y2);
}

void printRoad(const char* prefix, RouteSegment* segment) {
    if (!segment) {
        return;
    }
	const auto& parentRoute = segment->getParentRoute();
    string name = segment->getRoad()->getName();
    if (!name.empty()) {
        name = ", name ('" + name +"')";
    }
    string parent = "";
    if (parentRoute != nullptr && parentRoute->road != nullptr) {
        parent = "parent=Road (" + to_string(parentRoute->getRoad()->id / 64) + ")";
        string lang = "";
        string parentName = parentRoute->getRoad()->getName(lang, false);
        if (!parentName.empty()) {
            parent = parent + ", name ('" + parentName + "')";
        }
    }
	OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Debug, "%s Road (%lld) %s ind=%d->%d ds=%f es=%f pend=%d %s",
					  prefix, segment->getRoad()->id / 64, name.c_str(), segment->getSegmentStart(), segment->getSegmentEnd(),
					  segment->distanceFromStart, segment->distanceToEnd,
					  parentRoute != nullptr ? parentRoute->segmentEnd : 0,
					  parent.c_str());
}

void printRoad(const char* prefix, const SHARED_PTR<RouteSegment>& segment) { printRoad(prefix, segment.get()); }

int64_t calculateRoutePointInternalId(const SHARED_PTR<RouteDataObject>& road, int pntId, int nextPntId) {
	int positive = nextPntId - pntId;
	int pntLen = road->getPointsLength();
	if (pntId < 0 || nextPntId < 0 || pntId >= pntLen || nextPntId >= pntLen || (positive != -1 && positive != 1)) {
		OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Warning, "Assert failed route point (should never happen)");
	}
	return (road->id << ROUTE_POINTS) + (pntId << 1) + (positive > 0 ? 1 : 0);
}

int64_t calculateRoutePointId(const SHARED_PTR<RouteSegment>& segm) {
	return calculateRoutePointInternalId(
		segm->getRoad(), segm->getSegmentStart(),
		segm->isPositive() ? segm->getSegmentStart() + 1 : segm->getSegmentStart() - 1);
	// return calculateRoutePointInternalId(segm.getRoad(), segm.getSegmentStart(), segm.getSegmentEnd());
}
float cost(float distanceFromStart, float distanceToEnd, RoutingContext* ctx) {
	return ctx->config->heurCoefficient * distanceToEnd + distanceFromStart;
}

static float estimatedDistance(const SHARED_PTR<RouteSegment>& seg, bool rev, const RoutingContext* ctx) {
	int x = seg->getStartPointX() / 2 + seg->getEndPointX() /  2;
	int y = seg->getStartPointY() / 2 + seg->getEndPointY() /  2;
	double distance = squareRootDist(x, y, rev ? ctx->startX : ctx->targetX, rev ? ctx->startY : ctx->targetY);
	return (float)(distance / ctx->config->router->getMaxSpeed());
}

static double h(RoutingContext* ctx, int begX, int begY, int endX, int endY) {
	if (ctx->dijkstraMode != 0) {
		return 0;
	}
	double distToFinalPoint = squareRootDist(begX, begY, endX, endY);
	double result = distToFinalPoint / ctx->config->router->getMaxSpeed();
	if (ctx->precalcRoute != nullptr) {
		float te = ctx->precalcRoute->timeEstimate(begX, begY, endX, endY);
		if (te > 0) return te;
	}
	return result;
}

static int doubleCompare(double d1, double d2) {
	if (d1 < d2)
		return -1;
	if (d1 > d2)
		return 1;
	return 0;
}

struct RouteSegmentCost {
	float segCost;
	SHARED_PTR<RouteSegment> segment;
			
	public:
		RouteSegmentCost(const SHARED_PTR<RouteSegment>& segment, RoutingContext* ctx) {
			segCost = cost(segment->distanceFromStart, segment->distanceToEnd, ctx);
			this->segment = segment;
		}
};

struct SegmentsComparator : public std::function<bool(SHARED_PTR<RouteSegmentCost>&, SHARED_PTR<RouteSegmentCost>&)> {
	bool operator()(const SHARED_PTR<RouteSegmentCost>& o1, const SHARED_PTR<RouteSegmentCost>& o2) {
		int cmp = doubleCompare(o1->segCost, o2->segCost);
		return cmp > 0;
	}
};


typedef UNORDERED(map)<int64_t, SHARED_PTR<RouteSegment>> VISITED_MAP;
typedef priority_queue<SHARED_PTR<RouteSegmentCost>, vector<SHARED_PTR<RouteSegmentCost>>, SegmentsComparator> SEGMENTS_QUEUE;
void processRouteSegment(RoutingContext* ctx, bool reverseWaySearch, SEGMENTS_QUEUE& graphSegments,
						 VISITED_MAP& visitedSegments, const SHARED_PTR<RouteSegment>& segment, const VISITED_MAP& oppositeSegments,
						 const VISITED_MAP & boundaries, bool direction, std::vector<int64_t> excludedKeys);

SHARED_PTR<RouteSegment> processIntersections(RoutingContext* ctx, SEGMENTS_QUEUE& graphSegments,
											  VISITED_MAP& visitedSegments, const SHARED_PTR<RouteSegment>& currentSegment,
											  bool reverseWaySearch, bool doNotAddIntersections);

bool processOneRoadIntersection(RoutingContext* ctx, bool reverseWaySearch, SEGMENTS_QUEUE& graphSegments,
								VISITED_MAP& visitedSegments, const SHARED_PTR<RouteSegment>& segment,
								const SHARED_PTR<RouteSegment>& next, bool nullGraph = false);

long calculateSizeOfSearchMaps(SEGMENTS_QUEUE& graphDirectSegments, SEGMENTS_QUEUE& graphReverseSegments,
							   VISITED_MAP& visitedDirectSegments, VISITED_MAP& visitedOppositeSegments) {
	long sz = visitedDirectSegments.size() * sizeof(pair<int64_t, SHARED_PTR<RouteSegment>>);
	sz += visitedOppositeSegments.size() * sizeof(pair<int64_t, SHARED_PTR<RouteSegment>>);
	sz += graphDirectSegments.size() * sizeof(SHARED_PTR<RouteSegment>);
	sz += graphReverseSegments.size() * sizeof(SHARED_PTR<RouteSegment>);
	return sz;
}

float calcRoutingSegmentTimeOnlyDist(const SHARED_PTR<GeneralRouter>& router, const SHARED_PTR<RouteSegment>& segment) {
	int prevX = segment->road->getPoint31XTile(segment->getSegmentStart());
	int prevY = segment->road->getPoint31YTile(segment->getSegmentStart());
	int x = segment->road->getPoint31XTile(segment->getSegmentEnd());
	int y = segment->road->getPoint31YTile(segment->getSegmentEnd());
	float priority = router->defineSpeedPriority(segment->road, segment->isPositive());
	float speed = (router->defineRoutingSpeed(segment->road, segment->isPositive()) * priority);
	if (speed == 0) {
		speed = router->getDefaultSpeed() * priority;
	}
	// speed can not exceed max default speed according to A*
	if (speed > router->getMaxSpeed()) {
		speed = router->getMaxSpeed();
	}
	float distOnRoadToPass = (float) squareRootDist(prevX, prevY, x, y);
	return distOnRoadToPass / speed;
}

float calculatePreciseStartTime(const RoutingContext* ctx, int projX, int projY, const SHARED_PTR<RouteSegment>& seg) {
	// compensate first segment difference to mid point (length) https://github.com/osmandapp/OsmAnd/issues/14148
	double fullTime = calcRoutingSegmentTimeOnlyDist(ctx->config->router, seg);
	double full = squareRootDist(seg->getStartPointX(), seg->getStartPointY(), seg->getEndPointX(), seg->getEndPointY()) + 0.01; // avoid div 0
	double fromStart = squareRootDist(projX, projY, seg->getStartPointX(), seg->getStartPointY());
	float dist = (float) (fromStart / full * fullTime);
	return dist;
}


SHARED_PTR<RouteSegment> initEdgeSegment(RoutingContext* ctx, SHARED_PTR<RouteSegmentPoint>& pnt, bool originalDir, SEGMENTS_QUEUE& graphSegments, bool reverseSearchWay) {
	if (!pnt) {
		return nullptr;
	}
	// originalDir = true: start points & end points equal to pnt
	auto segments = ctx->loadRouteSegment(originalDir ? pnt->getStartPointX() : pnt->getEndPointX(),
			originalDir ? pnt->getStartPointY() : pnt->getEndPointY(), reverseSearchWay);
	SHARED_PTR<RouteSegment> seg = nullptr;
	for (int i = 0; i < segments.size(); i++) {
		seg = segments[i];
		if (seg->getRoad()->getId() == pnt->getRoad()->getId() &&
				(seg->getSegmentStart() == (originalDir ? pnt->getSegmentStart() : pnt->getSegmentEnd()))) {
			break;
		}
	}
	if (ctx->isInterrupted() || segments.size() == 0 || seg == nullptr) {
		OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Warning, "initEdgeSegment() got empty segments (cancel)");
		return nullptr;
	}
	if (seg->getSegmentStart() != (originalDir ? pnt->getSegmentStart() : pnt->getSegmentEnd())
			|| seg->getSegmentEnd() != (originalDir ? pnt->getSegmentEnd() : pnt->getSegmentStart())) {
		seg = RouteSegment::initRouteSegment(seg, !seg->isPositive());
	}
	if (originalDir && (seg->getSegmentStart() != pnt->getSegmentStart() || seg->getSegmentEnd() != pnt->getSegmentEnd())) {
		//throw new IllegalStateException();
		// TODO
		return nullptr;
	}
	if (!originalDir && (seg->getSegmentStart() != pnt->getSegmentEnd() || seg->getSegmentEnd() != pnt->getSegmentStart())) {
		//throw new IllegalStateException();
		// TODO
		return nullptr;
	}
	if (!originalDir && ctx->config->initialDirection == NO_DIRECTION && ctx->config->penaltyForReverseDirection < 0) {
		// special case for single side spread point-dijkstra
		return nullptr;
	}
	seg->parentRoute = createNull();
	float dist = -calculatePreciseStartTime(ctx, pnt->preciseX, pnt->preciseY, seg);
	// full segment length will be added on first visit
	seg->distanceFromStart = dist;

	if ((!reverseSearchWay && ctx->config->initialDirection != NO_DIRECTION) ||
	        (reverseSearchWay && ctx->config->targetDirection != NO_DIRECTION)) {
		// for start : f(start) = g(start) + h(start) = 0 + h(start) = h(start)
		// mark here as positive for further check
		double plusDir = seg->getRoad()->directionRoute(seg->getSegmentStart(), seg->isPositive());
		double diff = plusDir - (reverseSearchWay ? ctx->config->targetDirection : ctx->config->initialDirection);
		if (abs(alignAngleDifference(diff - M_PI)) <= M_PI / 3) {
			seg->distanceFromStart += ctx->config->penaltyForReverseDirection;
		}
	}
	if (checkMovementAllowed(ctx, reverseSearchWay, seg)) {
		seg->distanceToEnd = estimatedDistance(seg, reverseSearchWay, ctx);
		SHARED_PTR<RouteSegmentCost> rsc = make_shared<RouteSegmentCost>(seg, ctx);
		graphSegments.push(rsc);
		return seg;
	}
	return nullptr;
}

SHARED_PTR<RouteSegment> createNull() { return std::make_shared<RouteSegment>(nullptr, 0, 1); }

void initQueuesWithStartEnd(RoutingContext* ctx, SHARED_PTR<RouteSegmentPoint>& start, SHARED_PTR<RouteSegmentPoint>& end,
							SEGMENTS_QUEUE& graphDirectSegments, SEGMENTS_QUEUE& graphReverseSegments) {
	if (start) {
		if (ctx->precalcRoute && ctx->precalcRoute->startPoint == PrecalculatedRouteDirection::calc(ctx->startX, ctx->startY)) {
			ctx->precalcRoute->startPoint = PrecalculatedRouteDirection::calc(start->preciseX, start->preciseY);
		}
		ctx->startX = start->preciseX;
		ctx->startY = start->preciseY;
	}
	if (end) {
		if (ctx->precalcRoute && ctx->precalcRoute->endPoint == PrecalculatedRouteDirection::calc(ctx->targetX, ctx->targetY)) {
			ctx->precalcRoute->endPoint = PrecalculatedRouteDirection::calc(end->preciseX, end->preciseY);
		}
		ctx->targetX = end->preciseX;
		ctx->targetY = end->preciseY;
	}
	auto startPos = initEdgeSegment(ctx, start, true, graphDirectSegments, false);
	auto startNeg = initEdgeSegment(ctx, start, false, graphDirectSegments, false);
	auto endPos = initEdgeSegment(ctx, end, true, graphReverseSegments, true);
	auto endNeg = initEdgeSegment(ctx, end, false, graphReverseSegments, true);
	if (TRACE_ROUTING) {
		printRoad("Initial segment start positive: ", startPos);
		printRoad("Initial segment start negative: ", startNeg);
		printRoad("Initial segment end positive: ", endPos);
		printRoad("Initial segment end negative: ", endNeg);
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
				float estimatedDist = estimatedDistance(next, reverseWaySearch, ctx);
				SHARED_PTR<RouteSegment> pos = RouteSegment::initRouteSegment(next, true);
				if (pos && !containsKey(visited, calculateRoutePointId(pos)) &&
					checkMovementAllowed(ctx, reverseWaySearch, pos)) {
					pos->parentRoute.reset();
					pos->distanceFromStart = 0;
					pos->distanceToEnd = estimatedDist;
					SHARED_PTR<RouteSegmentCost> rsc = make_shared<RouteSegmentCost>(pos, ctx);
					graphSegments.push(rsc);
				}
				SHARED_PTR<RouteSegment> neg = RouteSegment::initRouteSegment(next, false);
				if (neg && !containsKey(visited, calculateRoutePointId(neg)) &&
					checkMovementAllowed(ctx, reverseWaySearch, neg)) {
					neg->parentRoute.reset();
					neg->distanceFromStart = 0;
					neg->distanceToEnd = estimatedDist;
					SHARED_PTR<RouteSegmentCost> rsc = make_shared<RouteSegmentCost>(neg, ctx);
					graphSegments.push(rsc);
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
vector<SHARED_PTR<RouteSegment>> searchRouteInternal(RoutingContext* ctx, SHARED_PTR<RouteSegmentPoint> start,
											 SHARED_PTR<RouteSegmentPoint> end, const VISITED_MAP & boundaries, std::vector<int64_t> excludedKeys) {
	// FIXME intermediate points
	// measure time

	int iterationsToUpdate = 0;
	if (ctx->progress.get()) {
		ctx->progress->timeToCalculate.Start();
	}

	// Initializing priority queue to visit way segments
	SegmentsComparator sgmCmp;
	SEGMENTS_QUEUE graphDirectSegments(sgmCmp);
	SEGMENTS_QUEUE graphReverseSegments(sgmCmp);

	// Set to not visit one segment twice (stores road.id << X + segmentStart)
	VISITED_MAP visitedDirectSegments;
	VISITED_MAP visitedOppositeSegments;

	initQueuesWithStartEnd(ctx, start, end, graphDirectSegments, graphReverseSegments);

	bool onlyBackward = ctx->getPlanRoadDirection() < 0;
	bool onlyForward = ctx->getPlanRoadDirection() > 0;
	// Extract & analyze segment with min(f(x)) from queue while final segment is not found
	bool forwardSearch = !onlyForward;

	vector<SHARED_PTR<RouteSegment>> result;
	ctx->dijkstraMode = !end ? 1 : (!start ? -1 : 0);
	if (ctx->dijkstraMode == 1) {
		start->others.clear();
		forwardSearch = true;
	} else if (ctx->dijkstraMode == -1) {
		end->others.clear();
		forwardSearch = false;
	}
	SEGMENTS_QUEUE* graphSegments = forwardSearch ?  &graphDirectSegments : &graphReverseSegments;
	float minCost[2] = { -INFINITY, -INFINITY};
	
	while (graphSegments->size() > 0) {
		SHARED_PTR<RouteSegmentCost> cst = graphSegments->top();
		SHARED_PTR<RouteSegment> segment = cst->segment;
		graphSegments->pop();
		// Memory management
		// ctx.memoryOverhead = (visitedDirectSegments.size() + visitedOppositeSegments.size()) * STANDARD_ROAD_VISITED_OVERHEAD +
		//					  (graphDirectSegments.size() + graphReverseSegments.size()) * STANDARD_ROAD_IN_QUEUE_OVERHEAD;
        long visitedCnt = (start ? visitedDirectSegments.size() : 0) + (end ? visitedOppositeSegments.size() : 0);
		if (TRACE_ROUTING) {
			printRoad(forwardSearch ? "F>" : "B>", segment);
		}
        if (ctx->config->MAX_VISITED > 0 && visitedCnt > ctx->config->MAX_VISITED) {
            break;
        }
		bool skipSegment = false;
		if (segment->isFinalSegment) {
            if (TRACE_ROUTING) {
                printRoad(" >>FINAL segment: ", segment);
            }
			result.push_back(segment);
			if (ctx->dijkstraMode != 0) {
				skipSegment = true;
			} else {
				break;
			}
		}


		if (ctx->progress.get()) {
			ctx->progress->visitedSegments++;
		}
		int64_t routePointId = calculateRoutePointId(segment);
		bool visited = false;
		if (forwardSearch) {
			auto it = visitedDirectSegments.find(routePointId);
			if (it != visitedDirectSegments.end()) {
				visited = true;
			}
		} else {
			auto it = visitedOppositeSegments.find(routePointId);
			if (it != visitedOppositeSegments.end()) {
				visited = true;
			}
		}
		
		if (visited) {
			if (TRACE_ROUTING) {
				OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Debug,  " %d >> Already visited by minimum", segment->segmentEnd);
			}
			skipSegment = true;
		} else if (cst->segCost + 0.1 < minCost[forwardSearch ? 1 : 0] && ASSERT_CHECKS && ctx->calculationMode != RouteCalculationMode::COMPLEX) {
			if (ctx->config->heurCoefficient <= 1) {
				//throw new IllegalStateException(cst.cost + " < ???  " + minCost[forwardSearch ? 1 : 0]);
				OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error,  " %f + < ???  %f ", cst->segCost, minCost[forwardSearch ? 1 : 0]);
				return result;
			}
		} else {
			minCost[forwardSearch ? 1 : 0] = cst->segCost;
		}
		
		if (!skipSegment) {
			if (forwardSearch) {
				bool doNotAddIntersections = onlyBackward;
				processRouteSegment(ctx, false, graphDirectSegments, visitedDirectSegments, segment,
									visitedOppositeSegments, boundaries, doNotAddIntersections, excludedKeys);
			} else {
				bool doNotAddIntersections = onlyForward;
				processRouteSegment(ctx, true, graphReverseSegments, visitedOppositeSegments, segment,
									visitedDirectSegments, boundaries, doNotAddIntersections, excludedKeys);
			}
		}
		if (ctx->progress.get() && iterationsToUpdate-- < 0) {
			iterationsToUpdate = 100;
			ctx->progress->updateStatus(
				graphDirectSegments.empty() ? 0 : graphDirectSegments.top()->segment->distanceFromStart,
				(int) graphDirectSegments.size(),
				graphReverseSegments.empty() ? 0 : graphReverseSegments.top()->segment->distanceFromStart,
				(int) graphReverseSegments.size());
			if (ctx->progress->isCancelled()) {
				break;
			}
		}
		
		
		bool reiterate = false;
		reiterate |= checkIfGraphIsEmpty(ctx, ctx->getPlanRoadDirection() <= 0, true, graphReverseSegments, end,
							visitedOppositeSegments, "Route is not found to selected target point.");
		reiterate |= checkIfGraphIsEmpty(ctx, ctx->getPlanRoadDirection() >= 0, false, graphDirectSegments, start,
							visitedDirectSegments, "Route is not found from selected start point.");
		if (reiterate) {
			minCost[0] = -INFINITY;
			minCost[1] = -INFINITY;
		}
		
		if (ctx->planRouteIn2Directions()) {
            // initial iteration make in 2 directions
            if (visitedDirectSegments.empty() && !graphDirectSegments.empty()) {
                forwardSearch = true;
            } else if (visitedOppositeSegments.empty() && !graphReverseSegments.empty()) {
                forwardSearch = false;
            } else if (graphDirectSegments.empty() || graphReverseSegments.empty()) {
                // can't proceed any more - check if final already exist
                graphSegments = graphDirectSegments.empty() ? &graphReverseSegments : &graphDirectSegments;
                if (result.empty()) {
                    while (!graphSegments->empty()) {
                        SHARED_PTR<RouteSegmentCost> pc = graphSegments->top();
                        SHARED_PTR<RouteSegment> segment = cst->segment;
                        graphSegments->pop();
                        if (segment->isFinalSegment) {
                            result.push_back(segment);
                            break;
                        }
                    }
                }
                return result;
			} else {
				SHARED_PTR<RouteSegment> fw = graphDirectSegments.top()->segment;
				SHARED_PTR<RouteSegment> bw = graphReverseSegments.top()->segment;
				forwardSearch = doubleCompare(cost(fw->distanceFromStart, fw->distanceToEnd, ctx),
									cost(bw->distanceFromStart, bw->distanceToEnd,ctx)) <= 0;
			}
		} else {
			// different strategy : use one directional graph
			forwardSearch = onlyForward;
			if (onlyBackward && !graphDirectSegments.empty()) {
				forwardSearch = true;
			}
			if (onlyForward && !graphReverseSegments.empty()) {
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
			return result;
		}
	}
	if (ctx->progress.get()) {
		ctx->progress->timeToCalculate.Pause();
		ctx->progress->visitedDirectSegments += visitedDirectSegments.size();
		ctx->progress->visitedOppositeSegments += visitedOppositeSegments.size();
		ctx->progress->directQueueSize += graphDirectSegments.size();
		ctx->progress->oppositeQueueSize += graphReverseSegments.size();
	}
	return result;
}

bool checkMovementAllowed(RoutingContext* ctx, bool reverseWaySearch, const SHARED_PTR<RouteSegment>& segment) {
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

bool checkViaRestrictions(const SHARED_PTR<RouteSegment>& from, const SHARED_PTR<RouteSegment>& to) {
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
	if (!s) {
		return nullptr;
	}
	auto res = s;
	while (res->getParentRoute() && res->getParentRoute()->getRoad() &&
		   res->getParentRoute()->getRoad()->id == res->getRoad()->id) {
		res = res->getParentRoute();
	}
	return res->getParentRoute();
}

bool checkIfOppositeSegmentWasVisited(RoutingContext* ctx, bool reverseWaySearch, SEGMENTS_QUEUE& graphSegments,
									  const SHARED_PTR<RouteSegment>& currentSegment, const VISITED_MAP & oppositeSegments,
									  const VISITED_MAP & boundaries, std::vector<int64_t> excludedKeys) {
	// check inverse direction for opposite
	int64_t currPoint = calculateRoutePointInternalId(currentSegment->getRoad(), currentSegment->getSegmentEnd(),
													  currentSegment->getSegmentStart());
	const VISITED_MAP * oppositeSegmentsPtr = &oppositeSegments;
    bool containsExcludedKeys = std::find(excludedKeys.begin(), excludedKeys.end(), currPoint) != excludedKeys.end();
    bool ignoreExcludedKeys = true;
    if (boundaries.size() > 0 && ctx->dijkstraMode != 0) {
        // limit by boundaries for dijkstra mode
        oppositeSegmentsPtr = &boundaries;
        ignoreExcludedKeys = false;
    }
	const auto opIt = oppositeSegmentsPtr->find(currPoint);
	if (opIt != oppositeSegmentsPtr->end() && (!containsExcludedKeys || ignoreExcludedKeys)) {
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
			float oppTime = !opposite ? 0 : opposite->distanceFromStart;
			frs->distanceFromStart = oppTime + currentSegment->distanceFromStart;
			frs->distanceToEnd = 0;
			frs->opposite = opposite;
			frs->isFinalSegment = true;
			if (frs->distanceFromStart < 0) {
				// impossible route (when start/point on same segment but different dir) don't add to queue
				return true;
			}
			SHARED_PTR<RouteSegmentCost> rsc = make_shared<RouteSegmentCost>(frs, ctx);
			graphSegments.push(rsc);
			if (TRACE_ROUTING) {
                string prefix = reverseWaySearch ? "B" : "F";
                prefix += "  " + to_string(currentSegment->getSegmentEnd());
                prefix += ">> Final segment : ";
				printRoad(prefix.c_str(), frs);
			}
			if (ctx->progress.get()) {
				ctx->progress->finalSegmentsFound++;
			}
			return true;
		}
	}
    if (boundaries.size() > 0 && ctx->dijkstraMode == 0
        && boundaries.find(currPoint) != boundaries.end()
        && !containsExcludedKeys) {
        // limit search by boundaries
        return true;
    }
	return false;
}

double calculateRouteSegmentTime(RoutingContext* ctx, bool reverseWaySearch, SHARED_PTR<RouteSegment> segment) {
	SHARED_PTR<RouteDataObject> road = segment->getRoad();
	// store <segment> in order to not have unique <segment, direction> in visitedSegments
	short segmentInd = reverseWaySearch ? segment->getSegmentStart() : segment->getSegmentEnd();
	short prevSegmentInd = !reverseWaySearch ? segment->getSegmentStart() : segment->getSegmentEnd();

	// calculate point and try to load neighbor ways if they are not loaded
	double distTimeOnRoadToPass = calcRoutingSegmentTimeOnlyDist(ctx->config->router, segment);
	// calculate possible obstacle plus time
	double obstacle = ctx->config->router->defineRoutingObstacle(road, segmentInd, prevSegmentInd > segmentInd);
	if (obstacle < 0) {
		return -1;
	}
	double heightObstacle = ctx->config->router->defineHeightObstacle(road, segmentInd, prevSegmentInd);
	if (heightObstacle < 0) {
		return -1;
	}
	return obstacle + heightObstacle + distTimeOnRoadToPass;
}

void processRouteSegment(RoutingContext* ctx, bool reverseWaySearch, SEGMENTS_QUEUE& graphSegments,
						 VISITED_MAP& visitedSegments, const SHARED_PTR<RouteSegment>& startSegment,
						 const VISITED_MAP& oppositeSegments, const VISITED_MAP & boundaries, bool doNotAddIntersections,
                         std::vector<int64_t> excludedKeys) {
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
		bool bothDirVisited = checkIfOppositeSegmentWasVisited(ctx, reverseWaySearch, graphSegments, currentSegment, oppositeSegments, boundaries, excludedKeys);

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
				if (TRACE_ROUTING) {
					OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, " %d >> Already visited", currentSegment->segmentEnd);
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
		
		if (bothDirVisited) {
			// We stop here for shortcut creation (we can't improve the neighbors if they're already visited cause the opposite is min - prove by contradiction)
			if (TRACE_ROUTING) {
				OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, " %d >> 2 dir visited", currentSegment->segmentEnd);
			}
			break;
		}

		// 4. load road connections at the end of segment
		nextCurrentSegment = processIntersections(ctx, graphSegments, visitedSegments, currentSegment, reverseWaySearch,
												  doNotAddIntersections);
		
		// Theoretically we should process each step separately but we don't have any issues with it.
		// a) final segment is always in queue & double checked b) using osm segment almost always is shorter routing than other connected
		if (DEBUG_BREAK_EACH_SEGMENT && nextCurrentSegment) {
			if (!doNotAddIntersections) {
				SHARED_PTR<RouteSegmentCost> rsc = make_shared<RouteSegmentCost>(nextCurrentSegment, ctx);
				graphSegments.push(rsc);
			}
			break;
		}
		if (doNotAddIntersections) {
			break;
		}
	}
}

void clearSegments(vector<SHARED_PTR<RouteSegment>>& segments) {
	segments.clear();
	segments.shrink_to_fit();
}

void processRestriction(RoutingContext* ctx, std::vector<SHARED_PTR<RouteSegment>>& inputNext, bool reverseWay,
						int64_t viaId, const SHARED_PTR<RouteDataObject>& road) {
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

bool proccessRestrictions(RoutingContext* ctx, const SHARED_PTR<RouteSegment>& segment,
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
											  VISITED_MAP& visitedSegments, const SHARED_PTR<RouteSegment>& currentSegment,
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
					if (!directionAllowed) {
						nextCurrentSegment = nullptr;
					}
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
			OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Debug, " %d >> There are restrictions", currentSegment->segmentEnd);
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
			// exception as it should not occur, if happens during approximation - should be investigated
			if (ASSERT_CHECKS) {
				//throw new IllegalStateException();
                OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "ctx->calculationMode != RouteCalculationMode::BASE");
			}
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
					road = std::make_shared<RouteSegmentPoint>(r, j - 1, j, currentsDist);
					road->preciseX = prx;
					road->preciseY = pry;
				}
			}
			if (road) {
				if (!transportStop) {
					float prio = ctx->config->router->defineDestinationPriority(road->road);
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
				if (transportStop && (*it)->dist > GPS_POSSIBLE_ERROR * GPS_POSSIBLE_ERROR) {
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

bool combineTwoSegmentResultPlanner(const SHARED_PTR<RouteSegmentResult>& toAdd, const SHARED_PTR<RouteSegmentResult>& previous,
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

void addRouteSegmentToResult(vector<SHARED_PTR<RouteSegmentResult>>& result, const SHARED_PTR<RouteSegmentResult>& res,
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
								VISITED_MAP& visitedSegments, const SHARED_PTR<RouteSegment>& segment,
								const SHARED_PTR<RouteSegment>& next, bool isNullGraph) {
	if (next) {
		if (!checkMovementAllowed(ctx, reverseWaySearch, next)) {
			return false;
		}
		// Can lose 1 during cast double to float
		float obstaclesTime = 0;
		if (next->road->getId() != segment->road->getId()) {
			obstaclesTime = (float) ctx->config->router->calculateTurnTime(next, segment);
		}
		
		if (obstaclesTime < 0) {
			return false;
		}
		float distFromStart = obstaclesTime + segment->distanceFromStart;
		const auto visIt = visitedSegments.find(calculateRoutePointId(next));
		if (visIt != visitedSegments.end() && visIt->second) {
			const auto& visitedSeg = visIt->second;
			if (TRACE_ROUTING) {
				printRoad(("  " + to_string(segment->segmentEnd) + ">?").c_str(), visitedSegments.at(calculateRoutePointId(next)));
			}
			// The segment was already visited! We can try to follow new route if it's shorter.
			// That is very exceptional situation and almost exception, it can happen
			// 1. We underestimate distanceToEnd - wrong h() of A* (heuristic > 1)
			// 2. We don't process small segments 1 by 1 as we should by Dijkstra
			if (distFromStart < visitedSeg->distanceFromStart) {
				double routeSegmentTime = calculateRouteSegmentTime(ctx, reverseWaySearch, visitedSeg);
				// we need to properly compare @distanceFromStart VISITED and NON-VISITED segment
				if (visitedSeg->distanceFromStart - (distFromStart + routeSegmentTime) > 0.01) { // cause we do double -> float we can get into infinite loop here
					// Here it's not very legitimate action cause in theory we need to go up to the final segment in the
					// queue & decrease final time But it's compensated by chain reaction cause next.distanceFromStart <
					// finalSegment.distanceFromStart and revisiting all segments

					// We don't check ```next.getParentRoute() == null``` cause segment could be unloaded
					// so we need to add segment back to the queue & reassign the parent (same as for
					// next.getParentRoute() == null)
					if (ctx->getHeuristicCoefficient() <= 1) {
						if (DEBUG_BREAK_EACH_SEGMENT && ASSERT_CHECKS) {
							//throw new IllegalStateException();
							OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error,  "DEBUG_BREAK_EACH_SEGMENT && ASSERT_CHECKS");
							return false;
						}
						if (PRINT_ROUTING_ALERTS) {
							OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Debug, "! ALERT new faster path to a visited segment: %f < %f",
											  (distFromStart + routeSegmentTime), visIt->second.get()->distanceFromStart);
							printRoad("next", next);
							printRoad("visIt", visitedSeg.get());
						} else {
							ctx->alertFasterRoadToVisitedSegments++;
						}
						
					}
					visitedSegments.erase(calculateRoutePointId(next));
				} else {
					return false;
				}
			} else {
				return false;
			}
		}
		if (!next->isSegmentAttachedToStart() || cost(next->distanceFromStart, next->distanceToEnd,
								ctx) > cost(distFromStart, segment->distanceToEnd, ctx)) {
			next->distanceFromStart = distFromStart;
			next->distanceToEnd = segment->distanceToEnd;
			if (TRACE_ROUTING) {
				string s = next->isSegmentAttachedToStart() ? "*" : "";
				printRoad((" " + s + to_string(segment->getSegmentEnd()) + ">>").c_str(), next);
			}
			// put additional information to recover whole route after
			next->parentRoute = segment;
			if (!isNullGraph) {
				SHARED_PTR<RouteSegmentCost> rsc = make_shared<RouteSegmentCost>(next, ctx);
				graphSegments.push(rsc);
			}
			return true;
		}
	}
	return false;
}

float distanceFromStart(SHARED_PTR<RouteSegment> s) {
    return s == nullptr ? 0 : s->distanceFromStart;
}

vector<SHARED_PTR<RouteSegmentResult>> convertFinalSegmentToResults(RoutingContext* ctx,
																	const SHARED_PTR<RouteSegment>& finalSegment) {
	vector<SHARED_PTR<RouteSegmentResult>> result;
	if (finalSegment) {
        ctx->conditionalTime += finalSegment->distanceFromStart;
        float correctionTime = finalSegment->opposite == nullptr ? 0 :
                        finalSegment->distanceFromStart - distanceFromStart(finalSegment->opposite) - distanceFromStart(finalSegment->parentRoute);
        SHARED_PTR<RouteSegment> thisSegment =  finalSegment->opposite == nullptr ? finalSegment : finalSegment->getParentRoute(); // for dijkstra
        SHARED_PTR<RouteSegment> segment = finalSegment->isReverseWaySearch() ? thisSegment : finalSegment->opposite;
		while (segment && segment->getRoad()) {
			auto res = std::make_shared<RouteSegmentResult>(segment->road, segment->getSegmentEnd(),
															segment->getSegmentStart());
			float parentRoutingTime =
				segment->getParentRoute() != nullptr ? segment->getParentRoute()->distanceFromStart : 0;
			res->routingTime = segment->distanceFromStart - parentRoutingTime + correctionTime;
            correctionTime = 0;
			segment = segment->getParentRoute();
			addRouteSegmentToResult(result, res, false);
		}
		// reverse it just to attach good direction roads
		std::reverse(result.begin(), result.end());
        segment = finalSegment->isReverseWaySearch() ? finalSegment->opposite : thisSegment;
		while (segment && segment->getRoad()) {
			auto res = std::make_shared<RouteSegmentResult>(segment->road, segment->getSegmentStart(),
															segment->getSegmentEnd());
			float parentRoutingTime =
				segment->getParentRoute() != nullptr ? segment->getParentRoute()->distanceFromStart : 0;
            res->routingTime = segment->distanceFromStart - parentRoutingTime + correctionTime;
            correctionTime = 0;
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
    vector<SHARED_PTR<RouteSegment>> results = searchRouteInternal(ctx, start, end, {}, {});
	SHARED_PTR<RouteSegment> finalSegment = nullptr;
	if (results.size() > 0) {
		finalSegment = results[0];
	}
	vector<SHARED_PTR<RouteSegmentResult>> res = convertFinalSegmentToResults(ctx, finalSegment);
    ctx->finalRouteSegment = finalSegment;
	attachConnectedRoads(ctx, res);
	return res;
}
