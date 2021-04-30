#ifndef _OSMAND_ROUTE_PLANNER_FRONT_END_CPP
#define _OSMAND_ROUTE_PLANNER_FRONT_END_CPP
#include "routePlannerFrontEnd.h"

#include "binaryRoutePlanner.h"
#include "routeResultPreparation.h"
#include "routeSegment.h"
#include "routeSegmentResult.h"
#include "routingConfiguration.h"

SHARED_PTR<RoutingContext> RoutePlannerFrontEnd::buildRoutingContext(
	SHARED_PTR<RoutingConfiguration> config, RouteCalculationMode rm /*= RouteCalculationMode::NORMAL*/) {
	return SHARED_PTR<RoutingContext>(new RoutingContext(config, rm));
}

SHARED_PTR<RouteSegment> RoutePlannerFrontEnd::getRecalculationEnd(RoutingContext* ctx) {
	SHARED_PTR<RouteSegment> recalculationEnd;
	bool runRecalculation = ctx->previouslyCalculatedRoute.size() > 0 && ctx->config->recalculateDistance != 0;
	if (runRecalculation) {
		vector<SHARED_PTR<RouteSegmentResult>> rlist;
		float distanceThreshold = ctx->config->recalculateDistance;
		float threshold = 0;
		for (auto rr : ctx->previouslyCalculatedRoute) {
			threshold += rr->distance;
			if (threshold > distanceThreshold) {
				rlist.push_back(rr);
			}
		}
		runRecalculation = rlist.size() > 0;
		if (rlist.size() > 0) {
			SHARED_PTR<RouteSegment> previous;
			for (int i = 0; i <= rlist.size() - 1; i++) {
				auto rr = rlist[i];
				SHARED_PTR<RouteSegment> segment = std::make_shared<RouteSegment>(rr->object, rr->getEndPointIndex());
				if (previous) {
					previous->parentRoute = segment;
					previous->parentSegmentEnd = rr->getStartPointIndex();
				} else {
					recalculationEnd = segment;
				}
				previous = segment;
			}
		}
	}
	return recalculationEnd;
}

void refreshProgressDistance(RoutingContext* ctx) {
	if (ctx->progress) {
		ctx->progress->distanceFromBegin = 0;
		ctx->progress->distanceFromEnd = 0;
		ctx->progress->reverseSegmentQueueSize = 0;
		ctx->progress->directSegmentQueueSize = 0;
		float rd = (float)squareRootDist31(ctx->startX, ctx->startY, ctx->targetX, ctx->targetY);
		float speed = 0.9f * ctx->config->router->maxSpeed;
		ctx->progress->totalEstimatedDistance = (float)(rd / speed);
	}
}

double projectDistance(vector<SHARED_PTR<RouteSegmentResult>>& res, int k, int px, int py) {
	auto sr = res[k];
	auto r = sr->object;
	std::pair<int, int> pp =
		getProjectionPoint(px, py, r->pointsX[sr->getStartPointIndex()], r->pointsY[sr->getStartPointIndex()],
						   r->pointsX[sr->getEndPointIndex()], r->pointsY[sr->getEndPointIndex()]);
	double currentsDist = squareRootDist31(pp.first, pp.second, px, py);
	return currentsDist;
}

void updateResult(SHARED_PTR<RouteSegmentResult>& routeSegmentResult, int px, int py, bool st) {
	int pind = st ? routeSegmentResult->getStartPointIndex() : routeSegmentResult->getEndPointIndex();

	auto r = routeSegmentResult->object;
	std::pair<int, int>* before = NULL;
	std::pair<int, int>* after = NULL;
	if (pind > 0) {
		before = new std::pair<int, int>(
			getProjectionPoint(px, py, r->pointsX[pind - 1], r->pointsY[pind - 1], r->pointsX[pind], r->pointsY[pind]));
	}
	if (pind < r->getPointsLength() - 1) {
		after = new std::pair<int, int>(
			getProjectionPoint(px, py, r->pointsX[pind + 1], r->pointsY[pind + 1], r->pointsX[pind], r->pointsY[pind]));
	}
	int insert = 0;
	double dd = measuredDist31(px, py, r->pointsX[pind], r->pointsY[pind]);
	double ddBefore = std::numeric_limits<double>::infinity();
	double ddAfter = std::numeric_limits<double>::infinity();
	std::pair<int, int>* i = NULL;
	if (before != NULL) {
		ddBefore = measuredDist31(px, py, before->first, before->second);
		if (ddBefore < dd) {
			insert = -1;
			i = before;
		}
	}

	if (after != NULL) {
		ddAfter = measuredDist31(px, py, after->first, after->second);
		if (ddAfter < dd && ddAfter < ddBefore) {
			insert = 1;
			i = after;
		}
	}

	if (insert != 0) {
		if (st && routeSegmentResult->getStartPointIndex() < routeSegmentResult->getEndPointIndex()) {
			routeSegmentResult->setEndPointIndex(routeSegmentResult->getEndPointIndex() + 1);
		}
		if (!st && routeSegmentResult->getStartPointIndex() > routeSegmentResult->getEndPointIndex()) {
			routeSegmentResult->setStartPointIndex(routeSegmentResult->getStartPointIndex() + 1);
		}
		if (insert > 0) {
			r->insert(pind + 1, i->first, i->second);
			if (st) {
				routeSegmentResult->setStartPointIndex(routeSegmentResult->getStartPointIndex() + 1);
			}
			if (!st) {
				routeSegmentResult->setEndPointIndex(routeSegmentResult->getEndPointIndex() + 1);
			}
		} else {
			r->insert(pind, i->first, i->second);
		}
	}
	if (before != NULL) {
		delete before;
	}
	if (after != NULL) {
		delete after;
	}
}

bool addSegment(int x31, int y31, RoutingContext* ctx, int indexNotFound, vector<SHARED_PTR<RouteSegmentPoint>>& res,
				bool transportStop) {
	auto f = findRouteSegment(x31, y31, ctx, transportStop);
	if (!f) {
		ctx->progress->segmentNotFound = indexNotFound;
		return false;
	} else {
		// OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "[Native] Route segment found %d %s",
		// f->getRoad()->getId(), f->getRoad()->getName().c_str());
		res.push_back(f);
		return true;
	}
}

void makeStartEndPointsPrecise(vector<SHARED_PTR<RouteSegmentResult>>& res, int startX, int startY, int endX, int endY,
							   vector<int> intermediatesX, vector<int> intermediatesY) {
	if (res.size() > 0) {
		updateResult(res[0], startX, startY, true);
		updateResult(res[res.size() - 1], endX, endY, false);
		if (!intermediatesX.empty()) {
			int k = 1;
			for (int i = 0; i < intermediatesX.size(); i++) {
				int px = intermediatesX[i];
				int py = intermediatesY[i];
				for (; k < res.size(); k++) {
					double currentsDist = projectDistance(res, k, px, py);
					if (currentsDist < 500 * 500) {
						for (int k1 = k + 1; k1 < res.size(); k1++) {
							double c2 = projectDistance(res, k1, px, py);
							if (c2 < currentsDist) {
								k = k1;
								currentsDist = c2;
							} else if (k1 - k > 15) {
								break;
							}
						}
						updateResult(res[k], px, py, false);
						if (k < res.size() - 1) {
							updateResult(res[k + 1], px, py, true);
						}
						break;
					}
				}
			}
		}
	}
}

vector<SHARED_PTR<RouteSegmentResult>> runRouting(RoutingContext* ctx, SHARED_PTR<RouteSegment> recalculationEnd) {
	refreshProgressDistance(ctx);

	OsmAnd::ElapsedTimer timer;
	timer.Start();

	vector<SHARED_PTR<RouteSegmentResult>> result = searchRouteInternal(ctx, false);

	timer.Pause();
	// OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "[Native] routing took %.3f seconds",
	// (double)timer.GetElapsedMs() / 1000.0);

	if (recalculationEnd) {
		OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "[Native] use precalculated route");
		SHARED_PTR<RouteSegment> current = recalculationEnd;

		while (current->parentRoute.lock()) {
			SHARED_PTR<RouteSegment> pr = current->parentRoute.lock();
			auto segmentResult = std::make_shared<RouteSegmentResult>(pr->road, current->parentSegmentEnd, pr->segmentStart);
			result.push_back(segmentResult);
			current = pr;
		}
	}
	if (!result.empty()) {
		for (auto seg : result) {
			seg->preAttachedRoutes = seg->attachedRoutes;
		}
	}
	if (ctx->finalRouteSegment && ctx->progress) {
		ctx->progress->routingCalculatedTime += ctx->finalRouteSegment->distanceFromStart;
	}

	return prepareResult(ctx, result);
}

vector<SHARED_PTR<RouteSegmentResult>> RoutePlannerFrontEnd::searchRouteInternalPrepare(
	RoutingContext* ctx, SHARED_PTR<RouteSegmentPoint> start, SHARED_PTR<RouteSegmentPoint> end,
	SHARED_PTR<PrecalculatedRouteDirection> routeDirection) {
	auto recalculationEnd = getRecalculationEnd(ctx);
	if (recalculationEnd) {
		ctx->initStartAndTargetPoints(start, recalculationEnd);
	} else {
		ctx->initStartAndTargetPoints(start, end);
	}
	if (routeDirection) {
		ctx->precalcRoute = routeDirection->adopt(ctx);
	}
	return runRouting(ctx, recalculationEnd);
}

double GpxRouteApproximation::distFromLastPoint(double lat, double lon) {
	if (result.size() > 0) {
		return getDistance(getLastPoint()->lat, getLastPoint()->lon, lat, lon);
	}
	return 0;
}

SHARED_PTR<LatLon> GpxRouteApproximation::getLastPoint() {
	if (result.size() > 0) {
		return result[result.size() - 1]->getEndPoint();
	}
	return nullptr;
}

GpxRouteApproximation* searchGpxRoute(GpxRouteApproximation* gctx, vector<SHARED_PTR<GpxPoint>> &gpxPoints) {
	if (gctx->ctx->progress == NULL) {
		gctx->ctx->progress = std::make_shared<RouteCalculationProgress>();
	}
	gctx->ctx->progress->timeToCalculate.Start();
	SHARED_PTR<GpxPoint> start;
	SHARED_PTR<GpxPoint> prev;
	if (gpxPoints.size() > 0) {
		gctx->ctx->progress->totalIterations =
			(int)(gpxPoints[gpxPoints.size() - 1]->cumDist / gctx->MAXIMUM_STEP_APPROXIMATION + 1);
		start = gpxPoints[0];
	}
	while (start && !gctx->ctx->progress->isCancelled()) {
		double routeDist = gctx->MAXIMUM_STEP_APPROXIMATION;
		SHARED_PTR<GpxPoint> next = findNextGpxPointWithin(gpxPoints, start, routeDist);
		bool routeFound = false;
		if (next && initRoutingPoint(start, gctx, gctx->MINIMUM_POINT_APPROXIMATION)) {
			gctx->ctx->progress->totalEstimatedDistance = 0;
			gctx->ctx->progress->iteration = (int)(next->cumDist / gctx->MAXIMUM_STEP_APPROXIMATION);
			while (routeDist >= gctx->MINIMUM_STEP_APPROXIMATION && !routeFound) {
				routeFound = initRoutingPoint(next, gctx, gctx->MINIMUM_POINT_APPROXIMATION);
				if (routeFound) {
					routeFound = findGpxRouteSegment(gctx, gpxPoints, start, next, prev != nullptr);
					if (routeFound) {
						// route is found - cut the end of the route and move to next iteration
						// start.stepBackRoute = new ArrayList<RouteSegmentResult>();
						// boolean stepBack = true;
						bool stepBack = stepBackAndFindPrevPointInRoute(gctx, gpxPoints, start, next);
						if (!stepBack) {
							// not supported case (workaround increase MAXIMUM_STEP_APPROXIMATION)
							OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info,
											  "Consider to increase MAXIMUM_STEP_APPROXIMATION to: %f", routeDist * 2);
							start->routeToTarget.clear();
							routeFound = false;
							break;
						}
					}
				}
				if (!routeFound) {
					// route is not found move next point closer to start point (distance / 2)
					routeDist = routeDist / 2;
					if (routeDist < gctx->MINIMUM_STEP_APPROXIMATION &&
						routeDist > gctx->MINIMUM_STEP_APPROXIMATION / 2 + 1) {
						routeDist = gctx->MINIMUM_STEP_APPROXIMATION;
					}
					next = findNextGpxPointWithin(gpxPoints, start, routeDist);
					if (next) {
						routeDist = min(next->cumDist - start->cumDist, routeDist);
					}
				}
			}
		}
		// route is not found skip segment and keep it as straight line on display
		if (!routeFound) {
			// route is not found, move start point by
			next = findNextGpxPointWithin(gpxPoints, start, gctx->MINIMUM_STEP_APPROXIMATION);
			if (prev) {
				prev->routeToTarget.insert(prev->routeToTarget.end(), prev->stepBackRoute.begin(),
										   prev->stepBackRoute.end());
				makeSegmentPointPrecise(prev->routeToTarget[prev->routeToTarget.size() - 1], start->lat, start->lon,
										false);
				if (next) {
					OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "NOT found route from: %s at %d",
									  start->pnt->getRoad()->getName().c_str(), start->pnt->getSegmentStart());
				}
			}
			prev = nullptr;
		} else {
			prev = start;
		}
		start = next;
	}
	if (gctx->ctx->progress != nullptr) {
		gctx->ctx->progress->timeToCalculate.Pause();
	}
	// BinaryRoutePlanner.printDebugMemoryInformation(gctx.ctx);
	calculateGpxRoute(gctx, gpxPoints);
	if (!gctx->result.empty() && !gctx->ctx->progress->isCancelled()) {
		// new RouteResultPreparation().printResults(gctx.ctx, gpxPoints[0]->loc, gpxPoints[gpxPoints.size() - 1]->loc,
		// gctx.result); System.out.println(gctx);
	}
	//+ if (resultMatcher != NULL) {
	//+ resultMatcher.publish(gctx.ctx->progress->isCancelled() ? NULL : gctx);
	//+ }
	return gctx;
}

void makeSegmentPointPrecise(SHARED_PTR<RouteSegmentResult> routeSegmentResult, double lat, double lon, bool st) {
	int px = get31TileNumberX(lon);
	int py = get31TileNumberY(lat);
	int pind = st ? routeSegmentResult->getStartPointIndex() : routeSegmentResult->getEndPointIndex();

	SHARED_PTR<RouteDataObject> r = std::make_shared<RouteDataObject>(routeSegmentResult->object);
	routeSegmentResult->object = r;
	std::pair<int, int>* before = NULL;
	std::pair<int, int>* after = NULL;

	if (pind > 0) {
		before = new std::pair<int, int>(
			getProjectionPoint(px, py, r->pointsX[pind - 1], r->pointsY[pind - 1], r->pointsX[pind], r->pointsY[pind]));
	}
	if (pind < r->pointsX.size() - 1) {
		after = new std::pair<int, int>(
			getProjectionPoint(px, py, r->pointsX[pind + 1], r->pointsY[pind + 1], r->pointsX[pind], r->pointsY[pind]));
	}
	int insert = 0;
	double dd = getDistance(lat, lon, get31LatitudeY(r->pointsY[pind]), get31LongitudeX(r->pointsX[pind]));
	double ddBefore = std::numeric_limits<double>::infinity();
	double ddAfter = std::numeric_limits<double>::infinity();
	std::pair<int, int>* i = NULL;
	if (before != NULL) {
		ddBefore = getDistance(lat, lon, get31LatitudeY(before->second), get31LongitudeX(before->first));
		if (ddBefore < dd) {
			insert = -1;
			i = before;
		}
	}

	if (after != NULL) {
		ddAfter = getDistance(lat, lon, get31LatitudeY(after->second), get31LongitudeX(after->first));
		if (ddAfter < dd && ddAfter < ddBefore) {
			insert = 1;
			i = after;
		}
	}

	if (insert != 0) {
		if (st && routeSegmentResult->getStartPointIndex() < routeSegmentResult->getEndPointIndex()) {
			routeSegmentResult->setEndPointIndex(routeSegmentResult->getEndPointIndex() + 1);
		}
		if (!st && routeSegmentResult->getStartPointIndex() > routeSegmentResult->getEndPointIndex()) {
			routeSegmentResult->setStartPointIndex(routeSegmentResult->getStartPointIndex() + 1);
		}
		if (insert > 0) {
			r->insert(pind + 1, i->first, i->second);
			if (st) {
				routeSegmentResult->setStartPointIndex(routeSegmentResult->getStartPointIndex() + 1);
			}
			if (!st) {
				routeSegmentResult->setEndPointIndex(routeSegmentResult->getEndPointIndex() + 1);
			}
		} else {
			r->insert(pind, i->first, i->second);
		}
	}
}

bool stepBackAndFindPrevPointInRoute(GpxRouteApproximation* gctx, vector<SHARED_PTR<GpxPoint>> &gpxPoints,
									 SHARED_PTR<GpxPoint> start, SHARED_PTR<GpxPoint> next) {
	// step back to find to be sure
	// 1) route point is behind GpxPoint - MINIMUM_POINT_APPROXIMATION (end route point could slightly ahead)
	// 2) we don't miss correct turn i.e. points could be attached to muliple routes
	// 3) to make sure that we perfectly connect to RoadDataObject points
	double STEP_BACK_DIST = max(gctx->MINIMUM_POINT_APPROXIMATION, gctx->MINIMUM_STEP_APPROXIMATION);
	double d = 0;
	int segmendInd = start->routeToTarget.size() - 1;
	bool search = true;
	start->stepBackRoute.clear();
mainLoop:
	for (; segmendInd >= 0 && search; segmendInd--) {
		RouteSegmentResult rr = *start->routeToTarget[segmendInd];
		bool minus = rr.getStartPointIndex() < rr.getEndPointIndex();
		int nextInd;
		for (int j = rr.getEndPointIndex(); j != rr.getStartPointIndex(); j = nextInd) {
			nextInd = minus ? j - 1 : j + 1;
			d += getDistance(rr.getPoint(j)->lat, rr.getPoint(j)->lon, rr.getPoint(nextInd)->lat,
							 rr.getPoint(nextInd)->lon);
			if (d > STEP_BACK_DIST) {
				if (nextInd == rr.getStartPointIndex()) {
					segmendInd--;
				} else {
					start->stepBackRoute.push_back(std::make_shared<RouteSegmentResult>(
						RouteSegmentResult(rr.object, nextInd, rr.getEndPointIndex())));
					rr.setEndPointIndex(nextInd);
				}
				search = false;
				goto mainLoop;
			}
		}
	}
	if (segmendInd == -1) {
		// here all route segments - 1 is longer than needed distance to step back
		return false;
	}

	while (start->routeToTarget.size() > segmendInd + 1) {
		SHARED_PTR<RouteSegmentResult> removed = start->routeToTarget[segmendInd + 1];
		start->routeToTarget.erase(start->routeToTarget.begin() + segmendInd + 1);
		start->stepBackRoute.push_back(removed);
	}
	SHARED_PTR<RouteSegmentResult> res = start->routeToTarget[segmendInd];
	next->pnt = std::make_shared<RouteSegmentPoint>(res->object, res->getEndPointIndex());
	return true;
}

void calculateGpxRoute(GpxRouteApproximation* gctx, vector<SHARED_PTR<GpxPoint>> &gpxPoints) {
	RoutingIndex* reg = new RoutingIndex();
	reg->initRouteEncodingRule(0, "highway", UNMATCHED_HIGHWAY_TYPE);
	vector<SHARED_PTR<LatLon>> lastStraightLine;
	SHARED_PTR<GpxPoint> straightPointStart = NULL;
	for (int i = 0; i < gpxPoints.size() && !gctx->ctx->progress->isCancelled();) {
		SHARED_PTR<GpxPoint> pnt = gpxPoints[i];
		if (!pnt->routeToTarget.empty()) {
			SHARED_PTR<LatLon> startPoint = std::make_shared<LatLon>(pnt->routeToTarget[0]->getStartPoint()->lat,
																	 pnt->routeToTarget[0]->getStartPoint()->lon);
			if (!lastStraightLine.empty()) {
				lastStraightLine.push_back(startPoint);
				addStraightLine(gctx, lastStraightLine, straightPointStart, reg);
				lastStraightLine.clear();
			}
			if (gctx->distFromLastPoint(startPoint->lat, startPoint->lon) > 1) {
				gctx->routeGapDistance += gctx->distFromLastPoint(startPoint->lat, startPoint->lon);
				OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info,
								  "????? gap of route point = %f, gap of actual gpxPoint = %f, lat = %f lon = %f ",
								  gctx->distFromLastPoint(startPoint->lat, startPoint->lon),
								  gctx->distFromLastPoint(pnt->lat, pnt->lon), pnt->lat, pnt->lon);
			}
			gctx->finalPoints.push_back(pnt);
			gctx->result.insert(gctx->result.end(), pnt->routeToTarget.begin(), pnt->routeToTarget.end());
			i = pnt->targetInd;
		} else {
			// add straight line from i -> i+1
			if (lastStraightLine.empty()) {
				straightPointStart = pnt;
				// make smooth connection
				if (gctx->distFromLastPoint(pnt->lat, pnt->lon) > 1) {
					lastStraightLine.push_back(gctx->getLastPoint());
				}
			}
			lastStraightLine.push_back(std::make_shared<LatLon>(pnt->lat, pnt->lon));
			i++;
		}
	}
	if (!lastStraightLine.empty()) {
		addStraightLine(gctx, lastStraightLine, straightPointStart, reg);
	}
	// clean turns to recaculate them
	cleanupResultAndAddTurns(gctx);
}

void addStraightLine(GpxRouteApproximation* gctx, vector<SHARED_PTR<LatLon>> &lastStraightLine,
					 SHARED_PTR<GpxPoint> strPnt, RoutingIndex* reg) {
	SHARED_PTR<RouteDataObject> rdo = std::make_shared<RouteDataObject>(reg);
	if (gctx->SMOOTHEN_POINTS_NO_ROUTE > 0) {
		simplifyDouglasPeucker(lastStraightLine, gctx->SMOOTHEN_POINTS_NO_ROUTE, 0, lastStraightLine.size() - 1);
	}
	int s = lastStraightLine.size();
	vector<uint32_t> x;
	vector<uint32_t> y;
	for (int i = 0; i < s; i++) {
		if (lastStraightLine[i] != NULL) {
			SHARED_PTR<LatLon> l = lastStraightLine[i];
			int t = x.size() - 1;
			x.push_back(get31TileNumberX(l->lon));
			y.push_back(get31TileNumberY(l->lat));
			if (t >= 0) {
				double dist = squareRootDist31(x[t], y[t], x[t + 1], y[t + 1]);
				gctx->routeDistanceUnmatched += dist;
			}
		}
	}
	rdo->pointsX = x;
	rdo->pointsY = y;
	rdo->types = {0};
	rdo->id = -1;
	strPnt->routeToTarget.clear();
	strPnt->straightLine = true;
	SHARED_PTR<RouteSegmentResult> rsr = std::make_shared<RouteSegmentResult>(rdo, 0, rdo->getPointsLength() - 1);
	strPnt->routeToTarget.push_back(rsr);

	prepareResult(gctx->ctx, strPnt->routeToTarget);

	// VIEW: comment to see road without straight connections
	gctx->finalPoints.push_back(strPnt);
	gctx->result.insert(gctx->result.end(), strPnt->routeToTarget.begin(), strPnt->routeToTarget.end());
}

void cleanupResultAndAddTurns(GpxRouteApproximation* gctx) {
	// cleanup double joints
	int LOOK_AHEAD = 4;
	for (int i = 0; i < gctx->result.size() && !gctx->ctx->progress->isCancelled(); i++) {
		SHARED_PTR<RouteSegmentResult> s = gctx->result[i];
		for (int j = i + 2; j <= i + LOOK_AHEAD && j < gctx->result.size(); j++) {
			SHARED_PTR<RouteSegmentResult> e = gctx->result[j];
			if (e->getStartPoint()->isEquals(*s->getEndPoint())) {
				while ((--j) != i) {
					gctx->result.erase(gctx->result.begin() + j);
				}
				break;
			}
		}
	}
	for (SHARED_PTR<RouteSegmentResult> r : gctx->result) {
		r->turnType = NULL;
		r->description = "";
	}
	if (!gctx->ctx->progress->isCancelled()) {
		prepareTurnResults(gctx->ctx, gctx->result);
	}
}

void simplifyDouglasPeucker(vector<SHARED_PTR<LatLon>> &l, double eps, int start, int end) {
	double dmax = -1;
	int index = -1;
	SHARED_PTR<LatLon> s = l[start];
	SHARED_PTR<LatLon> e = l[end];
	for (int i = start + 1; i <= end - 1; i++) {
		SHARED_PTR<LatLon> ip = l[i];
		double dist = getOrthogonalDistance(ip->lat, ip->lon, s->lat, s->lon, e->lat, e->lon);
		if (dist > dmax) {
			dmax = dist;
			index = i;
		}
	}
	if (dmax >= eps) {
		simplifyDouglasPeucker(l, eps, start, index);
		simplifyDouglasPeucker(l, eps, index, end);
	} else {
		l.erase(l.begin() + start + 1, l.begin() + end);
		// for(int i = start + 1; i < end; i++ ) {
		// 	l[i] = null;
		// }
	}
}

bool initRoutingPoint(SHARED_PTR<GpxPoint> start, GpxRouteApproximation* gctx, double distThreshold) {
	if (start != NULL && start->pnt == NULL) {
		gctx->routePointsSearched++;
		SHARED_PTR<RouteSegmentPoint> rsp =
			findRouteSegment(get31TileNumberX(start->lon), get31TileNumberY(start->lat), gctx->ctx);
		if (rsp != NULL) {
			SHARED_PTR<LatLon> point = rsp->getPreciseLatLon();
			if (getDistance(point->lat, point->lon, start->lat, start->lon) < distThreshold) {
				start->pnt = rsp;
			}
		}
	}
	if (start != NULL && start->pnt != NULL) {
		return true;
	}
	return false;
}

SHARED_PTR<GpxPoint> findNextGpxPointWithin(vector<SHARED_PTR<GpxPoint>> &gpxPoints, SHARED_PTR<GpxPoint> start,
											double dist) {
	// returns first point with that has slightly more than dist or last point
	int plus = dist > 0 ? 1 : -1;
	int targetInd = start->ind + plus;
	SHARED_PTR<GpxPoint> target;
	while (targetInd < gpxPoints.size() && targetInd >= 0) {
		target = gpxPoints[targetInd];
		if (abs(target->cumDist - start->cumDist) > abs(dist)) {
			break;
		}
		targetInd = targetInd + plus;
	}
	return target;
}

bool findGpxRouteSegment(GpxRouteApproximation* gctx, vector<SHARED_PTR<GpxPoint>> &gpxPoints,
						 SHARED_PTR<GpxPoint> start, SHARED_PTR<GpxPoint> target, bool prevRouteCalculated) {
	vector<SHARED_PTR<RouteSegmentResult>> res;
	bool routeIsCorrect = false;
	if (start->pnt != NULL && target->pnt != NULL) {
		start->pnt = std::make_shared<RouteSegmentPoint>(*start->pnt);
		target->pnt = std::make_shared<RouteSegmentPoint>(*target->pnt);
		gctx->routeDistCalculations += (target->cumDist - start->cumDist);
		gctx->routeCalculations++;
		RoutePlannerFrontEnd rpfe = RoutePlannerFrontEnd();
		res = rpfe.searchRouteInternalPrepare(gctx->ctx, start->pnt, target->pnt, NULL);
		// BinaryRoutePlanner.printDebugMemoryInformation(gctx.ctx);
		routeIsCorrect = !res.empty();
		for (int k = start->ind + 1; routeIsCorrect && k < target->ind; k++) {
			SHARED_PTR<GpxPoint> ipoint = gpxPoints[k];
			if (!pointCloseEnough(gctx, ipoint, res)) {
				routeIsCorrect = false;
			}
		}
		if (routeIsCorrect) {
			// correct start point though don't change end point
			if (!prevRouteCalculated) {
				// make first position precise
				makeSegmentPointPrecise(res[0], start->lat, start->lon, true);
			} else {
				if (res[0]->object->getId() == start->pnt->getRoad()->getId()) {
					// start point could shift to +-1 due to direction
					res[0]->setStartPointIndex(start->pnt->getSegmentStart());
				} else {
					// for native routing this is possible when point lies on intersection of 2 lines
					// solution here could be to pass to native routing id of the route
					// though it should not create any issue
					// System.out.println("??? not found " + start.pnt.getRoad().getId() + " instead "
					// + res.get(0).getObject().getId());
				}
			}
			start->routeToTarget = res;
			start->targetInd = target->ind;
		}
	}
	return routeIsCorrect;
}

bool pointCloseEnough(GpxRouteApproximation* gctx, SHARED_PTR<GpxPoint> ipoint,
					  vector<SHARED_PTR<RouteSegmentResult>> &res) {
	int px = get31TileNumberX(ipoint->lon);
	int py = get31TileNumberY(ipoint->lat);
	double SQR = gctx->MINIMUM_POINT_APPROXIMATION;
	SQR = SQR * SQR;
	for (SHARED_PTR<RouteSegmentResult> sr : res) {
		int start = sr->getStartPointIndex();
		int end = sr->getEndPointIndex();
		if (sr->getStartPointIndex() > sr->getEndPointIndex()) {
			start = sr->getEndPointIndex();
			end = sr->getStartPointIndex();
		}
		for (int i = start; i < end; i++) {
			RouteDataObject r = sr->object;
			std::pair<int, int> pp =
				getProjectionPoint(px, py, r.pointsX[i], r.pointsY[i], r.pointsX[i + 1], r.pointsY[i + 1]);
			double currentsDist = squareDist31TileMetric((int)pp.first, (int)pp.second, px, py);
			if (currentsDist <= SQR) {
				return true;
			}
		}
	}
	return false;
}

vector<SHARED_PTR<RouteSegmentResult>> RoutePlannerFrontEnd::searchRoute(
	RoutingContext* ctx, vector<SHARED_PTR<RouteSegmentPoint>>& points,
	SHARED_PTR<PrecalculatedRouteDirection> routeDirection) {
	if (points.size() <= 2) {
		if (!useSmartRouteRecalculation) {
			ctx->previouslyCalculatedRoute.clear();
		}
		return searchRouteInternalPrepare(ctx, points[0], points[1], routeDirection);
	}

	vector<SHARED_PTR<RouteSegmentResult>> firstPartRecalculatedRoute;
	vector<SHARED_PTR<RouteSegmentResult>> restPartRecalculatedRoute;
	if (!ctx->previouslyCalculatedRoute.empty()) {
		auto prev = ctx->previouslyCalculatedRoute;
		int64_t id = points[1]->getRoad()->getId();
		uint16_t ss = points[1]->getSegmentStart();
		int px = points[1]->getRoad()->pointsX[ss];
		int py = points[1]->getRoad()->pointsY[ss];
		for (int i = 0; i < prev.size(); i++) {
			auto rsr = prev[i];
			if (id == rsr->object->getId()) {
				if (measuredDist31(rsr->object->pointsX[rsr->getEndPointIndex()],
								   rsr->object->pointsY[rsr->getEndPointIndex()], px, py) < 50) {
					firstPartRecalculatedRoute.clear();
					restPartRecalculatedRoute.clear();
					for (int k = 0; k < prev.size(); k++) {
						if (k <= i) {
							firstPartRecalculatedRoute.push_back(prev[k]);
						} else {
							restPartRecalculatedRoute.push_back(prev[k]);
						}
					}
					OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info,
									  "[Native] Recalculate only first part of the route");
					break;
				}
			}
		}
	}
	vector<SHARED_PTR<RouteSegmentResult>> results;
	for (int i = 0; i < points.size() - 1; i++) {
		RoutingContext local(ctx);
		if (i == 0) {
			if (useSmartRouteRecalculation) {
				local.previouslyCalculatedRoute = firstPartRecalculatedRoute;
			}
		}
		local.progress = ctx->progress;
		auto res = searchRouteInternalPrepare(&local, points[i], points[i + 1], routeDirection);

		results.insert(results.end(), res.begin(), res.end());

		local.unloadAllData(ctx);
		if (!restPartRecalculatedRoute.empty()) {
			results.insert(results.end(), restPartRecalculatedRoute.begin(), restPartRecalculatedRoute.end());
			break;
		}
	}
	ctx->unloadAllData();
	return results;
}

vector<SHARED_PTR<RouteSegmentResult>> RoutePlannerFrontEnd::searchRoute(
	SHARED_PTR<RoutingContext> ctx, int startX, int startY, int endX, int endY, vector<int>& intermediatesX,
	vector<int>& intermediatesY, SHARED_PTR<PrecalculatedRouteDirection> routeDirection) {
	if (!ctx->progress) {
		ctx->progress = std::make_shared<RouteCalculationProgress>();
	}
	bool intermediatesEmpty = intermediatesX.empty();
	/* TODO missing functionality for private access recalculation
	List<LatLon> targets = new ArrayList<>();
	targets.add(end);
	if (!intermediatesEmpty) {
		targets.addAll(intermediates);
	}
	if (needRequestPrivateAccessRouting(ctx, targets)) {
		ctx.calculationProgress.requestPrivateAccessRouting = true;
	}
	 */
	double maxDistance = measuredDist31(startX, startY, endX, endY);
	if (!intermediatesEmpty) {
		int x31 = startX;
		int y31 = startY;
		int ix31 = 0;
		int iy31 = 0;
		for (int i = 0; i < intermediatesX.size(); i++) {
			ix31 = intermediatesX[i];
			iy31 = intermediatesY[i];
			maxDistance = max(measuredDist31(x31, y31, ix31, iy31), maxDistance);
			x31 = ix31;
			y31 = iy31;
		}
	}
	if (ctx->calculationMode == RouteCalculationMode::COMPLEX && !routeDirection &&
		maxDistance > ctx->config->DEVIATION_RADIUS * 6) {
		SHARED_PTR<RoutingContext> nctx = buildRoutingContext(ctx->config, RouteCalculationMode::BASE);
		nctx->progress = ctx->progress;
		vector<SHARED_PTR<RouteSegmentResult>> ls =
			searchRoute(nctx, startX, startY, endX, endY, intermediatesX, intermediatesY);
		routeDirection =
			PrecalculatedRouteDirection::build(ls, ctx->config->DEVIATION_RADIUS, ctx->config->router->maxSpeed);
	}

	if (intermediatesEmpty) {
		ctx->startX = startX;
		ctx->startY = startY;
		ctx->targetX = endX;
		ctx->targetY = endY;
		SHARED_PTR<RouteSegment> recalculationEnd = getRecalculationEnd(ctx.get());
		if (recalculationEnd) {
			ctx->initTargetPoint(recalculationEnd);
		}
		if (routeDirection) {
			ctx->precalcRoute = routeDirection->adopt(ctx.get());
		}
		auto res = runRouting(ctx.get(), recalculationEnd);
		if (!res.empty()) {
			printResults(ctx.get(), startX, startY, endX, endY, res);
		}
		makeStartEndPointsPrecise(res, startX, startY, endX, endY, intermediatesX, intermediatesY);
		return res;
	}
	int indexNotFound = 0;
	vector<SHARED_PTR<RouteSegmentPoint>> points;
	if (!addSegment(startX, startY, ctx.get(), indexNotFound++, points, ctx->startTransportStop)) {
		return vector<SHARED_PTR<RouteSegmentResult>>();
	}
	if (!intermediatesX.empty()) {
		for (int i = 0; i < intermediatesX.size(); i++) {
			int x31 = intermediatesX[i];
			int y31 = intermediatesY[i];
			if (!addSegment(x31, y31, ctx.get(), indexNotFound++, points, false)) {
				return vector<SHARED_PTR<RouteSegmentResult>>();
			}
		}
	}
	if (!addSegment(endX, endY, ctx.get(), indexNotFound++, points, ctx->targetTransportStop)) {
		return vector<SHARED_PTR<RouteSegmentResult>>();
	}
	auto res = searchRoute(ctx.get(), points, routeDirection);
	// make start and end more precise
	makeStartEndPointsPrecise(res, startX, startY, endX, endY, intermediatesX, intermediatesY);
	if (!res.empty()) {
		printResults(ctx.get(), startX, startY, endX, endY, res);
	}
	return res;
}

SHARED_PTR<RouteSegmentResult> RoutePlannerFrontEnd::generateStraightLineSegment(
	float averageSpeed, std::vector<pair<double, double>> points) {
	RoutingIndex* reg = new RoutingIndex();
	reg->initRouteEncodingRule(0, "highway", "unmatched");
	SHARED_PTR<RouteDataObject> rdo = make_shared<RouteDataObject>(reg, true);
	unsigned long size = points.size();

	vector<uint32_t> x(size);
	vector<uint32_t> y(size);
	double distance = 0;
	double distOnRoadToPass = 0;
	pair<double, double> prev = {NAN, NAN};
	for (int i = 0; i < size; i++) {
		const auto& l = points[i];
		if (!isnan(l.first) && !isnan(l.second)) {
			x.push_back(get31TileNumberX(l.second));
			y.push_back(get31TileNumberY(l.first));
			if (!isnan(prev.first) && !isnan(prev.second)) {
				double d = getDistance(l.first, l.second, prev.first, prev.second);
				distance += d;
				distOnRoadToPass += d / averageSpeed;
			}
		}
		prev = l;
	}
	rdo->pointsX = x;
	rdo->pointsY = y;
	rdo->types = {0};
	rdo->id = -1;
	SHARED_PTR<RouteSegmentResult> segment = make_shared<RouteSegmentResult>(rdo, 0, rdo->getPointsLength() - 1);
	segment->segmentTime = (float)distOnRoadToPass;
	segment->segmentSpeed = (float)averageSpeed;
	segment->distance = (float)distance;
	segment->turnType = TurnType::ptrStraight();
	return segment;
}

#endif /*_OSMAND_ROUTE_PLANNER_FRONT_END_CPP*/
