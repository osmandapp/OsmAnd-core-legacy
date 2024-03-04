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
	return std::make_shared<RoutingContext>(config, rm);
}

SHARED_PTR<RouteSegmentPoint> RoutePlannerFrontEnd::getRecalculationEnd(RoutingContext* ctx) {
	SHARED_PTR<RouteSegmentPoint> recalculationEnd;
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
		
		if (rlist.size() > 0) {
			SHARED_PTR<RouteSegment> previous;
			for (int i = 0; i < rlist.size(); i++) {
				auto rr = rlist[i];
				if (previous) {
					SHARED_PTR<RouteSegment> segment =
						std::make_shared<RouteSegment>(rr->object, rr->getStartPointIndex(), rr->getEndPointIndex());
					previous->parentRoute = segment;
					previous = segment;
				} else {
					recalculationEnd = std::make_shared<RouteSegmentPoint>(rr->object, rr->getStartPointIndex(), 0);
					if (abs(rr->getEndPointIndex() - rr->getStartPointIndex()) > 1) {
						SHARED_PTR<RouteSegment> segment = std::make_shared<RouteSegment>(rr->object, recalculationEnd->segmentEnd, rr->getEndPointIndex());
						recalculationEnd->parentRoute = segment;
						previous = segment;
					} else {
						previous = recalculationEnd;
					}
				}
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
		ctx->progress->updateTotalEstimatedDistance((float)(rd / speed));
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

void RoutePlannerFrontEnd::makeStartEndPointsPrecise(vector<SHARED_PTR<RouteSegmentResult>>& res,
                                                     int startX, int startY, int endX, int endY) {
	if (res.size() > 0) {
		makeSegmentPointPrecise(res[0], startX, startY, true);
		makeSegmentPointPrecise(res[res.size() - 1], endX, endY, false);
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
		if (!RoutePlannerFrontEnd::hasSegment(result, current)) {
			auto segmentResult = std::make_shared<RouteSegmentResult>(current->road, current->getSegmentStart(), current->getSegmentEnd());
			result.push_back(segmentResult);
		}

		while (current->getParentRoute() != nullptr) {
			SHARED_PTR<RouteSegment> pr = current->getParentRoute();
			auto segmentResult =
				std::make_shared<RouteSegmentResult>(pr->road, pr->getSegmentStart(), pr->getSegmentEnd());
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

	return result;
}

bool RoutePlannerFrontEnd::hasSegment(vector<SHARED_PTR<RouteSegmentResult>>& result, SHARED_PTR<RouteSegment>& current) {
	for (SHARED_PTR<RouteSegmentResult> r : result) {
		long currentId = r->object->id;
		if (currentId == current->road->id && r->getStartPointIndex() == current->getSegmentStart() &&
			r->getEndPointIndex() == current->getSegmentEnd()) {
			return true;
		}
	}
	return false;
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
		return getDistance(getLastPoint().lat, getLastPoint().lon, lat, lon);
	}
	return 0;
}

LatLon GpxRouteApproximation::getLastPoint() {
	return result[result.size() - 1]->getEndPoint();
}

void RoutePlannerFrontEnd::searchGpxRoute(SHARED_PTR<GpxRouteApproximation> &gctx, vector<SHARED_PTR<GpxPoint>>& gpxPoints, GpxRouteApproximationCallback acceptor) {
	if (!gctx->ctx->progress) {
		gctx->ctx->progress = std::make_shared<RouteCalculationProgress>();
	}
	gctx->ctx->progress->timeToCalculate.Start();
	SHARED_PTR<GpxPoint> start;
	SHARED_PTR<GpxPoint> prev;
	if (gpxPoints.size() > 0) {
		gctx->ctx->progress->updateTotalApproximateDistance(gpxPoints[gpxPoints.size() - 1]->cumDist);
		start = gpxPoints[0];
	}
	while (start && !gctx->ctx->progress->isCancelled()) {
		double routeDist = gctx->ctx->config->maxStepApproximation;
		SHARED_PTR<GpxPoint> next = findNextGpxPointWithin(gpxPoints, start, routeDist);
		bool routeFound = false;
		bool stepBack = false;
		if (next && initRoutingPoint(start, gctx, gctx->ctx->config->minPointApproximation)) {
			while (routeDist >= gctx->ctx->config->minStepApproximation && !routeFound) {
				routeFound = initRoutingPoint(next, gctx, gctx->ctx->config->minPointApproximation);
				if (routeFound) {
					routeFound = findGpxRouteSegment(gctx, gpxPoints, start, next, prev != nullptr);
					if (routeFound) {
						routeFound = isRouteCloseToGpxPoints(gctx, gpxPoints, start, next);
						if (!routeFound) {
							start->routeToTarget.clear();
						}
					}
					if (routeFound && next->ind == gpxPoints.size() - 1) {
						// last point - last route found
						makeSegmentPointPrecise(start->routeToTarget[start->routeToTarget.size() - 1], next->lat, next->lon, false);
					} else if (routeFound) {
						// route is found - cut the end of the route and move to next iteration
						// start.stepBackRoute = new ArrayList<RouteSegmentResult>();
						// boolean stepBack = true;
						stepBack = stepBackAndFindPrevPointInRoute(gctx, gpxPoints, start, next);
						if (!stepBack) {
							// not supported case (workaround increase maxStepApproximation)
							OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info,
											  "Consider to increase maxStepApproximation to: %f", routeDist * 2);
							start->routeToTarget.clear();
							routeFound = false;
						}
					}
				}
				if (!routeFound) {
					// route is not found move next point closer to start point (distance / 2)
					routeDist = routeDist / 2;
					if (routeDist < gctx->ctx->config->minStepApproximation &&
						routeDist > gctx->ctx->config->minStepApproximation / 2 + 1) {
						routeDist = gctx->ctx->config->minStepApproximation;
					}
					next = findNextGpxPointWithin(gpxPoints, start, routeDist);
					if (next) {
						routeDist = min(next->cumDist - start->cumDist, routeDist);
					}
				}
			}
		}
		// route is not found skip segment and keep it as straight line on display
		if (!routeFound && next) {
			// route is not found, move start point by
			next = findNextGpxPointWithin(gpxPoints, start, gctx->ctx->config->minStepApproximation);
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
		if (gctx->ctx->progress && start) {
			gctx->ctx->progress->updateApproximatedDistance(start->cumDist);
		}
	}
	if (gctx->ctx->progress) {
		gctx->ctx->progress->timeToCalculate.Pause();
	}
	calculateGpxRoute(gctx, gpxPoints);
	if (!gctx->result.empty() && !gctx->ctx->progress->isCancelled()) {
//		RouteResultPreparation.printResults(gctx->ctx, gpxPoints[0]->lat, gpxPoints[0]->lon,
//											 gctx->result);
//        OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Debug, gctx.toString();
	}
	if (acceptor)
		acceptor(gctx->ctx->progress->cancelled ? nullptr : gctx);
}

void RoutePlannerFrontEnd::makeSegmentPointPrecise(SHARED_PTR<RouteSegmentResult>& routeSegmentResult, double lat, double lon, bool st) {
	int px = get31TileNumberX(lon);
	int py = get31TileNumberY(lat);
	return makeSegmentPointPrecise(routeSegmentResult, px, py, st);
}

void RoutePlannerFrontEnd::makeSegmentPointPrecise(SHARED_PTR<RouteSegmentResult>& routeSegmentResult, int px, int py, bool st) {
	int pind = st ? routeSegmentResult->getStartPointIndex() : routeSegmentResult->getEndPointIndex();

	SHARED_PTR<RouteDataObject> r = std::make_shared<RouteDataObject>(routeSegmentResult->object);
	routeSegmentResult->object = r;
	std::pair<int, int> before(-1, -1);
	std::pair<int, int> after(-1, -1);

	if (pind > 0) {
		before = std::pair<int, int>(
			getProjectionPoint(px, py, r->pointsX[pind - 1], r->pointsY[pind - 1], r->pointsX[pind], r->pointsY[pind]));
	}
	if (pind < r->pointsX.size() - 1) {
		after = std::pair<int, int>(
			getProjectionPoint(px, py, r->pointsX[pind + 1], r->pointsY[pind + 1], r->pointsX[pind], r->pointsY[pind]));
	}
	int insert = 0;
	double dd = measuredDist31(px, py, r->pointsX[pind], r->pointsY[pind]);
	double ddBefore = std::numeric_limits<double>::infinity();
	double ddAfter = std::numeric_limits<double>::infinity();
	std::pair<int, int> i;
	if (before.first != -1) {
		ddBefore = measuredDist31(px, py, before.first, before.second);
		if (ddBefore < dd) {
			insert = -1;
			i = before;
		}
	}

	if (after.first != -1) {
		ddAfter = measuredDist31(px, py, after.first, after.second);
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
			r->insert(pind + 1, i.first, i.second);
			if (st) {
				routeSegmentResult->setStartPointIndex(routeSegmentResult->getStartPointIndex() + 1);
			}
			if (!st) {
				routeSegmentResult->setEndPointIndex(routeSegmentResult->getEndPointIndex() + 1);
			}
		} else {
			r->insert(pind, i.first, i.second);
		}
	}
}

bool RoutePlannerFrontEnd::isRouteCloseToGpxPoints(SHARED_PTR<GpxRouteApproximation>& gctx, vector<SHARED_PTR<GpxPoint>>& gpxPoints,
	                         SHARED_PTR<GpxPoint>& start, SHARED_PTR<GpxPoint>& next) {
	bool routeIsClose = true;
	for (SHARED_PTR<RouteSegmentResult> r : start->routeToTarget) {
		int st = r->getStartPointIndex();
		int end = r->getEndPointIndex();
		while (st != end) {
			LatLon point = r->getPoint(st);
			bool pointIsClosed = false;
			for (int k = start->ind; !pointIsClosed && k < next->ind; k++) {
				pointIsClosed = pointCloseEnough(gctx, point, gpxPoints[k], gpxPoints[k + 1]);
			}
			if (!pointIsClosed) {
				routeIsClose = false;
				break;
			}
			st += (st < end) ? 1 : -1;
		}
	}
	return routeIsClose;
}

bool RoutePlannerFrontEnd::stepBackAndFindPrevPointInRoute(SHARED_PTR<GpxRouteApproximation> &gctx, vector<SHARED_PTR<GpxPoint>>& gpxPoints,
	                                 SHARED_PTR<GpxPoint>& start, SHARED_PTR<GpxPoint>& next) {
	// step back to find to be sure
	// 1) route point is behind GpxPoint - minPointApproximation (end route point could slightly ahead)
	// 2) we don't miss correct turn i.e. points could be attached to muliple routes
	// 3) to make sure that we perfectly connect to RoadDataObject points
	double STEP_BACK_DIST = max(gctx->ctx->config->minPointApproximation, gctx->ctx->config->minStepApproximation);
	double d = 0;
	int64_t segmendInd = start->routeToTarget.size() - 1;
	bool search = true;
	start->stepBackRoute.clear();
mainLoop:
	for (; segmendInd >= 0 && search; segmendInd--) {
		const auto& rr = start->routeToTarget[segmendInd];
		bool minus = rr->getStartPointIndex() < rr->getEndPointIndex();
		int nextInd;
		for (int j = rr->getEndPointIndex(); j != rr->getStartPointIndex(); j = nextInd) {
			nextInd = minus ? j - 1 : j + 1;
			d += getDistance(rr->getPoint(j).lat, rr->getPoint(j).lon, rr->getPoint(nextInd).lat, rr->getPoint(nextInd).lon);
			if (d > STEP_BACK_DIST) {
				if (nextInd == rr->getStartPointIndex()) {
					segmendInd--;
				} else {
					start->stepBackRoute.push_back(std::make_shared<RouteSegmentResult>(rr->object, nextInd, rr->getEndPointIndex()));
					rr->setEndPointIndex(nextInd);
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
	start->routeToTarget.shrink_to_fit();
	SHARED_PTR<RouteSegmentResult>& res = start->routeToTarget[segmendInd];
	next->pnt = std::make_shared<RouteSegmentPoint>(res->object, res->getEndPointIndex(), 0);
	// OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "[Native] STEP BACK: %.5f %.5f	",
	//   					next->pnt->getPreciseLatLon().lat, next->pnt->getPreciseLatLon().lon);
	return true;
}

void RoutePlannerFrontEnd::calculateGpxRoute(SHARED_PTR<GpxRouteApproximation>& gctx, vector<SHARED_PTR<GpxPoint>>& gpxPoints) {
	auto reg = std::make_shared<RoutingIndex>();
	reg->initRouteEncodingRule(0, "highway", UNMATCHED_HIGHWAY_TYPE);
	vector<LatLon> lastStraightLine;
	SHARED_PTR<GpxPoint> straightPointStart;
	for (int i = 0; i < gpxPoints.size() && !gctx->ctx->progress->isCancelled();) {
		SHARED_PTR<GpxPoint>& pnt = gpxPoints[i];
		if (!pnt->routeToTarget.empty()) {
			LatLon startPoint = LatLon(pnt->routeToTarget[0]->getStartPoint().lat, pnt->routeToTarget[0]->getStartPoint().lon);
			if (!lastStraightLine.empty()) {
				lastStraightLine.push_back(startPoint);
				addStraightLine(gctx, lastStraightLine, straightPointStart, reg);
				lastStraightLine.clear();
			}
			if (gctx->distFromLastPoint(startPoint.lat, startPoint.lon) > 1) {
				gctx->routeGapDistance += gctx->distFromLastPoint(startPoint.lat, startPoint.lon);
				OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info,
								  "????? gap of route point = %f lat = %f lon = %f, gap of actual gpxPoint = %f, lat = %f lon = %f ",
								  gctx->distFromLastPoint(startPoint.lat, startPoint.lon), startPoint.lat, startPoint.lon,
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
			lastStraightLine.push_back(LatLon(pnt->lat, pnt->lon));
			i++;
		}
	}
	if (!lastStraightLine.empty()) {
		addStraightLine(gctx, lastStraightLine, straightPointStart, reg);
	}
	// clean turns to recaculate them
	cleanupResultAndAddTurns(gctx);
}

void RoutePlannerFrontEnd::addStraightLine(const SHARED_PTR<GpxRouteApproximation>& gctx, vector<LatLon>& lastStraightLine, const SHARED_PTR<GpxPoint>& strPnt,
                                           const SHARED_PTR<RoutingIndex>& reg) {
	SHARED_PTR<RouteDataObject> rdo = std::make_shared<RouteDataObject>(reg);
	if (gctx->ctx->config->smoothenPointsNoRoute > 0) {
		std::vector<bool> include(lastStraightLine.size(), true);
		simplifyDouglasPeucker(lastStraightLine, gctx->ctx->config->smoothenPointsNoRoute, 0, (int) lastStraightLine.size() - 1, include);
		vector<LatLon> simplifiedLine;
		for (int i = 0; i < include.size(); i++) {
			if (include[i]) {
				simplifiedLine.push_back(lastStraightLine[i]);
			}
		}
		lastStraightLine = simplifiedLine;
	}
	uint64_t s = lastStraightLine.size();
	vector<uint32_t> x;
	vector<uint32_t> y;
	for (int i = 0; i < s; i++) {
		LatLon l = lastStraightLine[i];
		uint64_t t = x.size() - 1;
		x.push_back(get31TileNumberX(l.lon));
		y.push_back(get31TileNumberY(l.lat));
		if (t >= 0) {
			double dist = squareRootDist31(x[t], y[t], x[t + 1], y[t + 1]);
			gctx->routeDistanceUnmatched += dist;
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

void RoutePlannerFrontEnd::cleanupResultAndAddTurns(SHARED_PTR<GpxRouteApproximation>& gctx) {
	// cleanup double joints
	int LOOK_AHEAD = 4;
	for (int i = 0; i < gctx->result.size() && !gctx->ctx->progress->isCancelled(); i++) {
		SHARED_PTR<RouteSegmentResult>& s = gctx->result[i];
		for (int j = i + 2; j <= i + LOOK_AHEAD && j < gctx->result.size(); j++) {
			SHARED_PTR<RouteSegmentResult>& e = gctx->result[j];
			if (e->getStartPoint().isEquals(s->getEndPoint())) {
				while ((--j) != i) {
					gctx->result.erase(gctx->result.begin() + j);
				}
				break;
			}
		}
	}
	gctx->result.shrink_to_fit();
	for (SHARED_PTR<RouteSegmentResult> r : gctx->result) {
		r->turnType = nullptr;
		r->description = "";
	}
	if (!gctx->ctx->progress->isCancelled()) {
		prepareTurnResults(gctx->ctx, gctx->result);
	}
	for (SHARED_PTR<RouteSegmentResult> r : gctx->result) {
		r->attachedRoutes.clear();
		r->preAttachedRoutes.clear();
	}
}

void RoutePlannerFrontEnd::simplifyDouglasPeucker(vector<LatLon>& l, double eps, int start, int end, std::vector<bool>& include) {
	double dmax = -1;
	int index = -1;
	LatLon s = l[start];
	LatLon e = l[end];
	for (int i = start + 1; i <= end - 1; i++) {
		LatLon ip = l[i];
		double dist = getOrthogonalDistance(ip.lat, ip.lon, s.lat, s.lon, e.lat, e.lon);
		if (dist > dmax) {
			dmax = dist;
			index = i;
		}
	}
	if (dmax >= eps) {
		simplifyDouglasPeucker(l, eps, start, index, include);
		simplifyDouglasPeucker(l, eps, index, end, include);
	} else {
		for (int i = start + 1; i < end; i++) {
			include[i] = false;
		}
	}
}

bool RoutePlannerFrontEnd::initRoutingPoint(SHARED_PTR<GpxPoint>& start, SHARED_PTR<GpxRouteApproximation>& gctx, double distThreshold) {
	if (start && !start->pnt) {
		gctx->routePointsSearched++;
		SHARED_PTR<RouteSegmentPoint> rsp =
			findRouteSegment(get31TileNumberX(start->lon), get31TileNumberY(start->lat), gctx->ctx);
		if (rsp) {
			LatLon point = rsp->getPreciseLatLon();
			if (getDistance(point.lat, point.lon, start->lat, start->lon) < distThreshold) {
				start->pnt = rsp;
			}
		}
	}
	if (start && start->pnt) {
		return true;
	}
	return false;
}

SHARED_PTR<GpxPoint> RoutePlannerFrontEnd::findNextGpxPointWithin(vector<SHARED_PTR<GpxPoint>>& gpxPoints, SHARED_PTR<GpxPoint>& start,
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

bool RoutePlannerFrontEnd::findGpxRouteSegment(SHARED_PTR<GpxRouteApproximation>& gctx, vector<SHARED_PTR<GpxPoint>>& gpxPoints,
						 SHARED_PTR<GpxPoint>& start, SHARED_PTR<GpxPoint>& target, bool prevRouteCalculated) {
	vector<SHARED_PTR<RouteSegmentResult>> res;
	bool routeIsCorrect = false;
	if (start->pnt && target->pnt) {
		start->pnt = std::make_shared<RouteSegmentPoint>(start->pnt);
		target->pnt = std::make_shared<RouteSegmentPoint>(target->pnt);
		// OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "[Native] Calculate: %.5f %.5f -> %.5f %.5f",
		// 				  start->pnt->getPreciseLatLon().lat, start->pnt->getPreciseLatLon().lon,
		// 				  target->pnt->getPreciseLatLon().lat, target->pnt->getPreciseLatLon().lon);
		gctx->routeDistCalculations += (target->cumDist - start->cumDist);
		gctx->routeCalculations++;
		RoutingContext* cp = new RoutingContext(gctx->ctx);
		res = searchRouteInternalPrepare(cp, start->pnt, target->pnt, nullptr);
		delete cp;
		// BinaryRoutePlanner.printDebugMemoryInformation(gctx.ctx);
		routeIsCorrect = !res.empty();
		for (int k = start->ind + 1; routeIsCorrect && k < target->ind; k++) {
			SHARED_PTR<GpxPoint>& ipoint = gpxPoints[k];
			if (!pointCloseEnough(gctx, ipoint, res)) {
				routeIsCorrect = false;
			}
		}
		if (routeIsCorrect) {
			// correct start point though don't change end point
			// makeSegmentPointPrecise(res[0], start->lat, start->lon, true);
			if (!prevRouteCalculated) {
			 	// make first position precise
			 	makeSegmentPointPrecise(res[0], start->lat, start->lon, true);
			} else {
				if (res[0]->object->getId() == start->pnt->getRoad()->getId()) {
					// start point could shift to +-1 due to direction
					res[0]->setStartPointIndex(start->pnt->getSegmentStart());
					if (res[0]->object->getPointsLength() != start->pnt->getRoad()->getPointsLength()) {
						res[0]->object = start->pnt->getRoad();
					}
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
			// OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "[Native] Calculate res: %.5f %.5f -> %.5f %.5f",
			// 				  res[0]->getStartPoint().lat, res[0]->getStartPoint().lon,
			// 				  res[res.size() - 1]->getEndPoint().lat, res[res.size() - 1]->getEndPoint().lon);
		}
	}
	return routeIsCorrect;
}

bool RoutePlannerFrontEnd::pointCloseEnough(SHARED_PTR<GpxRouteApproximation>& gctx, LatLon point,
                                            SHARED_PTR<GpxPoint>& gpxPoint, SHARED_PTR<GpxPoint>& gpxPointNext) {
	LatLon gpxPointLL(gpxPoint->pnt ? gpxPoint->pnt->getPreciseLatLon().lat : gpxPoint->lat,
					  gpxPoint->pnt ? gpxPoint->pnt->getPreciseLatLon().lon : gpxPoint->lon);
	LatLon gpxPointNextLL(gpxPointNext->pnt ? gpxPointNext->pnt->getPreciseLatLon().lat : gpxPointNext->lat,
						  gpxPointNext->pnt ? gpxPointNext->pnt->getPreciseLatLon().lon : gpxPointNext->lon);
	std::pair<double, double> projection =
		getProjection(point.lat, point.lon, gpxPointLL.lat, gpxPointLL.lon, gpxPointNextLL.lat, gpxPointNextLL.lon);
	return getDistance(projection.first, projection.second, point.lat, point.lon) <= gctx->ctx->config->minPointApproximation;
}

bool RoutePlannerFrontEnd::pointCloseEnough(SHARED_PTR<GpxRouteApproximation>& gctx, SHARED_PTR<GpxPoint>& ipoint,
					  vector<SHARED_PTR<RouteSegmentResult>>& res) {
	int px = get31TileNumberX(ipoint->lon);
	int py = get31TileNumberY(ipoint->lat);
	double SQR = gctx->ctx->config->minPointApproximation;
	SQR *= SQR;
	for (SHARED_PTR<RouteSegmentResult>& sr : res) {
		int start = sr->getStartPointIndex();
		int end = sr->getEndPointIndex();
		if (sr->getStartPointIndex() > sr->getEndPointIndex()) {
			start = sr->getEndPointIndex();
			end = sr->getStartPointIndex();
		}
		for (int i = start; i < end; i++) {
			const auto& r = sr->object;
			std::pair<int, int> pp =
				getProjectionPoint(px, py, r->pointsX[i], r->pointsY[i], r->pointsX[i + 1], r->pointsY[i + 1]);
			double currentsDist = squareDist31TileMetric(pp.first, pp.second, px, py);
			if (currentsDist <= SQR) {
				return true;
			}
		}
	}
	return false;
}

// BRP-ios main function with interpoints support (called by entry-point function)
vector<SHARED_PTR<RouteSegmentResult>> RoutePlannerFrontEnd::searchRoute(
	RoutingContext* ctx, vector<SHARED_PTR<RouteSegmentPoint>>& points,
	SHARED_PTR<PrecalculatedRouteDirection> routeDirection) {
	if (points.size() <= 2) {
		if (!useSmartRouteRecalculation) {
			ctx->previouslyCalculatedRoute.clear();
		}
		auto res = searchRouteInternalPrepare(ctx, points[0], points[1], routeDirection); // BRP-ios (no-interpoints)
		makeStartEndPointsPrecise(res, ctx->startX, ctx->startY, ctx->targetX, ctx->targetY);
		return res;
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

		auto res = searchRouteInternalPrepare(&local, points[i], points[i + 1], routeDirection); // BRP-ios (interpoints)
		makeStartEndPointsPrecise(res, local.startX, local.startY, local.targetX, local.targetY);
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

// HH-ios BRP-ios entry point
vector<SHARED_PTR<RouteSegmentResult>> RoutePlannerFrontEnd::searchRoute(
	SHARED_PTR<RoutingContext> ctx, int startX, int startY, int endX, int endY, vector<int>& intermediatesX,
	vector<int>& intermediatesY, SHARED_PTR<PrecalculatedRouteDirection> routeDirection) {
	if (!ctx->progress) {
		ctx->progress = std::make_shared<RouteCalculationProgress>();
	}
	vector<int> targetsX;
	vector<int> targetsY;
	bool intermediatesEmpty = intermediatesX.empty();
	if (!intermediatesEmpty) {
		targetsX.insert(targetsX.end(), intermediatesX.begin(), intermediatesX.end());
		targetsY.insert(targetsY.end(), intermediatesY.begin(), intermediatesY.end());
	}
	targetsX.push_back(endX);
	targetsY.push_back(endY);
	if (needRequestPrivateAccessRouting(ctx.get(), targetsX, targetsY)) {
		ctx->progress->requestPrivateAccessRouting = true;
	}
	if (HH_ROUTING_CONFIG != nullptr) {
		HHRoutePlanner routePlanner(ctx.get());
		HHNetworkRouteRes * res;
		HHNetworkRouteRes * r = nullptr;
		double dir = ctx->config->initialDirection ;
		for (int i = 0; i < targetsX.size(); i++) {
			double initialPenalty = ctx->config->penaltyForReverseDirection;
			if (i > 0) {
				ctx->config->penaltyForReverseDirection /= 2; // relax reverse-penalty (only for inter-points)
			}
			ctx->progress->hhTargetsProgress(i, targetsX.size());
			res = calculateHHRoute(routePlanner, ctx.get(), i == 0 ? startX : targetsX.at(i - 1),
									i == 0 ? startY : targetsY.at(i - 1),
									targetsX.at(i), targetsY.at(i), dir);
			ctx->config->penaltyForReverseDirection = initialPenalty;
			if (!r) {
				r = res;
			} else {
				r->append(res);
			}
			if (!r || !r->isCorrect()) {
				break;
			}
			if (r->detailed.size() > 0) {
				dir = (r->detailed[r->detailed.size() - 1]->getBearingEnd() / 180.0) * M_PI;
			}
		}
		if (r && (r->isCorrect() || USE_ONLY_HH_ROUTING)) { // note: USE_ONLY_HH_ROUTING is broken here
			prepareResult(ctx.get(), r->detailed);
			return r->detailed; // exit-point
		}
	}
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
			searchRoute(nctx, startX, startY, endX, endY, intermediatesX, intermediatesY); // iOS (interpoints) 2-phase
		routeDirection =
			PrecalculatedRouteDirection::build(ls, ctx->config->DEVIATION_RADIUS, ctx->config->router->maxSpeed);
		ctx->calculationProgressFirstPhase =  ctx->progress->capture(ctx->progress);
	}

	if (intermediatesEmpty) {
		ctx->startX = startX;
		ctx->startY = startY;
		ctx->targetX = endX;
		ctx->targetY = endY;
		SHARED_PTR<RouteSegmentPoint> recalculationEnd = getRecalculationEnd(ctx.get());
		if (recalculationEnd) {
			ctx->initTargetPoint(recalculationEnd);
		}
		if (routeDirection) {
			ctx->precalcRoute = routeDirection->adopt(ctx.get());
		}
		auto res = runRouting(ctx.get(), recalculationEnd); // iOS (no-interpoints)
		makeStartEndPointsPrecise(res, startX, startY, endX, endY);
		prepareResult(ctx.get(), res);
		if (!res.empty()) {
			printResults(ctx.get(), startX, startY, endX, endY, res);
		}
		return res; // exit-point
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
	auto res = searchRoute(ctx.get(), points, routeDirection); // iOS (interpoints)
	prepareResult(ctx.get(), res); // res is already precise after searchRoute
	printResults(ctx.get(), startX, startY, endX, endY, res);
	return res; // exit-point
}

bool RoutePlannerFrontEnd::needRequestPrivateAccessRouting(RoutingContext* ctx, vector<int>& targetsX,
														   vector<int>& targetsY) {
	bool res = false;
	SHARED_PTR<GeneralRouter> router = ctx->config->router;
	if (router && !router->allowPrivate) {
		ctx->unloadAllData();
		UNORDERED(map)<string, string> params({{GeneralRouterConstants::ALLOW_PRIVATE, "true"}, 
		                                       {GeneralRouterConstants::CHECK_ALLOW_PRIVATE_NEEDED, "true"}});
		auto generalRouter = GeneralRouter(parseGeneralRouterProfile(ctx->config->routerName, GeneralRouterProfile::CAR), params);
		ctx->config->router = generalRouter.build();
		for (int i = 0; i < targetsX.size(); i++) {
			int x31 = targetsX[i];
			int y31 = targetsY[i];
			SHARED_PTR<RouteSegmentPoint> rp = findRouteSegment(x31, y31, ctx);
			if (rp && rp->road) {
				if (rp->road->hasPrivateAccess(ctx->config->router->getProfile())) {
					res = true;
					break;
				}
			}
		}
		ctx->unloadAllData();
		ctx->progress->requestPrivateAccessRouting = res;
		ctx->config->router = router;
	}
	return res;
}

SHARED_PTR<RouteSegmentResult> RoutePlannerFrontEnd::generateStraightLineSegment(
	float averageSpeed, std::vector<pair<double, double>> points) {
	auto reg = std::make_shared<RoutingIndex>();
	reg->initRouteEncodingRule(0, "highway", "unmatched");
	auto rdo = make_shared<RouteDataObject>(reg);
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

std::vector<SHARED_PTR<GpxPoint>> RoutePlannerFrontEnd::generateGpxPoints(SHARED_PTR<GpxRouteApproximation>& gctx, const std::vector<std::pair<double, double>>& locationsHolder)
{
	vector<SHARED_PTR<GpxPoint>> gpxPoints(locationsHolder.size());
	SHARED_PTR<GpxPoint> prev = nullptr;
	for(int i = 0; i < locationsHolder.size(); i++)
	{
		const auto loc = locationsHolder[i];
		auto p = make_shared<GpxPoint>(i, loc.first, loc.second, prev != nullptr ? getDistance(loc.first, loc.second, prev->lat, prev->lon) + prev->cumDist : 0);
		gpxPoints[i] = p;
		gctx->routeDistance = (int) p->cumDist;
		prev = p;
	}
	return gpxPoints;
}

HHNetworkRouteRes * RoutePlannerFrontEnd::calculateHHRoute(HHRoutePlanner & routePlanner, RoutingContext* ctx, int startX, int startY, int endX, int endY, double dir)
{
	try {
		auto cfg = routePlanner.prepareDefaultRoutingConfig(HH_ROUTING_CONFIG);
		cfg->INITIAL_DIRECTION = dir;
		HHNetworkRouteRes * res = routePlanner.runRouting(startX, startY, endX, endY, cfg); // HH-cpp
		if (res != nullptr && res->error == "") {
			ctx->progress->hhIteration(RouteCalculationProgress::HHIteration::DONE);
			makeStartEndPointsPrecise(res->detailed, startX, startY, endX, endY);
			return res;
		}
		ctx->progress->hhIteration(RouteCalculationProgress::HHIteration::HH_NOT_STARTED);
	} catch (const std::exception e) {
		OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "%s", e.what());
		if (USE_ONLY_HH_ROUTING) {
			std::string ex = "Error during routing calculation : ";
			ex += e.what();
			auto * res = new HHNetworkRouteRes(ex);
			return res;
		}
	}
	return nullptr;
}

HHRoutingConfig * RoutePlannerFrontEnd::setDefaultRoutingConfig() {
	if (HH_ROUTING_CONFIG != nullptr) {
		delete HH_ROUTING_CONFIG;
	}
	HH_ROUTING_CONFIG = HHRoutingConfig::astar(0);
	HH_ROUTING_CONFIG->calcDetailed(HHRoutingConfig::CALCULATE_ALL_DETAILED);
	return HH_ROUTING_CONFIG;
}

// HH-cpp JNI entry point
vector<SHARED_PTR<RouteSegmentResult>> RoutePlannerFrontEnd::searchHHRoute(RoutingContext * ctx) {
	if (HH_ROUTING_CONFIG != nullptr) {
		if (!ctx->progress) {
			ctx->progress = std::make_shared<RouteCalculationProgress>();
		}
		vector<int> targetsX;
		vector<int> targetsY;
		bool intermediatesEmpty = ctx->intermediatesX.empty();
		if (!intermediatesEmpty) {
			targetsX.insert(targetsX.end(), ctx->intermediatesX.begin(), ctx->intermediatesX.end());
			targetsY.insert(targetsY.end(), ctx->intermediatesY.begin(), ctx->intermediatesY.end());
		}
		targetsX.push_back(ctx->targetX);
		targetsY.push_back(ctx->targetY);
		HHRoutePlanner routePlanner(ctx);
		HHNetworkRouteRes * r = nullptr;
		double dir = ctx->config->initialDirection ;
		for (int i = 0; i < targetsX.size(); i++) {
			double initialPenalty = ctx->config->penaltyForReverseDirection;
			if (i > 0) {
				ctx->config->penaltyForReverseDirection /= 2; // relax reverse-penalty (only for inter-points)
			}
			ctx->progress->hhTargetsProgress(i, (int)targetsX.size());
			HHNetworkRouteRes * res = calculateHHRoute(routePlanner, ctx, i == 0 ? ctx->startX : targetsX.at(i - 1),
								i == 0 ? ctx->startY : targetsY.at(i - 1),
								targetsX.at(i), targetsY.at(i), dir);
			ctx->config->penaltyForReverseDirection = initialPenalty;
			if (!r) {
				r = res;
			} else {
				r->append(res);
			}
			if (!r || !r->isCorrect()) {
				break;
			}
			if (r->detailed.size() > 0) {
				dir = (r->detailed[r->detailed.size() - 1]->getBearingEnd() / 180.0) * M_PI;
			}
		}
		if ((r && r->isCorrect()) || USE_ONLY_HH_ROUTING) {
			OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "Finish searchHHRoute Native");
			return r->detailed; // prepareResult() will be called in Java
		}
	}
	return {};
}

#endif /*_OSMAND_ROUTE_PLANNER_FRONT_END_CPP*/
