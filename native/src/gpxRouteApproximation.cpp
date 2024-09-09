#include "gpxRouteApproximation.h"

#include "routeResultPreparation.h"
#include "routePlannerFrontEnd.h"
#include "binaryRoutePlanner.h"
#include "routeCalculationProgress.h"

GpxRouteApproximation::GpxRouteApproximation(RoutingContext* rctx) {
	ctx = rctx;
}

GpxRouteApproximation::GpxRouteApproximation(const GpxRouteApproximation& gctx) {
	ctx = gctx.ctx;
	router = gctx.router;
	// routeDistance = gctx.routeDistance;
}

// setRouter() must be called by router before approximation
// both constructors are saved "as is" for iOS compatibility
void GpxRouteApproximation::setRouter(RoutePlannerFrontEnd* router) {
	this->router = router;
}

void GpxRouteApproximation::searchGpxRouteByRouting(SHARED_PTR<GpxRouteApproximation>& gctx,
                                                    vector<SHARED_PTR<GpxPoint>>& gpxPoints) {
	gctx->ctx->progress->timeToCalculate.Start();
	SHARED_PTR<GpxPoint> start;
	SHARED_PTR<GpxPoint> prev;
	if (gpxPoints.size() > 0) {
		gctx->ctx->progress->updateTotalApproximateDistance(gpxPoints[gpxPoints.size() - 1]->cumDist);
		start = gpxPoints[0];
	}
	float minPointApproximation = gctx->ctx->config->minPointApproximation;
	while (start && !gctx->ctx->progress->isCancelled()) {
		double routeDist = gctx->ctx->config->maxStepApproximation;
		SHARED_PTR<GpxPoint> next = findNextGpxPointWithin(gpxPoints, start, routeDist);
		bool routeFound = false;
		if (next && initRoutingPoint(start, gctx, minPointApproximation)) {
			while (routeDist >= gctx->ctx->config->minStepApproximation && !routeFound) {
				routeFound = initRoutingPoint(next, gctx, minPointApproximation);
				if (routeFound) {
					routeFound = findGpxRouteSegment(gctx, gpxPoints, start, next, prev != nullptr);
					if (routeFound) {
						routeFound = isRouteCloseToGpxPoints(minPointApproximation, gpxPoints, start, next);
						if (!routeFound) {
							start->routeToTarget.clear();
						}
					}
					if (routeFound && next->ind < gpxPoints.size() - 1) {
						// route is found - cut the end of the route and move to next iteration
						// start.stepBackRoute = new ArrayList<RouteSegmentResult>();
						// boolean stepBack = true;
						bool stepBack = stepBackAndFindPrevPointInRoute(gctx, gpxPoints, start, next);
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
//	if (!gctx->fullRoute.empty() && !gctx->ctx->progress->isCancelled()) {
//		RouteResultPreparation.printResults(gctx->ctx, gpxPoints[0]->lat, gpxPoints[0]->lon,
//											 gctx->fullRoute);
//        OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Debug, gctx.toString();
//	}
}

bool GpxRouteApproximation::initRoutingPoint(SHARED_PTR<GpxPoint>& start, SHARED_PTR<GpxRouteApproximation>& gctx, double distThreshold) {
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

SHARED_PTR<GpxPoint> GpxRouteApproximation::findNextGpxPointWithin(vector<SHARED_PTR<GpxPoint>>& gpxPoints, SHARED_PTR<GpxPoint>& start,
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

bool GpxRouteApproximation::findGpxRouteSegment(SHARED_PTR<GpxRouteApproximation>& gctx, vector<SHARED_PTR<GpxPoint>>& gpxPoints,
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
		res = router->searchRouteInternalPrepare(cp, start->pnt, target->pnt, nullptr);
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
			if (prevRouteCalculated) {
				if (res[0]->object->getId() == start->pnt->getRoad()->getId()) {
					// start point could shift to +-1 due to direction
					res[0]->setStartPointIndex(start->pnt->getSegmentEnd());
					if (res[0]->object->getPointsLength() != start->pnt->getRoad()->getPointsLength()) {
						res[0]->object = start->pnt->getRoad();
					}
					if (res[0]->getStartPointIndex() == res[0]->getEndPointIndex()) {
						res.erase(res.begin());
					}
				} else {
					// for native routing this is possible when point lies on intersection of 2 lines
					// solution here could be to pass to native routing id of the route
					// though it should not create any issue
					// System.out.println("??? not found " + start.pnt.getRoad().getId() + " instead "
					// + res.get(0).getObject().getId());
				}
			}
			for (SHARED_PTR<RouteSegmentResult>& seg : res) {
				seg->setGpxPointIndex(start->ind);
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


double GpxRouteApproximation::distFromLastPoint(double lat, double lon) {
	if (fullRoute.size() > 0) {
		return getDistance(getLastPoint().lat, getLastPoint().lon, lat, lon);
	}
	return 0;
}

LatLon GpxRouteApproximation::getLastPoint() {
	if (fullRoute.size() > 0) {
		return fullRoute[fullRoute.size() - 1]->getEndPoint();
	}
	throw std::out_of_range("getLastPoint(): fullRoute is empty");
}

void GpxRouteApproximation::reconstructFinalPointsFromFullRoute() {
	// create gpx-to-final index map, clear routeToTarget(s)
	UNORDERED(map)<int, int> gpxIndexFinalIndex;
	for (int i = 0; i < finalPoints.size(); i++) {
		gpxIndexFinalIndex.insert(std::make_pair(finalPoints.at(i)->ind, i));
		finalPoints.at(i)->routeToTarget.clear();
	}

	// reconstruct routeToTarget from scratch
	int lastIndex = 0;
	for (const auto& seg : fullRoute) {
		int index = seg->getGpxPointIndex();
		if (index == -1) {
			index = lastIndex;
		} else {
			lastIndex = index;
		}
		finalPoints.at(gpxIndexFinalIndex.at(index))->routeToTarget.push_back(seg);
	}

	// finally remove finalPoints with empty route
	vector <SHARED_PTR<GpxPoint>> emptyFinalPoints;
	for (const auto& gpx : this->finalPoints) {
		const auto& route = gpx->routeToTarget;
		if (route.empty()) {
			emptyFinalPoints.push_back(gpx);
		}
	}
	if (emptyFinalPoints.size() > 0) {
		auto& fp = this->finalPoints; // modify
		fp.erase(std::remove_if(fp.begin(), fp.end(),
			[emptyFinalPoints](SHARED_PTR<GpxPoint> pnt) {
				for (const auto& del : emptyFinalPoints) {
					if (del == pnt) {
						return true;
					}
				}
				return false;
			}
		), fp.end());
	}
}

bool GpxRouteApproximation::pointCloseEnough(float minPointApproximation, LatLon point,
                                            SHARED_PTR<GpxPoint>& gpxPoint, SHARED_PTR<GpxPoint>& gpxPointNext) {
	LatLon gpxPointLL(gpxPoint->pnt ? gpxPoint->pnt->getPreciseLatLon().lat : gpxPoint->lat,
					  gpxPoint->pnt ? gpxPoint->pnt->getPreciseLatLon().lon : gpxPoint->lon);
	LatLon gpxPointNextLL(gpxPointNext->pnt ? gpxPointNext->pnt->getPreciseLatLon().lat : gpxPointNext->lat,
						  gpxPointNext->pnt ? gpxPointNext->pnt->getPreciseLatLon().lon : gpxPointNext->lon);
	std::pair<double, double> projection =
		getProjection(point.lat, point.lon, gpxPointLL.lat, gpxPointLL.lon, gpxPointNextLL.lat, gpxPointNextLL.lon);
	return getDistance(projection.first, projection.second, point.lat, point.lon) <= minPointApproximation;
}

bool GpxRouteApproximation::pointCloseEnough(SHARED_PTR<GpxRouteApproximation>& gctx, SHARED_PTR<GpxPoint>& ipoint,
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

bool GpxRouteApproximation::isRouteCloseToGpxPoints(float minPointApproximation, vector<SHARED_PTR<GpxPoint>>& gpxPoints,
	                         SHARED_PTR<GpxPoint>& start, SHARED_PTR<GpxPoint>& next) {
	bool routeIsClose = true;
	for (SHARED_PTR<RouteSegmentResult> r : start->routeToTarget) {
		int st = r->getStartPointIndex();
		int end = r->getEndPointIndex();
		while (st != end) {
			LatLon point = r->getPoint(st);
			bool pointIsClosed = false;
			int delta = 5;
			int startInd = std::max(0, start->ind - delta);
			int nextInd = std::min((int)gpxPoints.size() - 1, next->ind + delta);
			for (int k = startInd; !pointIsClosed && k < nextInd; k++) {
				pointIsClosed = pointCloseEnough(minPointApproximation, point, gpxPoints[k], gpxPoints[k + 1]);
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

bool GpxRouteApproximation::stepBackAndFindPrevPointInRoute(SHARED_PTR<GpxRouteApproximation> &gctx, vector<SHARED_PTR<GpxPoint>>& gpxPoints,
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
					SHARED_PTR<RouteSegmentResult> seg = std::make_shared<RouteSegmentResult>(rr->object, nextInd, rr->getEndPointIndex());
					seg->setGpxPointIndex(start->ind);
					start->stepBackRoute.push_back(seg);
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

	int end = res->getEndPointIndex();
	int beforeEnd = res->isForwardDirection() ? end - 1 : end + 1;
	next->pnt = std::make_shared<RouteSegmentPoint>(res->object, beforeEnd, end, 0);
	next->pnt->preciseX = next->pnt->getEndPointX();
	next->pnt->preciseY = next->pnt->getEndPointY();

	// OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "[Native] STEP BACK: %.5f %.5f	",
	//   					next->pnt->getPreciseLatLon().lat, next->pnt->getPreciseLatLon().lon);
	return true;
}

void GpxRouteApproximation::calculateGpxRoute(SHARED_PTR<GpxRouteApproximation>& gctx, vector<SHARED_PTR<GpxPoint>>& gpxPoints) {
	auto reg = std::make_shared<RoutingIndex>();
	reg->initRouteEncodingRule(0, "highway", UNMATCHED_HIGHWAY_TYPE);
	vector<LatLon> lastStraightLine;
	SHARED_PTR<GpxPoint> straightPointStart;
	for (int i = 0; i < gpxPoints.size() && !gctx->ctx->progress->isCancelled();) {
		SHARED_PTR<GpxPoint>& pnt = gpxPoints[i];
		if (!pnt->routeToTarget.empty()) {
			SHARED_PTR<RouteSegmentResult> firstRouteRes = pnt->getFirstRouteRes();
			LatLon startPoint = firstRouteRes->getStartPoint();
			if (!lastStraightLine.empty()) {
				router->makeSegmentPointPrecise(gctx->ctx, firstRouteRes, pnt->lat, pnt->lon, true);
				startPoint = firstRouteRes->getStartPoint();
				lastStraightLine.push_back(startPoint);
				addStraightLine(gctx, lastStraightLine, straightPointStart, reg);
				lastStraightLine.clear();
			}
			if (gctx->distFromLastPoint(startPoint.lat, startPoint.lon) > 1) {
				// gctx->routeGapDistance += gctx->distFromLastPoint(startPoint.lat, startPoint.lon);
				OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info,
								  "????? gap of route point = %f lat = %f lon = %f, gap of actual gpxPoint = %f, lat = %f lon = %f ",
								  gctx->distFromLastPoint(startPoint.lat, startPoint.lon), startPoint.lat, startPoint.lon,
								  gctx->distFromLastPoint(pnt->lat, pnt->lon), pnt->lat, pnt->lon);
			}
			gctx->finalPoints.push_back(pnt);
			gctx->fullRoute.insert(gctx->fullRoute.end(), pnt->routeToTarget.begin(), pnt->routeToTarget.end());
			i = pnt->targetInd;
		} else {
			// add straight line from i -> i+1
			if (lastStraightLine.empty()) {
				if (gctx->fullRoute.size() > 0 && gctx->finalPoints.size() > 0) {
					SHARED_PTR<GpxPoint>& prev = gctx->finalPoints.at(gctx->finalPoints.size() - 1);
					SHARED_PTR<RouteSegmentResult> lastRouteRes = prev->getLastRouteRes();
					if (lastRouteRes) {
						router->makeSegmentPointPrecise(gctx->ctx, lastRouteRes, prev->lat, prev->lon, false);
					}
					lastStraightLine.push_back(gctx->getLastPoint());
				}
				straightPointStart = pnt;
			}
			lastStraightLine.push_back(LatLon(pnt->lat, pnt->lon));
			i++;
		}
	}
	if (!lastStraightLine.empty()) {
		addStraightLine(gctx, lastStraightLine, straightPointStart, reg);
	}

	if (router->useGeometryBasedApproximation) {
		prepareResult(gctx->ctx, gctx->fullRoute); // not required by classic method
	}

	// clean turns to recaculate them
	cleanupResultAndAddTurns(gctx);
}

void GpxRouteApproximation::addStraightLine(const SHARED_PTR<GpxRouteApproximation>& gctx, vector<LatLon>& lastStraightLine, const SHARED_PTR<GpxPoint>& strPnt,
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
			// gctx->routeDistanceUnmatched += dist;
		}
	}
	rdo->pointsX = x;
	rdo->pointsY = y;
	rdo->types = {0};
	rdo->id = -1;
	strPnt->routeToTarget.clear();
	strPnt->straightLine = true;
	SHARED_PTR<RouteSegmentResult> rsr = std::make_shared<RouteSegmentResult>(rdo, 0, rdo->getPointsLength() - 1);
	rsr->setGpxPointIndex(strPnt->ind);
	strPnt->routeToTarget.push_back(rsr);

	prepareResult(gctx->ctx, strPnt->routeToTarget);

	// VIEW: comment to see road without straight connections
	gctx->finalPoints.push_back(strPnt);
	gctx->fullRoute.insert(gctx->fullRoute.end(), strPnt->routeToTarget.begin(), strPnt->routeToTarget.end());
}

void GpxRouteApproximation::simplifyDouglasPeucker(vector<LatLon>& l, double eps, int start, int end, std::vector<bool>& include) {
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

void GpxRouteApproximation::cleanupResultAndAddTurns(SHARED_PTR<GpxRouteApproximation>& gctx) {
	// cleanup double joints
	int LOOK_AHEAD = 4;
	for (int i = 0; i < gctx->fullRoute.size() && !gctx->ctx->progress->isCancelled(); i++) {
		SHARED_PTR<RouteSegmentResult>& s = gctx->fullRoute[i];
		for (int j = i + 2; j <= i + LOOK_AHEAD && j < gctx->fullRoute.size(); j++) {
			SHARED_PTR<RouteSegmentResult>& e = gctx->fullRoute[j];
			if (e->getStartPoint().isEquals(s->getEndPoint())) {
				while ((--j) != i) {
					gctx->fullRoute.erase(gctx->fullRoute.begin() + j);
				}
				break;
			}
		}
	}
	gctx->fullRoute.shrink_to_fit();

	for (SHARED_PTR<RouteSegmentResult> r : gctx->fullRoute) {
		r->turnType = nullptr;
		r->description = "";
	}
	if (!gctx->ctx->progress->isCancelled()) {
		prepareTurnResults(gctx->ctx, gctx->fullRoute);
	}
	for (SHARED_PTR<RouteSegmentResult> r : gctx->fullRoute) {
		r->attachedRoutes.clear();
		r->preAttachedRoutes.clear();
	}
}

