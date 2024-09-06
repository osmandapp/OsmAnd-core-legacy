#include <gpxMultiSegmentsApproximation.h>
#ifndef _OSMAND_ROUTE_PLANNER_FRONT_END_CPP
#define _OSMAND_ROUTE_PLANNER_FRONT_END_CPP

#include "routePlannerFrontEnd.h"

#include "binaryRoutePlanner.h"
#include "routeResultPreparation.h"
#include "routeSegment.h"
#include "routeSegmentResult.h"
#include "routingConfiguration.h"
#include "gpxRouteApproximation.h"
// #include "gpxMultiSegmentsApproximation.h"
#include "gpxSimplePointsMatchApproximation.h"
// #include "gpxAdvancedPointsMatchApproximation.h"

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

void RoutePlannerFrontEnd::makeStartEndPointsPrecise(RoutingContext* ctx, vector<SHARED_PTR<RouteSegmentResult>>& res,
                                                     int startX, int startY, int endX, int endY) {
	if (res.size() > 0) {
		makeSegmentPointPrecise(ctx, res[0], startX, startY, true);
		makeSegmentPointPrecise(ctx, res[res.size() - 1], endX, endY, false);
	}
}

void addPrecalculatedToResult(SHARED_PTR<RouteSegment> recalculationEnd, vector<SHARED_PTR<RouteSegmentResult>>& result) {
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
}

void copyAttachedToPreAttachedRoutes(vector<SHARED_PTR<RouteSegmentResult>>& segments) {
	for (auto& seg : segments) {
		seg->preAttachedRoutes = seg->attachedRoutes;
	}
}

vector<SHARED_PTR<RouteSegmentResult>> runRouting(RoutingContext* ctx, SHARED_PTR<RouteSegment> recalculationEnd) {
	refreshProgressDistance(ctx);

	vector<SHARED_PTR<RouteSegmentResult>> result = searchRouteInternal(ctx, false);
	// convertFinalSegmentToResults() and attachConnectedRoads() already called
	addPrecalculatedToResult(recalculationEnd, result);
	copyAttachedToPreAttachedRoutes(result);

	if (ctx->finalRouteSegment && ctx->progress) {
		ctx->progress->routingCalculatedTime += ctx->finalRouteSegment->distanceFromStart;
	}

	return prepareResult(ctx, result);
}

bool RoutePlannerFrontEnd::hasSegment(vector<SHARED_PTR<RouteSegmentResult>>& result, SHARED_PTR<RouteSegment>& current) {
	for (SHARED_PTR<RouteSegmentResult> r : result) {
		int64_t currentId = r->object->id;
		if (currentId == current->road->id && r->getStartPointIndex() == current->getSegmentStart() &&
			r->getEndPointIndex() == current->getSegmentEnd()) {
			return true;
		}
	}
	return false;
}

vector<SHARED_PTR<RouteSegmentResult>> RoutePlannerFrontEnd::searchRouteInternalPrepare(
	RoutingContext* ctx, SHARED_PTR<RouteSegmentPoint> start, SHARED_PTR<RouteSegmentPoint> end,
	SHARED_PTR<PrecalculatedRouteDirection> routeDirection)
{
	if (!start || !end) {
		OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "searchRouteInternalPreparerunRouting() got empty start/end");
		return vector<SHARED_PTR<RouteSegmentResult>>();
	}

	auto recalculationEnd = getRecalculationEnd(ctx);
	if (recalculationEnd) {
		ctx->initStartAndTargetPoints(start, recalculationEnd);
	} else {
		ctx->initStartAndTargetPoints(start, end);
	}
	if (routeDirection) {
		ctx->precalcRoute = routeDirection->adopt(ctx);
	}

	refreshProgressDistance(ctx);

	vector<SHARED_PTR<RouteSegment>> segments = searchRouteInternal(ctx, start, end, {}, {});

	if (segments.empty()) {
		// OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Warning, "searchRouteInternalPrepare() got empty segments");
		return vector<SHARED_PTR<RouteSegmentResult>>();
	}

	SHARED_PTR<RouteSegment> finalSegment = ctx->finalRouteSegment = segments[0];
	if (ctx->progress) ctx->progress->routingCalculatedTime += finalSegment->distanceFromStart;

	vector<SHARED_PTR<RouteSegmentResult>> result = convertFinalSegmentToResults(ctx, finalSegment);
	attachConnectedRoads(ctx, result); // native-only method
	addPrecalculatedToResult(recalculationEnd, result);
	copyAttachedToPreAttachedRoutes(result);
	prepareResult(ctx, result);
	return result;
}

void RoutePlannerFrontEnd::makeSegmentPointPrecise(RoutingContext* ctx, SHARED_PTR<RouteSegmentResult>& routeSegmentResult,
												   double lat, double lon, bool st) {
	int px = get31TileNumberX(lon);
	int py = get31TileNumberY(lat);
	return makeSegmentPointPrecise(ctx, routeSegmentResult, px, py, st);
}

void RoutePlannerFrontEnd::makeSegmentPointPrecise(RoutingContext* ctx, SHARED_PTR<RouteSegmentResult>& routeSegmentResult,
												   int px, int py, bool st) {
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
		// correct distance
		calculateTimeSpeed(ctx, routeSegmentResult);
	}
}

// BRP-ios main function with interpoints support (called by entry-point function)
vector<SHARED_PTR<RouteSegmentResult>> RoutePlannerFrontEnd::searchRoute(
	RoutingContext* ctx, vector<SHARED_PTR<RouteSegmentPoint>>& points,
	SHARED_PTR<PrecalculatedRouteDirection> routeDirection) {
	if (points.size() <= 2) {
		if (!useSmartRouteRecalculation) {
			ctx->previouslyCalculatedRoute.clear();
		}
		auto res = searchRouteInternalPrepare(ctx, points[0], points[1], routeDirection); // BRP-ios (never reached code)
		makeStartEndPointsPrecise(ctx, res, points[0]->preciseX, points[0]->preciseY, points[1]->preciseX, points[1]->preciseY);
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
		makeStartEndPointsPrecise(&local, res, points[i]->preciseX, points[i]->preciseY, points[i + 1]->preciseX, points[i + 1]->preciseY);
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
	if (HH_ROUTING_CONFIG != nullptr && ctx->calculationMode != RouteCalculationMode::BASE) {
		HHRoutePlanner routePlanner(ctx.get());
		HHNetworkRouteRes * r = nullptr;
		double dir = ctx->config->initialDirection ;
		for (int i = 0; i < targetsX.size(); i++) {
			double initialPenalty = ctx->config->penaltyForReverseDirection;
			if (i > 0) {
				ctx->config->penaltyForReverseDirection /= 2; // relax reverse-penalty (only for inter-points)
			}
			ctx->progress->hhTargetsProgress(i, targetsX.size());
			int sx = i == 0 ? startX : targetsX.at(i - 1);
			int sy = i == 0 ? startY : targetsY.at(i - 1);
			int ex = targetsX.at(i);
			int ey = targetsY.at(i);
			HHNetworkRouteRes* res = calculateHHRoute(routePlanner, ctx.get(), sx, sy, ex, ey, dir);
			ctx->config->penaltyForReverseDirection = initialPenalty;
			if (res) {
				prepareResult(ctx.get(), res->detailed);
				makeStartEndPointsPrecise(ctx.get(), res->detailed, sx, sy, ex, ey);
			}
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
		if (r && (r->isCorrect() || USE_ONLY_HH_ROUTING)) {
			return r->detailed; // exit-point
		}
		ctx->unloadAllData();
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
		if (ls.size() == 0) {
			return ls;
		}
		routeDirection =
			PrecalculatedRouteDirection::build(ls, ctx->config->DEVIATION_RADIUS, ctx->config->router->maxSpeed);
		ctx->calculationProgressFirstPhase =  ctx->progress->capture(ctx->progress);
	}
	if (!useSmartRouteRecalculation && intermediatesEmpty) {
		ctx->previouslyCalculatedRoute = {};
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
		makeStartEndPointsPrecise(ctx.get(), res, startX, startY, endX, endY);
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
			if (res->error == "") {
				ctx->progress->hhIteration(RouteCalculationProgress::HHIteration::DONE);
				// makeStartEndPointsPrecise(ctx, res->detailed, startX, startY, endX, endY); // called by parent
				return res;
			} else {
				OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "%s", res->error.c_str());
			}
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
			int sx = i == 0 ? ctx->startX : targetsX.at(i - 1);
			int sy = i == 0 ? ctx->startY : targetsY.at(i - 1);
			int ex = targetsX.at(i);
			int ey = targetsY.at(i);
			HHNetworkRouteRes* res = calculateHHRoute(routePlanner, ctx, sx, sy, ex, ey, dir);
			ctx->config->penaltyForReverseDirection = initialPenalty;
			if (res && !intermediatesEmpty) {
				// makePrecise must be called inside native interpoints cycle
				makeStartEndPointsPrecise(ctx, res->detailed, sx, sy, ex, ey);
			}
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
		if (r && (r->isCorrect() || USE_ONLY_HH_ROUTING)) {
			OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "Finish searchHHRoute Native");
			attachConnectedRoads(ctx, r->detailed);
			return r->detailed;
		}
		ctx->unloadAllData();
	}
	return {};
}

RoutePlannerFrontEnd::RoutePlannerFrontEnd() : useSmartRouteRecalculation(true) { }

RoutePlannerFrontEnd::RoutePlannerFrontEnd(HHRoutingConfig* hhConfig) : useSmartRouteRecalculation(true) {
	if (HH_ROUTING_CONFIG != nullptr) {
		delete HH_ROUTING_CONFIG; // unreachable?
	}
	HH_ROUTING_CONFIG = hhConfig;
}

void RoutePlannerFrontEnd::setUseFastRecalculation(bool use) {
	useSmartRouteRecalculation = use;
}

void RoutePlannerFrontEnd::setUseGeometryBasedApproximation(bool enabled) {
	useGeometryBasedApproximation = enabled;
}

// JNI/iOS entry point for all types of GPX Approximation algorithms
void RoutePlannerFrontEnd::searchGpxRoute(SHARED_PTR<GpxRouteApproximation>& gctx,
                                          vector<SHARED_PTR<GpxPoint>>& gpxPoints,
                                          GpxRouteApproximationCallback acceptor) {
	gctx->setRouter(this);

	if (!gctx->ctx->progress) {
		gctx->ctx->progress = std::make_shared<RouteCalculationProgress>();
	}

	if (useGeometryBasedApproximation) {
		switch (GPX_SEGMENT_ALGORITHM) {
			case GPX_OSM_POINTS_MATCH_ALGORITHM:
				GpxSimplePointsMatchApproximation().gpxApproximation(this, gctx, gpxPoints);
				break;
			case GPX_OSM_MULTISEGMENT_SCAN_ALGORITHM:
				GpxMultiSegmentsApproximation(gctx, gpxPoints).gpxApproximation();
				break;
			case GPX_OSM_ADVANCED_POINTS_MATCH_ALGORITHM:
				// under construction
				break;
		}
	}
	else {
		gctx->searchGpxRouteByRouting(gctx, gpxPoints);
	}

	gctx->calculateGpxRoute(gctx, gpxPoints);
	gctx->reconstructFinalPointsFromFullRoute();

	if (acceptor) {
		acceptor(gctx->ctx->progress->cancelled ? nullptr : gctx);
	}
}

#endif /*_OSMAND_ROUTE_PLANNER_FRONT_END_CPP*/
