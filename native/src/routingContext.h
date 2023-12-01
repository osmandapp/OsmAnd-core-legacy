#ifndef _OSMAND_ROUTING_CONTEXT_H
#define _OSMAND_ROUTING_CONTEXT_H
#include <algorithm>
#include <ctime>

#include "CommonCollections.h"
#include "binaryRead.h"
#include "commonOsmAndCore.h"
#include "precalculatedRouteDirection.h"
#include "routeCalculationProgress.h"
#include "routeSegment.h"
#include "routeSegmentResult.h"
#include "routingConfiguration.h"

#ifdef _IOS_BUILD
#include <OsmAndCore/Logging.h>
#else
#include "Logging.h"
#endif

enum class RouteCalculationMode { BASE, NORMAL, COMPLEX };

struct RoutingSubregionTile {
	RouteSubregion subregion;
	// make it without get/set for fast access
	int access;
	int loaded;
	long size;
	// JAVA: UNORDERED(map)<int64_t, SHARED_PTR> routes;
	UNORDERED(map)<int64_t, std::vector<SHARED_PTR<RouteSegment>>> routes;
	UNORDERED(set)<int64_t> excludedIds;

	RoutingSubregionTile(RouteSubregion& sub) : subregion(sub), access(0), loaded(0) {
		size = sizeof(RoutingSubregionTile);
	}
	~RoutingSubregionTile() {}
	bool isLoaded() { return loaded > 0; }

	void setLoaded() { loaded = abs(loaded) + 1; }

	void unload() {
		routes = UNORDERED(map)<int64_t, std::vector<SHARED_PTR<RouteSegment>>>();
		size = 0;
		loaded = -abs(loaded);
	}

	int getUnloadCount() { return abs(loaded); }

	long getSize() { return size + routes.size() * sizeof(std::pair<int64_t, SHARED_PTR<RouteSegment>>); }

	void add(SHARED_PTR<RouteDataObject>& o) {
		size += o->getSize() + sizeof(RouteSegment) * o->pointsX.size();
		for (uint i = 0; i < o->pointsX.size(); i++) {
			uint64_t x31 = o->pointsX[i];
			uint64_t y31 = o->pointsY[i];
			uint64_t l = (((uint64_t)x31) << 31) + (uint64_t)y31;
			routes[l].push_back(std::make_shared<RouteSegment>(o, i));
		}
	}
};

static int64_t calcRouteId(SHARED_PTR<RouteDataObject>& o, int ind) { return ((int64_t)o->id << 10) + ind; }

inline int intpow(int base, int pw) {
	int r = 1;
	for (int i = 0; i < pw; i++) {
		r *= base;
	}
	return r;
}

inline int compareRoutingSubregionTile(SHARED_PTR<RoutingSubregionTile>& o1, SHARED_PTR<RoutingSubregionTile>& o2) {
	int v1 = (o1->access + 1) * intpow(10, o1->getUnloadCount() - 1);
	int v2 = (o2->access + 1) * intpow(10, o2->getUnloadCount() - 1);
	return v1 < v2;
}

struct RoutingContext {
	typedef UNORDERED(map)<int64_t, SHARED_PTR<RoutingSubregionTile>> MAP_SUBREGION_TILES;

	RouteCalculationMode calculationMode;
	SHARED_PTR<RoutingConfiguration> config;
	SHARED_PTR<RouteCalculationProgress> progress;
	SHARED_PTR<RouteCalculationProgress> calculationProgressFirstPhase;
	bool leftSideNavigation;

	int gcCollectIterations;

	int startX;
	int startY;
	int64_t startRoadId;
	int startSegmentInd;
	bool startTransportStop;

	int targetX;
	int targetY;
	int64_t targetRoadId;
	int targetSegmentInd;
	bool targetTransportStop;
	bool publicTransport;
    int dijkstraMode;
	bool basemap;
	bool geocoding;

	time_t conditionalTime;
	tm conditionalTimeStr;

	vector<SHARED_PTR<RouteSegmentResult>> previouslyCalculatedRoute;
	SHARED_PTR<PrecalculatedRouteDirection> precalcRoute;
	SHARED_PTR<RouteSegment> finalRouteSegment;

	vector<SHARED_PTR<RouteSegment>> segmentsToVisitNotForbidden;
	vector<SHARED_PTR<RouteSegment>> segmentsToVisitPrescripted;

	MAP_SUBREGION_TILES subregionTiles;
	UNORDERED(map)<int64_t, std::vector<SHARED_PTR<RoutingSubregionTile>>> indexedSubregions;

	int alertFasterRoadToVisitedSegments;
	int alertSlowerSegmentedWasVisitedEarlier;

	RoutingContext(RoutingContext* cp) {
		this->config = cp->config;
		this->calculationMode = cp->calculationMode;
		this->leftSideNavigation = cp->leftSideNavigation;
		this->startTransportStop = cp->startTransportStop;
		this->targetTransportStop = cp->targetTransportStop;
		this->publicTransport = cp->publicTransport;
		this->conditionalTime = cp->conditionalTime;
		this->conditionalTimeStr = cp->conditionalTimeStr;
		this->basemap = cp->basemap;
		this->geocoding = cp->geocoding;
		this->progress = cp->progress;
		this->calculationProgressFirstPhase = std::make_shared<RouteCalculationProgress>();
		this->alertFasterRoadToVisitedSegments = 0;
		this->alertSlowerSegmentedWasVisitedEarlier = 0;
	}

	RoutingContext(SHARED_PTR<RoutingConfiguration> config,
				   RouteCalculationMode calcMode = RouteCalculationMode::NORMAL)
		: calculationMode(calcMode), config(config), progress(new RouteCalculationProgress()),
		  calculationProgressFirstPhase(new RouteCalculationProgress()), leftSideNavigation(false),
		  startTransportStop(false), targetTransportStop(false), publicTransport(false), geocoding(false),
		  conditionalTime(0), precalcRoute(new PrecalculatedRouteDirection()), alertFasterRoadToVisitedSegments(0),
		  alertSlowerSegmentedWasVisitedEarlier(0) {
		this->basemap = RouteCalculationMode::BASE == calcMode;
	}

	void unloadAllData(RoutingContext* except = NULL) {
		auto it = subregionTiles.begin();
		for (; it != subregionTiles.end(); it++) {
			auto tl = it->second;
			if (tl->isLoaded()) {
				if (except == NULL || except->searchSubregionTile(tl->subregion) < 0) {
					tl->unload();
					if (progress) {
						progress->unloadedTiles++;
					}
				}
			}
		}
		subregionTiles = MAP_SUBREGION_TILES();
		indexedSubregions = UNORDERED(map)<int64_t, std::vector<SHARED_PTR<RoutingSubregionTile>>>();
	}

	bool isInterrupted() { return progress != nullptr ? progress->isCancelled() : false; }

	void setConditionalTime(time_t tm) {
		conditionalTime = tm;
		if (conditionalTime != 0) {
			conditionalTimeStr = *localtime(&conditionalTime);
		}
	}

	int searchSubregionTile(RouteSubregion& subregion) {
		auto it = subregionTiles.begin();
		int i = 0;
		int ind = -1;
		for (; it != subregionTiles.end(); it++, i++) {
			auto tl = it->second;
			if (ind == -1 && tl->subregion.left == subregion.left) {
				ind = i;
			}
			if (ind >= 0) {
				if (i == subregionTiles.size() || tl->subregion.left > subregion.left) {
					ind = -i - 1;
					return ind;
				}
				if (tl->subregion.filePointer == subregion.filePointer &&
					tl->subregion.mapDataBlock == subregion.mapDataBlock) {
					return i;
				}
			}
		}
		return ind;
	}

	bool acceptLine(SHARED_PTR<RouteDataObject>& r) { return config->router->acceptLine(r); }

	long getSize() {
		// multiply 2 for to maps
		long sz = subregionTiles.size() * sizeof(pair<int64_t, SHARED_PTR<RoutingSubregionTile>>) * 2;
		MAP_SUBREGION_TILES::iterator it = subregionTiles.begin();
		for (; it != subregionTiles.end(); it++) {
			sz += it->second->getSize();
		}
		return sz;
	}

	void unloadUnusedTiles(long memoryLimit) {
		long sz = getSize();
		float critical = 0.9f * memoryLimit * 1024 * 1024;
		if (sz < critical) {
			return;
		}
		float occupiedBefore = sz / (1024. * 1024.);
		float desirableSize = memoryLimit * 0.7f * 1024 * 1024;
		vector<SHARED_PTR<RoutingSubregionTile>> list;
		MAP_SUBREGION_TILES::iterator it = subregionTiles.begin();
		int loaded = 0;
		int unloadedTiles = 0;
		for (; it != subregionTiles.end(); it++) {
			if (it->second->isLoaded()) {
				list.push_back(it->second);
				loaded++;
			}
		}
		sort(list.begin(), list.end(), compareRoutingSubregionTile);
		uint i = 0;
		while (sz >= desirableSize && i < list.size()) {
			SHARED_PTR<RoutingSubregionTile> unload = list[i];
			i++;
			sz -= unload->getSize();
			unload->unload();
			unloadedTiles++;
			if (progress) {
				progress->unloadedTiles++;
			}
		}
		for (i = 0; i < list.size(); i++) {
			list[i]->access /= 3;
		}
		OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "Run GC (before %f Mb after %f Mb) unload %d of %d tiles",
						  occupiedBefore, getSize() / (1024.0 * 1024.0), unloadedTiles, loaded);
	}

	void loadHeaderObjects(int64_t tileId) {
		const auto itSubregions = indexedSubregions.find(tileId);
		if (itSubregions == indexedSubregions.end()) {
			return;
		}
		auto& subregions = itSubregions->second;
		bool gc = false;
		for (uint j = 0; j < subregions.size(); j++) {
			if (!subregions[j]->isLoaded()) {
				gc = true;
				break;
			}
		}
		if (gc) {
			unloadUnusedTiles(config->memoryLimitation);
		}
		bool load = false;
		for (uint j = 0; j < subregions.size(); j++) {
			if (!subregions[j]->isLoaded()) {
				load = true;
				break;
			}
		}
		if (load) {
			UNORDERED(set)<int64_t> excludedIds;
			for (uint j = 0; j < subregions.size() && !isInterrupted(); j++) {
				if (!subregions[j]->isLoaded()) {
					std::vector<SHARED_PTR<DirectionPoint>> points;
					if (config->directionPoints.count() > 0) {
						// retrieve direction points for attach to routing
						RouteSubregion& subregion = subregions[j]->subregion;
						SkIRect rect =
							SkIRect::MakeLTRB(subregion.left, subregion.top, subregion.right, subregion.bottom);
						config->directionPoints.query_in_box(rect, points);
						uint32_t createType = subregion.routingIndex->findOrCreateRouteType(DirectionPoint_TAG,
																							DirectionPoint_CREATE_TYPE);
						for (SHARED_PTR<DirectionPoint>& d : points) {
							d->types.clear();
							for (std::pair<std::string, std::string>& e : d->tags) {
								uint32_t type = subregion.routingIndex->searchRouteEncodingRule(e.first, e.second);
								if (type != -1) {
									d->types.push_back(type);
								}
							}
							d->types.push_back(createType);
						}
					}

					if (progress) {
						if (subregions[j]->getUnloadCount() > 1) {
							// skip
						} else if (subregions[j]->getUnloadCount() == 1) {
							progress->loadedPrevUnloadedTiles++;
						} else {
							progress->distinctLoadedTiles++;
						}
						progress->loadedTiles++;
					}
					subregions[j]->setLoaded();
					SearchQuery q;
					vector<RouteDataObject*> res;
					searchRouteDataForSubRegion(&q, res, &subregions[j]->subregion, geocoding);
					vector<RouteDataObject*>::iterator i = res.begin();
					for (; i != res.end(); i++) {
						if (*i != NULL) {
							SHARED_PTR<RouteDataObject> o;
							o.reset(*i);
							if (conditionalTime != 0) {
								o->processConditionalTags(conditionalTimeStr);
							}
							if (acceptLine(o)) {
								if (excludedIds.find(o->getId()) == excludedIds.end()) {
									if (!points.empty() && !config->router->checkAllowPrivateNeeded) {
										connectPoint(subregions[j], o, points);
									}
									subregions[j]->add(o);
								}
							}
							if (o->getId() > 0) {
								excludedIds.insert(o->getId());
								subregions[j]->excludedIds.insert(o->getId());
							}
						}
					}
				} else {
					excludedIds.insert(subregions[j]->excludedIds.begin(), subregions[j]->excludedIds.end());
				}
			}
		}
	}

	void loadHeaders(uint32_t xloc, uint32_t yloc) {
		if (progress && progress.get()) {
			progress->timeToLoadHeaders.Start();
		}
		int z = config->zoomToLoad;
		int tz = 31 - z;
		int64_t tileId = (xloc << z) + yloc;
		if (indexedSubregions.find(tileId) == indexedSubregions.end()) {
			SearchQuery q((uint32_t)(xloc << tz), (uint32_t)((xloc + 1) << tz), (uint32_t)(yloc << tz),
						  (uint32_t)((yloc + 1) << tz));
			std::vector<RouteSubregion> tempResult;
			searchRouteSubregions(&q, tempResult, basemap, geocoding);
			std::vector<SHARED_PTR<RoutingSubregionTile>> collection;
			for (uint i = 0; i < tempResult.size(); i++) {
				RouteSubregion& rs = tempResult[i];
				int64_t key = ((int64_t)rs.left << 31) + rs.filePointer;
				if (subregionTiles.find(key) == subregionTiles.end()) {
					subregionTiles[key] = std::make_shared<RoutingSubregionTile>(rs);
				}
				collection.push_back(subregionTiles[key]);
			}
			indexedSubregions[tileId] = collection;
		}
		if (progress && progress.get()) {
			progress->timeToLoadHeaders.Pause();
		}
		if (progress && progress.get()) {
			progress->timeToLoad.Start();
		}
		loadHeaderObjects(tileId);
		if (progress && progress.get()) {
			progress->timeToLoad.Pause();
		}
	}

	void loadTileData(int x31, int y31, int zoomAround, vector<SHARED_PTR<RouteDataObject>>& dataObjects) {
		int t = config->zoomToLoad - zoomAround;
		int coordinatesShift = (1 << (31 - config->zoomToLoad));
		if (t <= 0) {
			t = 1;
			coordinatesShift = (1 << (31 - zoomAround));
		} else {
			t = 1 << t;
		}
		int z = config->zoomToLoad;
		UNORDERED(set)<int64_t> excludeDuplications;
		for (int i = -t; i <= t && !isInterrupted(); i++) {
			for (int j = -t; j <= t && !isInterrupted(); j++) {
				uint32_t xloc = (x31 + i * coordinatesShift) >> (31 - z);
				uint32_t yloc = (y31 + j * coordinatesShift) >> (31 - z);
				int64_t tileId = (xloc << z) + yloc;
				loadHeaders(xloc, yloc);
				const auto itSubregions = indexedSubregions.find(tileId);
				if (itSubregions == indexedSubregions.end()) continue;
				if (progress && progress.get()) {
					progress->timeToLoad.Start();
				}
				auto& subregions = itSubregions->second;
				for (uint j = 0; j < subregions.size(); j++) {
					if (subregions[j]->isLoaded()) {
						UNORDERED(map)<int64_t, std::vector<SHARED_PTR<RouteSegment>>>::iterator s =
							subregions[j]->routes.begin();
						while (s != subregions[j]->routes.end()) {
							auto segments = s->second;
							for (auto& segment : segments) {
								SHARED_PTR<RouteDataObject> ro = segment->road;
								if (!isExcluded(ro->id, j, subregions) && excludeDuplications.insert(ro->id).second) {
									dataObjects.push_back(ro);
								}
							}
							s++;
						}
					}
				}
				if (progress) {
					progress->timeToLoad.Pause();
				}
			}
		}
	}

	std::vector<SHARED_PTR<RouteSegment>> loadRouteSegment(int x31, int y31) {
		return loadRouteSegment(x31, y31, false);
	}

	// void searchRouteRegion(SearchQuery* q, std::vector<RouteDataObject*>& list, RoutingIndex* rs, RouteSubregion*
	// sub)
	std::vector<SHARED_PTR<RouteSegment>> loadRouteSegment(int x31, int y31, bool reverseWaySearch) {
		std::vector<SHARED_PTR<RouteSegment>> segmentsResult;

		int z = config->zoomToLoad;
		int64_t xloc = x31 >> (31 - z);
		int64_t yloc = y31 >> (31 - z);
		uint64_t l = (((uint64_t)x31) << 31) + (uint64_t)y31;
		int64_t tileId = (xloc << z) + yloc;
		loadHeaders(xloc, yloc);
		const auto itSubregions = indexedSubregions.find(tileId);
		if (itSubregions == indexedSubregions.end()) {
			return segmentsResult;
		}
		auto& subregions = itSubregions->second;
		UNORDERED(map)<int64_t, SHARED_PTR<RouteDataObject>> excludeDuplications;
		SHARED_PTR<RouteSegment> original;
		for (uint j = 0; j < subregions.size(); j++) {
			if (subregions[j]->isLoaded()) {
				std::vector<SHARED_PTR<RouteSegment>> segments = subregions[j]->routes[l];
				subregions[j]->access++;
				for (auto& segment : segments) {
					SHARED_PTR<RouteDataObject> ro = segment->road;
					SHARED_PTR<RouteDataObject> toCmp =
						excludeDuplications[calcRouteId(ro, segment->getSegmentStart())];
					if (!isExcluded(ro->id, j, subregions) && (!toCmp || toCmp->pointsX.size() < ro->pointsX.size())) {
						excludeDuplications[calcRouteId(ro, segment->getSegmentStart())] = ro;
						if (reverseWaySearch) {
							if (segment->reverseSearch.expired()) {
								auto seg = std::make_shared<RouteSegment>(ro, segment->getSegmentStart());
								seg->reverseSearch = segment;
								segment->reverseSearch = seg;
								segment = seg;
							} else {
								segment = segment->reverseSearch.lock();
							}
						}
						segmentsResult.push_back(segment);
						original = segment;
					}
				}
			}
		}

		std::reverse(segmentsResult.begin(), segmentsResult.end());
		return segmentsResult;
	}

	bool isExcluded(int64_t roadId, uint subIndex, vector<shared_ptr<RoutingSubregionTile>>& subregions) {
		for (uint j = 0; j < subIndex; j++) {
			if (subregions.at(j)->excludedIds.count(roadId) > 0) {
				return true;
			}
		}
		return false;
	}

	float getHeuristicCoefficient() { return config->heurCoefficient; }

	bool planRouteIn2Directions() { return getPlanRoadDirection() == 0; }

	int getPlanRoadDirection() { return config->planRoadDirection; }

	void initTargetPoint(SHARED_PTR<RouteSegmentPoint>& end) {
		targetX = end->preciseX;
		targetY = end->preciseY;
		targetRoadId = end->road->id;
		targetSegmentInd = end->segmentStart;
	}

	void initStartAndTargetPoints(SHARED_PTR<RouteSegmentPoint> start, SHARED_PTR<RouteSegmentPoint> end) {
		initTargetPoint(end);
		startX = start->preciseX;
		startY = start->preciseY;
		startRoadId = start->road->id;
		startSegmentInd = start->segmentStart;
	}

	void connectPoint(SHARED_PTR<RoutingSubregionTile> subRegTile, SHARED_PTR<RouteDataObject> ro,
					  std::vector<SHARED_PTR<DirectionPoint>>& points) {
		uint32_t createType = ro->region->findOrCreateRouteType(DirectionPoint_TAG, DirectionPoint_CREATE_TYPE);
		uint32_t deleteType = ro->region->findOrCreateRouteType(DirectionPoint_TAG, DirectionPoint_DELETE_TYPE);

		for (SHARED_PTR<DirectionPoint>& np : points) {
			if (np->types.size() == 0) {
				continue;
			}

			int wptX = np->x31;
			int wptY = np->y31;
			int x = ro->pointsX.at(0);
			int y = ro->pointsY.at(0);
			double mindist = config->directionPointsRadius * 2;
			int indexToInsert = 0;
			int mprojx = 0;
			int mprojy = 0;
			for (int i = 1; i < ro->pointsX.size(); i++) {
				int nx = ro->pointsX.at(i);
				int ny = ro->pointsY.at(i);
				bool sgnx = nx - wptX > 0;
				bool sgx = x - wptX > 0;
				bool sgny = ny - wptY > 0;
				bool sgy = y - wptY > 0;
				bool checkPreciseProjection = true;
				if (sgny == sgy && sgx == sgnx) {
					// Speedup: point outside of rect (line is diagonal) distance is likely be bigger
					double dist = squareRootDist31(wptX, wptY, abs(nx - wptX) < abs(x - wptX) ? nx : x,
												   abs(ny - wptY) < abs(y - wptY) ? ny : y);
					checkPreciseProjection = dist < config->directionPointsRadius;
				}
				if (checkPreciseProjection) {
					std::pair<int, int> pnt = getProjectionPoint(wptX, wptY, x, y, nx, ny);
					int projx = (int)pnt.first;
					int projy = (int)pnt.second;
					double dist = squareRootDist31(wptX, wptY, projx, projy);
					if (dist < mindist) {
						indexToInsert = i;
						mindist = dist;
						mprojx = projx;
						mprojy = projy;
					}
				}
				x = nx;
				y = ny;
			}
			bool sameRoadId = np->connected && np->connected->getId() == ro->getId();
			bool pointShouldBeAttachedByDist =
				(mindist < config->directionPointsRadius && (mindist < np->distance || np->distance < 0));

			double npAngle = np->getAngle();
			bool restrictionByAngle = npAngle != np->NO_ANGLE;

			if (pointShouldBeAttachedByDist) {
				if (restrictionByAngle) {
					int oneWay = ro->getOneway();  // -1 backward, 0 two way, 1 forward
					double forwardAngle = ro->directionRoute(indexToInsert, true, 5);
					forwardAngle = forwardAngle * 180 / M_PI;
					if (oneWay == 1 || oneWay == 0) {
						double diff = std::abs(degreesDiff(npAngle, forwardAngle));
						if (diff <= np->MAX_ANGLE_DIFF) {
							restrictionByAngle = false;
						}
					}
					if (restrictionByAngle && (oneWay == -1 || oneWay == 0)) {
						double diff = std::abs(degreesDiff(npAngle, forwardAngle + 180));
						if (diff <= np->MAX_ANGLE_DIFF) {
							restrictionByAngle = false;
						}
					}
				}
				if (restrictionByAngle) {
					continue;
				}
				if (!sameRoadId) {
					// cout << "INSERT " << ro->getId() / 64 << " (" << indexToInsert << "-" << indexToInsert + 1 << ")
					// " << mindist << "m " << "["
					//	<< get31LatitudeY(wptY) << "/" <<  get31LongitudeX(wptX) << "] x:" << wptX << " y:" << wptY << "
					// ro.id:" << ro->getId() << endl;
					if (np->connected) {
						// check old connected points
						int pointIndex = findPointIndex(np, createType);
						if (pointIndex != -1) {
							// set type "deleted" for old connected point
							std::vector<uint32_t> osmand_dp_vector{deleteType};
							np->connected->setPointTypes(pointIndex, osmand_dp_vector);
						}
					}
				} else {
					int sameRoadPointIndex = findPointIndex(np, createType);
					if (sameRoadPointIndex != -1 && np->connected) {
						if (mprojx == np->connectedx && mprojy == np->connectedy) {
							continue;  // was found the same point
						} else {
							// set type "deleted" for old connected point
							std::vector<uint32_t> osmand_dp_vector{deleteType};
							np->connected->setPointTypes(sameRoadPointIndex, osmand_dp_vector);
						}
					}
				}
				np->connectedx = mprojx;
				np->connectedy = mprojy;
				ro->insert(indexToInsert, mprojx, mprojy);
				ro->setPointTypes(indexToInsert, np->types);  // np->types contains DirectionPoint_CREATE_TYPE
				np->distance = mindist;
				np->connected = ro;
			}
		}
	}

	int findPointIndex(SHARED_PTR<DirectionPoint> np, int createType) {
		// using search by coordinates because by index doesn't work (parallel updates)
		int samePointIndex = -1;
		for (int i = 0; np->connected && i < np->connected->getPointsLength(); i++) {
			int tx = np->connected->pointsX.at(i);
			int ty = np->connected->pointsY.at(i);
			if (tx == np->connectedx && ty == np->connectedy && np->connected->hasPointType(i, createType)) {
				samePointIndex = i;
				break;
			}
		}
		return samePointIndex;
	}
};

#endif /*_OSMAND_ROUTING_CONTEXT_H*/
