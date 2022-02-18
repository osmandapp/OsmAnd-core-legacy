#ifndef _OSMAND_ROUTE_CALCULATION_PROGRESS_H
#define _OSMAND_ROUTE_CALCULATION_PROGRESS_H

#include <algorithm>

#include "CommonCollections.h"
#include "ElapsedTimer.h"
#include "commonOsmAndCore.h"

class RouteCalculationProgress {
   public:
	OsmAnd::ElapsedTimer timeToLoad;
	OsmAnd::ElapsedTimer timeToCalculate;
	OsmAnd::ElapsedTimer timeToLoadHeaders;
	OsmAnd::ElapsedTimer timeToFindInitialSegments;
	OsmAnd::ElapsedTimer timeExtra;

	int segmentNotFound;
	float distanceFromBegin;
	int directSegmentQueueSize;
	float distanceFromEnd;
	int reverseSegmentQueueSize;
	float totalEstimatedDistance;

	float routingCalculatedTime;
	int visitedSegments;
	int visitedDirectSegments;
	int visitedOppositeSegments;
	int directQueueSize;
	int oppositeQueueSize;

	int loadedTiles;
	int unloadedTiles;
	int loadedPrevUnloadedTiles;
	int distinctLoadedTiles;

	int totalIterations;
	int iteration;

	bool cancelled;

	int maxLoadedTiles;

   public:
	RouteCalculationProgress()
		: segmentNotFound(-1), distanceFromBegin(0), directSegmentQueueSize(0), distanceFromEnd(0),
		  reverseSegmentQueueSize(0), totalEstimatedDistance(0), routingCalculatedTime(0), visitedSegments(0),
		  visitedDirectSegments(0), visitedOppositeSegments(0), directQueueSize(0), oppositeQueueSize(0),
		  loadedTiles(0), unloadedTiles(0), loadedPrevUnloadedTiles(0), distinctLoadedTiles(0), totalIterations(1),
		  iteration(-1), cancelled(false), maxLoadedTiles(0) {
	}

	virtual SHARED_PTR<RouteCalculationProgress> capture(SHARED_PTR<RouteCalculationProgress>& cp);
	virtual UNORDERED(map) < string,
		UNORDERED(map) < string, string >> getInfo(SHARED_PTR<RouteCalculationProgress> firstPhase);
	virtual bool isCancelled() { return cancelled; }
	virtual void setSegmentNotFound(int s) { segmentNotFound = s; }
	virtual void updateStatus(float distanceFromBegin, int directSegmentQueueSize, float distanceFromEnd,
							  int reverseSegmentQueueSize);
	virtual float getLinearProgress();
};

#endif /*_OSMAND_ROUTE_CALCULATION_PROGRESS_H*/
