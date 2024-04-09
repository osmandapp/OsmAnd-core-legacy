#ifndef _OSMAND_ROUTE_CALCULATION_PROGRESS_H
#define _OSMAND_ROUTE_CALCULATION_PROGRESS_H

#include <algorithm>

#include "CommonCollections.h"
#include "ElapsedTimer.h"
#include "commonOsmAndCore.h"

class RouteCalculationProgress {
	const float INITIAL_PROGRESS = 0.05f;
	const float FIRST_ITERATION = 0.72f;
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

	float totalApproximateDistance;
	float approximatedDistance;

	float routingCalculatedTime;
	int visitedSegments;
	int visitedDirectSegments;
	int visitedOppositeSegments;
	int directQueueSize;
	int oppositeQueueSize;
    int finalSegmentsFound;

	int loadedTiles;
	int unloadedTiles;
	int loadedPrevUnloadedTiles;
	int distinctLoadedTiles;

	int totalIterations;
	int iteration;

	bool cancelled;

	int maxLoadedTiles;
	bool requestPrivateAccessRouting;

   public:
	RouteCalculationProgress()
		: segmentNotFound(-1), distanceFromBegin(0), directSegmentQueueSize(0), distanceFromEnd(0),
		  reverseSegmentQueueSize(0), totalEstimatedDistance(0), totalApproximateDistance(0),
		  approximatedDistance(0), routingCalculatedTime(0), visitedSegments(0),
		  visitedDirectSegments(0), visitedOppositeSegments(0), directQueueSize(0), oppositeQueueSize(0),
          finalSegmentsFound(0), loadedTiles(0), unloadedTiles(0), loadedPrevUnloadedTiles(0), distinctLoadedTiles(0),
          totalIterations(1), iteration(-1), cancelled(false), maxLoadedTiles(0),
          requestPrivateAccessRouting(false),
          hhIterationStep(HH_NOT_STARTED), hhCurrentStepProgress(0), hhTargetsDone(0), hhTargetsTotal(0)
		{
	}

	virtual ~RouteCalculationProgress() { }

	virtual SHARED_PTR<RouteCalculationProgress> capture(SHARED_PTR<RouteCalculationProgress>& cp);
	virtual UNORDERED(map) < string,
		UNORDERED(map) < string, string >> getInfo(SHARED_PTR<RouteCalculationProgress> firstPhase);
	virtual bool isCancelled() { return cancelled; }
	virtual void setSegmentNotFound(int s) { segmentNotFound = s; }
	virtual void updateIteration(int i) { iteration = i; }
	virtual void updateTotalEstimatedDistance(float distance) { totalEstimatedDistance = distance; }
	virtual void updateTotalApproximateDistance(float distance) { totalApproximateDistance = distance; }
	virtual void updateApproximatedDistance(float distance) { approximatedDistance = distance; }
	virtual void updateStatus(float distanceFromBegin, int directSegmentQueueSize, float distanceFromEnd,
							  int reverseSegmentQueueSize);
	virtual float getLinearProgressHH();
	virtual float getLinearProgress();
	virtual float getApproximationProgress();

	enum HHIteration {
		HH_NOT_STARTED,
		SELECT_REGIONS,
		LOAD_POINTS,
		START_END_POINT,
		ROUTING,
		DETAILED,
		ALTERNATIVES,
		DONE,
		Count
	};

	inline int hhIterationPercent(int step) {
		switch(step) {
			case HH_NOT_STARTED: return 0; // hhIteration is not filled
			case SELECT_REGIONS: return 5;
			case LOAD_POINTS: return 5;
			case START_END_POINT: return 15;
			case ROUTING: return 25;
			case DETAILED: return 50;
			case ALTERNATIVES: return 0;
			case DONE: return 0;
		}
		return 0;
	}

	int hhIterationStep;
	double hhCurrentStepProgress;
	int hhTargetsDone, hhTargetsTotal;

	virtual void hhTargetsProgress(int done, int total);
	virtual void hhIteration(HHIteration step);
	virtual void hhIterationProgress(double k);
};

#endif /*_OSMAND_ROUTE_CALCULATION_PROGRESS_H*/
