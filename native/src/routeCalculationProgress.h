#ifndef _OSMAND_ROUTE_CALCULATION_PROGRESS_H
#define _OSMAND_ROUTE_CALCULATION_PROGRESS_H

#include <algorithm>

#include "CommonCollections.h"
#include "ElapsedTimer.h"
#include "commonOsmAndCore.h"

struct FastRoutingState {
	enum Status {
		READY,

		// MissingMapsCalculator
		MIXED_MAPS_INTERMEDIATES,
		MISSING_MAPS_INTERMEDIATES,
		MIXED_MAPS_AT_START_OR_END,
		MISSING_MAPS_AT_START_OR_END,

		// HHRoutePlanner
		FAILED_WITH_MIXED_MAPS,
		FAILED_WITH_MISSING_MAPS,
		FAILED_NO_HH_ROUTING_DATA, // pedestrian profile, ancient maps, etc
		FAILED_WITHOUT_MAP_ISSUES, // unsupported parameters, unusual geometry (Roma to Barcelona), etc

		CANCELLED,
		SUCCESS
	};

	static bool isSuccessStatus(Status status);
	static bool isCancelledStatus(Status status);
	static bool isFailedStatus(Status status);
	static Status get(int ordinal);
	static int reset();
	static int raise(int old, Status status);
	static int fail(int old);
	static bool hasMixedOrMissingMaps(int ordinal);
	static bool isSlowRoutingActive(int ordinal);

   private:
	static bool isMixedMaps(int ordinal);
	static bool isMissingMaps(int ordinal);
};

class RouteCalculationProgress {
	const float INITIAL_PROGRESS = 0.01f;
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
	int fastRoutingStatusOrdinal;

	int maxLoadedTiles;
	bool requestPrivateAccessRouting;

   public:
	RouteCalculationProgress()
		: segmentNotFound(-1), distanceFromBegin(0), directSegmentQueueSize(0), distanceFromEnd(0),
		  reverseSegmentQueueSize(0), totalEstimatedDistance(0), totalApproximateDistance(0),
		  approximatedDistance(0), routingCalculatedTime(0), visitedSegments(0),
		  visitedDirectSegments(0), visitedOppositeSegments(0), directQueueSize(0), oppositeQueueSize(0),
		  finalSegmentsFound(0), loadedTiles(0), unloadedTiles(0), loadedPrevUnloadedTiles(0), distinctLoadedTiles(0),
		  totalIterations(1), iteration(-1), cancelled(false), fastRoutingStatusOrdinal(FastRoutingState::READY), maxLoadedTiles(0),
		  requestPrivateAccessRouting(false), hhIterationStep(HH_NOT_STARTED),
		  hhCurrentStepProgress(0), hhTargetsDone(0), hhTargetsTotal(0), hhCalcCounter(0)
		{
	}

	virtual ~RouteCalculationProgress() { }

	virtual SHARED_PTR<RouteCalculationProgress> capture(SHARED_PTR<RouteCalculationProgress>& cp);
	virtual UNORDERED(map) < string,
		UNORDERED(map) < string, string >> getInfo(SHARED_PTR<RouteCalculationProgress> firstPhase);
	virtual bool isCancelled() { return cancelled; }
	virtual int getFastRoutingStatusOrdinal() { return fastRoutingStatusOrdinal; }
	virtual void setFastRoutingStatusOrdinal(int status) { fastRoutingStatusOrdinal = status; }
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

	virtual bool hasMixedOrMissingMaps();
	virtual bool isSlowRoutingActive();
	virtual FastRoutingState::Status getFastRoutingStatus();
	virtual void resetFastRoutingStatus();
	virtual void failFastRoutingStatus();
	virtual void raiseFastRoutingStatus(FastRoutingState::Status status);

	enum HHIteration {
		HH_NOT_STARTED,
		SELECT_REGIONS,
		LOAD_POINTS,
		START_END_POINT,
		ROUTING,
		DETAILED,
		RECALCULATION,
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
			case ROUTING: return 15;
			case DETAILED: return 50;
			case RECALCULATION: return 10;
			case ALTERNATIVES: return 0;
			case DONE: return 0;
		}
		return 0;
	}

	int hhIterationStep;
	double hhCurrentStepProgress;
	int hhTargetsDone, hhTargetsTotal;
	int hhCalcCounter;

	virtual void hhUpdateCalcCounter(int counter) { hhCalcCounter = counter; }
	virtual int hhGetCalcCounter() { return hhCalcCounter; }

	virtual void hhTargetsProgress(int done, int total);
	virtual void hhIteration(HHIteration step);
	virtual void hhIterationProgress(double k);
};

#endif /*_OSMAND_ROUTE_CALCULATION_PROGRESS_H*/
