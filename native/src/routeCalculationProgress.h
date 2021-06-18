#ifndef _OSMAND_ROUTE_CALCULATION_PROGRESS_H
#define _OSMAND_ROUTE_CALCULATION_PROGRESS_H

#include <algorithm>

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
	int loadedTiles;

	int visitedDirectSegments;
	int visitedOppositeSegments;
	int directQueueSize;
	int oppositeQueueSize;

	int totalIterations;
	int iteration;

	bool cancelled;
	
public:
	RouteCalculationProgress() : segmentNotFound(-1), distanceFromBegin(0),
		directSegmentQueueSize(0), distanceFromEnd(0),  reverseSegmentQueueSize(0), totalEstimatedDistance(0),
		routingCalculatedTime(0), visitedSegments(0), loadedTiles(0), 
		visitedDirectSegments(0), visitedOppositeSegments(0), directQueueSize(0),  oppositeQueueSize(0), 
		totalIterations(1), iteration(-1), cancelled(false) {
	}

	virtual bool isCancelled(){
		return cancelled;
	}

	virtual void setSegmentNotFound(int s){
		segmentNotFound = s;
	}

	virtual void updateStatus(float distanceFromBegin,	int directSegmentQueueSize,	float distanceFromEnd,
			int reverseSegmentQueueSize) {
		this->distanceFromBegin = std::max(distanceFromBegin, this->distanceFromBegin);
		this->distanceFromEnd = std::max(distanceFromEnd,this->distanceFromEnd);
		this->directSegmentQueueSize = directSegmentQueueSize;
		this->reverseSegmentQueueSize = reverseSegmentQueueSize;
	}
	
	virtual float getLinearProgress() {
		const float INITIAL_PROGRESS = 0.05f;
		const float FIRST_ITERATION = 0.72f;
		float p = std::max(this->distanceFromBegin, this->distanceFromEnd);
		float all = totalEstimatedDistance * 1.35f;
		float pr = 0;
		if (all > 0) {
			pr = std::fmin(p * p / (all * all), 1);
		}
		float progress = INITIAL_PROGRESS;
		if (totalIterations <= 1) {
			progress = INITIAL_PROGRESS + pr * (1 - INITIAL_PROGRESS);
		} else if (totalIterations <= 2) {
			if (iteration < 1) {
				progress = pr * FIRST_ITERATION + INITIAL_PROGRESS;
			} else {
				progress = (INITIAL_PROGRESS + FIRST_ITERATION) + pr * (1 - FIRST_ITERATION - INITIAL_PROGRESS);
			}
		} else {
			progress = (float) ((iteration + std::fmin(pr, 0.7)) / totalIterations);
		}
		return std::fmin(progress * 100.0, 99);
	}
};

#endif /*_OSMAND_ROUTE_CALCULATION_PROGRESS_H*/
