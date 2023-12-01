#ifndef _OSMAND_ROUTE_CALCULATION_PROGRESS_CPP
#define _OSMAND_ROUTE_CALCULATION_PROGRESS_CPP

#include <algorithm>
#include "routeCalculationProgress.h"

SHARED_PTR<RouteCalculationProgress> RouteCalculationProgress::capture(SHARED_PTR<RouteCalculationProgress>& cp) {
	SHARED_PTR<RouteCalculationProgress> p = std::make_shared<RouteCalculationProgress>();
	p->routingCalculatedTime = cp->routingCalculatedTime;
	p->timeToCalculate = cp->timeToCalculate;
	p->timeToLoadHeaders = cp->timeToLoadHeaders;
	p->timeToFindInitialSegments = cp->timeToFindInitialSegments;
	p->timeToLoad = cp->timeToLoad;

	p->visitedSegments = cp->visitedSegments;
	p->directQueueSize = cp->directQueueSize;
	p->reverseSegmentQueueSize = cp->reverseSegmentQueueSize;
	p->visitedDirectSegments = cp->visitedDirectSegments;
	p->visitedOppositeSegments = cp->visitedOppositeSegments;

	p->loadedTiles = cp->loadedTiles;
	p->unloadedTiles = cp->unloadedTiles;
	p->distinctLoadedTiles = cp->distinctLoadedTiles;
	p->maxLoadedTiles = cp->maxLoadedTiles;
	p->loadedPrevUnloadedTiles = cp->loadedPrevUnloadedTiles;
	p->timeExtra = cp->timeExtra;
	cp->maxLoadedTiles = 0;
	return p;
}

UNORDERED(map) < string, UNORDERED(map) < string, string >> 
    RouteCalculationProgress::getInfo(SHARED_PTR<RouteCalculationProgress> firstPhase) {
	UNORDERED(map) < string, UNORDERED(map) < string, string >> map;
	UNORDERED(map)<string, string> tiles;
	if (!firstPhase) {
		firstPhase = std::make_shared<RouteCalculationProgress>();
	}
	tiles.insert({"loadedTiles", std::to_string(this->loadedTiles - firstPhase->loadedTiles)});
	tiles.insert({"unloadedTiles", std::to_string(this->unloadedTiles - firstPhase->unloadedTiles)});
	tiles.insert({"loadedTilesDistinct", std::to_string(this->distinctLoadedTiles - firstPhase->distinctLoadedTiles)});
	tiles.insert({"loadedTilesPrevUnloaded",
				  std::to_string(this->loadedPrevUnloadedTiles - firstPhase->loadedPrevUnloadedTiles)});
	tiles.insert({"loadedTilesMax", std::to_string(max(this->maxLoadedTiles, this->distinctLoadedTiles))});
	map.insert({"tiles", tiles});

	UNORDERED(map)<string, string> segms;
	segms.insert({"visited", std::to_string(this->visitedSegments - firstPhase->visitedSegments)});
	segms.insert({"queueDirectSize", std::to_string(this->directQueueSize - firstPhase->directQueueSize)});
	segms.insert(
		{"queueOppositeSize", std::to_string(this->reverseSegmentQueueSize - firstPhase->reverseSegmentQueueSize)});
	segms.insert(
		{"visitedDirectPoints", std::to_string(this->visitedDirectSegments - firstPhase->visitedDirectSegments)});
	segms.insert({"visitedOppositePoints",
				  std::to_string(this->visitedOppositeSegments - firstPhase->visitedOppositeSegments)});
    segms.insert({"finalSegmentsFound",
            std::to_string(this->finalSegmentsFound -  firstPhase->finalSegmentsFound)});
	map.insert({"segments", segms});

	UNORDERED(map)<string, string> time;
	float timeToCalc =
		(float)(((this->timeToCalculate).GetElapsedMs() - (firstPhase->timeToCalculate).GetElapsedMs()) / 1.0e3);
	time.insert({"timeToCalculate", std::to_string(timeToCalc)});
	float timeToLoad = (float)(((this->timeToLoad).GetElapsedMs() - (firstPhase->timeToLoad).GetElapsedMs()) / 1.0e3);
	time.insert({"timeToLoad", std::to_string(timeToLoad)});
	float timeToLoadHeaders =
		(float)(((this->timeToLoadHeaders).GetElapsedMs() - (firstPhase->timeToLoadHeaders).GetElapsedMs()) / 1.0e3);
	time.insert({"timeToLoadHeaders", std::to_string(timeToLoadHeaders)});
	float timeToFindInitialSegments = (float)(((this->timeToFindInitialSegments).GetElapsedMs() -
											   (firstPhase->timeToFindInitialSegments).GetElapsedMs()) /
											  1.0e3);
	time.insert({"timeToFindInitialSegments", std::to_string(timeToFindInitialSegments)});
	float timeExtra = (float)(((this->timeExtra).GetElapsedMs() - (firstPhase->timeExtra).GetElapsedMs()) / 1.0e3);
	time.insert({"timeExtra", std::to_string(timeExtra)});
	map.insert({"time", time});

	UNORDERED(map)<string, string> metrics;
	if (timeToLoad + timeToLoadHeaders > 0) {
		metrics.insert({"tilesPerSec", std::to_string((this->loadedTiles - firstPhase->loadedTiles) /
													  (timeToLoad + timeToLoadHeaders))});
	}
	float pureTime = timeToCalc - (timeToLoad + timeToLoadHeaders + timeToFindInitialSegments);
	if (pureTime > 0) {
		metrics.insert(
			{"segmentsPerSec", std::to_string((this->visitedSegments - firstPhase->visitedSegments) / pureTime)});
	} else {
		metrics.insert({"segmentsPerSec", std::to_string((float)0)});
	}
	map.insert({"metrics", metrics});
	return map;
}

void RouteCalculationProgress::updateStatus(float distanceFromBegin, int directSegmentQueueSize, float distanceFromEnd,
											int reverseSegmentQueueSize) {
	this->distanceFromBegin = std::max(distanceFromBegin, this->distanceFromBegin);
	this->distanceFromEnd = std::max(distanceFromEnd, this->distanceFromEnd);
	this->directSegmentQueueSize = directSegmentQueueSize;
	this->reverseSegmentQueueSize = reverseSegmentQueueSize;
}

float RouteCalculationProgress::getLinearProgress() {
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
		progress = (float)((iteration + std::fmin(pr, 0.7)) / totalIterations);
	}
	return std::fmin(progress * 100.0, 99);
}

float RouteCalculationProgress::getApproximationProgress() {
	float progress = 0;
	if (totalApproximateDistance > 0) {
		progress = approximatedDistance / totalApproximateDistance;
	}
	progress = INITIAL_PROGRESS + progress * (1 - INITIAL_PROGRESS);
	return std::fmin(progress * 100.0, 99);
}

#endif /*_OSMAND_ROUTE_CALCULATION_PROGRESS_CPP*/
