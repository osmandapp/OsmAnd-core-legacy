#include <inttypes.h>
#ifndef _OSMAND_TRANSPORT_ROUTE_RESULT_CPP
#define _OSMAND_TRANSPORT_ROUTE_RESULT_CPP
#include "transportRouteResult.h"

#include "Logging.h"
#include "transportRouteResultSegment.h"
#include "transportRoutingConfiguration.h"
#include "transportRoutingContext.h"
#include "transportRoutingObjects.h"

TransportRouteResult::TransportRouteResult(SHARED_PTR<TransportRoutingConfiguration>& cfg) {
	config = cfg;
}

// ui/logging
double TransportRouteResult::TransportRouteResult::getWalkDist() {
	double d = finishWalkDist;
	for (vector<SHARED_PTR<TransportRouteResultSegment>>::iterator it = segments.begin(); it != segments.end(); it++) {
		d += (*it)->walkDist;
	}
	return d;
}

float TransportRouteResult::getWalkSpeed() {
	return config->walkSpeed;
}

// logging only
int TransportRouteResult::getStops() {
	int stops = 0;
	for (vector<SHARED_PTR<TransportRouteResultSegment>>::iterator it = segments.begin(); it != segments.end(); it++) {
		stops += ((*it)->end - (*it)->start);
	}
	return stops;
}

// ui only:
// bool TransportRouteResult::isRouteStop (TransportStop stop) {
//     for (vector<TransportRouteResultSegment>::iterator it = segments.begin();
//     it != segments.end(); it++) {
//         if (find(*it->getTravelStops().begin(), *it->getTravelStops().end(),
//         stop) != *it->getTravelStops().end()) {
//             return true;
//         }
//     }
//     return false;
// }

// for ui/logs
double TransportRouteResult::getTravelDist() {
	double d = 0;
	for (SHARED_PTR<TransportRouteResultSegment>& it : segments) {
		d += it->getTravelDist();
	}
	return d;
}

// for ui/logs
double TransportRouteResult::getTravelTime() {
	double t = 0;
	SHARED_PTR<TransportRouteResultSegment> prev = nullptr;
	for (SHARED_PTR<TransportRouteResultSegment>& seg : segments) {
		if (config->useSchedule) {
			// TransportSchedule& sts = seg->route->schedule;
			for (int k = seg->start; k < seg->end; k++) {
				t += seg->route->schedule.avgStopIntervals[k] * 10;
			}
		} else {
			// t += config->getBoardingTime();
			if (prev != nullptr) {
				t += config->getChangeTime(prev->route->getType(), seg->route->getType());
			}
			// part of s.getTravelTime()
			// t += cfg.getBoardingTime(s.route.getType());
			t += seg->travelTime;
		}
		prev = seg;
	}
	return t;
}

// for ui/logs
double TransportRouteResult::getWalkTime() {
	return getWalkDist() / config->walkSpeed;
}

int32_t TransportRouteResult::getChangeTime(const SHARED_PTR<TransportRouteResultSegment>& current,
                                            const SHARED_PTR<TransportRouteResultSegment>& next)
{
	if (next == nullptr) {
		return 0;
	}
	return config->getChangeTime(current->route->getType(), next->route->getType());
}

// for ui/logs
int TransportRouteResult::getChanges() {
	return segments.size() - 1;
}

void TransportRouteResult::toStringPrint() {
	LogPrintf(OsmAnd::LogSeverityLevel::Info, "%s", toString().c_str());
}

std::string TransportRouteResult::toString() {
	char buff[1024];
	snprintf(buff, sizeof(buff),
	         "Route %d stops, %d changes, %.2f min: %.2f m (%.1f min) to walk, %.2f m (%.1f min) to travel\n",
	         getStops(), getChanges(), routeTime / 60, getWalkDist(), getWalkTime() / 60.0, getTravelDist(),
	         getTravelTime() / 60.0);

	std::string result(buff);

	for (int i = 0; i < segments.size(); i++) {
		SHARED_PTR<TransportRouteResultSegment>& s = segments[i];
		string time;
		string arrivalTime;
		if (s->depTime != -1) {
			time = "at " + std::to_string(s->depTime);	// formatTransportTime(s->deptTime);
		}
		int aTime = s->getArrivalTime();
		if (aTime != -1) {
			arrivalTime = "and arrive at " + std::to_string(aTime);	 // formatTransportTime(s->getArrivalTime());
		}
		snprintf(buff, sizeof(buff),
						  "%d. %s [%" PRId64 "]: walk %.1f m to '%s' and travel %s to '%s' by %s %d stops %s\n",
						  i + 1, s->route->ref.c_str(), OsmAndObfConstants::getOsmIdFromBinaryMapObjectId(s->route->id),
						  s->walkDist, s->getStart().name.c_str(), time.c_str(), s->getEnd().name.c_str(),
						  s->route->name.c_str(), s->end - s->start, arrivalTime.c_str());

		const std::string line(buff);
		result += line;
	}

	return result;
}

const vector<SHARED_PTR<TransportRouteResult>>& TransportRouteResult::getAlternativeRoutes() const {
	return alternativeRoutes;
}

#endif /*_OSMAND_TRANSPORT_ROUTE_RESULT_CPP*/
