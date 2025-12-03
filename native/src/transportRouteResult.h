#ifndef _OSMAND_TRANSPORT_ROUTE_RESULT_H
#define _OSMAND_TRANSPORT_ROUTE_RESULT_H
#include "CommonCollections.h"
#include "commonOsmAndCore.h"

struct TransportRouteResultSegment;
struct TransportRoutingContext;
struct TransportRoutingConfiguration;

struct TransportRouteResult {
	friend class TransportRoutePlanner;

	vector<SHARED_PTR<TransportRouteResultSegment>> segments;
	double finishWalkDist;
	double routeTime;
	SHARED_PTR<TransportRoutingConfiguration> config;

	// alternative routes always match with number of segments
	std::vector<SHARED_PTR<TransportRouteResult>> alternativeRoutes;

	TransportRouteResult(SHARED_PTR<TransportRoutingConfiguration>& cfg);

	double getWalkDist();
	float getWalkSpeed();
	int getStops();
	// bool isRouteStop(TransportStop stop);
	double getTravelDist();
	double getTravelTime();
	double getWalkTime();

	int32_t getChangeTime(const SHARED_PTR<TransportRouteResultSegment>& current,
	                      const SHARED_PTR<TransportRouteResultSegment>& next);

	int getChanges();
	void toStringPrint();
	std::string toString();

private:
	const vector<SHARED_PTR<TransportRouteResult>>& getAlternativeRoutes() const;
};

#endif /*_OSMAND_TRANSPORT_ROUTE_RESULT_H*/
