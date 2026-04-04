#ifndef _OSMAND_RAPTOR_ENGINE_TRANSPORT_PLANNER_H
#define _OSMAND_RAPTOR_ENGINE_TRANSPORT_PLANNER_H

#include "CommonCollections.h"
#include "commonOsmAndCore.h"

struct TransportRoutingContext;
struct TransportRouteResult;

class RaptorTransportPlanner {
public:
	RaptorTransportPlanner();
	~RaptorTransportPlanner();

	bool buildTransportRoute(unique_ptr<TransportRoutingContext>& ctx,
	                         vector<SHARED_PTR<TransportRouteResult>>& res);
};

#endif // _OSMAND_RAPTOR_ENGINE_TRANSPORT_PLANNER_H
