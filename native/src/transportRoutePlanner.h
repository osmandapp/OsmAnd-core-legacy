#ifndef _OSMAND_TRANSPORT_ROUTE_PLANNER_H
#define _OSMAND_TRANSPORT_ROUTE_PLANNER_H
#include <queue>

#include "CommonCollections.h"
#include "commonOsmAndCore.h"
#include "routeCalculationProgress.h"

const bool MEASURE_TIME = false;

struct TransportSegmentsComparator;
struct TransportRouteSegment;
struct TransportRoutingContext;
struct TransportRouteResult;

typedef priority_queue<SHARED_PTR<TransportRouteSegment>, vector<SHARED_PTR<TransportRouteSegment>>,
					   TransportSegmentsComparator>
	TRANSPORT_SEGMENTS_QUEUE;

class TransportRoutePlanner {
   public:
	TransportRoutePlanner();
	~TransportRoutePlanner();

	void buildTransportRoute(unique_ptr<TransportRoutingContext>& ctx, vector<SHARED_PTR<TransportRouteResult>>& res);
	// void updateCalculationProgress(unique_ptr<TransportRoutingContext>& ctx, TRANSPORT_SEGMENTS_QUEUE& queue); // TODO remove
	void prepareResults(unique_ptr<TransportRoutingContext>& ctx,
															vector<SHARED_PTR<TransportRouteSegment>>& results,
                                                            vector<SHARED_PTR<TransportRouteResult>>& routes);
	// bool includeRoute(TransportRouteResult& fastRoute, TransportRouteResult& testRoute); // TODO remove

   private:
	// bool includeRoute(SHARED_PTR<TransportRouteResult>& fastRoute, SHARED_PTR<TransportRouteResult>& testRoute); // TODO remove
	void updateCalculationProgress(unique_ptr<TransportRoutingContext>& ctx,
								   priority_queue<SHARED_PTR<TransportRouteSegment>>& queue);
	static int64_t segmentWithParentId(const SHARED_PTR<TransportRouteSegment>&, const SHARED_PTR<TransportRouteSegment>&);
	bool excludeRoute(const unique_ptr<TransportRoutingContext>& ctx,
	                  const SHARED_PTR<TransportRouteResult>& fastRoute,
	                  const SHARED_PTR<TransportRouteResult>& testRoute);
	bool checkAlternative(const unique_ptr<TransportRoutingContext>& ctx,
	                      const SHARED_PTR<TransportRouteResult>& fastRoute,
	                      const SHARED_PTR<TransportRouteResult>& testRoute);
	bool sameRouteWithExtraSegments(const SHARED_PTR<TransportRouteResult>& fastRoute,
	                                const SHARED_PTR<TransportRouteResult>& testRoute);
};

#endif	// _OSMAND_TRANSPORT_ROUTE_PLANNER_H
