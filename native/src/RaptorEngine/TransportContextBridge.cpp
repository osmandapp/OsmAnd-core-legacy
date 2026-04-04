#include "RaptorEngine/TransportContextBridge.h"

#include "binaryRead.h"
#include "transportRouteStopsReader.h"
#include "transportRoutingContext.h"

namespace OsmAnd
{
    namespace RaptorEngine
    {
        namespace TransportContextBridge
        {
            std::vector<SHARED_PTR<TransportStop>> loadMergedTransportStops(
                TransportRoutingContext& ctx,
                int32_t left,
                int32_t right,
                int32_t top,
                int32_t bottom)
            {
                std::vector<SHARED_PTR<TransportStop>> stops;
                SearchQuery query;
                ctx.buildSearchTransportRequest(&query, left, right, top, bottom, -1, stops);
                ctx.searchTransportIndexTime.Start();
                stops = ctx.transportStopsReader->readMergedTransportStops(&query, true);
                ctx.searchTransportIndexTime.Pause();
                return stops;
            }
        }
    }
}
