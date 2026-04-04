#ifndef _OSMAND_RAPTOR_ENGINE_TRANSPORT_CONTEXT_BRIDGE_H
#define _OSMAND_RAPTOR_ENGINE_TRANSPORT_CONTEXT_BRIDGE_H

#include "CommonCollections.h"
#include "commonOsmAndCore.h"

struct TransportRoutingContext;
struct TransportStop;

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
                int32_t bottom);
        }
    }
}

#endif // _OSMAND_RAPTOR_ENGINE_TRANSPORT_CONTEXT_BRIDGE_H
