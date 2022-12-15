#ifndef _OSMAND_ROUTE_DATA_RESOURCES_CPP
#define _OSMAND_ROUTE_DATA_RESOURCES_CPP
#include "routeDataResources.h"

#include <stdexcept>

Location::Location()
	: latitude(NAN), longitude(NAN), altitude(NAN), speed(NAN), bearing(NAN), accuracy(NAN), verticalAccuracy(NAN) {
}

Location::Location(double latitude, double longitude)
	: latitude(latitude), longitude(longitude), altitude(NAN), speed(NAN), bearing(NAN), accuracy(NAN),
	  verticalAccuracy(NAN) {
}

bool Location::isInitialized() {
	return !isnan(latitude) && !isnan(longitude);
}

RouteDataResources::RouteDataResources() : currentSegmentStartLocationIndex(0) {
}

RouteDataResources::RouteDataResources(vector<Location> locations, vector<int>& routePointIndexes)
    : currentSegmentStartLocationIndex(0)
    , locations(locations)
    , routePointIndexes(routePointIndexes) {
}

Location RouteDataResources::getCurrentSegmentLocation(int offset) {
    int locationIndex = currentSegmentStartLocationIndex + offset;
    if (locationIndex >= locations.size()) {
        throw std::invalid_argument("Locations index: " + to_string(locationIndex) + " out of bounds");
    }
    return locations[locationIndex];
}

int RouteDataResources::getCurrentSegmentStartLocationIndex()
{
    return currentSegmentStartLocationIndex;
}

void RouteDataResources::updateNextSegmentStartLocation(int currentSegmentLength) {
    const auto it = find(routePointIndexes.begin(), routePointIndexes.end(), currentSegmentStartLocationIndex + currentSegmentLength);
    int routePointIndex = -1;
    if (it != routePointIndexes.end())
        routePointIndex = (int) (it - routePointIndexes.begin());
    bool overlappingNextRouteSegment = !(routePointIndex > 0 && routePointIndex < routePointIndexes.size() - 1);
    currentSegmentStartLocationIndex += overlappingNextRouteSegment ? currentSegmentLength - 1 : currentSegmentLength;
}

#endif /*_OSMAND_ROUTE_DATA_RESOURCES_CPP*/
