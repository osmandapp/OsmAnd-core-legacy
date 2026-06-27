#ifndef _OSMAND_TAG_BASED_OCEAN_DETECTION_H
#define _OSMAND_TAG_BASED_OCEAN_DETECTION_H

#include <algorithm>
#include <cstdint>
#include <vector>

#include "binaryRead.h"

struct TagBasedOceanDetectionContext {
	bool hasLandCover = false;
	float landSignalBlockThreshold = 0;
	int landSignalCount = 0;
	bool hasOceanContext = false;
};

inline bool isTagBasedOceanLandCoverTags(const std::vector<tag_value>& tags, const tag_value*& landCoverTag,
										 bool& hasSeamarkType) {
	for (const auto& tag : tags) {
		if (tag.first == "natural" &&
			tag.second != "water" && tag.second != "lake" && tag.second != "bay" &&
			tag.second != "coastline" && tag.second != "coastline_line" &&
			tag.second != "coastline_broken" && tag.second != "land" &&
			tag.second != "spring" && tag.second != "sand" &&
			tag.second != "desert" && tag.second != "beach" &&
			tag.second != "wetland" && tag.second != "strait" &&
			tag.second != "glacier" && tag.second != "bare_rock" &&
			tag.second != "ridge") {
			landCoverTag = &tag;
			return true;
		}
		if (tag.first == "landuse" &&
			tag.second != "water" && tag.second != "reservoir" && tag.second != "basin" &&
			tag.second != "military" && tag.second != "residential") {
			landCoverTag = &tag;
		} else if (tag.first == "seamark:type") {
			hasSeamarkType = true;
		}
	}
	return false;
}

inline bool isTagBasedOceanSettlementPlace(const tag_value& tag) {
	if (tag.first != "place") {
		return false;
	}
	return tag.second == "city" || tag.second == "town" || tag.second == "village" || tag.second == "hamlet" ||
		   tag.second == "suburb" || tag.second == "neighbourhood" || tag.second == "quarter" ||
		   tag.second == "farm" || tag.second == "isolated_dwelling";
}

inline float getTagBasedOceanLandSignalThreshold(const tag_value& tag) {
	if (isTagBasedOceanSettlementPlace(tag)) {
		return 0.5f;
	}
	if (tag.first == "waterway") {
		return (tag.second == "river" || tag.second == "stream") ? 0.75f : 0;
	}
	if (tag.first == "natural") {
		return tag.second == "volcano" ? 0.75f : 0;
	}
	return 0;
}

inline float getTagBasedOceanLandSignalThreshold(const std::vector<tag_value>& tags) {
	float threshold = 0;
	for (const auto& tag : tags) {
		threshold = std::max(threshold, getTagBasedOceanLandSignalThreshold(tag));
	}
	return threshold;
}

inline bool getTagBasedOceanTileOverlap(SearchQuery* q, const MapDataObject* obj, int64_t& overlap, int64_t& tile) {
	if (obj->points.empty()) {
		return false;
	}
	int minX = INT32_MAX;
	int maxX = INT32_MIN;
	int minY = INT32_MAX;
	int maxY = INT32_MIN;
	for (const auto& point : obj->points) {
		minX = std::min(minX, point.first);
		maxX = std::max(maxX, point.first);
		minY = std::min(minY, point.second);
		maxY = std::max(maxY, point.second);
	}
	const int left = std::max(minX, q->oceanLeft);
	const int right = std::min(maxX, q->oceanRight);
	const int top = std::max(minY, q->oceanTop);
	const int bottom = std::min(maxY, q->oceanBottom);
	if (left >= right || top >= bottom) {
		return false;
	}
	overlap = (int64_t)(right - left) * (bottom - top);
	tile = (int64_t)(q->oceanRight - q->oceanLeft) * (q->oceanBottom - q->oceanTop);
	return tile > 0 && overlap >= tile / 2;
}

inline bool isTagBasedOceanLandCoverObject(SearchQuery* q, const MapDataObject* obj) {
	if (obj == NULL) {
		return false;
	}
	const tag_value* landCoverTag = NULL;
	bool hasSeamarkType = false;
	bool hasLandCoverTag = isTagBasedOceanLandCoverTags(obj->types, landCoverTag, hasSeamarkType);
	hasLandCoverTag = isTagBasedOceanLandCoverTags(obj->additionalTypes, landCoverTag, hasSeamarkType) || hasLandCoverTag;
	if (hasLandCoverTag || (landCoverTag != NULL && !hasSeamarkType)) {
		int64_t overlap = 0;
		int64_t tile = 0;
		return getTagBasedOceanTileOverlap(q, obj, overlap, tile);
	}
	return false;
}

inline float getTagBasedOceanLandSignalBlockThreshold(const MapDataObject* obj) {
	if (obj == NULL) {
		return 0;
	}
	return std::max(getTagBasedOceanLandSignalThreshold(obj->types),
					getTagBasedOceanLandSignalThreshold(obj->additionalTypes));
}

inline bool isTagBasedOceanContextTags(const std::vector<tag_value>& tags) {
	for (const auto& tag : tags) {
		if (tag.first == "natural" && tag.second == "bay") {
			return true;
		}
	}
	return false;
}

inline bool isTagBasedOceanContextObject(const MapDataObject* obj) {
	return obj != NULL && (isTagBasedOceanContextTags(obj->types) || isTagBasedOceanContextTags(obj->additionalTypes));
}

inline void collectTagBasedOceanDetectionData(SearchQuery* q, const MapDataObject* obj,
											  TagBasedOceanDetectionContext& context) {
	if (q == NULL || !q->useTagBasedOceanDetection || obj == NULL) {
		return;
	}
	if (!context.hasLandCover && isTagBasedOceanLandCoverObject(q, obj)) {
		context.hasLandCover = true;
	}
	if (context.landSignalCount < 2) {
		float objectLandSignalBlockThreshold = getTagBasedOceanLandSignalBlockThreshold(obj);
		if (objectLandSignalBlockThreshold > 0) {
			context.landSignalBlockThreshold =
				std::max(context.landSignalBlockThreshold, objectLandSignalBlockThreshold);
			context.landSignalCount++;
		}
	}
	if (!context.hasOceanContext && isTagBasedOceanContextObject(obj)) {
		context.hasOceanContext = true;
	}
}

inline bool shouldAddOceanObjectTagBased(SearchQuery* q, float ocean, bool coastlinesWereAdded,
										 const TagBasedOceanDetectionContext& context) {
	if (q == NULL || !q->useTagBasedOceanDetection) {
		return !coastlinesWereAdded && ocean >= 0.5;
	}
	bool landSignalBlocksOcean = context.landSignalBlockThreshold > 0 && ocean <= context.landSignalBlockThreshold &&
								 context.landSignalCount > 1;
	bool enoughOceanEvidence = ocean >= 0.25 || (q->ocean > 0 && context.hasOceanContext);
	return !coastlinesWereAdded && enoughOceanEvidence && !context.hasLandCover && !landSignalBlocksOcean;
}

#endif
