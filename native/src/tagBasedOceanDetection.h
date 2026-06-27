#ifndef _OSMAND_TAG_BASED_OCEAN_DETECTION_H
#define _OSMAND_TAG_BASED_OCEAN_DETECTION_H

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

#include "binaryRead.h"

static const float TAG_BASED_OCEAN_MIN_RATIO = 0.25f;
static const float TAG_BASED_OCEAN_PLACE_SIGNAL_THRESHOLD = 0.5f;
static const float TAG_BASED_OCEAN_STRONG_SIGNAL_THRESHOLD = 0.75f;
static const int TAG_BASED_OCEAN_SIGNALS_TO_BLOCK = 2;

struct TagBasedOceanDetectionContext {
	bool hasLandCover = false;
	float landSignalBlockThreshold = 0;
	int landSignalCount = 0;
	bool hasBayContext = false;
};

struct TagBasedOceanLandCoverScan {
	bool hasNaturalLandCover = false;
	bool hasLanduseCandidate = false;
	bool hasSeamarkType = false;
};

inline bool isTagBasedOceanIgnoredNatural(const std::string& value) {
	return value == "water" || value == "lake" || value == "bay" ||
		   value == "coastline" || value == "coastline_line" ||
		   value == "coastline_broken" || value == "land" ||
		   value == "spring" || value == "sand" || value == "desert" ||
		   value == "beach" || value == "wetland" || value == "strait" ||
		   value == "glacier" || value == "bare_rock" || value == "ridge";
}

inline bool isTagBasedOceanIgnoredLanduse(const std::string& value) {
	return value == "water" || value == "reservoir" || value == "basin" ||
		   value == "military" || value == "residential";
}

inline void scanTagBasedOceanLandCoverTags(const std::vector<tag_value>& tags, TagBasedOceanLandCoverScan& scan) {
	for (const auto& tag : tags) {
		if (tag.first == "natural" && !isTagBasedOceanIgnoredNatural(tag.second)) {
			scan.hasNaturalLandCover = true;
			return;
		}
		if (tag.first == "landuse" && !isTagBasedOceanIgnoredLanduse(tag.second)) {
			scan.hasLanduseCandidate = true;
		} else if (tag.first == "seamark:type") {
			scan.hasSeamarkType = true;
		}
	}
}

inline bool hasTagBasedOceanLandCoverTags(const MapDataObject* obj) {
	TagBasedOceanLandCoverScan scan;
	scanTagBasedOceanLandCoverTags(obj->types, scan);
	if (!scan.hasNaturalLandCover) {
		scanTagBasedOceanLandCoverTags(obj->additionalTypes, scan);
	}
	return scan.hasNaturalLandCover || (scan.hasLanduseCandidate && !scan.hasSeamarkType);
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
		return TAG_BASED_OCEAN_PLACE_SIGNAL_THRESHOLD;
	}
	if (tag.first == "waterway") {
		return (tag.second == "river" || tag.second == "stream") ? TAG_BASED_OCEAN_STRONG_SIGNAL_THRESHOLD : 0;
	}
	if (tag.first == "natural") {
		return tag.second == "volcano" ? TAG_BASED_OCEAN_STRONG_SIGNAL_THRESHOLD : 0;
	}
	return 0;
}

inline float getTagBasedOceanLandSignalThreshold(const std::vector<tag_value>& tags) {
	float threshold = 0;
	for (const auto& tag : tags) {
		threshold = std::max(threshold, getTagBasedOceanLandSignalThreshold(tag));
		if (threshold >= TAG_BASED_OCEAN_STRONG_SIGNAL_THRESHOLD) {
			break;
		}
	}
	return threshold;
}

inline bool doesTagBasedOceanObjectCoverHalfTile(const SearchQuery* q, const MapDataObject* obj) {
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
	const int64_t overlap = (int64_t)(right - left) * (bottom - top);
	const int64_t tile = (int64_t)(q->oceanRight - q->oceanLeft) * (q->oceanBottom - q->oceanTop);
	return tile > 0 && overlap >= tile / 2;
}

inline bool isTagBasedOceanLandCoverObject(const SearchQuery* q, const MapDataObject* obj) {
	if (obj == NULL) {
		return false;
	}
	return hasTagBasedOceanLandCoverTags(obj) && doesTagBasedOceanObjectCoverHalfTile(q, obj);
}

inline float getTagBasedOceanLandSignalBlockThreshold(const MapDataObject* obj) {
	if (obj == NULL) {
		return 0;
	}
	float threshold = getTagBasedOceanLandSignalThreshold(obj->types);
	if (threshold < TAG_BASED_OCEAN_STRONG_SIGNAL_THRESHOLD) {
		threshold = std::max(threshold, getTagBasedOceanLandSignalThreshold(obj->additionalTypes));
	}
	return threshold;
}

inline bool hasTagBasedOceanBayTag(const std::vector<tag_value>& tags) {
	for (const auto& tag : tags) {
		if (tag.first == "natural" && tag.second == "bay") {
			return true;
		}
	}
	return false;
}

inline bool hasTagBasedOceanBayTag(const MapDataObject* obj) {
	return obj != NULL && (hasTagBasedOceanBayTag(obj->types) || hasTagBasedOceanBayTag(obj->additionalTypes));
}

inline void collectTagBasedOceanDetectionData(const SearchQuery* q, const MapDataObject* obj,
											  TagBasedOceanDetectionContext& context) {
	if (obj == NULL) {
		return;
	}
	if (!context.hasLandCover && isTagBasedOceanLandCoverObject(q, obj)) {
		context.hasLandCover = true;
	}
	if (context.landSignalCount < TAG_BASED_OCEAN_SIGNALS_TO_BLOCK) {
		float objectLandSignalBlockThreshold = getTagBasedOceanLandSignalBlockThreshold(obj);
		if (objectLandSignalBlockThreshold > 0) {
			context.landSignalBlockThreshold =
				std::max(context.landSignalBlockThreshold, objectLandSignalBlockThreshold);
			context.landSignalCount++;
		}
	}
	if (!context.hasBayContext && hasTagBasedOceanBayTag(obj)) {
		context.hasBayContext = true;
	}
}

inline bool shouldAddOceanObjectTagBased(const SearchQuery* q, float ocean, bool coastlinesWereAdded,
										 const TagBasedOceanDetectionContext& context) {
	bool landSignalBlocksOcean = context.landSignalBlockThreshold > 0 && ocean <= context.landSignalBlockThreshold &&
								 context.landSignalCount >= TAG_BASED_OCEAN_SIGNALS_TO_BLOCK;
	bool enoughOceanEvidence = ocean >= TAG_BASED_OCEAN_MIN_RATIO || (q->ocean > 0 && context.hasBayContext);
	return !coastlinesWereAdded && enoughOceanEvidence && !context.hasLandCover && !landSignalBlocksOcean;
}

#endif
