#ifndef _OSMAND_MVT_WRITE_H
#define _OSMAND_MVT_WRITE_H

#include <stdio.h>
#include <stdlib.h>

#include <fstream>
#include <map>
#include <string>

#include <vtzero/builder.hpp>
#include <vtzero/index.hpp>

#include "CommonCollections.h"
#include "commonOsmAndCore.h"

typedef vtzero::key_index<std::unordered_map> KeyIdx;
typedef vtzero::value_index<vtzero::string_value_type, std::string, std::unordered_map> ValueIdx;
typedef vtzero::value_index<vtzero::sint_value_type, int32_t, std::unordered_map> IntValueIdx;

static const int MVT_TILE_EXTENT_SHIFT = 12;
static const int MVT_TILE_WIDTH = 1 << MVT_TILE_EXTENT_SHIFT; // 4096
static const int MVT_TILE_INCREASE_DETAILS_BEFORE_DETAILED_ZOOM = 9; // increase basemap details

inline int scaleToTile(int coord, int tileStart, int shift) {
	int delta = coord - tileStart;
	if (shift == 0)
		return delta;
	if (shift < 0)
		return delta * (1u << (-shift));
	if (delta < 0)
	{
        auto udelta = (1u << 31) + static_cast<unsigned int>(delta);
        return static_cast<int>((udelta >> shift) + ((udelta >> (shift - 1)) & 1) - (1u << (31 - shift)));
	}
	return (delta >> shift) + ((delta >> (shift - 1)) & 1);
}

inline bool scalePoint(std::pair<int, int>& point, std::pair<int, int> prevPoint,
	std::pair<int, int> tileCorner, int shift) {
	point.first = scaleToTile(point.first, tileCorner.first, shift);
	point.second = scaleToTile(point.second, tileCorner.second, shift);
	if (point == prevPoint)
		return false;
	return true;
}

inline int computeOutCode(std::pair<int, int> p, std::pair<int, int> topLeft, std::pair<int, int> bottomRight) {
	int result =
		(p.first < topLeft.first ? 1 : 0) | (p.first > bottomRight.first ? 2 : 0)
		| (p.second < topLeft.second ? 4 : 0) | (p.second > bottomRight.second ? 8 : 0);
	return result;
}

inline std::pair<int, int> getIntersectionX(std::pair<int, int> p0, std::pair<int, int> p1, int limit) {
	std::pair<int, int> result;
	int64_t dx = p1.first - p0.first;
	auto offset = dx / 2;
    auto num = static_cast<int64_t>(limit - p0.first) * (p1.second - p0.second);
	result.first = limit;
	result.second = p0.second + static_cast<int>((num + ((num ^ dx) < 0 ? -offset : offset)) / dx);
	return result;
}

inline std::pair<int, int> getIntersectionY(std::pair<int, int> p0, std::pair<int, int> p1, int limit) {
	std::pair<int, int> result;
	int64_t dy = p1.second - p0.second;
	auto offset = dy / 2;
    auto num = static_cast<int64_t>(limit - p0.second) * (p1.first - p0.first);
	result.first = p0.first + static_cast<int>((num + ((num ^ dy) < 0 ? -offset : offset)) / dy);
	result.second = limit;
	return result;
}

inline std::pair<int, int> getIntersection(int code, std::pair<int, int> p0, std::pair<int, int> p1,
	std::pair<int, int> topLeft, std::pair<int, int> bottomRight) {
	if ((code & 1) > 0)
		return getIntersectionX(p0, p1, topLeft.first);
	if ((code & 2) > 0)
		return getIntersectionX(p0, p1, bottomRight.first);
	if ((code & 4) > 0)
		return getIntersectionY(p0, p1, topLeft.second);
	return getIntersectionY(p0, p1, bottomRight.second);
}

inline void clipPolylineForTile(const coordinates& polyline, std::pair<int, int> topLeft,
	std::pair<int, int> bottomRight, std::pair<int, int> tileCorner, int shift,
	std::vector<coordinates>& result, std::pair<int, int>& center) {
	result.clear();
	if (polyline.size() < 2)
		return;
    coordinates segment;
	const auto size = polyline.size();
	segment.reserve(size);
    auto prevPoint = polyline[0];
	std::pair<int, int> sm(INT32_MIN, INT32_MIN);
	std::pair<int, int> m;
	int prevCode = computeOutCode(prevPoint, topLeft, bottomRight);
	int64_t sumX = prevPoint.first;
	int64_t sumY = prevPoint.second;
	for (size_t i = 1; i < size; i++) {
        auto nextPoint = polyline[i];
		sumX += nextPoint.first;
		sumY += nextPoint.second;
    	int nextCode = computeOutCode(nextPoint, topLeft, bottomRight);
		auto p0 = prevPoint;
		auto p1 = nextPoint;
		int code0 = prevCode;
		int code1 = nextCode;
    	bool accept = false;
		while (true) {
			if ((code0 | code1) == 0) {
				accept = true;
				break;
			} else if ((code0 & code1) > 0)
				break;
			else if (code0 > 0) {
				p0 = getIntersection(code0, p0, p1, topLeft, bottomRight);
				code0 = computeOutCode(p0, topLeft, bottomRight);
			} else {
				p1 = getIntersection(code1, p0, p1, topLeft, bottomRight);
				code1 = computeOutCode(p1, topLeft, bottomRight);
			}
		}
		if (accept) {
			bool isEmpty = segment.empty();
			m = p0;
			if (isEmpty)
				sm = {INT32_MIN, INT32_MIN};
			bool withPoint = scalePoint(m, sm, tileCorner, shift);
			if (isEmpty || segment.back() != m) {
				if (!isEmpty) {
			        result.push_back(std::move(segment));
					segment.clear();
				}
				if (withPoint) {
					segment.emplace_back(m);
					sm = m;
				}
			}
			m = p1;
			if (scalePoint(m, sm, tileCorner, shift)) {
				segment.emplace_back(m);
				sm = m;
			}
        } else if (!segment.empty()) {
	        result.push_back(std::move(segment));
			segment.clear();
        }
		prevPoint = nextPoint;
		prevCode = nextCode;
    }
    if (!segment.empty()) {
		result.push_back(std::move(segment));
    }
	center.first = static_cast<int>(sumX / size);
	center.second = static_cast<int>(sumY / size);
}

template<bool IsY, bool IsGreater>
inline bool isOutside(std::pair<int, int> p, int limit) {
    if (IsY)
        return IsGreater ? (p.second > limit) : (p.second < limit);
    return IsGreater ? (p.first > limit) : (p.first < limit);
}

template<bool IsY, bool IsGreater>
inline bool clipPolygonAgainstEdge(const coordinates& polygon,
	coordinates& result, int limit, std::pair<int, int> tileCorner, int shift, int64_t& sumX, int64_t& sumY) {
	auto sp = polygon.back();
	std::pair<int, int> sm(INT32_MIN, INT32_MIN);
	std::pair<int, int> m;
	bool sOutside = isOutside<IsY, IsGreater>(sp, limit);
    for (const auto p : polygon) {
		if (IsY && IsGreater)	{
			sumX += p.first;
			sumY += p.second;
		}
		bool pOutside = isOutside<IsY, IsGreater>(p, limit);
        if (!pOutside) {
            if (sOutside) {
				if (IsY)
					m = getIntersectionY(sp, p, limit);
				else
					m = getIntersectionX(sp, p, limit);
				if (!IsY && !IsGreater) {
					if (scalePoint(m, sm, tileCorner, shift)) {
						result.push_back(m);
						sm = m;
					}
				} else
					result.push_back(m);
			}
			if (!IsY && !IsGreater) {
				m = p;
				if (scalePoint(m, sm, tileCorner, shift)) {
					result.push_back(m);
					sm = m;
				}
			} else
				result.push_back(p);
        } else if (!sOutside) {
			if (IsY)
				m = getIntersectionY(sp, p, limit);
			else
				m = getIntersectionX(sp, p, limit);
			if (!IsY && !IsGreater) {
				if (scalePoint(m, sm, tileCorner, shift)) {
					result.push_back(m);
					sm = m;
				}
			} else
				result.push_back(m);
		}
        sp = p;
		sOutside = pOutside;
    }
	if (result.size() < 3)
		return false;
	return true;
}

inline void clipPolygonForTile(const coordinates& polygon, std::pair<int, int> topLeft,
	std::pair<int, int> bottomRight, std::pair<int, int> tileCorner, int shift,
	coordinates& temp, coordinates& result, std::pair<int, int>& center) {
	result.clear();
	const auto size = polygon.size();
	if (size < 3)
		return;
    result.reserve(size + 4);
	temp.clear();
	temp.reserve(size + 4);
	int64_t sumX = 0;
	int64_t sumY = 0;
	if (!clipPolygonAgainstEdge<true, true>(polygon, temp, bottomRight.second, tileCorner, shift, sumX, sumY))
		return;
	if (!clipPolygonAgainstEdge<false, true>(temp, result, bottomRight.first, tileCorner, shift, sumX, sumY)) {
		result.clear();
		return;
	}
	temp.clear();
	if (!clipPolygonAgainstEdge<true, false>(result, temp, topLeft.second, tileCorner, shift, sumX, sumY)) {
		result.clear();
		return;
	}
	result.clear();
	if (!clipPolygonAgainstEdge<false, false>(temp, result, topLeft.first, tileCorner, shift, sumX, sumY)) {
		result.clear();
		return;
	}
	center.first = static_cast<int>(sumX / size);
	center.second = static_cast<int>(sumY / size);
}

inline void addObjectDataToMapboxVectorTile(vtzero::feature_builder& feature,
	MapDataObject& obj, std::pair<int, int> center, const std::string& osmandLayer,
	KeyIdx& keyIdx, ValueIdx& valueIdx, IntValueIdx& intValueIdx) {

	for (const auto& tag : obj.types)
		feature.add_property(keyIdx(tag.first), valueIdx(tag.second));
	for (const auto& tag : obj.additionalTypes)
		feature.add_property(keyIdx(tag.first), valueIdx(tag.second));
	// feature.add_property(keyIdx("osmand_layer"), valueIdx(osmandLayer)); // flatten into single layer
	for (const auto& name : obj.namesOrder)
	{
		const auto it = obj.objectNames.find(name);
		if (it != obj.objectNames.end())
			feature.add_property(keyIdx(name), valueIdx(it->second));
	}
	// feature.add_property(keyIdx("labelX"), intValueIdx(center.first)); // not supported by MVT renderer
	// feature.add_property(keyIdx("labelY"), intValueIdx(center.second)); // replaced with hide_caption
	feature.add_property(keyIdx("osm_id"), OsmAndObfConstants::getOsmIdFromBinaryMapObjectId(obj.id));
	if (center.first < 0 || center.first >= MVT_TILE_WIDTH || center.second < 0 || center.second >= MVT_TILE_WIDTH)
		feature.add_property(keyIdx("hide_caption"), valueIdx("yes")); // out-of-tile (later parsed by style)
}

inline std::string buildMapboxVectorTile(
	std::vector<FoundMapDataObject>& foundMapDataObjects, int x, int y, int zoom) {
	int s = 31 - zoom;
	const auto h = 1 << (std::max(s - 1, 0));
	const std::pair<int, int> corner(x << s, y << s);
	const std::pair<int, int> tl(std::max(corner.first - h, 0), std::max(corner.second - h, 0));
	const std::pair<int, int> br(
		static_cast<int>(std::min(((static_cast<int64_t>(x) + 1) << s) + h, static_cast<int64_t>(INT32_MAX))),
		static_cast<int>(std::min(((static_cast<int64_t>(y) + 1) << s) + h, static_cast<int64_t>(INT32_MAX))));
	s -= MVT_TILE_EXTENT_SHIFT;

	vtzero::tile_builder tile;

	vtzero::layer_builder layer{tile, "osmand"};
	const std::string osmandLayers[] = {"tunnel_layer", "ground_layer", "bridge_layer"};
	KeyIdx key_index(layer);
	ValueIdx value_index(layer);
	IntValueIdx int_value_index(layer);

	size_t i;
	std::pair<int, int> center, empty;
	coordinates temp;
	coordinates polygon;
	std::vector<coordinates> polylines;
	auto LABEL_SHIFT = 31 - LABEL_ZOOM_ENCODE;
	for (auto& foundMapDataObject : foundMapDataObjects) {
		auto& obj = *foundMapDataObject.obj;
	    int layer_idx = obj.getSimpleLayer() + 1;
		if (layer_idx < 0 || layer_idx > 2)
			continue;
		const auto& osmandLayer = osmandLayers[layer_idx];

		auto size = obj.points.size();
		if (size < 1)
			continue;

		const bool isArea = obj.area;
		const bool isPoint = size == 1;
		bool isCycle = obj.cycle();

        if (isPoint && !isArea) {
			auto p = obj.points[0];
			if (p.first < tl.first || p.first >= br.first || p.second < tl.second || p.second >= br.second)
				continue;
			vtzero::point_feature_builder feature(layer);
			feature.set_id(obj.id);
	        feature.add_point(scaleToTile(p.first, corner.first, s), scaleToTile(p.second, corner.second, s));
			center.first = scaleToTile(p.first + (obj.labelX * (1 << LABEL_SHIFT)), corner.first, s);
			center.second = scaleToTile(p.second + (obj.labelY * (1 << LABEL_SHIFT)), corner.second, s);
			addObjectDataToMapboxVectorTile(feature, obj, center, osmandLayer, key_index, value_index, int_value_index);
			feature.commit();
		} else if (!isPoint && (isArea || isCycle)) {
			if ((isCycle && size < 4) || size < 3)
				continue;
			clipPolygonForTile(obj.points, tl, br, corner, s, temp, polygon, center);
			center.first = scaleToTile(center.first + (obj.labelX * (1 << LABEL_SHIFT)), corner.first, s);
			center.second = scaleToTile(center.second + (obj.labelY * (1 << LABEL_SHIFT)), corner.second, s);
			size = polygon.size();
			if (size < 1)
				continue;
			isCycle = polygon.front() == polygon.back();
			if ((isCycle && size < 4) || size < 3)
				continue;
			vtzero::polygon_feature_builder feature(layer);
			feature.set_id(obj.id);
	        feature.add_ring(isCycle ? size : size + 1);
			for (auto p : polygon) {
				feature.set_point(p.first, p.second);
			}
			if (!isCycle)
				feature.set_point(polygon.front().first, polygon.front().second);
            for (const auto& ring : obj.polygonInnerCoordinates) {
				size = ring.size();
				isCycle = ring.front() == ring.back();
				if ((isCycle && size < 4) || size < 3)
					continue;
				clipPolygonForTile(ring, tl, br, corner, s, temp, polygon, empty);
				size = polygon.size();
				if (size < 1)
					continue;
				isCycle = polygon.front() == polygon.back();
				if ((isCycle && size < 4) || size < 3)
					continue;
				feature.add_ring(isCycle ? size : size + 1);
                for (auto p : polygon) {
					feature.set_point(p.first, p.second);
				}
				if (!isCycle)
					feature.set_point(polygon.front().first, polygon.front().second);
			}
			addObjectDataToMapboxVectorTile(feature, obj, center, osmandLayer, key_index, value_index, int_value_index);
	        feature.commit();
		} else if (!isPoint || !isArea) {
			if ((isCycle && size < 3) || size < 2)
				continue;
			clipPolylineForTile(obj.points, tl, br, corner, s, polylines, center);
			center.first = scaleToTile(center.first + (obj.labelX * (1 << LABEL_SHIFT)), corner.first, s);
			center.second = scaleToTile(center.second + (obj.labelY * (1 << LABEL_SHIFT)), corner.second, s);
			std::unique_ptr<vtzero::linestring_feature_builder> feature;
			bool withSegments = false;
			for (const auto& polyline : polylines) {
				size = polyline.size();
				if (size < 1)
					continue;
				isCycle = polyline.front() == polyline.back();
				if ((isCycle && size < 3) || size < 2)
					continue;
				if (!withSegments) {
					feature.reset(new vtzero::linestring_feature_builder(layer));
					feature->set_id(obj.id);
					withSegments = true;
				}
				feature->add_linestring(size);
				for (auto p : polyline) {
					feature->set_point(p.first, p.second);
				}
			}
			if (withSegments) {
				addObjectDataToMapboxVectorTile(*feature, obj, center, osmandLayer, key_index, value_index, int_value_index);
				feature->commit();
			}
        }
    }
	return tile.serialize();
}

#endif
