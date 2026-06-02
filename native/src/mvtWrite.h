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

inline void clipPolylineForTile(const coordinates& polyline,
	std::pair<int, int> topLeft, std::pair<int, int> bottomRight, std::vector<coordinates>& result) {
	result.clear();
	if (polyline.size() < 2)
		return;
    coordinates segment;
	const auto size = polyline.size();
	segment.reserve(size);
    auto prevPoint = polyline[0];
	int prevCode = computeOutCode(prevPoint, topLeft, bottomRight);
    for (size_t i = 1; i < size; i++) {
        auto nextPoint = polyline[i];
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
            if (segment.empty())
                segment.emplace_back(p0);
            else if (segment.back() != p0) {
				result.push_back(segment);
				segment.clear();
				segment.emplace_back(p0);
            }
            segment.emplace_back(p1);
        } else if (!segment.empty()) {
			result.push_back(segment);
			segment.clear();
        }
		prevPoint = nextPoint;
		prevCode = nextCode;
    }
    if (!segment.empty())
        result.push_back(std::move(segment));
}

template<bool IsY, bool IsGreater>
inline bool isOutside(std::pair<int, int> p, int limit) {
    if constexpr (IsY)
        return IsGreater ? (p.second > limit) : (p.second < limit);
    return IsGreater ? (p.first > limit) : (p.first < limit);
}

template<bool IsY, bool IsGreater>
inline bool clipPolygonAgainstEdge(const coordinates& polygon, coordinates& result, int limit) {
	auto s = polygon.back();
	bool sOutside = isOutside<IsY, IsGreater>(s, limit);
    for (const auto& p : polygon) {
		bool pOutside = isOutside<IsY, IsGreater>(p, limit);
        if (!pOutside) {
            if (sOutside) {
				if constexpr (IsY)
                	result.push_back(getIntersectionY(s, p, limit));
				else
					result.push_back(getIntersectionX(s, p, limit));
			}
            result.push_back(p);
        } else if (!sOutside) {
			if constexpr (IsY)
	            result.push_back(getIntersectionY(s, p, limit));
			else
				result.push_back(getIntersectionX(s, p, limit));
		}
        s = p;
		sOutside = pOutside;
    }
	if (result.size() < 3)
		return false;
	return true;
}

inline void clipPolygonForTile(const coordinates& polygon,
	std::pair<int, int> topLeft, std::pair<int, int> bottomRight, coordinates& temp, coordinates& result) {
	result.clear();
	const auto size = polygon.size();
	if (size < 3)
		return;
    result.reserve(size + 4);
	temp.clear();
	temp.reserve(size + 4);
	if (!clipPolygonAgainstEdge<true, false>(polygon, temp, topLeft.second))
		return;
	if (!clipPolygonAgainstEdge<false, false>(temp, result, topLeft.first)) {
		result.clear();
		return;
	}
	temp.clear();
	if (!clipPolygonAgainstEdge<true, true>(result, temp, bottomRight.second)) {
		result.clear();
		return;
	}
	result.clear();
	if (!clipPolygonAgainstEdge<false, true>(temp, result, bottomRight.first)) {
		result.clear();
		return;
	}
}

inline void addObjectDataToMapboxVectorTile(
	vtzero::feature_builder* feature, MapDataObject& obj, KeyIdx& keyIdx, ValueIdx& valueIdx) {

	for (const auto& tag : obj.types)
		feature->add_property(keyIdx(tag.first), valueIdx(tag.second));
	for (const auto& tag : obj.additionalTypes)
		feature->add_property(keyIdx(tag.first), valueIdx(tag.second));
	for (const auto& name : obj.namesOrder)
	{
		const auto it = obj.objectNames.find(name);
		if (it != obj.objectNames.end())
			feature->add_property(keyIdx(name), valueIdx(it->second));
	}
}

inline int scaleToTile(int coord, int tileStart, int shift) {
	if (shift <= 0)
		return (coord - tileStart) << (-shift);
	int delta = coord - tileStart;
	return (delta >> shift) + (delta >> (shift - 1) & 1);
}

inline void makeNewScaledPoint(std::pair<int, int> point, std::pair<int, int> tileCorner, int shift,
	std::pair<int, int>& scaled) {
	std::pair<int, int> result(
		scaleToTile(point.first, tileCorner.first, shift),
		scaleToTile(point.second, tileCorner.second, shift));
	if (result == scaled) {
		result.first ^= 1;
	}
	scaled = result;
}

inline void makeNewOriginalPoint(std::pair<int, int> point, std::pair<int, int> tileCorner, int shift,
	std::pair<int, int> start, std::pair<int, int>& scaled) {
	std::pair<int, int> result(
		scaleToTile(point.first, tileCorner.first, shift),
		scaleToTile(point.second, tileCorner.second, shift));
	if (result == scaled) {
		result.first ^= 1;
		if (result == start)
			result.second ^= 1;
	} else if (result == start) {
		result.second ^= 1;
		if (result == scaled)
			result.first ^= 1;
	}
	scaled = result;
}

inline std::string buildMapboxVectorTile(
	std::vector<FoundMapDataObject>& foundMapDataObjects, int x, int y, int zoom) {
	int s = 31 - zoom;
	std::pair<int, int> tl(x << s, y << s);
	std::pair<int, int> br(
		static_cast<int>(std::min((static_cast<int64_t>(x) + 1) << s, static_cast<int64_t>(INT32_MAX))),
		static_cast<int>(std::min((static_cast<int64_t>(y) + 1) << s, static_cast<int64_t>(INT32_MAX))));
	s -= 12;

	vtzero::tile_builder tile;

	vtzero::layer_builder layers[] = {{tile, "tunnel_layer"}, {tile, "ground_layer"}, {tile, "bridge_layer"}};
	KeyIdx key_indices[] = {KeyIdx(layers[0]), KeyIdx(layers[1]), KeyIdx(layers[2])};
	ValueIdx value_indices[] = {ValueIdx(layers[0]), ValueIdx(layers[1]), ValueIdx(layers[2])};

	size_t i;
	std::pair<int, int> f, p;
	coordinates temp;
	coordinates polygon;
	std::vector<coordinates> polylines;
	for (auto& foundMapDataObject : foundMapDataObjects) {
		auto& obj = *foundMapDataObject.obj;
        int layer_idx = obj.getSimpleLayer() + 1;
		if (layer_idx < 0 || layer_idx > 2)
			continue;
		auto& key_index = key_indices[layer_idx];
		auto& value_index = value_indices[layer_idx];

		auto size = obj.points.size();
		if (size < 1)
			continue;

		const bool isArea = obj.area;
		const bool isPoint = size == 1;
		bool isCycle = obj.cycle();

        if (isPoint && !isArea) {
			p = obj.points[0];
			if (p.first < tl.first || p.first >= br.first || p.second < tl.second || p.second >= br.second)
				continue;
			vtzero::point_feature_builder feature(layers[layer_idx]);
			feature.set_id(obj.id);
            feature.add_point(scaleToTile(p.first, tl.first, s), scaleToTile(p.second, tl.second, s));
			addObjectDataToMapboxVectorTile(&feature, obj, key_index, value_index);
			feature.commit();
		} else if (!isPoint && (isArea || isCycle)) {
			if ((isCycle && size < 4) || size < 3)
				continue;
			clipPolygonForTile(obj.points, tl, br, temp, polygon);
			size = polygon.size();
			if (size < 1)
				continue;
			isCycle = polygon.front() == polygon.back();
			if ((isCycle && size < 4) || size < 3)
				continue;
			vtzero::polygon_feature_builder feature(layers[layer_idx]);
			feature.set_id(obj.id);
            feature.add_ring(isCycle ? size : size + 1);
			f = {-1, -1};
			makeNewScaledPoint(polygon[0], tl, s, f);
			feature.set_point(f.first, f.second);
			p = f;
			for (i = 1; i < size - 2; i++) {
				makeNewScaledPoint(polygon[i], tl, s, p);
				feature.set_point(p.first, p.second);
			}
			if (isCycle)
				makeNewOriginalPoint(polygon[i], tl, s, f, p);
			else
				makeNewScaledPoint(polygon[i], tl, s, p);
			feature.set_point(p.first, p.second);
			if (!isCycle) {
				makeNewOriginalPoint(polygon[i + 1], tl, s, f, p);
				feature.set_point(p.first, p.second);
			}
			feature.set_point(f.first, f.second);
            for (const auto& ring : obj.polygonInnerCoordinates) {
				size = ring.size();
				isCycle = ring.front() == ring.back();
				if ((isCycle && size < 4) || size < 3)
					continue;
				clipPolygonForTile(ring, tl, br, temp, polygon);
				size = polygon.size();
				if (size < 1)
					continue;
				isCycle = polygon.front() == polygon.back();
				if ((isCycle && size < 4) || size < 3)
					continue;
				feature.add_ring(isCycle ? size : size + 1);
				f = {-1, -1};
				makeNewScaledPoint(polygon[0], tl, s, f);
				feature.set_point(f.first, f.second);
				p = f;
                for (i = 1; i < size - 2; i++) {
					makeNewScaledPoint(polygon[i], tl, s, p);
					feature.set_point(p.first, p.second);
				}
				if (isCycle)
					makeNewOriginalPoint(polygon[i], tl, s, f, p);
				else
					makeNewScaledPoint(polygon[i], tl, s, p);
				feature.set_point(p.first, p.second);
				if (!isCycle) {
					makeNewOriginalPoint(polygon[i + 1], tl, s, f, p);
					feature.set_point(p.first, p.second);
				}
				feature.set_point(f.first, f.second);
			}
			addObjectDataToMapboxVectorTile(&feature, obj, key_index, value_index);
	        feature.commit();
		} else if (!isPoint || !isArea) {
			if ((isCycle && size < 3) || size < 2)
				continue;
			clipPolylineForTile(obj.points, tl, br, polylines);
			for (const auto& polyline : polylines) {
				size = polyline.size();
				if (size < 1)
					continue;
				isCycle = polyline.front() == polyline.back();
				if ((isCycle && size < 3) || size < 2)
					continue;
				vtzero::linestring_feature_builder feature(layers[layer_idx]);
				feature.set_id(obj.id);
				feature.add_linestring(size);
				p = {-1, -1};
				for (auto point : polyline) {
					makeNewScaledPoint(point, tl, s, p);
					feature.set_point(p.first, p.second);
				}
				addObjectDataToMapboxVectorTile(&feature, obj, key_index, value_index);
				feature.commit();
			}
        }
    }

	return tile.serialize();
}

#endif
