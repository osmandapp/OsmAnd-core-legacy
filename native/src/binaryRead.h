#ifndef _OSMAND_BINARY_READ_H
#define _OSMAND_BINARY_READ_H

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <algorithm>
#if defined(_WIN32)
#include <io.h>
#else
#include <unistd.h>
#endif
#include <stdint.h>
#include <stdio.h>

#include <fstream>
#include <map>
#include <string>

#include "CommonCollections.h"
#include "commonOsmAndCore.h"
#include "multipolygons.h"
#include "routeTypeRule.h"
#include "transportRoutingObjects.h"

#if defined(WIN32)
#define close _close
#endif
#include "renderRules.h"

#ifdef _IOS_BUILD
#include <OsmAndCore/ICU.h>
#include <OsmAndCore/Logging.h>
#else
#include "Logging.h"
#endif

enum class GeneralRouterProfile;
static const uint MAP_VERSION = 2;
static const int SHIFT_ID = 6;
struct MapTreeBounds {
	uint64_t length;
	uint64_t filePointer;
	uint64_t mapDataBlock;
	uint32_t left;
	uint32_t right;
	uint32_t top;
	uint32_t bottom;

	MapTreeBounds() {
	}
};
struct RestrictionInfo {
	uint64_t to;
	uint64_t via;
	uint32_t type;

	RestrictionInfo() : to(0), via(0), type(0) {
	}
};

struct RoutingIndex;
struct RouteSubregion {
	uint64_t length;
	uint64_t filePointer;
	uint64_t mapDataBlock;
	uint32_t left;
	uint32_t right;
	uint32_t top;
	uint32_t bottom;
	std::vector<RouteSubregion> subregions;
	SHARED_PTR<RoutingIndex> routingIndex;

	RouteSubregion(const SHARED_PTR<RoutingIndex>& ind) : length(0), filePointer(0), mapDataBlock(0), routingIndex(ind) {
	}
};

struct MapRoot : MapTreeBounds {
	uint minZoom;
	uint maxZoom;
	std::vector<MapTreeBounds> bounds;
};

enum PART_INDEXES {
	MAP_INDEX = 1,
	POI_INDEX,
	ADDRESS_INDEX,
	TRANSPORT_INDEX,
	ROUTING_INDEX,
    HH_INDEX
};

struct BinaryPartIndex {
	uint64_t length;
	uint64_t filePointer;
	PART_INDEXES type;
	std::string name;

	BinaryPartIndex(PART_INDEXES tp) : type(tp) {
	}
};

struct RoutingIndex : BinaryPartIndex {
	vector<RouteTypeRule> routeEncodingRules;
	UNORDERED_map<std::string, uint32_t> decodingRules;
	std::vector<RouteSubregion> subregions;
	std::vector<RouteSubregion> basesubregions;

	int nameTypeRule;
	int refTypeRule;
	int destinationTypeRule;
	int destinationRefTypeRule;
	int directionForward = -1;
	int directionBackward = -1;
	int maxheightForward = -1;
	int maxheightBackward = -1;
	int directionTrafficSignalsForward = -1;
	int directionTrafficSignalsBackward = -1;
	int trafficSignals = -1;
	int stopSign = -1;
	int stopMinor = -1;
	int giveWaySign = -1;

	RoutingIndex()
		: BinaryPartIndex(ROUTING_INDEX), nameTypeRule(-1), refTypeRule(-1), destinationTypeRule(-1),
		  destinationRefTypeRule(-1) {
	}

	void initRouteEncodingRule(uint32_t id, std::string tag, std::string val);

	void completeRouteEncodingRules();

	uint32_t findOrCreateRouteType(const std::string& tag, const std::string& value);

	uint32_t searchRouteEncodingRule(const std::string& tag, const std::string& value);

	RouteTypeRule& quickGetEncodingRule(uint32_t id) {
		return routeEncodingRules[id];
	}
};

struct HHRoutePointsBox {
    uint64_t length;
    uint64_t filePointer;
    int32_t left, right, bottom, top;
    bool init;
    
    HHRoutePointsBox(): length(0), filePointer(0), left(0), right(0), bottom(0), top(0), init(false) {
    }

    SkRect getSkRect() {
        SkRect qr = SkRect::MakeLTRB(left, top, right, bottom);
        return qr;
    }

    bool contains(int x, int y) {
        return x >= left && x <= right && y >= top && y <= bottom;
    }
};

struct HHRouteBlockSegments {
    int idRangeStart;
    int32_t idRangeLength;
    int profileId;
    uint64_t length;
    uint64_t filePointer;
    
    ~HHRouteBlockSegments() {
        for (auto & s : sublist) {
            delete s;
        }
    }
    
    std::vector<HHRouteBlockSegments *> sublist;
};

struct TagValuePair {
    std::string tag;
    std::string value;
    int32_t additionalAttribute;
    
    TagValuePair(string tag, string value, int32_t additionalAttribute): tag(tag), value(value), additionalAttribute(additionalAttribute) {
    }
};

struct HHRouteIndex : BinaryPartIndex {
    //HHRouteRegion
    uint64_t edition;
    std::string profile;
    std::vector<std::string> profileParams;
    SHARED_PTR<HHRoutePointsBox> top;
    // not stored in cache
    std::vector<HHRouteBlockSegments *> segments;
    SkRect * rect;
    std::vector<TagValuePair> encodingRules;
    
    HHRouteIndex() : BinaryPartIndex(HH_INDEX), edition(0), profile(""), rect(nullptr) {
    }
    
    ~HHRouteIndex() {
        for (auto & s : segments) {
            delete s;
        }
        delete rect;
    }
    
    SkRect * getSkRect() {
        if (rect == nullptr) {
            if (!top) {
                rect = new SkRect();
            } else {
                rect = new SkRect(top->getSkRect());
            }
        }
        return rect;
    }
    
    double intersectionArea(SkRect b) {
        if (rect == nullptr) {
            rect = getSkRect();
        }
        double xleft = std::max(std::min(rect->left(), rect->right()), std::min(b.left(), b.right()));
        double xright = std::min(std::max(rect->left(), rect->right()), std::max(b.left(), b.right()));
        double ytop = std::max(std::min(rect->top(), rect->bottom()), std::min(b.top(), b.bottom()));
        double ybottom = std::min(std::max(rect->top(), rect->bottom()), std::max(b.top(), b.bottom()));
        if (xright <= xleft || ybottom <= ytop) {
            return 0;
        }
        double intersectionArea = (xright - xleft) * (ybottom - ytop);
        return intersectionArea;
    }
    
    HHRouteBlockSegments * createHHRouteBlockSegments() {
        // all segments stored in std::vector<HHRouteBlockSegments *> segments
        HHRouteBlockSegments * np = new HHRouteBlockSegments();
        return np;
    }
};

struct RouteDataObject {
	const static int RESTRICTION_SHIFT = 3;
	const static uint64_t RESTRICTION_MASK = 7;
	const static int HEIGHT_UNDEFINED = -80000;

	SHARED_PTR<RoutingIndex> region;
	std::vector<uint32_t> types;
	std::vector<uint32_t> pointsX;
	std::vector<uint32_t> pointsY;
	std::vector<RestrictionInfo> restrictions;
	std::vector<std::vector<uint32_t>> pointTypes;
	std::vector<std::vector<uint32_t>> pointNameTypes;
	std::vector<std::vector<uint32_t>> pointNameIds;
	std::vector<std::vector<std::string>> pointNames;
	std::vector<double> heightDistanceArray;
	int64_t id;

	void setPointTypes(int pntInd, std::vector<uint32_t> array) {
		if (pointTypes.size() <= pntInd) {
			std::vector<std::vector<uint32_t>> npointTypes(pntInd + 1);
			for (int k = 0; k < pointTypes.size(); k++) {
				npointTypes[k] = pointTypes[k];
			}
			pointTypes = npointTypes;
		}
		pointTypes[pntInd] = array;
	}

	UNORDERED(map)<int, std::string> names;
	vector<pair<uint32_t, uint32_t>> namesIds;

	RouteDataObject() : region(nullptr), id(0) {
	}

	RouteDataObject(const SHARED_PTR<RoutingIndex>& region) : region(region), id(0) {
	}

	RouteDataObject(SHARED_PTR<RouteDataObject>& copy) {
		region = copy->region;
		pointsX = copy->pointsX;
		pointsY = copy->pointsY;
		types = copy->types;
		names = copy->names;
		namesIds = copy->namesIds;
		restrictions = copy->restrictions;
		restrictions = copy->restrictions;
		pointTypes = copy->pointTypes;
		pointNames = copy->pointNames;
		pointNameTypes = copy->pointNameTypes;
		id = copy->id;
	}

	~RouteDataObject() {
	}

	inline string getName() {
		if (names.size() > 0) {
			return names.begin()->second;
		}
		return "";
	}

	inline string getName(string& lang, bool transliter) {
		if (!names.empty()) {
			if (lang.empty()) {
				return names[region->nameTypeRule];
			}
			for (auto it = names.begin(); it != names.end(); ++it) {
				int k = it->first;
				if (region->routeEncodingRules.size() > k) {
					if (("name:" + lang) == region->routeEncodingRules[k].getTag()) {
						return names[k];
					}
				}
			}
			string nmDef = names[region->nameTypeRule];
			if (transliter && !nmDef.empty()) {
				return transliterate(nmDef);
			}
			return nmDef;
		}
		return "";
	}

	inline string getName(string& lang) {
		return getName(lang, false);
	}

	inline string getRef(string& lang, bool transliter, bool direction) {
		if (!names.empty()) {
			if (lang.empty()) {
				return names[region->refTypeRule];
			}
			for (auto it = names.begin(); it != names.end(); ++it) {
				int k = it->first;
				if (region->routeEncodingRules.size() > k) {
					if (("ref:" + lang) == region->routeEncodingRules[k].getTag()) {
						return names[k];
					}
				}
			}
			string refDefault = names[region->refTypeRule];
			if (transliter && !refDefault.empty() && refDefault.length() > 0) {
				return transliterate(refDefault);
			}
			return refDefault;
		}
		return "";
	}

	inline string getDestinationRef(bool direction) {
		if (!names.empty()) {
			string refTag = (direction == true) ? "destination:ref:forward" : "destination:ref:backward";
			string refTagDefault = "destination:ref";
			string refDefault = "";

			for (auto it = names.begin(); it != names.end(); ++it) {
				int k = it->first;
				if (region->routeEncodingRules.size() > k) {
					if (refTag == region->routeEncodingRules[k].getTag()) {
						return names[k];
					}
					if (refTagDefault == region->routeEncodingRules[k].getTag()) {
						refDefault = names[k];
					}
				}
			}
			if (!refDefault.empty()) {
				return refDefault;
			}
			// return names.get(region.refTypeRule);
		}
		return "";
	}

	inline bool isExitPoint() {
		const auto ptSz = pointTypes.size();
		for (int i = 0; i < ptSz; i++) {
			const auto& point = pointTypes[i];
			const auto pSz = point.size();
			for (int j = 0; j < pSz; j++) {
				auto k = point[j];
				if (region->routeEncodingRules.size() > k && region->routeEncodingRules[k].getValue() == "motorway_junction") {
					return true;
				}
			}
		}
		return false;
	}

	inline string getExitName() {
		const auto pnSz = pointNames.size();
		for (int i = 0; i < pnSz; i++) {
			const auto& point = pointNames[i];

			const auto pSz = point.size();
			for (int j = 0; j < pSz; j++) {
				if (pointNameTypes[i][j] == region->nameTypeRule) {
					return point[j];
				}
			}
		}
		return "";
	}

	inline string getExitRef() {
		const auto pnSz = pointNames.size();
		for (int i = 0; i < pnSz; i++) {
			const auto& point = pointNames[i];
			const auto pSz = point.size();
			for (int j = 0; j < pSz; j++) {
				if (pointNameTypes[i][j] == region->refTypeRule) {
					return point[j];
				}
			}
		}
		return "";
	}

	inline bool hasNameTagStartsWith(string tagStartsWith) {
		for (int nm = 0; nm < namesIds.size(); nm++) {
			if (namesIds[nm].first >= region->routeEncodingRules.size())
				continue;
			auto& rtr = region->quickGetEncodingRule(namesIds[nm].first);
			if (rtr.getTag().rfind(tagStartsWith, 0) == 0) {
				return true;
			}
		}
		return false;
	}

	void removePointType(int ind, int type) {
		if (ind < pointTypes.size()) {
			std::vector<uint32_t>& indArr = pointTypes[ind];
			std::vector<uint32_t>::iterator it = std::find(indArr.begin(), indArr.end(), type);
			if (it != indArr.end()) {
				indArr.erase(it);
			}
		}
	}

	void processConditionalTags(const tm& time);

   #ifdef _IOS_BUILD
	inline string transliterate(const string& s) {
		QString transliterateName = OsmAnd::ICU::transliterateToLatin(QString::fromStdString(s));
		return transliterateName.toStdString();
	}
   #endif

   #ifndef _IOS_BUILD
   inline string transliterate(const string& s) {
	   return s;
   }
   #endif

	inline string getDestinationName(string& lang, bool translit, bool direction) {
		if (!names.empty()) {			
			map<string, int> tagPriorities;
			string directionStr = direction ? "forward" : "backward";
			int tagPriority = 1;
			if (!lang.empty()) {
				tagPriorities["destination:lang:" + lang + ":" + directionStr] = tagPriority++;
			}
			tagPriorities["destination:" + directionStr] = tagPriority++;
			if (!lang.empty()) {
				tagPriorities["destination:lang:" + lang] = tagPriority++;
			}
			tagPriorities["destination"] = tagPriority;

			int highestPriorityNameKey = -1;
			int highestPriority = numeric_limits<int>::max();
			
			for (const auto& entry : names) {
				int nameKey = entry.first;
				if (region->routeEncodingRules.size() > nameKey) {
					string tag = region->routeEncodingRules[nameKey].getTag();
					auto priority = tagPriorities.find(tag);
					if (priority != tagPriorities.end() && priority->second < highestPriority) {
						highestPriority = priority->second;
						highestPriorityNameKey = nameKey;
					}
				}
			}
			if (highestPriorityNameKey > 0) {
				string name = names[highestPriorityNameKey];
				return translit ? transliterate(name) : name;
			}
		}
		return "";
	}

	inline int64_t getId() {
		return id;
	}

	int getSize() {
		int s = sizeof(this);
		s += pointsX.capacity() * sizeof(uint32_t);
		s += pointsY.capacity() * sizeof(uint32_t);
		s += types.capacity() * sizeof(uint32_t);
		s += restrictions.capacity() * sizeof(uint64_t);
		std::vector<std::vector<uint32_t>>::iterator t = pointTypes.begin();
		for (; t != pointTypes.end(); t++) {
			s += (*t).capacity() * sizeof(uint32_t);
		}
		t = pointNameTypes.begin();
		for (; t != pointNameTypes.end(); t++) {
			s += (*t).capacity() * sizeof(uint32_t);
		}
		std::vector<std::vector<std::string>>::iterator ts = pointNames.begin();
		for (; ts != pointNames.end(); ts++) {
			s += (*ts).capacity() * 10;
		}
		s += namesIds.capacity() * sizeof(pair<uint32_t, uint32_t>);
		s += names.size() * sizeof(pair<int, string>) * 10;
		return s;
	}

	inline int getLanes() {
		auto sz = types.size();
		for (int i = 0; i < sz; i++) {
			auto& r = region->quickGetEncodingRule(types[i]);
			int ln = r.lanes();
			if (ln > 0) {
				return ln;
			}
		}
		return -1;
	}

	inline int getRestrictionLength() {
		return restrictions.empty() ? 0 : (int)restrictions.size();
	}

	inline int getRestrictionType(int i) {
		return restrictions[i].type;
	}

	inline long getRestrictionId(int i) {
		return restrictions[i].to;
	}

	inline long getRestrictionVia(int i) {
		return restrictions[i].via;
	}

	bool tunnel();
	int getOneway();
	string getValue(const string& tag);
	string getValue(uint32_t pnt, const string& tag);

	inline int getPointsLength() {
		return (int)pointsX.size();
	}

	inline int getPoint31XTile(int i) {
		return pointsX[i];
	}

	inline int getPoint31XTile(int s, int e) {
		return pointsX[s] / 2 + pointsX[e] / 2;
	}

	inline int getPoint31YTile(int i) {
		return pointsY[i];
	}

	inline int getPoint31YTile(int s, int e) {
		return pointsY[s] / 2 + pointsY[e] / 2;
	}

	std::vector<uint32_t> getPointTypes(int ind) {
		if (ind >= pointTypes.size()) {
			return {};
		}
		return pointTypes[ind];
	}

	void insert(int pos, int x31, int y31) {
		pointsX.insert(pointsX.begin() + pos, x31);
		pointsY.insert(pointsY.begin() + pos, y31);
		if (pointTypes.size() > pos) {
			std::vector<uint32_t> types;
			pointTypes.insert(pointTypes.begin() + pos, types);
		}
	}

	std::vector<double> calculateHeightArray();

	string getHighway();

	bool hasPrivateAccess(GeneralRouterProfile profile);

	bool platform();

	bool roundabout();

	bool hasTrafficLightAt(int i);

	double simplifyDistance(int x, int y, int px, int py) {
		return abs(px - x) * 0.011 + abs(py - y) * 0.01863;
	}

	double distance(int startPoint, int endPoint) {
		if (startPoint > endPoint) {
			int k = endPoint;
			endPoint = startPoint;
			startPoint = k;
		}
		double d = 0;
		for (int k = startPoint; k < endPoint && k < getPointsLength() - 1; k++) {
			int x = pointsX[k];
			int y = pointsY[k];
			int kx = pointsX[k + 1];
			int ky = pointsY[k + 1];
			d += simplifyDistance(kx, ky, x, y);
		}
		return d;
	}

	float getMaximumSpeed(bool direction) {
		return getMaximumSpeed(direction, RouteTypeRule::PROFILE_NONE);
	}

	float getMaximumSpeed(bool direction, int profile) {
		float maxSpeed = 0, maxProfileSpeed = 0;
		for (int type : types) {
			RouteTypeRule r = region->quickGetEncodingRule(type);
			bool forwardDirection = r.isForward() >= 0;
			if ((direction && r.isForward() == 1) ||
				(!direction && r.isForward() == -1) ||
				r.isForward() == 0) {
				// priority over default
				maxSpeed = r.maxSpeed(RouteTypeRule::PROFILE_NONE) > 0 ? r.maxSpeed(RouteTypeRule::PROFILE_NONE) : maxSpeed;
				maxProfileSpeed = r.maxSpeed(profile) > 0 ? r.maxSpeed(profile) : maxProfileSpeed;
			}
		}
		return maxProfileSpeed > 0 ? maxProfileSpeed : maxSpeed;
	}

	double directionRoute(int startPoint, bool plus) {
		// look at comment JAVA
		return directionRoute(startPoint, plus, 5);
	}

	bool bearingVsRouteDirection(double bearing) {
		bool direction = true;
		if (bearing >= 0) {
			double diff = alignAngleDifference(directionRoute(0, true) - bearing / 180.f * M_PI);
			direction = fabs(diff) < M_PI / 2.f;
		}
		return direction;
	}

	bool isRoadDeleted() {
		auto sz = types.size();
		for (int i = 0; i < sz; i++) {
			auto& r = region->quickGetEncodingRule(types[i]);
			if ("osmand_change" == r.getTag() && "delete" == r.getValue()) {
				return true;
			}
		}
		return false;
	}

	bool isDirectionApplicable(bool direction, int ind, int startPointInd, int endPointInd) {
		auto pt = pointTypes[ind];
		auto sz = pt.size();
		for (int i = 0; i < sz; i++) {
			auto& r = region->quickGetEncodingRule(pt[i]);
			// Evaluate direction tag if present
			if ("direction" == r.getTag()) {
				auto dv = r.getValue();
				if ((dv == "forward" && direction) || (dv == "backward" && !direction)) {
					return true;
				} else if ((dv == "forward" && !direction) || (dv == "backward" && direction)) {
					return false;
				}
			}
		}
		if (startPointInd >= 0 ) {
			// Heuristic fallback: Distance analysis for STOP with no recognized directional tagging:
			// Mask STOPs closer to the start than to the end of the routing segment if it is within 50m of start, but do
			// not mask STOPs mapped directly on start/end (likely intersection node)
			auto d2Start = distance(startPointInd, ind);
			auto d2End = distance(ind, endPointInd);
			if ((d2Start < d2End) && d2Start != 0 && d2End != 0 && d2Start < 50) {
				return false;
			}
		}
		// No directional info detected
		return true;
	}

	// Gives route direction of EAST degrees from NORTH ]-PI, PI]
	double directionRoute(int startPoint, bool plus, float dist) {
		int x = pointsX[startPoint];
		int y = pointsY[startPoint];
		int nx = startPoint;
		int px = x;
		int py = y;
		double total = 0;
		do {
			if (plus) {
				nx++;
				if (nx >= (int)pointsX.size()) {
					break;
				}
			} else {
				nx--;
				if (nx < 0) {
					break;
				}
			}
			px = pointsX[nx];
			py = pointsY[nx];
			// translate into meters
			total += abs(px - x) * 0.011 + abs(py - y) * 0.01863;
		} while (total < dist);
		return -atan2(x - px, y - py);
	}

	static double parseLength(string v, double def) {
		double f = 0;
		// 14'10" 14 - inches, 10 feet
		int i = findFirstNumberEndIndex(v);
		if (i > 0) {
			f += strtod_li(v.substr(0, i));
			string pref = v.substr(i, v.length());
			float add = 0;
			for (int ik = 0; ik < pref.length(); ik++) {
				if ((pref[ik] >= '0' && pref[ik] <= '9') || pref[ik] == '.' || pref[ik] == '-') {
					int first = findFirstNumberEndIndex(pref.substr(ik));
					if (first != -1) {
						add = parseLength(pref.substr(ik), 0);
						pref = pref.substr(0, ik);
					}
					break;
				}
			}
			if (pref.find("km") != string::npos) {
				f *= 1000;
			}
			if (pref.find("in") != string::npos || pref.find("\"") != string::npos) {
				f *= 0.0254;
			} else if (pref.find("\'") != string::npos || pref.find("ft") != string::npos ||
					   pref.find("feet") != string::npos) {
				// foot to meters
				f *= 0.3048;
			} else if (pref.find("cm") != string::npos) {
				f *= 0.01;
			} else if (pref.find("mile") != string::npos) {
				f *= 1609.34f;
			}
			return f + add;
		}
		return def;
	}

	//	static double parseLength(string v, double def) {
	//		// 14"10' not supported
	//		int i = findFirstNumberEndIndex(v);
	//		if (i > 0) {
	//			double f = strtod_li(v.substr(0, i));
	//			if (v.find("\"") != string::npos  || v.find("ft") != string::npos) {
	//				// foot to meters
	//				f *= 0.3048;
	//			}
	//			return f;
	//		}
	//		return def;
	//	}

	static double parseWeightInTon(string v, double def) {
		int i = findFirstNumberEndIndex(v);
		if (i > 0) {
			double f = strtod_li(v.substr(0, i));
			if (v.find("\"") != string::npos || v.find("lbs") != string::npos) {
				// lbs -> kg -> ton
				f = (f * 0.4535) / 1000.0;
			}
			return f;
		}
		return def;
	}

	bool hasPointType(int pntId, uint32_t type) {
		for (int k = 0; pointTypes.size() > pntId && k < pointTypes[pntId].size(); k++) {
			if (pointTypes[pntId][k] == type) {
				return true;
			}
		}
		return false;
	}
};

struct IndexStringTable {
	uint64_t fileOffset;
	uint32_t length;
	UNORDERED(map)<int32_t, string> stringTable;
};

struct TransportIndex : BinaryPartIndex {
	int left;
	int right;
	int top;
	int bottom;

	uint64_t stopsFileOffset;
	uint64_t stopsFileLength;
	uint64_t incompleteRoutesOffset;
	uint64_t incompleteRoutesLength;

	IndexStringTable* stringTable;

	TransportIndex() : BinaryPartIndex(TRANSPORT_INDEX), left(0), right(0), top(0), bottom(0) {
	}
};

struct MapIndex : BinaryPartIndex {
	std::vector<MapRoot> levels;

	UNORDERED(map)<int, tag_value> decodingRules;
	// DEFINE hash
	// UNORDERED(map)<tag_value, int> encodingRules;

	int nameEncodingType;
	int refEncodingType;
	int coastlineEncodingType;
	int coastlineBrokenEncodingType;
	int landEncodingType;
	int onewayAttribute;
	int onewayReverseAttribute;
	UNORDERED(set)<int> positiveLayers;
	UNORDERED(set)<int> negativeLayers;

	MapIndex() : BinaryPartIndex(MAP_INDEX) {
		nameEncodingType = refEncodingType = coastlineBrokenEncodingType = coastlineEncodingType = -1;
		landEncodingType = onewayAttribute = onewayReverseAttribute = -1;
	}

	void finishInitializingTags() {
		int free = (int)decodingRules.size() * 2 + 1;
		coastlineBrokenEncodingType = free++;
		initMapEncodingRule(0, coastlineBrokenEncodingType, "natural", "coastline_broken");
		if (landEncodingType == -1) {
			landEncodingType = free++;
			initMapEncodingRule(0, landEncodingType, "natural", "land");
		}
	}

	void initMapEncodingRule(uint32_t type, uint32_t id, std::string tag, std::string val) {
		tag_value pair = tag_value(tag, val);
		// DEFINE hash
		// encodingRules[pair] = id;
		decodingRules[id] = pair;

		if ("name" == tag) {
			nameEncodingType = id;
		} else if ("natural" == tag && "coastline" == val) {
			coastlineEncodingType = id;
		} else if ("natural" == tag && "land" == val) {
			landEncodingType = id;
		} else if ("oneway" == tag && "yes" == val) {
			onewayAttribute = id;
		} else if ("oneway" == tag && "-1" == val) {
			onewayReverseAttribute = id;
		} else if ("ref" == tag) {
			refEncodingType = id;
		} else if ("layer" == tag) {
			if (val != "" && val != "0") {
				if (val[0] == '-') {
					negativeLayers.insert(id);
				} else {
					positiveLayers.insert(id);
				}
			}
		}
	}
};

struct BinaryMapFile {
	std::string inputName;
	uint32_t version;
	uint64_t dateCreated;
	std::vector<SHARED_PTR<MapIndex>> mapIndexes;
	std::vector<SHARED_PTR<RoutingIndex>> routingIndexes;
	std::vector<SHARED_PTR<TransportIndex>> transportIndexes;
	std::vector<SHARED_PTR<BinaryPartIndex>> indexes;
    std::vector<SHARED_PTR<HHRouteIndex>> hhIndexes;
	UNORDERED(map)<uint64_t, shared_ptr<IncompleteTransportRoute>> incompleteTransportRoutes;
	bool incompleteLoaded = false;
	int fd = -1;
	int routefd = -1;
	int geocodingfd = -1;
    int hhfd = -1;
	bool basemap;
	bool external;
	bool roadOnly;
	bool liveMap;

	int openFile() {
#if defined(_WIN32)
		int fileDescriptor = open(inputName.c_str(), O_RDONLY | O_BINARY);
#else
		int fileDescriptor = open(inputName.c_str(), O_RDONLY);
#endif
		if (fileDescriptor < 0) {
			OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, " Native File could not be open to read: %s",
							  inputName.c_str());
		} else {
			OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "Native open file: %s", inputName.c_str());
		}
		return fileDescriptor;
	}

	int getFD() {
		if (fd <= 0) {
			fd = openFile();
		}
		return fd;
	}

	int getRouteFD() {
		if (routefd <= 0) {
			routefd = openFile();
		}
		return routefd;
	}

	int getGeocodingFD() {
		if (geocodingfd <= 0) {
			geocodingfd = openFile();
		}
		return geocodingfd;
	}
    
    int getHhFD() {
        if (hhfd <= 0) {
            hhfd = openFile();
        }
        return hhfd;
    }

	bool isBasemap() {
		return basemap;
	}

	bool isExternal() {
		return external;
	}

	bool isLiveMap() {
		return liveMap;
	}
	bool isRoadOnly() {
		return roadOnly;
	}

	~BinaryMapFile() {
		if (fd >= 0) {
			close(fd);
		}
		if (routefd >= 0) {
			close(routefd);
		}
		if (geocodingfd >= 0) {
			close(geocodingfd);
		}
	}
};

struct ResultPublisher {
	std::vector<FoundMapDataObject> result;
	UNORDERED(map)<uint64_t, FoundMapDataObject> ids;

	bool publish(FoundMapDataObject r);

	shared_ptr<vector<MapDataObject*>> getObjects() {
		shared_ptr<vector<MapDataObject*>> objs(new vector<MapDataObject*>);
		objs->reserve(result.size());
		for (uint i = 0; i < result.size(); i++) {
			objs->push_back(result[i].obj);
		}
		return objs;
	}

	bool publishOnlyUnique(std::vector<FoundMapDataObject>& r) {
		for (uint i = 0; i < r.size(); i++) {
			if (!publish(r[i])) {
				delete r[i].obj;
			}
		}
		r.clear();
		return true;
	}

	bool publishAll(std::vector<FoundMapDataObject>& r) {
		for (uint i = 0; i < r.size(); i++) {
			result.push_back(r[i]);
		}
		r.clear();
		return true;
	}

	void clear() {
		result.clear();
		ids.clear();
	}

	bool isCancelled() {
		return false;
	}
	virtual ~ResultPublisher() {
		deleteObjects(result);
	}
};

struct SearchQuery {
	RenderingRuleSearchRequest* req;
	int left;
	int right;
	int top;
	int bottom;
	int oceanLeft;
	int oceanRight;
	int oceanTop;
	int oceanBottom;
	uint zoom;
	ResultPublisher* publisher;

	coordinates cacheCoordinates;
	uint ocean;
	uint oceanTiles;

	uint numberOfVisitedObjects;
	uint numberOfAcceptedObjects;
	uint numberOfReadSubtrees;
	uint numberOfAcceptedSubtrees;

	int limit;

	vector<SHARED_PTR<TransportStop>> transportResults;

	// cache information
	vector<int32_t> cacheTypes;
	vector<int64_t> cacheIdsA;
	vector<int64_t> cacheIdsB;

	SearchQuery(int l, int r, int t, int b, RenderingRuleSearchRequest* req, ResultPublisher* publisher)
		: req(req), left(l), right(r), top(t), bottom(b), publisher(publisher) {
		numberOfAcceptedObjects = numberOfVisitedObjects = 0;
		numberOfAcceptedSubtrees = numberOfReadSubtrees = 0;
		oceanTiles = 0;
		ocean = 0;
		limit = -1;
	}
	SearchQuery(int l, int r, int t, int b) : left(l), right(r), top(t), bottom(b) {
	}

	SearchQuery() {
		numberOfAcceptedObjects = numberOfVisitedObjects = 0;
		numberOfAcceptedSubtrees = numberOfReadSubtrees = 0;
		oceanTiles = 0;
		ocean = 0;
		limit = -1;
	}

	bool publish(MapDataObject* obj, MapIndex* ind, int8_t zoom) {
		return publisher->publish(FoundMapDataObject(obj, ind, zoom));
	}
};

std::vector<BinaryMapFile*> getOpenMapFiles();

void searchTransportIndex(SearchQuery* q, BinaryMapFile* file);

void loadTransportRoutes(BinaryMapFile* file, vector<int32_t> filePointers, UNORDERED(map) < int64_t,
						 SHARED_PTR<TransportRoute> > &result);

void searchRouteSubregions(SearchQuery* q, std::vector<RouteSubregion>& tempResult, bool basemap, bool geocoding);

void searchRouteDataForSubRegion(SearchQuery* q, std::vector<RouteDataObject*>& list, RouteSubregion* sub,
								 bool geocoding);

ResultPublisher* searchObjectsForRendering(SearchQuery* q, bool skipDuplicates, std::string msgNothingFound,
										   int& renderedState);

BinaryMapFile* initBinaryMapFile(std::string inputName, bool useLive, bool routingOnly);

bool initMapFilesFromCache(std::string inputName);

bool cacheBinaryMapFileIfNeeded(const std::string& inputName, bool routingOnly);

bool addToCache(BinaryMapFile* mapfile, bool routingOnly);

bool writeMapFilesCache(const std::string& filePath);

bool closeBinaryMapFile(std::string inputName);

void getIncompleteTransportRoutes(BinaryMapFile* file);

struct NetworkDBPoint;
struct HHRoutingContext;
struct HHRouteRegionPointsCtx;
void initHHPoints(BinaryMapFile* file, SHARED_PTR<HHRouteIndex> reg, HHRoutingContext * ctx, short mapId, UNORDERED_map<int64_t, NetworkDBPoint *> & resPoints);
int loadNetworkSegmentPoint(HHRoutingContext * ctx, SHARED_PTR<HHRouteRegionPointsCtx> regCtx, HHRouteBlockSegments * block, int searchInd);
#endif
