#ifndef _OSMAND_COMMON_CORE_H
#define _OSMAND_COMMON_CORE_H

#include <ElapsedTimer.h>
#include <SkPath.h>

#include <sstream>
#include <string>
#include <vector>

#include "Internal.h"
#include "CommonCollections.h"

// M_PI is no longer part of math.h/cmath by standart, but some GCC's define them
#define _USE_MATH_DEFINES
#include <math.h>
#if !defined(M_PI)
const double M_PI = 3.14159265358979323846;
#endif
#if !defined(M_PI_2)
const double M_PI_2 = M_PI / 2.0;
#endif

static const int SHIFT_COORDINATES = 5;
static const int LABEL_ZOOM_ENCODE = 26;
// Better don't do this
using namespace std;

inline double toRadians(double angdeg) {
	return angdeg / 180 * M_PI;
}

inline std::tm localtime(const std::time_t& time) {
	std::tm tm_snapshot;
#if (defined(WIN32) || defined(_WIN32) || defined(__WIN32__))
	localtime_s(&tm_snapshot, &time);
#else
	localtime_r(&time, &tm_snapshot);  // POSIX
#endif
	return tm_snapshot;
}

template <typename T>
class quad_tree {
   private:
	struct node {
		typedef std::vector<T> cont_t;
		cont_t data;
		std::unique_ptr<node> children[4];
		SkRect bounds;
		SkIRect bounds_int;

		node(SkRect& b) : bounds(b) {
		}

		node(SkIRect& b) : bounds_int(b) {
		}

		node(const node& b) : bounds(b.bounds), bounds_int(b.bounds_int) {
			data = b.data;
			for (int i = 0; i < 4; i++) {
				if (b.children[i] != NULL) {
					children[i] = std::unique_ptr<node>(new node(*b.children[i]));
				} else {
					children[i] = NULL;
				}
			}
		}
	};
	typedef typename node::cont_t cont_t;
	typedef typename cont_t::iterator node_data_iterator;
	double ratio;
	unsigned int max_depth;
	std::unique_ptr<node> root;

   public:
	quad_tree(SkRect r = SkRect::MakeIWH(0x7FFFFFFF, 0x7FFFFFFF), int depth = 8, double ratio = 0.55)
		: ratio(ratio), max_depth(depth), root(new node(r)) {
	}

	quad_tree(SkIRect r, int depth, double ratio)
		: ratio(ratio), max_depth(depth), root(new node(r)) {
	}

	quad_tree(const quad_tree& ref) : ratio(ref.ratio), max_depth(ref.max_depth), root(new node(*ref.root)) {
	}

	quad_tree<T>& operator=(const quad_tree<T>& ref) {
		ratio = ref.ratio;
		max_depth = ref.max_depth;
		root = std::unique_ptr<node>(new node(*ref.root));
		return *this;
	}

	uint count() {
		return size_node(root);
	}

	void insert(T data, SkRect& box) {
		unsigned int depth = 0;
		do_insert_data(data, box, root, depth);
	}

	void insert(T data, SkIRect& box) {
		unsigned int depth = 0;
		do_insert_data(data, box, root, depth);
	}

	void query_in_box(SkRect& box, std::vector<T>& result) {
		result.clear();
		query_node(box, result, root);
	}

	void query_in_box(SkIRect& box, std::vector<T>& result) {
		result.clear();
		query_node(box, result, root);
	}

	bool compareSkIRects(const SkIRect& r1, const SkIRect& r2) {
		return min(r1.fLeft, r1.fRight) <= min(r2.fLeft, r2.fRight) &&
			   max(r1.fLeft, r1.fRight) >= max(r2.fLeft, r2.fRight) &&
			   min(r1.fTop, r1.fBottom) <= min(r2.fTop, r2.fBottom) &&
			   max(r1.fTop, r1.fBottom) >= max(r2.fTop, r2.fBottom);
	}

   private:
	uint size_node(std::unique_ptr<node>& node) const {
		int sz = node->data.size();
		for (int k = 0; k < 4; ++k) {
			if (node->children[k]) {
				sz += size_node(node->children[k]);
			}
		}
		return sz;
	}

	void query_node(SkRect& box, std::vector<T>& result, std::unique_ptr<node>& node) const {
		if (node) {
			if (SkRect::Intersects(box, node->bounds)) {
				node_data_iterator i = node->data.begin();
				node_data_iterator end = node->data.end();
				while (i != end) {
					result.push_back(*i);
					++i;
				}
				for (int k = 0; k < 4; ++k) {
					query_node(box, result, node->children[k]);
				}
			}
		}
	}

	void query_node(SkIRect& box, std::vector<T>& result, std::unique_ptr<node>& node) const {
		if (node) {
			if (SkIRect::Intersects(box, node->bounds_int)) {
				node_data_iterator i = node->data.begin();
				node_data_iterator end = node->data.end();
				while (i != end) {
					result.push_back(*i);
					++i;
				}
				for (int k = 0; k < 4; ++k) {
					query_node(box, result, node->children[k]);
				}
			}
		}
	}

	void do_insert_data(T data, SkRect& box, std::unique_ptr<node>& n, unsigned int& depth) {
		if (++depth >= max_depth) {
			n->data.push_back(data);
		} else {
			SkRect& node_extent = n->bounds;
			SkRect ext[4];
			split_box(node_extent, ext);
			for (int i = 0; i < 4; ++i) {
				if (ext[i].contains(box)) {
					if (!n->children[i]) {
						n->children[i] = std::unique_ptr<node>(new node(ext[i]));
					}
					do_insert_data(data, box, n->children[i], depth);
					return;
				}
			}
			n->data.push_back(data);
		}
	}

	void do_insert_data(T data, SkIRect& box, std::unique_ptr<node>& n, unsigned int& depth) {
		if (++depth >= max_depth) {
			n->data.push_back(data);
		} else {
			SkIRect& node_extent = n->bounds_int;
			SkIRect ext[4];
			split_box(node_extent, ext);
			for (int i = 0; i < 4; ++i) {
				if (compareSkIRects(ext[i], box)) {
					if (!n->children[i]) {
						n->children[i] = std::unique_ptr<node>(new node(ext[i]));
					}
					do_insert_data(data, box, n->children[i], depth);
					return;
				}
			}
			n->data.push_back(data);
		}
	}

	void split_box(SkRect& node_extent, SkRect* ext) {
		// coord2d c=node_extent.center();

		float width = node_extent.width();
		float height = node_extent.height();

		float lox = node_extent.fLeft;
		float loy = node_extent.fTop;
		float hix = node_extent.fRight;
		float hiy = node_extent.fBottom;

		ext[0] = SkRect::MakeLTRB(lox, loy, lox + width * ratio, loy + height * ratio);
		ext[1] = SkRect::MakeLTRB(hix - width * ratio, loy, hix, loy + height * ratio);
		ext[2] = SkRect::MakeLTRB(lox, hiy - height * ratio, lox + width * ratio, hiy);
		ext[3] = SkRect::MakeLTRB(hix - width * ratio, hiy - height * ratio, hix, hiy);
	}

	void split_box(SkIRect& node_extent, SkIRect* ext) {
		int32_t width = node_extent.width();
		int32_t height = node_extent.height();

		int32_t lox = node_extent.fLeft;
		int32_t loy = node_extent.fTop;
		int32_t hix = node_extent.fRight;
		int32_t hiy = node_extent.fBottom;

		ext[0] = SkIRect::MakeLTRB(lox, loy, lox + width * ratio, loy + height * ratio);
		ext[1] = SkIRect::MakeLTRB(hix - width * ratio, loy, hix, loy + height * ratio);
		ext[2] = SkIRect::MakeLTRB(lox, hiy - height * ratio, lox + width * ratio, hiy);
		ext[3] = SkIRect::MakeLTRB(hix - width * ratio, hiy - height * ratio, hix, hiy);
	}
};

typedef pair<std::string, std::string> tag_value;
typedef pair<int, int> int_pair;
typedef vector<pair<int, int> > coordinates;

struct LatLon {
	double lat;
	double lon;
	LatLon(double lat, double lon) : lat(lat), lon(lon){};

	bool isEquals(LatLon other) { return abs(lat - other.lat) < 0.00001 && abs(lon - other.lon) < 0.00001; }
};

class MapDataObject {
   public:
	std::vector<tag_value> types;
	std::vector<tag_value> additionalTypes;
	coordinates points;
	std::vector<coordinates> polygonInnerCoordinates;

	UNORDERED(map)<std::string, unsigned int> stringIds;

	UNORDERED(map)<std::string, std::string> objectNames;
	std::vector<std::string> namesOrder;
	bool area;
	int64_t id;
	int32_t labelX;
	int32_t labelY;

	bool cycle() {
		return points[0] == points[points.size() - 1];
	}
	bool containsAdditional(std::string key, std::string val) {
		auto it = additionalTypes.begin();
		bool valEmpty = (val == "");
		while (it != additionalTypes.end()) {
			if (it->first == key && (valEmpty || it->second == val)) {
				return true;
			}
			it++;
		}
		return false;
	}

	bool contains(std::string key, std::string val) {
		auto it = types.begin();
		while (it != types.end()) {
			if (it->first == key) {
				return it->second == val;
			}
			it++;
		}
		return false;
	}

	bool isLabelSpecified() {
		return (labelX != 0 || labelY != 0) && points.size() > 0;
	}

	int32_t getLabelX() {
		int64_t sum = 0;
		int32_t LABEL_SHIFT = 31 - LABEL_ZOOM_ENCODE;
		int32_t len = points.size();
		for (int32_t i = 0; i < len; i++) {
			sum += points.at(i).first;
		}
		int32_t average = ((sum >> SHIFT_COORDINATES) / len) << (SHIFT_COORDINATES - LABEL_SHIFT);
		int32_t label31X = (average + labelX) << LABEL_SHIFT;
		return label31X;
	}

	int32_t getLabelY() {
		int64_t sum = 0;
		int32_t LABEL_SHIFT = 31 - LABEL_ZOOM_ENCODE;
		int32_t len = points.size();
		for (int32_t i = 0; i < len; i++) {
			sum += points.at(i).second;
		}
		int32_t average = ((sum >> SHIFT_COORDINATES) / len) << (SHIFT_COORDINATES - LABEL_SHIFT);
		int32_t label31Y = (average + labelY) << LABEL_SHIFT;
		return label31Y;
	}

	int getSimpleLayer() {
		auto it = additionalTypes.begin();
		bool tunnel = false;
		bool bridge = false;
		while (it != additionalTypes.end()) {
			if (it->first == "layer") {
				if (it->second.length() > 0) {
					if (it->second[0] == '-') {
						return -1;
					} else if (it->second[0] == '0') {
						return 0;
					} else {
						return 1;
					}
				}
			} else if (it->first == "tunnel") {
				tunnel = "yes" == it->second;
			} else if (it->first == "bridge") {
				bridge = "yes" == it->second;
			}
			it++;
		}
		if (tunnel) {
			return -1;
		} else if (bridge) {
			return 1;
		}
		return 0;
	}
};

struct FoundMapDataObject {
	MapDataObject* obj;
	void* ind;
	uint8_t zoom;
	FoundMapDataObject(MapDataObject* obj = NULL, void* ind = NULL, uint8_t zoom = 15) {
		this->obj = obj;
		this->ind = ind;
		this->zoom = zoom;
	}
	FoundMapDataObject(const FoundMapDataObject& c) {
		this->obj = c.obj;
		this->ind = c.ind;
		this->zoom = c.zoom;
	}
};

void deleteObjects(std::vector<FoundMapDataObject>& v);

int get31TileNumberX(double longitude);
int get31TileNumberY(double latitude);

double getPowZoom(float zoom);

double getLongitudeFromTile(float zoom, double x);
double getLatitudeFromTile(float zoom, double y);

double get31LongitudeX(int tileX);
double get31LatitudeY(int tileY);
double getTileNumberX(float zoom, double longitude);
double getTileNumberY(float zoom, double latitude);
double getDistance(double lat1, double lon1, double lat2, double lon2);
double getPowZoom(float zoom);

double measuredDist31(int x1, int y1, int x2, int y2);
double getTileDistanceWidth(double lat, float zoom);
double squareDist31TileMetric(int x1, int y1, int x2, int y2);
double squareRootDist31(int x1, int y1, int x2, int y2);
double getTileWidth(int y31);
double alignAngleDifference(double diff);
bool calculateIntersection(int x, int y, int px, int py, int leftX, int rightX, int bottomY, int topY, int_pair& b);

double strtod_li(string v);

double degreesDiff(const double a1, const double a2);
double normalizeDegrees360(double degrees);

int findFirstNumberEndIndex(string value);
double parseSpeed(string v, double def);

std::pair<int, int> getProjectionPoint(int px, int py, int xA, int yA, int xB, int yB);
std::pair<double, double> getProjection(double lat, double lon, double fromLat, double fromLon, double toLat, double toLon);
double getOrthogonalDistance(double lat, double lon, double fromLat, double fromLon, double toLat, double toLon);
double scalarMultiplication(double xA, double yA, double xB, double yB, double xC, double yC);

std::string to_lowercase(const std::string& in);
std::vector<std::string> split_string(const std::string& str, const std::string& delimiters);
bool endsWith(const std::string& str, const std::string& suffix);
bool startsWith(const std::string& str, const std::string& prefix);

const static char* trim_chars = " \t\n\r\f\v";

std::string rtrim(const std::string& in, const char* t = trim_chars);
std::string ltrim(const std::string& in, const char* t = trim_chars);
std::string trim(const std::string& in, const char* t = trim_chars);

void trimspec(std::string &text);

#endif /*_OSMAND_COMMON_CORE_H*/
