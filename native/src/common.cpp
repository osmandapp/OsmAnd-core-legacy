#include "CommonCollections.h"
#include "Logging.h"
#include "commonOsmAndCore.h"
#include "json.hpp"

#if defined(_WIN32)
//#	include <windows.h>
//#	include <mmsystem.h>
#define isnan _isnan
#define isinf !_finite
#elif defined(__APPLE__)
#include <mach/mach_time.h>

#else
#include <time.h>
#endif

#include <limits.h>

void deleteObjects(std::vector<FoundMapDataObject>& v) {
	for (size_t i = 0; i < v.size(); i++) {
		delete v[i].obj;
	}
	v.clear();
}

double getPowZoom(float zoom) {
	if (zoom >= 0 && zoom - floor(zoom) < 0.05f) {
		return 1 << ((int)zoom);
	} else {
		return pow(2, zoom);
	}
}

static double getTileDistanceWidth(double zoom) {
	return getDistance(30, getLongitudeFromTile(zoom, 0), 30, getLongitudeFromTile(zoom, 1));
}

double measuredDist31(int x1, int y1, int x2, int y2) {
	return getDistance(get31LatitudeY(y1), get31LongitudeX(x1), get31LatitudeY(y2), get31LongitudeX(x2));
}

double dabs(double d) {
	if (d < 0) {
		return -d;
	} else {
		return d;
	}
}

const uint precisionPower = 10;
const uint precisionDiv = 1 << (31 - precisionPower);

double coefficientsY[1 << precisionPower];
bool initializeYArray = false;
double convert31YToMeters(int y1, int y2, int x) {
	if (!initializeYArray) {
		coefficientsY[0] = 0;
		for (uint i = 0; i < (1 << precisionPower) - 1; i++) {
			coefficientsY[i + 1] =
				coefficientsY[i] + measuredDist31(0, i << (31 - precisionPower), 0, ((i + 1) << (31 - precisionPower)));
		}
		initializeYArray = true;
	}
	uint div1 = y1 / precisionDiv;
	uint mod1 = y1 % precisionDiv;
	uint div2 = y2 / precisionDiv;
	uint mod2 = y2 % precisionDiv;
	double h1;
		if(div1 + 1 >= sizeof(coefficientsY)/sizeof(*coefficientsY)) {
			h1 = coefficientsY[div1] + mod1 / ((double)precisionDiv) * (coefficientsY[div1] - coefficientsY[div1 - 1]);
		} else {
			h1 = coefficientsY[div1] + mod1 / ((double)precisionDiv) * (coefficientsY[div1 + 1] - coefficientsY[div1]);
		}
		double h2 ;
		if(div2 + 1 >= sizeof(coefficientsY)/sizeof(*coefficientsY)) {
			h2 = coefficientsY[div2] + mod2 / ((double)precisionDiv) * (coefficientsY[div2] - coefficientsY[div2 - 1]);
		} else {
			h2 = coefficientsY[div2] + mod2 / ((double)precisionDiv) * (coefficientsY[div2 + 1] - coefficientsY[div2]);
		}
	double res = h1 - h2;
	// OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Debug, "ind %f != %f", res,  measuredDist31(x, y1, x, y2));
	return res;
}
double coefficientsX[1 << precisionPower];
bool initializeXArray = false;
double convert31XToMeters(int x1, int x2, int y) {
	if (!initializeXArray) {
		for (uint i = 0; i < (1 << precisionPower); i++) {
			coefficientsX[i] = 0;
		}
		initializeXArray = true;
	}
	int ind = y / precisionDiv;
	if (coefficientsX[ind] == 0) {
		double md = measuredDist31(x1, y, x2, y);
		if (md < 10 || x1 == x2) {
			return md;
		}
		coefficientsX[ind] = md / dabs((double)x1 - (double)x2);
	}
	// translate into meters
	return ((double)x1 - x2) * coefficientsX[ind];
}

double scalarMultiplication(double xA, double yA, double xB, double yB, double xC, double yC) {
	// Scalar multiplication between (AB, AC)
	return (xB - xA) * (xC - xA) + (yB - yA) * (yC - yA);
}

double getDistance(std::pair<double, double> l, double latitude, double longitude) {
	return getDistance(l.first, l.second, latitude, longitude);
}

double getOrthogonalDistance(double lat, double lon, double fromLat, double fromLon, double toLat, double toLon) {
	return getDistance(getProjection(lat, lon, fromLat, fromLon, toLat, toLon), lat, lon);
}

std::pair<double, double> getProjection(double lat, double lon, double fromLat, double fromLon, double toLat,
										double toLon) {
	// not very accurate computation on sphere but for distances < 1000m it is ok
	double mDist = (fromLat - toLat) * (fromLat - toLat) + (fromLon - toLon) * (fromLon - toLon);
	double projection = scalarMultiplication(fromLat, fromLon, toLat, toLon, lat, lon);
	double prlat;
	double prlon;
	if (projection < 0) {
		prlat = fromLat;
		prlon = fromLon;
	} else if (projection >= mDist) {
		prlat = toLat;
		prlon = toLon;
	} else {
		prlat = fromLat + (toLat - fromLat) * (projection / mDist);
		prlon = fromLon + (toLon - fromLon) * (projection / mDist);
	}
	return std::pair<double, double>(prlat, prlon);
}

std::pair<int, int> getProjectionPoint(int px, int py, int xA, int yA, int xB, int yB) {
	double mDist = measuredDist31(xA, yA, xB, yB);
	int prx = xA;
	int pry = yA;
	double projection = calculateProjection31TileMetric(xA, yA, xB, yB, px, py);
	if (projection < 0) {
		prx = xA;
		pry = yA;
	} else if (projection >= mDist * mDist) {
		prx = xB;
		pry = yB;
	} else {
		double c = projection / (mDist * mDist);
		prx = (int)((double)xA + ((double)xB - xA) * c);
		pry = (int)((double)yA + ((double)yB - yA) * c);
	}
	return std::pair<int, int>(prx, pry);
}

double calculateProjection31TileMetric(int xA, int yA, int xB, int yB, int xC, int yC) {
	// Scalar multiplication between (AB, AC)
	double multiple = convert31XToMeters(xB, xA, yA) * convert31XToMeters(xC, xA, yA) +
					  convert31YToMeters(yB, yA, xA) * convert31YToMeters(yC, yA, xA);
	return multiple;
}
double squareDist31TileMetric(int x1, int y1, int x2, int y2) {
	// translate into meters
	double dy = convert31YToMeters(y1, y2, x1);
	double dx = convert31XToMeters(x1, x2, y1);
	return dx * dx + dy * dy;
}

double squareRootDist31(int x1, int y1, int x2, int y2) {
	// translate into meters
	double dy = convert31YToMeters(y1, y2, x1);
	double dx = convert31XToMeters(x1, x2, y1);
	return sqrt(dx * dx + dy * dy);
}

double degreesDiff(const double a1, const double a2) {
	auto diff = a1 - a2;
	while (diff > 180.0) diff -= 360.0;
	while (diff <= -180.0) diff += 360.0;
	return diff;
}

double normalizeDegrees360(double degrees)
{
    while (degrees < 0.0f) {
        degrees += 360.0f;
    }
    while (degrees >= 360.0f) {
        degrees -= 360.0f;
    }
    return degrees;
}

double checkLongitude(double longitude) {
	while (longitude < -180 || longitude > 180) {
		if (longitude < 0) {
			longitude += 360;
		} else {
			longitude -= 360;
		}
	}
	return longitude;
}

double checkLatitude(double latitude) {
	while (latitude < -90 || latitude > 90) {
		if (latitude < 0) {
			latitude += 180;
		} else {
			latitude -= 180;
		}
	}
	if (latitude < -85.0511) {
		return -85.0511;
	} else if (latitude > 85.0511) {
		return 85.0511;
	}
	return latitude;
}

int get31TileNumberX(double longitude) {
	longitude = checkLongitude(longitude);
	int64_t l = 1;
	l <<= 31;
    
    double tileNumberX = ((longitude + 180) / 360 * l);
    if (tileNumberX > INT_MAX)
        tileNumberX = INT_MAX;
    	return (int)tileNumberX;
}

int get31TileNumberY(double latitude) {
	latitude = checkLatitude(latitude);
	double eval = log(tan(toRadians(latitude)) + 1 / cos(toRadians(latitude)));
	int64_t l = 1;
	l <<= 31;
	if (eval > M_PI) {
		eval = M_PI;
	}
	return (int)((1 - eval / M_PI) / 2 * l);
}

double getLongitudeFromTile(float zoom, double x) {
	return x / getPowZoom(zoom) * 360.0 - 180.0;
}

double getLatitudeFromTile(float zoom, double y) {
	int sign = y < 0 ? -1 : 1;
	double result = atan(sign * sinh(M_PI * (1 - 2 * y / getPowZoom(zoom)))) * 180. / M_PI;
	return result;
}

double get31LongitudeX(int tileX) {
	return getLongitudeFromTile(21, tileX / 1024.);
}

double get31LatitudeY(int tileY) {
	return getLatitudeFromTile(21, tileY / 1024.);
}

double getTileNumberX(float zoom, double longitude) {
	if (longitude == 180.) {
		return getPowZoom(zoom) - 1;
	}
	longitude = checkLongitude(longitude);
	return (longitude + 180.) / 360. * getPowZoom(zoom);
}

double getTileNumberY(float zoom, double latitude) {
	latitude = checkLatitude(latitude);
	double eval = log(tan(toRadians(latitude)) + 1 / cos(toRadians(latitude)));
	if (isinf(eval) || isnan(eval)) {
		latitude = latitude < 0 ? -89.9 : 89.9;
		eval = log(tan(toRadians(latitude)) + 1 / cos(toRadians(latitude)));
	}
	double result = (1 - eval / M_PI) / 2 * getPowZoom(zoom);
	return result;
}

double getDistance(double lat1, double lon1, double lat2, double lon2) {
	double R = 6372.8;  // km
	double dLat = toRadians(lat2 - lat1);
	double dLon = toRadians(lon2 - lon1);
	double sinHalfLat = sin(dLat / 2);
	double sinHalfLon = sin(dLon / 2);
	double a = sinHalfLat * sinHalfLat + cos(toRadians(lat1)) * cos(toRadians(lat2)) * sinHalfLon * sinHalfLon;
	double c = 2 * atan2(sqrt(a), sqrt(1 - a));
	return R * c * 1000;
}

double strtod_li(string s) {
	std::istringstream text( s );
	text.imbue(std::locale::classic());
	double result;
	text >> result;
	return result;
}

int findFirstNumberEndIndex(string value) {
	uint i = 0;
	bool valid = false;
	if (value.length() > 0 && value[0] == '-') {
		i++;
	}
	int state = 0;
	while (i < value.length() && ((value[i] >= '0' && value[i] <= '9') || value[i] == '.')) {
		if (value[i] == '.') {
			if (state == 2) {
				return i - 1;
			}
			if (state != 1)	{
				return -1;
			}
			state = 2;
		} else {
			if (state == 2)	{
				// last digits
				state = 3;
			} else if (state == 0) {
				// first digits started
				state = 1;
			}
		}
		i++;
	}
	if (state == 2) {	
		return i - 1;
	}
	if (state == 0) {
		return -1;
	}
	return i;
}

double parseSpeed(string v, double def) {
	if (v == "none") {
		return 40;	// RouteDataObject::NONE_MAX_SPEED;
	} else {
		int i = findFirstNumberEndIndex(v);
		if (i > 0) {
			double f = strtod_li(v.substr(0, i));
			f /= 3.6;  // km/h -> m/s
			if (v.find("mph") != string::npos) {
				f *= 1.6;
			}
			return f;
		}
	}
	return def;
}

double alignAngleDifference(double diff) {
	while (diff > M_PI) {
		diff -= 2 * M_PI;
	}
	while (diff <= -M_PI) {
		diff += 2 * M_PI;
	}
	return diff;
}

/**
 * @return -1 if there is no instersection or x<<32 | y
 */
bool calculateIntersection(int x, int y, int px, int py, int leftX, int rightX, int bottomY, int topY, int_pair& b) {
	// firstly try to search if the line goes in
	if (py < topY && y >= topY) {
		int tx = (int)(px + ((double)(x - px) * (topY - py)) / (y - py));
		if (leftX <= tx && tx <= rightX) {
			b.first = tx;
			b.second = topY;
			return true;
		}
	}
	if (py > bottomY && y <= bottomY) {
		int tx = (int)(px + ((double)(x - px) * (py - bottomY)) / (py - y));
		if (leftX <= tx && tx <= rightX) {
			b.first = tx;
			b.second = bottomY;
			return true;
		}
	}
	if (px < leftX && x >= leftX) {
		int ty = (int)(py + ((double)(y - py) * (leftX - px)) / (x - px));
		if (ty >= topY && ty <= bottomY) {
			b.first = leftX;
			b.second = ty;
			return true;
		}
	}
	if (px > rightX && x <= rightX) {
		int ty = (int)(py + ((double)(y - py) * (px - rightX)) / (px - x));
		if (ty >= topY && ty <= bottomY) {
			b.first = rightX;
			b.second = ty;
			return true;
		}
	}

	// try to search if point goes out
	if (py > topY && y <= topY) {
		int tx = (int)(px + ((double)(x - px) * (topY - py)) / (y - py));
		if (leftX <= tx && tx <= rightX) {
			b.first = tx;
			b.second = topY;
			return true;
		}
	}
	if (py < bottomY && y >= bottomY) {
		int tx = (int)(px + ((double)(x - px) * (py - bottomY)) / (py - y));
		if (leftX <= tx && tx <= rightX) {
			b.first = tx;
			b.second = bottomY;
			return true;
		}
	}
	if (px > leftX && x <= leftX) {
		int ty = (int)(py + ((double)(y - py) * (leftX - px)) / (x - px));
		if (ty >= topY && ty <= bottomY) {
			b.first = leftX;
			b.second = ty;
			return true;
		}
	}
	if (px < rightX && x >= rightX) {
		int ty = (int)(py + ((double)(y - py) * (px - rightX)) / (px - x));
		if (ty >= topY && ty <= bottomY) {
			b.first = rightX;
			b.second = ty;
			return true;
		}
	}

	if (px == rightX || px == leftX || py == topY || py == bottomY) {
		b.first = px;
		b.second = py;
		//		return true;
		// Is it right? to not return anything?
	}
	return false;
}

std::string to_lowercase(const std::string& in) {
	std::string out(in);
	for (uint i = 0; i < in.length(); i++) {
		out[i] = std::tolower(in[i]);
	}
	return out;
}

std::vector<std::string> split_string(const std::string& str, const std::string& delimiters) {
	std::vector<std::string> tokens;
	std::string::size_type pos, lastPos = 0, length = str.length();

	while (lastPos < length + 1) {
		pos = str.find(delimiters, lastPos);
		if (pos == std::string::npos) pos = length;
		if (pos != lastPos)
			tokens.push_back(str.substr(lastPos, pos - lastPos));
		else
			tokens.push_back("");

		lastPos = pos + delimiters.length();
	}
	return tokens;
}

bool endsWith(const std::string& str, const std::string& suffix) {
	return str.size() >= suffix.size() && !str.compare(str.size() - suffix.size(), suffix.size(), suffix);
}

bool startsWith(const std::string& str, const std::string& prefix) {
	return str.size() >= prefix.size() && !str.compare(0, prefix.size(), prefix);
}

std::string rtrim(const std::string& in, const char* t) {
	string s(in);
	s.erase(s.find_last_not_of(t) + 1);
	return s;
}

std::string ltrim(const std::string& in, const char* t) {
	string s(in);
	s.erase(0, s.find_first_not_of(t));
	return s;
}

std::string trim(const std::string& in, const char* t) {
	return ltrim(rtrim(in, t), t);
}

void trimspec(std::string &text) {
	// unicode symbols \u200e \u200f \u202a \u202c \u202b
	const char *symbols[] = { "\xE2\x80\x8E", "\xE2\x80\x8F", "\xE2\x80\xAA", "\xE2\x80\xAC", "\xE2\x80\xAB"};
	int length = text.length();
	for (auto t : symbols) {
		if (length >= 3 && t[0] == text.at(0) && t[1] == text.at(1) && t[2] == text.at(2)) {
			text.erase(0, 3);
			length = text.length();
		}
		if (length >= 3 && t[0] == text.at(length - 3) && t[1] == text.at(length - 2) && t[2] == text.at(length - 1)) {
			text.erase(length - 3);
			length = text.length();
		}
		if (length < 3) {
			return;
		}
	}
}

std::string RenderableObject::toJson() const {
	json::JSON j;
	// general
	j["id"] = getId();
	j["type"] = type;
	j["points"] = json::Array();

	for (const auto& p : getPoints()) {
		json::JSON point = json::Array();
		point.append(p.first);
		point.append(p.second);
		j["points"].append(point);
	}

	j["types"] = json::Array();
	for (const auto& t : getTypes()) {
		json::JSON type;
		type["tag"] = t.first;
		type["value"] = t.second;
		j["types"].append(type);
	}

	j["additionalTypes"] = json::Array();
	for (const auto& at : getAdditionalTypes()) {
		json::JSON additionalType;
		additionalType["tag"] = at.first;
		additionalType["value"] = at.second;
		j["additionalTypes"].append(additionalType);
	}

	// points
	j["mainIcon"] = mainIcon;
	j["additionalIcons"] = json::Array();
	for (const auto& icon : additionalIcons) {
		j["additionalIcons"].append(icon);
	}

	j["shield"] = shield;
	j["iconOrder"] = iconOrder;
	j["iconSize"] = iconSize;
	j["iconX"] = iconX;
	j["iconY"] = iconY;

	// text
	j["text"] = text;
	j["textSize"] = textSize;
	j["textOnPath"] = textOnPath;
	j["textColor"] = textColor;
	j["textShadow"] = textShadow;
	j["textShadowColor"] = textShadowColor;
	j["bold"] = bold;
	j["italic"] = italic;
	j["shieldRes"] = shieldRes;
	j["shieldResIcon"] = shieldResIcon;

	return j.dump();
}
