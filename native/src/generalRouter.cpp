#ifndef _OSMAND_GENERAL_ROUTER_CPP
#define _OSMAND_GENERAL_ROUTER_CPP
#include "generalRouter.h"

#include <cmath>
#include <sstream>

#include "Logging.h"
#include "binaryRoutePlanner.h"
#include "routeSegment.h"

const int RouteAttributeExpression::LESS_EXPRESSION = 1;
const int RouteAttributeExpression::GREAT_EXPRESSION = 2;
const int RouteAttributeExpression::EQUAL_EXPRESSION = 3;
const int RouteAttributeExpression::MIN_EXPRESSION = 4;
const int RouteAttributeExpression::MAX_EXPRESSION = 5;


const double GeneralRouterConstants::CAR_SHORTEST_DEFAULT_SPEED = 55 / 3.6f;
const double GeneralRouterConstants::BICYCLE_SHORTEST_DEFAULT_SPEED = 15 / 3.6f;
const char* GeneralRouterConstants::USE_SHORTEST_WAY = "short_way";
const char* GeneralRouterConstants::USE_HEIGHT_OBSTACLES = "height_obstacles";
const char* GeneralRouterConstants::GROUP_RELIEF_SMOOTHNESS_FACTOR = "relief_smoothness_factor";
const char* GeneralRouterConstants::ALLOW_PRIVATE = "allow_private";
const char* GeneralRouterConstants::CHECK_ALLOW_PRIVATE_NEEDED = "check_allow_private_needed";
const char* GeneralRouterConstants::ALLOW_PRIVATE_FOR_TRUCK = "allow_private_for_truck";
const char* GeneralRouterConstants::DEFAULT_SPEED = "default_speed";
const char* GeneralRouterConstants::MIN_SPEED = "min_speed";
const char* GeneralRouterConstants::MAX_SPEED = "max_speed";

static const bool USE_CACHE = true;

GeneralRouter::GeneralRouter()
	: profile(GeneralRouterProfile::CAR), _restrictionsAware(true), heightObstacles(false), minSpeed(0.28),
	  defaultSpeed(1.0), maxSpeed(10.0), shortestRoute(false), allowPrivate(false), checkAllowPrivateNeeded(false) {
	cacheEval.resize((std::size_t)RouteDataObjectAttribute::COUNT);
}

GeneralRouter::GeneralRouter(const GeneralRouterProfile profile, const MAP_STR_STR& attributes)
	: profile(GeneralRouterProfile::CAR), _restrictionsAware(true), heightObstacles(false), sharpTurn(.0),
	  roundaboutTurn(.0), slightTurn(.0), minSpeed(0.28), defaultSpeed(1.0), maxSpeed(10.0), shortestRoute(false),
	  allowPrivate(false), checkAllowPrivateNeeded(false) {
	this->profile = profile;
	cacheEval.resize((std::size_t)RouteDataObjectAttribute::COUNT);
	MAP_STR_STR::const_iterator it = attributes.begin();
	for (; it != attributes.end(); it++) {
		addAttribute(it->first, it->second);
	}
	for (int i = 0; i < (int)RouteDataObjectAttribute::COUNT; i++) {
		newRouteAttributeContext();
	}
}

GeneralRouter::GeneralRouter(const GeneralRouter& parent, const MAP_STR_STR& params)
	: profile(GeneralRouterProfile::CAR), _restrictionsAware(true), heightObstacles(false), sharpTurn(.0),
	  roundaboutTurn(.0), slightTurn(.0), minSpeed(0.28), defaultSpeed(1.0), maxSpeed(10.0), shortestRoute(false),
	  allowPrivate(false), checkAllowPrivateNeeded(false) {
	this->profile = parent.profile;
	cacheEval.resize((std::size_t)RouteDataObjectAttribute::COUNT);
	MAP_STR_STR::const_iterator it = parent.attributes.begin();
	for (; it != parent.attributes.end(); it++) {
		addAttribute(it->first, it->second);
	}

	universalRules = parent.universalRules;
	universalRulesById = parent.universalRulesById;
	parameterValues = params;
	tagRuleMask = parent.tagRuleMask;
	ruleToValue = parent.ruleToValue;
	parameters = parent.parameters;
	parametersList = parent.parametersList;
	profileName = parent.profileName;

	for (int i = 0; i < (int)RouteDataObjectAttribute::COUNT; i++) {
		newRouteAttributeContext(parent.objectAttributes[i], params);
	}

	this->allowPrivate = parseBool(params, GeneralRouterConstants::ALLOW_PRIVATE, false);
	this->checkAllowPrivateNeeded = parseBool(params, GeneralRouterConstants::CHECK_ALLOW_PRIVATE_NEEDED, false);
	this->shortestRoute = parseBool(params, GeneralRouterConstants::USE_SHORTEST_WAY, false);
	this->heightObstacles = parseBool(params, GeneralRouterConstants::USE_HEIGHT_OBSTACLES, false);
	this->defaultSpeed = parseFloat(params, GeneralRouterConstants::DEFAULT_SPEED, this->defaultSpeed);
	this->minSpeed = parseFloat(params, GeneralRouterConstants::MIN_SPEED, this->minSpeed);
	this->maxSpeed = parseFloat(params, GeneralRouterConstants::MAX_SPEED, this->maxSpeed);
	this->maxVehicleSpeed = this->maxSpeed;
	if (shortestRoute) {
		if (this->profile == GeneralRouterProfile::BICYCLE) {
			this->maxSpeed = min(GeneralRouterConstants::BICYCLE_SHORTEST_DEFAULT_SPEED, this->maxSpeed);
		} else {
			this->maxSpeed = min(GeneralRouterConstants::CAR_SHORTEST_DEFAULT_SPEED, this->maxSpeed);
		}
	}
}

float parseFloat(MAP_STR_STR attributes, string key, float def) {
	if (attributes.find(key) != attributes.end() && attributes[key] != "") {
		return strtod_li(attributes[key]);
	}
	return def;
}

float parseFloat(string value, float def) {
	if (value != "") {
		return strtod_li(value);
	}
	return def;
}

bool parseBool(MAP_STR_STR attributes, string key, bool def) {
	if (attributes.find(key) != attributes.end() && attributes[key] != "") {
		return attributes[key] == "true";
	}
	return def;
}

bool parseBool(string value, bool def) {
	if (value != "") {
		return value == "true";
	}
	return def;
}

string parseString(MAP_STR_STR attributes, string key, string def) {
	if (attributes.find(key) != attributes.end() && attributes[key] != "") {
		return attributes[key];
	}
	return def;
}

string parseString(string value, string def) {
	if (value != "") {
		return value;
	}
	return def;
}

dynbitset& increaseSize(dynbitset& t, uint targetSize) {
	if (t.size() < targetSize) {
		t.resize(targetSize);
	}
	return t;
}

dynbitset& align(dynbitset& t, uint targetSize) {
	if (t.size() < targetSize) {
		t.resize(targetSize);
	} else if (t.size() > targetSize) {
		OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "Bitset %d is longer than target %d", t.size(), targetSize);
	}
	return t;
}

double parseValue(string value, string type) {
	double vl = -1;
	if ("speed" == type) {
		vl = parseSpeed(value, vl);
	} else if ("weight" == type) {
		vl = RouteDataObject::parseWeightInTon(value, vl);
	} else if ("length" == type) {
		vl = RouteDataObject::parseLength(value, vl);
	} else {
		int i = findFirstNumberEndIndex(value);
		if (i > 0) {
			// could be negative
			return strtod_li(value.substr(0, i));
		}
	}
	if (vl == -1) {
		return DOUBLE_MISSING;
	}
	return vl;
}

UNORDERED_map<std::string, RoutingParameter> GeneralRouter::getParameters(const std::string &derivedProfile) {
	UNORDERED_map<std::string, RoutingParameter> params;
	for (auto it = parameters.begin(); it != parameters.end(); ++it) {
		const auto profiles = it->second.profiles;
		if (profiles.empty() || find(profiles.begin(), profiles.end(), derivedProfile) != profiles.end()) {
			params[it->first] = it->second;
		}
	}
	return params;
}

void GeneralRouter::addAttribute(string k, string v) {
	attributes[k] = v;
	if (k == "restrictionsAware") {
		_restrictionsAware = parseBool(attributes, k, _restrictionsAware);
	} else if (k == "sharpTurn" || k == "leftTurn") {
		sharpTurn = parseFloat(attributes, k, sharpTurn);
	} else if (k == "slightTurn" || k == "rightTurn") {
		slightTurn = parseFloat(attributes, k, slightTurn);
	} else if (k == "roundaboutTurn") {
		roundaboutTurn = parseFloat(attributes, k, roundaboutTurn);
	} else if (k == "minDefaultSpeed" || k == "defaultSpeed") {
		defaultSpeed = parseFloat(attributes, k, defaultSpeed * 3.6f) / 3.6f;
	} else if (k == "minSpeed") {
		minSpeed = parseFloat(attributes, k, minSpeed * 3.6f) / 3.6f;
	} else if (k == "maxDefaultSpeed" || k == "maxSpeed") {
		maxSpeed = parseFloat(attributes, k, maxSpeed * 3.6f) / 3.6f;
	}
}

void toStr(std::ostringstream& s, UNORDERED(set) < string > &v) {
	s << "[";
	UNORDERED(set)<string>::iterator i = v.begin();
	for (; i != v.end(); i++) {
		if (i != v.begin()) s << ", ";
		s << (string)*i;
	}

	s << "]";
}

void RouteAttributeEvalRule::printRule(GeneralRouter* r) {
	std::ostringstream s;
	s << " Select ";
	if (selectValue == DOUBLE_MISSING) {
		s << selectValueDef;
	} else {
		s << selectValue;
	}

	bool f = true;
	for (uint k = 0; k < filterTypes.size(); k++) {
		if (filterTypes.test(k)) {
			if (f) {
				s << " if ";
				f = !f;
			}
			tag_value key = r->universalRulesById[k];
			s << key.first << "/" << key.second;
		}
	}
	f = true;
	for (uint k = 0; k < filterNotTypes.size(); k++) {
		if (filterNotTypes.test(k)) {
			if (f) {
				s << " if ";
				f = !f;
			}
			tag_value key = r->universalRulesById[k];
			s << key.first << "/" << key.second;
		}
	}
	for (uint k = 0; k < parameters.size(); k++) {
		s << " param=" << parameters[k];
	}
	if (onlyTags.size() > 0) {
		s << " match tag = ";
		toStr(s, onlyTags);
	}
	if (onlyNotTags.size() > 0) {
		s << " not match tag = ";
		toStr(s, onlyNotTags);
	}
	if (conditionExpressions.size() > 0) {
		s << " conditionExpressions " << conditionExpressions.size();
	}
	if (selectExpression.expressionType != 0) {
		s << " selectexpression " << selectExpression.expressionType;
	}
	OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "%s", s.str().c_str());
}

RouteAttributeExpression::RouteAttributeExpression(vector<string>& vls, int type, string vType)
	: values(vls), expressionType(type), valueType(vType) {
	cacheValues.resize(vls.size());
	for (uint i = 0; i < vls.size(); i++) {
		if (vls[i][0] != '$' && vls[i][0] != ':') {
			double o = parseValue(vls[i], valueType);
			cacheValues[i] = o;
		} else {
			cacheValues[i] = DOUBLE_MISSING;
		}
	}
}

RouteAttributeEvalRule::RouteAttributeEvalRule(const SHARED_PTR<RouteAttributeEvalRule>& original) {
	parameters = original->parameters;
	selectValue = original->selectValue;
	selectValueDef = original->selectValueDef;
	selectType = original->selectType;
	filterTypes = original->filterTypes;
	filterNotTypes = original->filterNotTypes;

	onlyTags = original->onlyTags;
	onlyNotTags = original->onlyNotTags;
	selectExpression = original->selectExpression;
	conditionExpressions = original->conditionExpressions;

	tagValueCondDefValue = original->tagValueCondDefValue;
	tagValueCondDefTag = original->tagValueCondDefTag;
	tagValueCondDefNot = original->tagValueCondDefNot;
}

void RouteAttributeEvalRule::registerAndTagValueCondition(GeneralRouter* r, string tag, string value, bool nt) {
	tagValueCondDefTag.push_back(tag);
	tagValueCondDefValue.push_back(value);
	tagValueCondDefNot.push_back(nt);
	if (value.empty()) {
		if (nt) {
			onlyNotTags.insert(tag);
		} else {
			onlyTags.insert(tag);
		}
	} else {
		tag_value t = tag_value(tag, value);
		int vtype = r->registerTagValueAttribute(t);
		if (nt) {
			increaseSize(filterNotTypes, vtype + 1).set(vtype);
		} else {
			increaseSize(filterTypes, vtype + 1).set(vtype);
		}
	}
}

void RouteAttributeEvalRule::registerParamConditions(vector<string>& params) {
	parameters.insert(parameters.end(), params.begin(), params.end());
}

void RouteAttributeEvalRule::registerAndParamCondition(string param, bool nt) {
	param = nt ? "-" + param : param;
	parameters.push_back(param);
}

void RouteAttributeEvalRule::registerSelectValue(string value, string type) {
	this->selectType = type;
	this->selectValueDef = value;
	if (selectValueDef.length() > 0 && (selectValueDef[0] == '$' || selectValueDef[0] == ':')) {
		// init later
		selectValue = DOUBLE_MISSING;
	} else {
		selectValue = parseValue(value, type);
		if (selectValue == DOUBLE_MISSING) {
			//	System.err.println("Routing.xml select value '" + value+"' was not registered");
		}
	}
}

double GeneralRouter::parseValueFromTag(uint id, string type, GeneralRouter* router) {
	while (ruleToValue.size() <= id) {
		ruleToValue.push_back(DOUBLE_MISSING);
	}
	double res = ruleToValue[id];
	if (res == DOUBLE_MISSING) {
		tag_value v = router->universalRulesById[id];
		res = parseValue(v.second, type);
		if (res == DOUBLE_MISSING) {
			res = DOUBLE_MISSING - 1;
		}
		ruleToValue[id] = res;
	}
	if (res == DOUBLE_MISSING - 1) {
		return DOUBLE_MISSING;
	}
	return res;
}

bool GeneralRouter::containsAttribute(string attribute) {
	return attributes.find(attribute) != attributes.end();
}

string GeneralRouter::getAttribute(string attribute) {
	return attributes[attribute];
}

float GeneralRouter::getFloatAttribute(string attr, float defVal) {
	return parseFloat(getAttribute(attr), defVal);
}

int GeneralRouter::getIntAttribute(string attr, int defVal) {
	return (int)parseFloat(getAttribute(attr), (float)defVal);
}

double GeneralRouter::evaluateCache(RouteDataObjectAttribute attr, const SHARED_PTR<RouteDataObject>& way, double def) {
	return evaluateCache(attr, way->region, way->types, def, false, false);
}

SHARED_PTR<vector<uint32_t>> filterDirectionTags(const SHARED_PTR<RoutingIndex>& reg, vector<uint32_t>& pointTypes, bool forwardDir) {
	int wayDirection = forwardDir ? 1 : -1;
	int direction = 0;
	int tdirection = 0;
	int hdirection = 0;
	std::shared_ptr<vector<uint32_t>> ptr = nullptr;
	for (uint32_t type : pointTypes) {
		if (type == reg->directionBackward) {
			direction = -1;
		} else if (type == reg->directionForward) {
			direction = 1;
		} else if (type == reg->directionTrafficSignalsBackward) {
			tdirection = -1;
		} else if (type == reg->directionTrafficSignalsForward) {
			tdirection = 1;
		} else if (type == reg->maxheightBackward) {
			hdirection = -1;
		} else if (type == reg->maxheightForward) {
			hdirection = 1;
		}
	}
	if (direction != 0 || tdirection != 0 || hdirection != 0) {
		ptr = shared_ptr<vector<uint32_t>>(new vector<uint32_t>());
		for (uint32_t type : pointTypes) {
			if (((type == reg->stopSign || type == reg->giveWaySign) && direction == wayDirection) ||
				(type == reg->trafficSignals && tdirection == wayDirection) ||
				(hdirection == wayDirection)) {
				continue;
			}
			ptr->push_back(type);
		}
	}
	return ptr;
}

double GeneralRouter::evaluateCache(RouteDataObjectAttribute attr, const SHARED_PTR<RoutingIndex>& reg, std::vector<uint32_t>& types,
									double def, bool extra, bool filter) {
	auto regCacheIt = cacheEval[(unsigned int)attr].find(reg);
	MAP_INTV_DOUBLE* regCache;
	if (regCacheIt == cacheEval[(unsigned int)attr].end()) {
		cacheEval[(unsigned int)attr][reg] = MAP_INTV_DOUBLE();
		regCache = &cacheEval[(unsigned int)attr][reg];
	} else {
		regCache = &regCacheIt->second;
	}
	types.push_back(extra ? 1 : 0);
	auto r = regCache->find(types);
	types.pop_back();
	if (r != regCache->end()) {
		return r->second;
	}
	SHARED_PTR<vector<uint32_t>> ptr = nullptr;
	if (filter) {
		ptr = filterDirectionTags(reg, types, extra);
	}
	double res = getObjContext(attr).evaluateDouble(reg, ptr != nullptr ? *ptr : types, def);
	types.push_back(extra ? 1 : 0);
	(*regCache)[types] = res;
	types.pop_back();
	return res;
}

bool GeneralRouter::acceptLine(const SHARED_PTR<RouteDataObject>& way) {
	int res = (int)evaluateCache(RouteDataObjectAttribute::ACCESS, way, 0);
	if (impassableRoadIds.find(way->id) != impassableRoadIds.end()) {
		return false;
	}
	return res >= 0;
}

int GeneralRouter::isOneWay(const SHARED_PTR<RouteDataObject>& road) {
	return (int)evaluateCache(RouteDataObjectAttribute::ONEWAY, road, 0);
}

bool GeneralRouter::isArea(const SHARED_PTR<RouteDataObject>& road) {
	return ((int)evaluateCache(RouteDataObjectAttribute::AREA, road, 0)) == 1;
}

double GeneralRouter::defineObstacle(const SHARED_PTR<RouteDataObject>& road, uint point, bool dir) {
	if (road->pointTypes.size() > point && road->pointTypes[point].size() > 0) {
		return evaluateCache(RouteDataObjectAttribute::OBSTACLES, road->region, road->pointTypes[point], 0, dir, true);
	}
	return 0;
}

double GeneralRouter::defineHeightObstacle(const SHARED_PTR<RouteDataObject>& road, uint startIndex, uint endIndex) {
	if (!heightObstacles) {
		return 0;
	}
	vector<double> heightArray = road->calculateHeightArray();
	if (heightArray.size() == 0) {
		return 0;
	}

	double sum = 0;
	int knext;
	vector<uint32_t> types;
	RouteAttributeContext& objContext = getObjContext(RouteDataObjectAttribute::OBSTACLE_SRTM_ALT_SPEED);
	for (uint k = startIndex; k != endIndex; k = knext) {
		knext = startIndex < endIndex ? k + 1 : k - 1;
		double dist = startIndex < endIndex ? heightArray[2 * knext] : heightArray[2 * k];
		double diff = heightArray[2 * knext + 1] - heightArray[2 * k + 1];
		if (diff != 0 && dist > 0) {
			double incl = abs(diff / dist);
			int percentIncl = (int)(incl * 100);
			percentIncl = (percentIncl + 2) / 3 * 3 - 2;  // 1, 4, 7, 10, .
			if (percentIncl >= 1) {
				objContext.paramContext.incline = diff > 0 ? percentIncl : -percentIncl;
				sum += objContext.evaluateDouble(road->region, road->types, 0) * (diff > 0 ? diff : -diff);
			}
		}
	}
	return sum;
}

double GeneralRouter::defineRoutingObstacle(const SHARED_PTR<RouteDataObject>& road, uint point, bool dir) {
	if (road->pointTypes.size() > point && road->pointTypes[point].size() > 0) {
		return evaluateCache(RouteDataObjectAttribute::ROUTING_OBSTACLES, road->region, road->pointTypes[point], 0, dir, true);
	}
	return 0;
}

double GeneralRouter::defineRoutingSpeed(const SHARED_PTR<RouteDataObject>& road, bool dir) {
    double spd = evaluateCache(RouteDataObjectAttribute::ROAD_SPEED, road->region, road->types, defaultSpeed, dir, false);
	return max(min(spd, maxSpeed), minSpeed);
}

double GeneralRouter::defineVehicleSpeed(const SHARED_PTR<RouteDataObject>& road, bool dir) {
    double spd = evaluateCache(RouteDataObjectAttribute::ROAD_SPEED, road->region, road->types, defaultSpeed, dir, false);
	return max(min(spd, maxVehicleSpeed), minSpeed);
}

double GeneralRouter::definePenaltyTransition(const SHARED_PTR<RouteDataObject>& road) {
	if (!isObjContextAvailable(RouteDataObjectAttribute::PENALTY_TRANSITION)) {
		return 0;
	}
	return evaluateCache(RouteDataObjectAttribute::PENALTY_TRANSITION, road, 0);
}

double GeneralRouter::defineSpeedPriority(const SHARED_PTR<RouteDataObject>& road, bool dir) {
    evaluateCache(RouteDataObjectAttribute::ROAD_PRIORITIES, road, 1.);
    return evaluateCache(RouteDataObjectAttribute::ROAD_PRIORITIES, road->region, road->types, 1., dir, false);
}

double GeneralRouter::defineDestinationPriority(const SHARED_PTR<RouteDataObject>& road) {
	return evaluateCache(RouteDataObjectAttribute::DESTINATION_PRIORITIES, road, 1.);
}

double GeneralRouter::getDefaultSpeed() {
	return defaultSpeed;
}

double GeneralRouter::getMinSpeed() {
	return minSpeed;
}

double GeneralRouter::getMaxSpeed() {
	return maxSpeed;
}

bool GeneralRouter::restrictionsAware() {
	return _restrictionsAware;
}

double GeneralRouter::calculateTurnTime(const SHARED_PTR<RouteSegment>& segment, const SHARED_PTR<RouteSegment>& prev) {
	double ts = definePenaltyTransition(segment->getRoad());
	double prevTs = definePenaltyTransition(prev->getRoad());

	double totalPenalty = 0;

	if (prevTs != ts) {
		totalPenalty += abs(ts - prevTs) / 2;
	}

	// if(prev->road->pointTypes.size() > (uint)prevSegmentEnd && prev->road->pointTypes[prevSegmentEnd].size() > 0){
	// 	RoutingIndex* reg = prev->getRoad()->region;
	// 	vector<uint32_t> pt = prev->road->pointTypes[prevSegmentEnd];
	// 	for (uint i = 0; i < pt.size(); i++) {
	// 		tag_value r = reg->decodingRules[pt[i]];
	// 		if ("highway" == r.first && "traffic_signals" == r.second) {
	// 			// traffic signals don't add turn info
	// 			return 0;
	// 		}
	// 	}
	// }

	if (shortestRoute) {
		return totalPenalty;
	}

	if (segment->getRoad()->roundabout() && !prev->getRoad()->roundabout()) {
		double rt = roundaboutTurn;
		if (rt > 0) {
			totalPenalty += rt;
		}
	} else if (sharpTurn > 0 || slightTurn > 0) {
        double a1 = segment->getRoad()->directionRoute(segment->getSegmentStart(), segment->isPositive());
        double a2 = prev->getRoad()->directionRoute(prev->getSegmentEnd(), !prev->isPositive());
		double diff = abs(alignAngleDifference(a1 - a2 - M_PI));
		// more like UT
		if (diff > 2 * M_PI / 3) {
			totalPenalty += sharpTurn;
		} else if (diff > M_PI / 3) {
			totalPenalty += slightTurn;
		}
	}
	return totalPenalty;
}

std::vector<std::string> GeneralRouter::serializeParameterValues(MAP_STR_STR vls) {
    std::vector<std::string> ls;
    for (auto const& e : vls) {
        const auto & it = parameters.find(e.first);
        if (it != parameters.end()) {
            if (it->second.type == RoutingParameterType::BOOLEAN) {
                ls.push_back(e.first);
            } else {
                ls.push_back(e.first + "=" + e.second);
            }
        }
    }
    return ls;
}

void GeneralRouter::printRules() {
	for (uint k = 0; k < objectAttributes.size(); k++) {
		OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "RouteAttributeContext  %d", k + 1);
		objectAttributes[k]->printRules();
	}
}

uint GeneralRouter::registerTagValueAttribute(const tag_value& r) {
	string key = r.first + "$" + r.second;
	MAP_STR_INT::iterator it = universalRules.find(key);
	if (it != universalRules.end()) {
		return ((uint)it->second);
	}
	int id = (int)universalRules.size();
	universalRulesById.push_back(r);
	universalRules[key] = id;
	dynbitset& d = increaseSize(tagRuleMask[r.first], id + 1);
	d.set(id);
	return id;
}

uint64_t GeneralRouter::getBitSetSize() {
	return universalRules.size();
}

dynbitset RouteAttributeContext::convert(const SHARED_PTR<RoutingIndex>& reg, std::vector<uint32_t>& types) {
	dynbitset b(router->universalRules.size());
	MAP_INT_INT map;
	if (router->regionConvert.find(reg) != router->regionConvert.end()) {
		map = router->regionConvert[reg];
	} else {
		router->regionConvert[reg] = map;
	}
	for (uint k = 0; k < types.size(); k++) {
		MAP_INT_INT::iterator nid = map.find(types[k]);
		int vl;
		if (nid == map.end()) {
			auto i = types[k];
			if (reg->routeEncodingRules.size() > i) {
				auto& r = reg->routeEncodingRules[i];
				vl = router->registerTagValueAttribute(tag_value(r.getTag(), r.getValue()));
				map[types[k]] = vl;
			} else {
				continue;
			}
		} else {
			vl = nid->second;
		}
		increaseSize(b, (uint)router->universalRules.size()).set(vl);
	}
	return b;
}

double RouteAttributeEvalRule::eval(dynbitset& types, ParameterContext& paramContext, GeneralRouter* router) {
	if (matches(types, paramContext, router)) {
		return calcSelectValue(types, paramContext, router);
	}
	return DOUBLE_MISSING;
}

double RouteAttributeEvalRule::calcSelectValue(dynbitset& types, ParameterContext& paramContext,
											   GeneralRouter* router) {
	if (selectExpression.expressionType != 0) {
		selectValue = selectExpression.calculateExprValue(types, paramContext, router);
	}
	if (selectValue != DOUBLE_MISSING) {
		return selectValue;
	}
	if (selectValueDef.length() > 0 && selectValueDef[0] == '$') {
		UNORDERED(map)<string, dynbitset>::iterator ms = router->tagRuleMask.find(selectValueDef.substr(1));
		if (ms != router->tagRuleMask.end() && align(ms->second, types.size()).intersects(types)) {
			dynbitset findBit(ms->second.size());
			findBit |= ms->second;
			findBit &= types;
			uint value = findBit.find_first();
			double vd = router->parseValueFromTag(value, selectType, router);
			;
			return vd;
		}
	} else if (selectValueDef.length() > 0 && selectValueDef[0] == ':') {
		string p = selectValueDef.substr(1);
		MAP_STR_STR::iterator it = paramContext.vars.find(p);
		if (it != paramContext.vars.end()) {
			selectValue = parseValue(it->second, selectType);
		} else {
			return DOUBLE_MISSING;
		}
	}
	return selectValue;
}

bool RouteAttributeExpression::matches(dynbitset& types, ParameterContext& paramContext, GeneralRouter* router) {
	double f1 = calculateExprValue(0, types, paramContext, router);
	double f2 = calculateExprValue(1, types, paramContext, router);
	if (f1 == DOUBLE_MISSING || f2 == DOUBLE_MISSING) {
		return false;
	}
	if (expressionType == LESS_EXPRESSION) {
		return f1 <= f2;
	} else if (expressionType == GREAT_EXPRESSION) {
		return f1 >= f2;
	} else if (expressionType == EQUAL_EXPRESSION) {
		return f1 == f2;
	}
	return false;
}

double RouteAttributeExpression::calculateExprValue(dynbitset& types, ParameterContext& paramContext, GeneralRouter* router) {
	double f1 = calculateExprValue(0, types, paramContext, router);
	double f2 = calculateExprValue(1, types, paramContext, router);
	if (f1 != DOUBLE_MISSING && f2 != DOUBLE_MISSING) {
		switch (expressionType) {
			case MIN_EXPRESSION:
				return min(f1, f2);
			case MAX_EXPRESSION:
				return max(f1, f2);
		}
	}
	return DOUBLE_MISSING;
}

double RouteAttributeExpression::calculateExprValue(int id, dynbitset& types, ParameterContext& paramContext,
													GeneralRouter* router) {
	double cacheValue = cacheValues[id];
	string value = values[id];
	double o = DOUBLE_MISSING;
	if (cacheValue != DOUBLE_MISSING) {
		return cacheValue;
	}
	if (value.length() > 0 && value[0] == '$') {
		UNORDERED(map)<string, dynbitset>::iterator ms = router->tagRuleMask.find(value.substr(1));
		if (ms != router->tagRuleMask.end() && align(ms->second, types.size()).intersects(types)) {
			dynbitset findBit(ms->second.size());
			findBit |= ms->second;
			findBit &= types;
			uint value = findBit.find_first();
			return router->parseValueFromTag(value, valueType, router);
		}
	} else if (value == ":incline") {
		return paramContext.incline;
	} else if (value.length() > 0 && value[0] == ':') {
		string p = value.substr(1);
		MAP_STR_STR::iterator it = paramContext.vars.find(p);
		if (it != paramContext.vars.end()) {
			o = parseValue(it->second, valueType);
		} else {
			return DOUBLE_MISSING;
		}
	}
	cacheValues[id] = o;
	return o;
}

bool RouteAttributeEvalRule::matches(dynbitset& types, ParameterContext& paramContext, GeneralRouter* router) {
	if (!checkAllTypesShouldBePresent(types, paramContext, router)) {
		return false;
	}
	if (!checkAllTypesShouldNotBePresent(types, paramContext, router)) {
		return false;
	}
	if (!checkFreeTags(types, paramContext, router)) {
		return false;
	}
	if (!checkNotFreeTags(types, paramContext, router)) {
		return false;
	}
	if (!checkExpressions(types, paramContext, router)) {
		return false;
	}
	return true;
}

bool RouteAttributeEvalRule::checkExpressions(dynbitset& types, ParameterContext& paramContext, GeneralRouter* router) {
	for (uint i = 0; i < conditionExpressions.size(); i++) {
		if (!conditionExpressions[i].matches(types, paramContext, router)) {
			return false;
		}
	}
	return true;
}

bool RouteAttributeEvalRule::checkFreeTags(dynbitset& types, ParameterContext& paramContext, GeneralRouter* router) {
	for (UNORDERED(set) < string > ::iterator it = onlyTags.begin(); it != onlyTags.end(); it++) {
		UNORDERED(map)<string, dynbitset>::iterator ms = router->tagRuleMask.find(*it);
		if (ms == router->tagRuleMask.end() || !align(ms->second, types.size()).intersects(types)) {
			return false;
		}
	}
	return true;
}

bool RouteAttributeEvalRule::checkNotFreeTags(dynbitset& types, ParameterContext& paramContext, GeneralRouter* router) {
	for (UNORDERED(set) < string > ::iterator it = onlyNotTags.begin(); it != onlyNotTags.end(); it++) {
		UNORDERED(map)<string, dynbitset>::iterator ms = router->tagRuleMask.find(*it);
		if (ms != router->tagRuleMask.end() && align(ms->second, types.size()).intersects(types)) {
			return false;
		}
	}
	return true;
}

bool RouteAttributeEvalRule::checkAllTypesShouldNotBePresent(dynbitset& types, ParameterContext& paramContext,
															 GeneralRouter* router) {
	if (align(filterNotTypes, types.size()).intersects(types)) {
		return false;
	}
	return true;
}

bool RouteAttributeEvalRule::checkAllTypesShouldBePresent(dynbitset& types, ParameterContext& paramContext,
														  GeneralRouter* router) {
	// Bitset method subset is missing "filterTypes.isSubset(types)"
	// reset previous evaluation
	return align(filterTypes, types.size()).is_subset_of(types);
	// evalFilterTypes.clear();
	// evalFilterTypes |= filterTypes;
	//// evaluate bit intersection and check if filterTypes contained as set in types
	// evalFilterTypes &= types;
	// if(!evalFilterTypes == filterTypes) {
	//	return false;
	//}
	// return true;
}

#endif /*_OSMAND_GENERAL_ROUTER_CPP*/
