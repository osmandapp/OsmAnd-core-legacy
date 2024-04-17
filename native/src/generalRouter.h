#ifndef _OSMAND_GENERAL_ROUTER_H
#define _OSMAND_GENERAL_ROUTER_H

#include   <algorithm>

#include "CommonCollections.h"
#include "binaryRead.h"
#include "boost/dynamic_bitset.hpp"
#include "boost/functional/hash.hpp"
#include "commonOsmAndCore.h"

struct RouteSegment;
class GeneralRouter;
class RouteAttributeContext;

//#include "binaryRoutePlanner.h"

template <typename Container>  // we can make this generic for any container [1]
struct container_hash {
	std::size_t operator()(Container const& c) const {
		return boost::hash_range(c.begin(), c.end());
	}
};

typedef UNORDERED(map)<string, float> MAP_STR_FLOAT;
typedef UNORDERED(map)<string, string> MAP_STR_STR;
typedef UNORDERED(map)<int, int> MAP_INT_INT;
typedef UNORDERED(map)<vector<uint32_t>, double, container_hash<vector<uint32_t>>> MAP_INTV_DOUBLE;
typedef UNORDERED(map)<string, int> MAP_STR_INT;
typedef boost::dynamic_bitset<> dynbitset;

#define DOUBLE_MISSING -1.1e9  // random big negative number

struct GeneralRouterConstants {
	static const double CAR_SHORTEST_DEFAULT_SPEED;
	static const double BICYCLE_SHORTEST_DEFAULT_SPEED;

	static const char* USE_SHORTEST_WAY;
	static const char* USE_HEIGHT_OBSTACLES;
	static const char* GROUP_RELIEF_SMOOTHNESS_FACTOR;
	static const char* ALLOW_PRIVATE;
	static const char* CHECK_ALLOW_PRIVATE_NEEDED;
	static const char* ALLOW_PRIVATE_FOR_TRUCK;
	static const char* DEFAULT_SPEED;
	static const char* MIN_SPEED;
	static const char* MAX_SPEED;
};

enum class RouteDataObjectAttribute : unsigned int {
	ROAD_SPEED = 0,				  //"speed"
	ROAD_PRIORITIES = 1,		  // "priority"
	DESTINATION_PRIORITIES = 2,	  // "destination_priority"
	ACCESS = 3,					  // "access"
	OBSTACLES = 4,				  // "obstacle_time"
	ROUTING_OBSTACLES = 5,		  // "obstacle"
	ONEWAY = 6,					  // "oneway"
	PENALTY_TRANSITION = 7,		  // "penalty_transition"
	OBSTACLE_SRTM_ALT_SPEED = 8,  // "obstacle_srtm_alt_speed"
	AREA = 9,					  // "area"

	UNDEFINED = 100000,
	COUNT = 10
};

static RouteDataObjectAttribute parseRouteDataObjectAttribute(string attr, RouteDataObjectAttribute def) {
	if ("speed" == to_lowercase(attr)) {
		return RouteDataObjectAttribute::ROAD_SPEED;
	} else if ("priority" == to_lowercase(attr)) {
		return RouteDataObjectAttribute::ROAD_PRIORITIES;
	} else if ("destination_priority" == to_lowercase(attr)) {
		return RouteDataObjectAttribute::DESTINATION_PRIORITIES;
	} else if ("access" == to_lowercase(attr)) {
		return RouteDataObjectAttribute::ACCESS;
	} else if ("obstacle_time" == to_lowercase(attr)) {
		return RouteDataObjectAttribute::OBSTACLES;
	} else if ("obstacle" == to_lowercase(attr)) {
		return RouteDataObjectAttribute::ROUTING_OBSTACLES;
	} else if ("oneway" == to_lowercase(attr)) {
		return RouteDataObjectAttribute::ONEWAY;
	} else if ("penalty_transition" == to_lowercase(attr)) {
		return RouteDataObjectAttribute::PENALTY_TRANSITION;
	} else if ("obstacle_srtm_alt_speed" == to_lowercase(attr)) {
		return RouteDataObjectAttribute::OBSTACLE_SRTM_ALT_SPEED;
	} else if ("area" == to_lowercase(attr)) {
		return RouteDataObjectAttribute::AREA;
	} else {
		return def;
	}
}

enum class GeneralRouterProfile { CAR, PEDESTRIAN, BICYCLE, BOAT, SKI, MOPED, TRAIN, PUBLIC_TRANSPORT, HORSEBACKRIDING };

static GeneralRouterProfile parseGeneralRouterProfile(string profile, GeneralRouterProfile def) {
	if ("car" == to_lowercase(profile)) {
		return GeneralRouterProfile::CAR;
	} else if ("pedestrian" == to_lowercase(profile)) {
		return GeneralRouterProfile::PEDESTRIAN;
	} else if ("bicycle" == to_lowercase(profile)) {
		return GeneralRouterProfile::BICYCLE;
	} else if ("ski" == to_lowercase(profile)) {
			return GeneralRouterProfile::SKI;
	} else if ("moped" == to_lowercase(profile)) {
			return GeneralRouterProfile::MOPED;
	} else if ("train" == to_lowercase(profile)) {
			return GeneralRouterProfile::TRAIN;
	} else if ("public_transport" == to_lowercase(profile)) {
		return GeneralRouterProfile::PUBLIC_TRANSPORT;
	} else if ("horsebackriding" == to_lowercase(profile)) {
		return GeneralRouterProfile::HORSEBACKRIDING;
	} else if ("boat" == to_lowercase(profile)) {
		return GeneralRouterProfile::BOAT;
	} else {
		return def;
	}
}

static std::string profileToString(GeneralRouterProfile prof) {
	switch(prof) {
		case GeneralRouterProfile::CAR:
			return "car";
		case GeneralRouterProfile::PEDESTRIAN:
			return "pedestrian";
		case GeneralRouterProfile::BICYCLE:
			return "bicycle";
		case GeneralRouterProfile::BOAT:
			return "boat";
		case GeneralRouterProfile::PUBLIC_TRANSPORT:
			return "public_transport";
		case GeneralRouterProfile::SKI:
			return "ski";
		case GeneralRouterProfile::MOPED:
			return "moped";
		case GeneralRouterProfile::TRAIN:
			return "train";
		default:
			return "";
	}
}

enum class RoutingParameterType { NUMERIC, BOOLEAN, SYMBOLIC };

struct RoutingParameter {
	string id;
	string group;
	string name;
	string description;
	RoutingParameterType type;
	vector<double> possibleValues;	// Object TODO;
	vector<string> possibleValueDescriptions;
	vector<string> profiles;
	bool defaultBoolean;
};

struct ParameterContext {
	MAP_STR_STR vars;
	double incline;
	ParameterContext() : incline(0) {
	}
};

struct RouteAttributeExpression {
	static const int LESS_EXPRESSION;
	static const int GREAT_EXPRESSION;
	static const int EQUAL_EXPRESSION;
	static const int MIN_EXPRESSION;
	static const int MAX_EXPRESSION;

	vector<string> values;
	int expressionType;
	string valueType;
	vector<double> cacheValues;

	RouteAttributeExpression(): expressionType(0){};
	RouteAttributeExpression(vector<string>& vls, int type, string vType);

	bool matches(dynbitset& types, ParameterContext& paramContext, GeneralRouter* router);
	double calculateExprValue(dynbitset& types, ParameterContext& paramContext, GeneralRouter* router);
	double calculateExprValue(int id, dynbitset& types, ParameterContext& paramContext, GeneralRouter* router);
};

class RouteAttributeEvalRule {
	friend class RouteAttributeContext;

   private:
	vector<string> parameters;
	double selectValue;
	string selectValueDef;
	string selectType;
	dynbitset filterTypes;
	dynbitset filterNotTypes;

	UNORDERED(set)<string> onlyTags;
	UNORDERED(set)<string> onlyNotTags;
	vector<RouteAttributeExpression> conditionExpressions;

	vector<string> tagValueCondDefValue;
	vector<string> tagValueCondDefTag;
	vector<bool> tagValueCondDefNot;

	bool matches(dynbitset& types, ParameterContext& paramContext, GeneralRouter* router);
	double eval(dynbitset& types, ParameterContext& paramContext, GeneralRouter* router);
	double calcSelectValue(dynbitset& types, ParameterContext& paramContext, GeneralRouter* router);

	bool checkAllTypesShouldBePresent(dynbitset& types, ParameterContext& paramContext, GeneralRouter* router);
	bool checkAllTypesShouldNotBePresent(dynbitset& types, ParameterContext& paramContext, GeneralRouter* router);
	bool checkNotFreeTags(dynbitset& types, ParameterContext& paramContext, GeneralRouter* router);
	bool checkFreeTags(dynbitset& types, ParameterContext& paramContext, GeneralRouter* router);
	bool checkExpressions(dynbitset& types, ParameterContext& paramContext, GeneralRouter* router);

	void printRule(GeneralRouter* r);

   public:
	RouteAttributeExpression selectExpression;
	RouteAttributeEvalRule() : selectValue(0), selectValueDef(""), selectType("") {
	}

	RouteAttributeEvalRule(const SHARED_PTR<RouteAttributeEvalRule>& original);

	void registerAndTagValueCondition(GeneralRouter* r, string tag, string value, bool nt);

	// formated as [param1,-param2]
	void registerParamConditions(vector<string>& params);

	void registerAndParamCondition(string param, bool nt);

	void registerSelectValue(string selectValue, string selectType);

	void registerExpression(RouteAttributeExpression& expression) {
		conditionExpressions.push_back(expression);
	}

	void registerLessCondition(string value1, string value2, string valueType) {
		vector<string> vls{value1, value2};
		RouteAttributeExpression exp(vls, RouteAttributeExpression::LESS_EXPRESSION, valueType);
		registerExpression(exp);
	}

	void registerGreatCondition(string value1, string value2, string valueType) {
		vector<string> vls{value1, value2};
		RouteAttributeExpression exp(vls, RouteAttributeExpression::GREAT_EXPRESSION, valueType);
		registerExpression(exp);
	}

	void registerEqualCondition(string value1, string value2, string valueType) {
		vector<string> vls{value1, value2};
		RouteAttributeExpression exp(vls, RouteAttributeExpression::EQUAL_EXPRESSION, valueType);
		registerExpression(exp);
	}

	void registerMinExpression(string value1, string value2, string valueType) {
		vector<string> vls{value1, value2};
		RouteAttributeExpression exp(vls, RouteAttributeExpression::MIN_EXPRESSION, valueType);
		selectExpression = exp;
	}

	void registerMaxExpression(string value1, string value2, string valueType) {
		vector<string> vls{value1, value2};
		RouteAttributeExpression exp(vls, RouteAttributeExpression::MAX_EXPRESSION, valueType);
		selectExpression = exp;
	}
};

class RouteAttributeContext {
	friend class GeneralRouter;

   private:
	vector<SHARED_PTR<RouteAttributeEvalRule>> rules;
	ParameterContext paramContext;
	GeneralRouter* router;

   public:
	RouteAttributeContext(GeneralRouter* r) : router(r) {
	}

	RouteAttributeContext(GeneralRouter* r, RouteAttributeContext* original, MAP_STR_STR params) : router(r) {
		if (!params.empty()) {
			paramContext.vars = params;
		}
		for (auto rt : original->rules) {
			if (checkParameter(rt)) {
				rules.push_back(std::make_shared<RouteAttributeEvalRule>(rt));
			}
		}
	}

	bool checkParameter(SHARED_PTR<RouteAttributeEvalRule>& r) {
		if (r->parameters.size() > 0) {
			for (string p : r->parameters) {
				bool _not = false;
				if (!p.empty() && p[0] == '-') {
					_not = true;
					p = p.substr(1);
				}
				auto& vars = paramContext.vars;
				bool val = vars.find(p) != vars.end();
				if (_not && val) {
					return false;
				} else if (!_not && !val) {
					return false;
				}
			}
		}
		return true;
	}

	void registerParams(vector<string>& keys, vector<string>& vls) {
		for (uint i = 0; i < keys.size(); i++) {
			paramContext.vars[keys[i]] = vls[i];
		}
	}

	SHARED_PTR<RouteAttributeEvalRule> newEvaluationRule() {
		auto c = std::make_shared<RouteAttributeEvalRule>();
		rules.push_back(c);
		return rules.back();
	}

	void printRules() {
		for (uint k = 0; k < rules.size(); k++) {
			auto r = rules[k];
			r->printRule(router);
		}
	}

	SHARED_PTR<RouteAttributeEvalRule> getLastRule() {
		return rules.back();
	}

	int evaluateInt(dynbitset& rawTypes, int defValue) {
		double o = evaluate(rawTypes);
		if (o == DOUBLE_MISSING) {
			return defValue;
		}
		return (int)o;
	}

	float evaluateFloat(dynbitset& rawTypes, float defValue) {
		double o = evaluate(rawTypes);
		if (o == DOUBLE_MISSING) {
			return defValue;
		}
		return (float)o;
	}

   private:
	double evaluate(dynbitset& types) {
		for (uint k = 0; k < rules.size(); k++) {
			double o = rules[k]->eval(types, paramContext, router);
			if (o != DOUBLE_MISSING) {
				return o;
			}
		}
		return DOUBLE_MISSING;
	}

	dynbitset convert(const SHARED_PTR<RoutingIndex>& reg, std::vector<uint32_t>& types);

	double evaluateDouble(const SHARED_PTR<RoutingIndex>& reg, std::vector<uint32_t>& types, double defValue) {
		dynbitset local = convert(reg, types);
		double d = evaluate(local);
		if (d == DOUBLE_MISSING) {
			return defValue;
		}
		return d;
	}
};

float parseFloat(MAP_STR_STR attributes, string key, float def);
float parseFloat(string value, float def);

bool parseBool(MAP_STR_STR attributes, string key, bool def);
bool parseBool(string value, bool def);

string parseString(MAP_STR_STR attributes, string key, string def);
string parseString(string value, string def);

class GeneralRouter {
	friend class RouteAttributeContext;
	friend class RouteAttributeEvalRule;
	friend struct RouteAttributeExpression;

   private:
	GeneralRouterProfile profile;
	vector<RouteAttributeContext*> objectAttributes;
	MAP_STR_STR attributes;
	vector<RoutingParameter> parametersList;
	UNORDERED(map)<string, RoutingParameter> parameters;
	MAP_STR_INT universalRules;
	MAP_STR_STR parameterValues;
	vector<tag_value> universalRulesById;
	UNORDERED(map)<string, dynbitset> tagRuleMask;
	vector<double> ruleToValue;	 // Object TODO;

	UNORDERED(map)<SHARED_PTR<RoutingIndex>, MAP_INT_INT> regionConvert;
	vector<UNORDERED(map) <SHARED_PTR<RoutingIndex>, MAP_INTV_DOUBLE>> cacheEval;

   public:
	// cached values
	bool _restrictionsAware;
	bool heightObstacles;
	double sharpTurn;
	double roundaboutTurn;
	double slightTurn;
	double minSpeed;
	double defaultSpeed;
	double maxSpeed;
	double maxVehicleSpeed;
	UNORDERED(set)<int64_t> impassableRoadIds;
	bool shortestRoute;
	bool allowPrivate;
	bool checkAllowPrivateNeeded;
	string profileName;
	string fileName;
	UNORDERED(map)<string, string> hhNativeFilter;

	GeneralRouter();
	GeneralRouter(const GeneralRouterProfile profile, const MAP_STR_STR& attributes = MAP_STR_STR());
	GeneralRouter(const GeneralRouter& parent, const MAP_STR_STR& params = MAP_STR_STR());

	~GeneralRouter() {
		for (uint k = 0; k < objectAttributes.size(); k++) {
			delete objectAttributes[k];
		}
	}

	SHARED_PTR<GeneralRouter> build(const MAP_STR_STR& params = MAP_STR_STR()) {
		return SHARED_PTR<GeneralRouter>(new GeneralRouter(*this, params));
	}

	GeneralRouterProfile getProfile() {
		return profile;
	}

	UNORDERED(map)<string, RoutingParameter>& getParameters() {
		return parameters;
	}

	vector<RoutingParameter>& getParametersList() {
		return parametersList;
	}

	void registerBooleanParameter(string id, string group, string name, string description, vector<string> profiles, bool defaultValue) {
		RoutingParameter rp{};
		rp.group = group;
		rp.name = name;
		rp.description = description;
		rp.id = id;
		rp.profiles = profiles;
		rp.type = RoutingParameterType::BOOLEAN;
		rp.defaultBoolean = defaultValue;
		parameters[rp.id] = rp;
		parametersList.push_back(rp);
	}

	void registerNumericParameter(string id, string name, string description, vector<double> vls, vector<string> profiles,
								  vector<string> vlsDescriptions) {
		RoutingParameter rp{};
		rp.name = name;
		rp.description = description;
		rp.id = id;
		rp.profiles = profiles;
		rp.possibleValues = vls;
		rp.possibleValueDescriptions = vlsDescriptions;
		rp.type = RoutingParameterType::NUMERIC;
		parameters[rp.id] = rp;
		parametersList.push_back(rp);
	}

	RouteAttributeContext* newRouteAttributeContext() {
		RouteAttributeContext* c = new RouteAttributeContext(this);
		objectAttributes.push_back(c);
		return objectAttributes.back();
	}

	RouteAttributeContext* newRouteAttributeContext(RouteAttributeContext* original, const MAP_STR_STR& params) {
		RouteAttributeContext* c = new RouteAttributeContext(this, original, params);
		objectAttributes.push_back(c);
		return objectAttributes.back();
	}
	
	UNORDERED_map<string, RoutingParameter> getParameters(const string &derivedProfile);
	
	void addAttribute(string k, string v);

	bool containsAttribute(string attribute);

	string getAttribute(string attribute);

	float getFloatAttribute(string attr, float defVal);

	int getIntAttribute(string attr, int defVal);

	uint64_t getBitSetSize();

	/**
	 * return if the road is accepted for routing
	 */
	bool acceptLine(const SHARED_PTR<RouteDataObject>& way);

	/**
	 * return oneway +/- 1 if it is oneway and 0 if both ways
	 */
	int isOneWay(const SHARED_PTR<RouteDataObject>& road);

	/**
	 * return true if area == 1 and false otherwise
	 */
	bool isArea(const SHARED_PTR<RouteDataObject>& road);

	/**
	 * return delay in seconds (0 no obstacles)
	 */
	double defineObstacle(const SHARED_PTR<RouteDataObject>& road, uint point, bool dir);

	/**
	 * return delay in seconds for height obstacles
	 */
	double defineHeightObstacle(const SHARED_PTR<RouteDataObject>& road, uint startIndex, uint endIndex);

	/**
	 * return delay in seconds (0 no obstacles)
	 */
	double defineRoutingObstacle(const SHARED_PTR<RouteDataObject>& road, uint point, bool dir);

	/**
	 * return routing speed in m/s for vehicle for specified road
	 */
	double defineRoutingSpeed(const SHARED_PTR<RouteDataObject>& road, bool dir);

	/*
	 * return transition penalty between different road classes in seconds
	 */
	double definePenaltyTransition(const SHARED_PTR<RouteDataObject>& road);

	/**
	 * return real speed in m/s for vehicle for specified road
	 */
	double defineVehicleSpeed(const SHARED_PTR<RouteDataObject>& road, bool dir);

	/**
	 * define priority to multiply the speed for g(x) A*
	 */
	double defineSpeedPriority(const SHARED_PTR<RouteDataObject>& road, bool dir);

	/**
	 * define destination priority
	 */
	double defineDestinationPriority(const SHARED_PTR<RouteDataObject>& road);

	/**
	 * Used for A* routing to calculate g(x)
	 *
	 * @return minimal speed at road in m/s
	 */
	double getDefaultSpeed();

	/**
	 * Used as minimal threshold of default speed
	 *
	 * @return minimal speed at road in m/s
	 */
	double getMinSpeed();

	/**
	 * Used for A* routing to predict h(x) : it should be great any g(x)
	 *
	 * @return maximum speed to calculate shortest distance
	 */
	double getMaxSpeed();

	/**
	 * aware of road restrictions
	 */
	bool restrictionsAware();

	/**
	 * Calculate turn time
	 */
	double calculateTurnTime(const SHARED_PTR<RouteSegment>& segment, const SHARED_PTR<RouteSegment>& prev);
	
	MAP_STR_STR getParameterValues() {
		return parameterValues;        
	}
	
	std::vector<std::string> serializeParameterValues(MAP_STR_STR vls);

	void printRules();
	
	void setProfile(GeneralRouterProfile prof) {
		profile = prof;
	};

   private:
	double parseValueFromTag(uint id, string type, GeneralRouter* router);

	double evaluateCache(RouteDataObjectAttribute attr, const SHARED_PTR<RoutingIndex>& reg, std::vector<uint32_t>& types, double def,
						 bool dir, bool filter);
	double evaluateCache(RouteDataObjectAttribute attr, const SHARED_PTR<RouteDataObject>& way, double def);

   public:
	uint registerTagValueAttribute(const tag_value& r);
	bool isObjContextAvailable(RouteDataObjectAttribute a) {
		return objectAttributes.size() > (unsigned int)a;
	}

	RouteAttributeContext& getObjContext(RouteDataObjectAttribute a) {
		return *objectAttributes[(unsigned int)a];
	}
};

#endif /*_OSMAND_GENERAL_ROUTER_H*/
