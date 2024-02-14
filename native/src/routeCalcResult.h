#ifndef _OSMAND_ROUTE_CALC_RESULT_H
#define _OSMAND_ROUTE_CALC_RESULT_H

#include "CommonCollections.h"
#include "routeSegmentResult.h"

struct RouteCalcResult {
    std::vector<SHARED_PTR<RouteSegmentResult>> detailed;
    std::string error;
    
    RouteCalcResult(): error("") {
    }
    
    RouteCalcResult(std::vector<SHARED_PTR<RouteSegmentResult>> list) {
        if(list.size() == 0) {
            error = "Result is empty";
        } else {
            detailed = list;
        }
    }
		
	RouteCalcResult(std::string error): error(error) {
    }
		
    std::vector<SHARED_PTR<RouteSegmentResult>> getList() {
        return detailed;
    }
		
	std::string getError() {
        return error;
    }

	bool isCorrect() {
        return error.empty() && !detailed.empty();
    }
};

#endif /*_OSMAND_ROUTE_CALC_RESULT_H*/
