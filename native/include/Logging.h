#ifndef _OSMAND_LOGGING_H_
#define _OSMAND_LOGGING_H_

#include "CommonCollections.h"

#if defined(__clang__) || defined(__GNUC__)
#define CHECK_PRINTF_FORMAT __attribute__((format(printf, 2, 3))) // gcc/clang -Wformat for custom printf-functions
#else
#define CHECK_PRINTF_FORMAT
#endif

namespace OsmAnd {
	enum class LogSeverityLevel { Error = 1, Warning, Debug, Info };
	void LogPrintf(LogSeverityLevel level, const char* format, ...) CHECK_PRINTF_FORMAT;
}  // namespace OsmAnd

#endif	// _OSMAND_LOGGING_H_
