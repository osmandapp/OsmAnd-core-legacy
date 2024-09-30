#ifndef _OSMAND_HEIGHTMAP_RENDERER_H
#define _OSMAND_HEIGHTMAP_RENDERER_H

#include <string>
#include <fstream>
#include <gdal_priv.h>
#include <gdal_utils.h>
#include <cpl_conv.h>

#include "Logging.h"

enum class RasterType
{
    Heightmap,
	Hillshade,
	Slope,
	Height
};        

struct ProcessingParameters
{
	RasterType rasterType;
	std::string resultColorsFilename;
	std::string intermediateColorsFilename;
};

template<typename T>
struct Point
{
	T x, y;

	inline Point()
	{
		this->x = 0;
		this->y = 0;
	}

	inline Point(const Point<T>& that)
	{
		this->x = that.x;
		this->y = that.y;
	}

	template<typename T_>
	explicit inline Point(const Point<T_>& that)
	{
		this->x = static_cast<T>(that.x);
		this->y = static_cast<T>(that.y);
	}

	inline Point(const T& x, const T& y)
	{
		this->x = x;
		this->y = y;
	}
};
typedef Point<double> PointD;
typedef Point<float> PointF;
typedef Point<int32_t> PointI;
typedef Point<int64_t> PointI64;

inline static double metersFrom31(const double position31)
{
	auto earthInMeters = 2.0 * M_PI * 6378137;
	auto earthIn31 = 1.0 + INT32_MAX;
	return (position31 / earthIn31 - 0.5) * earthInMeters;
}

inline static PointD metersFrom31(const PointD& location31)
{
	auto earthIn31 = 1.0 + INT32_MAX;
	return PointD(metersFrom31(location31.x), metersFrom31(earthIn31 - location31.y));
}

uint64_t multiplyParts(const uint64_t shade, const uint64_t slope);
void blendHillshade(const uint32_t tileSize, char* shade, char* slope, char* blend);
void mergeHeights(const uint32_t tileSize, const float scaleFactor, const bool forceReplace,
	void* destination, float* source);
bool postProcess(
	const char* pByteBuffer,
	const ProcessingParameters& procParameters,
	const uint32_t tileSize,
	const uint32_t overlap,
	const PointD& tileOrigin,
	const PointD& tileResolution,
	const int gcpCount,
	const void* gcpList,
	void* pBuffer);

bool getGeotiffData(std::string& tilePath, std::string& outColorFilename, std::string& midColorFilename,
	int type, int size, int zoom, int xTileCoord, int yTileCoord, void* pBuffer);

#endif /*_OSMAND_HEIGHTMAP_RENDERER_H*/
