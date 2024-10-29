#include "heightmapRenderer.h"

#ifndef TIFF_NODATA
#define TIFF_NODATA (-32768.0)
#endif

inline uint64_t multiplyParts(const uint64_t shade, const uint64_t slope)
{
    const uint64_t mask0 = 0xFFULL;
    const uint64_t mask1 = 0xFF0000ULL;
    const uint64_t mask2 = 0xFF00000000ULL;
    const uint64_t mask3 = 0xFF000000000000ULL;
    return ((shade & mask0) != 0 ? (shade & mask0) * (slope & mask0) + 0x0100ULL : 0) |
        ((shade & mask1) != 0 ? (shade & mask1) * ((slope & mask1) >> 16) + 0x01000000ULL : 0) |
        ((shade & mask2) != 0 ? (shade & mask2) * ((slope & mask2) >> 32) + 0x010000000000ULL : 0) |
        ((shade & mask3) != 0 ? (shade & mask3) * (slope >> 48) + 0x0100000000000000ULL : 0);
}

inline void blendHillshade(
    const uint32_t tileSize,
    char* shade,
    char* slope,
    char* blend)
{
    if ((tileSize & 7) > 0)
    {
        // Simple pixel-by-pixel blending
        const auto pixelCount = tileSize * tileSize;
        for (uint32_t idx = 0; idx < pixelCount; idx++)
            blend[idx] =
                shade[idx] != 0 ? (static_cast<ushort>(shade[idx]) * static_cast<ushort>(slope[idx]) >> 8) + 1 : 0;
    }
    else
    {
        // Optimized blending
        const uint64_t bigMask = 0xFF00FF00FF00FF00ULL;
        const uint64_t litMask = 0x00FF00FF00FF00FFULL;
        const auto quadCount = tileSize * tileSize / 8;
        auto pShade = reinterpret_cast<uint64_t*>(shade);
        auto pSlope = reinterpret_cast<uint64_t*>(slope);
        auto pBlend = reinterpret_cast<uint64_t*>(blend);
        for (int i = 0; i < quadCount; i++)
        {
            auto quad = *pShade++;
            auto hsb = (quad & bigMask) >> 8;
            auto hsl = quad & litMask;
            quad = *pSlope++;
            auto spb = (quad & bigMask) >> 8;
            auto spl = quad & litMask;
            *pBlend = (multiplyParts(hsb, spb) & bigMask) | (multiplyParts(hsl, spl) >> 8 & litMask);
            pBlend++;
        }
    }
}

inline void mergeHeights(
    const uint32_t tileSize,
    const float scaleFactor,
    const bool forceReplace,
    char* destination,
    float* source)
{
    const auto noData = static_cast<float>(TIFF_NODATA);
    auto result = reinterpret_cast<float*>(destination);
    const auto valueCount = tileSize * tileSize;
    for (uint32_t idx = 0; idx < valueCount; idx++)
    {
        if (forceReplace || result[idx] == noData)
        {
            const auto src = source[idx];
            result[idx] = src == noData ? noData : src * scaleFactor;
        }
    }
}

inline bool postProcess(
    const char* pByteBuffer,
    const ProcessingParameters& procParameters,
    const uint32_t tileSize,
    const uint32_t overlap,
    const PointD& tileOrigin,
    const PointD& tileResolution,
    const int gcpCount,
    const void* gcpList,
    void* pBuffer)
{
    bool result = false;
    char dataPointer[24];
    auto pCharBuffer = const_cast<char*>(pByteBuffer);
    auto pGCPList = reinterpret_cast<const GDAL_GCP*>(gcpList);
    dataPointer[CPLPrintPointer(dataPointer, pCharBuffer, 23)] = 0;
    auto filename = new char[1024];
    sprintf(filename,
        "MEM:::DATATYPE=Float32,DATAPOINTER=%s,PIXELS=%d,LINES=%d,GEOTRANSFORM=%.17g/%.17g/0.0/%.17g/0.0/%.17g",
        dataPointer, tileSize, tileSize, tileOrigin.x, tileResolution.x, tileOrigin.y, tileResolution.y);
    if (const auto heightmapDataset = (GDALDataset*) GDALOpen(filename, GA_ReadOnly))
    {
        auto band = heightmapDataset->GetRasterBand(1);
        if (procParameters.rasterType == RasterType::Height)
        {
            // Produce colorized height raster
            if (band && heightmapDataset->SetGCPs(gcpCount, pGCPList, "") == CE_None)
            {
                band->SetNoDataValue(TIFF_NODATA);
                char outputFormat[] = "MEM";
                const char* colorizeArgs[] = { "-of", outputFormat, "-alpha", NULL };
                if (GDALDEMProcessingOptions* colorizeOptions = GDALDEMProcessingOptionsNew(
                    const_cast<char **>(colorizeArgs), NULL))
                {
                    const auto resultDataset = (GDALDataset*) GDALDEMProcessing(
                        outputFormat,
                        heightmapDataset,
                        "color-relief",
                        procParameters.resultColorsFilename.c_str(),
                        colorizeOptions,
                        nullptr);
                    if (resultDataset)
                    {
                        const auto resultOffset = overlap / 2;
                        const auto resultSize = tileSize - overlap;
                        result = resultDataset->RasterIO(GF_Read,
                            resultOffset, resultOffset, resultSize, resultSize,
                            pBuffer, resultSize, resultSize, GDT_Byte, 4, nullptr,
                            4, resultSize * 4, 1, nullptr) == CE_None;
                        GDALClose(resultDataset);
                    }
                    GDALDEMProcessingOptionsFree(colorizeOptions);
                }
            }
        }
        else if (band && heightmapDataset->SetGCPs(gcpCount, pGCPList, "") == CE_None)
        {
            band->SetNoDataValue(TIFF_NODATA);
            // Prepare common slope raster
            char outputFormat[] = "MEM";
            const char* slopeArgs[] = { "-of", outputFormat, NULL };
            if (GDALDEMProcessingOptions* slopeOptions = GDALDEMProcessingOptionsNew(
                const_cast<char **>(slopeArgs), NULL))
            {
                if (const auto slopeDataset = (GDALDataset*) GDALDEMProcessing(outputFormat, heightmapDataset,
                    "slope", nullptr, slopeOptions, nullptr))
                {
                    if (procParameters.rasterType == RasterType::Hillshade)
                    {
                        // Compose hybrid hillshade raster
                        const char* hillshadeArgs[] = { "-of", outputFormat, "-z", "2", NULL };
                        if (GDALDEMProcessingOptions* hillshadeOptions = GDALDEMProcessingOptionsNew(
                            const_cast<char **>(hillshadeArgs), NULL))
                        {
                            const auto hillshadeDataset = (GDALDataset*) GDALDEMProcessing(
                                outputFormat,
                                heightmapDataset,
                                "hillshade",
                                nullptr,
                                hillshadeOptions,
                                nullptr);
                            if (hillshadeDataset)
                            {
                                const auto grayscaleDataset = (GDALDataset*) GDALDEMProcessing(
                                    outputFormat,
                                    slopeDataset,
                                    "color-relief",
                                    procParameters.intermediateColorsFilename.c_str(),
                                    slopeOptions,
                                    nullptr);
                                if (grayscaleDataset)
                                {
                                    const auto resultOffset = overlap / 2;
                                    const auto resultSize = tileSize - overlap;
                                    const auto resultLength = resultSize * resultSize;
                                    auto gsBuffer = new char[resultLength];
                                    GDALRasterBand* gsBand = grayscaleDataset->GetRasterBand(1);
                                    if (gsBand->RasterIO(GF_Read, resultOffset, resultOffset, resultSize, resultSize,
                                        gsBuffer, resultSize, resultSize, GDT_Byte, 0, 0, nullptr) == CE_None)
                                    {
                                        auto hsBuffer = new char[resultLength];
                                        GDALRasterBand* band = hillshadeDataset->GetRasterBand(1);
                                        if (band->RasterIO(GF_Read, resultOffset, resultOffset, resultSize, resultSize,
                                            hsBuffer, resultSize, resultSize, GDT_Byte, 0, 0, nullptr) == CE_None)
                                        {
                                            auto mixBuffer = new char[resultLength];
                                            blendHillshade(resultSize, hsBuffer, gsBuffer, mixBuffer);
                                            dataPointer[CPLPrintPointer(dataPointer, mixBuffer, 23)] = 0;
                                            sprintf(filename,"MEM:::DATATYPE=Byte,DATAPOINTER=%s,PIXELS=%d,LINES=%d,"
                                                "GEOTRANSFORM=%.17g/%.17g/0.0/%.17g/0.0/%.17g",
                                                dataPointer, resultSize, resultSize,
                                                tileOrigin.x, tileResolution.x, tileOrigin.y, tileResolution.y);
                                            if (const auto mixDataset = (GDALDataset*) GDALOpen(filename, GA_ReadOnly))
                                            {
                                                if (mixDataset->SetGCPs(gcpCount, pGCPList, "") == CE_None)
                                                {
                                                    const char* colorizeArgs[] = {"-of", outputFormat, "-alpha", NULL};
                                                    if (GDALDEMProcessingOptions* colorizeOptions =
                                                        GDALDEMProcessingOptionsNew(
                                                        const_cast<char **>(colorizeArgs), NULL))
                                                    {
                                                        const auto resultDataset = (GDALDataset*) GDALDEMProcessing(
                                                            outputFormat,
                                                            mixDataset,
                                                            "color-relief",
                                                            procParameters.resultColorsFilename.c_str(),
                                                            colorizeOptions,
                                                            nullptr);
                                                        if (resultDataset)
                                                        {
                                                            result = resultDataset->RasterIO(GF_Read,
                                                                0, 0, resultSize, resultSize,
                                                                pBuffer, resultSize, resultSize, GDT_Byte, 4, nullptr,
                                                                4, resultSize * 4, 1, nullptr) == CE_None;
                                                            GDALClose(resultDataset);                                                        
                                                        }
                                                        GDALDEMProcessingOptionsFree(colorizeOptions);
                                                    }
                                                }
                                                GDALClose(mixDataset);
                                            }
                                            delete[] mixBuffer;
                                        }
                                        delete[] hsBuffer;
                                    }
                                    delete[] gsBuffer;
                                    GDALClose(grayscaleDataset);
                                }
                                GDALClose(hillshadeDataset);
                            }
                            GDALDEMProcessingOptionsFree(hillshadeOptions);
                        }
                    }
                    else // RasterType::Slope
                    {
                        // Produce colorized slope raster
                        const char* colorizeArgs[] = { "-of", outputFormat, "-alpha", NULL };
                        if (GDALDEMProcessingOptions* colorizeOptions = GDALDEMProcessingOptionsNew(
                            const_cast<char **>(colorizeArgs), NULL))
                        {
                            const auto resultDataset = (GDALDataset*) GDALDEMProcessing(
                                outputFormat,
                                slopeDataset,
                                "color-relief",
                                procParameters.resultColorsFilename.c_str(),
                                colorizeOptions,
                                nullptr);
                            if (resultDataset)
                            {
                                const auto resultOffset = overlap / 2;
                                const auto resultSize = tileSize - overlap;
                                result = resultDataset->RasterIO(GF_Read,
                                    resultOffset, resultOffset, resultSize, resultSize,
                                    pBuffer, resultSize, resultSize, GDT_Byte, 4, nullptr,
                                    4, resultSize * 4, 1, nullptr) == CE_None;
                                GDALClose(resultDataset);
                            }
                            GDALDEMProcessingOptionsFree(colorizeOptions);
                        }
                    }
                    GDALClose(slopeDataset);
                }
                GDALDEMProcessingOptionsFree(slopeOptions);
            }
        }
        GDALClose(heightmapDataset);
    }
    delete[] filename;
    return result;
}

bool getGeotiffData(std::string& tilePath, std::string& outColorFilename, std::string& midColorFilename,
    int type, int size, int zoom, int xTileCoord, int yTileCoord, void* pBuffer)
{
    const uint32_t overlap = 4;
    const uint32_t tileSize = size + overlap;
    const uint32_t bandCount = 4;
    const bool toBytes = true;
    const int32_t minZoom = 9;
    const int32_t maxZoom = 12;
    const auto valueCount = tileSize * tileSize;

    ProcessingParameters parameters;
    parameters.rasterType = static_cast<RasterType>(type);
    parameters.intermediateColorsFilename = midColorFilename;
    parameters.resultColorsFilename = outColorFilename;

    ProcessingParameters* procParameters = &parameters;

    // Use suitable downsampling algorithm
    auto resamplingAlgorithm = procParameters && zoom < minZoom ? GRIORA_Cubic : GRIORA_Average;

    // Composite tiles can be made up of data from multiple files
    char* compositeTile = nullptr;
    void* compositeGCPList = nullptr;
    int compositeGCPCount;
    PointD compositeOrigin;
    PointD compositeResolution;
    bool compose = false;
    bool atLeastOnePresent = false;
    if (procParameters)
    {
        if (zoom > maxZoom)
            // Use suitable upsampling algorithm
            resamplingAlgorithm = GRIORA_CubicSpline;
        else if (zoom < minZoom)
        {
            const auto bufSize = valueCount * sizeof(float);
            compositeTile = reinterpret_cast<char*>(malloc(bufSize));
            memset(compositeTile, bufSize, 0);
            compose = true;
        }
    }
    else if (zoom > maxZoom)
        resamplingAlgorithm = GRIORA_Bilinear;
    else if (zoom < minZoom)
        return false;

    // Calculate tile edges to find corresponding data
    const auto zoomDelta = 31 - zoom;
    const auto tileShift = 31 - minZoom;
    const auto overlapOffset = static_cast<double>(1u << zoomDelta) * 0.5 *
        static_cast<double>(overlap) / static_cast<double>(tileSize - overlap);
    PointI upperLeft31(xTileCoord << zoomDelta, yTileCoord << zoomDelta);
    PointI lowerRight31;
    PointI startTile(upperLeft31.x >> tileShift, upperLeft31.y >> tileShift);
    PointI64 lowerRight64(xTileCoord, yTileCoord);
    lowerRight64.x = (lowerRight64.x + 1) << zoomDelta;
    lowerRight64.y = (lowerRight64.y + 1) << zoomDelta;
    PointI endTile;
    endTile.x = (lowerRight64.x - 1) >> tileShift;
    endTile.y = (lowerRight64.y - 1) >> tileShift;
    PointD upperLeftNative(
        static_cast<double>(upperLeft31.x) - overlapOffset,
        static_cast<double>(upperLeft31.y) - overlapOffset);
    PointD lowerRightNative(
        static_cast<double>(lowerRight64.x) + overlapOffset,
        static_cast<double>(lowerRight64.y) + overlapOffset);
    upperLeft31.x = std::floor(upperLeftNative.x);
    upperLeft31.y = std::floor(upperLeftNative.y);
    lowerRight64.x = std::floor(lowerRightNative.x);
    lowerRight64.y = std::floor(lowerRightNative.y);
    if (lowerRight64.x > INT32_MAX)
    {
        upperLeft31.x = upperLeft31.x - INT32_MAX - 1;
        lowerRight64.x = lowerRight64.x - INT32_MAX - 1;
    }
    if (lowerRight64.y > INT32_MAX)
    {
        upperLeft31.y = upperLeft31.y - INT32_MAX - 1;
        lowerRight64.y = lowerRight64.y - INT32_MAX - 1;
    }
    lowerRight31.x = lowerRight64.x - 1;
    lowerRight31.y = lowerRight64.y - 1;

    PointD upperLeftOverscaled;
    PointD lowerRightOverscaled;

    GDALRasterIOExtraArg extraArg;
    extraArg.nVersion = RASTERIO_EXTRA_ARG_CURRENT_VERSION;
    extraArg.eResampleAlg = resamplingAlgorithm;
    extraArg.pfnProgress = CPL_NULLPTR;
    extraArg.pProgressData = CPL_NULLPTR;
    extraArg.bFloatingPointWindowValidity = TRUE;

    const auto destDataType = toBytes ? GDT_Byte : GDT_Float32;
    auto pByteBuffer = static_cast<char*>(pBuffer);
    uint32_t bandSize;

    // Files can have data for this tile
    bool available = false;

    // Some file failed to provide data for this tile
    bool incomplete = false;

    bool result = false;
    double noData = TIFF_NODATA;
    for (int32_t y = startTile.y; y <= endTile.y; y ++)
    {
        for (int32_t x = startTile.x; x <= endTile.x; x ++)
        {
            const std::string filePath =
                tilePath + std::to_string(minZoom) + "/" + std::to_string(x) + "/" + std::to_string(y) + ".tif";
            if (FILE *file = fopen(filePath.c_str(), "r"))
            {
                fclose(file);
                available = true;
                const int zoomShift = zoom - minZoom;
                bool tileFound = true;
                if (tileFound)
                {
                    // Tile is empty
                    bool empty = false;

                    result = false;
                    if (const auto dataset = (GDALDataset*) GDALOpen(filePath.c_str(), GA_ReadOnly))
                    {
                        // Read raster data from source Geotiff file
                        const auto rasterBandCount = dataset->GetRasterCount();
                        GDALRasterBand* band;
                        result = rasterBandCount > 0 && rasterBandCount < 10;
                        double geoTransform[6];
                        if (result && dataset->GetGeoTransform(geoTransform) == CE_None &&
                            geoTransform[2] == 0.0 && geoTransform[4] == 0.0)
                        {
                            const auto rasterSize = PointI(dataset->GetRasterXSize(), dataset->GetRasterYSize());
                            const auto gcpCount = dataset->GetGCPCount();
                            const auto gcpList = dataset->GetGCPs();
                            auto upperLeft = metersFrom31(upperLeftNative);
                            auto lowerRight = metersFrom31(lowerRightNative);
                            const auto tileOrigin = upperLeft;
                            PointD tileResolution;
                            tileResolution.x = (lowerRight.x - upperLeft.x) / tileSize;
                            tileResolution.y = (lowerRight.y - upperLeft.y) / tileSize;
                            upperLeft.x = (upperLeft.x - geoTransform[0]) / geoTransform[1];
                            upperLeft.y = (upperLeft.y - geoTransform[3]) / geoTransform[5];
                            if (upperLeft.x > -1e-9)
                                upperLeft.x = std::fmax(upperLeft.x, 0.0);
                            if (upperLeft.y > -1e-9)
                                upperLeft.y = std::fmax(upperLeft.y, 0.0);
                            lowerRight.x = (lowerRight.x - geoTransform[0]) / geoTransform[1];
                            lowerRight.y = (lowerRight.y - geoTransform[3]) / geoTransform[5];
                            PointI dataOffset(std::floor(upperLeft.x), std::floor(upperLeft.y));
                            PointI dataSize(
                                static_cast<int32_t>(std::ceil(lowerRight.x)) - dataOffset.x,
                                static_cast<int32_t>(std::ceil(lowerRight.y)) - dataOffset.y);
                            PointI tileOffset(0, 0);
                            PointI tileLength(tileSize, tileSize);
                            const bool outRaster = dataOffset.x < 0 || dataOffset.y < 0 ||
                                dataOffset.x + dataSize.x > rasterSize.x ||
                                dataOffset.y + dataSize.y > rasterSize.y;
                            if (outRaster)
                            {
                                const double resSize = tileSize;
                                const PointD resFactor(
                                    (lowerRight.x - upperLeft.x) / resSize,
                                    (lowerRight.y - upperLeft.y) / resSize);
                                tileOffset.x = std::ceil(std::fmax(-upperLeft.x, 0.0) / resFactor.x);
                                tileOffset.y = std::ceil(std::fmax(-upperLeft.y, 0.0) / resFactor.y);
                                if (tileOffset.x > 0)
                                {
                                    tileLength.x -= tileOffset.x;
                                    upperLeft.x += static_cast<double>(tileOffset.x) * resFactor.x;
                                    dataOffset.x = std::floor(upperLeft.x);
                                    dataSize.x = static_cast<int32_t>(std::ceil(lowerRight.x)) - dataOffset.x;
                                }
                                if (tileOffset.y > 0)
                                {
                                    tileLength.y -= tileOffset.y;
                                    upperLeft.y += static_cast<double>(tileOffset.y) * resFactor.y;
                                    dataOffset.y = std::floor(upperLeft.y);
                                    dataSize.y = static_cast<int32_t>(std::ceil(lowerRight.y)) - dataOffset.y;
                                }
                                if (dataOffset.x + dataSize.x > rasterSize.x)
                                {
                                    tileLength.x = rasterSize.x - dataOffset.x;
                                    tileLength.x = std::floor(static_cast<double>(tileLength.x) / resFactor.x);
                                    lowerRight.x = upperLeft.x + static_cast<double>(tileLength.x) * resFactor.x;
                                    dataSize.x = static_cast<int32_t>(std::ceil(lowerRight.x)) - dataOffset.x;
                                    if (dataOffset.x + dataSize.x > rasterSize.x)
                                    {
                                        dataSize.x--;
                                        lowerRight.x = dataOffset.x + dataSize.x;
                                    }
                                }
                                if (dataOffset.y + dataSize.y > rasterSize.y)
                                {
                                    tileLength.y = rasterSize.y - dataOffset.y;
                                    tileLength.y = std::floor(static_cast<double>(tileLength.y) / resFactor.y);
                                    lowerRight.y = upperLeft.y + static_cast<double>(tileLength.y) * resFactor.y;
                                    dataSize.y = static_cast<int32_t>(std::ceil(lowerRight.y)) - dataOffset.y;
                                    if (dataOffset.y + dataSize.y > rasterSize.y)
                                    {
                                        dataSize.y--;
                                        lowerRight.y = dataOffset.y + dataSize.y;
                                    }
                                }
                            }
                            extraArg.dfXOff = upperLeft.x;
                            extraArg.dfYOff = upperLeft.y;
                            extraArg.dfXSize = lowerRight.x - upperLeft.x;
                            extraArg.dfYSize = lowerRight.y - upperLeft.y;
                            auto numOfBands = bandCount;
                            auto dataType = destDataType;
                            if (procParameters)
                            {
                                pByteBuffer = new char[valueCount * sizeof(float)];
                                numOfBands = 1;
                                dataType = GDT_Float32;
                            }
                            const auto pixelSizeInBytes = dataType == GDT_Byte ? 1 : (dataType == GDT_Int16 ? 2 : 4);
                            bandSize = valueCount * pixelSizeInBytes;
                            auto pData = pByteBuffer;
                            int pShift = outRaster ? (tileOffset.y * tileSize + tileOffset.x) * pixelSizeInBytes : 0;
                            const auto side = tileSize * pixelSizeInBytes;
                            for (int bandIndex = 1; bandIndex <= numOfBands; bandIndex++)
                            {
                                result = result && (band = dataset->GetRasterBand(bandIndex));
                                result = result && band->GetRasterDataType() != GDT_Unknown;
                                result = result && !band->GetColorTable();
                                if (result && compose && outRaster)
                                {
                                    auto pValues = reinterpret_cast<float*>(pData);
                                    std::fill(pValues, pValues + valueCount, static_cast<float>(noData));
                                }
                                result = result && band->RasterIO(GF_Read,
                                    dataOffset.x, dataOffset.y,
                                    dataSize.x, dataSize.y,
                                    pData + pShift, tileLength.x, tileLength.y,
                                    dataType, 0, side, &extraArg) == CE_None;
                                if (!result) break;
                                pData += bandSize;
                            }
                            // Hillshade/slope/height raster processing
                            if (procParameters)
                            {
                                if (compose)
                                {
                                    if (result)
                                    {
                                        // Compensate height blur effect for much downsampled tiles
                                        const float scaleFactor =
                                            pow(1.5f, static_cast<float>(std::min(minZoom - zoom, 3))) - 0.2f;
                                        
                                        // Combine heightmap data
                                        mergeHeights(tileSize, scaleFactor, !atLeastOnePresent,
                                            compositeTile, reinterpret_cast<float*>(pByteBuffer));

                                        if (!atLeastOnePresent)
                                        {
                                            compositeOrigin = tileOrigin;
                                            compositeResolution = tileResolution;
                                            compositeGCPCount = gcpCount;
                                            const auto listSize = sizeof(GDAL_GCP) * gcpCount;
                                            compositeGCPList = malloc(listSize);
                                            memcpy(compositeGCPList, reinterpret_cast<const char*>(gcpList), listSize);
                                            atLeastOnePresent = true;
                                        }
                                    }
                                    else
                                        incomplete = true;
                                }
                                else
                                {
                                    // Produce hillshade/slope/height raster from heightmap data
                                    result = result && destDataType == GDT_Byte && bandCount == 4 &&
                                        postProcess(pByteBuffer, *procParameters, tileSize, overlap, tileOrigin,
                                            tileResolution, gcpCount, gcpList, pBuffer);                                
                                }
                                delete[] pByteBuffer;
                                pByteBuffer = static_cast<char*>(pBuffer);
                                bandSize = (tileSize - overlap) * (tileSize - overlap) *
                                    (destDataType == GDT_Byte ? 1 : (destDataType == GDT_Int16 ? 2 : 4));
                            }
                        }
                        GDALClose(dataset);
                    }
                }
            }
        }
    }

    // Build result composite tile and put it in cache
    if (compose && available && destDataType == GDT_Byte && bandCount == 4)
    {
        result = true;
        if (atLeastOnePresent)
        {
            // Produce hillshade/slope/height raster from heightmap data
            result = postProcess(compositeTile, *procParameters, tileSize, overlap, compositeOrigin,
                compositeResolution, compositeGCPCount, compositeGCPList, pBuffer);
        }
        else if (incomplete)
        {
            // No data due to failed data request
            result = false;
        }
        else
        {
            // Produce empty raster if no data was found
            memset(pBuffer, bandCount * bandSize, 0);
        }
    }

    if (compositeTile)
        free(compositeTile);

    if (compositeGCPList)
        free(compositeGCPList);

    return result;
}
