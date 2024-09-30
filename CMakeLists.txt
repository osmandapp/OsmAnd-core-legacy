cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# OsmAnd Core Root
set(OSMAND_CORE_ROOT "${CMAKE_CURRENT_LIST_DIR}")

# Output root
set(OSMAND_CORE_OR "${OSMAND_CORE_ROOT}/binaries/${CMAKE_TARGET_OS}/${CMAKE_TARGET_CPU_ARCH}/${CMAKE_TARGET_CPU_NAME}/${CMAKE_BUILD_TYPE}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${OSMAND_CORE_OR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${OSMAND_CORE_OR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${OSMAND_CORE_OR}")

# Include CPU specific settings
if(DEFINED CMAKE_TARGET_CPU_NAME)
	include("./build-tools/cpu-specific/${CMAKE_TARGET_CPU_NAME}.cmake")
endif()

# Externals
add_subdirectory("${OSMAND_CORE_ROOT}/externals/zlib" "./externals/zlib") # zlibstatic
#add_subdirectory("${OSMAND_CORE_ROOT}/externals/giflib" "./externals/giflib")
add_subdirectory("${OSMAND_CORE_ROOT}/externals/skia/upstream.patched/third_party/externals/libjpeg-turbo" "./externals/jpeg")
add_subdirectory("${OSMAND_CORE_ROOT}/externals/skia/upstream.patched/third_party/libpng" "./externals/libpng")
add_subdirectory("${OSMAND_CORE_ROOT}/externals/skia/upstream.patched/third_party/externals/freetype" "./externals/freetype")
add_subdirectory("${OSMAND_CORE_ROOT}/externals/skia/upstream.patched/third_party/externals/expat" "./externals/expat")
add_subdirectory("${OSMAND_CORE_ROOT}/externals/libskia" "./externals/libskia")
add_subdirectory("${OSMAND_CORE_ROOT}/externals/protobuf" "./externals/protobuf")
add_subdirectory("${OSMAND_CORE_ROOT}/externals/sqlite" "./externals/sqlite")
add_subdirectory("${OSMAND_CORE_ROOT}/externals/proj" "./externals/proj")
add_subdirectory("${OSMAND_CORE_ROOT}/externals/gdal" "./externals/gdal")

#add_subdirectory("${OSMAND_CORE_ROOT}/externals/zlib" "./externals/zlib") # zlibstatic
#add_subdirectory("${OSMAND_CORE_ROOT}/externals/giflib" "./externals/giflib")
#add_subdirectory("${OSMAND_CORE_ROOT}/externals/jpeg" "./externals/jpeg")
#add_subdirectory("${OSMAND_CORE_ROOT}/externals/libpng" "./externals/libpng")
#add_subdirectory("${OSMAND_CORE_ROOT}/externals/freetype" "./externals/freetype")
#add_subdirectory("${OSMAND_CORE_ROOT}/externals/expat" "./externals/expat")
#add_subdirectory("${OSMAND_CORE_ROOT}/externals/libskia" "./externals/libskia")
#add_subdirectory("${OSMAND_CORE_ROOT}/externals/protobuf" "./externals/protobuf")


# OsmAnd Core
add_subdirectory("${OSMAND_CORE_ROOT}/osmand" "./osmand")
add_dependencies(osmand
	zlibstatic
)
