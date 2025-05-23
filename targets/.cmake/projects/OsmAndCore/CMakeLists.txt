project(osmand)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
include("../../common.cmake")

set(ROOT "${OSMAND_ROOT}/native")
include_directories(AFTER SYSTEM
	"${OSMAND_ROOT}/externals/skia/upstream.patched"
	"${OSMAND_ROOT}/externals/skia/upstream.patched/include/core"
	"${OSMAND_ROOT}/externals/skia/upstream.patched/include/codec"
	"${OSMAND_ROOT}/externals/skia/upstream.patched/include/images"
	"${OSMAND_ROOT}/externals/skia/upstream.patched/include/utils"
	"${OSMAND_ROOT}/externals/skia/upstream.patched/include/config"
	"${OSMAND_ROOT}/externals/skia/upstream.patched/include/effects"
	"${OSMAND_ROOT}/externals/skia/upstream.patched/include/ports"
	"${OSMAND_ROOT}/externals/skia/upstream.patched/tools"
	"${OSMAND_ROOT}/externals/skia/upstream.patched/include/utils/android"
	"${OSMAND_ROOT}/externals/skia/upstream.patched/src/core"
	"${OSMAND_ROOT}/externals/skia/upstream.patched/third_party/externals/expat/lib"
	"${OSMAND_ROOT}/externals/skia/upstream.patched/third_party/externals/freetype/include"
	"${OSMAND_ROOT}/externals/skia/upstream.patched/src/shaders"
	"${OSMAND_ROOT}/externals/skia/upstream.patched/third_party/externals/icu/source/common"
	#"${OSMAND_ROOT}/externals/giflib/upstream.patched/lib"
	"${OSMAND_ROOT}/externals/skia/upstream.patched/third_party/externals/libjpeg-turbo"
	"${OSMAND_ROOT}/externals/skia/upstream.patched/third_party/libpng"
	#"${OSMAND_ROOT}/externals/harfbuzz/upstream.patched/src"
	"${OSMAND_ROOT}/externals/protobuf/upstream.patched/src"
	"${OSMAND_ROOT}/native/include"
	"${OSMAND_ROOT}/native/src"
	"$ENV{JAVA_HOME}/include"
)
if(CMAKE_TARGET_OS STREQUAL "windows")
	include_directories(AFTER SYSTEM
		"${OSMAND_ROOT}/externals/gdal"
		"$ENV{JAVA_HOME}/include/win32"
	)
	add_definitions(-DSK_BUILD_FOR_WIN32)
elseif((CMAKE_TARGET_OS STREQUAL "darwin") OR (CMAKE_TARGET_OS STREQUAL "macosx"))
	include_directories(AFTER SYSTEM
		"${OSMAND_ROOT}/externals/gdal"
		"$ENV{JAVA_HOME}/include/darwin"
	)
	add_definitions(-DSK_BUILD_FOR_MAC)
elseif(CMAKE_TARGET_OS STREQUAL "linux")
	include_directories(AFTER SYSTEM
		"${OSMAND_ROOT}/externals/gdal"
		"$ENV{JAVA_HOME}/include/linux"
	)
	add_definitions(-DSK_BUILD_FOR_UNIX)
endif()
add_definitions(
	-DGOOGLE_PROTOBUF_NO_RTTI
	-DSK_ALLOW_STATIC_GLOBAL_INITIALIZERS=0
	-DSK_RELEASE
	-DSK_CPU_LENDIAN
	-DGR_RELEASE=1
	-DXML_STATIC
)
if(CMAKE_COMPILER_FAMILY STREQUAL "gcc" OR CMAKE_COMPILER_FAMILY STREQUAL "clang")
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
endif()
set(pd_sources "")
if(CMAKE_TARGET_OS STREQUAL "linux")
    set(pd_sources
		"${OSMAND_ROOT}/targets/linux/OsmAndCore/src/Logging.cpp"
		"${ROOT}/src/heightmapRenderer.cpp"
	)
elseif((CMAKE_TARGET_OS STREQUAL "darwin") OR (CMAKE_TARGET_OS STREQUAL "macosx"))
    set(pd_sources
		"${OSMAND_ROOT}/targets/darwin/OsmAndCore/src/Logging.cpp"
		"${ROOT}/src/heightmapRenderer.cpp"
	)
elseif(CMAKE_TARGET_OS STREQUAL "windows")
    set(pd_sources
		"${OSMAND_ROOT}/targets/windows/OsmAndCore/src/Logging.cpp"
		"${ROOT}/src/heightmapRenderer.cpp"
		"${OSMAND_ROOT}/targets/windows_desktop/OsmAndCore/src/DllMain.cpp"
	)
endif()
add_library(osmand SHARED
	"${ROOT}/src/ElapsedTimer.cpp"
	"${ROOT}/src/common.cpp"
	"${ROOT}/src/commonRendering.cpp"
	"${ROOT}/src/multipolygons.cpp"
	"${ROOT}/src/renderRules.cpp"
	"${ROOT}/src/rendering.cpp"
	"${ROOT}/src/openingHoursParser.cpp"
	"${ROOT}/src/routeTypeRule.cpp"
	"${ROOT}/src/binaryRead.cpp"
	"${ROOT}/src/precalculatedRouteDirection.cpp"
	"${ROOT}/src/generalRouter.cpp"
	"${ROOT}/src/binaryRoutePlanner.cpp"
	"${ROOT}/src/transportRouteResultSegment.cpp"
	"${ROOT}/src/transportRoutingObjects.cpp"
	"${ROOT}/src/transportRouteSegment.cpp"
	"${ROOT}/src/transportRouteResult.cpp"
	"${ROOT}/src/transportRoutingConfiguration.cpp"
	"${ROOT}/src/transportRoutingContext.cpp"
	"${ROOT}/src/transportRoutePlanner.cpp"
	"${ROOT}/src/transportRouteStopsReader.cpp"
	"${ROOT}/src/routeDataBundle.cpp"
	"${ROOT}/src/routeDataResources.cpp"
	"${ROOT}/src/routePlannerFrontEnd.cpp"
	"${ROOT}/src/routeResultPreparation.cpp"
	"${ROOT}/src/routeSegmentResult.cpp"
	"${ROOT}/src/turnType.cpp"
	"${ROOT}/src/proto/osmand_index.pb.cc"
	"${ROOT}/src/java_wrap.cpp"
	"${ROOT}/src/routeCalculationProgress.cpp"
	"${ROOT}/src/hhRouteDataStructure.cpp"
	"${ROOT}/src/hhRoutePlanner.cpp"
	"${ROOT}/src/NetworkDBPointRouteInfo.cpp"
	"${ROOT}/src/gpxRouteApproximation.cpp"
	"${ROOT}/src/gpxMultiSegmentsApproximation.cpp"
	"${ROOT}/src/gpxSimplePointsMatchApproximation.cpp"
	"${ROOT}/src/roundaboutTurn.cpp"
	${pd_sources}
)

target_link_libraries(osmand LINK_PUBLIC
	skia_osmand
	protobuf_osmand
	harfbuzz
)

if (NOT CMAKE_TARGET_OS STREQUAL "android")
	target_link_libraries(osmand LINK_PUBLIC
		gdal_osmand
	)
endif()
