LOCAL_PATH := $(call my-dir)

# example of local shared library
#include $(CLEAR_VARS)
# LOCAL_MODULE := libavian_ms
# LOCAL_SRC_FILES := /home/victor/projects/OsmAnd/avian-core/avian-arm/libavian_ms.so
# include $(PREBUILT_SHARED_LIBRARY)
include $(CLEAR_VARS)

OSMAND_PROTOBUF := $(LOCAL_PATH)/../../../externals/protobuf/upstream.patched
OSMAND_SKIA_ROOT := $(LOCAL_PATH)/../../../externals/skia
OSMAND_SKIA := $(LOCAL_PATH)/../../../externals/skia/upstream.patched
OSMAND_EXPAT := $(LOCAL_PATH)/../../../externals/skia/upstream.patched/third_party/externals/expat
OSMAND_CORE_RELATIVE := ../../../native
OSMAND_CORE := $(LOCAL_PATH)/$(OSMAND_CORE_RELATIVE)
OSMAND_HARFBUZZ_ROOT := $(LOCAL_PATH)/../../../externals/skia/upstream.patched/third_party/harfbuzz
OSMAND_HARFBUZZ := $(LOCAL_PATH)/../../../externals/skia/upstream.patched/third_party/externals/harfbuzz

LOCAL_C_INCLUDES := $(LOCAL_PATH)/src \
	$(OSMAND_PROTOBUF)/src \
	$(OSMAND_SKIA_ROOT) \
	$(OSMAND_SKIA) \
	$(OSMAND_EXPAT)/lib \
	$(OSMAND_SKIA)/include/core \
	$(OSMAND_SKIA)/include/codec \
	$(OSMAND_SKIA)/include/config \
	$(OSMAND_SKIA)/include/effects \
	$(OSMAND_SKIA)/include/images \
	$(OSMAND_SKIA)/include/ports \
	$(OSMAND_SKIA)/include/utils \
	$(OSMAND_SKIA)/include/shaders \
	$(OSMAND_SKIA)/include/utils/android \
	$(OSMAND_SKIA)/src/core \
	${OSMAND_SKIA}/src/shaders \
	$(OSMAND_CORE)/include \
	$(OSMAND_CORE)/src \
	$(OSMAND_HARFBUZZ)/src \
	$(OSMAND_HARFBUZZ_ROOT)

LOCAL_CPP_EXTENSION := .cc .cpp
LOCAL_SRC_FILES := \
	src/Logging.cpp \
	$(OSMAND_CORE_RELATIVE)/src/ElapsedTimer.cpp \
	$(OSMAND_CORE_RELATIVE)/src/common.cpp \
	$(OSMAND_CORE_RELATIVE)/src/commonRendering.cpp \
	$(OSMAND_CORE_RELATIVE)/src/multipolygons.cpp \
	$(OSMAND_CORE_RELATIVE)/src/renderRules.cpp \
	$(OSMAND_CORE_RELATIVE)/src/rendering.cpp \
	$(OSMAND_CORE_RELATIVE)/src/openingHoursParser.cpp \
	$(OSMAND_CORE_RELATIVE)/src/routeTypeRule.cpp \
	$(OSMAND_CORE_RELATIVE)/src/precalculatedRouteDirection.cpp \
	$(OSMAND_CORE_RELATIVE)/src/routingConfiguration.cpp \
	$(OSMAND_CORE_RELATIVE)/src/routingContext.cpp \
	$(OSMAND_CORE_RELATIVE)/src/binaryRead.cpp \
	$(OSMAND_CORE_RELATIVE)/src/generalRouter.cpp \
	$(OSMAND_CORE_RELATIVE)/src/binaryRoutePlanner.cpp \
	$(OSMAND_CORE_RELATIVE)/src/transportRouteResultSegment.cpp \
	$(OSMAND_CORE_RELATIVE)/src/transportRoutingObjects.cpp \
	$(OSMAND_CORE_RELATIVE)/src/transportRouteSegment.cpp \
	$(OSMAND_CORE_RELATIVE)/src/transportRouteResult.cpp \
	$(OSMAND_CORE_RELATIVE)/src/transportRoutingConfiguration.cpp \
	$(OSMAND_CORE_RELATIVE)/src/transportRoutingContext.cpp \
	$(OSMAND_CORE_RELATIVE)/src/transportRoutePlanner.cpp \
	$(OSMAND_CORE_RELATIVE)/src/transportRouteStopsReader.cpp \
	$(OSMAND_CORE_RELATIVE)/src/routeDataBundle.cpp \
	$(OSMAND_CORE_RELATIVE)/src/routeDataResources.cpp \
	$(OSMAND_CORE_RELATIVE)/src/routePlannerFrontEnd.cpp \
	$(OSMAND_CORE_RELATIVE)/src/routeResultPreparation.cpp \
	$(OSMAND_CORE_RELATIVE)/src/routeSegmentResult.cpp \
	$(OSMAND_CORE_RELATIVE)/src/turnType.cpp \
	$(OSMAND_CORE_RELATIVE)/src/proto/osmand_index.pb.cc \
	$(OSMAND_CORE_RELATIVE)/src/java_wrap.cpp \
	$(OSMAND_CORE_RELATIVE)/src/routeCalculationProgress.cpp \
	$(OSMAND_CORE_RELATIVE)/src/hhRouteDataStructure.cpp \
	$(OSMAND_CORE_RELATIVE)/src/hhRoutePlanner.cpp \
	$(OSMAND_CORE_RELATIVE)/src/NetworkDBPointRouteInfo.cpp \
	$(OSMAND_CORE_RELATIVE)/src/gpxSegmentsApproximation.cpp
	
ifdef OSMAND_PROFILE_NATIVE_OPERATIONS
	LOCAL_CFLAGS += \
		-DOSMAND_NATIVE_PROFILING
endif

LOCAL_MODULE := osmand

LOCAL_CFLAGS := \
	-DGOOGLE_PROTOBUF_NO_RTTI \
	-DSK_BUILD_FOR_ANDROID \
	-DSK_BUILD_FOR_ANDROID_NDK \
	-DSK_ALLOW_STATIC_GLOBAL_INITIALIZERS=0 \
	-DSK_RELEASE \
	-DSK_CPU_LENDIAN \
	-DGR_RELEASE=1 \
	-DANDROID_BUILD \
	-fPIC
	
LOCAL_STATIC_LIBRARIES := \
	osmand_protobuf \
	osmand_jpeg \
	osmand_ft2 \
	osmand_png \
	osmand_expat

LOCAL_WHOLE_STATIC_LIBRARIES := \
	osmand_skia \
	osmand_harfbuzz


LOCAL_LDLIBS := -lz -llog -ldl -ljnigraphics
# example of local shared library
# LIB_PATH := /home/victor/projects/OsmAnd/avian-core/avian-arm/libavian_ms.so
# LOCAL_SHARE_LIBRARIES := libavian_ms
# LOCAL_LDLIBS := -L/home/victor/projects/OsmAnd/avian-core/avian-arm/ -lz -llog -ldl -lavian_ms


include $(BUILD_SHARED_LIBRARY)
