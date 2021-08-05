LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)

PROJECT_ROOT_RELATIVE := ../../../../platforms/android/OsmAnd
OSMAND_LIBPNG_ROOT_RELATIVE := ../../../externals/skia/upstream.patched/third_party/libpng
OSMAND_LIBPNG_ROOT := $(LOCAL_PATH)/$(OSMAND_LIBPNG_ROOT_RELATIVE)
OSMAND_LIBPNG_RELATIVE := ../../../externals/skia/upstream.patched/third_party/libpng
OSMAND_LIBPNG := $(LOCAL_PATH)/$(OSMAND_LIBPNG_RELATIVE)
	
LOCAL_CFLAGS += -DPNG_CONFIGURE_LIBPNG -fPIC

LOCAL_SRC_FILES := \
	$(OSMAND_LIBPNG_RELATIVE)/png.c \
	$(OSMAND_LIBPNG_RELATIVE)/pngerror.c \
	$(OSMAND_LIBPNG_RELATIVE)/pngget.c \
	$(OSMAND_LIBPNG_RELATIVE)/pngmem.c \
	$(OSMAND_LIBPNG_RELATIVE)/pngpread.c \
	$(OSMAND_LIBPNG_RELATIVE)/pngread.c \
	$(OSMAND_LIBPNG_RELATIVE)/pngrio.c \
	$(OSMAND_LIBPNG_RELATIVE)/pngrtran.c \
	$(OSMAND_LIBPNG_RELATIVE)/pngrutil.c \
	$(OSMAND_LIBPNG_RELATIVE)/pngset.c \
	$(OSMAND_LIBPNG_RELATIVE)/pngtrans.c \
	$(OSMAND_LIBPNG_RELATIVE)/pngwio.c \
	$(OSMAND_LIBPNG_RELATIVE)/pngwrite.c \
	$(OSMAND_LIBPNG_RELATIVE)/pngwtran.c \
	$(OSMAND_LIBPNG_RELATIVE)/pngwutil.c \
	$(OSMAND_LIBPNG_RELATIVE)/arm/arm_init.c \
    $(OSMAND_LIBPNG_RELATIVE)/arm/filter_neon_intrinsics.c

LOCAL_MODULE := osmand_png

ifneq ($(OSMAND_USE_PREBUILT),true)
	include $(BUILD_STATIC_LIBRARY)
else
	LOCAL_SRC_FILES := \
		$(PROJECT_ROOT_RELATIVE)/libs/$(TARGET_ARCH_ABI)/lib$(LOCAL_MODULE).a
	include $(PREBUILT_STATIC_LIBRARY)
endif