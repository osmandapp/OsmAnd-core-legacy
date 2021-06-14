LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)

PROJECT_ROOT_RELATIVE := ../../../../platforms/android/OsmAnd
OSMAND_LIBPNG_RELATIVE := ../../../externals/skia/upstream.patched/third_party/libpng
OSMAND_LIBPNG := $(LOCAL_PATH)/$(OSMAND_LIBPNG_RELATIVE)
	
LOCAL_CFLAGS += -DPNG_CONFIGURE_LIBPNG -fPIC

LOCAL_C_INCLUDES := \
	$(OSMAND_LIBPNG) \
	$(OSMAND_LIBPNG)/../externals/libpng

LOCAL_SRC_FILES := \
	$(OSMAND_LIBPNG_RELATIVE)/../externals/libpng/png.c \
	$(OSMAND_LIBPNG_RELATIVE)/../externals/libpng/pngerror.c \
	$(OSMAND_LIBPNG_RELATIVE)/../externals/libpng/pngget.c \
	$(OSMAND_LIBPNG_RELATIVE)/../externals/libpng/pngmem.c \
	$(OSMAND_LIBPNG_RELATIVE)/../externals/libpng/pngpread.c \
	$(OSMAND_LIBPNG_RELATIVE)/../externals/libpng/pngread.c \
	$(OSMAND_LIBPNG_RELATIVE)/../externals/libpng/pngrio.c \
	$(OSMAND_LIBPNG_RELATIVE)/../externals/libpng/pngrtran.c \
	$(OSMAND_LIBPNG_RELATIVE)/../externals/libpng/pngrutil.c \
	$(OSMAND_LIBPNG_RELATIVE)/../externals/libpng/pngset.c \
	$(OSMAND_LIBPNG_RELATIVE)/../externals/libpng/pngtrans.c \
	$(OSMAND_LIBPNG_RELATIVE)/../externals/libpng/pngwio.c \
	$(OSMAND_LIBPNG_RELATIVE)/../externals/libpng/pngwrite.c \
	$(OSMAND_LIBPNG_RELATIVE)/../externals/libpng/pngwtran.c \
	$(OSMAND_LIBPNG_RELATIVE)/../externals/libpng/pngwutil.c


ifeq ($(TARGET_ARCH),$(filter $(TARGET_ARCH),arm arm64))
	LOCAL_SRC_FILES += \
		$(OSMAND_LIBPNG_RELATIVE)/../externals/libpng/arm/arm_init.c \
		$(OSMAND_LIBPNG_RELATIVE)/../externals/libpng/arm/filter_neon_intrinsics.c \
		$(OSMAND_LIBPNG_RELATIVE)/../externals/libpng/arm/palette_neon_intrinsics.c

else ifeq ($(TARGET_ARCH),$(filter $(TARGET_ARCH),x86 x86_64))
	LOCAL_SRC_FILES += \
		$(OSMAND_LIBPNG_RELATIVE)/../externals/libpng/intel/filter_sse2_intrinsics.c \
		$(OSMAND_LIBPNG_RELATIVE)/../externals/libpng/intel/intel_init.c
endif		

LOCAL_MODULE := osmand_png

ifneq ($(OSMAND_USE_PREBUILT),true)
	include $(BUILD_STATIC_LIBRARY)
else
	LOCAL_SRC_FILES := \
		$(PROJECT_ROOT_RELATIVE)/libs/$(TARGET_ARCH_ABI)/lib$(LOCAL_MODULE).a
	include $(PREBUILT_STATIC_LIBRARY)
endif