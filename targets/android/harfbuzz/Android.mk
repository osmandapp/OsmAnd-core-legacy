LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_ARM_MODE := arm
LOCAL_MODULE_TAGS := optional

PROJECT_ROOT_RELATIVE := ../../../../platforms/android/OsmAnd
OSMAND_HARFBUZZ_RELATIVE := ../../../externals/harfbuzz
OSMAND_HARFBUZZ := $(LOCAL_PATH)/$(OSMAND_HARFBUZZ_RELATIVE)

LOCAL_C_INCLUDES += $(OSMAND_HARFBUZZ)/src

LOCAL_CPP_EXTENSION := .cc
LOCAL_SRC_FILES := \
	$(OSMAND_HARFBUZZ_RELATIVE)/src/harfbuzz.cc

LOCAL_CFLAGS += \
	-DHAVE_PTHREAD	

LOCAL_CPPFLAGS := \
	-fno-rtti \
	-fno-exceptions \
	-fno-threadsafe-statics

LOCAL_MODULE := osmand_harfbuzz

ifneq ($(OSMAND_USE_PREBUILT),true)
	include $(BUILD_STATIC_LIBRARY)
else
	LOCAL_SRC_FILES := \
		$(PROJECT_ROOT_RELATIVE)/libs/$(TARGET_ARCH_ABI)/lib$(LOCAL_MODULE).a
	include $(PREBUILT_STATIC_LIBRARY)
endif