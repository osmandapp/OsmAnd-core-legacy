LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)

PROJECT_ROOT_RELATIVE := ../../../../platforms/android/OsmAnd
OSMAND_LUA_RELATIVE := ../../../externals/skia/upstream.patched/third_party/externals/lua
OSMAND_LUA := $(LOCAL_PATH)/$(OSMAND_LUA_RELATIVE)

LOCAL_C_INCLUDES += $(OSMAND_LUA)

LOCAL_SRC_FILES += \
	$(OSMAND_LUA_RELATIVE)/lapi.c \
	$(OSMAND_LUA_RELATIVE)/lauxlib.c \
	$(OSMAND_LUA_RELATIVE)/lbaselib.c \
	$(OSMAND_LUA_RELATIVE)/lbitlib.c \
	$(OSMAND_LUA_RELATIVE)/lcode.c \
	$(OSMAND_LUA_RELATIVE)/lcorolib.c \
	$(OSMAND_LUA_RELATIVE)/lctype.c \
	$(OSMAND_LUA_RELATIVE)/ldblib.c \
	$(OSMAND_LUA_RELATIVE)/ldebug.c \
	$(OSMAND_LUA_RELATIVE)/ldo.c \
	$(OSMAND_LUA_RELATIVE)/ldump.c \
	$(OSMAND_LUA_RELATIVE)/lfunc.c \
	$(OSMAND_LUA_RELATIVE)/lgc.c \
	$(OSMAND_LUA_RELATIVE)/linit.c \
	$(OSMAND_LUA_RELATIVE)/liolib.c \
	$(OSMAND_LUA_RELATIVE)/llex.c \
	$(OSMAND_LUA_RELATIVE)/lmathlib.c \
	$(OSMAND_LUA_RELATIVE)/lmem.c \
	$(OSMAND_LUA_RELATIVE)/loadlib.c \
	$(OSMAND_LUA_RELATIVE)/lobject.c \
	$(OSMAND_LUA_RELATIVE)/lopcodes.c \
	$(OSMAND_LUA_RELATIVE)/loslib.c \
	$(OSMAND_LUA_RELATIVE)/lparser.c \
	$(OSMAND_LUA_RELATIVE)/lstate.c \
	$(OSMAND_LUA_RELATIVE)/lstring.c \
	$(OSMAND_LUA_RELATIVE)/lstrlib.c \
	$(OSMAND_LUA_RELATIVE)/ltable.c \
	$(OSMAND_LUA_RELATIVE)/ltablib.c \
	$(OSMAND_LUA_RELATIVE)/ltm.c \
	$(OSMAND_LUA_RELATIVE)/lundump.c \
	$(OSMAND_LUA_RELATIVE)/lutf8lib.c \
	$(OSMAND_LUA_RELATIVE)/lvm.c \
	$(OSMAND_LUA_RELATIVE)/lzio.c

LOCAL_MODULE := osmand_lua

ifneq ($(OSMAND_USE_PREBUILT),true)
	include $(BUILD_STATIC_LIBRARY)
else
	LOCAL_SRC_FILES := \
		$(PROJECT_ROOT_RELATIVE)/libs/$(TARGET_ARCH_ABI)/lib$(LOCAL_MODULE).a
	include $(PREBUILT_STATIC_LIBRARY)
endif