LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

LOCAL_SRC_FILES := \
	rk_emmcutils.c

LOCAL_MODULE := librk_emmcutils
LOCAL_STATIC_LIBRARIES = libcutils

ifeq ($(strip $(TARGET_BOARD_PLATFORM)), rk3399)
	LOCAL_CFLAGS += -DTARGET_RK3399
endif

include $(BUILD_STATIC_LIBRARY)
