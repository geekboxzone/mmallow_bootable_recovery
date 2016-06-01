# Copyright (C) 2007 The Android Open Source Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

LOCAL_PATH := $(call my-dir)
prebuilt_stdcxx_PATH := prebuilts/ndk/current/sources/cxx-stl/

include $(CLEAR_VARS)

LOCAL_SRC_FILES := fuse_sideload.c

LOCAL_CFLAGS := -O2 -g -DADB_HOST=0 -Wall -Wno-unused-parameter
LOCAL_CFLAGS += -D_XOPEN_SOURCE -D_GNU_SOURCE

LOCAL_MODULE := libfusesideload

LOCAL_STATIC_LIBRARIES := libcutils libc libmincrypt
include $(BUILD_STATIC_LIBRARY)

include $(CLEAR_VARS)

LOCAL_SRC_FILES := \
    adb_install.cpp \
    asn1_decoder.cpp \
    bootloader.cpp \
    device.cpp \
    fuse_sdcard_provider.c \
    install.cpp \
    recovery.cpp \
    roots.cpp \
    screen_ui.cpp \
    ui.cpp \
    verifier.cpp \
    rkimage.cpp	    \

LOCAL_MODULE := recovery

LOCAL_FORCE_STATIC_EXECUTABLE := true

ifeq ($(HOST_OS),linux)
LOCAL_REQUIRED_MODULES := mkfs.f2fs
endif

RECOVERY_API_VERSION := 3
RECOVERY_FSTAB_VERSION := 2
LOCAL_CFLAGS += -DRECOVERY_API_VERSION=$(RECOVERY_API_VERSION)
LOCAL_CFLAGS += -D_FILE_OFFSET_BITS=64

#redirect to SDCARD、CACHE、UART
#SDCARD: save log to sdcard
#CACHE: save log to /cache/recovery/ dir
#UART: redirect log to uart output
REDIRECT_LOG_TO := UART

LOCAL_C_INCLUDES := \
	$(prebuilt_stdcxx_PATH)/gnu-libstdc++/include\
	$(prebuilt_stdcxx_PATH)/gnu-libstdc++/libs/armeabi-v7a/include\
	bionic \
	bionic/libstdc++/include \
	$(LOCAL_PATH)/rkupdate
LOCAL_CPPFLAGS += -fexceptions -frtti
LOCAL_CFLAGS += -Wno-unused-parameter

LOCAL_C_INCLUDES += \
    system/vold \
    system/extras/ext4_utils \
    system/core/adb \

LOCAL_STATIC_LIBRARIES := \
    libext4_utils_static \
    libsparse_static \
    libminzip \
    libz \
    libmtdutils \
    libmincrypt \
    libminadbd \
    libfusesideload \
    libminui \
    libpng \
    libfs_mgr \
    libbase \
    libcutils \
    liblog \
    libselinux \
    librkupdate\
    libext2_uuid\
    librkrsa\
    libgnustl_static\
    libstdc++ \
    libutils \
    libm \
    libc \
    libedify \
    libapplypatch \
    librsa \
    libcrc32 \
    librk_emmcutils  

ifeq ($(TARGET_USERIMAGES_USE_EXT4), true)
    LOCAL_CFLAGS += -DUSE_EXT4
    LOCAL_C_INCLUDES += system/extras/ext4_utils
    LOCAL_STATIC_LIBRARIES += libext4_utils_static libz
endif

LOCAL_MODULE_PATH := $(TARGET_RECOVERY_ROOT_OUT)/sbin

ifeq ($(TARGET_RECOVERY_UI_LIB),)
  LOCAL_SRC_FILES += default_device.cpp
else
  LOCAL_STATIC_LIBRARIES += $(TARGET_RECOVERY_UI_LIB)
endif
ifeq ($(strip $(TARGET_BOARD_HARDWARE)),rk30board)
LOCAL_CFLAGS += -DTARGET_RK30
endif
ifeq ($(strip $(TARGET_BOARD_HARDWARE)),rk2928board)
LOCAL_CFLAGS += -DTARGET_RK30
endif

ifeq ($(strip $(TARGET_BOARD_PLATFORM)),rk3368)
LOCAL_CFLAGS += -DTARGET_RK33xx
endif

ifeq ($(strip $(TARGET_BOARD_PLATFORM)),rk3399)
LOCAL_CFLAGS += -DTARGET_RK33xx
endif

ifeq ($(strip $(TARGET_BOARD_PLATFORM)),rk3188)
LOCAL_CFLAGS += -DTARGET_RK3188
endif

ifeq ($(strip $(REDIRECT_LOG_TO)),SDCARD)
$(warning *** Redirect log to SDCARD)
LOCAL_CFLAGS += -DLogToSDCard
endif

ifeq ($(strip $(REDIRECT_LOG_TO)),UART)
$(warning *** Redirect log to UART)
LOCAL_CFLAGS += -DLogToSerial
endif

ifeq ($(strip $(REDIRECT_LOG_TO)),CACHE)
$(warning *** Redirect log to CACHE)
LOCAL_CFLAGS += -DLogToCache
endif

include $(BUILD_EXECUTABLE)

# All the APIs for testing
include $(CLEAR_VARS)
LOCAL_MODULE := libverifier
LOCAL_MODULE_TAGS := tests
LOCAL_SRC_FILES := \
    asn1_decoder.cpp
include $(BUILD_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := verifier_test
LOCAL_FORCE_STATIC_EXECUTABLE := true
LOCAL_MODULE_TAGS := tests
LOCAL_CFLAGS += -Wno-unused-parameter
LOCAL_SRC_FILES := \
    verifier_test.cpp \
    asn1_decoder.cpp \
    verifier.cpp \
    ui.cpp
LOCAL_STATIC_LIBRARIES := \
    libmincrypt \
    libminui \
    libminzip \
    libcutils \
    libstdc++ \
    libc
include $(BUILD_EXECUTABLE)


include $(LOCAL_PATH)/minui/Android.mk \
    $(LOCAL_PATH)/minzip/Android.mk \
    $(LOCAL_PATH)/minadbd/Android.mk \
    $(LOCAL_PATH)/mtdutils/Android.mk \
    $(LOCAL_PATH)/tests/Android.mk \
    $(LOCAL_PATH)/tools/Android.mk \
    $(LOCAL_PATH)/edify/Android.mk \
    $(LOCAL_PATH)/uncrypt/Android.mk \
    $(LOCAL_PATH)/updater/Android.mk \
    $(LOCAL_PATH)/emmcutils/Android.mk	\
    $(LOCAL_PATH)/applypatch/Android.mk \
    $(LOCAL_PATH)/rsa/Android.mk	\
    $(LOCAL_PATH)/crc/Android.mk	\
    $(LOCAL_PATH)/libxml2/Android.mk \
    $(LOCAL_PATH)/rkupdate/stl/Android.mk \
    $(LOCAL_PATH)/rkupdate/rsa/Android.mk \
    $(LOCAL_PATH)/rkupdate/update/Android.mk
    
