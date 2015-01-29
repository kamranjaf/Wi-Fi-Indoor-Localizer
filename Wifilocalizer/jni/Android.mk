LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE    := LocalizationCore
LOCAL_SRC_FILES := LocalizationCore.c LocalizationCore_data.c

include $(BUILD_SHARED_LIBRARY)