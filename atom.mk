
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := libvideo-buffers
LOCAL_CATEGORY_PATH := libs
LOCAL_DESCRIPTION := Video buffers library
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/include
# Public API headers - top level headers first
# This header list is currently used to generate a python binding
LOCAL_EXPORT_CUSTOM_VARIABLES := LIBVIDEOBUFFERS_HEADERS=$\
	$(LOCAL_PATH)/include/video-buffers/vbuf.h;
LOCAL_CFLAGS := -DVBUF_API_EXPORTS -fvisibility=hidden -std=gnu99
LOCAL_SRC_FILES := \
	src/vbuf.c \
	src/vbuf_pool.c \
	src/vbuf_queue.c
LOCAL_LIBRARIES := \
	libfutils \
	libpomp \
	libulog

include $(BUILD_LIBRARY)


include $(CLEAR_VARS)
LOCAL_MODULE := libvideo-buffers-generic
LOCAL_CATEGORY_PATH := libs
LOCAL_DESCRIPTION := Video buffers library, generic implementation
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/implem/generic/include
LOCAL_EXPORT_CUSTOM_VARIABLES := LIBVIDEOBUFFERSGENERIC_HEADERS=$\
	$(LOCAL_PATH)/implem/generic/include/video-buffers/vbuf_generic.h;
LOCAL_CFLAGS := -DVBUF_API_EXPORTS -fvisibility=hidden -std=gnu99
LOCAL_SRC_FILES := \
	implem/generic/src/vbuf_generic.c
LOCAL_LIBRARIES := \
	libfutils \
	libulog \
	libvideo-buffers

include $(BUILD_LIBRARY)
