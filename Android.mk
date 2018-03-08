LOCAL_PATH:= $(call my-dir)

#
# libdce.so
#

include $(CLEAR_VARS)

LOCAL_C_INCLUDES += \
    $(LOCAL_PATH)/packages/codec_engine/ \
    $(LOCAL_PATH)/packages/ivahd_codecs/ \
    $(LOCAL_PATH)/packages/xdais/ \
    $(LOCAL_PATH)/packages/xdctools/ \
    $(LOCAL_PATH)/ \
    hardware/ti/ipc/packages/ \
    external/libdrm/include/drm \
    external/libdrm/omap \
    external/libdrm

LOCAL_HEADER_LIBRARIES += libutils_headers

LOCAL_SHARED_LIBRARIES := \
    libmmrpc \
    libc \
    libcutils \
    liblog \
    libdrm \
    libdrm_omap

LOCAL_CFLAGS += -DBUILDOS_ANDROID -DDCE_DEBUG_ENABLE=1 -DDCE_DEBUG_LEVEL=1

LOCAL_MODULE_TAGS:= optional
LOCAL_VENDOR_MODULE := true

LOCAL_SRC_FILES:= libdce.c libdce_android.c memplugin_android.c


LOCAL_MODULE:= libdce
include $(BUILD_SHARED_LIBRARY)






