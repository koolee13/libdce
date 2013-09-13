#
# Copyright (C) 2010 The Android Open Source Project
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
#
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
    $(LOCAL_PATH)/../osal/inc \
    $(LOCAL_PATH)/../../ipc/packages/ti/ipc/mm \

LOCAL_SHARED_LIBRARIES := \
    libipc \
    libc \
    libcutils \
    liblog \
    libosal

LOCAL_CFLAGS += -Dxdc_target_types__=google/targets/arm/std.h -DBUILDOS_ANDROID -DDCE_DEBUG_ENABLE=1 -DDCE_DEBUG_LEVEL=1

LOCAL_MODULE_TAGS:= optional

LOCAL_SRC_FILES:= libdce.c


LOCAL_MODULE:= libdce
include $(BUILD_HEAPTRACKED_SHARED_LIBRARY)






