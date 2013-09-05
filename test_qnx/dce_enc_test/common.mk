###################### QNX DCE Enc Test App Build Config #######################

#### Include qconfig.mk
ifndef QCONFIG
QCONFIG=qconfig.mk
endif
include $(QCONFIG)

define PINFO
PINFO DESCRIPTION=DCE Encoder Test
endef

NAME = dce_enc_test
INSTALLDIR = bin

IPCHEADERS  ?= $(INSTALL_ROOT_nto)

#Add extra include path
EXTRA_INCVPATH += $(PROJECT_ROOT)/../../packages/codec_engine
EXTRA_INCVPATH += $(PROJECT_ROOT)/../../packages/ivahd_codecs
EXTRA_INCVPATH += $(PROJECT_ROOT)/../../packages/xdais
EXTRA_INCVPATH += $(PROJECT_ROOT)/../../packages/xdctools

EXTRA_INCVPATH += $(PROJECT_ROOT)/../../
EXTRA_INCVPATH += $(IPCHEADERS)/usr/include/memmgr
EXTRA_INCVPATH += $(IPCHEADERS)/usr/include/ti/syslink
EXTRA_INCVPATH += $(IPCHEADERS)/usr/include

CCOPTS+=-g -O0

EXTRA_LIBVPATH += $(PROJECT_ROOT)/../nto/arm/so.le.v7 \
                  $(INSTALL_ROOT_nto)/armle-v7/usr/lib

LDOPTS+= -ldce -lmemmgr -ltilerusr -lsharedmemallocator

include $(MKFILES_ROOT)/qtargets.mk

