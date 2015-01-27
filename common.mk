###################### QNX DCE Build Config #######################

#### Include qconfig.mk
ifndef QCONFIG
QCONFIG=qconfig.mk
endif

include $(QCONFIG)

#### Overriding qrules.mk macros before including qtargets.mk

# Flags to add to the C compiler command line
CCFLAGS+=-O2 -DBUILDOS_QNX=1 -DDCE_DEBUG_ENABLE=1 -DDCE_DEBUG_LEVEL=1

# To get final library name as "libdce". Needed as project name is not dce
NAME=dce

# Path to IPC Header files
IPCHEADERS	?= $(INSTALL_ROOT_nto)

EXTRA_INCVPATH += $(PROJECT_ROOT)/packages/codec_engine
EXTRA_INCVPATH += $(PROJECT_ROOT)/packages/ivahd_codecs
EXTRA_INCVPATH += $(PROJECT_ROOT)/packages/xdais
EXTRA_INCVPATH += $(PROJECT_ROOT)/packages/xdctools
EXTRA_INCVPATH += $(PROJECT_ROOT)/packages/framework_components

EXTRA_INCVPATH += $(IPCHEADERS)/usr/include/memmgr
EXTRA_INCVPATH += $(IPCHEADERS)/usr/include/ti/syslink/
EXTRA_INCVPATH += $(IPCHEADERS)/usr/include/ti/ipc/mm
EXTRA_INCVPATH += $(IPCHEADERS)/usr/include/ti/shmemallocator
EXTRA_INCVPATH += $(IPCHEADERS)/usr/include/

# Include IPC libraries
LIBS += memmgr mmrpc sharedmemallocatorS

# Exclude Linux & Android files for compile
EXCLUDE_OBJS=memplugin_linux.o memplugin_android.o libdce_linux.o libdce_android.o

# Include qmacros.mk
include $(MKFILES_ROOT)/qmacros.mk

#### Overriding qtargets.mk macros before including qtargets.mk
INSTALLDIR=usr/lib
INSTALLDIR+=usr/local

define PINFO
PINFO DESCRIPTION = libdce codec
endef

#### Include qtargets.mk, it internally includes qrules.mk
include $(MKFILES_ROOT)/qtargets.mk

#### Post-set make macros after including qtargets.mk (if-any)
