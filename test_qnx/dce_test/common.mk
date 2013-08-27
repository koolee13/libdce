###################### QNX DCE Test App Build Config #######################

#### Include qconfig.mk
ifndef QCONFIG
QCONFIG=qconfig.mk
endif
include $(QCONFIG)

define PINFO
PINFO DESCRIPTION=DCE Test
endef

NAME = dce_test
INSTALLDIR = bin

# Different tool versions can easily be programmed by defining below variables
# in your environment.
CEVERSION   ?= codec_engine_3_23_00_07
FCVERSION   ?= framework_components_3_23_03_17
XDAISVERSION    ?= xdais_7_23_00_06
XDCVERSION  ?= xdctools_3_25_00_48
IPCHEADERS  ?= $(INSTALL_ROOT_nto)
IVAHDCODECS ?= ipumm/extrel/ti/ivahd_codecs

# Generate the full package paths for tools
CEPROD      = $(TIVIDEOTOOLSROOT)/$(CEVERSION)
FCPROD      = $(TIVIDEOTOOLSROOT)/$(FCVERSION)
XDAISPROD   = $(TITOOLSROOT)/$(XDAISVERSION)
XDCPROD     = $(TITOOLSROOT)/$(XDCVERSION)

#Add extra include path
EXTRA_INCVPATH += $(CEPROD)/packages
EXTRA_INCVPATH += $(FCPROD)/packages
EXTRA_INCVPATH += $(XDAISPROD)/packages
EXTRA_INCVPATH += $(XDCPROD)/packages
EXTRA_INCVPATH += $(IVAHDCODECS)/packages
EXTRA_INCVPATH += $(PROJECT_ROOT)/../../
EXTRA_INCVPATH += $(IPCHEADERS)/usr/include/memmgr
EXTRA_INCVPATH += $(IPCHEADERS)/usr/include/ti/syslink
EXTRA_INCVPATH += $(IPCHEADERS)/usr/include

CCOPTS+=-g -O0 -Dxdc_target_types__=qnx/targets/arm/std.h

EXTRA_LIBVPATH += $(PROJECT_ROOT)/../nto/arm/so.le.v7 \
                  $(INSTALL_ROOT_nto)/armle-v7/usr/lib

LDOPTS+= -ldce -lmemmgr -ltilerusr -lipc_client -lsharedmemallocator

include $(MKFILES_ROOT)/qtargets.mk

