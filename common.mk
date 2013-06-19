###################### QNX DCE Build Config #######################

#### Include qconfig.mk
ifndef QCONFIG
QCONFIG=qconfig.mk
endif

include $(QCONFIG)

#### Overriding qrules.mk macros before including qtargets.mk

# Flags to add to the C compiler command line
CCFLAGS+=-O2 -Dxdc_target_types__=qnx/targets/arm/std.h -DBUILDOS_QNX=1

# To get final library name as "libdce". Needed as project name is not dce
NAME=dce

### Add files to be included for Build
TITOOLSROOT	?= /usr/local
TIVIDEOTOOLSROOT	?= $(TITOOLSROOT)
# Different tool versions can easily be programmed by defining below variables
# in your environment.
CEVERSION	?= codec_engine_3_24_00_08
FCVERSION	?= framework_components_3_24_00_09
XDAISVERSION	?= xdais_7_24_00_04
XDCVERSION	?= xdctools_3_25_01_65
IPCHEADERS	?= $(INSTALL_ROOT_nto)

# Generate the full package paths for tools
CEPROD		= $(TIVIDEOTOOLSROOT)/$(CEVERSION)
FCPROD		= $(TIVIDEOTOOLSROOT)/$(FCVERSION)
XDAISPROD	= $(TITOOLSROOT)/$(XDAISVERSION)
XDCPROD	    = $(TITOOLSROOT)/$(XDCVERSION)

EXTRA_INCVPATH += $(CEPROD)/packages
EXTRA_INCVPATH += $(FCPROD)/packages
EXTRA_INCVPATH += $(XDAISPROD)/packages
EXTRA_INCVPATH += $(XDCPROD)/packages
EXTRA_INCVPATH += $(IPCHEADERS)/usr/include/memmgr
EXTRA_INCVPATH += $(IPCHEADERS)/usr/include/ti/syslink/
EXTRA_INCVPATH += $(IPCHEADERS)/usr/include/ti/ipc/mm
EXTRA_INCVPATH += $(IPCHEADERS)/usr/include/ti/shmemallocator
EXTRA_INCVPATH += $(IPCHEADERS)/usr/include/

# Include IPC libraries
LIBS += memmgr mmrpc sharedmemallocatorS

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
