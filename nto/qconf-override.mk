# If you wish to override the values of some of the macros defined in qconfig.mk
# without modifying the contents of the file, set the QCONF_OVERRIDE environment
# variable to this file (which will be included at the end of the qconfig.mk file)
# and override the qconfig.mk macros here.

# Override INSTALL_ROOT_nto macro
INSTALL_ROOT ?= $(shell pwd)
INSTALL_ROOT_nto := $(INSTALL_ROOT)
USE_INSTALL_ROOT = 1
