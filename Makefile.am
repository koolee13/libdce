## Process this file with automake to produce Makefile.in


lib_LTLIBRARIES              = libdce.la


CE_CFLAGS                    = \
                               -I$(CODEC_ENGINE)/ \
                               -I$(FRAMEWORK_COMPONENTS) \
                               -I$(IVAHD_CODECS) \
                               -I$(XDAIAS) \
                               -I$(XDCTOOLS)/ \
                               -I$(IPC)/packages/ti/ipc/mm \
                               -DBUILDOS_LINUX=1 \
                               -Wno-pointer-to-int-cast


libdce_la_SOURCES            = libdce.c memplugin_linux.c
libdce_la_CFLAGS             = $(WARN_CFLAGS) $(CE_CFLAGS) $(DRM_CFLAGS) $(X11_CFLAGS) $(WAYLAND_CFLAGS)
libdce_la_LDFLAGS            = -no-undefined -version-info 1:0:0
libdce_la_LIBADD             = $(DRM_LIBS) $(X11_LIBS) $(WAYLAND_LIBS) $(MMRPC_LIBS)

if ENABLE_X11
libdce_la_SOURCES           += linux/libdce-x11.c
endif
if ENABLE_WAYLAND
libdce_la_SOURCES           += linux/libdce-wayland.c linux/wayland-drm-protocol.c
endif

libdce_la_includedir         = $(includedir)
libdce_la_include_HEADERS    = libdce.h

pkgconfig_DATA               = libdce.pc
pkgconfigdir                 = $(libdir)/pkgconfig
