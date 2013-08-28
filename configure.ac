AC_PREREQ(2.52)

dnl initialize autoconf
AC_INIT([libdce], [1.0.0], [http://www.ti.com])

dnl define PACKAGE_VERSION_* variables
dnl AS_VERSION

dnl can autoconf find the source ?
AC_CONFIG_SRCDIR([libdce.c])

dnl set the autotools aux directory
AC_CONFIG_AUX_DIR([config])

dnl AM_MAINTAINER_MODE only provides the option to configure to enable it
AM_MAINTAINER_MODE

dnl sets host_*, target_ variables
AC_CANONICAL_SYSTEM

dnl initialize automake
AM_INIT_AUTOMAKE([foreign])

dnl use pretty build output with automake >= 1.11
m4_ifdef([AM_SILENT_RULES],[AM_SILENT_RULES([yes])],
  [AM_DEFAULT_VERBOSITY=1
   AC_SUBST(AM_DEFAULT_VERBOSITY)])

dnl find a compiler
AC_PROG_CC
AM_PROG_AS
AM_PROG_LIBTOOL

dnl *** autotools stuff ***
dnl allow for different autotools
dnl AS_AUTOTOOLS_ALTERNATE

dnl *** checks for platform ***
AC_DEFINE_UNQUOTED(CPU_`echo $target_cpu | tr a-z- A-Z_`,[1])
AC_DEFINE_UNQUOTED(OS_` echo $target_os  | tr a-z- A-Z_`,[1])

dnl *** checks for programs ***
AC_PROG_INSTALL

dnl Check for pkgconfig first
AC_CHECK_PROG([HAVE_PKGCONFIG], [pkg-config], [yes], [no])

dnl *** checks for libraries ***
dnl Check for libdrm
PKG_CHECK_MODULES(DRM, libdrm libdrm_omap)

dnl *** checks for libraries ***
dnl Check for libmmrpc
PKG_CHECK_MODULES(MMRPC, libmmrpc)

dnl *** checks for header files ***
dnl check if we have ANSI C header files
AC_HEADER_STDC

dnl *** checks for types/defines ***

dnl *** checks for structures ***

dnl *** checks for compiler characteristics ***

dnl *** checks for library functions ***

dnl *** checks for dependancy libraries ***

dnl *** set variables based on configure arguments ***


if test "$CODEC_ENGINE" != "" ; then
	AC_MSG_NOTICE([Checking component : CODEC_ENGINE])
	AC_CHECK_FILES(["$CODEC_ENGINE"],,[AC_MSG_ERROR(["$CODEC_ENGINE" not found Set path variable CODEC_ENGINE])])
	AC_SUBST(CODEC_ENGINE)
else
	AC_MSG_ERROR([Set path variable CODEC_ENGINE])
fi

if test "$FRAMEWORK_COMPONENTS" != "" ; then
	AC_MSG_NOTICE([Checking component : FRAMEWORK_COMPONENTS])
        AC_CHECK_FILES(["$FRAMEWORK_COMPONENTS"],,[AC_MSG_ERROR(["$FRAMEWORK_COMPONENTS" not found Set path variable FRAMEWORK_COMPONENTS])])
	AC_SUBST(FRAMEWORK_COMPONENTS)
else
        AC_MSG_ERROR([Set path variable FRAMEWORK_COMPONENTS])
fi

if test "$IVAHD_CODECS" != "" ; then
	AC_MSG_NOTICE([Checking component : IVAHD_CODECS])
        AC_CHECK_FILES(["$IVAHD_CODECS"],,[AC_MSG_ERROR(["$IVAHD_CODECS" not fou nd Set path variable IVAHD_CODECS])])
	AC_SUBST(IVAHD_CODECS)
else
        AC_MSG_ERROR([Set path variable IVAHD_CODECS])
fi

if test "$XDAIAS" != "" ; then
	AC_MSG_NOTICE([Checking component : XDAIAS])
        AC_CHECK_FILES(["$XDAIAS"],,[AC_MSG_ERROR(["$XDAIAS" not found Set path variable XDAIAS])])
	AC_SUBST(XDAIAS)
else
        AC_MSG_ERROR([Set path variable XDAIAS])
fi

if test "$XDCTOOLS" != "" ; then
        AC_MSG_NOTICE([Checking component : XDCTOOLS])
        AC_CHECK_FILES(["$XDCTOOLS"],,[AC_MSG_ERROR(["$XDCTOOLS" not found Set path variable XDCTOOLS])])
	AC_SUBST(XDCTOOLS)
else
        AC_MSG_ERROR([Set path variable XDCTOOLS])
fi

if test "$IPC" != "" ; then
        AC_MSG_NOTICE([Checking component : IPC])
        AC_CHECK_FILES(["$IPC"],,[AC_MSG_ERROR(["$IPC" not found Set path variable IPC])])
        AC_SUBST(IPC)
else
        AC_MSG_ERROR([Set path variable IPC])
fi






AC_CONFIG_FILES([
libdce.pc
Makefile
])
AC_OUTPUT
