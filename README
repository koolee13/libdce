************** LIBDCE Build **************

## For QNX ##

Exporting QNX variables:
export QNX_ROOT=/opt/qnx650
export QNX_HOST=${QNX_ROOT}/host/linux/x86
export LD_LIBRARY_PATH=${QNX_HOST}/usr/lib
export QNX_TARGET=${QNX_ROOT}/target/qnx6
export MAKEFLAGS=-I${QNX_TARGET}/usr/include
export QNX_CONFIGURATION=/etc/qnx
export QNX_JAVAHOME=${QNX_ROOT}/_jvm
export PATH=${QNX_HOST}/usr/bin:${PATH}:${QNX_CONFIGURATION}/bin
export QNX_USERNAME=<registered email-id at QNX> //not important
eval `qconfig -n "QNX Software Development Platform 6.5.0" -e`

If previous eval doesn't work, that is it doesn't output:
"
export QNX_HOST="/opt/qnx650/host/linux/x86";
export QNX_TARGET="/opt/qnx650/target/qnx6";
export PATH="/opt/qnx650/host/linux/x86/usr/bin:/opt/qnx650/host/linux/x86/bin:/opt/qnx650/host/
             linux/x86/sbin:/opt/qnx650/host/linux/x86/usr/sbin:/opt/qnx650/host/linux/x86/usr/photon/appbuilder:
             /usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/etc/qnx/bin:/etc/qnx/bin:
             /opt/qnx650:/etc/qnx/bin:/etc/qnx/bin:/etc/qnx/bin:/etc/qnx/bin";
export LD_LIBRARY_PATH="/opt/qnx650/host/linux/x86/usr/lib";
export MAKEFLAGS="-I/opt/qnx650/target/qnx6/usr/include";
"

then try : /opt/qnx650/host/linux/x86/usr/bin/qconfig -n "QNX Software Development Platform 6.5.0" -e

Exporting LIBDCE variables:
export IPCHEADERS=<path to IPC Headers>
export INSTALL_ROOT=<path for copying output binaries>
export QCONF_OVERRIDE=<absolute path to libdce/nto/qconf-override.mk>

For IPCHEADERS - Headers should be at:
$(IPCHEADERS)/usr/include/memmgr
$(IPCHEADERS)/usr/include/ti/syslink/
$(IPCHEADERS)/usr/include/ti/ipc/mm
$(IPCHEADERS)/usr/include/ti/shmemallocator
$(IPCHEADERS)/usr/include/

Building:
make install

Clean:
make clean

Location of Binaries:
INSTALL_ROOT/armle-v7/usr/lib/libdce.so
INSTALL_ROOT/armle-v7/usr/lib/libdce.so.1
INSTALL_ROOT/armle-v7/bin/dce_test
INSTALL_ROOT/armle-v7/bin/dce_enc_test


**** Version Info of Headers included in packages folder ****

Tools:
XDC version  : xdctools_3_25_02_70
CE version   : codec_engine_3_24_00_08
XDAIS version: xdais_7_24_00_04

IVAHD_Codecs:
H.264 Dec  : 02.00.13.00
MJPEG Dec  : 01.00.11.01
MPEG-4 Dec : 01.00.13.00
VC-1 Dec   : 01.00.00.11
MPEG-2 Dec : 01.00.12.00
SVC Dec    : 00.06.00.00 
H.264 Enc  : 02.00.06.01
MJPEG Enc  : 01.00.02.01
SVC Enc    : 00.02.00.05

