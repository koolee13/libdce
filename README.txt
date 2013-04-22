## Building LIBDCE for QNX ##

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
export QNX_HOST="/opt/qnx650/host/linux/x86";
export QNX_TARGET="/opt/qnx650/target/qnx6";
export PATH="/opt/qnx650/host/linux/x86/usr/bin:/opt/qnx650/host/linux/x86/bin:/opt/qnx650/host/
             linux/x86/sbin:/opt/qnx650/host/linux/x86/usr/sbin:/opt/qnx650/host/linux/x86/usr/photon/appbuilder:
             /usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/etc/qnx/bin:/etc/qnx/bin:
             /opt/qnx650:/etc/qnx/bin:/etc/qnx/bin:/etc/qnx/bin:/etc/qnx/bin";
export LD_LIBRARY_PATH="/opt/qnx650/host/linux/x86/usr/lib";
export MAKEFLAGS="-I/opt/qnx650/target/qnx6/usr/include";

then try : /opt/qnx650/host/linux/x86/usr/bin/qconfig -n "QNX Software Development Platform 6.5.0" -e

Exporting LIBDCE variables:
export TITOOLSROOT=<path to XDC, XDAIS>
export TIVIDEOTOOLSROOT=<path to CE, FC>
export IPCHEADERS=<path to IPC Headers>
export INSTALL_ROOT=<path for copying output binaries>
export QCONF_OVERRIDE=<absolute path to libdce/nto/qconf-override.mk>
export IVAHDCODECS=<path to ipumm/extrel/ti/ivahd_codecs>

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
