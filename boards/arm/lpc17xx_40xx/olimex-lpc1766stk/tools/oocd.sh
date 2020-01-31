#!/usr/bin/env bash
#
# See boards/olimex-lpc1766stk/README.txt for information about
# this file.

TOPDIR=$1
USAGE="$0 <TOPDIR> [-d]"
if [ -z "${TOPDIR}" ]; then
	echo "Missing argument"
	echo $USAGE
	exit 1
fi

# Assume that OpenOCD was installed and at /usr/local/bin or maybe c:\OpenOCD
#OPENOCD_PATH="/cygdrive/c/OpenOCD/openocd-0.4.0/src"
#TARGET_PATH="c:\OpenOCD\openocd-0.4.0\tcl"
OPENOCD_PATH="/usr/local/bin"
TARGET_PATH="/usr/local/share/openocd/scripts"

OPENOCD_EXE=openocd.exe
#OPENOCD_CFG=`cygpath -w "${TOPDIR}/boards/olimex-lpc1766stk/tools/olimex.cfg"`
OPENOCD_CFG="${TOPDIR}/boards/olimex-lpc1766stk/tools/olimex.cfg"
OPENOCD_ARGS="-f ${OPENOCD_CFG} -s ${TARGET_PATH}"

if [ "X$2" = "X-d" ]; then
	OPENOCD_ARGS=$OPENOCD_ARGS" -d3"
	set -x
fi

if [ ! -d ${OPENOCD_PATH} ]; then
	echo "OpenOCD path does not exist: ${OPENOCD_PATH}"
	exit 1
fi
if [ ! -x ${OPENOCD_PATH}/${OPENOCD_EXE} ]; then
	echo "OpenOCD does not exist: ${OPENOCD_PATH}/${OPENOCD_EXE}"
	exit 1
fi
if [ ! -f ${OPENOCD_CFG} ]; then
	echo "OpenOCD config file does not exist: ${OPENOCD_CFG}"
	exit 1
fi

echo "Starting OpenOCD"
cd ${OPENOCD_PATH} || { echo "Failed to CD to ${OPENOCD_PATH}"; exit 1; }
${OPENOCD_EXE} ${OPENOCD_ARGS} &
echo "OpenOCD daemon started"
ps -ef | grep openocd
echo "In GDB: target remote localhost:3333"
