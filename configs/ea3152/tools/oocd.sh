#!/bin/sh

TOPDIR=$1
USAGE="$0 <TOPDIR> [-d]"
if [ -z "${TOPDIR}" ]; then
	echo "Missing argument"
	echo $USAGE
	exit 1
fi

###############################################################################
# OpenOCD 0.4.0 ###############################################################
###############################################################################
# This script *probably* only works with the following version of OpenOCD:

OPENOCD_PATH="/cygdrive/c/OpenOCD/openocd-0.4.0/src"
OPENOCD_EXE=openocd.exe

# Local search directory and configurations

OPENOCD_SEARCHDIR="${TOPDIR}/configs/ea3152/tools"
OPENOCD_WSEARCHDIR="`cygpath -w ${OPENOCD_SEARCHDIR}`"
OPENOCD_INTERFACE="olimex-arm-usb-ocd.cfg"
OPENOCD_TARGET="lpc3152.cfg"
OPENOCD_ARGS="-s ${OPENOCD_WSEARCHDIR} -f ${OPENOCD_INTERFACE} -f ${OPENOCD_TARGET}"

# Verify that everything is what it claims it is and is located where it claims it is.

if [ ! -d "${OPENOCD_PATH}" ]; then
	echo "OpenOCD path does not exist: ${OPENOCD_PATH}"
	exit 1
fi
if [ ! -x "${OPENOCD_PATH}/${OPENOCD_EXE}" ]; then
	echo "OpenOCD does not exist: ${OPENOCD_PATH}/${OPENOCD_EXE}"
	exit 1
fi
if [ ! -f "${OPENOCD_SEARCHDIR}/${OPENOCD_TARGET}" ]; then
	echo "OpenOCD target config file does not exist: ${OPENOCD_SEARCHDIR}/${OPENOCD_TARGET}"
	exit 1
fi
if [ ! -f "${OPENOCD_SEARCHDIR}/${OPENOCD_INTERFACE}" ]; then
	echo "OpenOCD interface config file does not exist: ${OPENOCD_SEARCHDIR}/${OPENOCD_INTERFACE}"
	exit 1
fi
###############################################################################

###############################################################################
# Older OpenOCD that Shipped with the ARM-OCD JTAG ############################
###############################################################################
# This script *probably* only works with the following version of OpenOCD:
#
#OPENOCD_PATH="/cygdrive/c/gccfd/openocd/bin"
#OPENOCD_EXE=openocd-ftd2xx.exe
#
# Local search directory and configurations
#
#OPENOCD_CFG="${TOPDIR}/configs/ea3152/tools/armusbocd.cfg"
#OPENOCD_ARGS="-f `cygpath -w ${OPENOCD_CFG}`"
#
#if [ ! -f ${OPENOCD_CFG} ]; then
#	echo "OpenOCD config file does not exist: ${OPENOCD_CFG}"
#	exit 1
#fi
#
# Verify that everything is what it claims it is and is located where it claims it is.
#
#if [ ! -d "${OPENOCD_PATH}" ]; then
#	echo "OpenOCD path does not exist: ${OPENOCD_PATH}"
#	exit 1
#fi
#if [ ! -x "${OPENOCD_PATH}/${OPENOCD_EXE}" ]; then
#	echo "OpenOCD does not exist: ${OPENOCD_PATH}/${OPENOCD_EXE}"
#	exit 1
#fi
#if [ ! -f ${OPENOCD_CFG} ]; then
#	echo "OpenOCD config file does not exist: ${OPENOCD_CFG}"
#	exit 1
#fi
###############################################################################

# Enable debug if so requested

if [ "X$2" = "X-d" ]; then
	OPENOCD_ARGS=$OPENOCD_ARGS" -d3"
	set -x
fi

# Okay... do it!

echo "Starting OpenOCD"
${OPENOCD_PATH}/${OPENOCD_EXE} ${OPENOCD_ARGS} &
echo "OpenOCD daemon started"
ps -ef | grep openocd
echo "In GDB: target remote localhost:3333"
