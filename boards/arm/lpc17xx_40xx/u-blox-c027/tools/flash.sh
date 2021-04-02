#!/usr/bin/env bash
####################################################################################
# flash.sh
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
#
####################################################################################

USAGE="$0 <nuttx-path>"

echo "############################################################################"
echo "# Assumptions:"
echo "#"
echo "#   - Windows 7"
echo "#   - LPCXpresso 3.6 installed at /cygdrive/c/nxp/lpcxpresso_3.6"
echo "#   - AXF image built with Code Red"
echo "#   - LPC1768"
echo "#"
echo "# You will need to edit this is any of the above are false"
echo "#"
echo "############################################################################"
echo ""

# This is the default install location for binaries on Windows (note that this
# path could change with the Code Red version number)
BINDIR="/cygdrive/c/nxp/lpcxpresso_3.6/bin"
if [ ! -d "${BINDIR}" ]; then
	echo "Directory ${BINDIR} does not exist"
	exit 1
fi

# This is the relative path to the booLPCXpresso utility
BOOTLPC="Scripts/bootLPCXpresso.cmd"
if [ ! -x "${BINDIR}/$BOOTLPC" ]; then
	echo "No executable at ${BINDIR}/${BOOTLPC}"
	exit 1
fi

# BOOTLPC_ARG=winusb # WinXP
BOOTLPC_ARG=hid      # Win7

# FLASHUTIL="crt_emu_lpc11_13" # for LPC11xx or LPC13xx parts)
FLASHUTIL="crt_emu_cm3_nxp"  # for LPC17xx parts
# FLASHUTIL="crt_emu_a7_nxp"   # for LPC21/22/23/24 parts)
# FLASHUTIL="crt_emu_a9_nxp"   # for LPC31/32 and LPC29xx parts)
# FLASHUTIL="crt_emu_cm3_lmi"  # for TI Stellaris LM3S parts

if [ ! -x "${BINDIR}/${FLASHUTIL}" ]; then
	echo "No executable file at ${BINDIR}/${FLASHUTIL}"
	exit 1
fi

# unset WIRE          # for Red Probe+, Red Probe, RDB1768v1, or TI Stellaris evaluation boards
# WIRE="-wire=hi"     # for RDB1768v2 without upgraded firmware)
# WIRE="-wire=winusb" # for RDB1768v2 with upgraded firmware)
# WIRE="-wire=winusb" # for LPC-Link on Windows XP)
WIRE="-wire=hid"      # for LPC-Link on Windows Vista/Windows 7)

TARGET=LPC1768

# The nuttx directory must be provided as an argument

NUTTX=$1
if [ -z "${NUTTX}" ]; then
	echo "Missing argument"
	echo $USAGE
	exit 1
fi

if [ ! -d "${NUTTX}" ]; then
	echo "Directory ${NUTTX} does not exist"
	echo $USAGE
	exit 1
fi

# The binary to download:

if [ ! -f "${NUTTX}/nuttx.axf" ]; then
	if [ -f "${NUTTX}/nuttx" ]; then
		echo "Renaming ${NUTTX}/nuttx to ${NUTTX}/nuttx.axf"
		mv ${NUTTX}/nuttx ${NUTTX}/nuttx.axf
	fi
else
	if [ -f "${NUTTX}/nuttx" ]; then
		echo "Both ${NUTTX}/nuttx ${NUTTX}/nuttx.axf exist.."
		echo "  Deleting ${NUTTX}/nuttx.axf"
		rm -f ${NUTTX}/nuttx.axf
		echo "Renaming ${NUTTX}/nuttx to ${NUTTX}/nuttx.axf"
		mv ${NUTTX}/nuttx ${NUTTX}/nuttx.axf
	fi
fi
NUTTXPATH=`cygpath -w "${NUTTX}/nuttx.axf"`

# First of all boot the LPC-Link using the script:

cd ${BINDIR} || \
	{ echo "Failed to CD to ${BINDIR}"; exit 1; }
./${BOOTLPC} ${BOOTLPC_ARG} || \
	{ echo "'${BOOTLPC} ${BOOTLPC_ARG}' Failed"; }

echo ""
echo "Wait a bit"
echo "5..."
sleep 1
echo "4..."
sleep 1
echo "3..."
sleep 1
echo "2..."
sleep 1
echo "1..."
sleep 1
echo "0..."
echo ""

# Then program the FLASH

./${FLASHUTIL} ${WIRE} -p${TARGET} -flash-load-exec="${NUTTXPATH}"
