#!/usr/bin/env bash
############################################################################
# boards/arm/stm32f2/stm3220g-eval/tools/oocd.sh
#
# SPDX-License-Identifier: Apache-2.0
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
############################################################################
# Get command line parameters

USAGE="USAGE: $0 [-dh] <TOPDIR>"
ADVICE="Try '$0 -h' for more information"

while [ ! -z "$1" ]; do
	case $1 in
	-d )
		set -x
		;;
	-h )
		echo "$0 is a tool for generation of proper version files for the NuttX build"
		echo ""
		echo $USAGE
		echo ""
		echo "Where:"
		echo "  -d"
		echo "	  Enable script debug"
		echo "  -h"
		echo "	  show this help message and exit"
		echo "	  Use the OpenOCD 0.4.0"
		echo "  <TOPDIR>"
		echo "	  The full path to the top-level NuttX directory"
		exit 0
		;;
	* )
		break;
		;;
	esac
	shift
done

TOPDIR=$1
if [ -z "${TOPDIR}" ]; then
	echo "Missing argument"
	echo $USAGE
	echo $ADVICE
	exit 1
fi

# This script *probably* only works with the following versions of OpenOCD:

# Local search directory and configurations

OPENOCD_SEARCHDIR="${TOPDIR}/boards/arm/stm32f2/stm3220g-eval/tools"
OPENOCD_WSEARCHDIR="`cygpath -w ${OPENOCD_SEARCHDIR}`"

OPENOCD_PATH="/cygdrive/c/Program Files (x86)/OpenOCD/0.4.0/bin"
OPENOCD_EXE=openocd.exe
OPENOCD_INTERFACE="olimex-arm-usb-ocd.cfg"

OPENOCD_TARGET="stm32.cfg"
OPENOCD_ARGS="-s ${OPENOCD_WSEARCHDIR} -f ${OPENOCD_INTERFACE} -f ${OPENOCD_TARGET}"

echo "Trying OpenOCD 0.4.0 path: ${OPENOCD_PATH}/${OPENOCD_EXE}"

# Verify that everything is what it claims it is and is located where it claims it is.

if [ ! -x "${OPENOCD_PATH}/${OPENOCD_EXE}" ]; then
	echo "OpenOCD executable does not exist: ${OPENOCD_PATH}/${OPENOCD_EXE}"
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

# Enable debug if so requested

if [ "X$2" = "X-d" ]; then
	OPENOCD_ARGS=$OPENOCD_ARGS" -d3"
	set -x
fi

# Okay... do it!

echo "Starting OpenOCD"
"${OPENOCD_PATH}/${OPENOCD_EXE}" ${OPENOCD_ARGS} &
echo "OpenOCD daemon started"
ps -ef | grep openocd
echo "In GDB: target remote localhost:3333"
