#!/usr/bin/env bash
############################################################################
# boards/arm/lpc31xx/ea3131/tools/mklpc.sh
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
# This script lies in sub-directory boards/arm/lpc31xx/ea3131/tools but make be executed
# from either that directory or TOPDIR

MYNAME=`basename $0`
if [ -x "$PWD/$MYNAME" ]; then
	TOPDIR="$PWD/../../.."
else
	if [ -x "$PWD/boards/arm/lpc31xx/ea3131/tools/$MYNAME" ]; then
		TOPDIR="$PWD"
	else
		echo "This script must be executed from a known director"
		exit 1
	fi
fi
echo "TOOLDIR: $TOOLDIR"

# The lpchdr could be named lpchdr.exe if we are running under Cygwin or
# just lpchdr under Linux

TOOLDIR=$TOPDIR/boards/arm/lpc31xx/ea3131/tools

if [ ! -d "$TOOLDIR" ]; then
	echo "Tool directory $TOOLDIR does not exist"
	exit 1
fi

if [ -x "$TOOLDIR/lpchdr.exe" ]; then
	LPCHDR="$TOOLDIR/lpchdr.exe"
else
	if  [ -x "$TOOLDIR/lpchdr" ]; then
		LPCHDR="$TOOLDIR/lpchdr"
	else
		echo "lpchdr executable does not exist in $TOODIR"
		echo " - cd $TOOLDIR"
		echo " - make"
	fi
fi
echo "LPCHDR: $LPCHDR"

# Now get the path to the NuttX executable

NUTTXPATH="$TOPDIR/nuttx.bin"

if [ ! -f "$NUTTXPATH" ]; then
	echo "NuttX binary does not exist at $NUTTXPATH"
	echo " - cd $TOPDIR"
	echo " - make"
	exit 1
fi
echo "NUTTXPATH: $NUTTXPATH"

# Create the binary

echo "COMMAND: $LPCHDR -o $TOPDIR/nuttx.lpc $NUTTXPATH"
"$LPCHDR" -o "$TOPDIR/nuttx.lpc" "$NUTTXPATH" || \
	{ echo "$LPCHDR failed" ; exit 1 ; }
echo "Successfully created binary"
