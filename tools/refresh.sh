#!/bin/sh
# refresh.sh
#
#   Copyright (C) 2014 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <gnutt@nuttx.org>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name NuttX nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

USAGE="USAGE: $0 [--debug|--help] <board>/<config>"
ADVICE="Try '$0 --help' for more information"

unset CONFIG
silent=n

while [ ! -z "$1" ]; do
    case $1 in
    --debug )
        set -x
        ;;
    --silent )
        silent=y
        ;;
    --help )
        echo "$0 is a tool for refreshing board configurations"
        echo ""
        echo $USAGE
        echo ""
        echo "Where:"
        echo "  --debug"
        echo "     Enable script debug"
        echo "  --silent"
        echo "     Update board configuration without interaction"
        echo "  --help"
        echo "     Show this help message and exit"
        echo "  <board>"
        echo "     The board directory under nuttx/configs"
        echo "  <config>"
        echo "     The board configuration directory under nuttx/configs/<board>"
        exit 0
        ;;
    * )
        CONFIG=$1
        break
        ;;
    esac
    shift
done

# Get the board configuration

if [ -z "$1" ]; then
    echo "ERROR: No configuration provided"
    echo $USAGE
    echo $ADVICE
    exit 1
fi

BOARDSUBDIR=`echo $1 | cut -d'/' -f1`
CONFIGSUBDIR=`echo $1 | cut -d'/' -f2`

# Where are we

MYNAME=`basename $0`

if [ -x ./${MYNAME} ] ; then
   cd .. || { echo "ERROR: cd .. failed" ; exit 1 ; }
fi

if [ ! -x tools/${MYNAME} ] ; then
   echo "ERROR:  This file must be executed from the top-level NuttX directory: $PWD"
   exit 1
fi

# Set up the environment

WD=${PWD}

BOARDDIR=configs/$BOARDSUBDIR
CONFIGDIR=$BOARDDIR/$CONFIGSUBDIR
DEFCONFIG=$CONFIGDIR/defconfig

CMPCONFIG_TARGET=cmpconfig
CMPCONFIG1=tools/cmpconfig
CMPCONFIG2=tools/cmpconfig.exe
CMPCONFIGMAKEFILE=Makefile.host
CMPCONFIGMAKEDIR=tools

# Check the board configuration directory

if [ ! -d "$BOARDDIR" ]; then
  echo "No board directory found at $BOARDDIR"
  exit 1
fi

if [ ! -d "$CONFIGDIR" ]; then
  echo "No configuration directory found at $CONFIGDIR"
  exit 1
fi

if [ ! -r "$DEFCONFIG" ]; then
  echo "No readable defconfig file in $DEFCONFIG"
  exit 1
fi

# If the cmpconfig executable does not exist, then build it

if [ -x ${CMPCONFIG1} ]; then
  CMPCONFIG=${CMPCONFIG1}
else
  if [ -x ${CMPCONFIG2} ]; then
    CMPCONFIG=${CMPCONFIG2}
  else
    make -C ${CMPCONFIGMAKEDIR} -f ${CMPCONFIGMAKEFILE} ${CMPCONFIG_TARGET} || \
      { echo "ERROR: make ${CMPCONFIG1} failed" ; exit 1 ; }
  fi
fi

if [ -x ${CMPCONFIG1} ]; then
  CMPCONFIG=${CMPCONFIG1}
else
  if [ -x ${CMPCONFIG2} ]; then
    CMPCONFIG=${CMPCONFIG2}
  else
    echo "ERROR: Failed to create  ${CMPCONFIG1}"
    exit 1
  fi
fi

# Copy the .config to the toplevel directory

rm -f SAVEconfig
if [ -e .config ]; then
  mv .config SAVEconfig || \
    { echo "ERROR: Failed to move .config to SAVEconfig"; exit 1; }
fi

cp -a $DEFCONFIG .config || \
  { echo "ERROR: Failed to copy $DEFCONFIG to .config"; exit 1; }

# Then run oldconfig or oldefconfig

if [ "X${silent}" == "Xy" ]; then
  make olddefconfig
else
  make oldconfig
fi

# Show differences

sed -i -e "s/^CONFIG_APPS_DIR/# CONFIG_APPS_DIR/g" .config
$CMPCONFIG $DEFCONFIG .config

# Save the refreshed configuration

if [ "X${silent}" == "Xy" ]; then
  echo "Saving the new configuration file"
  mv .config $DEFCONFIG || \
      { echo "ERROR: Failed to move .config to $DEFCONFIG"; exit 1; }
  chmod 644 $DEFCONFIG
else
  read -p "Save the new configuration (y/n)?" -n 1 -r
  echo
  if [[ $REPLY =~ ^[Yy]$ ]]
  then
    echo "Saving the new configuration file"
    mv .config $DEFCONFIG || \
        { echo "ERROR: Failed to move .config to $DEFCONFIG"; exit 1; }
    chmod 644 $DEFCONFIG
  fi
fi

# Restore any previous .config file

if [ -e SAVEconfig ]; then
  mv SAVEconfig .config || \
    { echo "ERROR: Failed to move SAVEconfig to .config"; exit 1; }
fi
