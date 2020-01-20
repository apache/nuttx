#!/usr/bin/env bash
# tools/testbuild.sh
#
#   Copyright (C) 2016-2019 Gregory Nutt. All rights reserved.
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

WD=$PWD
nuttx=$WD/../nuttx

progname=$0
APPSDIR=../apps
MAKE_FLAGS=-i
MAKE=make
unset testfile
unset HOPTION
unset JOPTION

function showusage {
  echo ""
  echo "USAGE: $progname [-l|m|c|u|g|n] [-d] [-x] [-j <ncpus>] [-a <appsdir>] [-t <topdir>] <testlist-file>"
  echo "       $progname -h"
  echo ""
  echo "Where:"
  echo "  -l|m|c|u|g|n selects Linux (l), macOS (m), Cygwin (c),"
  echo "     Ubuntu under Windows 10 (u), MSYS/MSYS2 (g) or Windows native (n).  Default Linux"
  echo "  -d enables script debug output"
  echo "  -x exit on build failures"
  echo "  -j <ncpus> passed on to make.  Default:  No -j make option."
  echo "  -a <appsdir> provides the relative path to the apps/ directory.  Default ../apps"
  echo "  -t <topdir> provides the absolute path to top nuttx/ directory.  Default $PWD/../nuttx"
  echo "  -h will show this help test and terminate"
  echo "  <testlist-file> selects the list of configurations to test.  No default"
  echo ""
  echo "Your PATH variable must include the path to both the build tools and the"
  echo "kconfig-frontends tools"
  echo ""
  exit 1
}

# Parse command line

while [ ! -z "$1" ]; do
  case $1 in
  -l | -m | -c | -u | -g | -n )
    HOPTION+=" $1"
    ;;
  -d )
    set -x
    ;;
  -x )
    MAKE_FLAGS='--silent --no-print-directory'
    set -e
    ;;
  -a )
    shift
    APPSDIR="$1"
    ;;
  -j )
    shift
    JOPTION="-j $1"
    ;;
  -t )
    shift
    nuttx="$1"
    ;;
  -h )
    showusage
    ;;
  * )
    testfile="$1"
    shift
    break;
    ;;
  esac
  shift
done

if [ ! -z "$1" ]; then
  echo "ERROR: Garbage at the end of line"
  showusage
fi

if [ -z "$testfile" ]; then
  echo "ERROR: Missing test list file"
  showusage
fi

if [ ! -r "$testfile" ]; then
  echo "ERROR: No readable file exists at $testfile"
  showusage
fi

if [ ! -d "$nuttx" ]; then
  echo "ERROR: Expected to find nuttx/ at $nuttx"
  showusage
fi

# Clean up after the last build

function distclean {
  cd $nuttx || { echo "ERROR: failed to CD to $nuttx"; exit 1; }
  if [ -f .config ]; then
    echo "  Cleaning..."
    ${MAKE} ${JOPTION} ${MAKE_FLAGS} distclean 1>/dev/null
  fi
}

# Configure for the next build

function configure {
  cd $nuttx/tools || { echo "ERROR: failed to CD to $nuttx/tools"; exit 1; }
  echo "  Configuring..."
  ./configure.sh ${HOPTION} $config

  if [ "X$toolchain" != "X" ]; then
    setting=`grep _TOOLCHAIN_ $nuttx/.config | grep -v CONFIG_ARCH_TOOLCHAIN_*=y | grep =y`
    varname=`echo $setting | cut -d'=' -f1`
    if [ ! -z "$varname" ]; then
      echo "  Disabling $varname"
      sed -i -e "/$varname/d" $nuttx/.config
    fi

    echo "  Enabling $toolchain"
    echo "$toolchain=y" >> $nuttx/.config
  fi
}

# Perform the next build

function build {
  cd $nuttx || { echo "ERROR: failed to CD to $nuttx"; exit 1; }
  echo "  Building NuttX..."
  echo "------------------------------------------------------------------------------------"
  ${MAKE} ${JOPTION} ${MAKE_FLAGS} 1>/dev/null
}

# Coordinate the steps for the next build test

function dotest {
  echo "------------------------------------------------------------------------------------"
  distclean
  configure
  build
}

# Perform the build test for each entry in the test list file

if [ ! -d $APPSDIR ]; then
  echo "ERROR: No directory found at $APPSDIR"
  exit 1
fi

export APPSDIR

# Shouldn't have to do this

testlist=`cat $testfile`

#while read -r line || [[ -n $line ]]; do
for line in $testlist; do
  echo "===================================================================================="
  firstch=${line:0:1}
  if [ "X$firstch" == "X#" ]; then
    echo "Skipping: $line"
  else
    echo "Configuration/Tool: $line"

    # Parse the next line

    config=`echo $line | cut -d',' -f1`
    configdir=`echo $config | cut -s -d':' -f2`
    if [ -z "${configdir}" ]; then
      configdir=`echo $config | cut -s -d'/' -f2`
      if [ -z "${configdir}" ]; then
        echo "ERROR: Malformed configuration: ${config}"
        showusage
      else
        boarddir=`echo $config | cut -d'/' -f1`
      fi
    else
      boarddir=`echo $config | cut -d':' -f1`
    fi

    path=$nuttx/boards/*/*/$boarddir/configs/$configdir
    if [ ! -r $path/defconfig ]; then
      echo "ERROR: no configuration found at $path"
      showusage
    fi

    unset toolchain;
    if [ "X$config" != "X$line" ]; then
      toolchain=`echo $line | cut -d',' -f2`
      if [ -z "$toolchain" ]; then
        echo "  Warning: no tool configuration"
      fi
    fi

    # Perform the build test

    dotest
  fi
  cd $WD || { echo "ERROR: Failed to CD to $WD"; exit 1; }
done # < $testfile

echo "===================================================================================="
