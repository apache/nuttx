#!/bin/bash
# testbuild.sh
#
#   Copyright (C) 2016-2018 Gregory Nutt. All rights reserved.
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
TOOLSDIR=$nuttx/tools
UNLINK=$TOOLSDIR/unlink.sh

progname=$0
host=linux
wenv=cygwin
sizet=uint
APPSDIR=../apps
NXWDIR=../NxWidgets
MAKE_FLAGS=-i
MAKE=make
unset testfile

function showusage {
    echo ""
    echo "USAGE: $progname [-w|l] [-c|u|n] [-s] [-a <appsdir>] [-n <nxdir>] <testlist-file>"
    echo "       $progname -h"
    echo ""
    echo "Where:"
    echo "  -w|l selects Windows (w) or Linux (l).  Default: Linux"
    echo "  -c|u|n selects Windows environment option:  Cygwin (c), Ubuntu under"
    echo "     Windows 10 (u), or Windows native (n).  Default Cygwin"
    echo "  -s Use C++ unsigned long size_t in new operator. Default unsigned int"
    echo "  -a <appsdir> provides the relative path to the apps/ directory.  Default ../apps"
    echo "  -t <topdir> provides the absolute path to top nuttx/ directory.  Default $PWD/../nuttx"
    echo "  -n <nxdir> provides the relative path to the NxWidgets/ directory.  Default ../NxWidgets"
    echo "  -d enables script debug output"
    echo "  -x exit on build failures"
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
    -w )
    host=windows
    ;;
    -l )
    host=linux
    ;;
    -c )
    host=windows
    wenv=cygwin
    ;;
    -d )
    set -x
    ;;
    -u )
    host=windows
    wenv=ubuntu
    ;;
    -n )
    host=windows
    wenv=native
    ;;
    -s )
    host=windows
    sizet=long
    ;;
    -x )
    MAKE_FLAGS='--silent --no-print-directory'
    set -e
    ;;
    -a )
    shift
    APPSDIR="$1"
    ;;
    -t )
    shift
    nuttx="$1"
    ;;
    -n )
    shift
    NXWDIR="$1"
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
        ${MAKE} ${MAKE_FLAGS} distclean 1>/dev/null
    fi
}

# Configure for the next build

function configure {
    cd $nuttx/tools || { echo "ERROR: failed to CD to $nuttx/tools"; exit 1; }
    echo "  Configuring..."
    ./configure.sh $config

    cd $nuttx || { echo "ERROR: failed to CD to $nuttx"; exit 1; }

    if [ "X$host" == "Xlinux" ]; then
        echo "  Select CONFIG_HOST_LINUX=y"

        kconfig-tweak --file $nuttx/.config --enable CONFIG_HOST_LINUX
        kconfig-tweak --file $nuttx/.config --disable CONFIG_HOST_WINDOWS

        kconfig-tweak --file $nuttx/.config --disable CONFIG_WINDOWS_NATIVE
        kconfig-tweak --file $nuttx/.config --disable CONFIG_WINDOWS_CYGWIN
        kconfig-tweak --file $nuttx/.config --disable CONFIG_WINDOWS_MSYS
        kconfig-tweak --file $nuttx/.config --disable CONFIG_WINDOWS_OTHER

        kconfig-tweak --file $nuttx/.config --enable CONFIG_SIM_X8664_SYSTEMV
        kconfig-tweak --file $nuttx/.config --disable CONFIG_SIM_X8664_MICROSOFT
        kconfig-tweak --file $nuttx/.config --disable CONFIG_SIM_M32
    else
        echo "  Select CONFIG_HOST_WINDOWS=y"
        kconfig-tweak --file $nuttx/.config --enable CONFIG_HOST_WINDOWS
        kconfig-tweak --file $nuttx/.config --disable CONFIG_HOST_LINUX

        if [ "X$wenv" == "Xcygwin" ]; then
          echo "  Select CONFIG_WINDOWS_CYGWIN=y"
          kconfig-tweak --file $nuttx/.config --enable CONFIG_WINDOWS_CYGWIN
          kconfig-tweak --file $nuttx/.config --disable CONFIG_WINDOWS_UBUNTU
          kconfig-tweak --file $nuttx/.config --disable CONFIG_WINDOWS_NATIVE
        else
          kconfig-tweak --file $nuttx/.config --disable CONFIG_WINDOWS_CYGWIN
          if [ "X$wenv" == "Xubuntu" ]; then
            echo "  Select CONFIG_WINDOWS_UBUNTU=y"
            kconfig-tweak --file $nuttx/.config --enable CONFIG_WINDOWS_UBUNTU
            kconfig-tweak --file $nuttx/.config --disable CONFIG_WINDOWS_NATIVE
          else
            echo "  Select CONFIG_WINDOWS_NATIVE=y"
            kconfig-tweak --file $nuttx/.config --disable CONFIG_WINDOWS_UBUNTU
            kconfig-tweak --file $nuttx/.config --enable CONFIG_WINDOWS_NATIVE
          fi
        fi

        kconfig-tweak --file $nuttx/.config --disable CONFIG_WINDOWS_MSYS
        kconfig-tweak --file $nuttx/.config --disable CONFIG_WINDOWS_OTHER

        kconfig-tweak --file $nuttx/.config --enable CONFIG_SIM_X8664_MICROSOFT
        kconfig-tweak --file $nuttx/.config --disable CONFIG_SIM_X8664_SYSTEMV
        kconfig-tweak --file $nuttx/.config --disable CONFIG_SIM_M32
    fi

    kconfig-tweak --file $nuttx/.config --disable CONFIG_HOST_MACOS
    kconfig-tweak --file $nuttx/.config --disable CONFIG_HOST_OTHER

    if [ "X$sizet" == "Xlong" ]; then
        echo "  Select CONFIG_CXX_NEWLONG=y"
        kconfig-tweak --file $nuttx/.config --enable CONFIG_CXX_NEWLONG
    else
        echo "  Disable CONFIG_CXX_NEWLONG"
        kconfig-tweak --file $nuttx/.config --disable CONFIG_CXX_NEWLONG
    fi

    if [ "X$toolchain" != "X" ]; then
        setting=`grep TOOLCHAIN $nuttx/.config | grep -v CONFIG_ARCH_TOOLCHAIN_GNU=y | grep =y`
        varname=`echo $setting | cut -d'=' -f1`
        if [ ! -z "$varname" ]; then
            echo "  Disabling $varname"
            kconfig-tweak --file $nuttx/.config --disable $varname
        fi

        echo "  Enabling $toolchain"
        kconfig-tweak --file $nuttx/.config --enable $toolchain
    fi

    echo "  Refreshing..."
    cd $nuttx || { echo "ERROR: failed to CD to $nuttx"; exit 1; }
    ${MAKE} ${MAKE_FLAGS} olddefconfig 1>/dev/null 2>&1
}

# Build the NxWidgets libraries

function nxbuild {
    if [ -e $APPSDIR/external ]; then
        $UNLINK $APPSDIR/external
    fi

    unset nxconfig
    if [ -d $NXWDIR ]; then
        nxconfig=`grep CONFIG_NXWM=y $nuttx/.config` || true
    fi

    if [ ! -z "$nxconfig" ]; then
        echo "  Building NxWidgets..."
        echo "------------------------------------------------------------------------------------"

        cd $nuttx/$NXTOOLS || { echo "Failed to CD to $NXTOOLS"; exit 1; }
        ./install.sh $nuttx/$APPSDIR nxwm 1>/dev/null

        ${MAKE} ${MAKE_FLAGS} -C $nuttx/$APPSDIR/external TOPDIR=$nuttx APPDIR=$nuttx/$APPSDIR TOPDIR=$nuttx clean 1>/dev/null

        cd $nuttx || { echo "Failed to CD to $nuttx"; exit 1; }
        ${MAKE} ${MAKE_FLAGS} context 1>/dev/null

        cd $nuttx/$NXWIDGETSDIR || { echo "Failed to CD to $NXWIDGETSDIR"; exit 1; }
        ${MAKE} ${MAKE_FLAGS} TOPDIR=$nuttx clean 1>/dev/null
        ${MAKE} ${MAKE_FLAGS} TOPDIR=$nuttx  1>/dev/null

        cd $nuttx/$NXWMDIR || { echo "Failed to CD to $NXWMDIR"; exit 1; }
        ${MAKE} ${MAKE_FLAGS} TOPDIR=$nuttx clean 1>/dev/null
        ${MAKE} ${MAKE_FLAGS} TOPDIR=$nuttx  1>/dev/null
    fi
}

# Perform the next build

function build {
    cd $nuttx || { echo "ERROR: failed to CD to $nuttx"; exit 1; }
    echo "  Building NuttX..."
    echo "------------------------------------------------------------------------------------"
    ${MAKE} ${MAKE_FLAGS} 1>/dev/null
}

# Coordinate the steps for the next build test

function dotest {
    echo "------------------------------------------------------------------------------------"
    distclean
    configure
    nxbuild
    build
}

# Perform the build test for each entry in the test list file

if [ ! -d $APPSDIR ]; then
  export "ERROR: No directory found at $APPSDIR"
  exit 1
fi

export APPSDIR

if [ -d $NXWDIR ]; then
    NXWIDGETSDIR=$NXWDIR/libnxwidgets
    NXWMDIR=$NXWDIR/nxwm
    NXTOOLS=$NXWDIR/tools
fi

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

      path=$nuttx/configs/$config
      if [ ! -r "$path/defconfig" ]; then
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
