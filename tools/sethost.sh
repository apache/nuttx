#!/bin/bash
# tools/sethost.sh
#
#   Copyright (C) 2016-2017 Gregory Nutt. All rights reserved.
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

progname=$0
host=linux
wenv=cygwin
hsize=64
unset configfile

function showusage {
    echo ""
    echo "USAGE: $progname [-w|l|m] [-c|u|g|n] [-32|64] [<config>]"
    echo "       $progname -h"
    echo ""
    echo "Where:"
    echo "  -w|l|m selects Windows (w), Linux (l), or macOS (m).  Default: Linux"
    echo "  -c|u|g|n selects Windows environment option:  Cygwin (c), Ubuntu under"
    echo "     Windows 10 (u), MSYS/MSYS2 (g) or Windows native (n).  Default Cygwin"
    echo "  -32|64 selects 32- or 64-bit host.  Default 64"
    echo "  -h will show this help test and terminate"
    echo "  <config> selects configuration file.  Default: .config"
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
    -g )
      host=windows
      wenv=msys
      ;;
    -u )
      host=windows
      wenv=ubuntu
      ;;
    -m )
      host=macos
      ;;
    -n )
      host=windows
      wenv=native
      ;;
    -32 )
      hsize=32
      ;;
    -64 )
      hsize=32
      ;;
    -h )
      showusage
      ;;
    * )
      configfile="$1"
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

if [ -x sethost.sh ]; then
  nuttx=$PWD/..
else
  if [ -x tools/sethost.sh ]; then
    nuttx=$PWD
  else
    echo "This script must be execute in nuttx/ or nutts/tools directories"
    exit 1
  fi
fi

rm -f $nuttx/SAVEconfig
rm -f $nuttx/SAVEMake.defs

unset dotconfig
if [ -z "$configfile" ]; then
  dotconfig=y
else
  if [ "X$configfile" = "X.config" ]; then
    dotconfig=y
  else
    if [ "X$configfile" = "X$nuttx/.config" ]; then
      dotconfig=y
    fi
  fi
fi

if [ "X$dotconfig" = "Xy" ]; then
  unset configfile
  if [ -r $nuttx/.config ]; then
    configfile=$nuttx/.config
  else
    echo "There is no .config at $nuttx"
    exit 1
  fi

  if [ ! -r $nuttx/Make.defs ]; then
    echo "ERROR: No readable Make.defs file exists at $nuttx"
    exit 1
  fi
else
  if [ ! -r "$configfile" ]; then
    echo "ERROR: No readable configuration file exists at $configfile"
    exit 1
  fi

  configdir=`dirname $configfile`
  makedefs=$configdir/Make.defs

  if [ ! -r $makedefs ]; then
    echo "ERROR: No readable Make.defs file exists at $configdir"
    exit 1
  fi

  if [ -f $nuttx/.config ]; then
    mv $nuttx/.config $nuttx/SAVEconfig
  fi
  cp $configfile $nuttx/.config || \
    { echo "ERROR: cp to $nuttx/.config failed"; exit 1; }

  if [ -f $nuttx/Make.defs ]; then
    mv $nuttx/Make.defs $nuttx/SAVEMake.defs
  fi
  cp $makedefs $nuttx/Make.defs || \
    { echo "ERROR: cp to $nuttx/Make.defs failed"; exit 1; }
fi

# Modify the configuration

if [ "X$host" == "Xlinux" -o "X$host" == "Xmacos" ]; then

    # Enable Linux or macOS

    if [ "X$host" == "Xlinux" ]; then
      echo "  Select CONFIG_HOST_LINUX=y"

      kconfig-tweak --file $nuttx/.config --enable CONFIG_HOST_LINUX
      kconfig-tweak --file $nuttx/.config --disable CONFIG_HOST_MACOS
    else
      echo "  Select CONFIG_HOST_MACOS=y"

      kconfig-tweak --file $nuttx/.config --disable CONFIG_HOST_LINUX
      kconfig-tweak --file $nuttx/.config --enable CONFIG_HOST_MACOS
    fi

    # Disable all Windows options

    kconfig-tweak --file $nuttx/.config --disable CONFIG_HOST_WINDOWS
    kconfig-tweak --file $nuttx/.config --disable CONFIG_SIM_X8664_MICROSOFT
    kconfig-tweak --file $nuttx/.config --enable CONFIG_SIM_X8664_SYSTEMV

    kconfig-tweak --file $nuttx/.config --disable CONFIG_WINDOWS_NATIVE
    kconfig-tweak --file $nuttx/.config --disable CONFIG_WINDOWS_CYGWIN
    kconfig-tweak --file $nuttx/.config --disable CONFIG_WINDOWS_UBUNTU
    kconfig-tweak --file $nuttx/.config --disable CONFIG_WINDOWS_MSYS
    kconfig-tweak --file $nuttx/.config --disable CONFIG_WINDOWS_OTHER

    kconfig-tweak --file $nuttx/.config --enable CONFIG_SIM_X8664_SYSTEMV
    kconfig-tweak --file $nuttx/.config --disable CONFIG_SIM_X8664_MICROSOFT
    kconfig-tweak --file $nuttx/.config --disable CONFIG_SIM_M32
else
    echo "  Select CONFIG_HOST_WINDOWS=y"

    # Enable Windows

    kconfig-tweak --file $nuttx/.config --enable CONFIG_HOST_WINDOWS
    kconfig-tweak --file $nuttx/.config --disable CONFIG_HOST_LINUX
    kconfig-tweak --file $nuttx/.config --disable CONFIG_HOST_MACOS

    kconfig-tweak --file $nuttx/.config --enable CONFIG_SIM_X8664_MICROSOFT
    kconfig-tweak --file $nuttx/.config --disable CONFIG_SIM_X8664_SYSTEMV

    # Enable Windows environment

    kconfig-tweak --file $nuttx/.config --disable CONFIG_WINDOWS_OTHER
    if [ "X$wenv" == "Xcygwin" ]; then
      echo "  Select CONFIG_WINDOWS_CYGWIN=y"
      kconfig-tweak --file $nuttx/.config --enable CONFIG_WINDOWS_CYGWIN
      kconfig-tweak --file $nuttx/.config --disable CONFIG_WINDOWS_MSYS
      kconfig-tweak --file $nuttx/.config --disable CONFIG_WINDOWS_UBUNTU
      kconfig-tweak --file $nuttx/.config --disable CONFIG_WINDOWS_NATIVE
    else
      kconfig-tweak --file $nuttx/.config --disable CONFIG_WINDOWS_CYGWIN
      if [ "X$wenv" == "Xmsys" ]; then
        echo "  Select CONFIG_WINDOWS_MSYS=y"
        kconfig-tweak --file $nuttx/.config --enable CONFIG_WINDOWS_MSYS
        kconfig-tweak --file $nuttx/.config --disable CONFIG_WINDOWS_UBUNTU
        kconfig-tweak --file $nuttx/.config --disable CONFIG_WINDOWS_NATIVE
      else
        kconfig-tweak --file $nuttx/.config --disable CONFIG_WINDOWS_MSYS
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
    fi

    if [ "X$hsize" == "X32" ]; then
      kconfig-tweak --file $nuttx/.config --enable CONFIG_SIM_M32
    else
      kconfig-tweak --file $nuttx/.config --disable CONFIG_SIM_M32
    fi
fi

kconfig-tweak --file $nuttx/.config --disable CONFIG_HOST_MACOS
kconfig-tweak --file $nuttx/.config --disable CONFIG_HOST_OTHER

echo "  Refreshing..."
cd $nuttx || { echo "ERROR: failed to cd to $nuttx"; exit 1; }
make clean_context 1>/dev/null 2>&1
make olddefconfig 1>/dev/null 2>&1

# Move config file to correct location and restore any previous .config
# and Make.defs files

if [ "X$dotconfig" != "Xy" ]; then
  sed -i -e "s/^CONFIG_APPS_DIR/# CONFIG_APPS_DIR/g" .config

  mv .config $configfile || \
      { echo "ERROR: Failed to move .conig to $configfile"; exit 1; }

  if [ -e SAVEconfig ]; then
    mv SAVEconfig .config || \
      { echo "ERROR: Failed to move SAVEconfig to .config"; exit 1; }
  fi

  if [ -e SAVEMake.defs ]; then
    mv SAVEMake.defs Make.defs || \
      { echo "ERROR: Failed to move SAVEMake.defs to Make.defs"; exit 1; }
  fi
fi
