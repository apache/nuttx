#!/usr/bin/env bash
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

progname=$0
debug=n
host=linux
wenv=cygwin
unset configfile

function showusage {
  echo ""
  echo "USAGE: $progname -d [-l|m|c|u|g|n] [<config>]"
  echo "       $progname -h"
  echo ""
  echo "Where:"
  echo "  -d enables script debug output"
  echo "  -l|m|c|u|g|n selects Linux (l), macOS (m), Cygwin (c),"
  echo "     Ubuntu under Windows 10 (u), MSYS/MSYS2 (g)"
  echo "     or Windows native (n). Default Linux"
  echo "  -h will show this help test and terminate"
  echo "  <config> selects configuration file.  Default: .config"
  exit 1
}

# Parse command line

while [ ! -z "$1" ]; do
  case $1 in
  -d )
    debug=y
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

    echo "CONFIG_HOST_LINUX=y" >> $nuttx/.config
    sed -i -e "/CONFIG_HOST_MACOS/d" $nuttx/.config
  else
    echo "  Select CONFIG_HOST_MACOS=y"

    sed -i -e "/CONFIG_HOST_LINUX/d" $nuttx/.config
    echo "CONFIG_HOST_MACOS=y" >> $nuttx/.config
  fi

  # Disable all Windows options

  sed -i -e "/CONFIG_HOST_WINDOWS/d" $nuttx/.config
  sed -i -e "/CONFIG_SIM_X8664_MICROSOFT/d" $nuttx/.config
  echo "CONFIG_SIM_X8664_SYSTEMV=y" >> $nuttx/.config

  sed -i -e "/CONFIG_WINDOWS_NATIVE/d" $nuttx/.config
  sed -i -e "/CONFIG_WINDOWS_CYGWIN/d" $nuttx/.config
  sed -i -e "/CONFIG_WINDOWS_UBUNTU/d" $nuttx/.config
  sed -i -e "/CONFIG_WINDOWS_MSYS/d" $nuttx/.config
  sed -i -e "/CONFIG_WINDOWS_OTHER/d" $nuttx/.config
else
  echo "  Select CONFIG_HOST_WINDOWS=y"

  # Enable Windows

  echo "CONFIG_HOST_WINDOWS=y" >> $nuttx/.config
  sed -i -e "/CONFIG_HOST_LINUX/d" $nuttx/.config
  sed -i -e "/CONFIG_HOST_MACOS/d" $nuttx/.config

  echo "CONFIG_SIM_X8664_MICROSOFT=y" >> $nuttx/.config
  sed -i -e "/CONFIG_SIM_X8664_SYSTEMV/d" $nuttx/.config

  # Enable Windows environment

  sed -i -e "/CONFIG_WINDOWS_OTHER/d" $nuttx/.config
  if [ "X$wenv" == "Xcygwin" ]; then
    echo "  Select CONFIG_WINDOWS_CYGWIN=y"
    echo "CONFIG_WINDOWS_CYGWIN=y" >> $nuttx/.config
    sed -i -e "/CONFIG_WINDOWS_MSYS/d" $nuttx/.config
    sed -i -e "/CONFIG_WINDOWS_UBUNTU/d" $nuttx/.config
    sed -i -e "/CONFIG_WINDOWS_NATIVE/d" $nuttx/.config
  else
    sed -i -e "/CONFIG_WINDOWS_CYGWIN/d" $nuttx/.config
    if [ "X$wenv" == "Xmsys" ]; then
      echo "  Select CONFIG_WINDOWS_MSYS=y"
      echo "CONFIG_WINDOWS_MSYS=y" >> $nuttx/.config
      sed -i -e "/CONFIG_WINDOWS_UBUNTU/d" $nuttx/.config
      sed -i -e "/CONFIG_WINDOWS_NATIVE/d" $nuttx/.config
    else
      sed -i -e "/CONFIG_WINDOWS_MSYS/d" $nuttx/.config
      if [ "X$wenv" == "Xubuntu" ]; then
        echo "  Select CONFIG_WINDOWS_UBUNTU=y"
        echo "CONFIG_WINDOWS_UBUNTU=y" >> $nuttx/.config
        sed -i -e "/CONFIG_WINDOWS_NATIVE/d" $nuttx/.config
      else
        echo "  Select CONFIG_WINDOWS_NATIVE=y"
        sed -i -e "/CONFIG_WINDOWS_UBUNTU/d" $nuttx/.config
        echo "CONFIG_WINDOWS_NATIVE=y" $nuttx/.config
      fi
    fi
  fi
fi

sed -i -e "/CONFIG_HOST_OTHER/d" $nuttx/.config

echo "  Refreshing..."
cd $nuttx || { echo "ERROR: failed to cd to $nuttx"; exit 1; }
make clean_context 1>/dev/null 2>&1
if [ "X${debug}" = "Xy" ]; then
  make olddefconfig V=1 || { echo "ERROR: failed to refresh"; exit 1; }
else
  make olddefconfig 1>/dev/null || { echo "ERROR: failed to refresh"; exit 1; }
fi

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
