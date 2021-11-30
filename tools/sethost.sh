#!/usr/bin/env bash
# tools/sethost.sh
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

set -e

progname=$0
host=
wenv=
MAKECMD="make"

function showusage {
  echo ""
  echo "USAGE: $progname [-l|m|c|g|n|B] [make-opts]"
  echo "       $progname -h"
  echo ""
  echo "Where:"
  echo "  -l|m|c|g|n|B selects Linux (l), macOS (m), Cygwin (c), BSD (B),"
  echo "     MSYS/MSYS2 (g) or Windows native (n). Default Linux"
  echo "  make-opts directly pass to make"
  echo "  -h will show this help test and terminate"
  exit 1
}

# Parse command line

while [ ! -z "$1" ]; do
  case $1 in
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
  -m )
    host=macos
    ;;
  -n )
    host=windows
    wenv=native
    ;;
  -B )
    host=bsd
    MAKECMD="gmake"
    ;;
  -h )
    showusage
    ;;
  * )
    break
    ;;
  esac
  shift
done

# If the host was not explicitly given, try to guess.
# Examples of "uname -s" outputs:
#   macOS: Darwin
#   Cygwin: CYGWIN_NT-10.0-WOW
#   Linux: Linux
#   MSYS: MINGW32_NT-6.2
#   BSD: FreeBSD, OpenBSD, NetBSD, *BSD

if [ -z "$host" ]; then
  case $(uname -s) in
    Darwin)
      host=macos
      ;;
    *BSD)
      host=bsd
      MAKECMD="gmake"
      ;;
    CYGWIN*)
      host=windows
      wenv=cygwin
      ;;
    MINGW32*)
      host=windows
      wenv=msys
      ;;
    *)
      # Assume linux as a fallback

      host=linux
      ;;
  esac
fi

WD=`test -d ${0%/*} && cd ${0%/*}; pwd`
cd $WD

if [ -x sethost.sh ]; then
  cd ..
fi

if [ -x tools/sethost.sh ]; then
  nuttx=$PWD
else
  echo "This script must be executed in nuttx/ or nuttx/tools directories"
  exit 1
fi

if [ ! -r $nuttx/.config ]; then
  echo "There is no .config at $nuttx"
  exit 1
fi

if [ ! -r $nuttx/Make.defs ]; then
  echo "ERROR: No readable Make.defs file exists at $nuttx"
  exit 1
fi

# Modify the configuration

if [ "X$host" == "Xlinux" -o "X$host" == "Xmacos" -o "X$host" == "Xbsd" ]; then

  # Disable Windows (to suppress warnings from Window Environment selections)

  kconfig-tweak --file $nuttx/.config --disable CONFIG_HOST_WINDOWS

  # Enable Linux or macOS or BSD

  if [ "X$host" == "Xlinux" ]; then
    echo "  Select CONFIG_HOST_LINUX=y"
    kconfig-tweak --file $nuttx/.config --disable CONFIG_HOST_MACOS
    kconfig-tweak --file $nuttx/.config --disable CONFIG_HOST_BSD
    kconfig-tweak --file $nuttx/.config --enable CONFIG_HOST_LINUX

  elif [ "X$host" == "Xbsd" ]; then
    echo "  Select CONFIG_HOST_BSD=y"
    kconfig-tweak --file $nuttx/.config --disable CONFIG_HOST_MACOS
    kconfig-tweak --file $nuttx/.config --disable CONFIG_HOST_LINUX
    kconfig-tweak --file $nuttx/.config --enable CONFIG_HOST_BSD

  else
    echo "  Select CONFIG_HOST_MACOS=y"
    kconfig-tweak --file $nuttx/.config --disable CONFIG_HOST_LINUX
    kconfig-tweak --file $nuttx/.config --disable CONFIG_HOST_BSD
    kconfig-tweak --file $nuttx/.config --enable CONFIG_HOST_MACOS
  fi

  # Enable the System V ABI

  kconfig-tweak --file $nuttx/.config --enable CONFIG_SIM_X8664_SYSTEMV
else
  echo "  Select CONFIG_HOST_WINDOWS=y"

  kconfig-tweak --file $nuttx/.config --disable CONFIG_HOST_LINUX
  kconfig-tweak --file $nuttx/.config --disable CONFIG_HOST_MACOS
  kconfig-tweak --file $nuttx/.config --disable CONFIG_HOST_BSD

  # Enable Windows and the Microsoft ABI

  kconfig-tweak --file $nuttx/.config --enable CONFIG_HOST_WINDOWS
  kconfig-tweak --file $nuttx/.config --enable CONFIG_SIM_X8664_MICROSOFT

  # Enable Windows environment

  if [ "X$wenv" == "Xcygwin" ]; then
    echo "  Select CONFIG_WINDOWS_CYGWIN=y"
    kconfig-tweak --file $nuttx/.config --enable CONFIG_WINDOWS_CYGWIN
  else
    if [ "X$wenv" == "Xmsys" ]; then
      echo "  Select CONFIG_WINDOWS_MSYS=y"
      kconfig-tweak --file $nuttx/.config --enable CONFIG_WINDOWS_MSYS
    else
      echo "  Select CONFIG_WINDOWS_NATIVE=y"
      kconfig-tweak --file $nuttx/.config --enable CONFIG_WINDOWS_NATIVE
    fi
  fi
fi

echo "  Refreshing..."

${MAKECMD} olddefconfig $* || { echo "ERROR: failed to refresh"; exit 1; }
