#!/usr/bin/env bash
# testbuild.sh
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
TOOLSDIR=$nuttx/tools
UNLINK=$TOOLSDIR/unlink.sh

progname=$0
host=linux
wenv=cygwin
sizet=uint
APPSDIR=../apps
MAKE_FLAGS=-i
MAKE=make
unset testfile
unset JOPTION

function showusage {
    echo ""
    echo "USAGE: $progname [-w|l] [-c|u|n] [-s] [-d] [-x] [-j <ncpus>] [-a <apps-dir>] [-t <nuttx-dir><testlist-file>"
    echo "       $progname -h"
    echo ""
    echo "Where:"
    echo "  -w|l selects Windows (w) or Linux (l).  Default: Linux"
    echo "  -c|u|n selects Windows environment option:  Cygwin (c), Ubuntu under"
    echo "     Windows 10 (u), or Windows native (n).  Default Cygwin"
    echo "  -s Use C++ unsigned long size_t in new operator. Default unsigned int"
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
  export "ERROR: No directory found at $APPSDIR"
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

      # Detect the architecture of this board.

      ARCHLIST="arm avr hc mips misoc or1k renesas risc-v sim x86 xtensa z16 z80"
      CHIPLIST="a1x am335x c5471 cxd56xx dm320 efm32 imx6 imxrt kinetis kl lc823450
        lpc17xx_40xx lpc214x lpc2378 lpc31xx lpc43xx lpc54xx max326xx moxart nrf52
        nuc1xx rx65n s32k1xx sam34 sama5 samd2l2 samd5e5 samv7 stm32 stm32f0l0g0 stm32f7 stm32h7
        stm32l4 str71x tiva tms570 xmc4 at32uc3 at90usb atmega mcs92s12ne64 pic32mx
        pic32mz lm32 mor1kx m32262f8 sh7032 gap8 nr5m100 sim qemu esp32 z16f2811
        ez80 z180 z8 z80"

      for arch in ${ARCHLIST}; do
        for chip in ${CHIPLIST}; do
          if [ -f ${nuttx}/boards/${arch}/${chip}/${boarddir}/Kconfig ]; then
            archdir=${arch}
            chipdir=${chip}
          fi
        done
      done

      if [ -z "${archdir}" ]; then
        echo "ERROR:  Architecture of ${boarddir} not found"
        exit 1
      fi

      path=$nuttx/boards/$archdir/$chipdir/$boarddir/configs/$configdir
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
