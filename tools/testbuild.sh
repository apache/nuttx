#!/usr/bin/env bash
# tools/testbuild.sh
#
#   Copyright (C) 2016-2020 Gregory Nutt. All rights reserved.
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

WD=$(cd $(dirname $0) && cd .. && pwd)
nuttx=$WD/../nuttx

progname=$0
fail=0
APPSDIR=$WD/../apps
MAKE_FLAGS=-k
EXTRA_FLAGS="EXTRAFLAGS="
MAKE=make
unset testfile
unset HOPTION
unset JOPTION
PRINTLISTONLY=0
GITCLEAN=0

function showusage {
  echo ""
  echo "USAGE: $progname [-l|m|c|g|n] [-d] [-e <extraflags>] [-x] [-j <ncpus>] [-a <appsdir>] [-t <topdir>] [-p] [-G] <testlist-file>"
  echo "       $progname -h"
  echo ""
  echo "Where:"
  echo "  -l|m|c|g|n selects Linux (l), macOS (m), Cygwin (c),"
  echo "     MSYS/MSYS2 (g) or Windows native (n). Default Linux"
  echo "  -d enables script debug output"
  echo "  -e pass extra c/c++ flags such as -Wno-cpp via make command line"
  echo "  -x exit on build failures"
  echo "  -j <ncpus> passed on to make.  Default:  No -j make option."
  echo "  -a <appsdir> provides the relative path to the apps/ directory.  Default ../apps"
  echo "  -t <topdir> provides the absolute path to top nuttx/ directory.  Default $PWD/../nuttx"
  echo "  -p only print the list of configs without running any builds"
  echo "  -G Use \"git clean -xfdq\" instead of \"make distclean\" to clean the tree."
  echo "     This option may speed up the builds. However, note that:"
  echo "       * This assumes that your trees are git based."
  echo "       * This assumes that only nuttx and apps repos need to be cleaned."
  echo "       * If the tree has files not managed by git, they will be removed"
  echo "         as well."
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
  -l | -m | -c | -g | -n )
    HOPTION+=" $1"
    ;;
  -d )
    set -x
    ;;
  -e )
    shift
    EXTRA_FLAGS+="$1"
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
  -p )
    PRINTLISTONLY=1
    ;;
  -G )
    GITCLEAN=1
    ;;
  -h )
    showusage
    ;;
  * )
    testfile="$1"
    shift
    break
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

if [ ! -d $APPSDIR ]; then
  echo "ERROR: No directory found at $APPSDIR"
  exit 1
fi

export APPSDIR

testlist=`grep -v -E "^(-|#)" $testfile || true`
blacklist=`grep "^-" $testfile || true`

cd $nuttx || { echo "ERROR: failed to CD to $nuttx"; exit 1; }

function makefunc {
  if ! ${MAKE} ${MAKE_FLAGS} "${EXTRA_FLAGS}" ${JOPTION} $@ 1>/dev/null; then
    fail=1
  fi

  return $fail
}

# Clean up after the last build

function distclean {
  echo "  Cleaning..."
  if [ -f .config ]; then
    if [ ${GITCLEAN} -eq 1 ]; then
      git -C $nuttx clean -xfdq
      git -C $APPSDIR clean -xfdq
    else
      makefunc distclean

      # Remove .version manually because this file is shipped with
      # the release package and then distclean has to keep it

      rm -f .version

      # Ensure nuttx and apps directory in clean state even with --ignored

      if [ -d $nuttx/.git ] || [ -d $APPSDIR/.git ]; then
        if [[ -n $(git -C $nuttx status --ignored -s) ]]; then
          git -C $nuttx status --ignored
          fail=1
        fi
        if [[ -n $(git -C $APPSDIR status --ignored -s) ]]; then
          git -C $APPSDIR status --ignored
          fail=1
        fi
      fi
    fi
  fi

  return $fail
}

# Configure for the next build

function configure {
  echo "  Configuring..."
  if ! ./tools/configure.sh ${HOPTION} $config ${JOPTION} 1>/dev/null; then
    fail=1
  fi

  if [ "X$toolchain" != "X" ]; then
    setting=`grep _TOOLCHAIN_ $nuttx/.config | grep -v CONFIG_ARCH_TOOLCHAIN_* | grep =y`
    varname=`echo $setting | cut -d'=' -f1`
    if [ ! -z "$varname" ]; then
      echo "  Disabling $varname"
      sed -i -e "/$varname/d" $nuttx/.config
    fi

    echo "  Enabling $toolchain"
    sed -i -e "/$toolchain/d" $nuttx/.config
    echo "$toolchain=y" >> $nuttx/.config

    if [ "X$sizet" == "Xuint" ]; then
      echo "  Disabling CONFIG_ARCH_SIZET_LONG"
      sed -i -e "/CONFIG_ARCH_SIZET_LONG/d" $nuttx/.config
    elif [ "X$sizet" == "Xulong" ]; then
      echo "  Enabling CONFIG_ARCH_SIZET_LONG"
      sed -i -e "\$aCONFIG_ARCH_SIZET_LONG=y" $nuttx/.config
    fi

    makefunc olddefconfig
  fi

  return $fail
}

# Perform the next build

function build {
  echo "  Building NuttX..."
  makefunc

  # Ensure defconfig in the canonical form

  if ! ./tools/refresh.sh --silent $config; then
    fail=1
  fi

  # Ensure nuttx and apps directory in clean state

  if [ -d $nuttx/.git ] || [ -d $APPSDIR/.git ]; then
    if [[ -n $(git -C $nuttx status -s) ]]; then
      git -C $nuttx status
      fail=1
    fi
    if [[ -n $(git -C $APPSDIR status -s) ]]; then
      git -C $APPSDIR status
      fail=1
    fi
  fi

  return $fail
}

# Coordinate the steps for the next build test

function dotest {
  echo "===================================================================================="
  config=`echo $1 | cut -d',' -f1`
  re="-${config/\//:}[[:space:]]"
  if [[ "${blacklist} " =~ $re ]]; then
    echo "Skipping: $1"
  else
    echo "Configuration/Tool: $1"
    if [ ${PRINTLISTONLY} -eq 1 ]; then
      return
    fi

    # Parse the next line

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

    unset toolchain
    unset sizet
    if [ "X$config" != "X$1" ]; then
      toolchain=`echo $1 | cut -d',' -f2`
      if [ -z "$toolchain" ]; then
        echo "  Warning: no tool configuration"
      fi
      archsizet=`echo $line | cut -d',' -f3`
      if [ "X$archsizet" == "XCONFIG_ARCH_SIZET_LONG" ]; then
        sizet=ulong
      elif [ "X$archsizet" == "X-CONFIG_ARCH_SIZET_LONG" ]; then
        sizet=uint
      fi
    fi

    # Perform the build test

    echo "------------------------------------------------------------------------------------"
    distclean
    configure
    build
  fi
}

# Perform the build test for each entry in the test list file

for line in $testlist; do
  firstch=${line:0:1}
  if [ "X$firstch" == "X/" ]; then
    dir=`echo $line | cut -d',' -f1`
    list=`find boards$dir -name defconfig | cut -d'/' -f4,6`
    for i in ${list}; do
      dotest $i${line/$dir/}
    done
  else
    dotest $line
  fi
done

echo "===================================================================================="

exit $fail
