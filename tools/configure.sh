#!/bin/bash
# tools/configure.sh
#
#   Copyright (C) 2007, 2008, 2011, 2015, 2017-2018 Gregory Nutt. All rights
#     reserved.
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

WD=`test -d ${0%/*} && cd ${0%/*}; pwd`
TOPDIR="${WD}/.."
USAGE="

USAGE: ${0} [-d] [-l|m|c|u|g|n] [-a <app-dir>] <board-name>/<config-name>

Where:
  -l selects the Linux (l) host environment.
  -m selects the macOS (m) host environment.
  -c selects the Windows host and Cygwin (c) environment.
  -u selects the Windows host and Ubuntu under Windows 10 (u) environment.
  -g selects the Windows host and MinGW/MSYS environment.
  -n selects the Windows host and Windows native (n) environment.
  Default: Use host setup in the defconfig file
  Default Windows: Cygwin
  <board-name> is the name of the board in the configs directory
  <config-name> is the name of the board configuration sub-directory
  <app-dir> is the path to the apps/ directory, relative to the nuttx
     directory

"

# A list of optional files that may be installed

OPTFILES="\
  .gdbinit\
  .cproject\
  .project\
"

# Parse command arguments

unset boardconfig
unset appdir
unset host
unset wenv

while [ ! -z "$1" ]; do
  case "$1" in
    -a )
      shift
      appdir=$1
      ;;
    -c )
      host=windows
      wenv=cygwin
      ;;
    -d )
      set -x
      ;;
    -g )
      host=windows
      wenv=msys
      ;;
    -h )
      echo "$USAGE"
      exit 0
      ;;
    -l )
      host=linux
      ;;
    -m )
      host=macos
      ;;
    -n )
      host=windows
      wenv=native
      ;;
    -u )
      host=windows
      wenv=ubuntu
      ;;
    *)
      if [ ! -z "${boardconfig}" ]; then
        echo ""
        echo "<board/config> defined twice"
        echo "$USAGE"
        exit 1
      fi
      boardconfig=$1
      ;;
  esac
  shift
done

# Sanity checking

if [ -z "${boardconfig}" ]; then
  echo ""
  echo "Missing <board/config> argument"
  echo "$USAGE"
  exit 2
fi

configpath=${TOPDIR}/configs/${boardconfig}
if [ ! -d "${configpath}" ]; then
  # Try direct path for convenience.

  configpath=${TOPDIR}/${boardconfig}
  if [ ! -d "${configpath}" ]; then
    echo "Directory ${configpath} does not exist.  Options are:"
    echo ""
    echo "Select one of the following options for <board-name>:"
    configlist=`find ${TOPDIR}/configs -name defconfig`
    for defconfig in ${configlist}; do
      config=`dirname ${defconfig} | sed -e "s,${TOPDIR}/configs/,,g"`
      echo "  ${config}"
    done
    echo ""
    echo "$USAGE"
    exit 3
  fi
fi

src_makedefs="${configpath}/Make.defs"
dest_makedefs="${TOPDIR}/Make.defs"

if [ ! -r "${src_makedefs}" ]; then
  boardpath=`dirname $configpath`
  src_makedefs="${boardpath}/scripts/Make.defs"

  if [ ! -r "${src_makedefs}" ]; then
    echo "File Make.defs could not be found"
    exit 4
  fi
fi

src_config="${configpath}/defconfig"
dest_config="${TOPDIR}/.config"

if [ ! -r "${src_config}" ]; then
  echo "File \"${src_config}\" does not exist"
  exit 6
fi

if [ -r ${dest_config} ]; then
  echo "Already configured!"
  echo "Do 'make distclean' and try again."
  exit 7
fi

# Extract values needed from the defconfig file.  We need:
# (1) The CONFIG_WINDOWS_NATIVE setting to know it this is target for a
#     native Windows
# (2) The CONFIG_APPS_DIR setting to see if there is a configured location for the
#     application directory.  This can be overridden from the command line.

# If we are going to some host other then windows native or to a windows
# native host, then don't even check what is in the defconfig file.

oldnative=`grep CONFIG_WINDOWS_NATIVE= "${src_config}" | cut -d'=' -f2`
if [ "X$host" != "Xwindows" -o "X$wenv" != "Xnative" ]; then
  unset winnative
else
  if [ "X$host" == "Xwindows" -a "X$wenv" == "Xnative" ]; then
    winnative=y
  else
    winnative=$oldnative
  fi
fi

# If no application directory was provided on the command line and we are
# switching between a windows native host and some other host then ignore the
# path to the apps/ directory in the defconfig file.  It will most certainly
# not be in a usable form.

defappdir=y
if [ -z "${appdir}" -a "X$oldnative" = "$winnative" ]; then
  quoted=`grep "^CONFIG_APPS_DIR=" "${src_config}" | cut -d'=' -f2`
  if [ ! -z "${quoted}" ]; then
    appdir=`echo ${quoted} | sed -e "s/\"//g"`
    defappdir=n
  fi
fi

# Check for the apps/ directory in the usual place if appdir was not provided

if [ -z "${appdir}" ]; then

  # Check for a version file

  unset CONFIG_VERSION_STRING
  if [ -x "${TOPDIR}/.version" ]; then
    . "${TOPDIR}/.version"
  fi

  # Check for an unversioned apps/ directory

  if [ -d "${TOPDIR}/../apps" ]; then
    appdir="../apps"
  else
    # Check for a versioned apps/ directory

    if [ -d "${TOPDIR}/../apps-${CONFIG_VERSION_STRING}" ]; then
      appdir="../apps-${CONFIG_VERSION_STRING}"
    fi
  fi
fi

# For checking the apps dir path, we need a POSIX version of the relative path.

posappdir=`echo "${appdir}" | sed -e 's/\\\\/\\//g'`
winappdir=`echo "${appdir}" | sed -e 's/\\//\\\\/g'`

# If appsdir was provided (or discovered) then make sure that the apps/
# directory exists

if [ ! -z "${appdir}" -a ! -d "${TOPDIR}/${posappdir}" ]; then
  echo "Directory \"${TOPDIR}/${posappdir}\" does not exist"
  exit 7
fi

# Okay... Everything looks good.  Setup the configuration

echo "  Copy files"
install -m 644 "${src_makedefs}" "${dest_makedefs}" || \
  { echo "Failed to copy \"${src_makedefs}\"" ; exit 7 ; }
install -m 644 "${src_config}" "${dest_config}" || \
  { echo "Failed to copy \"${src_config}\"" ; exit 9 ; }

# Install any optional files

for opt in ${OPTFILES}; do
  test -f "${configpath}/${opt}" && install "${configpath}/${opt}" "${TOPDIR}/"
done

# If we did not use the CONFIG_APPS_DIR that was in the defconfig config file,
# then append the correct application information to the tail of the .config
# file

if [ "X${defappdir}" = "Xy" ]; then
  # In-place edit can mess up permissions on Windows
  # sed -i -e "/^CONFIG_APPS_DIR/d" "${dest_config}"
  sed -e "/^CONFIG_APPS_DIR/d" "${dest_config}" > "${dest_config}-temp"
  mv "${dest_config}-temp" "${dest_config}"

  if [ "X${winnative}" = "Xy" ]; then
    echo "CONFIG_APPS_DIR=\"$winappdir\"" >> "${dest_config}"
  else
    echo "CONFIG_APPS_DIR=\"$posappdir\"" >> "${dest_config}"
  fi
fi

if [ ! -z "$host" ]; then
  sed -i -e "/CONFIG_HOST_LINUX/d" ${dest_config}
  sed -i -e "/CONFIG_HOST_WINDOWS/d" ${dest_config}
  sed -i -e "/CONFIG_HOST_MACOS/d" ${dest_config}
  sed -i -e "/CONFIG_HOST_OTHER/d" ${dest_config}
  sed -i -e "/CONFIG_WINDOWS_NATIVE/d" ${dest_config}
  sed -i -e "/CONFIG_WINDOWS_CYGWIN/d" ${dest_config}
  sed -i -e "/CONFIG_WINDOWS_MSYS/d" ${dest_config}
  sed -i -e "/CONFIG_WINDOWS_UBUNTU/d" ${dest_config}
  sed -i -e "/CONFIG_WINDOWS_OTHER/d" ${dest_config}
  sed -i -e "/CONFIG_SIM_X8664_MICROSOFT/d" ${dest_config}
  sed -i -e "/CONFIG_SIM_X8664_SYSTEMV/d" ${dest_config}
  sed -i -e "/CONFIG_SIM_M32/d" ${dest_config}

  case "$host" in
    "linux")
      echo "  Select CONFIG_HOST_LINUX=y"
      echo "CONFIG_HOST_LINUX=y" >> "${dest_config}"
      echo "CONFIG_SIM_X8664_SYSTEMV=y" >> "${dest_config}"
      ;;

    "macos")
      echo "  Select CONFIG_HOST_MACOS=y"
      echo "CONFIG_HOST_MACOS=y" >> "${dest_config}"
      ;;

    "windows")
      echo "  Select CONFIG_HOST_WINDOWS=y"
      echo "CONFIG_HOST_WINDOWS=y" >> "${dest_config}"
      echo "CONFIG_SIM_X8664_MICROSOFT=y" >> "${dest_config}"

      case "$wenv" in
          "cygwin")
            echo "  Select CONFIG_WINDOWS_CYGWIN=y"
            echo "CONFIG_WINDOWS_CYGWIN=y" >> "${dest_config}"
            ;;

          "msys")
            echo "  Select CONFIG_WINDOWS_MSYS=y"
            echo "CONFIG_WINDOWS_MSYS=y" >> "${dest_config}"
            ;;

          "ubuntu")
            echo "  Select CONFIG_WINDOWS_UBUNTU=y"
            echo "CONFIG_WINDOWS_UBUNTU=y" >> "${dest_config}"
            ;;

          *)
            echo "  Select CONFIG_WINDOWS_NATIVE=y"
            echo "CONFIG_WINDOWS_NATIVE=y" >> "${dest_config}"
            ;;
      esac
  esac
fi

# The saved defconfig files are all in compressed format and must be
# reconstitued before they can be used.

echo "  Refreshing..."
cd ${TOPDIR} || { echo "Failed to cd to ${TOPDIR}"; exit 1; }

MAKE_BIN=make
if [ ! -z `which gmake 2>/dev/null` ]; then
  MAKE_BIN=gmake
fi

${MAKE_BIN} olddefconfig 1>/dev/null

