#!/usr/bin/env bash
# tools/configure.sh
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

WD=`test -d ${0%/*} && cd ${0%/*}; pwd`
TOPDIR="${WD}/.."
MAKECMD="make"
USAGE="

USAGE: ${0} [-E] [-e] [-l|m|c|g|n|B] [L] [-a <app-dir>] <board-name>:<config-name> [make-opts]

Where:
  -E enforces distclean if already configured.
  -e performs distclean if configuration changed.
  -l selects the Linux (l) host environment.
  -m selects the macOS (m) host environment.
  -c selects the Windows host and Cygwin (c) environment.
  -g selects the Windows host and MinGW/MSYS environment.
  -n selects the Windows host and Windows native (n) environment.
  -B selects the *BSD (B) host environment.
  Default: Use host setup in the defconfig file
  Default Windows: Cygwin
  -L  Lists all available configurations.
  -a <app-dir> is the path to the apps/ directory, relative to the nuttx
     directory
  <board-name> is the name of the board in the boards directory
  configs/<config-name> is the name of the board configuration sub-directory
  make-opts directly pass to make

"

# A list of optional files that may be installed

OPTFILES="\
  .gdbinit\
  .cproject\
  .project\
"

# Parse command arguments

unset boardconfig
unset winnative
unset appdir
unset host
unset enforce_distclean
unset distclean

function dumpcfgs
{
  configlist=`find ${TOPDIR}/boards -name defconfig`
  for defconfig in ${configlist}; do
    config=`dirname ${defconfig} | sed -e "s,${TOPDIR}/boards/,,g"`
    boardname=`echo ${config} | cut -d'/' -f3`
    configname=`echo ${config} | cut -d'/' -f5`
    echo "  ${boardname}:${configname}"
  done
}

while [ ! -z "$1" ]; do
  case "$1" in
  -a )
    shift
    appdir=$1
    ;;
  -c | -g | -l | -m )
    winnative=n
    host+=" $1"
    ;;
  -n )
    winnative=y
    host+=" $1"
    ;;
  -B )
    winnative=n
    host+=" $1"
    MAKECMD="gmake"
    ;;
  -E )
    enforce_distclean=y
    ;;
  -e )
    distclean=y
    ;;
  -h )
    echo "$USAGE"
    exit 0
    ;;
  -L )
    dumpcfgs
    exit 0
    ;;
  *)
    boardconfig=$1
    shift
    break
    ;;
  esac
  shift
done

# Sanity checking

if [ -z "${boardconfig}" ]; then
  echo "" 1>&2
  echo "Missing <board/config> argument" 1>&2
  echo "$USAGE" 1>&2
  exit 2
fi

configdir=`echo ${boardconfig} | cut -s -d':' -f2`
if [ -z "${configdir}" ]; then
  boarddir=`echo ${boardconfig} | cut -d'/' -f1`
  configdir=`echo ${boardconfig} | cut -d'/' -f2`
else
  boarddir=`echo ${boardconfig} | cut -d':' -f1`
fi

configpath=${TOPDIR}/boards/*/*/${boarddir}/configs/${configdir}
if [ ! -d ${configpath} ]; then
  # Try direct path used with custom configurations.

  configpath=${TOPDIR}/${boardconfig}
  if [ ! -d ${configpath} ]; then
    configpath=${boardconfig}
    if [ ! -d ${configpath} ]; then
      echo "Directory for ${boardconfig} does not exist." 1>&2
      echo "" 1>&2
      echo "Run tools/configure.sh -L to list available configurations." 1>&2
      echo "$USAGE" 1>&2
      exit 3
    fi
  fi
fi

src_makedefs=${TOPDIR}/boards/*/*/${boarddir}/configs/${configdir}/Make.defs
dest_makedefs="${TOPDIR}/Make.defs"

if [ ! -r ${src_makedefs} ]; then
  src_makedefs=${TOPDIR}/boards/*/*/${boarddir}/scripts/Make.defs

  if [ ! -r ${src_makedefs} ]; then
    src_makedefs=${configpath}/Make.defs
    if [ ! -r ${src_makedefs} ]; then
      src_makedefs=${configpath}/../../scripts/Make.defs

      if [ ! -r ${src_makedefs} ]; then
        echo "File Make.defs could not be found"
        exit 4
      fi
    fi
  fi
fi

src_config=${configpath}/defconfig
dest_config="${TOPDIR}/.config"
backup_config="${TOPDIR}/defconfig"

if [ ! -r ${src_config} ]; then
  echo "File ${src_config} does not exist"
  exit 5
fi

if [ -r ${dest_config} ]; then
  if [ "X${enforce_distclean}" = "Xy" ]; then
    ${MAKECMD} -C ${TOPDIR} distclean
  else
    if cmp -s ${src_config} ${backup_config}; then
      echo "No configuration change."
      exit 0
    fi

    if [ "X${distclean}" = "Xy" ]; then
      ${MAKECMD} -C ${TOPDIR} distclean
    else
      echo "Already configured!"
      echo "Please 'make distclean' and try again."
      exit 6
    fi
  fi
fi

# Extract values needed from the defconfig file.  We need:
# (1) The CONFIG_WINDOWS_NATIVE setting to know it this is target for a
#     native Windows
# (2) The CONFIG_APPS_DIR setting to see if there is a configured location for the
#     application directory.  This can be overridden from the command line.

# If we are going to some host other than windows native or to a windows
# native host, then don't even check what is in the defconfig file.

oldnative=`grep CONFIG_WINDOWS_NATIVE= ${src_config} | cut -d'=' -f2`
if [ -z "${oldnative}" ]; then
  oldnative=n
fi
if [ -z "${winnative}" ]; then
  winnative=$oldnative
fi

# If no application directory was provided on the command line and we are
# switching between a windows native host and some other host then ignore the
# path to the apps/ directory in the defconfig file.  It will most certainly
# not be in a usable form.

defappdir=y
if [ -z "${appdir}" -a "X$oldnative" = "X$winnative" ]; then
  quoted=`grep "^CONFIG_APPS_DIR=" ${src_config} | cut -d'=' -f2`
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
winappdir=`echo "${appdir}" | sed -e 's/\\//\\\\\\\/g'`

# If appsdir was provided (or discovered) then make sure that the apps/
# directory exists

if [ ! -z "${appdir}" -a ! -d "${TOPDIR}/${posappdir}" ]; then
  echo "Directory \"${TOPDIR}/${posappdir}\" does not exist"
  exit 7
fi

# Okay... Everything looks good.  Setup the configuration

echo "  Copy files"
install -m 644 ${src_makedefs} "${dest_makedefs}" || \
  { echo "Failed to copy ${src_makedefs}" ; exit 8 ; }
install -m 644 ${src_config} "${dest_config}" || \
  { echo "Failed to copy ${src_config}" ; exit 9 ; }
install -m 644 ${src_config} "${backup_config}" || \
  { echo "Failed to backup ${src_config}" ; exit 10 ; }

# Install any optional files

for opt in ${OPTFILES}; do
  test -f ${configpath}/${opt} && install ${configpath}/${opt} "${TOPDIR}/"
done

# If we did not use the CONFIG_APPS_DIR that was in the defconfig config file,
# then append the correct application information to the tail of the .config
# file

if [ "X${defappdir}" = "Xy" ]; then
  # In-place edit can mess up permissions on Windows
  # sed -i.bak -e "/^CONFIG_APPS_DIR/d" "${dest_config}"
  sed -e "/^CONFIG_APPS_DIR/d" "${dest_config}" > "${dest_config}-temp"
  mv "${dest_config}-temp" "${dest_config}"

  if [ "X${winnative}" = "Xy" ]; then
    echo "CONFIG_APPS_DIR=\"$winappdir\"" >> "${dest_config}"
  else
    echo "CONFIG_APPS_DIR=\"$posappdir\"" >> "${dest_config}"
  fi
fi

# The saved defconfig files are all in compressed format and must be
# reconstitued before they can be used.

${TOPDIR}/tools/sethost.sh $host $*
