#!/usr/bin/env bash
# tools/version.sh
#
#   Copyright (C) 2011, 2019 Gregory Nutt. All rights reserved.
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

WD=$(dirname $0)

# Get command line parameters

USAGE="USAGE: $0 [-d|-h] [-b <build>] [-v <major.minor.patch>] <outfile-path>"
ADVICE="Try '$0 -h' for more information"

unset VERSION
unset BUILD
unset OUTFILE

while [ ! -z "$1" ]; do
  case $1 in
  -b )
    shift
    BUILD=$1
    ;;
  -d )
    set -x
    ;;
  -v )
    shift
    VERSION=$1
    ;;
  -h )
    echo "$0 is a tool for generation of proper version files for the NuttX build"
    echo ""
    echo $USAGE
    echo ""
    echo "Where:"
    echo "  -b <build>"
    echo "    Use this build identification string.  Default: use GIT build ID"
    echo "    NOTE: GIT build information may not be available in a snapshot"
    echo "  -d"
    echo "    Enable script debug"
    echo "  -h"
    echo "    show this help message and exit"
    echo "  -v <major.minor.patch>"
    echo "    The NuttX version number expressed as a major, minor and patch"
    echo "    number seperated by a period"
    echo "   <outfile-path>"
    echo "    The full path to the version file to be created"
    exit 0
    ;;
  * )
    break
    ;;
  esac
  shift
done

OUTFILE=$1

if [ -z ${VERSION} ] ; then
  VERSION=`git -C ${WD} tag --sort=taggerdate | tail -1 | cut -d'-' -f2`

  # Earlier tags used the format "major.minor", append a "0" for a patch.

  if [[ ${VERSION} =~ ^([0-9]+[\.][0-9]+)$ ]] ; then
    VERSION=${VERSION}.0
  fi
fi

# Make sure we know what is going on

if [ -z ${VERSION} ] ; then
  echo "Missing versioning information"
  echo $USAGE
  echo $ADVICE
  exit 1
fi

if [ -z ${OUTFILE} ] ; then
  echo "Missing path to the output file"
  echo $USAGE
  echo $ADVICE
  exit 2
fi

# Get the major, minor and patch version numbers

MAJOR=`echo ${VERSION} | cut -d'.' -f1`
if [ "X${MAJOR}" = "X${VERSION}" ]; then
  echo "Missing minor version number"
  echo $USAGE
  echo $ADVICE
  exit 3
fi

MINOR=`echo ${VERSION} | cut -d'.' -f2`
if [ "X${MAJOR}.${MINOR}" = "X${VERSION}" ]; then
  echo "Missing patch version number"
  echo $USAGE
  echo $ADVICE
  exit 4
fi
PATCH=`echo ${VERSION} | cut -d'.' -f3`

# Get GIT information (if not provided on the command line)

if [ -z "${BUILD}" ]; then
  BUILD=`git -C ${WD} log --oneline -1 | cut -d' ' -f1 2>/dev/null`
  if [ -z "${BUILD}" ]; then
    echo "GIT version information is not available"
    exit 5
  fi
  if [ -n "`git -C ${WD} diff-index --name-only HEAD | head -1`" ]; then
    BUILD=${BUILD}-dirty
  fi
fi

# Write a version file into the NuttX directory.  The syntax of file is such that it
# may be sourced by a bash script or included by a Makefile.

echo "#!/bin/bash" >${OUTFILE}
echo "" >>${OUTFILE}
echo "CONFIG_VERSION_STRING=\"${VERSION}\"" >>${OUTFILE}
echo "CONFIG_VERSION_MAJOR=${MAJOR}" >>${OUTFILE}
echo "CONFIG_VERSION_MINOR=${MINOR}" >>${OUTFILE}
echo "CONFIG_VERSION_PATCH=${PATCH}" >>${OUTFILE}
echo "CONFIG_VERSION_BUILD=\"${BUILD}\"" >>${OUTFILE}
