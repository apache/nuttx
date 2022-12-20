#!/usr/bin/env bash
# tools/version.sh
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
    echo "    number separated by a period"
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
  VERSION=`git -C ${WD} describe --match "nuttx-*" 2>/dev/null | tail -1 | cut -d'-' -f2`

  # If the VERSION does not match X.Y.Z, retrieve version from the tag

  if [[ ! ${VERSION} =~ ([0-9]+)\.([0-9]+)\.([0-9]+) ]] ; then
    VERSION=`git -C ${WD} -c 'versionsort.suffix=-' tag --sort=v:refname 2>/dev/null | grep -E "nuttx-[0-9]+\.[0-9]+\.[0-9]+" | tail -1 | cut -d'-' -f2-`
  fi

fi

# Make sure we know what is going on

if [ -z ${VERSION} ] ; then
  echo "Missing versioning information. Using the dummy value. (0.0.0)"
  VERSION="0.0.0"
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
PATCH=`echo ${VERSION} | grep -Eo "[0-9]+\.[0-9]+\.[0-9]+" | cut -d'.' -f3`

# Get GIT information (if not provided on the command line)

if [ -z "${BUILD}" ]; then
  BUILD=`git -C ${WD} log --oneline -1 2>/dev/null | cut -d' ' -f1`
  if [ -z "${BUILD}" ]; then
    echo "GIT version information is not available"
  fi
  if [ -n "`git -C ${WD} diff-index --name-only HEAD 2>/dev/null | head -1`" ]; then
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
