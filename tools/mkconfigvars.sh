#!/usr/bin/env bash
# tools/mkconfivars.sh
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

USAGE="USAGE: $0 [-d|h] [-v <major.minor.patch>]"
ADVICE="Try '$0 -h' for more information"

unset VERSION

while [ ! -z "$1" ]; do
  case $1 in
  -v )
    shift
    VERSION=$1
    ;;
  -d )
    set -x
    ;;
  -h )
    echo "$0 is a tool for generation of configuration variable documentation"
    echo ""
    echo $USAGE
    echo ""
    echo "Where:"
    echo "  -v <major.minor.patch>"
    echo "     The NuttX version number expressed as a major, minor and patch number separated"
    echo "     by a period"
    echo "  -d"
    echo "     Enable script debug"
    echo "  -h"
    echo "     show this help message and exit"
    exit 0
    ;;
  * )
    echo "Unrecognized option: ${1}"
    echo $USAGE
    echo $ADVICE
    exit 1
    ;;
  esac
  shift
done

# Find the directory we were executed from and were we expect to
# see the directories to tar up

MYNAME=`basename $0`
KCONFIG2HTML_TARGET=kconfig2html
KCONFIG2HTML1=tools/kconfig2html
KCONFIG2HTML2=tools/kconfig2html.exe
KCONFIG2MAKEFILE=Makefile.host
KCONFIG2MAKEDIR=tools
HTMLFILE=Documentation/NuttXConfigVariables.html
BKUPFILE=Documentation/NuttXConfigVariables.bkp

if [ -x ./${MYNAME} ] ; then
  cd .. || { echo "ERROR: cd .. failed" ; exit 1 ; }
fi

if [ ! -x tools/${MYNAME} ] ; then
  echo "ERROR:  This file must be executed from the top-level NuttX directory: $PWD"
  exit 1
fi

WD=${PWD}

# Find the application directory

if [ -d ../apps ]; then
  APPSDIR="../apps"
else
  if [ -d "../apps-${VERSION}" ]; then
    APPSDIR="../apps-${VERSION}"
  else
    echo "ERROR:  Cannot find the application directory"
    exit 1
  fi
fi

# If the kconfig2html executable does not exist, then build it

if [ -x ${KCONFIG2HTML1} ]; then
  KCONFIG2HTML=${KCONFIG2HTML1}
else
  if [ -x ${KCONFIG2HTML2} ]; then
    KCONFIG2HTML=${KCONFIG2HTML2}
  else
    make -C ${KCONFIG2MAKEDIR} -f ${KCONFIG2MAKEFILE} ${KCONFIG2HTML_TARGET} 1>/dev/null || \
      { echo "ERROR: make ${KCONFIG2HTML1} failed" ; exit 1 ; }
  fi
fi

if [ -x ${KCONFIG2HTML1} ]; then
  KCONFIG2HTML=${KCONFIG2HTML1}
else
  if [ -x ${KCONFIG2HTML2} ]; then
    KCONFIG2HTML=${KCONFIG2HTML2}
  else
    echo "ERROR: Failed to create  ${KCONFIG2HTML1}"
    exit 1
  fi
fi

# Keep a backup of the previous HTML file.  This is usefully primarily
# for testing the effects of changes.

if [ -e "${HTMLFILE}" ]; then
  rm -f ${BKUPFILE} || { echo "ERROR: Failed to remove ${BKUPFILE}" ; exit 1 ; }
  mv ${HTMLFILE} ${BKUPFILE} || { echo "ERROR: Failed to move ${HTMLFILE}" ; exit 1 ; }
fi

# Now re-create the configuration variable document

${KCONFIG2HTML} -a "${APPSDIR}" -o ${HTMLFILE}
