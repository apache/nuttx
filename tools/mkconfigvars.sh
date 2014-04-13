#!/bin/sh
# mkconfivars.sh
#
#   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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

USAGE="USAGE: $0 [-d|h] [-v <major.minor>]"
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
        echo "  -v <major.minor>"
        echo "     The NuttX version number expressed as a major and minor number separated"
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
    make -C ${KCONFIG2MAKEDIR} -f ${KCONFIG2MAKEFILE} ${KCONFIG2HTML_TARGET} || \
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
