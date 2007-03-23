#!/bin/sh
# configure.sh
#
#   Copyright (C) 2007 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
# 3. Neither the name Gregory Nutt nor the names of its contributors may be
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
#set -x

BOARD=$1
WD=`pwd`
TOPDIR=${WD}/..

function show_usage ()
{
  echo "${0} <board-name>"
  exit 1
}

if [ "${BOARD}X" = "X" ]; then
  echo "Missing argument"
  show_usage
fi

BOARDDIR=${TOPDIR}/configs/${BOARD}
if [ ! -d ${BOARDDIR} ]; then
  echo "Directory ${BOARDDIR} does not exist"
  show_usage
fi

if [ ! -r ${BOARDDIR}/Make.defs ]; then
  echo "File ${BOARDDIR}/Make.defs does not exist"
  exit 1
fi

if [ ! -r ${BOARDDIR}/setenv.sh ]; then
  echo "File ${BOARDDIR}/setenv.sh does not exist"
  exit 1
fi

if [ ! -r ${BOARDDIR}/defconfig ]; then
  echo "File ${BOARDDIR}/defconfig does not exist"
  exit 1
fi

cp -f ${BOARDDIR}/Make.defs ${TOPDIR}/. || \
  { echo "Failed to copy ${BOARDDIR}/Make.defs" ; exit 1 ; }
cp -f ${BOARDDIR}/setenv.sh ${TOPDIR}/. || \
  { echo "Failed to copy ${BOARDDIR}/setenv.sh" ; exit 1 ; }
cp -f ${BOARDDIR}/defconfig ${TOPDIR}/.config || \
  { echo "Failed to copy ${BOARDDIR}/defconfig" ; exit 1 ; }

