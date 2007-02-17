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

ARCH=$1
WD=`pwd`
TOPDIR=${WD}/..

function show_usage ()
{
  echo "${0} <arch>"
  exit 1
}

if [ "${ARCH}X" = "X" ]; then
  echo "Missing argument"
  show_usage
fi

ARCHDIR=${TOPDIR}/arch/${ARCH}
if [ ! -d ${ARCHDIR} ]; then
  echo "Directory ${ARCHDIR} does not exist"
  show_usage
fi

if [ ! -r ${ARCHDIR}/Make.defs ]; then
  echo "File ${ARCHDIR}/Make.defs does not exist"
  exit 1
fi

if [ ! -r ${ARCHDIR}/setenv.sh ]; then
  echo "File ${ARCHDIR}/setenv.sh does not exist"
  exit 1
fi

if [ ! -r ${ARCHDIR}/defconfig ]; then
  echo "File ${ARCHDIR}/defconfig does not exist"
  exit 1
fi

cp -f ${ARCHDIR}/Make.defs ${TOPDIR}/. || \
  { echo "Failed to copy ${ARCHDIR}/Make.defs" ; exit 1 ; }
cp -f ${ARCHDIR}/setenv.sh ${TOPDIR}/. || \
  { echo "Failed to copy ${ARCHDIR}/setenv.sh" ; exit 1 ; }
cp -f ${ARCHDIR}/defconfig ${TOPDIR}/.config || \
  { echo "Failed to copy ${ARCHDIR}/defconfig" ; exit 1 ; }

