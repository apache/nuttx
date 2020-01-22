#!/usr/bin/env bash
############################################################################
# boards/z16/z16f/z16f2800100zcog/tools/dopatch.sh
#
#   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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
############################################################################

USAGE="${0} [-R] \$PWD"
WD=`pwd`
TOOLDIR=${WD}/boards/z16f2800100zcog/tools
ME=${TOOLDIR}/dopatch.sh
PATCH=${TOOLDIR}/zneo-zdsii-5_0_1-variadic-func-fix.patch
ARGS=${1}

if [ ! -x ${ME} ]; then
  echo "ERROR:  This script must be executed from the top-level NuttX directory"
  echo ${USAGE}
  exit 1
fi

if [ ! -r ${PATCH} ]; then
  echo "ERROR: Readable patch not found at ${PATCH}"
  echo ${USAGE}
  exit 1
fi

cd .. || \
  { echo "ERROR: failed to CD to the parent directory"; exit 1; }

cat ${PATCH} | patch ${ARGS} -p1 || \
  { echo "ERROR: patch failed" ; exit 1; }
