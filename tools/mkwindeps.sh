#!/bin/bash
# tools/mkwindeps.sh
#
#   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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

# Uncomment to enable debug options
# set -x
# DEBUG=--dep-debug

# Make sure that we know where we are

TOOLDIR=$(dirname $0)

if [ ! -x ${TOOLDIR}/mkwindeps.sh ]; then
  echo "# ERROR: tools/ directory not found"
  exit 1
fi

# Make sure that executables are ready

MKDEPS=${TOOLDIR}/mkdeps.exe
CNVWINDEPS=${TOOLDIR}/cnvwindeps.exe

if [ ! -x ${MKDEPS} ]; then
  echo "# ERROR: tools/mkdeps.exe does not exist"
  exit 1
fi

if [ ! -x ${CNVWINDEPS} ]; then
  echo "# ERROR: tools/cnvwindeps.exe does not exist"
  exit 1
fi

# Run the mkdeps.exe program to generate a Windows dependency file

TMPFILE=$(mktemp)
${MKDEPS} ${DEBUG} --winpath $* > ${TMPFILE} || { echo "# ERROR: mkdeps.exe failed"; exit 1; }

# Then convert this to a POSIX dependency file (on stdout)

${CNVWINDEPS} ${TMPFILE}
rm -f ${TMPFILE}
