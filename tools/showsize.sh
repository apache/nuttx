#!/bin/bash
############################################################################
# tools/showsize.sh
#
#   Copyright (C) 2016 Gregory Nutt. All rights reserved.
#   Author: Lorenz Meier (Original concept)
#           Gregory Nutt (This instantiation)
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

# set -x

# Host nm should always work
# vs. NM=arm-none-eabi-nm

NM=nm

# This should be executed from the top-level NuttX directory

if [ ! -x "tools/showsize.sh" ]; then
  echo "This script must executed from the top-level NuttX directory"
  exit 1
fi

# On the cywin simulation, the executable will be nuttx.exe

if [ -f "nuttx" ]; then
  NUTTX=nuttx
else
  if [ -x "nuttx.exe" ]; then
    NUTTX=nuttx.exe
  else
    echo "Cannot find the NuttX executable"
    exit 1
  fi
fi

# Show what we were asked for

echo "TOP 10 BIG DATA"
$NM --print-size --size-sort --radix dec -C $NUTTX | grep ' [DdBb] ' | tail -20

echo "TOP 10 BIG CODE"
$NM --print-size --size-sort --radix dec -C $NUTTX | grep ' [TtWw] ' | tail -20
