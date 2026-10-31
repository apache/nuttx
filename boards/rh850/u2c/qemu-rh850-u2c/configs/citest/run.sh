#!/usr/bin/env bash
############################################################################
# boards/rh850/u2c/qemu-rh850-u2c/configs/citest/run.sh
#
# SPDX-License-Identifier: Apache-2.0
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
############################################################################

# ostest powers off through the QEMU test-exit device, so the exit code is
# the ostest result; a PANIC exits with 1, a hang with 124 (timeout).

set -o xtrace

# start from nuttx dir
olddir=$(pwd)
nuttdir=${CURRENTCONFDIR}/../../../../../../
cd "${nuttdir}" || exit 1

elf=./nuttx
if [ -f ./build/nuttx ]; then
  elf=./build/nuttx
fi

timeout 900 qemu-system-rh850 -M rh850-u2c,test-exit=on \
  -icount shift=4,sleep=on -nographic -kernel "${elf}" \
  < /dev/null > qemu.log 2>&1
ret="$?"
cat qemu.log
echo $ret

# export test results
artifacts=${ARTIFACTCONFDIR}/qemu
mkdir -p "${artifacts}"
mv qemu.log "${artifacts}"

# restore old dir
cd "${olddir}" || exit 1

exit $ret
