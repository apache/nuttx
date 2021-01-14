#!/usr/bin/env bash
############################################################################
# tools/esp32/btdecode.sh
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

# This script can be used to decode the backtrace that's dumped on assertions.
#
# On assertions we can find the raw backtrace dump similar to:
# ...
# xtensa_btdump: Backtrace0: 400d3ed7:3ffb1300
# xtensa_btdump: Backtrace1: 400d3f33:3ffb1320
# xtensa_btdump: Backtrace2: 400d2875:3ffb1340
# xtensa_btdump: BACKTRACE CONTINUES...
# ...
#
# Copy that to a file and call this script as:
#    ./tools/esp32/btdecode.sh backtrace_file
#
# The result should be similar to the following:
#    0x400d3ed7: xtensa_assert at xtensa_assert.c:108
#    0x400d3f33: up_assert at xtensa_assert.c:184 (discriminator 2)
#    0x400d2875: _assert at lib_assert.c:36

USAGE="USAGE: ${0} backtrace_file"

# Make sure we have the required argument(s)

if [ -z "$1" ]; then
  echo "No backtrace supplied!"
  echo "$USAGE"
  exit 1
fi

# Make sure the project was built

if [ ! -f nuttx ]; then
  echo "NuttX binaries not found!"
  exit 2
fi

# Check that the toolchain is in the PATH

if [ ! -x "$(command -v xtensa-esp32-elf-addr2line)" ]; then
  echo "ESP32 toolchain not found!"
  exit 3
fi

# Decode backtrace

for bt in `cat $1 | cut -d':' -f3`; do
  xtensa-esp32-elf-addr2line -pfiaCs -e nuttx $bt
done

