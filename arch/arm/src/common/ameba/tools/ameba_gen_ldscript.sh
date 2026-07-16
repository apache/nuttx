#!/bin/sh
############################################################################
# arch/arm/src/common/ameba/tools/ameba_gen_ldscript.sh
#
# Generate the combined Ameba AP image2 linker script (ld.script.gen) the
# `nuttx` ELF is linked with, WITHOUT editing any SDK source file.  This is the
# single implementation of the three-step recipe the make PREBUILD used inline,
# so the make and cmake builds produce a byte-identical script:
#
#   1. C-preprocess ameba_img2_all.ld (it #includes ameba_layout.ld and the
#      config-derived platform_autoconf.h, staged as
#      project_<proj>/platform_autoconf.h on the include path).
#   2. Append ameba_rom_symbol_acut_s.ld (ROM symbol addresses).
#   3. Fold NuttX's .vectors orphan section into the loadable SRAM data region
#      so the large power-of-two aligned vector table does not land in and
#      overflow the tiny fixed KM4_IMG2_ENTRY region.
#
# This reproduces the SDK-generated rlx8721d.ld; the SDK tree stays read-only.
#
# Usage: ameba_gen_ldscript.sh <cc> <img2_ld> <rom_ld> <autoconf> \
#                              <prebuilt_dir> <ap_project> <out>
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

set -e

CC="$1"
IMG2_LD="$2"
ROM_LD="$3"
AUTOCONF="$4"
PREBUILT="$5"
AP_PROJECT="$6"
OUT="$7"

if [ -z "$CC" ] || [ -z "$IMG2_LD" ] || [ -z "$ROM_LD" ] || [ -z "$AUTOCONF" ] \
   || [ -z "$PREBUILT" ] || [ -z "$AP_PROJECT" ] || [ -z "$OUT" ]; then
  echo "usage: ameba_gen_ldscript.sh <cc> <img2_ld> <rom_ld> <autoconf>" \
       "<prebuilt_dir> <ap_project> <out>" >&2
  exit 1
fi

# Stage the autoconf where ameba_img2_all.ld's #include resolves it (-I prebuilt
# then #include "project_<proj>/platform_autoconf.h").
mkdir -p "$PREBUILT/project_$AP_PROJECT"
cp "$AUTOCONF" "$PREBUILT/project_$AP_PROJECT/platform_autoconf.h"

# 1) preprocess, 2) append ROM symbols
"$CC" -E -P -xc -c "$IMG2_LD" -o "$OUT" -I "$PREBUILT"
cat "$ROM_LD" >> "$OUT"

# 3) fold NuttX .vectors into the loadable SRAM data region
sed -i 's|^\([[:space:]]*\)\*(\.data\*)|\1*(.vectors*)\n\1*(.data*)|' "$OUT"
