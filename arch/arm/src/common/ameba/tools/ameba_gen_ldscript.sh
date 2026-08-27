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
#   2. Append the ROM symbol linker script(s) (ROM symbol addresses).  Most
#      ICs pass a single script; rtl8721f passes four scripts (secure + wifi +
#      os + NS), matching the make build's cat.
#   3. Fold NuttX's .vectors orphan section into the loadable SRAM data region
#      so the large power-of-two aligned vector table does not land in and
#      overflow the tiny fixed KM4_IMG2_ENTRY region.
#
# This reproduces the SDK-generated rlx8721d.ld; the SDK tree stays read-only.
#
# Usage: ameba_gen_ldscript.sh <cc> <img2_ld> <autoconf> <prebuilt_dir> \
#                              <ap_project> <out> <rom_ld>...
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
AUTOCONF="$3"
PREBUILT="$4"
AP_PROJECT="$5"
OUT="$6"
shift 6
ROM_LDS="$@"

if [ -z "$CC" ] || [ -z "$IMG2_LD" ] || [ -z "$AUTOCONF" ] || [ -z "$PREBUILT" ] \
   || [ -z "$AP_PROJECT" ] || [ -z "$OUT" ] || [ -z "$ROM_LDS" ]; then
  echo "usage: ameba_gen_ldscript.sh <cc> <img2_ld> <autoconf> <prebuilt_dir>" \
       "<ap_project> <out> <rom_ld>..." >&2
  exit 1
fi

# Stage the autoconf where ameba_img2_all.ld's #include resolves it (-I prebuilt
# then #include "project_<proj>/platform_autoconf.h").
mkdir -p "$PREBUILT/project_$AP_PROJECT"
cp "$AUTOCONF" "$PREBUILT/project_$AP_PROJECT/platform_autoconf.h"

# 1) preprocess, 2) append ROM symbol script(s), in the given order
"$CC" -E -P -xc -c "$IMG2_LD" -o "$OUT" -I "$PREBUILT"
cat $ROM_LDS >> "$OUT"

# 3) fold NuttX .vectors into the loadable SRAM data region
sed -i 's|^\([[:space:]]*\)\*(\.data\*)|\1*(.vectors*)\n\1*(.data*)|' "$OUT"
