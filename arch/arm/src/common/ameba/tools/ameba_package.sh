#!/bin/sh
############################################################################
# arch/arm/src/common/ameba/tools/ameba_package.sh
#
# Package the linked Ameba AP image2 ELF (`nuttx`) into a flashable nuttx.bin,
# folding the prototype package_app.sh into the NuttX build.  This is the single
# implementation of the make-side ameba_board.mk POSTBUILD, invoked from both
# the make POSTBUILD and the cmake nuttx_post_build target so the two build
# systems produce an identical image.  Steps:
#
#   1. Replicate the SDK image2 postbuild prologue (copy map/axf, nm/objdump).
#   2. (Re)build the NP (km0) image2 + boot from the pinned SDK source, fed the
#      AP disassembly so the WiFi "noused" generator keeps only referenced APIs
#      (must run AFTER the AP link -- see ameba_build_np.sh).
#   3. Run the SDK AP (km4) image2 postbuild.cmake (axf2bin) -> km4_image2_all.bin.
#   4. Run the SDK firmware_package postbuild.cmake combining the AP + NP image2
#      into the packed app image, copied out as nuttx.bin.
#
# Unlike the make build (whose parse-time toolchain.mk puts asdk on PATH), this
# script runs in the build tool's environment, so it resolves the SDK-pinned
# asdk toolchain + prebuilts + python venv itself and prepends them to PATH.
#
# Usage: ameba_package.sh <sdk> <py_soc> <soc_name> <prebuilt> <ap_project> \
#                         <km_proj> <np_target> <toolchain_dir> <nuttx_elf> \
#                         <out_bin>
#
#   <np_target>  NP-core image name (km0 on RTL8721Dx, km4ns on RTL8720F)
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

SDK="$1"
PY_SOC="$2"
SOC_NAME="$3"
PREBUILT="$4"
AP_PROJECT="$5"
KM_PROJ="$6"
NP_TARGET="$7"
TCDIR="${8:-$HOME/rtk-toolchain}"
NUTTX_ELF="$9"
OUT_BIN="${10}"

TOOLS_DIR=$(CDPATH= cd "$(dirname "$0")" && pwd)

if [ -z "$SDK" ] || [ -z "$OUT_BIN" ]; then
  echo "usage: ameba_package.sh <sdk> <py_soc> <soc_name> <prebuilt>" \
       "<ap_project> <km_proj> <np_target> <toolchain_dir> <nuttx_elf>" \
       "<out_bin>" >&2
  exit 1
fi

echo "PACK: nuttx.bin (Ameba AP $AP_PROJECT image2 + NP $NP_TARGET image2)"

# --- Resolve the SDK-pinned asdk toolchain + prebuilts + venv onto PATH ------

VER=$(sed -n 's/.*v_ASDK_VER[ \t][ \t]*\([0-9.][0-9.]*\).*/\1/p' \
      "$SDK/cmake/global_define.cmake")
TC_CMAKE="$SDK/cmake/toolchain/ameba-toolchain-asdk-$VER.cmake"
BUILD=$(sed -n 's/.*ToolChainVerMinor[ \t][ \t]*\([0-9][0-9]*\).*/\1/p' "$TC_CMAKE")
ASDK_BIN="$TCDIR/asdk-$VER-$BUILD/linux/newlib/bin"
[ -d "$ASDK_BIN" ] && PATH="$ASDK_BIN:$PATH"

PBV=$(sed -n 's/^PREBUILTS_VERSION=//p' "$SDK/env.sh" | head -1)
[ -n "$PBV" ] && PATH="$TCDIR/prebuilts-linux-$PBV/bin:$PATH"

# Provision + resolve the python interpreter the SDK cmake steps need.
sh "$TOOLS_DIR/ameba_setup_env.sh" "$SDK" "$TCDIR"
VENV_BIN=$(cat "$SDK/.amebapy/bindir" 2>/dev/null || true)
[ -n "$VENV_BIN" ] && PATH="$VENV_BIN:$PATH"
export PATH

CROSSDEV=arm-none-eabi-
NM="${CROSSDEV}nm"
OBJDUMP="${CROSSDEV}objdump"
STRIP="${CROSSDEV}strip"
OBJCOPY="${CROSSDEV}objcopy"
SIZE="${CROSSDEV}size"
CMAKE="${CMAKE:-cmake}"

SOC_PROJ="$SDK/component/soc/$SOC_NAME/project"
KM4_PROJ="$SOC_PROJ/$KM_PROJ"
PKGDIR="$PREBUILT/pkg"

# --- 1. image2 postbuild prologue -------------------------------------------

rm -rf "$PKGDIR"
mkdir -p "$PKGDIR"
cp "$NUTTX_ELF" "$PKGDIR/target_img2.axf"
"$NM" "$PKGDIR/target_img2.axf" | sort > "$PKGDIR/target_img2.map"
"$OBJDUMP" -d "$PKGDIR/target_img2.axf" > "$PKGDIR/target_img2.asm"

# --- 2. (re)build NP (km0) image2 from SDK source, fed the AP disassembly ----

sh "$TOOLS_DIR/ameba_build_np.sh" "$SDK" "$PY_SOC" "$PREBUILT" \
   "$PKGDIR/target_img2.asm" "$NP_TARGET" "$AP_PROJECT"

cp "$PKGDIR/target_img2.axf" "$PKGDIR/target_pure_img2.axf"
"$STRIP" "$PKGDIR/target_pure_img2.axf"
cp "$PREBUILT/config_km4" "$PKGDIR/.config_km4"
cp "$PREBUILT/config_fw" "$PKGDIR/.config"
cp "$PREBUILT/${NP_TARGET}_image2_all.bin" "$PKGDIR/${NP_TARGET}_image2_all.bin"
printf '{\n    "soc": {\n        "name": "%s"\n    }\n}\n' "$PY_SOC" \
  > "$PKGDIR/soc_info.json"

# --- 3. AP (km4) image2 axf2bin -> km4_image2_all.bin -----------------------

TARGET_SOC="$PY_SOC" "$CMAKE" \
  -Dc_BASEDIR="$SDK" \
  -Dc_CMAKE_FILES_DIR="$SDK/cmake" \
  -Dc_SOC_PROJECT_DIR="$SOC_PROJ" \
  -Dc_MCU_PROJECT_DIR="$KM4_PROJ" \
  -Dc_MCU_PROJECT_NAME="$AP_PROJECT" \
  -Dc_MCU_KCONFIG_FILE="$PKGDIR/.config_km4" \
  -Dc_SDK_IMAGE_TARGET_DIR="$PKGDIR" \
  -DKM4_BUILDDIR= \
  -DFINAL_IMAGE_DIR="$PKGDIR" \
  -DBUILD_TYPE=NONE -DANALYZE_MP_IMG=0 -DDAILY_BUILD=0 \
  -DEXTERN_DIR="$PKGDIR" -DCODE_ANALYZE_RETRY= \
  -DIMAGESCRIPTDIR="$SDK/tools/image_scripts" \
  -DCMAKE_SIZE="$SIZE" -DCMAKE_OBJCOPY="$OBJCOPY" \
  -P "$KM4_PROJ/make/image2/postbuild.cmake"

# --- 4. firmware_package: combine AP + NP image2 -> app.bin -> nuttx.bin -----

TARGET_SOC="$PY_SOC" "$CMAKE" \
  -Dc_BASEDIR="$SDK" \
  -Dc_CMAKE_FILES_DIR="$SDK/cmake" \
  -Dc_MCU_KCONFIG_FILE="$PKGDIR/.config" \
  -Dc_SOC_PROJECT_DIR="$SOC_PROJ" \
  -Dc_SDK_IMAGE_TARGET_DIR= \
  -Dc_IMAGE_OUTPUT_DIR="$PKGDIR" \
  -Dc_APP_BINARY_NAME=app.bin \
  -Dc_IMAGE1_ALL_FILES= \
  -Dc_IMAGE2_ALL_FILES="$PKGDIR/${NP_TARGET}_image2_all.bin;$PKGDIR/${AP_PROJECT}_image2_all.bin" \
  -Dc_IMAGE3_ALL_FILES= \
  -DFINAL_IMAGE_DIR="$PKGDIR" \
  -DANALYZE_MP_IMG=0 -DEXTERN_DIR="$PKGDIR" \
  -P "$SOC_PROJ/postbuild.cmake"

cp "$PKGDIR/app.bin" "$OUT_BIN"
echo "PACK: wrote $OUT_BIN"
# Flash hint is caller-supplied (AMEBA_FLASH_HINT) so the make and cmake build
# paths each print their own flash command; printed only when provided.
if [ -n "$AMEBA_FLASH_HINT" ]; then
  echo "      Flash with: $AMEBA_FLASH_HINT"
fi
