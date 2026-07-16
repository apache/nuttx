#!/bin/sh
############################################################################
# arch/arm/src/common/ameba/tools/ameba_flash.sh
#
# Download the built Ameba images to the chip over serial, via the SDK's
# AmebaFlash.py.  This is the single implementation shared by the make `flash`
# target (tools/ameba/Config.mk) and the cmake `flash` target
# (common/ameba/cmake/ameba_board.cmake), so both flash identically.
#
# Two images are written, each at its own flash offset:
#   boot.bin  (SDK bootloader, prebuilt) @ CONFIG_FLASH_BOOT_OFFSET
#   nuttx.bin (AP+NP application image)   @ CONFIG_FLASH_(APP_)OTA1_OFFSET
#
# The offsets are NOT hardcoded: they are read from the config-derived
# platform_autoconf.h, so a flash-layout change is tracked automatically.  The
# app-slot macro is CONFIG_FLASH_APP_OTA1_OFFSET on some ICs (RTL8720F) and
# CONFIG_FLASH_OTA1_OFFSET on others (RTL8721Dx); match either.
#
# Usage: ameba_flash.sh <sdk> <profile> <autoconf> <prebuilt_dir> <nuttx_bin>
#   Serial port + baud come from the environment (so a cmake custom target can
#   forward them without a reconfigure):
#     AMEBA_PORT   serial port (required, e.g. /dev/ttyUSB0)
#     AMEBA_BAUD   baud rate   (default 1500000)
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
PROFILE_NAME="$2"
AUTOCONF="$3"
PREBUILT="$4"
NUTTX_BIN="$5"

AMEBA_BAUD="${AMEBA_BAUD:-1500000}"

if [ -z "$AMEBA_PORT" ]; then
  echo "FLASH error: Missing serial port device." >&2
  echo "USAGE: AMEBA_PORT=/dev/ttyUSB0 [AMEBA_BAUD=$AMEBA_BAUD] <flash cmd>" >&2
  exit 1
fi

BOOT_OFFSET=$(awk '$2=="CONFIG_FLASH_BOOT_OFFSET"{print $3}' "$AUTOCONF" 2>/dev/null)
APP_OFFSET=$(awk '$2 ~ /^CONFIG_FLASH_(APP_)?OTA1_OFFSET$/{print $3}' \
             "$AUTOCONF" 2>/dev/null)
if [ -z "$BOOT_OFFSET" ] || [ -z "$APP_OFFSET" ]; then
  echo "FLASH error: could not read flash offsets from $AUTOCONF" >&2
  echo "  Build first so platform_autoconf.h is generated." >&2
  exit 1
fi

AMEBAPY="$(cat "$SDK/.amebapy/bindir" 2>/dev/null)/python"
[ -x "$AMEBAPY" ] || AMEBAPY="python3"

SCRIPT="$SDK/tools/ameba/Flash/AmebaFlash.py"
[ -f "$SCRIPT" ] || { echo "FLASH error: AmebaFlash.py not found at $SCRIPT" >&2; exit 1; }

PROFILE="$SDK/tools/ameba/Flash/Devices/Profiles/$PROFILE_NAME.rdev"
[ -f "$PROFILE" ] || { echo "FLASH error: profile not found: $PROFILE" >&2; exit 1; }

for spec in "$PREBUILT/boot.bin:$BOOT_OFFSET" "$NUTTX_BIN:$APP_OFFSET"; do
  IMG="${spec%:*}"
  ADDR="${spec##*:}"
  if [ ! -f "$IMG" ]; then
    echo "FLASH error: image not found: $IMG (build first)" >&2
    exit 1
  fi
  echo "FLASH: $IMG @ $ADDR"
  "$AMEBAPY" "$SCRIPT" \
    --download \
    --profile "$PROFILE" \
    --memory-type nor \
    --image "$IMG" \
    --start-address "$ADDR" \
    --port "$AMEBA_PORT" \
    --baudrate "$AMEBA_BAUD" \
    --log-level info
done
