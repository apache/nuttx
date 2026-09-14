#!/bin/sh
############################################################################
# arch/arm/src/common/ameba/tools/ameba_smart_flash.sh
#
# Package NuttX as BL33 into a FIP, combine with the vendor KM0/KM4/ATF
# prefix, and flash the result to RTL8730E (AmebaSmart CA32) over UART.
#
# Unlike the Cortex-M33 Ameba ICs where nuttx.bin is the complete image,
# RTL8730E uses an ATF boot chain (BL1→BL2→BL32/SP_MIN→BL33).  NuttX
# runs as BL33 inside a Firmware Image Package (FIP).  The final app.bin
# is a concatenation of a vendor-provided prefix (KM0+KM4 firmware + ATF
# BL1) and a prepend-header + FIP (BL2 + BL32 + NuttX-as-BL33).
#
# Flash layout (RTL8730E_NOR.rdev):
#   boot.bin  @ 0x08000000  (KM4 bootloader)
#   app.bin   @ 0x0803FC00  (KM0+KM4 app + ATF FIP containing BL33=NuttX)
#
# Usage: ameba_smart_flash.sh <sdk> <prebuilt_dir> <nuttx_bin>
#   AMEBA_PORT  serial port (required, e.g. /dev/ttyUSB1)
#   AMEBA_BAUD  baud rate   (default 1500000)
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
PREBUILT="$2"
NUTTX_BIN="$3"

AMEBA_BAUD="${AMEBA_BAUD:-1500000}"

if [ -z "$AMEBA_PORT" ]; then
  echo "FLASH error: Missing serial port device." >&2
  echo "USAGE: AMEBA_PORT=/dev/ttyUSB1 [AMEBA_BAUD=$AMEBA_BAUD] make flash" >&2
  exit 1
fi

if [ ! -f "$NUTTX_BIN" ]; then
  echo "FLASH error: nuttx.bin not found: $NUTTX_BIN (build first)" >&2
  exit 1
fi

FIPTOOL="$PREBUILT/fiptool"
if [ ! -x "$FIPTOOL" ]; then
  echo "FLASH error: fiptool not found or not executable: $FIPTOOL" >&2
  exit 1
fi

TMPDIR=$(mktemp -d)
trap 'rm -rf "$TMPDIR"' EXIT

# -------------------------------------------------------------------------
# Step 1: Build FIP with NuttX as BL33
# -------------------------------------------------------------------------
echo "FLASH: building FIP (BL2 + BL32 + NuttX-as-BL33)..."
"$FIPTOOL" create \
  --tb-fw  "$PREBUILT/bl2.bin"  \
  --tos-fw "$PREBUILT/bl32.bin" \
  --nt-fw  "$NUTTX_BIN"         \
  "$TMPDIR/fip.bin"

# -------------------------------------------------------------------------
# Step 2: Build 32-byte prepend header and assemble app.bin
#
# Header layout (RTL8730E image2 format):
#   [0:8]   ASCII magic "81958711"
#   [8:12]  FIP payload size (LE u32)
#   [12:16] FIP load address 0x70300000 (LE u32, __ca32_fip_dram_start__)
#   [16:32] 0xFF * 16 (padding)
# -------------------------------------------------------------------------
echo "FLASH: assembling app.bin (vendor_prefix + fip_prepend + fip)..."
python3 - "$TMPDIR/fip.bin" "$PREBUILT/vendor_prefix.bin" "$TMPDIR/app.bin" << 'PYEOF'
import sys, struct

fip_path, prefix_path, out_path = sys.argv[1], sys.argv[2], sys.argv[3]
fip_data    = open(fip_path,    "rb").read()
prefix_data = open(prefix_path, "rb").read()

header = (b"81958711" +
          struct.pack("<I", len(fip_data)) +
          struct.pack("<I", 0x70300000) +
          b"\xff" * 16)

with open(out_path, "wb") as f:
    f.write(prefix_data)
    f.write(header)
    f.write(fip_data)

print(f"  vendor_prefix: {len(prefix_data)} B  fip: {len(fip_data)} B"
      f"  app.bin: {len(prefix_data)+32+len(fip_data)} B")
PYEOF

# -------------------------------------------------------------------------
# Step 3: Flash via AmebaFlash.py --image-dir (profile handles addresses)
# -------------------------------------------------------------------------
AMEBAPY="$(cat "$SDK/.amebapy/bindir" 2>/dev/null)/python"
[ -x "$AMEBAPY" ] || AMEBAPY="python3"

SCRIPT="$SDK/tools/ameba/Flash/AmebaFlash.py"
if [ ! -f "$SCRIPT" ]; then
  echo "FLASH error: AmebaFlash.py not found at $SCRIPT" >&2
  exit 1
fi

PROFILE="$SDK/tools/ameba/Flash/Devices/Profiles/RTL8730E_NOR.rdev"
if [ ! -f "$PROFILE" ]; then
  echo "FLASH error: RTL8730E_NOR.rdev profile not found" >&2
  exit 1
fi

cp "$PREBUILT/boot.bin" "$TMPDIR/boot.bin"

echo "FLASH: boot.bin + app.bin via profile $PROFILE"
"$AMEBAPY" "$SCRIPT" \
  --download \
  --profile "$PROFILE" \
  --memory-type nor \
  --image-dir "$TMPDIR" \
  --port "$AMEBA_PORT" \
  --baudrate "$AMEBA_BAUD" \
  --log-level info

echo "FLASH: done"
