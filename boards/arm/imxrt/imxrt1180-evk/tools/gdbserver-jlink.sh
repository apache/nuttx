#!/usr/bin/env sh
# SPDX-License-Identifier: Apache-2.0
#
# Start a J-Link GDB server attached to the Cortex-M7 on the
# MIMXRT1180-EVK.  Once running, connect from another shell with:
#
#     arm-none-eabi-gdb nuttx
#     (gdb) target extended-remote localhost:2331
#     (gdb) monitor reset
#     (gdb) continue
#
# Overrides via environment variables (same as flash-jlink.sh):
#   JLINK_GDBSERVER  - JLinkGDBServer binary (default: JLinkGDBServer)
#   JLINK_SPEED      - SWD speed in kHz (default: 4000)
#   JLINK_IF         - Debug interface (default: SWD)
#   JLINK_PORT       - GDB listen port (default: 2331)

set -eu

# Fixed for the MIMXRT1180-EVK (MIMXRT1189CVM8C, Cortex-M7 target).

JLINK_DEVICE=MIMXRT1189xxx8_M7

JLINK_GDBSERVER="${JLINK_GDBSERVER:-JLinkGDBServer}"
JLINK_SPEED="${JLINK_SPEED:-4000}"
JLINK_IF="${JLINK_IF:-SWD}"
JLINK_PORT="${JLINK_PORT:-2331}"

if ! command -v "$JLINK_GDBSERVER" >/dev/null 2>&1; then
    echo "Error: '$JLINK_GDBSERVER' not found on PATH." >&2
    echo "       Install the SEGGER J-Link Software Pack from" >&2
    echo "       https://www.segger.com/downloads/jlink/" >&2
    exit 1
fi

exec "$JLINK_GDBSERVER" \
     -device "$JLINK_DEVICE" \
     -if "$JLINK_IF" \
     -speed "$JLINK_SPEED" \
     -port "$JLINK_PORT" \
     -singlerun \
     -nogui
