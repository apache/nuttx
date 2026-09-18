#!/usr/bin/env sh
# SPDX-License-Identifier: Apache-2.0
#
# Flash a NuttX image to the MIMXRT1180-EVK using SEGGER J-Link.
#
# Usage:
#     ./flash-jlink.sh <image.bin> <flash-offset>
#
# Arguments:
#     image.bin     Binary to program.
#     flash-offset  Offset within FlexSPI1 NOR, either as an absolute
#                   flash address (0x28080000) or as an offset from the
#                   flash base (0x80000).  Both forms are accepted.
#
# The two images of the MIMXRT1180-EVK build are programmed with two
# separate invocations:
#
#     # Cortex-M33 bootloader / application (imxrt1180-evk:bl, :nsh-m33)
#     ./flash-jlink.sh flash.bin 0x0
#
#     # Cortex-M7 NuttX payload (imxrt1180-evk:nsh)
#     ./flash-jlink.sh nuttx.bin 0x80000
#
# J-Link only erases the sectors the image actually covers, so flashing
# one image leaves the other intact.
#
# The script assumes:
#   * A SEGGER J-Link probe is attached to the EVK JTAG connector (J37)
#     or an MCU-Link running SEGGER's "MCU-Link J-Link" firmware is used
#     via J53.
#   * SW5 = 0100 (FlexSPI Quad SPI NOR boot).
#   * The J-Link software (v7.98a or later, which adds MIMXRT1189
#     device support) is installed and JLinkExe is on PATH.
#
# Overrides via environment variables:
#   JLINK_EXE    - JLinkExe binary (default: JLinkExe)
#   JLINK_SPEED  - SWD speed in kHz (default: 4000)
#   JLINK_IF     - Debug interface (default: SWD)

set -eu

FLASH_BASE=0x28000000

usage() {
    sed -n '4,39p' "$0" | sed 's/^# \{0,1\}//'
    exit "${1:-1}"
}

# Programming targets the Cortex-M33 profile.  On RT1180 the M33 is the
# boot core; the ROM initializes FlexSPI1 as part of the M33 boot, which
# is what the SEGGER flash loader needs.  Connecting via the _M7 profile
# releases the M7 without running the M33/ROM init and leaves FlexSPI in
# a state where the RAM-side loader fails with "RAMCode-sided Prepare()".
# For interactive debugging of the running M7 use MIMXRT1189xxx8_M7
# (see gdbserver-jlink.sh).

JLINK_DEVICE=MIMXRT1189xxx8_M33

if [ $# -eq 0 ]; then
    usage 0
fi

if [ $# -ne 2 ]; then
    echo "Error: expected 2 arguments, got $#." >&2
    echo >&2
    usage 1
fi

FLASH_BIN="$1"
OFFSET_ARG="$2"

JLINK_EXE="${JLINK_EXE:-JLinkExe}"
JLINK_SPEED="${JLINK_SPEED:-4000}"
JLINK_IF="${JLINK_IF:-SWD}"

if [ ! -f "$FLASH_BIN" ]; then
    echo "Error: $FLASH_BIN not found." >&2
    echo "       Run 'make' first to produce it, or pass the path as an" >&2
    echo "       argument to this script." >&2
    exit 1
fi

if ! command -v "$JLINK_EXE" >/dev/null 2>&1; then
    echo "Error: '$JLINK_EXE' not found on PATH." >&2
    echo "       Install the SEGGER J-Link Software Pack from" >&2
    echo "       https://www.segger.com/downloads/jlink/" >&2
    exit 1
fi

# Resolve the offset to an absolute flash address.  Values already inside
# the FlexSPI1 XIP window are taken as-is; anything smaller is treated as
# an offset relative to the flash base.

OFFSET=$(printf "%d" "$OFFSET_ARG" 2>/dev/null) || {
    echo "Error: '$OFFSET_ARG' is not a valid numeric offset." >&2
    exit 1
}

if [ "$OFFSET" -lt "$(printf "%d" $FLASH_BASE)" ]; then
    ADDR=$((OFFSET + FLASH_BASE))
else
    ADDR=$OFFSET
fi

ADDR_HEX=$(printf "0x%08X" "$ADDR")

# JLinkExe reads the commander script from -CommanderScript.  Generate it
# on the fly so the load address can be parameterized, and stage both in
# a temp working directory that JLinkExe is invoked from.

WORK_DIR=$(mktemp -d)
trap 'rm -rf "$WORK_DIR"' EXIT

cp "$FLASH_BIN" "$WORK_DIR/image.bin"

cat > "$WORK_DIR/flash.jlink" <<JLINK
r
halt
loadbin image.bin, $ADDR_HEX
r
g
q
JLINK

FLASH_SIZE=$(stat -c%s "$FLASH_BIN")
echo "Flashing $FLASH_BIN ($FLASH_SIZE B) at $ADDR_HEX"
echo "  via $JLINK_EXE ($JLINK_DEVICE, $JLINK_IF @ ${JLINK_SPEED}kHz)"
(cd "$WORK_DIR" && \
 "$JLINK_EXE" -device "$JLINK_DEVICE" \
              -if "$JLINK_IF" \
              -speed "$JLINK_SPEED" \
              -autoconnect 1 \
              -CommanderScript "$WORK_DIR/flash.jlink")

echo "Done."
