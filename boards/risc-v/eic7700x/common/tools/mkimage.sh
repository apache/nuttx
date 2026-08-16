#!/usr/bin/env bash
############################################################################
# boards/risc-v/eic7700x/common/tools/mkimage.sh
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
#
# Build the bootable image: the kernel, then padding, then the RAM disk.
#
# The padding is computed rather than fixed, and that is the whole point of
# this script.  The RAM disk is appended to the kernel and found at run time
# by searching memory for its header, and that search happens after BSS has
# been cleared.  So the disk has to start above _ebss or it is zeroed before
# anything looks for it.  A fixed pad works only until BSS grows past it,
# and when it does the failure is a panic before the console exists, which
# looks like a board that hangs with no output at all.  The kernel's own
# sanity check compares against the idle stack top, which sits below BSS
# rather than above it, so it does not catch this either.
#
# Usage:
#   mkimage.sh [-o output] [-a apps-dir] [-n nm-tool] [-l load-addr]
#
# The default output name is Image-<board>, taken from the configuration,
# so each board's image names itself.
#
# Run from the nuttx directory after "make" and after the apps have been
# built and installed into their bin directory.

set -e

APPSDIR="../apps"
BOARD=$(sed -n 's/^CONFIG_ARCH_BOARD="\(.*\)"$/\1/p' .config 2>/dev/null)
OUTPUT="Image-${BOARD:-eic7700x}"
NM="${CROSSDEV:-riscv-none-elf-}nm"
LOADADDR="0x80200000"
MARGIN=$((64 * 1024))

usage()
{
  sed -n '25,43p' "$0"
  echo
  echo "  -o  output image             (default: $OUTPUT)"
  echo "  -a  apps directory           (default: $APPSDIR)"
  echo "  -n  nm for the target        (default: $NM)"
  echo "  -l  kernel load address      (default: $LOADADDR)"
  exit 1
}

while getopts "o:a:n:l:h" opt; do
  case $opt in
    o) OUTPUT="$OPTARG" ;;
    a) APPSDIR="$OPTARG" ;;
    n) NM="$OPTARG" ;;
    l) LOADADDR="$OPTARG" ;;
    *) usage ;;
  esac
done

if [ ! -f nuttx ] || [ ! -f nuttx.bin ]; then
  echo "mkimage: run this from the nuttx directory after make" >&2
  exit 1
fi

if [ ! -d "$APPSDIR/bin" ]; then
  echo "mkimage: no $APPSDIR/bin; build and install the apps first" >&2
  exit 1
fi

# Where BSS ends, which is the earliest the RAM disk may start.

EBSS=$("$NM" nuttx | awk '/ _ebss$/ { print $1 }')
if [ -z "$EBSS" ]; then
  echo "mkimage: no _ebss in nuttx; is $NM right for this target?" >&2
  exit 1
fi

TMPDIR_="$(mktemp -d)"
trap 'rm -rf "$TMPDIR_"' EXIT

genromfs -f "$TMPDIR_/initrd" -d "$APPSDIR/bin" -V "NuttXBootVol"

# Pad so that the disk lands past the end of BSS, with a margin.  wc -c is
# used rather than stat, whose flags differ between GNU and BSD.

BINSZ=$(wc -c < nuttx.bin)
NEED=$(( 0x$EBSS - LOADADDR + MARGIN ))
PAD=$(( NEED - BINSZ ))
if [ "$PAD" -lt 0 ]; then
  PAD=0
fi

head -c "$PAD" /dev/zero > "$TMPDIR_/pad"
cat nuttx.bin "$TMPDIR_/pad" "$TMPDIR_/initrd" > "$OUTPUT"

printf '%s: _ebss 0x%s, kernel %s, pad %s, disk at %s\n' \
       "$OUTPUT" "$EBSS" "$BINSZ" "$PAD" "$(( BINSZ + PAD ))"
