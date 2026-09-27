#!/usr/bin/env bash
############################################################################
# tools/nxflat/testsuite.sh
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
# Builds every module of apps/examples/nxflat/tests through the NXFLAT
# toolchain and reports, per module, where it stopped and which relocations
# it carried.  It needs no target and no board: a module is built for
# cortex-m3 against a configured tree's headers, which is what the boards
# using NXFLAT do.
#
# Usage:
#   tools/nxflat/testsuite.sh [module ...]
#
# Environment:
#   NUTTX_DIR     configured, contexted NuttX tree     [the tree of this file]
#   APPS_DIR      apps tree holding examples/nxflat    [$NUTTX_DIR/../apps]
#   CROSSDEV      toolchain prefix                     [arm-none-eabi-]
#   MKNXFLAT      thunk generator                      [$NUTTX_DIR/tools/mknxflat]
#   LDNXFLAT      the converter                        [$NUTTX_DIR/tools/ldnxflat]
#   CPU           -mcpu= value for the modules         [cortex-m3]
#   OUTDIR        where to build                       [./nxflat-testsuite]
#
############################################################################

set -o pipefail

here=$(cd "$(dirname "$0")" && pwd)
NUTTX_DIR=${NUTTX_DIR:-$(cd "$here/../.." && pwd)}
APPS_DIR=${APPS_DIR:-$(cd "$NUTTX_DIR/../apps" 2>/dev/null && pwd)}
CROSSDEV=${CROSSDEV:-arm-none-eabi-}
MKNXFLAT=${MKNXFLAT:-$NUTTX_DIR/tools/mknxflat}
LDNXFLAT=${LDNXFLAT:-$NUTTX_DIR/tools/ldnxflat}
CPU=${CPU:-cortex-m3}
OUTDIR=${OUTDIR:-$PWD/nxflat-testsuite}

TESTS=$APPS_DIR/examples/nxflat/tests
LDSCRIPT=$NUTTX_DIR/binfmt/libnxflat/gnu-nxflat-gotoff.ld

[ -r "$NUTTX_DIR/include/nuttx/config.h" ] || \
  { echo "$NUTTX_DIR is not configured: run tools/configure.sh and make context"; exit 1; }
[ -d "$TESTS" ] || { echo "No $TESTS -- set APPS_DIR"; exit 1; }
[ -x "$MKNXFLAT" ] || \
  { echo "No $MKNXFLAT -- build it with: make -C tools -f Makefile.host mknxflat"; exit 1; }

PIC="-fpic -msingle-pic-base -mpic-register=r9 -mno-pic-data-is-text-relative"
CFLAGS="$PIC -mcpu=$CPU -mthumb -Os -fno-builtin -Wall -isystem $NUTTX_DIR/include -D__NuttX__"
CXXFLAGS="$CFLAGS -fno-exceptions -fno-rtti -nostdinc++ -isystem $NUTTX_DIR/include/cxx"

# A module is one directory, except hello++, which is four separate modules.

# romfs/ is where the modules are installed, not one of them.

modules=${*:-$(cd "$TESTS" && ls -d */ | tr -d / | grep -v '^romfs$')}
rc=0

report()   { printf '%-12s %-9s %s\n' "$1" "$2" "$3"; }

finish()   # $1 module name: convert and report
{
  local name=$1 d=$OUTDIR/$1 relocs size

  relocs=$(${CROSSDEV}readelf -r "$d/m.r2" |
           grep -oE 'R_ARM_[A-Z0-9_]+' | sort | uniq -c | tr -s ' ' | tr '\n' ' ')

  if ! "$LDNXFLAT" -e main -o "$d/m.nxf" "$d/m.r2" > "$d/ld.log" 2>&1; then
    report "$name" LDNXFLAT "$(grep -m1 -iE 'error' "$d/ld.log")"
    return 1
  fi

  size=$(wc -c < "$d/m.nxf" | tr -d ' ')
  report "$name" OK "${size}B  | $relocs"
  return 0
}

build()    # $1 module name  $2... sources
{
  local name=$1; shift
  local d=$OUTDIR/$name objs= src obj
  rm -rf "$d"; mkdir -p "$d" || return 1

  for src in "$@"; do
    case $src in
      *.cxx) obj=$d/$(basename "$src" .cxx).o
             ${CROSSDEV}g++ $CXXFLAGS -c "$src" -o "$obj" 2> "$d/err" || \
               { report "$name" COMPILE "$(grep -m1 -i error "$d/err")"; return 1; } ;;
      *)     obj=$d/$(basename "$src" .c).o
             ${CROSSDEV}gcc $CFLAGS -c "$src" -o "$obj" 2> "$d/err" || \
               { report "$name" COMPILE "$(grep -m1 -i error "$d/err")"; return 1; } ;;
    esac
    objs="$objs $obj"
  done

  ${CROSSDEV}ld -r -d -warn-common -o "$d/m.r1" $objs 2> "$d/err" || \
    { report "$name" LD1 "$(head -1 "$d/err")"; return 1; }
  "$MKNXFLAT" -a thumb2 -o "$d/m-thunk.S" "$d/m.r1" 2> "$d/err" || \
    { report "$name" MKNXFLAT "$(head -1 "$d/err")"; return 1; }
  ${CROSSDEV}gcc $CFLAGS -c "$d/m-thunk.S" -o "$d/m-thunk.o" 2> "$d/err" || \
    { report "$name" AS "$(head -1 "$d/err")"; return 1; }
  ${CROSSDEV}ld -r -d -warn-common -T "$LDSCRIPT" -no-check-sections \
    -o "$d/m.r2" $objs "$d/m-thunk.o" 2> "$d/err" || \
    { report "$name" LD2 "$(head -1 "$d/err")"; return 1; }
  return 0
}

for m in $modules; do
  case $m in
    hello++) list="1 2 3 4" ;;
    *)       list="" ;;
  esac

  if [ -n "$list" ]; then
    for n in $list; do
      name=hello++$n
      build "$name" "$TESTS/hello++/hello++$n.cxx" || { rc=1; continue; }
      finish "$name" || rc=1
    done
    continue
  fi

  name=$m
  srcs=$(ls "$TESTS/$m"/*.c "$TESTS/$m"/*.cxx 2>/dev/null)
  [ -n "$srcs" ] || { report "$m" SKIP "no sources"; continue; }
  build "$name" $srcs || { rc=1; continue; }
  finish "$name" || rc=1
done

exit $rc
