#!/bin/sh
############################################################################
# tools/fdpic/fdpic-verify.sh
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

# Check that a built module is loadable before it ever reaches the target.
#
# Two things go wrong quietly:
#   - the object is not actually FDPIC (wrong toolchain, or -shared omitted),
#     which the loader rejects with a generic -ENOEXEC;
#   - it imports a symbol the firmware does not export, which the loader
#     reports as -ENOENT with no indication of which symbol.
#
# Usage: fdpic-verify.sh <module.fdpic> [exports-file] [lib.so ...]
#
# Any shared libraries the module links against are passed after the
# exports file; the symbols they define count as satisfied, exactly as they
# will at load time when the loader resolves DT_NEEDED.

set -e
MOD="${1:?usage: fdpic-verify.sh <module> [exports-file] [libs...]}"
EXPORTS="$2"
shift 2 2>/dev/null || shift $#
LIBS="$*"
READELF="${READELF:-arm-uclinuxfdpiceabi-readelf}"

fail=0

osabi=$("$READELF" -h "$MOD" | sed -n 's/.*OS\/ABI:[[:space:]]*//p')
case "$osabi" in
    *"ARM FDPIC"*) ;;
    *)
        echo "FAIL  not an FDPIC object (OS/ABI: ${osabi:-unknown})"
        echo "      check the toolchain is arm-uclinuxfdpiceabi and that"
        echo "      the link used -Wl,-shared"
        fail=1
        ;;
esac

etype=$("$READELF" -h "$MOD" | sed -n 's/.*Type:[[:space:]]*\([A-Z]*\).*/\1/p')
[ "$etype" = "DYN" ] || { echo "FAIL  e_type is $etype, expected DYN"; fail=1; }

nload=$("$READELF" -lW "$MOD" | grep -c '^  LOAD' || true)
[ "$nload" -ge 2 ] || {
    echo "FAIL  expected 2 LOAD segments (RX + RW), found $nload"; fail=1; }

# Undefined dynamic symbols are the module's imports
imports=$("$READELF" --dyn-syms -W "$MOD" \
          | awk '$7 == "UND" && $8 != "" { print $8 }' | sort -u)

if [ -n "$EXPORTS" ] && [ -f "$EXPORTS" ]; then
    tmp=$(mktemp)
    avail=$(mktemp)
    echo "$imports" > "$tmp"
    cat "$EXPORTS" > "$avail"

    # Symbols the linked libraries define are resolved at load time
    for lib in $LIBS; do
        [ -f "$lib" ] || continue
        "$READELF" --dyn-syms -W "$lib" \
          | awk '$7 != "UND" && $4 != "SECTION" && $8 != "" { print $8 }' \
          >> "$avail"
    done

    sort -u "$avail" -o "$avail"
    missing=$(comm -23 "$tmp" "$avail" || true)
    rm -f "$tmp" "$avail"
    if [ -n "$missing" ]; then
        echo "FAIL  imports the firmware does not export:"
        echo "$missing" | sed 's/^/        /'
        fail=1
    fi
fi

if [ "$fail" -eq 0 ]; then
    entry=$("$READELF" -h "$MOD" | sed -n 's/.*Entry point address:[[:space:]]*//p')
    nimp=$(echo "$imports" | grep -c . || true)
    echo "OK    $(basename "$MOD"): FDPIC, entry $entry, $nimp imports resolved"
fi

exit $fail
