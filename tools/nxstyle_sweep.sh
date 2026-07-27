#!/usr/bin/env bash
############################################################################
# tools/nxstyle_sweep.sh
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

# Run nxstyle over the whole repository, or over a subdirectory, and collect
# what it reports.
#
# Usage: tools/nxstyle_sweep.sh [options] [<dir> ...]
#
#   -o <file>   Write the diagnostics to <file>  (default: nxstyle-errors.txt)
#   -f <file>   Write the failing file names, with a count each and worst
#               first, to <file>                 (default: nxstyle-files.txt)
#   -l          List failing files on stdout instead of writing the reports
#   -s          Summarise the diagnostics by message on stdout
#   -b <path>   Use an existing nxstyle binary rather than building one
#   -j <n>      Run <n> checks in parallel        (default: number of CPUs)
#   -a          Check every file, not only those tracked by git
#   -h          Show this help
#
# With no directory given the whole repository is checked.  Paths are taken
# relative to the top of the repository, as nxstyle verifies the path
# recorded in each file header.
#
# Examples:
#   tools/nxstyle_sweep.sh
#   tools/nxstyle_sweep.sh -s arch/arm/src/stm32h7
#   tools/nxstyle_sweep.sh -l drivers | head
#   tools/nxstyle_sweep.sh -o /tmp/errors.txt arch drivers

set -u

tooldir=$(cd "$(dirname "$0")" && pwd)
topdir=$(cd "$tooldir/.." && pwd)

outfile=nxstyle-errors.txt
filefile=nxstyle-files.txt
listonly=0
summary=0
nxstyle=
jobs=$( (nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4) )
allfiles=0
tmpdir=

usage()
{
  # Print the block of comments that follows the licence header

  awk '/^#####/                { sep++; next }
       sep < 2                 { next }
       /^#/                    { line = $0; sub(/^# ?/, "", line);
                                 print line; started = 1; next }
       started                 { exit }' "$0"
  exit "${1:-1}"
}

cleanup()
{
  if [ -n "$tmpdir" ]; then
    rm -rf "$tmpdir"
  fi
}

while getopts ":o:f:b:j:lsah" opt; do
  case $opt in
    o) outfile=$OPTARG ;;
    f) filefile=$OPTARG ;;
    b) nxstyle=$OPTARG ;;
    j) jobs=$OPTARG ;;
    l) listonly=1 ;;
    s) summary=1 ;;
    a) allfiles=1 ;;
    h) usage 0 ;;
    *) usage ;;
  esac
done
shift $((OPTIND - 1))

trap cleanup EXIT
tmpdir=$(mktemp -d)

# Build nxstyle unless an existing binary was given

if [ -z "$nxstyle" ]; then
  nxstyle=$tmpdir/nxstyle
  if ! ${CC:-cc} -O2 -o "$nxstyle" "$topdir/tools/nxstyle.c"; then
    echo "ERROR: failed to build nxstyle" >&2
    exit 1
  fi
fi

cd "$topdir" || exit 1

# Collect the files to check.  Only the sources that nxstyle understands are
# of interest, and by default only those that are tracked by git.

listing=$tmpdir/files
: > "$listing"

if [ $# -eq 0 ]; then
  set -- .
fi

for dir in "$@"; do
  if [ ! -d "$dir" ]; then
    echo "ERROR: no such directory: $dir" >&2
    exit 1
  fi

  if [ "$allfiles" -eq 0 ] && git rev-parse --git-dir >/dev/null 2>&1; then
    git ls-files -- "$dir/*.c" "$dir/*.h" >> "$listing"
  else
    find "$dir" -name '*.c' -o -name '*.h' >> "$listing"
  fi
done

sort -u "$listing" -o "$listing"
total=$(wc -l < "$listing")

if [ "$total" -eq 0 ]; then
  echo "No C sources found" >&2
  exit 1
fi

# Check each file, keeping the output of one file together.  Writing to a
# file per source and concatenating afterwards avoids the interleaving that
# a shared pipe would produce.

results=$tmpdir/results
mkdir -p "$results"
export nxstyle results

# shellcheck disable=SC2016
xargs -a "$listing" -P "$jobs" -n 1 sh -c '
  out=$("$nxstyle" "$1" 2>&1)
  if [ -n "$out" ]; then
    printf "%s\n" "$out" > "$results/$(printf "%s" "$1" | tr / _)"
  fi
  exit 0
' sh

collected=$tmpdir/all
cat "$results"/* 2>/dev/null | sed "s#^$topdir/##" |
  sort -t: -k1,1 -k2,2n > "$collected"

ndiag=$(wc -l < "$collected")
nfail=$(cut -d: -f1 "$collected" | sort -u | wc -l)

if [ "$listonly" -eq 1 ]; then
  cut -d: -f1 "$collected" | sort | uniq -c | sort -rn |
    awk '{printf "%6d  %s\n", $1, $2}'
else
  cp "$collected" "$outfile"
  cut -d: -f1 "$collected" | sort | uniq -c | sort -rn |
    awk '{printf "%6d  %s\n", $1, $2}' > "$filefile"
  echo "$outfile:  $ndiag diagnostics"
  echo "$filefile: $nfail files"
fi

if [ "$summary" -eq 1 ]; then
  echo
  echo "Diagnostics by message:"
  sed 's/.*: \(error\|warning\|info\): //' "$collected" |
    sort | uniq -c | sort -rn | awk '{$1=$1; printf "%6d  ", $1;
                                      $1=""; sub(/^ /, ""); print}'
fi

echo
echo "Checked $total files, $nfail failed"

[ "$ndiag" -eq 0 ]
