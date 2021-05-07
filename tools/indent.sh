#!/usr/bin/env bash
############################################################################
# tools/indent.sh
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
# This script uses the Linux 'indent' utility to re-format C source files
# to match the coding style that I use.  It differs from the NuttX coding
# style in that:
#
# 1. I normally put the trailing */ of a multi-line comment on a separate
#    line.  If your C file already has properly formatted comments then
#    using -nfca instead of -fca eliminates that bad behavior
# 2. I usually align things vertically (like '=' in assignments),
# 3. indent.sh puts a bogus blank line at the top of the file,
# 4. I don't like the way it handles nested conditional compilation
#    intermixed with code.  I prefer the preprocessor conditiona tests
#    be all right justified in that case.
# 5. I also indent brackets differently on structures than does this script.
# 6. I normally use no spaces in casts.  indent.sh adds spaces in casts like
#   "(FAR void *)&foo" becomes "(FAR void *) & foo".
# 7. When used with header files, the initial idempotence conditional test
#    causes all preprecessor directives to be indented in the file.  So for
#    header files, you will need to substitute "^#  " with "#" in the
#    converted header file.
#
# You will manually need to check for the issues listed above after
# performing the conversion.

# Constants

advice="Try '$0 -h' for more information"

# Parse inputs

unset filelist
unset outfile
files=none
mode=inplace
fca=-fca

while [ ! -z "${1}" ]; do
  case ${1} in
  -d )
    set -x
    ;;
  -p )
    fca=-nfca
    ;;
  -o )
    shift
    outfile=${1}
    mode=copy
    ;;
  -h )
    echo "$0 is a tool for generation of proper version files for the NuttX build"
    echo ""
    echo "USAGE:"
    echo "  $0 [-d] [-p] -o <out-file> <in-file>"
    echo "  $0 [-d] [-p] <in-file-list>"
    echo "  $0 [-d] -h"
    echo ""
    echo "Where:"
    echo "  -<in-file>"
    echo "    A single, unformatted input file"
    echo "  -<in-file-list>"
    echo "    A list of unformatted input files that will be reformatted in place."
    echo "  -o <out-file>"
    echo "    Write the single, reformatted <in-file> to <out-file>.  <in-file>"
    echo "    will not be modified."
    echo "  -d"
    echo "    Enable script debug"
    echo "  -p"
    echo "    Comments are pre-formatted.  Do not reformat."
    echo "  -h"
    echo "    Show this help message and exit"
    exit 0
    ;;
  * )
    if [ ! -r ${1} ]; then
      echo "Readable ${1} does not exist"
      echo ${advice}
      exit 1
    fi
    if [ -z "${filelist}" ]; then
      filelist="${1}"
      files=single
    else
      filelist="${filelist} ${1}"
      files=multiple
    fi
    ;;
  esac
  shift
done

# Verify that at least one input file was provided

if [ "X${files}" == "Xnone" ]; then
  echo "ERROR: Neither <in-file> nor <in-file-list> provided"
  echo ${advice}
  exit 1
fi

# Options

options="-nbad -bap -bbb -nbbo -nbc -bl -bl2 -bls -nbs -cbi2 -ncdw -nce -ci2 -cli0 -cp40 -ncs -nbfda -nbfde -di1 -nfc1 ${fca} -i2 -l80 -lp -ppi2 -lps -npcs -pmt -nprs -npsl -saf -sai -sbi2 -saw -sc -sob -nss -nut"

# Perform the indentation

if [ "X${mode}" == "Xcopy" ]; then
  if [ "X${files}" == "Xmultiple" ]; then
    echo "ERROR: Only a single <in-file> can be used with the -o option"
    echo ${advice}
    exit 1
  fi
  if [ -f $outfile ]; then
    echo "Removing old $outfile"
    rm $outfile || { echo "Failed to remove $outfile" ; exit 1 ; }
  fi
  indent $options $filelist -o $outfile
else
  indent $options $filelist
fi
