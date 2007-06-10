#!/bin/sh
# mkdeps.sh
#
#   Copyright (C) 2007 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name Gregory Nutt nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
#
# Usage:

function show_usage ()
{
  echo ""
  echo "$progname  [OPTIONS] CC -- CFLAGS -- file [file [file...]]"
  echo ""
  echo "Where:"
  echo "  CC"
  echo "    A variable number of arguments that define how to execute the compiler"
  echo "  CFLAGS"
  echo "    The compiler compilation flags"
  echo "  file"
  echo "    One or more C files whose dependencies will be checked.  Each file is expected"
  echo "    to reside in the current directory unless --dep-path is provided on the command line"
  echo ""
  echo "And [OPTIONS] include:"
  echo "  --dep-debug"
  echo "    Enable script debug"
  echo "  --dep-path <path>"
  echo "    Do not look in the current directory for the file.  Instead, look in <path> to see"
  echo "    if the file resides there.  --dep-path may be used multiple times to specifid"
  echo "    multiple alternative location"
  echo "  --help"
  echo "    Shows this message and exits"
  exit 1
}

function dodep ()
{
  unset fullpath
  if [ -z "$altpath" ]; then
    if [ -r $1 ]; then
      fullpath=$1
    else
      echo "# ERROR: No readable file at $1"
      show_usage
    fi
  else
    for path in $altpath; do
      tmppath=$path/$1
      if [ -r $tmppath ]; then
        fullpath=$tmppath
        break;
      fi
    done
    if [ -z "$fullpath" ]; then
      echo "# ERROR: No readable file for $1 found at any location"
      show_usage
    fi
  fi

  $cc -M $cflags $fullpath || \
    { echo "# ERROR: $cc -M $cflags $fullpath FAILED" ; exit 4 ; }
}

unset cc
unset cflags
unset files
unset args
unset altpath

# Accumulate CFLAGS up to "--"
progname=$0
while [ ! -z "$1" ]; do
  case $1 in
  -- )
    cc=$cflags
    cflags=$args
    args=
    ;;
  --dep-debug )
    if [ -z "$args" ]; then
      set -x
    else
      args="$args $1"
    fi
    ;;
  --dep-path )
    if [ -z "$args" ]; then
      shift
      altpath="$altpath $1"
    else
      args="$args $1"
    fi
    ;;
  --help )
    show_usage
    ;;
  *)
    args="$args $1"
    ;;
  esac
  shift
done
files=$args

if [ -z "$cc" ]; then
  echo "ERROR: No compiler specified"
  show_usage
  exit 1
fi

if [ -z "$files" ]; then
  echo "No files specified"
  show_usage
  exit 2
fi

for file in $files ; do
    dodep $file
done

