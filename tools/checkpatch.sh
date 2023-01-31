#!/usr/bin/env bash
# tools/checkpatch.sh
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.
# The ASF licenses this file to you under the Apache License, Version 2.0
# (the "License"); you may not use this file except in compliance with
# the License.  You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

TOOLDIR=$(dirname $0)

check=check_patch
fail=0
range=0
spell=0
encoding=0
message=0

usage() {
  echo "USAGE: ${0} [options] [list|-]"
  echo ""
  echo "Options:"
  echo "-h"
  echo "-c spell check with codespell (install with: pip install codespell)"
  echo "-u encoding check with cvt2utf (install with: pip install cvt2utf)"
  echo "-r range check only (coupled with -p or -g)"
  echo "-p <patch file names> (default)"
  echo "-m Change-Id check in commit message (coupled with -g)"
  echo "-g <commit list>"
  echo "-f <file list>"
  echo "-  read standard input mainly used by git pre-commit hook as below:"
  echo "   git diff --cached | ./tools/checkpatch.sh -"
  echo "Where a <commit list> is any syntax supported by git for specifying git revision, see GITREVISIONS(7)"
  echo "Where a <patch file names> is a space separated list of patch file names or wildcard. or *.patch"

  exit $@
}

is_rust_file() {
  file_ext=${@##*.}
  file_ext_r=${file_ext/R/r}
  file_ext_rs=${file_ext_r/S/s}

  if [ "$file_ext_rs" == "rs" ]; then
    echo 1
  else
    echo 0
  fi
}

check_file() {
  if [ -x $@ ]; then
    case $@ in
    *.bat | *.sh | *.py)
      ;;
    *)
      echo "$@: error: execute permissions detected!"
      fail=1
      ;;
    esac
  fi

  if [ "$(is_rust_file $@)" == "1" ]; then
    if ! command -v rustfmt &> /dev/null; then
      fail=1
    elif ! rustfmt --edition 2021 --check $@ 2>&1; then
      fail=1
    fi
  elif ! $TOOLDIR/nxstyle $@ 2>&1; then
    fail=1
  fi

  if [ $spell != 0 ]; then
    if ! codespell -q 7 ${@: -1}; then
      fail=1
    fi
  fi

  if [ $encoding != 0 ]; then
    md5="$(md5sum $@)"
    cvt2utf convert --nobak "$@" &> /dev/null
    if [ "$md5" != "$(md5sum $@)" ]; then
      echo "$@: error: Non-UTF8 characters detected!"
      fail=1
    fi
  fi
}

check_ranges() {
  while read; do
    if [[ $REPLY =~ ^(\+\+\+\ (b/)?([^[:blank:]]+).*)$ ]]; then
      if [ "$ranges" != "" ]; then
        if [ $range != 0 ]; then
          check_file $ranges $path
        else
          check_file $path
        fi
      fi
      path=$(realpath "${BASH_REMATCH[3]}")
      ranges=""
    elif [[ $REPLY =~ @@\ -[0-9]+(,[0-9]+)?\ \+([0-9]+,[0-9]+)?\ @@.* ]]; then
      ranges+="-r ${BASH_REMATCH[2]} "
    fi
  done
  if [ "$ranges" != "" ]; then
    if [ $range != 0 ]; then
      check_file $ranges $path
    else
      check_file $path
    fi
  fi
}

check_patch() {
  if ! git apply --check $1; then
    fail=1
  else
    git apply $1
    diffs=`cat $1`
    check_ranges <<< "$diffs"
    git apply -R $1
  fi
}

check_msg() {
  while read; do
    if [[ $REPLY =~  ^Change-Id ]]; then
      echo "Remove Gerrit Change-ID's before submitting upstream"
      fail=1
    fi
  done
}

check_commit() {
  if [ $message != 0 ]; then
    msg=`git show -s --format=%B $1`
    check_msg <<< "$msg"
  fi
  diffs=`git diff $1`
  check_ranges <<< "$diffs"
}

make -C $TOOLDIR -f Makefile.host nxstyle 1>/dev/null

if [ -z "$1" ]; then
  usage
  exit 0
fi

while [ ! -z "$1" ]; do
  case "$1" in
  - )
    check_ranges
    ;;
  -c )
    spell=1
    ;;
  -u )
    encoding=1
    ;;
  -f )
    check=check_file
    ;;
  -m )
    message=1
    ;;
  -g )
    check=check_commit
    ;;
  -h )
    usage 0
    ;;
  -p )
    check=check_patch
    ;;
  -r )
    range=1
    ;;
  -* )
    usage 1
    ;;
  * )
    break
    ;;
  esac
  shift
done

for arg in $@; do
  $check $arg
done

exit $fail
