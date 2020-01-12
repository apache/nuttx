#!/usr/bin/env bash
#
# Copyright (C) 2019 Xiaomi
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

TOOLDIR=$(dirname $0)

usage() {
  echo "USAGE: ${0} [options] [list|-]"
  echo ""
  echo "Options:"
  echo "-h"
  echo "-p <patch list> (default)"
  echo "-c <commit list>"
  echo "-f <file list>"
  echo "-  read standard input mainly used by git pre-commit hook as below:"
  echo "   git diff --cached | ./tools/checkpatch.sh -"
}

check_file() {
  $TOOLDIR/nxstyle $@ 2>&1
}

check_ranges() {
  local fail=0

  while read; do
    if [[ $REPLY =~ \+\+\+\ (b/)?([^[:blank:]]+).* ]]; then
      if [ "$ranges" != "" ]; then
        check_file $ranges $path 2>&1
        if [ $? != 0 ]; then
          fail=1
        fi
      fi
      path=${BASH_REMATCH[2]}
      ranges=""
    elif [[ $REPLY =~ @@\ -[0-9]+(,[0-9]+)?\ \+([0-9]+,[0-9]+)?\ @@.* ]]; then
      ranges+="-r ${BASH_REMATCH[2]} "
    fi
  done
  if [ "$ranges" != "" ]; then
    check_file $ranges $path 2>&1
    if [ $? != 0 ]; then
      fail=1
    fi
  fi
  if [ $fail = 1 ]; then
    exit 1
  fi
}

check_patch() {
  git apply --check $1
  if [ $? != 0 ]; then
    exit 1
  fi
  git apply $1
  cat $1 | check_ranges
  git apply -R $1
}

check_commit() {
  git show $1 | check_ranges
}

make -C $TOOLDIR -f Makefile.host nxstyle 1>/dev/null

if [ -z "$1" ]; then
  usage
  exit 0
fi

while [ ! -z "$1" ]; do
  case "$1" in
  -h )
    usage
    exit 0
    ;;
  -p )
    shift
    patches=$@
    break
    ;;
  -c )
    shift
    commits=$@
    break
    ;;
  -f )
    shift
    files=$@
    break
    ;;
  - )
    check_ranges
    exit 0
    ;;
  * )
    patches=$@
    break
    ;;
  esac
done

for patch in $patches; do
  check_patch $patch
done

for commit in $commits; do
  check_commit $commit
done

for file in $files; do
  check_file $file
done
