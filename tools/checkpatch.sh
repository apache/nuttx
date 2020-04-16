#!/usr/bin/env bash
# tools/checkpatch.sh
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

fail=0
range=0
spell=0

usage() {
  echo "USAGE: ${0} [options] [list|-]"
  echo ""
  echo "Options:"
  echo "-h"
  echo "-c spell check with codespell(install with: pip install codespell)"
  echo "-r range check only (coupled with -p or -g)"
  echo "-p <patch file names> (default)"
  echo "-g <commit list>"
  echo "-f <file list>"
  echo "-  read standard input mainly used by git pre-commit hook as below:"
  echo "   git diff --cached | ./tools/checkpatch.sh -"
  echo "Where a <commit list> is any syntax supported by git for specifying git revision, see GITREVISIONS(7)"
  echo "Where a <patch file names> is a space separated list of patch file names or wildcard. or *.patch"
}

check_file() {
  $TOOLDIR/nxstyle $@
  ret=$?
  if [ $ret != 0 ]; then
    fail=$ret
  fi

  if [ $spell != 0 ]; then
    codespell -q 7 ${@: -1}
    ret=$?
    if [ $ret != 0 ]; then
      fail=$ret
    fi
  fi
}

check_ranges() {
  while read; do
    if [[ $REPLY =~ ^(\+\+\+\ (b/)?([^[:blank:]]+).*)$ ]]; then
      if [ "$ranges" != "" ]; then
        if [ $range != 0 ]; then
          check_file $ranges $path 2>&1
        else
          check_file $path 2>&1
        fi
      fi
      path=${BASH_REMATCH[3]}
      ranges=""
    elif [[ $REPLY =~ @@\ -[0-9]+(,[0-9]+)?\ \+([0-9]+,[0-9]+)?\ @@.* ]]; then
      ranges+="-r ${BASH_REMATCH[2]} "
    fi
  done
  if [ "$ranges" != "" ]; then
    if [ $range != 0 ]; then
      check_file $ranges $path 2>&1
    else
      check_file $path 2>&1
    fi
  fi
}

check_patch() {
  git apply --check $1
  ret=$?
  if [ $ret != 0 ]; then
    fail=$ret
  else
    git apply $1
    diffs=`cat $1`
    check_ranges <<< "$diffs"
    git apply -R $1
  fi
}

check_commit() {
  diffs=`git show $1`
  check_ranges <<< "$diffs"
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
  -c )
    spell=1
    ;;
  -r )
    range=1
    ;;
  -p )
    shift
    patches=$@
    break
    ;;
  -g )
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
    break
    ;;
  * )
    patches=$@
    break
    ;;
  esac
  shift
done

for patch in $patches; do
  check_patch $patch
done

for commit in $commits; do
  check_commit $commit
done

for file in $files; do
  check_file $file 2>&1
done

exit $fail
