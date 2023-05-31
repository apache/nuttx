#!/usr/bin/env bash
# tools/testbuild.sh
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

WD=$(cd $(dirname $0) && cd .. && pwd)
nuttx=$WD/../nuttx

progname=$0
fail=0
APPSDIR=$WD/../apps
if [ -z $ARTIFACTDIR ]; then
  ARTIFACTDIR=$WD/../buildartifacts
fi
MAKE_FLAGS=-k
EXTRA_FLAGS="EXTRAFLAGS="
MAKE=make
unset testfile
unset HOPTION
unset JOPTION
PRINTLISTONLY=0
GITCLEAN=0
SAVEARTIFACTS=0
CHECKCLEAN=1
CODECHECKER=0
RUN=0

case $(uname -s) in
  Darwin*)
    HOST=Darwin
    ;;
  CYGWIN*)
    HOST=Cygwin
    ;;
  MINGW32*)
    HOST=MinGw
    ;;
  *)

    # Assume linux as a fallback
    HOST=Linux
    ;;
esac

# Include utilities for generating test reports
source $(dirname $0)/test_results.sh

function showusage {
  echo ""
  echo "USAGE: $progname [-l|m|c|g|n] [-d] [-e <extraflags>] [-x] [-j <ncpus>] [-a <appsdir>] [-t <topdir>] [-p] [-G] [--codechecker] <testlist-file>"
  echo "       $progname -h"
  echo ""
  echo "Where:"
  echo "  -l|m|c|g|n selects Linux (l), macOS (m), Cygwin (c),"
  echo "     MSYS/MSYS2 (g) or Windows native (n). Default Linux"
  echo "  -d enables script debug output"
  echo "  -e pass extra c/c++ flags such as -Wno-cpp via make command line"
  echo "  -x exit on build failures"
  echo "  -j <ncpus> passed on to make.  Default:  No -j make option."
  echo "  -a <appsdir> provides the relative path to the apps/ directory.  Default ../apps"
  echo "  -t <topdir> provides the absolute path to top nuttx/ directory.  Default ../nuttx"
  echo "  -p only print the list of configs without running any builds"
  echo "  -A store the build executable artifact in ARTIFACTDIR (defaults to ../buildartifacts"
  echo "  -C Skip tree cleanness check."
  echo "  -G Use \"git clean -xfdq\" instead of \"make distclean\" to clean the tree."
  echo "     This option may speed up the builds. However, note that:"
  echo "       * This assumes that your trees are git based."
  echo "       * This assumes that only nuttx and apps repos need to be cleaned."
  echo "       * If the tree has files not managed by git, they will be removed"
  echo "         as well."
  echo "  -R execute \"run\" script in the config directories if exists."
  echo "  -h will show this help test and terminate"
  echo "  --codechecker enables CodeChecker statically analyze the code."
  echo "  <testlist-file> selects the list of configurations to test.  No default"
  echo ""
  echo "Your PATH variable must include the path to both the build tools and the"
  echo "kconfig-frontends tools"
  echo ""
  exit 1
}

# Parse command line

while [ ! -z "$1" ]; do
  case $1 in
  -l | -m | -c | -g | -n )
    HOPTION+=" $1"
    ;;
  -d )
    set -x
    ;;
  -e )
    shift
    EXTRA_FLAGS+="$1"
    ;;
  -x )
    MAKE_FLAGS='--silent --no-print-directory'
    set -e
    ;;
  -a )
    shift
    APPSDIR="$1"
    ;;
  -j )
    shift
    JOPTION="-j $1"
    ;;
  -t )
    shift
    nuttx="$1"
    ;;
  -p )
    PRINTLISTONLY=1
    ;;
  -G )
    GITCLEAN=1
    ;;
  -A )
    SAVEARTIFACTS=1
    ;;
  -C )
    CHECKCLEAN=0
    ;;
  -R )
    RUN=1
    ;;
  --codechecker )
    CODECHECKER=1
    ;;
  -h )
    showusage
    ;;
  * )
    testfile="$1"
    shift
    break
    ;;
  esac
  shift
done

if [ ! -z "$1" ]; then
  echo "ERROR: Garbage at the end of line"
  showusage
fi

if [ -z "$testfile" ]; then
  echo "ERROR: Missing test list file"
  showusage
fi

if [ ! -r "$testfile" ]; then
  echo "ERROR: No readable file exists at $testfile"
  showusage
fi

if [ ! -d "$nuttx" ]; then
  echo "ERROR: Expected to find nuttx/ at $nuttx"
  showusage
fi

if [ ! -d $APPSDIR ]; then
  echo "ERROR: No directory found at $APPSDIR"
  exit 1
fi

export APPSDIR

testlist=`grep -v -E "^(-|#)" $testfile || true`
blacklist=`grep "^-" $testfile || true`

cd $nuttx || { echo "ERROR: failed to CD to $nuttx"; exit 1; }

function exportandimport {
  # Do nothing until we finish to build the nuttx
  if [ ! -f nuttx ]; then
    return $fail
  fi

  # If CONFIG_BUILD_KERNEL=y does not exist in .config, do nothing
  if ! grep CONFIG_BUILD_KERNEL=y .config 1>/dev/null; then
    return $fail
  fi

  if ! ${MAKE} export ${JOPTION} 1>/dev/null; then
    report_test_failure "Failed to export NuttX."
    fail=1
    return $fail
  fi

  pushd ../apps/

  if ! ./tools/mkimport.sh -z -x ../nuttx/nuttx-export-*.tar.gz 1>/dev/null; then
    report_test_failure "Failed to mkimport NuttX."
    fail=1
    popd
    return $fail
  fi

  if ! ${MAKE} import ${JOPTION} 1>/dev/null; then
    report_test_failure "Failed to import NuttX."
    fail=1
  fi
  popd
  return $fail
}

function compressartifacts {
  local target_path=$1
  local target_name=$2

  pushd $target_path >/dev/null

  tar zcf ${target_name}.tar.gz ${target_name}
  rm -rf ${target_name}

  popd >/dev/null
}

function makefunc {
  if ! ${MAKE} ${MAKE_FLAGS} "${EXTRA_FLAGS}" ${JOPTION} $@ 1>/dev/null; then
    report_test_failure "Make target 'make $@' failed. See CI logs."
    fail=1
  else
    exportandimport
  fi

  return $fail
}

function checkfunc {
  start_test_case checkfunc

  build_cmd="${MAKE} ${MAKE_FLAGS} \"${EXTRA_FLAGS}\" ${JOPTION} 1>/dev/null"

  local config_sub_path=$(echo "$config" | sed "s/:/\//")
  local sub_target_name=${config_sub_path#$(dirname "${config_sub_path}")/}
  local codechecker_dir=${ARTIFACTDIR}/codechecker_logs/${config_sub_path}

  mkdir -p "${codechecker_dir}"

  echo "    Checking NuttX by Codechecker..."
  CodeChecker check -b "${build_cmd}" -o "${codechecker_dir}/logs" -e sensitive --ctu
  codecheck_ret=$?
  echo "    Storing analysis result to CodeChecker..."
  echo "      Generating HTML report..."
  CodeChecker parse --export html --output "${codechecker_dir}/html" "${codechecker_dir}/logs" 1>/dev/null
  echo "      Compressing logs..."
  compressartifacts "$(dirname "${codechecker_dir}")" "${sub_target_name}"

  if [ $codecheck_ret -ne 0 ]; then
    report_test_failure "See CI logs for details."
    # If you need to stop CI, uncomment the following line.
    # fail=1
  else
    report_test_pass
  fi

  end_test_case
  return $fail
}

# Clean up after the last build

function distclean {
  start_test_case distclean
  local tmp_fail=$fail
  fail=0

  echo "  Cleaning..."
  if [ -f .config ]; then
    if [ ${GITCLEAN} -eq 1 ]; then
      report_test_skip
      git -C $nuttx clean -xfdq
      git -C $APPSDIR clean -xfdq
    else
      makefunc distclean

      # Remove .version manually because this file is shipped with
      # the release package and then distclean has to keep it

      rm -f .version

      # Ensure nuttx and apps directory in clean state even with --ignored

      if [ ${CHECKCLEAN} -ne 0 ]; then
        if [ -d $nuttx/.git ] || [ -d $APPSDIR/.git ]; then
          if [[ -n $(git -C $nuttx status --ignored -s) ]]; then
            git -C $nuttx status --ignored
            report_test_failure "OS repo not clean."
            fail=1
          fi
          if [[ -n $(git -C $APPSDIR status --ignored -s) ]]; then
            git -C $APPSDIR status --ignored
            report_test_failure "Apps repo not clean."
            fail=1
          fi
        fi
      fi
    fi
  fi

  if [ ${fail} -eq 0 ]; then
    report_test_pass
    # Restore fail flag.
    fail=$tmp_fail
  fi
  end_test_case

  return $fail
}

# Configure for the next build

function configure {
  start_test_case configure
  local tmp_fail=$fail
  fail=0

  echo "  Configuring..."
  if ! ./tools/configure.sh ${HOPTION} $config ${JOPTION} 1>/dev/null; then
    report_test_failure "Failed to configure."
    fail=1
  fi

  if [ "X$toolchain" != "X" ]; then
    setting=`grep _TOOLCHAIN_ $nuttx/.config | grep -v CONFIG_ARCH_TOOLCHAIN_* | grep =y`
    varname=`echo $setting | cut -d'=' -f1`
    if [ ! -z "$varname" ]; then
      echo "  Disabling $varname"
      kconfig-tweak --file $nuttx/.config -d $varname
    fi

    echo "  Enabling $toolchain"
    kconfig-tweak --file $nuttx/.config -e $toolchain

    makefunc olddefconfig
  fi

  if [ ${fail} -eq 0 ]; then
    report_test_pass
    # Restore fail flag.
    fail=$tmp_fail
  fi
  end_test_case

  return $fail
}

# Perform the next build

function build {
  echo "  Building NuttX..."
  if [ "${CODECHECKER}" -eq 1 ]; then
    checkfunc
  else
    start_test_case build
    local tmp_fail=$fail
    fail=0
    makefunc
    if [ ${fail} -eq 0 ]; then
      report_test_pass
      # Restore fail flag.
      fail=$tmp_fail
    fi
    end_test_case
  fi

  if [ ${SAVEARTIFACTS} -eq 1 ]; then
    artifactconfigdir=$ARTIFACTDIR/$(echo $config | sed "s/:/\//")/
    mkdir -p $artifactconfigdir
    xargs -I "{}" cp "{}" $artifactconfigdir < $nuttx/nuttx.manifest
  fi

  return $fail
}

function refresh {
  # Ensure defconfig in the canonical form
  start_test_case refresh
  local tmp_fail=$fail

  if ! ./tools/refresh.sh --silent $config; then
    report_test_failure "Could not refresh config."
    fail=1
  fi

  # Ensure nuttx and apps directory in clean state

  if [ ${CHECKCLEAN} -ne 0 ]; then
    if [ -d $nuttx/.git ] || [ -d $APPSDIR/.git ]; then
      if [[ -n $(git -C $nuttx status -s) ]]; then
        git -C $nuttx status
        report_test_failure "OS repo not clean."
        fail=1
      fi
      if [[ -n $(git -C $APPSDIR status -s) ]]; then
        git -C $APPSDIR status
        report_test_failure "Apps repo not clean."
        fail=1
      fi
    fi
  fi

  if [ ${fail} -eq 0 ]; then
    report_test_pass
    # Restore fail flag.
    fail=$tmp_fail
  fi
  end_test_case

  return $fail
}

function run {
  start_test_case run

  if [ ${RUN} -ne 0 ]; then
    run_script="$path/run"
    if [ -x $run_script ]; then
      echo "  Running NuttX..."
      if ! $run_script; then
        fail=1
        report_test_failure "Runtime tests failed see logs."
      else
        report_test_pass
      fi
    fi
  else
    report_test_skip
  fi
  end_test_case

  return $fail
}
# Coordinate the steps for the next build test

function dotest {
  start_test_suite $1

  echo "===================================================================================="
  config=`echo $1 | cut -d',' -f1`
  check=${HOST},${config/\//:}

  skip=0
  for re in $blacklist; do
    if [[ "${check}" =~ ${re:1}$ ]]; then
      echo "Skipping: $1"
      skip=1
    fi
  done

  echo "Configuration/Tool: $1"
  if [ ${PRINTLISTONLY} -eq 1 ]; then
    return
  fi
  start_test_case configure_tool
  config_err=0

  # Parse the next line

  configdir=`echo $config | cut -s -d':' -f2`
  if [ -z "${configdir}" ]; then
    configdir=`echo $config | cut -s -d'/' -f2`
    if [ -z "${configdir}" ]; then
      local message="ERROR: Malformed configuration: ${config}"
      echo $message
      report_test_error $message
      config_err=1
      showusage
    else
      boarddir=`echo $config | cut -d'/' -f1`
    fi
  else
    boarddir=`echo $config | cut -d':' -f1`
  fi

  path=$nuttx/boards/*/*/$boarddir/configs/$configdir
  if [ ! -r $path/defconfig ]; then
    local message="ERROR: no configuration found at $path"
    echo $message
    report_test_error $message
    config_err=1
    showusage
  fi

  unset toolchain
  if [ "X$config" != "X$1" ]; then
    toolchain=`echo $1 | cut -d',' -f2`
    if [ -z "$toolchain" ]; then
      echo "  Warning: no tool configuration"
    fi
  fi

  if [ ${config_err} -eq 0 ]; then
    report_test_pass
  fi
  end_test_case
  # Perform the build test

  echo "------------------------------------------------------------------------------------"
  distclean
  configure
  if [ ${skip} -ne 1 ]; then
    build
    run
  fi
  refresh
  end_test_suite
}

# Perform the build test for each entry in the test list file
start_test_report $testfile
for line in $testlist; do
  firstch=${line:0:1}
  if [ "X$firstch" == "X/" ]; then
    dir=`echo $line | cut -d',' -f1`
    list=`find boards$dir -name defconfig | cut -d'/' -f4,6`
    for i in ${list}; do
      dotest $i${line/"$dir"/}
    done
  else
    dotest $line
  fi
done
end_test_suites

echo "===================================================================================="

exit $fail
