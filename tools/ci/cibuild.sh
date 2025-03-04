#!/usr/bin/env bash
############################################################################
# tools/ci/cibuild.sh
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
set -e
set -o xtrace


CID=$(cd "$(dirname "$0")" && pwd)
CIWORKSPACE=$(cd "${CID}"/../../../ && pwd -P)
CIPLAT=${CIWORKSPACE}/nuttx/tools/ci/platforms
nuttx=${CIWORKSPACE}/nuttx
apps=${CIWORKSPACE}/apps

os=$(uname -s)
osarch=$(uname -m)
if [ -f /etc/os-release ]; then
  osname=$(grep "^ID=" /etc/os-release | cut -d'=' -f2 | tr -d '"')
else
  osname=${os}
fi

function to_do {
  echo ""
  echo "NuttX TODO: $1"
  echo "The $1 platform does not appear to have been added to this project."
  echo ""
  exit 1
}

function install_tools {
  export NUTTXTOOLS=${CIWORKSPACE}/tools
  mkdir -p "${NUTTXTOOLS}"

  case ${osname} in
    alpine)
      to_do "alpine"
      ;;
    arch)
      to_do "arch"
      ;;
    CYGWIN*)
      to_do "CYGWIN"
      ;;
    debian)
      to_do "debian"
      ;;
    fedora)
      to_do "fedora"
      ;;
    freebsd)
      to_do "freebsd"
      ;;
    Darwin)
      if [ "X$osarch" == "Xx86_64" ]; then
        "${CIPLAT}"/darwin.sh
      else
        "${CIPLAT}"/darwin_arm64.sh
      fi
      ;;
    Linux)
      "${CIPLAT}"/linux.sh
      ;;
    manjaro)
      to_do "manjaro"
      ;;
    msys2)
      "${CIPLAT}"/msys2.sh
      ;;
    ubuntu)
      "${CIPLAT}"/ubuntu.sh
      ;;
    *)
      to_do "unknown"
      ;;
  esac

  source "${CIWORKSPACE}"/tools/env.sh
}

function usage {
  echo ""
  echo "USAGE: $0 [-i] [-s] [-c] [-*] <testlist>"
  echo "       $0 -h"
  echo ""
  echo "Where:"
  echo "  -i install tools"
  echo "  -s setup repos"
  echo "  -c enable ccache"
  echo "  -* support all options in testbuild.sh"
  echo "  -h will show this help text and terminate"
  echo "  <testlist> select testlist file"
  echo ""
  exit 1
}

function enable_ccache {
  export CCACHE_DIR="${CIWORKSPACE}"/tools/ccache
}

function setup_repos {
  pushd .
  if [ -d "${nuttx}" ]; then
    cd "${nuttx}"; git pull
  else
    git clone https://github.com/apache/nuttx.git "${nuttx}"
    cd "${nuttx}"
  fi
  git log -1

  if [ -d "${apps}" ]; then
    cd "${apps}"; git pull
  else
    git clone https://github.com/apache/nuttx-apps.git "${apps}"
    cd "${apps}"
  fi
  git log -1
  popd
}

function run_builds {
  local ncpus
  if [ "X$osname" == "XDarwin" ]; then
    ncpus=$(sysctl -n hw.ncpu)
  else
    ncpus=$(grep -c ^processor /proc/cpuinfo)
  fi

  if [ "X$osname" == "Xmsys2" ]; then
    export MAKEFLAGS="-j"
  else
    options+="-j ${ncpus}"
  fi

  for build in "${builds[@]}"; do
    "${nuttx}"/tools/testbuild.sh ${options} -e "-Wno-cpp -Werror" "${build}"
  done

  if [ -d "${CCACHE_DIR}" ]; then
    # Print a summary of configuration and statistics counters
    ccache -s
  fi
}

if [ -z "$1" ]; then
   usage
fi

while [ -n "$1" ]; do
  case "$1" in
  -h )
    usage
    ;;
  -i )
    install_tools
    ;;
  -c )
    enable_ccache
    ;;
  -s )
    setup_repos
    ;;
  -* )
    options+="$1 "
    ;;
  * )
    builds=( "$@" )
    break
    ;;
  esac
  shift
done

run_builds
