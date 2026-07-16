#!/bin/sh
############################################################################
# tools/ameba/env.sh
#
# Source this before building the Realtek Ameba (RTL872x) boards:
#
#   . tools/ameba/env.sh
#   cmake -B build -DBOARD_CONFIG=pke8721daf:nsh -GNinja && cmake --build build
#     -- or --
#   ./tools/configure.sh pke8721daf:nsh && make
#
# It resolves everything the build needs and exports it into the current shell,
# reusing the same helper scripts the make build already uses (so make and cmake
# share one setup path):
#
#   1. AMEBA_SDK   -- the ameba-rtos SDK checkout (external $AMEBA_SDK if valid,
#                     else the shared in-tree one, fetched on demand).
#   2. asdk toolchain -- the SDK-pinned arm-none-eabi (asdk) compiler, fetched if
#                     absent and prepended to PATH.  This is why CMake works with
#                     no per-chip toolchain hook: the compiler is simply on PATH
#                     before `cmake` probes it, exactly like every other board.
#   3. prebuilts + venv -- ninja/cmake bundle + the SDK python venv, prepended to
#                     PATH for the image packaging steps.
#
# The make build does NOT require sourcing this (its Make.defs still auto-fetches
# everything); it is optional there and REQUIRED for the CMake build.
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

# --- Must be sourced (it exports into the caller's shell) ------------------

_ameba_self="${BASH_SOURCE:-${ZSH_VERSION:+${(%):-%x}}}"
[ -n "$_ameba_self" ] || _ameba_self="$0"
case "$0" in
  *env.sh)
    echo "ameba env.sh: source it, do not execute -- '. tools/ameba/env.sh'" >&2
    exit 1 ;;
esac

_ameba_here="$(CDPATH= cd "$(dirname "$_ameba_self")" && pwd)"
AMEBA_NUTTX="$(CDPATH= cd "$_ameba_here/../.." && pwd)"
AMEBA_COMMON="$AMEBA_NUTTX/arch/arm/src/common/ameba"
AMEBA_TOOLS="$AMEBA_COMMON/tools"
AMEBA_TOOLCHAIN_DIR="${AMEBA_TOOLCHAIN_DIR:-$HOME/rtk-toolchain}"

# --- 1. Resolve / fetch the SDK (pin comes from ameba_sdk.mk, single source) --

if [ -n "$AMEBA_SDK" ] && [ -d "$AMEBA_SDK/component/soc" ]; then
  : # external checkout, use as-is
else
  AMEBA_SDK="$AMEBA_COMMON/ameba-rtos"
  if [ ! -d "$AMEBA_SDK/component/soc" ]; then
    _url=$(sed -n 's/^[[:space:]]*AMEBA_SDK_URL[[:space:]]*=[[:space:]]*\([^[:space:]]*\).*/\1/p' \
           "$AMEBA_COMMON/ameba_sdk.mk" | head -1)
    _ver=$(sed -n 's/^[[:space:]]*AMEBA_SDK_VERSION[[:space:]]*=[[:space:]]*\([0-9a-fA-F]*\).*/\1/p' \
           "$AMEBA_COMMON/ameba_sdk.mk" | head -1)
    sh "$AMEBA_TOOLS/ameba_fetch_sdk.sh" "$_url" "$_ver" "$AMEBA_SDK" || return 1 2>/dev/null || exit 1
  fi
fi
export AMEBA_SDK
export AMEBA_TOOLCHAIN_DIR

# --- 2. asdk toolchain: fetch if absent, prepend to PATH, pin GCCVER ---------

sh "$AMEBA_TOOLS/ameba_fetch_toolchain.sh" "$AMEBA_SDK" "$AMEBA_TOOLCHAIN_DIR" \
  || return 1 2>/dev/null || exit 1

_ver=$(sed -n 's/.*v_ASDK_VER[ \t][ \t]*\([0-9.][0-9.]*\).*/\1/p' \
       "$AMEBA_SDK/cmake/global_define.cmake")
_build=$(sed -n 's/.*ToolChainVerMinor[ \t][ \t]*\([0-9][0-9]*\).*/\1/p' \
         "$AMEBA_SDK/cmake/toolchain/ameba-toolchain-asdk-$_ver.cmake")
_asdk_bin="$AMEBA_TOOLCHAIN_DIR/asdk-$_ver-$_build/linux/newlib/bin"
export GCCVER="${_ver%%.*}"

# --- 3. prebuilts (ninja/cmake) + python venv (SDK env) ----------------------

sh "$AMEBA_TOOLS/ameba_setup_env.sh" "$AMEBA_SDK" "$AMEBA_TOOLCHAIN_DIR" \
  || return 1 2>/dev/null || exit 1

_pbv=$(sed -n 's/^PREBUILTS_VERSION=//p' "$AMEBA_SDK/env.sh" | head -1)
_prebuilts_bin="$AMEBA_TOOLCHAIN_DIR/prebuilts-linux-$_pbv/bin"
_venv_bin="$(cat "$AMEBA_SDK/.amebapy/bindir" 2>/dev/null)"

# --- 4. Prepend to PATH (idempotent) -----------------------------------------

for _d in "$_venv_bin" "$_prebuilts_bin" "$_asdk_bin"; do
  [ -n "$_d" ] || continue
  case ":$PATH:" in
    *":$_d:"*) ;;
    *) PATH="$_d:$PATH" ;;
  esac
done
export PATH

echo "ameba: environment ready"
echo "  AMEBA_SDK = $AMEBA_SDK"
echo "  asdk      = $_asdk_bin"
echo "  now build with cmake (source once) or make"
