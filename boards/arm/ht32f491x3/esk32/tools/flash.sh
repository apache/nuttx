#!/usr/bin/env bash
############################################################################
# boards/arm/ht32f491x3/esk32/tools/flash.sh
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

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TOPDIR="$(cd "${SCRIPT_DIR}/../../../../.." && pwd)"

DEFAULT_BIN="${TOPDIR}/nuttx.bin"
WINDOWS_SETUP="Windows 10 Pro with WSL2"
HT32_IDE_VERSION="HT32-IDE 1.0.6 (Build Date: 2025/12/04)"
HT32_IDE_ROOT="/mnt/c/Program Files (x86)/Holtek HT32 Series/HT32-IDE"
OPENOCD_PACKAGE="xPack OpenOCD 0.11.0-4"
OPENOCD_ROOT="${HT32_IDE_ROOT}/xPack/xpack-openocd-0.11.0-4"
OPENOCD_EXE="${OPENOCD_ROOT}/bin/openocd.exe"
SCRIPTS_DIR="${OPENOCD_ROOT}/scripts"
FLASH_LOADER="${OPENOCD_ROOT}/FlashLoader/HT32F491x3_256.HLM"
DEVICE_NAME="HT32F49163_100LQFP"
FLASH_BASE="0x08000000"
FLASH_END="0x0803FFFF"
SRAM_BASE="0x20000000"
WORKAREA_SIZE="0xC000"
BIN_PATH="${DEFAULT_BIN}"
DRY_RUN=0

print_assumptions() {
  cat <<EOF
############################################################################
# Assumptions:
#
#   - ${WINDOWS_SETUP}; the Windows C: drive is available in WSL at /mnt/c
#   - This is the WSL backend; use flash.py from the same directory for
#     automatic backend selection, or run this script directly from WSL2
#   - ${HT32_IDE_VERSION} installed at:
#       ${HT32_IDE_ROOT}
#   - ${OPENOCD_PACKAGE} available at:
#       ${OPENOCD_ROOT}
#   - Holtek HT-Link probe using interface/htlink.cfg
#   - ESK32 board with ${DEVICE_NAME} and FlashLoader/HT32F491x3_256.HLM
#
# Update this script if any of the above are not true.
#
############################################################################

EOF
}

usage() {
  cat <<EOF
Usage: $0 [options]

Options:
  --bin PATH         Binary to flash. Default: ${DEFAULT_BIN}
  --device NAME      Holtek expected device name. Default: ${DEVICE_NAME}
  --openocd-root DIR Holtek xPack OpenOCD root.
  --dry-run          Print the OpenOCD command without executing it.
  --help             Show this help.

Examples:
  $0
  $0 --dry-run
  $0 --device HT32F49163_100LQFP
EOF
}

print_assumptions

while (($# > 0)); do
  case "$1" in
    --bin)
      BIN_PATH="$2"
      shift 2
      ;;
    --device)
      DEVICE_NAME="$2"
      shift 2
      ;;
    --openocd-root)
      OPENOCD_ROOT="$2"
      OPENOCD_EXE="${OPENOCD_ROOT}/bin/openocd.exe"
      SCRIPTS_DIR="${OPENOCD_ROOT}/scripts"
      FLASH_LOADER="${OPENOCD_ROOT}/FlashLoader/HT32F491x3_256.HLM"
      shift 2
      ;;
    --dry-run)
      DRY_RUN=1
      shift
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
done

if [[ "${DRY_RUN}" -eq 0 ]]; then
  if [[ ! -f "${BIN_PATH}" ]]; then
    echo "Binary not found: ${BIN_PATH}" >&2
    exit 1
  fi

  if [[ ! -f "${OPENOCD_EXE}" ]]; then
    echo "OpenOCD executable not found: ${OPENOCD_EXE}" >&2
    exit 1
  fi

  if [[ ! -f "${FLASH_LOADER}" ]]; then
    echo "Flash loader not found: ${FLASH_LOADER}" >&2
    exit 1
  fi
fi

BIN_WIN="$(wslpath -m "${BIN_PATH}")"
SCRIPTS_WIN="$(wslpath -m "${SCRIPTS_DIR}")"
LOADER_WIN="$(wslpath -m "${FLASH_LOADER}")"

OPENOCD_CMD=(
  "${OPENOCD_EXE}"
  -s "${SCRIPTS_WIN}"
  -c "hlm_SRAM ${SRAM_BASE} ${WORKAREA_SIZE}"
  -c "hlm_loader {${LOADER_WIN}} ${FLASH_BASE} ${FLASH_END}"
  -c "ht_flags erase_sector"
  -c "set WORKAREASIZE ${WORKAREA_SIZE}"
  -f interface/htlink.cfg
  -f target/HLM491x3.cfg
  -c "set_expected_name ${DEVICE_NAME}"
  -c "program ${BIN_WIN} verify reset exit ${FLASH_BASE}"
)

printf 'TOPDIR      : %s\n' "${TOPDIR}"
printf 'Binary      : %s\n' "${BIN_PATH}"
printf 'Device      : %s\n' "${DEVICE_NAME}"
printf 'OpenOCD     : %s\n' "${OPENOCD_EXE}"
printf 'Flash loader: %s\n' "${FLASH_LOADER}"

if [[ "${DRY_RUN}" -eq 1 ]]; then
  [[ -f "${BIN_PATH}" ]] || printf 'Warning     : binary not found yet\n'
  [[ -f "${OPENOCD_EXE}" ]] || printf 'Warning     : OpenOCD executable not found\n'
  [[ -f "${FLASH_LOADER}" ]] || printf 'Warning     : flash loader not found\n'
  printf 'Command     :'
  printf ' %q' "${OPENOCD_CMD[@]}"
  printf '\n'
  exit 0
fi

exec "${OPENOCD_CMD[@]}"
