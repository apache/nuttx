#!/usr/bin/env bash
############################################################################
# tools/esp32/mk_qemu_img.sh
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

SCRIPT_NAME=$(basename "${0}")

BOOTLOADER_IMG=""
PARTITION_IMG=""
BOOTLOADER_OFFSET=0x1000
PARTITION_OFFSET=0x8000
NUTTX_OFFSET=0x10000
NUTTX_IMG="nuttx.bin"
FLASH_IMG="esp32_qemu_img.bin"

usage() {
  echo ""
  echo "USAGE: ${SCRIPT_NAME} [-h] -b <bootloader> -p <partition_table> [-n <nuttx>] [-i <image_name>]"
  echo ""
  echo "Where:"
  echo "  -b <bootloader> path to the bootloader image"
  echo "  -p <partition_table> path to the partition table image"
  echo "  -n <nuttx> path to the nuttx image (default nuttx.bin)"
  echo "  -i <image_name> name of the resulting image (default esp32_qemu_img.bin)"
  echo "  -h will show this help and terminate"
  echo ""
}

imgappend() {
  dd of="${1}" if="${2}" bs=1 seek="$(printf '%d' ${3})" conv=notrunc status=none
}

while [ -n "${1}" ]; do
  case "${1}" in
  -b )
    shift
    BOOTLOADER_IMG=${1}
    ;;
  -p )
    shift
    PARTITION_IMG=${1}
    ;;
  -n )
    shift
    NUTTX_IMG=${1}
    ;;
  -i )
    shift
    FLASH_IMG=${1}
    ;;
  -h )
    usage
    exit 0
    ;;
  *)
    usage
    exit 1
    ;;
  esac
  shift
done

# Make sure we have the required argument(s)

if [ -z "${BOOTLOADER_IMG}" ] || [ -z "${PARTITION_IMG}" ] ; then
  echo ""
  echo "${SCRIPT_NAME}: Missing bootloader and partition table binary images."
  usage
  exit 1
fi

printf "Generating %s...\n" "${FLASH_IMG}"
printf "\tBootloader: %s\n" "${BOOTLOADER_IMG}"
printf "\tPartition Table: %s\n" "${PARTITION_IMG}"

dd if=/dev/zero bs=1024 count=4096 of="${FLASH_IMG}" status=none
imgappend ${FLASH_IMG} ${BOOTLOADER_IMG} ${BOOTLOADER_OFFSET}
imgappend ${FLASH_IMG} ${PARTITION_IMG} ${PARTITION_OFFSET}
imgappend ${FLASH_IMG} ${NUTTX_IMG} ${NUTTX_OFFSET}

if [ ${?} -ne 0 ]; then
  printf "Failed to generate %s!\n" "${FLASH_IMG}"
  exit 1
fi

printf "Generated %s successfully!\n" "${FLASH_IMG}"
printf "You can run it with QEMU using:\n"
printf "\tqemu-system-xtensa -nographic -machine esp32 -drive file=%s,if=mtd,format=raw\n" "${FLASH_IMG}"

echo "${FLASH_IMG}" >> nuttx.manifest
