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
USAGE="USAGE: ${SCRIPT_NAME} <bootloader_img> <partition_table_img>"

# Make sure we have the required argument(s)

if [ -z "${1}" ] || [ -z "${2}" ] ; then
  printf "%s requires the bootloader and partition table binary images.\n " "${SCRIPT_NAME}"
  printf "%s\n " "${USAGE}"
  exit 1
fi

BOOTLOADER=${1}
PARTITION_TABLE=${2}

printf "Generating esp32_qemu_image.bin...\n"
printf "\tBootloader: %s\n" "${BOOTLOADER}"
printf "\tPartition Table: %s\n" "${PARTITION_TABLE}"

dd if=/dev/zero bs=1024 count=4096 of=esp32_qemu_image.bin && \
dd if="${BOOTLOADER}" bs=1 seek="$(printf '%d' 0x1000)" of=esp32_qemu_image.bin conv=notrunc && \
dd if="${PARTITION_TABLE}" bs=1 seek="$(printf '%d' 0x8000)" of=esp32_qemu_image.bin conv=notrunc && \
dd if=nuttx.bin bs=1 seek="$(printf '%d' 0x10000)" of=esp32_qemu_image.bin conv=notrunc

if [ ${?} -ne 0 ]; then
  printf "Failed to generate esp32_qemu_image.bin.\n"
  exit 1
fi

printf "Generated esp32_qemu_image.bin successfully!\n"
printf "You can use QEMU for executing it with the following command line:\n"
printf "\tqemu-system-xtensa -nographic -machine esp32 -drive file=esp32_qemu_image.bin,if=mtd,format=raw\n"

echo "esp32_qemu_image.bin" >> nuttx.manifest
