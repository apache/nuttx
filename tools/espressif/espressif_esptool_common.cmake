# ##############################################################################
# tools/espressif/espressif_esptool_common.cmake
#
# SPDX-License-Identifier: Apache-2.0
#
# Licensed to the Apache Software Foundation (ASF) under one or more contributor
# license agreements.  See the NOTICE file distributed with this work for
# additional information regarding copyright ownership.  The ASF licenses this
# file to you under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License.  You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations under
# the License.
#
# ##############################################################################

# cmake-format: off
# ##############################################################################
# Shared Espressif + esptool layout and flash parameters for CMake scripts.
#
# Prerequisites (callers must do this first): - include(nuttx_kconfig.cmake) -
# nuttx_export_kconfig(${DOTCONFIG}) - BINARY_DIR and SOURCE_DIR set (for
# FLASH_ENC_KEY_PATH)
#
# Defines set by including this file:
#   CHIP_SERIES        - Chip string for esptool -c (from CONFIG_ESPRESSIF_CHIP_SERIES)
#   BL_OFFSET          - Bootloader/app base offset for MCUboot & simple boot
#   EFUSE_FLASH_OFFSET - Virtual eFuse blob offset in flash (from Config.mk EFUSE_FLASH_OFFSET)
#   EFUSE_OFFSET       - Alias for EFUSE_FLASH_OFFSET (for mkimage.cmake compatibility)
#   MCUBOOT_APP_OFFSET - NuttX signed image offset (primary or secondary OTA slot)
#   FLASH_SIZE         - Flash size (e.g. 2MB, 4MB), for merge_bin/elf2image -fs
#   FLASH_MODE         - Flash mode (dio, dout, qio, qout), for elf2image -fm only
#   FLASH_FREQ         - Flash speed (e.g. 40m), for elf2image -ff/write_flash -ff
#   FLASH_ENC_KEY_PATH - Resolved path to CONFIG_ESPRESSIF_SECURE_FLASH_ENC_HOST_KEY_NAME
#                        (empty if unset; relative paths checked vs BINARY_DIR then SOURCE_DIR)
#
# ##############################################################################
# cmake-format: on

if(NOT DEFINED CHIP_SERIES)
  if(NOT DEFINED CONFIG_ESPRESSIF_CHIP_SERIES)
    message(
      FATAL_ERROR
        "espressif_esptool_common.cmake: CONFIG_ESPRESSIF_CHIP_SERIES not in .config"
    )
  endif()
  string(REPLACE "\"" "" CHIP_SERIES "${CONFIG_ESPRESSIF_CHIP_SERIES}")
endif()

# Bootloader base (MCUboot placement; simple boot app start on ESP32-P4)
if(CONFIG_ARCH_CHIP_ESP32P4)
  set(BL_OFFSET 0x2000)
else()
  set(BL_OFFSET 0x0000)
endif()

# Virtual eFuse sector offset (tools/espressif/Config.mk EFUSE_FLASH_OFFSET)
if(CONFIG_ESPRESSIF_EFUSE_VIRTUAL_KEEP_IN_FLASH
   AND DEFINED CONFIG_ESPRESSIF_EFUSE_VIRTUAL_KEEP_IN_FLASH_OFFSET)
  set(EFUSE_FLASH_OFFSET ${CONFIG_ESPRESSIF_EFUSE_VIRTUAL_KEEP_IN_FLASH_OFFSET})
else()
  set(EFUSE_FLASH_OFFSET 0x10000)
endif()
set(EFUSE_OFFSET ${EFUSE_FLASH_OFFSET})

# MCUboot application slot (Config.mk APP_OFFSET); default when not MCUboot boot
if(CONFIG_ESPRESSIF_BOOTLOADER_MCUBOOT)
  if(CONFIG_ESPRESSIF_ESPTOOL_TARGET_PRIMARY)
    set(MCUBOOT_APP_OFFSET ${CONFIG_ESPRESSIF_OTA_PRIMARY_SLOT_OFFSET})
  elseif(CONFIG_ESPRESSIF_ESPTOOL_TARGET_SECONDARY)
    set(MCUBOOT_APP_OFFSET ${CONFIG_ESPRESSIF_OTA_SECONDARY_SLOT_OFFSET})
  else()
    message(FATAL_ERROR "Missing MCUBoot slot target: PRIMARY or SECONDARY")
  endif()
endif()

# Flash capacity (merge_bin --fill-flash-size, elf2image -fs)
if(CONFIG_ESPRESSIF_FLASH_2M)
  set(FLASH_SIZE "2MB")
elseif(CONFIG_ESPRESSIF_FLASH_4M)
  set(FLASH_SIZE "4MB")
elseif(CONFIG_ESPRESSIF_FLASH_8M)
  set(FLASH_SIZE "8MB")
elseif(CONFIG_ESPRESSIF_FLASH_16M)
  set(FLASH_SIZE "16MB")
elseif(CONFIG_ESPRESSIF_FLASH_32M)
  set(FLASH_SIZE "32MB")
else()
  set(FLASH_SIZE "4MB")
endif()

# SPI mode for elf2image (Config.mk ELF2IMAGE -fm; write_flash uses -fm dio in
# Config.mk)
if(CONFIG_ESPRESSIF_FLASH_MODE_DIO)
  set(FLASH_MODE "dio")
elseif(CONFIG_ESPRESSIF_FLASH_MODE_DOUT)
  set(FLASH_MODE "dout")
elseif(CONFIG_ESPRESSIF_FLASH_MODE_QIO)
  set(FLASH_MODE "qio")
elseif(CONFIG_ESPRESSIF_FLASH_MODE_QOUT)
  set(FLASH_MODE "qout")
else()
  set(FLASH_MODE "dio")
endif()

if(DEFINED CONFIG_ESPRESSIF_FLASH_FREQ)
  string(REPLACE "\"" "" FLASH_FREQ "${CONFIG_ESPRESSIF_FLASH_FREQ}")
else()
  set(FLASH_FREQ "40m")
endif()

# Host flash encryption key file (Config.mk / espressif_mkimage FLASH_ENC)
if((NOT DEFINED BINARY_DIR) OR (NOT DEFINED SOURCE_DIR))
  message(
    FATAL_ERROR
      "espressif_esptool_common.cmake: BINARY_DIR and SOURCE_DIR must be set before include()"
  )
endif()

set(FLASH_ENC_KEY_PATH "")
if(DEFINED CONFIG_ESPRESSIF_SECURE_FLASH_ENC_HOST_KEY_NAME)
  set(FLASH_ENC_KEY_PATH "${CONFIG_ESPRESSIF_SECURE_FLASH_ENC_HOST_KEY_NAME}")
  if(NOT IS_ABSOLUTE "${FLASH_ENC_KEY_PATH}")
    if(EXISTS "${BINARY_DIR}/${FLASH_ENC_KEY_PATH}")
      set(FLASH_ENC_KEY_PATH "${BINARY_DIR}/${FLASH_ENC_KEY_PATH}")
    elseif(EXISTS "${SOURCE_DIR}/${FLASH_ENC_KEY_PATH}")
      set(FLASH_ENC_KEY_PATH "${SOURCE_DIR}/${FLASH_ENC_KEY_PATH}")
    endif()
  endif()
endif()
