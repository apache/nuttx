# ##############################################################################
# tools/espressif/espressif_flash.cmake
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
#
# Flash NuttX images via esptool.py (CMake equivalent of
# tools/espressif/Config.mk FLASH target).
#
# Typical use (RISC-V Espressif boards built with CMake):
#
# export ESPTOOL_PORT=/dev/ttyUSB0   # or set for a single command cmake --build
# <builddir> -t flash
#
# The ``flash`` target is defined in
# arch/risc-v/src/common/espressif/CMakeLists.txt. It runs this file via cmake
# -P with BINARY_DIR, SOURCE_DIR, and NUTTX_DIR set; the serial port must be set
# in the ESPTOOL_PORT environment variable (unset or empty is a fatal error
# unless you pass a non-empty -DESPTOOL_PORT to cmake -P).
#
# Standalone / scripting:
#
# ESPTOOL_PORT=/dev/ttyUSB0 cmake -P
# <NUTTX>/tools/espressif/espressif_flash.cmake \ -DBINARY_DIR=<builddir>
# -DSOURCE_DIR=<cmake-source-dir> \ -DNUTTX_DIR=<nuttx-tree>
#
# A non-empty -DESPTOOL_PORT on cmake -P skips the environment requirement.
#
# If -DNUTTX_DIR is omitted, it defaults to SOURCE_DIR. mcuboot-<chip>.bin is
# read from NUTTX_DIR.
#
# Optional -D variables: ESPTOOL_BAUD     Serial baud (default: 921600, same as
# Config.mk)
#
# Required for this script: BINARY_DIR, SOURCE_DIR, and a serial port via
# non-empty ESPTOOL_PORT in the environment or non-empty -DESPTOOL_PORT for
# cmake -P.
#
# ##############################################################################

cmake_minimum_required(VERSION 3.16)

if(NOT DEFINED BINARY_DIR)
  message(FATAL_ERROR "BINARY_DIR not defined")
endif()

if(NOT DEFINED SOURCE_DIR)
  message(FATAL_ERROR "SOURCE_DIR not defined")
endif()

if(NOT DEFINED NUTTX_DIR)
  set(NUTTX_DIR "${SOURCE_DIR}")
endif()

if((NOT DEFINED ESPTOOL_PORT) OR ("${ESPTOOL_PORT}" STREQUAL ""))
  if("$ENV{ESPTOOL_PORT}" STREQUAL "")
    message(
      FATAL_ERROR
        "FLASH error: ESPTOOL_PORT environment variable is not set.\n"
        "Example: ESPTOOL_PORT=/dev/ttyUSB0 cmake --build <builddir> -t flash\n"
        "Or with cmake -P: pass a non-empty -DESPTOOL_PORT=... (and -DBINARY_DIR=... "
        "-DSOURCE_DIR=... [ -DESPTOOL_BAUD=... ] [ -DNUTTX_DIR=... ])")
  endif()
  set(ESPTOOL_PORT "$ENV{ESPTOOL_PORT}")
endif()

find_program(ESPTOOL esptool esptool.py)
find_program(PYTHON3 python3)

if(NOT ESPTOOL)
  message(FATAL_ERROR "esptool / esptool.py not found in PATH")
endif()

if(NOT DEFINED ESPTOOL_BAUD)
  set(ESPTOOL_BAUD 921600)
endif()

include(${SOURCE_DIR}/cmake/nuttx_kconfig.cmake)

set(DOTCONFIG "${BINARY_DIR}/.config")
if(NOT EXISTS ${DOTCONFIG})
  message(FATAL_ERROR ".config not found at ${DOTCONFIG}")
endif()

nuttx_export_kconfig(${DOTCONFIG})

include(${SOURCE_DIR}/tools/espressif/espressif_esptool_common.cmake)

if(CONFIG_ESPRESSIF_FLASH_DETECT)
  set(ESPTOOL_WRITEFLASH_ARGS -fs detect -fm dio -ff ${FLASH_FREQ})
else()
  set(ESPTOOL_WRITEFLASH_ARGS -fs ${FLASH_SIZE} -fm dio -ff ${FLASH_FREQ})
endif()

if(CONFIG_ESPRESSIF_SECURE_FLASH_ENC_FLASH_DEVICE_ENCRYPTED)
  list(APPEND ESPTOOL_WRITEFLASH_ARGS --encrypt)
endif()

set(ESPTOOL_MIN_VERSION 4.8.0)
if(EXISTS "${SOURCE_DIR}/tools/espressif/check_esptool.py")
  execute_process(
    COMMAND ${PYTHON3} ${SOURCE_DIR}/tools/espressif/check_esptool.py -v
            ${ESPTOOL_MIN_VERSION}
    RESULT_VARIABLE ESPTOOL_CHECK_RESULT
    OUTPUT_QUIET ERROR_QUIET)
  if(NOT ESPTOOL_CHECK_RESULT EQUAL 0)
    message(
      WARNING "esptool.py version ${ESPTOOL_MIN_VERSION} or higher recommended")
  endif()
endif()

# ESPTOOL_OPTS: -c CHIP_SERIES -p PORT -b BAUD [ --no-stub ]
set(ESPTOOL_OPTS -c ${CHIP_SERIES} -p ${ESPTOOL_PORT} -b ${ESPTOOL_BAUD})
if(CONFIG_ESPRESSIF_ESPTOOLPY_NO_STUB)
  list(APPEND ESPTOOL_OPTS --no-stub)
endif()

# WRITEFLASH address/file list (same structure as Config.mk WRITEFLASH_OPTS)
set(ESPTOOL_BINS "")

if(CONFIG_ESPRESSIF_MERGE_BINS)
  if(NOT EXISTS "${BINARY_DIR}/nuttx.merged.bin")
    message(
      FATAL_ERROR
        "nuttx.merged.bin not found (CONFIG_ESPRESSIF_MERGE_BINS=y). Build first."
    )
  endif()
  set(WRITEFLASH_ARGS ${ESPTOOL_WRITEFLASH_ARGS} 0x0
                      "${BINARY_DIR}/nuttx.merged.bin")
else()
  if(CONFIG_ESPRESSIF_BOOTLOADER_MCUBOOT)
    set(BOOTLOADER "${NUTTX_DIR}/mcuboot-${CHIP_SERIES}.bin")
    if(NOT EXISTS "${BOOTLOADER}")
      message(FATAL_ERROR "MCUboot binary not found: ${BOOTLOADER}")
    endif()
    list(APPEND ESPTOOL_BINS ${BL_OFFSET} "${BOOTLOADER}")

    if(NOT EXISTS "${BINARY_DIR}/vefuse.bin")
      file(WRITE "${BINARY_DIR}/vefuse.bin" "")
    endif()
    list(APPEND ESPTOOL_BINS ${EFUSE_FLASH_OFFSET} "${BINARY_DIR}/vefuse.bin")

    if(NOT EXISTS "${BINARY_DIR}/nuttx.bin")
      message(FATAL_ERROR "nuttx.bin not found in ${BINARY_DIR}")
    endif()
    list(APPEND ESPTOOL_BINS ${MCUBOOT_APP_OFFSET} "${BINARY_DIR}/nuttx.bin")

  elseif(CONFIG_ESPRESSIF_SIMPLE_BOOT)
    if(NOT EXISTS "${BINARY_DIR}/nuttx.bin")
      message(FATAL_ERROR "nuttx.bin not found in ${BINARY_DIR}")
    endif()
    list(APPEND ESPTOOL_BINS ${BL_OFFSET} "${BINARY_DIR}/nuttx.bin")
  else()
    if(NOT EXISTS "${BINARY_DIR}/nuttx.bin")
      message(FATAL_ERROR "nuttx.bin not found in ${BINARY_DIR}")
    endif()
    list(APPEND ESPTOOL_BINS 0x0000 "${BINARY_DIR}/nuttx.bin")
  endif()

  if(CONFIG_ESPRESSIF_SECURE_FLASH_ENC_ENABLED AND CONFIG_ESPRESSIF_SPIFLASH)
    if(NOT EXISTS "${BINARY_DIR}/enc_mtd.bin")
      message(
        FATAL_ERROR
          "enc_mtd.bin not found; required for SPI flash + flash encryption (build/post-build first)."
      )
    endif()
    list(APPEND ESPTOOL_BINS ${CONFIG_ESPRESSIF_STORAGE_MTD_OFFSET}
         "${BINARY_DIR}/enc_mtd.bin")
  endif()

  set(WRITEFLASH_ARGS ${ESPTOOL_WRITEFLASH_ARGS} ${ESPTOOL_BINS})
endif()

execute_process(
  COMMAND ${ESPTOOL} ${ESPTOOL_OPTS} write-flash ${WRITEFLASH_ARGS}
  WORKING_DIRECTORY ${BINARY_DIR}
  RESULT_VARIABLE FLASH_RESULT)

if(NOT FLASH_RESULT EQUAL 0)
  message(FATAL_ERROR "esptool write-flash failed (exit ${FLASH_RESULT})")
endif()

message(STATUS "Flash completed successfully.")
