# ##############################################################################
# tools/espressif/espressif_mkimage.cmake
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

# This script is executed as a post-build step to generate ESP-compatible
# binaries from the NuttX ELF file.
#
# Required variables (passed via -D): BINARY_DIR       - CMake binary directory
# (contains .config and nuttx ELF) SOURCE_DIR       - CMake source directory
# (NuttX root)
#
# All CONFIG_* variables are automatically loaded from .config file.

# ##############################################################################
# Validate required variables
# ##############################################################################

if(NOT DEFINED BINARY_DIR)
  message(FATAL_ERROR "BINARY_DIR not defined")
endif()

if(NOT DEFINED SOURCE_DIR)
  message(FATAL_ERROR "SOURCE_DIR not defined")
endif()

# ##############################################################################
# Load Kconfig values from .config file
# ##############################################################################

# Include NuttX kconfig module
include(${SOURCE_DIR}/cmake/nuttx_kconfig.cmake)

# Load all CONFIG_* variables from .config
set(DOTCONFIG "${BINARY_DIR}/.config")
if(NOT EXISTS ${DOTCONFIG})
  message(FATAL_ERROR ".config not found at ${DOTCONFIG}")
endif()

nuttx_export_kconfig(${DOTCONFIG})

include(${SOURCE_DIR}/tools/espressif/espressif_esptool_common.cmake)

# ##############################################################################
# Find required tools for the post build process
# ##############################################################################

find_program(ESPTOOL esptool esptool.py)
find_program(IMGTOOL imgtool)
find_program(PYTHON3 python3)

# ##############################################################################
# Check esptool version. Older versions will fail to build a proper image.
# ##############################################################################

if(ESPTOOL
   AND PYTHON3
   AND EXISTS "${SOURCE_DIR}/tools/espressif/check_esptool.py")
  execute_process(
    COMMAND ${PYTHON3} ${SOURCE_DIR}/tools/espressif/check_esptool.py -v 4.8.0
    RESULT_VARIABLE ESPTOOL_CHECK_RESULT
    OUTPUT_QUIET ERROR_QUIET)
  if(NOT ESPTOOL_CHECK_RESULT EQUAL 0)
    message(WARNING "esptool.py version 4.8.0 or higher recommended")
  endif()
endif()

# ##############################################################################
# Generate binary
# ##############################################################################

if(CONFIG_ESPRESSIF_BOOTLOADER_MCUBOOT)
  # MCUboot: Use imgtool to sign the image
  message(STATUS "Generate NuttX signed image")

  # Check if nuttx.hex exists
  if(NOT EXISTS "${BINARY_DIR}/nuttx.hex")
    message(FATAL_ERROR "nuttx.hex not found in ${BINARY_DIR}")
  endif()

  # Get imgtool arguments from config
  if(CONFIG_ESPRESSIF_SECURE_FLASH_ENC_ENABLED)
    set(IMGTOOL_ALIGN_ARGS --align 32 --max-align 32)
  else()
    set(IMGTOOL_ALIGN_ARGS --align 4)
  endif()

  if(DEFINED CONFIG_ESPRESSIF_MCUBOOT_SIGN_IMAGE_VERSION)
    set(MCUBOOT_VERSION "${CONFIG_ESPRESSIF_MCUBOOT_SIGN_IMAGE_VERSION}")
  else()
    set(MCUBOOT_VERSION "0.0.0")
  endif()

  if(DEFINED CONFIG_ESPRESSIF_APP_MCUBOOT_HEADER_SIZE)
    set(HEADER_SIZE ${CONFIG_ESPRESSIF_APP_MCUBOOT_HEADER_SIZE})
  else()
    set(HEADER_SIZE 0x20)
  endif()

  if(DEFINED CONFIG_ESPRESSIF_OTA_SLOT_SIZE)
    set(SLOT_SIZE ${CONFIG_ESPRESSIF_OTA_SLOT_SIZE})
  else()
    set(SLOT_SIZE 0x100000)
  endif()

  if(CONFIG_ESPRESSIF_ESPTOOL_TARGET_PRIMARY)
    set(VERIFIED --confirm)
  else()
    set(VERIFIED "")
  endif()

  set(IMGTOOL_SIGN_ARGS
      --pad
      ${VERIFIED}
      ${IMGTOOL_ALIGN_ARGS}
      -v
      ${MCUBOOT_VERSION}
      -s
      auto
      -H
      ${HEADER_SIZE}
      --pad-header
      -S
      ${SLOT_SIZE})

  execute_process(
    COMMAND ${IMGTOOL} sign ${IMGTOOL_SIGN_ARGS} ${BINARY_DIR}/nuttx.hex
            ${BINARY_DIR}/nuttx.bin
    RESULT_VARIABLE IMGTOOL_RESULT
    WORKING_DIRECTORY ${BINARY_DIR})

  if(NOT IMGTOOL_RESULT EQUAL 0)
    message(FATAL_ERROR "imgtool sign failed")
  endif()

  message(STATUS "Generated: nuttx.bin (MCUboot compatible)")

else()
  # Simple boot or legacy: Use esptool elf2image
  message(STATUS "Generate NuttX image (esptool elf2image)")

  # Check if nuttx ELF exists
  if(NOT EXISTS "${BINARY_DIR}/nuttx")
    message(FATAL_ERROR "nuttx ELF not found in ${BINARY_DIR}")
  endif()

  # Build elf2image options
  set(ELF2IMAGE_OPTS -fs ${FLASH_SIZE} -fm ${FLASH_MODE} -ff ${FLASH_FREQ})

  if(CONFIG_ESPRESSIF_SIMPLE_BOOT)
    list(APPEND ELF2IMAGE_OPTS --ram-only-header)
  endif()

  execute_process(
    COMMAND ${ESPTOOL} -c ${CHIP_SERIES} elf2image ${ELF2IMAGE_OPTS} -o
            ${BINARY_DIR}/nuttx.bin ${BINARY_DIR}/nuttx
    RESULT_VARIABLE ESPTOOL_RESULT
    WORKING_DIRECTORY ${BINARY_DIR})

  if(NOT ESPTOOL_RESULT EQUAL 0)
    message(FATAL_ERROR "esptool.py elf2image failed")
  endif()

  message(STATUS "Generated: nuttx.bin")
endif()

# ##############################################################################
# Update manifest
# ##############################################################################

file(APPEND "${BINARY_DIR}/nuttx.manifest" "nuttx.bin\n")

# ##############################################################################
# Merge binaries (optional)
# ##############################################################################

if(CONFIG_ESPRESSIF_MERGE_BINS)
  message(STATUS "MERGEBIN: Creating merged flash image ${SOURCE_DIR}")

  if(NOT ESPTOOL)
    message(FATAL_ERROR "esptool.py not found - cannot merge binaries")
  endif()

  # Build the list of binaries to merge Format: offset1 file1 offset2 file2 ...
  set(ESPTOOL_BINS "")

  if(CONFIG_ESPRESSIF_BOOTLOADER_MCUBOOT)
    # MCUboot configuration

    if(EXISTS "${SOURCE_DIR}/mcuboot-${CHIP_SERIES}.bin")
      message(
        STATUS
          "Merge bin: ${BL_OFFSET} -> ${SOURCE_DIR}/mcuboot-${CHIP_SERIES}.bin")
      list(APPEND ESPTOOL_BINS ${BL_OFFSET}
           "${SOURCE_DIR}/mcuboot-${CHIP_SERIES}.bin")
    else()
      message(
        FATAL_ERROR "mcuboot-${CHIP_SERIES}.bin not found in ${SOURCE_DIR}")
    endif()

    # Create empty vefuse.bin if it doesn't exist
    if(NOT EXISTS "${BINARY_DIR}/vefuse.bin")
      file(WRITE "${BINARY_DIR}/vefuse.bin" "")
    endif()
    list(APPEND ESPTOOL_BINS ${EFUSE_OFFSET} "${BINARY_DIR}/vefuse.bin")
    message(STATUS "Merge bin: ${EFUSE_OFFSET} -> ${BINARY_DIR}/vefuse.bin")

    list(APPEND ESPTOOL_BINS ${MCUBOOT_APP_OFFSET} "${BINARY_DIR}/nuttx.bin")
    message(
      STATUS "Merge bin: ${MCUBOOT_APP_OFFSET} -> ${BINARY_DIR}/nuttx.bin")

  elseif(CONFIG_ESPRESSIF_SIMPLE_BOOT)
    # Simple boot: same base offset as BL_OFFSET (0x2000 on ESP32-P4, else 0x0)
    list(APPEND ESPTOOL_BINS ${BL_OFFSET} "${BINARY_DIR}/nuttx.bin")

  else()
    # Legacy boot: application at offset 0
    list(APPEND ESPTOOL_BINS 0x0000 "${BINARY_DIR}/nuttx.bin")
  endif()

  # Execute merge_bin
  execute_process(
    COMMAND ${ESPTOOL} -c ${CHIP_SERIES} merge-bin --pad-to-size ${FLASH_SIZE}
            --output ${BINARY_DIR}/nuttx.merged.bin ${ESPTOOL_BINS}
    RESULT_VARIABLE MERGEBIN_RESULT
    WORKING_DIRECTORY ${BINARY_DIR})

  if(NOT MERGEBIN_RESULT EQUAL 0)
    message(FATAL_ERROR "esptool merge-bin failed")
  endif()

  message(STATUS "Generated: nuttx.merged.bin")
  file(APPEND "${BINARY_DIR}/nuttx.manifest" "nuttx.merged.bin\n")
endif()
