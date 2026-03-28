# ##############################################################################
# arch/risc-v/src/common/espressif/Bootloader.cmake
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

# ##############################################################################
# Bootloader Configuration Variables
# ##############################################################################

set(TOOLSDIR ${NUTTX_DIR}/tools/espressif)
set(BOOTLOADER_SRCDIR ${CMAKE_CURRENT_SOURCE_DIR}/bootloader)
set(BOOTLOADER_OUTDIR ${BOOTLOADER_SRCDIR}/out)
set(BOOTLOADER_CONFIG ${BOOTLOADER_SRCDIR}/bootloader.conf)

# MCUboot

set(MCUBOOT_SRCDIR ${BOOTLOADER_SRCDIR}/mcuboot)
set(MCUBOOT_ESPDIR ${MCUBOOT_SRCDIR}/boot/espressif)
set(MCUBOOT_TOOLCHAIN ${TOOLSDIR}/mcuboot_toolchain_espressif.cmake)
set(HALDIR ${BOOTLOADER_SRCDIR}/esp-hal-3rdparty-mcuboot)

if(DEFINED ENV{MCUBOOT_VERSION})
  set(MCUBOOT_VERSION $ENV{MCUBOOT_VERSION})
elseif(DEFINED CONFIG_ESPRESSIF_MCUBOOT_VERSION)
  set(MCUBOOT_VERSION ${CONFIG_ESPRESSIF_MCUBOOT_VERSION})
else()
  message(
    WARNING
      "CONFIG_ESPRESSIF_MCUBOOT_VERSION is not defined; MCUBOOT_VERSION will be empty."
  )
  set(MCUBOOT_VERSION "")
endif()

if(DEFINED ENV{MCUBOOT_URL})
  set(MCUBOOT_URL $ENV{MCUBOOT_URL})
else()
  set(MCUBOOT_URL "https://github.com/mcu-tools/mcuboot")
endif()

if(NOT DEFINED ESP_HAL_3RDPARTY_VERSION_FOR_MCUBOOT)
  set(ESP_HAL_3RDPARTY_VERSION_FOR_MCUBOOT
      911dbec8e4a92e70056b58a3d2b0d965b8b7bcc9)
endif()

# ##############################################################################
# Bootloader Configuration File Generation
# ##############################################################################

function(generate_bootloader_config)
  # Create bootloader source directory if it doesn't exist
  file(MAKE_DIRECTORY ${BOOTLOADER_SRCDIR})

  # Start with base configuration
  set(CONFIG_CONTENT "")

  # NON_OS_BUILD is always enabled
  string(APPEND CONFIG_CONTENT "NON_OS_BUILD=1\n")

  # Flash size configuration
  if(CONFIG_ESPRESSIF_FLASH_2M)
    string(APPEND CONFIG_CONTENT "CONFIG_ESPTOOLPY_FLASHSIZE_2MB=1\n")
  endif()
  if(CONFIG_ESPRESSIF_FLASH_4M)
    string(APPEND CONFIG_CONTENT "CONFIG_ESPTOOLPY_FLASHSIZE_4MB=1\n")
  endif()

  # Flash mode configuration
  if(CONFIG_ESPRESSIF_FLASH_MODE_DIO)
    string(APPEND CONFIG_CONTENT "CONFIG_ESPTOOLPY_FLASHMODE_DIO=1\n")
  endif()
  if(CONFIG_ESPRESSIF_FLASH_MODE_DOUT)
    string(APPEND CONFIG_CONTENT "CONFIG_ESPTOOLPY_FLASHMODE_DOUT=1\n")
  endif()
  if(CONFIG_ESPRESSIF_FLASH_MODE_QIO)
    string(APPEND CONFIG_CONTENT "CONFIG_ESPTOOLPY_FLASHMODE_QIO=1\n")
  endif()
  if(CONFIG_ESPRESSIF_FLASH_MODE_QOUT)
    string(APPEND CONFIG_CONTENT "CONFIG_ESPTOOLPY_FLASHMODE_QOUT=1\n")
  endif()

  # Flash frequency configuration
  if(CONFIG_ESPRESSIF_FLASH_FREQ_80M)
    string(APPEND CONFIG_CONTENT "CONFIG_ESPTOOLPY_FLASHFREQ_80M=1\n")
  endif()
  if(CONFIG_ESPRESSIF_FLASH_FREQ_64M)
    string(APPEND CONFIG_CONTENT "CONFIG_ESPTOOLPY_FLASHFREQ_64M=1\n")
  endif()
  if(CONFIG_ESPRESSIF_FLASH_FREQ_48M)
    string(APPEND CONFIG_CONTENT "CONFIG_ESPTOOLPY_FLASHFREQ_48M=1\n")
  endif()
  if(CONFIG_ESPRESSIF_FLASH_FREQ_40M)
    string(APPEND CONFIG_CONTENT "CONFIG_ESPTOOLPY_FLASHFREQ_40M=1\n")
  endif()
  if(CONFIG_ESPRESSIF_FLASH_FREQ_26M)
    string(APPEND CONFIG_CONTENT "CONFIG_ESPTOOLPY_FLASHFREQ_26M=1\n")
  endif()
  if(CONFIG_ESPRESSIF_FLASH_FREQ_20M)
    string(APPEND CONFIG_CONTENT "CONFIG_ESPTOOLPY_FLASHFREQ_20M=1\n")
  endif()
  if(DEFINED CONFIG_ESPRESSIF_FLASH_FREQ)
    string(APPEND CONFIG_CONTENT
           "CONFIG_ESPTOOLPY_FLASHFREQ=${CONFIG_ESPRESSIF_FLASH_FREQ}\n")
  endif()

  # MCUboot specific configuration
  if(CONFIG_ESPRESSIF_BOOTLOADER_MCUBOOT)
    string(APPEND CONFIG_CONTENT "CONFIG_ESPRESSIF_BOOTLOADER_MCUBOOT=1\n")
    string(APPEND CONFIG_CONTENT "CONFIG_ESP_BOOTLOADER_OFFSET=0x0000\n")
    string(APPEND CONFIG_CONTENT "CONFIG_ESP_BOOTLOADER_SIZE=0xF000\n")
    string(
      APPEND
      CONFIG_CONTENT
      "CONFIG_ESP_IMAGE0_PRIMARY_START_ADDRESS=${CONFIG_ESPRESSIF_OTA_PRIMARY_SLOT_OFFSET}\n"
    )
    string(APPEND CONFIG_CONTENT
           "CONFIG_ESP_APPLICATION_SIZE=${CONFIG_ESPRESSIF_OTA_SLOT_SIZE}\n")
    string(
      APPEND
      CONFIG_CONTENT
      "CONFIG_ESP_IMAGE0_SECONDARY_START_ADDRESS=${CONFIG_ESPRESSIF_OTA_SECONDARY_SLOT_OFFSET}\n"
    )
    string(APPEND CONFIG_CONTENT "CONFIG_ESP_MCUBOOT_WDT_ENABLE=1\n")
    string(APPEND CONFIG_CONTENT "CONFIG_LIBC_NEWLIB=1\n")
    string(APPEND CONFIG_CONTENT
           "CONFIG_ESP_SCRATCH_OFFSET=${CONFIG_ESPRESSIF_OTA_SCRATCH_OFFSET}\n")
    string(APPEND CONFIG_CONTENT
           "CONFIG_ESP_SCRATCH_SIZE=${CONFIG_ESPRESSIF_OTA_SCRATCH_SIZE}\n")
    string(APPEND CONFIG_CONTENT "CONFIG_ESP_CONSOLE_UART=1\n")

    # UART console configuration
    if(CONFIG_UART0_SERIAL_CONSOLE)
      string(APPEND CONFIG_CONTENT "CONFIG_ESP_CONSOLE_UART_NUM=0\n")
      string(APPEND CONFIG_CONTENT "CONFIG_ESP_CONSOLE_ROM_SERIAL_PORT_NUM=0\n")
    endif()
    if(CONFIG_UART1_SERIAL_CONSOLE)
      string(APPEND CONFIG_CONTENT "CONFIG_ESP_CONSOLE_UART_NUM=1\n")
      string(APPEND CONFIG_CONTENT "CONFIG_ESP_CONSOLE_ROM_SERIAL_PORT_NUM=1\n")
    endif()
    if(CONFIG_ESPRESSIF_USBSERIAL)
      string(APPEND CONFIG_CONTENT "CONFIG_ESP_CONSOLE_UART_NUM=0\n")
    endif()

    # Secure flash encryption configuration
    if(CONFIG_ESPRESSIF_SECURE_FLASH_ENC_ENABLED)
      string(APPEND CONFIG_CONTENT "CONFIG_SECURE_FLASH_ENC_ENABLED=1\n")
    endif()
    if(CONFIG_ESPRESSIF_SECURE_FLASH_ENCRYPTION_MODE_DEVELOPMENT)
      string(APPEND CONFIG_CONTENT
             "CONFIG_SECURE_FLASH_ENCRYPTION_MODE_DEVELOPMENT=1\n")
    endif()
    if(CONFIG_ESPRESSIF_SECURE_FLASH_UART_BOOTLOADER_ALLOW_ENC)
      string(APPEND CONFIG_CONTENT
             "CONFIG_SECURE_FLASH_UART_BOOTLOADER_ALLOW_ENC=1\n")
    endif()
    if(CONFIG_ESPRESSIF_SECURE_FLASH_UART_BOOTLOADER_ALLOW_DEC)
      string(APPEND CONFIG_CONTENT
             "CONFIG_SECURE_FLASH_UART_BOOTLOADER_ALLOW_DEC=1\n")
    endif()
    if(CONFIG_ESPRESSIF_SECURE_FLASH_UART_BOOTLOADER_ALLOW_CACHE)
      string(APPEND CONFIG_CONTENT
             "CONFIG_SECURE_FLASH_UART_BOOTLOADER_ALLOW_CACHE=1\n")
    endif()

    string(APPEND CONFIG_CONTENT "CONFIG_BOOTLOADER_LOG_LEVEL=3\n")

    # EFUSE virtual configuration
    if(CONFIG_ESPRESSIF_EFUSE_VIRTUAL_KEEP_IN_FLASH)
      string(APPEND CONFIG_CONTENT "CONFIG_EFUSE_VIRTUAL=1\n")
      string(APPEND CONFIG_CONTENT "CONFIG_EFUSE_VIRTUAL_KEEP_IN_FLASH=1\n")
      string(
        APPEND
        CONFIG_CONTENT
        "CONFIG_EFUSE_VIRTUAL_OFFSET=${CONFIG_ESPRESSIF_EFUSE_VIRTUAL_KEEP_IN_FLASH_OFFSET}\n"
      )
      string(
        APPEND
        CONFIG_CONTENT
        "CONFIG_EFUSE_VIRTUAL_SIZE=${CONFIG_ESPRESSIF_EFUSE_VIRTUAL_KEEP_IN_FLASH_SIZE}\n"
      )
    endif()
  endif()

  # Write configuration file
  file(WRITE ${BOOTLOADER_CONFIG} ${CONFIG_CONTENT})
endfunction()

# ##############################################################################
# Bootloader Build Configuration
# ##############################################################################

set(BOOTLOADER_BIN ${NUTTX_DIR}/mcuboot-${CHIP_SERIES}.bin)

# Generate bootloader configuration file
generate_bootloader_config()

# Clone ESP HAL for MCUboot
if(NOT EXISTS ${HALDIR})
  message(
    STATUS "Cloning Espressif HAL for 3rd Party Platforms (MCUBoot build)")
  FetchContent_Declare(
    esp_hal_3rdparty_mcuboot
    GIT_REPOSITORY ${ESP_HAL_3RDPARTY_URL}
    GIT_TAG ${ESP_HAL_3RDPARTY_VERSION_FOR_MCUBOOT}
    GIT_SUBMODULES "" SOURCE_DIR ${HALDIR})

  FetchContent_MakeAvailable(esp_hal_3rdparty_mcuboot)
endif()

# Parse flash parameters from config file (matching build_mcuboot.sh logic)
set(MCUBOOT_BUILD_DIR "${NUTTX_DIR}/build-${CHIP_SERIES}-bootloader")
set(MCUBOOT_SOURCE_DIR "${MCUBOOT_SRCDIR}/boot/espressif")

# Determine flash size (default: 4MB)
if(CONFIG_ESPRESSIF_FLASH_2M)
  set(MCUBOOT_FLASH_SIZE "2MB")
elseif(CONFIG_ESPRESSIF_FLASH_4M)
  set(MCUBOOT_FLASH_SIZE "4MB")
else()
  set(MCUBOOT_FLASH_SIZE "4MB")
endif()

# Determine flash mode (default: dio)
if(CONFIG_ESPRESSIF_FLASH_MODE_DIO)
  set(MCUBOOT_FLASH_MODE "dio")
elseif(CONFIG_ESPRESSIF_FLASH_MODE_DOUT)
  set(MCUBOOT_FLASH_MODE "dout")
elseif(CONFIG_ESPRESSIF_FLASH_MODE_QIO)
  set(MCUBOOT_FLASH_MODE "qio")
elseif(CONFIG_ESPRESSIF_FLASH_MODE_QOUT)
  set(MCUBOOT_FLASH_MODE "qout")
else()
  set(MCUBOOT_FLASH_MODE "dio")
endif()

# Flash frequency for esptool (must match CONFIG_ESPTOOLPY_FLASHFREQ in
# bootloader.conf / build_mcuboot.sh). Kconfig maps some choices to different
# strings (e.g. ESP32-H2 64 MHz -> CONFIG_ESPRESSIF_FLASH_FREQ is 48m; esptool
# v5 has no 64m).
if(DEFINED CONFIG_ESPRESSIF_FLASH_FREQ AND NOT "${CONFIG_ESPRESSIF_FLASH_FREQ}"
                                           STREQUAL "")
  string(REPLACE "\"" "" MCUBOOT_FLASH_FREQ "${CONFIG_ESPRESSIF_FLASH_FREQ}")
elseif(CONFIG_ESPRESSIF_FLASH_FREQ_80M)
  set(MCUBOOT_FLASH_FREQ "80m")
elseif(CONFIG_ESPRESSIF_FLASH_FREQ_64M)
  set(MCUBOOT_FLASH_FREQ "48m")
elseif(CONFIG_ESPRESSIF_FLASH_FREQ_48M)
  set(MCUBOOT_FLASH_FREQ "48m")
elseif(CONFIG_ESPRESSIF_FLASH_FREQ_40M)
  set(MCUBOOT_FLASH_FREQ "40m")
elseif(CONFIG_ESPRESSIF_FLASH_FREQ_26M)
  set(MCUBOOT_FLASH_FREQ "26m")
elseif(CONFIG_ESPRESSIF_FLASH_FREQ_20M)
  set(MCUBOOT_FLASH_FREQ "20m")
else()
  set(MCUBOOT_FLASH_FREQ "40m")
endif()

# Check if ninja is available for generator
find_program(NINJA_EXE NAMES ninja)
if(NINJA_EXE)
  set(MCUBOOT_GENERATOR "-GNinja")
else()
  set(MCUBOOT_GENERATOR "")
endif()

ExternalProject_Add(
  bootloader
  GIT_REPOSITORY ${MCUBOOT_URL}
  GIT_TAG ${MCUBOOT_VERSION}
  SOURCE_DIR ${MCUBOOT_SRCDIR}
  PATCH_COMMAND
  COMMAND git submodule --quiet update --init --recursive ext/mbedtls
          WORKING_DIRECTORY ${MCUBOOT_ESPDIR}
  CONFIGURE_COMMAND
  CMAKE_ARGS -DCMAKE_TOOLCHAIN_FILE=${MCUBOOT_TOOLCHAIN}
             -DMCUBOOT_TARGET=${CHIP_SERIES}
             -DMCUBOOT_CONFIG_FILE=${BOOTLOADER_CONFIG}
             -DESP_HAL_PATH=${HALDIR}
             -DCONFIG_ESP_FLASH_SIZE=${MCUBOOT_FLASH_SIZE}
             -DESP_FLASH_MODE=${MCUBOOT_FLASH_MODE}
             -DESP_FLASH_FREQ=${MCUBOOT_FLASH_FREQ}
             -B
             ${MCUBOOT_BUILD_DIR}
             ${MCUBOOT_GENERATOR}
             ${MCUBOOT_SOURCE_DIR}
             WORKING_DIRECTORY
             ${MCUBOOT_ESPDIR}
  BUILD_COMMAND ${CMAKE_COMMAND} --build ${MCUBOOT_BUILD_DIR} WORKING_DIRECTORY
                ${MCUBOOT_ESPDIR}
  INSTALL_COMMAND
  COMMAND
    ${CMAKE_COMMAND} -E copy ${MCUBOOT_BUILD_DIR}/mcuboot_${CHIP_SERIES}.bin
    ${NUTTX_DIR}/mcuboot-${CHIP_SERIES}.bin WORKING_DIRECTORY ${MCUBOOT_ESPDIR}
  BUILD_BYPRODUCTS ${MCUBOOT_BUILD_DIR}/mcuboot_${CHIP_SERIES}.bin)

# Add bootloader files to clean list
set_property(
  DIRECTORY ${CMAKE_SOURCE_DIR}
  APPEND
  PROPERTY ADDITIONAL_CLEAN_FILES ${HALDIR} ${BOOTLOADER_SRCDIR}
           ${BOOTLOADER_BIN})
