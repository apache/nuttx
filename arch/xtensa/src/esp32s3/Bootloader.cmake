# ##############################################################################
# arch/xtensa/src/esp32s3/Bootloader.cmake
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

# MCUboot for the ESP32-S3, as Bootloader.mk: "cmake --build <dir> -t
# bootloader" writes mcuboot-esp32s3.bin to the NuttX directory, where the flash
# and merge steps look for it.

set(BOOTLOADER_DIR ${NUTTX_BINARY_DIR}/bootloader)
set(BOOTLOADER_CONFIG ${BOOTLOADER_DIR}/bootloader.conf)
set(BOOTLOADER_BIN ${NUTTX_DIR}/mcuboot-esp32s3.bin)
set(MCUBOOT_SRCDIR ${BOOTLOADER_DIR}/mcuboot)
set(MCUBOOT_TOOLCHAIN
    ${NUTTX_DIR}/tools/esp32s3/mcuboot_toolchain_esp32s3.cmake)
set(HALDIR ${BOOTLOADER_DIR}/esp-hal-3rdparty-mcuboot)

if(DEFINED ENV{MCUBOOT_VERSION})
  set(MCUBOOT_VERSION $ENV{MCUBOOT_VERSION})
else()
  string(REPLACE "\"" "" MCUBOOT_VERSION "${CONFIG_ESP32S3_MCUBOOT_VERSION}")
endif()

if(DEFINED ENV{MCUBOOT_URL})
  set(MCUBOOT_URL $ENV{MCUBOOT_URL})
else()
  set(MCUBOOT_URL https://github.com/mcu-tools/mcuboot)
endif()

if(DEFINED ENV{ESP_HAL_3RDPARTY_VERSION_FOR_MCUBOOT})
  set(ESP_HAL_3RDPARTY_VERSION_FOR_MCUBOOT
      $ENV{ESP_HAL_3RDPARTY_VERSION_FOR_MCUBOOT})
else()
  set(ESP_HAL_3RDPARTY_VERSION_FOR_MCUBOOT
      911dbec8e4a92e70056b58a3d2b0d965b8b7bcc9)
endif()

# bootloader.conf, line for line as Bootloader.mk writes it

set(CONF "")

macro(conf_en name)
  string(APPEND CONF "${name}=1\n")
endmacro()

macro(conf_val name value)
  string(APPEND CONF "${name}=${value}\n")
endmacro()

string(REPLACE "\"" "" FLASH_FREQ "${CONFIG_ESPRESSIF_FLASH_FREQ}")

conf_en(NON_OS_BUILD)
foreach(size 4M 8M 16M 32M)
  if(CONFIG_ESP32S3_FLASH_${size})
    conf_en(CONFIG_ESPTOOLPY_FLASHSIZE_${size}B)
  endif()
endforeach()
foreach(mode DIO DOUT QIO QOUT)
  if(CONFIG_ESP32S3_FLASH_MODE_${mode})
    conf_en(CONFIG_ESPTOOLPY_FLASHMODE_${mode})
  endif()
endforeach()
foreach(freq 120M 80M 40M 20M)
  if(CONFIG_ESP32S3_FLASH_FREQ_${freq})
    conf_en(CONFIG_ESPTOOLPY_FLASHFREQ_${freq})
  endif()
endforeach()
conf_val(CONFIG_ESPTOOLPY_FLASHFREQ "${FLASH_FREQ}")

conf_en(CONFIG_ESPRESSIF_BOOTLOADER_MCUBOOT)
conf_val(CONFIG_ESP_BOOTLOADER_OFFSET 0x0000)
conf_val(CONFIG_ESP_BOOTLOADER_SIZE 0xF000)
conf_val(CONFIG_ESP_IMAGE0_PRIMARY_START_ADDRESS
         "${CONFIG_ESP32S3_OTA_PRIMARY_SLOT_OFFSET}")
conf_val(CONFIG_ESP_APPLICATION_SIZE "${CONFIG_ESP32S3_OTA_SLOT_SIZE}")
conf_val(CONFIG_ESP_IMAGE0_SECONDARY_START_ADDRESS
         "${CONFIG_ESP32S3_OTA_SECONDARY_SLOT_OFFSET}")
conf_en(CONFIG_ESP_MCUBOOT_WDT_ENABLE)
conf_en(CONFIG_LIBC_NEWLIB)
conf_val(CONFIG_ESP_SCRATCH_OFFSET "${CONFIG_ESP32S3_OTA_SCRATCH_OFFSET}")
conf_val(CONFIG_ESP_SCRATCH_SIZE "${CONFIG_ESP32S3_OTA_SCRATCH_SIZE}")
conf_en(CONFIG_ESP_CONSOLE_UART)
if(CONFIG_UART0_SERIAL_CONSOLE)
  conf_val(CONFIG_ESP_CONSOLE_UART_NUM 0)
endif()
if(CONFIG_UART1_SERIAL_CONSOLE)
  conf_val(CONFIG_ESP_CONSOLE_UART_NUM 1)
endif()
if(CONFIG_UART0_SERIAL_CONSOLE)
  conf_val(CONFIG_ESP_CONSOLE_ROM_SERIAL_PORT_NUM 0)
endif()
if(CONFIG_UART1_SERIAL_CONSOLE)
  conf_val(CONFIG_ESP_CONSOLE_ROM_SERIAL_PORT_NUM 1)
endif()
conf_en(CONFIG_BOOTLOADER_FLASH_XMC_SUPPORT)
conf_en(CONFIG_IDF_TARGET_ARCH_XTENSA)
conf_val(CONFIG_BOOTLOADER_LOG_LEVEL 3)
foreach(
  opt
  SECURE_FLASH_ENC_ENABLED SECURE_FLASH_ENCRYPTION_MODE_DEVELOPMENT
  SECURE_FLASH_UART_BOOTLOADER_ALLOW_ENC SECURE_FLASH_UART_BOOTLOADER_ALLOW_DEC
  SECURE_FLASH_UART_BOOTLOADER_ALLOW_CACHE EFUSE_VIRTUAL)
  if(CONFIG_ESPRESSIF_${opt})
    conf_en(CONFIG_${opt})
  endif()
endforeach()
if(CONFIG_ESPRESSIF_EFUSE_VIRTUAL_KEEP_IN_FLASH)
  conf_en(CONFIG_EFUSE_VIRTUAL_KEEP_IN_FLASH)
  conf_val(CONFIG_EFUSE_VIRTUAL_OFFSET
           "${CONFIG_ESPRESSIF_EFUSE_VIRTUAL_KEEP_IN_FLASH_OFFSET}")
  conf_val(CONFIG_EFUSE_VIRTUAL_SIZE
           "${CONFIG_ESPRESSIF_EFUSE_VIRTUAL_KEEP_IN_FLASH_SIZE}")
endif()

file(MAKE_DIRECTORY ${BOOTLOADER_DIR})
file(
  GENERATE
  OUTPUT ${BOOTLOADER_CONFIG}
  CONTENT "${CONF}")

# Clone MCUboot and its HAL on first use and build with build_mcuboot.sh, as
# Bootloader.mk does; not part of the default build.

add_custom_target(
  bootloader
  COMMAND
    sh -c
    "[ -d '${MCUBOOT_SRCDIR}' ] || { echo 'Cloning MCUboot' && git clone --quiet '${MCUBOOT_URL}' '${MCUBOOT_SRCDIR}' && git -C '${MCUBOOT_SRCDIR}' checkout --quiet '${MCUBOOT_VERSION}' && git -C '${MCUBOOT_SRCDIR}' submodule --quiet update --init --recursive ext/mbedtls; }"
  COMMAND
    sh -c
    "[ -d '${HALDIR}' ] || { echo 'Cloning Espressif HAL for 3rd Party Platforms (MCUBoot build)' && git clone --quiet '${ESP_HAL_3RDPARTY_URL}' '${HALDIR}' && git -C '${HALDIR}' checkout --quiet '${ESP_HAL_3RDPARTY_VERSION_FOR_MCUBOOT}'; }"
  COMMAND
    ${NUTTX_DIR}/tools/espressif/build_mcuboot.sh -c esp32s3 -f
    ${BOOTLOADER_CONFIG} -p ${BOOTLOADER_DIR} -e ${HALDIR} -d
    ${MCUBOOT_TOOLCHAIN}
  COMMAND ${CMAKE_COMMAND} -E copy ${BOOTLOADER_DIR}/out/mcuboot-esp32s3.bin
          ${BOOTLOADER_BIN}
  BYPRODUCTS ${BOOTLOADER_BIN}
  COMMENT "Building Bootloader"
  VERBATIM)
