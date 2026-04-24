# ##############################################################################
# arch/risc-v/src/common/espressif/CMakeLists.txt
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

set(SRCS)

# Head/startup file
list(APPEND SRCS esp_head.S)

# Custom vector table (skip common RISC-V vector table)
list(APPEND SRCS esp_vectors.S)

# Required common source files
list(APPEND SRCS esp_allocateheap.c esp_start.c esp_idle.c)
list(APPEND SRCS esp_irq.c esp_gpio.c esp_rtc_gpio.c esp_libc_stubs.c)
list(APPEND SRCS esp_lowputc.c esp_serial.c)
list(APPEND SRCS esp_systemreset.c)

if(CONFIG_ARCH_HAVE_EXTRA_HEAPS)
  list(APPEND SRCS esp_extraheaps.c)
  if(CONFIG_ESPRESSIF_RETENTION_HEAP)
    list(APPEND SRCS esp_retentionheap.c)
  endif()
endif()

if(CONFIG_SCHED_TICKLESS)
  list(APPEND SRCS esp_tickless.c)
else()
  list(APPEND SRCS esp_timerisr.c)
endif()

if(CONFIG_ESPRESSIF_WDT)
  list(APPEND SRCS esp_wdt.c)
endif()

if(CONFIG_DEV_RANDOM OR CONFIG_DEV_URANDOM_ARCH)
  list(APPEND SRCS esp_random.c)
endif()

if(CONFIG_TIMER)
  list(APPEND SRCS esp_gptimer.c)
endif()

if(CONFIG_ONESHOT)
  list(APPEND SRCS esp_oneshot.c)
endif()

if(CONFIG_RTC)
  list(APPEND SRCS esp_rtc.c)
endif()

if(CONFIG_ESPRESSIF_HR_TIMER)
  list(APPEND SRCS esp_timer_adapter.c esp_ets_timer_legacy.c)
endif()

if(CONFIG_ESPRESSIF_EFUSE)
  list(APPEND SRCS esp_efuse.c)
endif()

if(CONFIG_ESPRESSIF_TWAI)
  list(APPEND SRCS esp_twai.c)
endif()

if(CONFIG_ESPRESSIF_LEDC)
  list(APPEND SRCS esp_ledc.c)
endif()

if(CONFIG_ESP_PCNT)
  list(APPEND SRCS esp_pcnt.c)
  if(CONFIG_ESP_PCNT_AS_QE)
    list(APPEND SRCS esp_qencoder.c)
  endif()
endif()

if(CONFIG_ESPRESSIF_USBSERIAL)
  list(APPEND SRCS esp_usbserial.c)
endif()

if(CONFIG_ESP_RMT)
  list(APPEND SRCS esp_rmt.c)
  if(CONFIG_DRIVERS_RC)
    list(APPEND SRCS esp_lirc.c)
  endif()
  if(CONFIG_WS2812_NON_SPI_DRIVER)
    list(APPEND SRCS esp_ws2812.c)
  endif()
endif()

if(CONFIG_ESP_SDM)
  list(APPEND SRCS esp_sdm.c)
endif()

if(CONFIG_ESPRESSIF_DEDICATED_GPIO)
  list(APPEND SRCS esp_dedic_gpio.c)
endif()

if(CONFIG_ESPRESSIF_TEMP)
  list(APPEND SRCS esp_temperature_sensor.c)
endif()

if(CONFIG_ESPRESSIF_I2C)
  if(CONFIG_ESPRESSIF_I2C_PERIPH_MASTER_MODE)
    list(APPEND SRCS esp_i2c.c)
  endif()
  if(CONFIG_ESPRESSIF_I2C_BITBANG)
    list(APPEND SRCS esp_i2c_bitbang.c)
  endif()
  if(CONFIG_ESPRESSIF_I2C_PERIPH_SLAVE_MODE)
    list(APPEND SRCS esp_i2c_slave.c)
  endif()
endif()

if(CONFIG_ESPRESSIF_I2S)
  list(APPEND SRCS esp_i2s.c)
endif()

if(CONFIG_ESPRESSIF_SPI)
  if(CONFIG_ESPRESSIF_SPI_PERIPH)
    list(APPEND SRCS esp_spi.c)
  endif()
  if(CONFIG_SPI_SLAVE)
    list(APPEND SRCS esp_spi_slave.c)
  endif()
  if(CONFIG_ESPRESSIF_SPI_BITBANG)
    list(APPEND SRCS esp_spi_bitbang.c)
  endif()
endif()

if(CONFIG_ESPRESSIF_SPIFLASH)
  list(APPEND SRCS esp_spiflash.c)
  if(CONFIG_ESPRESSIF_MTD)
    list(APPEND SRCS esp_spiflash_mtd.c)
  endif()
endif()

if(CONFIG_ESPRESSIF_WIRELESS)
  if(CONFIG_ESPRESSIF_WIFI)
    list(APPEND SRCS esp_wifi_event_handler.c)
    list(APPEND SRCS esp_wifi_api.c)
    list(APPEND SRCS esp_wlan_netdev.c)
  endif()
  list(APPEND SRCS esp_wifi_utils.c)
endif()

if(CONFIG_ESP_MCPWM)
  list(APPEND SRCS esp_mcpwm.c)
endif()

if(CONFIG_ESPRESSIF_EMAC)
  list(APPEND SRCS esp_emac.c)
endif()

if(CONFIG_SYSTEM_NXDIAG_ESPRESSIF_CHIP_WO_TOOL)
  list(APPEND SRCS esp_nxdiag.c)
endif()

if(CONFIG_ESPRESSIF_ADC)
  list(APPEND SRCS esp_adc.c)
endif()

if(CONFIG_ESPRESSIF_SHA_ACCELERATOR)
  list(APPEND SRCS esp_sha.c)
endif()

if(CONFIG_ESPRESSIF_AES_ACCELERATOR)
  list(APPEND SRCS esp_aes.c)
endif()

if(CONFIG_CRYPTO_CRYPTODEV_HARDWARE)
  list(APPEND SRCS esp_crypto.c)
endif()

if(CONFIG_ESPRESSIF_USE_LP_CORE)
  message(
    WARNING
      "LP Core is not supported on CMake. Use Make to build the image instead.")
endif()

if(CONFIG_PM)
  if(NOT CONFIG_ARCH_CUSTOM_PMINIT)
    list(APPEND SRCS esp_pm_initialize.c)
  endif()
  list(APPEND SRCS esp_pm.c)
endif()

# ##############################################################################
# ESP HAL 3rd Party Repository
# ##############################################################################

set(ESP_HAL_3RDPARTY_REPO_NAME esp-hal-3rdparty)

if(DEFINED ENV{ESP_HAL_3RDPARTY_VERSION})
  set(ESP_HAL_3RDPARTY_VERSION
      $ENV{ESP_HAL_3RDPARTY_VERSION}
      CACHE STRING "ESP HAL 3rdparty version")
else()
  set(ESP_HAL_3RDPARTY_VERSION
      c32f1ad13f4ce8312de494e8b79c88fda10fe9ed
      CACHE STRING "ESP HAL 3rdparty version")
endif()

if(DEFINED ENV{ESP_HAL_3RDPARTY_URL})
  set(ESP_HAL_3RDPARTY_URL
      $ENV{ESP_HAL_3RDPARTY_URL}
      CACHE STRING "ESP HAL 3rdparty URL")
else()
  set(ESP_HAL_3RDPARTY_URL
      https://github.com/espressif/esp-hal-3rdparty.git
      CACHE STRING "ESP HAL 3rdparty URL")
endif()

option(
  NXTMPDIR
  "Enable and use a persistent ESP third-party HAL cache directory under nuttx/../nxtmpdir"
  OFF)

get_filename_component(
  ESP_HAL_3RDPARTY_CHIP_DIR
  "${CMAKE_BINARY_DIR}/arch/risc-v/src/common/espressif/${ESP_HAL_3RDPARTY_REPO_NAME}"
  REALPATH)

set(EXECUTE_CLONE_ESP_HAL_3RDPARTY_REPO TRUE)
set(ESP_HAL_3RDPARTY_REPO "${ESP_HAL_3RDPARTY_CHIP_DIR}")
string(REPLACE "\"" "" CHIP_SERIES "${CONFIG_ESPRESSIF_CHIP_SERIES}")

# Here we initialize the esp-hal-3rdparty repository and its submodules. Should
# be done only once, when CMake is configured.
if(NOT IS_DIRECTORY "${ESP_HAL_3RDPARTY_REPO}")
  # NXTMPDIR contains a cached version of the esp-hal-3rdparty repository, which
  # is located on nuttx/../nxtmpdir/esp-hal-3rdparty if it exists.
  if(NXTMPDIR)
    set(ESP_HAL_NXTMPDIR_CACHE "${NXTMPDIR_PATH}/${ESP_HAL_3RDPARTY_REPO_NAME}")
    get_filename_component(ESP_HAL_NXTMPDIR_CACHE "${ESP_HAL_NXTMPDIR_CACHE}"
                           REALPATH)
    nuttx_check_git_hash(
      "${ESP_HAL_NXTMPDIR_CACHE}" "${ESP_HAL_3RDPARTY_VERSION}"
      ESP_HAL_3RDPARTY_REVISION_OK)
    if(NOT ESP_HAL_3RDPARTY_REVISION_OK)
      if(EXISTS "${ESP_HAL_NXTMPDIR_CACHE}")
        message(
          STATUS
            "Removing esp-hal-3rdparty at ${ESP_HAL_NXTMPDIR_CACHE} (revision mismatch)"
        )
        file(REMOVE_RECURSE "${ESP_HAL_NXTMPDIR_CACHE}")
      endif()
    else()
      set(EXECUTE_CLONE_ESP_HAL_3RDPARTY_REPO FALSE)
    endif()
    set(ESP_HAL_3RDPARTY_FETCH_SOURCE_DIR "${ESP_HAL_NXTMPDIR_CACHE}")
  else()
    set(ESP_HAL_3RDPARTY_FETCH_SOURCE_DIR "${ESP_HAL_3RDPARTY_CHIP_DIR}")
  endif()

  # Only clone the esp-hal-3rdparty repository if it does not exist
  if(EXECUTE_CLONE_ESP_HAL_3RDPARTY_REPO
     AND NOT EXISTS "${ESP_HAL_3RDPARTY_FETCH_SOURCE_DIR}")
    message(STATUS "Cloning Espressif HAL for 3rd Party Platforms")
    FetchContent_Declare(
      esp_hal_3rdparty
      GIT_REPOSITORY ${ESP_HAL_3RDPARTY_URL}
      GIT_TAG ${ESP_HAL_3RDPARTY_VERSION}
      SOURCE_DIR ${ESP_HAL_3RDPARTY_FETCH_SOURCE_DIR}
      GIT_SUBMODULES "")
    FetchContent_MakeAvailable(esp_hal_3rdparty)
  endif()
  # cmake-format: off

  # Patch mbedtls
  execute_process(COMMAND git submodule --quiet update --init components/mbedtls/mbedtls
                  WORKING_DIRECTORY ${ESP_HAL_3RDPARTY_FETCH_SOURCE_DIR})
  execute_process(COMMAND git -C components/mbedtls/mbedtls reset --hard
                  WORKING_DIRECTORY ${ESP_HAL_3RDPARTY_FETCH_SOURCE_DIR})
  execute_process(COMMAND git apply --directory components/mbedtls/mbedtls nuttx/patches/components/mbedtls/mbedtls/0001-mbedtls_add_prefix.patch
                  WORKING_DIRECTORY ${ESP_HAL_3RDPARTY_FETCH_SOURCE_DIR})
  execute_process(COMMAND git apply --directory components/mbedtls/mbedtls nuttx/patches/components/mbedtls/mbedtls/0002-mbedtls_add_prefix_to_macro.patch
                  WORKING_DIRECTORY ${ESP_HAL_3RDPARTY_FETCH_SOURCE_DIR})

  if(NXTMPDIR AND NOT EXISTS "${ESP_HAL_3RDPARTY_CHIP_DIR}")
    message(STATUS
      "Copying from ${ESP_HAL_NXTMPDIR_CACHE} to ${ESP_HAL_3RDPARTY_CHIP_DIR}")
    file(COPY "${ESP_HAL_NXTMPDIR_CACHE}" DESTINATION "${ESP_HAL_3RDPARTY_CHIP_DIR}/..")
  endif()

  # Copy gpio_sig_map.h and irq.h from the esp-hal-3rdparty repository to the NuttX directory
  file(COPY ${ESP_HAL_3RDPARTY_REPO}/components/soc/${CHIP_SERIES}/include/soc/gpio_sig_map.h
       DESTINATION ${CMAKE_BINARY_DIR}/include/arch/chip/)

  file(COPY ${ESP_HAL_3RDPARTY_REPO}/nuttx/${CHIP_SERIES}/include/irq.h
       DESTINATION ${CMAKE_BINARY_DIR}/include/arch/chip/)

  if(CONFIG_ESPRESSIF_WIRELESS)
    message(
      STATUS "Espressif HAL for 3rd Party Platforms: initializing submodules...")
    execute_process(
      COMMAND
        git submodule --quiet update --init --depth=1 components/esp_phy/lib
        components/esp_wifi/lib components/bt/controller/lib_esp32c3_family
        components/esp_coex/lib
      WORKING_DIRECTORY ${ESP_HAL_3RDPARTY_REPO}
      RESULT_VARIABLE GIT_SUBMODULE_RESULT)
    if(NOT GIT_SUBMODULE_RESULT EQUAL 0)
      message(
        FATAL_ERROR
          "Failed to initialize Wi-Fi submodules. Check if the arch/risc-v/src/common/chip/ is populated and delete its contents."
      )
    endif()
  endif()
endif()
# cmake-format: on

# This is done previously on configure time but must be done again if a 'clean'
# is executed.
set(_gpio_sig_map ${CMAKE_BINARY_DIR}/include/arch/chip/gpio_sig_map.h)
set(_irq ${CMAKE_BINARY_DIR}/include/arch/chip/irq.h)

add_custom_command(
  OUTPUT ${_gpio_sig_map} ${_irq}
  COMMAND
    ${CMAKE_COMMAND} -E copy
    ${ESP_HAL_3RDPARTY_REPO}/components/soc/${CHIP_SERIES}/include/soc/gpio_sig_map.h
    ${_gpio_sig_map}
  COMMAND ${CMAKE_COMMAND} -E copy
          ${ESP_HAL_3RDPARTY_REPO}/nuttx/${CHIP_SERIES}/include/irq.h ${_irq}
  COMMENT "Copy esp-hal-3rdparty gpio_sig_map.h and irq.h to nuttx arch include"
  VERBATIM)

add_custom_target(copy_esp3rdparty_headers DEPENDS ${_gpio_sig_map} ${_irq})

add_dependencies(arch copy_esp3rdparty_headers)

# ##############################################################################
# Compiler and Linker Flags
# ##############################################################################

# Silent preprocessor warnings
target_compile_options(
  arch PRIVATE -Wno-shadow -Wno-undef -Wno-unused-variable -fno-jump-tables
               -fno-tree-switch-conversion -Wno-deprecated-declarations)

# Linker flags for initialization hooks. CMake deduplicates plain -u options
# (same leading '-'), collapsing multiple -u into one; use SHELL: so each
# --undefined survives (otherwise ld sees the second symbol as an input file).
set(_esp_startup_u_opts "SHELL:-u esp_system_include_startup_funcs")

list(APPEND _esp_startup_u_opts "SHELL:-u esp_timer_init_include_func")

if(CONFIG_ESPRESSIF_EFUSE)
  list(APPEND _esp_startup_u_opts "SHELL:-u esp_efuse_startup_include_func")
endif()

list(APPEND _esp_startup_u_opts "SHELL:-u esp_flash_spi_init_include_func")

target_link_options(nuttx PRIVATE ${_esp_startup_u_opts})

# ##############################################################################
# Include chip-specific HAL configuration
# ##############################################################################

# NOTE: HAL includes MUST come before espressif includes to avoid conflicts
include(${NUTTX_CHIP_ABS_DIR}/hal_${CHIP_SERIES}.cmake)

# ##############################################################################
# Include Paths (must come AFTER HAL includes)
# ##############################################################################

target_include_directories(
  arch PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
               ${CMAKE_CURRENT_SOURCE_DIR}/platform_include)

# ##############################################################################
# Include Bootloader configuration
# ##############################################################################

if(CONFIG_ESPRESSIF_BOOTLOADER_MCUBOOT)
  include(${CMAKE_CURRENT_SOURCE_DIR}/Bootloader.cmake)
else()
  add_custom_target(bootloader COMMAND ${CMAKE_COMMAND} -E echo
                                       "Using direct bootloader to boot NuttX.")
endif()

# ##############################################################################
# Include Wireless configuration (if enabled)
# ##############################################################################

if(CONFIG_ESPRESSIF_WIRELESS)
  include(${CMAKE_CURRENT_SOURCE_DIR}/Wireless.cmake)
endif()

# ##############################################################################
# Add sources to arch target
# ##############################################################################

target_sources(arch PRIVATE ${SRCS})

# ##############################################################################
# Post-build operations (ESP binary generation)
# ##############################################################################

# Create post-build target for ESP binary generation
if(NOT TARGET nuttx_post_build)
  add_custom_target(nuttx_post_build)
endif()

add_custom_command(
  TARGET nuttx_post_build
  POST_BUILD
  COMMAND
    ${CMAKE_COMMAND} -DBINARY_DIR=${CMAKE_BINARY_DIR}
    -DSOURCE_DIR=${CMAKE_SOURCE_DIR} -P
    ${NUTTX_DIR}/tools/espressif/espressif_mkimage.cmake
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  COMMENT "Generating ESP-compatible binary"
  VERBATIM)

# Flash encryption: burn key to eFuses (similar to tools/espressif/Config.mk
# BURN_EFUSES). Build: ESPTOOL_PORT=/dev/ttyUSB0 NOCHECK=1 cmake --build
# <builddir> -t burn_enc_key (NOCHECK is required by burn_flash_enc_key.py; same
# idea as make NOCHECK.)

if(CONFIG_ESPRESSIF_SECURE_FLASH_ENC_ENABLED)
  include(${NUTTX_DIR}/tools/espressif/espressif_burn_enc_key.cmake)
endif()

# ##############################################################################
# Flash operation Usage: ESPTOOL_PORT=/dev/ttyUSBx cmake --build <build_dir>
# --target flash
# ##############################################################################

add_custom_target(
  flash
  DEPENDS nuttx_post_build
  COMMAND
    ${CMAKE_COMMAND} -DBINARY_DIR=${CMAKE_BINARY_DIR}
    -DSOURCE_DIR=${CMAKE_SOURCE_DIR} -DNUTTX_DIR=${NUTTX_DIR} -P
    ${NUTTX_DIR}/tools/espressif/espressif_flash.cmake
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  COMMENT "Flashing firmware via esptool to $ENV{ESPTOOL_PORT}"
  VERBATIM)

# ##############################################################################
# ULP Support (TODO)
# ##############################################################################

if(CONFIG_ESPRESSIF_USE_LP_CORE)
  list(APPEND ESP_CLEAN_FILES ${CMAKE_CURRENT_SOURCE_DIR}/../chip/ulp)
endif()

# ##############################################################################
# Clean hook - replicate Make.defs distclean behavior
# ##############################################################################

set(ESP_CLEAN_FILES
    ${NUTTX_DIR}/arch/${CONFIG_ARCH}/include/${CONFIG_ARCH_CHIP}/gpio_sig_map.h
    ${NUTTX_DIR}/arch/${CONFIG_ARCH}/include/${CONFIG_ARCH_CHIP}/irq.h
    ${NUTTX_DIR}/vefuse.bin)

set_property(
  DIRECTORY ${CMAKE_SOURCE_DIR}
  APPEND
  PROPERTY ADDITIONAL_CLEAN_FILES ${ESP_CLEAN_FILES})
