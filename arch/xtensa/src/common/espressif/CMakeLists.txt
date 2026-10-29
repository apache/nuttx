# ##############################################################################
# arch/xtensa/src/common/espressif/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
# ##############################################################################

# The CMake build makes simple boot images only; MCUboot and the legacy app
# format (the bootloader build of the chips' Bootloader.mk) need Make for now.

if(NOT CONFIG_ESPRESSIF_SIMPLE_BOOT)
  message(
    FATAL_ERROR
      "CMake builds of ${CONFIG_ARCH_CHIP} support ESPRESSIF_SIMPLE_BOOT only, use Make for MCUboot or the legacy app format"
  )
endif()

# The ULP RISC-V program is built by the boards' Make files only.

if(CONFIG_ESPRESSIF_USE_ULP_RISCV_CORE)
  message(
    FATAL_ERROR
      "CMake builds of ${CONFIG_ARCH_CHIP} do not build the ULP RISC-V program, use Make for ESPRESSIF_USE_ULP_RISCV_CORE"
  )
endif()

set(SRCS)

if(CONFIG_ESP_RMT)
  list(APPEND SRCS esp_rmt.c)
  if(CONFIG_DRIVERS_RC)
    list(APPEND SRCS esp_lirc.c)
  endif()
  if(CONFIG_WS2812_NON_SPI_DRIVER)
    list(APPEND SRCS esp_ws2812.c)
  endif()
endif()

if(CONFIG_ESP_MCPWM)
  list(APPEND SRCS esp_mcpwm.c)
endif()

if(CONFIG_ESPRESSIF_SIMPLE_BOOT
   OR CONFIG_ESP32_APP_FORMAT_MCUBOOT
   OR CONFIG_ESP32S2_APP_FORMAT_MCUBOOT
   OR CONFIG_ESP32S3_APP_FORMAT_MCUBOOT)
  list(APPEND SRCS esp_loader.c)
endif()

if(CONFIG_ESPRESSIF_TEMP)
  list(APPEND SRCS esp_temperature_sensor.c)
endif()

if(CONFIG_ESP_SDM)
  list(APPEND SRCS esp_sdm.c)
endif()

if(CONFIG_ESPRESSIF_I2S)
  list(APPEND SRCS esp_i2s.c)
endif()

if(CONFIG_ESPRESSIF_I2C_BITBANG)
  list(APPEND SRCS esp_i2c_bitbang.c)
endif()

if(CONFIG_ESPRESSIF_SPI_BITBANG)
  list(APPEND SRCS esp_spi_bitbang.c)
endif()

if(CONFIG_ESP_PCNT)
  list(APPEND SRCS esp_pcnt.c)
  if(CONFIG_ESP_PCNT_AS_QE)
    list(APPEND SRCS esp_qencoder.c)
  endif()
endif()

if(CONFIG_ESPRESSIF_LEDC)
  list(APPEND SRCS esp_ledc.c)
endif()

if(CONFIG_ESPRESSIF_SPIFLASH)
  list(APPEND SRCS esp_spiflash.c)
  if(CONFIG_ESPRESSIF_MTD)
    list(APPEND SRCS esp_spiflash_mtd.c)
  endif()
endif()

if(CONFIG_ESPRESSIF_I2C_PERIPH_SLAVE_MODE)
  list(APPEND SRCS esp_i2c_slave.c)
endif()

if(CONFIG_ESPRESSIF_DEDICATED_GPIO)
  list(APPEND SRCS esp_dedic_gpio.c)
endif()

if(CONFIG_ESPRESSIF_SHA_ACCELERATOR)
  list(APPEND SRCS esp_sha.c)
endif()

if(CONFIG_ESPRESSIF_AES_ACCELERATOR)
  list(APPEND SRCS esp_aes.c)
endif()

if(NOT CONFIG_ARCH_CHIP_ESP32 AND CONFIG_CRYPTO_CRYPTODEV_HARDWARE)
  list(APPEND SRCS esp_crypto.c)
endif()

if(CONFIG_ESPRESSIF_USE_ULP_RISCV_CORE)
  list(APPEND SRCS esp_ulp.c)
endif()

if(CONFIG_SYSTEM_NXDIAG_ESPRESSIF_CHIP_WO_TOOL)
  list(APPEND SRCS esp_nxdiag.c)
endif()

if(CONFIG_ESPRESSIF_WIRELESS)
  list(APPEND SRCS esp_wireless.c esp_wifi_utils.c)
  if(CONFIG_ESPRESSIF_WIFI)
    list(APPEND SRCS esp_wifi_event_handler.c esp_wlan_netdev.c esp_wifi_api.c)
  endif()
  if(CONFIG_ESPRESSIF_ESPNOW_PKTRADIO)
    list(APPEND SRCS esp_espnow_pktradio.c)
  endif()
endif()

if(CONFIG_ESPRESSIF_ADC)
  list(APPEND SRCS esp_adc.c)
endif()

if(CONFIG_PM)
  if(NOT CONFIG_ARCH_CUSTOM_PMINIT)
    list(APPEND SRCS esp_pm_initialize.c)
  endif()
  list(APPEND SRCS esp_pm.c)
endif()

list(APPEND SRCS esp_efuse.c esp_irq.c esp_xtensa_intr.c esp_gpio.c)

if(CONFIG_RTC_DRIVER)
  list(APPEND SRCS esp_rtc.c)
endif()

if(CONFIG_ESPRESSIF_HR_TIMER)
  list(APPEND SRCS esp_timer_adapter.c)
endif()

target_compile_options(arch PRIVATE -Wno-shadow -Wno-undef)
target_link_options(nuttx PRIVATE "SHELL:-u esp_system_include_startup_funcs"
                    "SHELL:-u esp_timer_init_include_func")

if(CONFIG_ESPRESSIF_EFUSE)
  target_link_options(nuttx PRIVATE "SHELL:-u esp_efuse_startup_include_func")
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
      cf95a27f2aa263d77c7a8d7f56007abfc899fc47
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
  "${NUTTX_BINARY_DIR}/arch/xtensa/src/common/espressif/${ESP_HAL_3RDPARTY_REPO_NAME}"
  REALPATH)

set(EXECUTE_CLONE_ESP_HAL_3RDPARTY_REPO TRUE)
set(ESP_HAL_3RDPARTY_REPO "${ESP_HAL_3RDPARTY_CHIP_DIR}")
string(REPLACE "\"" "" CHIP_SERIES "${CONFIG_ESPRESSIF_CHIP_SERIES}")

# The Wi-Fi and Bluetooth libraries of each chip (submodules of the HAL)
if(CONFIG_ARCH_CHIP_ESP32)
  set(ESP_HAL_WIRELESS_SUBMODULES
      components/esp_phy/lib components/esp_wifi/lib
      components/bt/controller/lib_esp32 components/esp_coex/lib)
elseif(CONFIG_ARCH_CHIP_ESP32S2)
  set(ESP_HAL_WIRELESS_SUBMODULES components/esp_phy/lib
                                  components/esp_wifi/lib)
else()
  set(ESP_HAL_WIRELESS_SUBMODULES
      components/esp_phy/lib components/esp_wifi/lib
      components/bt/controller/lib_esp32c3_family components/esp_coex/lib)
endif()

# Clone the esp-hal-3rdparty repository and set up its submodules, once, when
# CMake is configured.
if(NOT IS_DIRECTORY "${ESP_HAL_3RDPARTY_REPO}")
  # NXTMPDIR holds a cached copy of the repository in nuttx/../nxtmpdir
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
  execute_process(COMMAND git -C components/mbedtls/mbedtls reset --quiet --hard
                  WORKING_DIRECTORY ${ESP_HAL_3RDPARTY_FETCH_SOURCE_DIR})
  file(GLOB ESP_HAL_MBEDTLS_PATCHES
       ${ESP_HAL_3RDPARTY_FETCH_SOURCE_DIR}/nuttx/patches/components/mbedtls/mbedtls/*.patch)
  execute_process(COMMAND git apply ${ESP_HAL_MBEDTLS_PATCHES}
                  WORKING_DIRECTORY ${ESP_HAL_3RDPARTY_FETCH_SOURCE_DIR}/components/mbedtls/mbedtls)

  if(CONFIG_ESPRESSIF_WIRELESS)
    message(
      STATUS "Espressif HAL for 3rd Party Platforms: initializing submodules...")
    execute_process(
      COMMAND git submodule --quiet update --init --depth=1 ${ESP_HAL_WIRELESS_SUBMODULES}
      WORKING_DIRECTORY ${ESP_HAL_3RDPARTY_FETCH_SOURCE_DIR}
      RESULT_VARIABLE GIT_SUBMODULE_RESULT)
    if(NOT GIT_SUBMODULE_RESULT EQUAL 0)
      message(FATAL_ERROR "Failed to initialize the Wi-Fi submodules of ${ESP_HAL_3RDPARTY_FETCH_SOURCE_DIR}")
    endif()
  endif()

  # cmake-format: on

  if(NXTMPDIR AND NOT EXISTS "${ESP_HAL_3RDPARTY_CHIP_DIR}")
    message(
      STATUS
        "Copying from ${ESP_HAL_NXTMPDIR_CACHE} to ${ESP_HAL_3RDPARTY_CHIP_DIR}"
    )
    file(COPY "${ESP_HAL_NXTMPDIR_CACHE}"
         DESTINATION "${ESP_HAL_3RDPARTY_CHIP_DIR}/..")
  endif()
endif()

# A repository cloned for a configuration without wireless lacks the wireless
# submodules: initialize them when wireless was enabled afterwards.
if(CONFIG_ESPRESSIF_WIRELESS
   AND NOT EXISTS
       "${ESP_HAL_3RDPARTY_REPO}/components/esp_phy/lib/${CHIP_SERIES}/libphy.a"
)
  message(
    STATUS "Espressif HAL for 3rd Party Platforms: initializing submodules...")
  execute_process(
    COMMAND git submodule --quiet update --init --depth=1
            ${ESP_HAL_WIRELESS_SUBMODULES}
    WORKING_DIRECTORY ${ESP_HAL_3RDPARTY_REPO}
    RESULT_VARIABLE GIT_SUBMODULE_RESULT)
  if(NOT GIT_SUBMODULE_RESULT EQUAL 0)
    message(
      FATAL_ERROR
        "Failed to initialize the wireless submodules of ${ESP_HAL_3RDPARTY_REPO}"
    )
  endif()
endif()

# ##############################################################################
# HAL sources and include paths (before the Espressif include paths)
# ##############################################################################

include(${NUTTX_CHIP_ABS_DIR}/hal_${CHIP_SERIES}.cmake)

if(CONFIG_ESPRESSIF_WIRELESS)
  include(${CMAKE_CURRENT_SOURCE_DIR}/Wireless.cmake)
endif()

target_include_directories(
  arch PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
               ${CMAKE_CURRENT_SOURCE_DIR}/platform_include)
target_sources(arch PRIVATE ${SRCS})

# ##############################################################################
# ESP binary image and flashing
# ##############################################################################

if(NOT TARGET nuttx_post_build)
  add_custom_target(nuttx_post_build)
endif()

add_custom_command(
  TARGET nuttx_post_build
  POST_BUILD
  COMMAND
    ${CMAKE_COMMAND} -DBINARY_DIR=${NUTTX_BINARY_DIR} -DSOURCE_DIR=${NUTTX_DIR}
    -P ${NUTTX_DIR}/tools/espressif/espressif_mkimage.cmake
  WORKING_DIRECTORY ${NUTTX_BINARY_DIR}
  COMMENT "Generating ESP-compatible binary"
  VERBATIM)

# Usage: ESPTOOL_PORT=/dev/ttyUSBx cmake --build <build_dir> --target flash

add_custom_target(
  flash
  DEPENDS nuttx_post_build
  COMMAND
    ${CMAKE_COMMAND} -DBINARY_DIR=${NUTTX_BINARY_DIR} -DSOURCE_DIR=${NUTTX_DIR}
    -DNUTTX_DIR=${NUTTX_DIR} -P
    ${NUTTX_DIR}/tools/espressif/espressif_flash.cmake
  WORKING_DIRECTORY ${NUTTX_BINARY_DIR}
  COMMENT "Flashing firmware via esptool to $ENV{ESPTOOL_PORT}"
  VERBATIM)
