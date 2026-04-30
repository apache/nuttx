# ##############################################################################
# arch/xtensa/src/common/espressif/hal_xtensa_espressif.cmake
#
# SPDX-License-Identifier: Apache-2.0
# ##############################################################################

if(NOT EXISTS ${ESP_HAL_3RDPARTY_REPO})
  message(
    FATAL_ERROR
      "ESP_HAL_3RDPARTY_REPO does not exist: ${ESP_HAL_3RDPARTY_REPO}. Please ensure the HAL 3rd party repository is cloned correctly."
  )
endif()

target_include_directories(
  arch
  PRIVATE
    ${ESP_HAL_3RDPARTY_REPO}/nuttx/include
    ${ESP_HAL_3RDPARTY_REPO}/nuttx/include/mbedtls
    ${ESP_HAL_3RDPARTY_REPO}/nuttx/${CHIP_SERIES}/include
    ${ESP_HAL_3RDPARTY_REPO}/components/bootloader_support/include
    ${ESP_HAL_3RDPARTY_REPO}/components/bootloader_support/private_include
    ${ESP_HAL_3RDPARTY_REPO}/components/bootloader_support/bootloader_flash/include
    ${ESP_HAL_3RDPARTY_REPO}/components/efuse/include
    ${ESP_HAL_3RDPARTY_REPO}/components/efuse/private_include
    ${ESP_HAL_3RDPARTY_REPO}/components/efuse/${CHIP_SERIES}/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_common/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_event/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_adc/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_adc/interface
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/include/esp_private
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/include/soc
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/include/soc/${CHIP_SERIES}
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_mm/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_phy/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_phy/${CHIP_SERIES}/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_pm/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_rom/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_rom/${CHIP_SERIES}
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_rom/${CHIP_SERIES}/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_system/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_system/port/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_timer/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_timer/private_include
    ${ESP_HAL_3RDPARTY_REPO}/components/hal/include
    ${ESP_HAL_3RDPARTY_REPO}/components/hal/${CHIP_SERIES}/include
    ${ESP_HAL_3RDPARTY_REPO}/components/log/include
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/port/include
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/include
    ${ESP_HAL_3RDPARTY_REPO}/components/soc/include
    ${ESP_HAL_3RDPARTY_REPO}/components/soc/${CHIP_SERIES}/include
    ${ESP_HAL_3RDPARTY_REPO}/components/soc/${CHIP_SERIES}/register
    ${ESP_HAL_3RDPARTY_REPO}/components/spi_flash/include
    ${ESP_HAL_3RDPARTY_REPO}/components/spi_flash/include/esp_flash_chips
    ${ESP_HAL_3RDPARTY_REPO}/components/xtensa/include
    ${ESP_HAL_3RDPARTY_REPO}/components/xtensa/${CHIP_SERIES}/include
    ${ESP_HAL_3RDPARTY_REPO}/components/upper_hal_gpio/include
    ${ESP_HAL_3RDPARTY_REPO}/components/upper_hal_rmt/include
    ${ESP_HAL_3RDPARTY_REPO}/components/upper_hal_rmt/src
    ${ESP_HAL_3RDPARTY_REPO}/nuttx/src/components/esp_libc/platform_include)

set(ESP_ROM_LD_DIR
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_rom/${CHIP_SERIES}/ld)
set(ESP_SOC_LD_DIR ${ESP_HAL_3RDPARTY_REPO}/components/soc/${CHIP_SERIES}/ld)
set_property(
  GLOBAL APPEND
  PROPERTY LD_SCRIPT ${ESP_ROM_LD_DIR}/${CHIP_SERIES}.rom.api.ld
           ${ESP_ROM_LD_DIR}/${CHIP_SERIES}.rom.ld
           ${ESP_SOC_LD_DIR}/${CHIP_SERIES}.peripherals.ld)

set(HAL_SRCS)
list(
  APPEND
  HAL_SRCS
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_common/src/esp_err_to_name.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_system/esp_system.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_system/esp_err.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_system/startup.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_system/startup_funcs.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_timer/src/esp_timer.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_timer/src/esp_timer_impl_common.c
  ${ESP_HAL_3RDPARTY_REPO}/components/efuse/src/esp_efuse_api.c
  ${ESP_HAL_3RDPARTY_REPO}/components/efuse/src/esp_efuse_utility.c
  ${ESP_HAL_3RDPARTY_REPO}/components/efuse/${CHIP_SERIES}/esp_efuse_fields.c
  ${ESP_HAL_3RDPARTY_REPO}/components/bootloader_support/src/flash_encrypt.c
  ${ESP_HAL_3RDPARTY_REPO}/components/bootloader_support/bootloader_flash/src/bootloader_flash.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_rom/patches/esp_rom_sys.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_rom/patches/esp_rom_print.c
  ${ESP_HAL_3RDPARTY_REPO}/components/spi_flash/esp_flash_api.c
  ${ESP_HAL_3RDPARTY_REPO}/components/spi_flash/spi_flash_wrap.c
  ${ESP_HAL_3RDPARTY_REPO}/components/log/src/log.c
  ${ESP_HAL_3RDPARTY_REPO}/components/upper_hal_gpio/src/gpio.c
  ${ESP_HAL_3RDPARTY_REPO}/nuttx/src/platform/os.c
  ${ESP_HAL_3RDPARTY_REPO}/nuttx/src/heap_caps.c)

target_sources(arch PRIVATE ${HAL_SRCS})
