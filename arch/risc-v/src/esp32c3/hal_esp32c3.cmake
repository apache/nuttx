# ##############################################################################
# arch/risc-v/src/esp32c3/hal_esp32c3.cmake
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
# Include Paths
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
    # NuttX specific includes
    ${ESP_HAL_3RDPARTY_REPO}/nuttx/include
    ${ESP_HAL_3RDPARTY_REPO}/nuttx/include/mbedtls
    ${ESP_HAL_3RDPARTY_REPO}/nuttx/${CHIP_SERIES}/include
    ${ESP_HAL_3RDPARTY_REPO}/nuttx/src/components/esp_driver_uart/include
    # Bootloader support
    ${ESP_HAL_3RDPARTY_REPO}/components/bootloader_support/bootloader_flash/include
    ${ESP_HAL_3RDPARTY_REPO}/components/bootloader_support/include
    ${ESP_HAL_3RDPARTY_REPO}/components/bootloader_support/private_include
    # Driver components
    ${ESP_HAL_3RDPARTY_REPO}/components/driver/twai/include
    ${ESP_HAL_3RDPARTY_REPO}/components/driver/spi/include
    # EFUSE
    ${ESP_HAL_3RDPARTY_REPO}/components/efuse/include
    ${ESP_HAL_3RDPARTY_REPO}/components/efuse/private_include
    ${ESP_HAL_3RDPARTY_REPO}/components/efuse/${CHIP_SERIES}/include
    ${ESP_HAL_3RDPARTY_REPO}/components/efuse/${CHIP_SERIES}/private_include
    # ESP ADC
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_adc/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_adc/interface
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_adc/${CHIP_SERIES}/include
    # ESP Common
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_common/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_event/include
    # ESP HAL components (upper HAL and per-chip)
    ${ESP_HAL_3RDPARTY_REPO}/components/upper_hal_dma/include
    ${ESP_HAL_3RDPARTY_REPO}/components/upper_hal_dma/src
    ${ESP_HAL_3RDPARTY_REPO}/components/upper_hal_gpio/include
    ${ESP_HAL_3RDPARTY_REPO}/components/upper_hal_rmt/include
    ${ESP_HAL_3RDPARTY_REPO}/components/upper_hal_rmt/src
    ${ESP_HAL_3RDPARTY_REPO}/components/upper_hal_uart/include
    # ESP HAL RMT (hal/rmt_hal.h)
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_rmt/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_rmt/${CHIP_SERIES}/include
    # ESP HW Support
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/dma/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/include/esp_private
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/include/soc
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/include/soc/${CHIP_SERIES}
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/ldo/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/mspi/mspi_intr/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/mspi/mspi_timing_tuning/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/port/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/port/${CHIP_SERIES}/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/port/${CHIP_SERIES}/private_include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/power_supply/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/mspi_timing_tuning/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/etm/include
    # ESP MM
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_mm
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_mm/include
    # ESP PHY
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_phy/${CHIP_SERIES}/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_phy/include
    # ESP PM
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_pm
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_pm/include
    # ESP ROM
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_rom
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_rom/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_rom/${CHIP_SERIES}
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_rom/${CHIP_SERIES}/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_rom/${CHIP_SERIES}/include/${CHIP_SERIES}
    # ESP System
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_system/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_system/port/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_system/port/include/private
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_system/port/public_compat
    # ESP Timer
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_timer/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_timer/private_include
    # ESP Wi-Fi
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_wifi/include
    # ESP Security
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_security/include
    # HAL
    ${ESP_HAL_3RDPARTY_REPO}/components/hal/${CHIP_SERIES}/include
    ${ESP_HAL_3RDPARTY_REPO}/components/hal/include
    ${ESP_HAL_3RDPARTY_REPO}/components/hal/platform_port/include
    # HAL sub-components (hal/*.h)
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_ana_conv/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_ana_conv/${CHIP_SERIES}/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_clock/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_clock/${CHIP_SERIES}/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_dma/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_dma/${CHIP_SERIES}/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_gpio/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_gpio/${CHIP_SERIES}/include
    # ESP HAL I2C, I2S, LEDC, Security, TWAI (for HAL headers)
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_i2c/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_i2c/${CHIP_SERIES}/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_i2s/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_i2s/${CHIP_SERIES}/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_ledc/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_ledc/${CHIP_SERIES}/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_security/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_security/${CHIP_SERIES}/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_twai/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_twai/${CHIP_SERIES}/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_gpspi/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_gpspi/${CHIP_SERIES}/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_mspi/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_mspi/${CHIP_SERIES}/include
    # ESP HAL PMU (for sleep_cpu: hal/rtc_hal.h)
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_pmu/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_pmu/${CHIP_SERIES}/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_rtc_timer/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_rtc_timer/${CHIP_SERIES}/include
    # ESP HAL TIMG (for wdt: hal/timg_ll.h)
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_timg/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_timg/${CHIP_SERIES}/include
    # ESP HAL UART (for sleep_uart: hal/uart_hal.h)
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_uart/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_uart/${CHIP_SERIES}/include
    # ESP HAL USB (for sleep_console: hal/usb_serial_jtag_ll.h)
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_usb/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_usb/${CHIP_SERIES}/include
    # ESP HAL WDT (for bootloader: hal/wdt_hal.h)
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_wdt/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_wdt/${CHIP_SERIES}/include
    # Heap
    ${ESP_HAL_3RDPARTY_REPO}/components/heap/include
    # Log
    ${ESP_HAL_3RDPARTY_REPO}/components/log
    ${ESP_HAL_3RDPARTY_REPO}/components/log/include
    ${ESP_HAL_3RDPARTY_REPO}/components/log/src/log_level/tag_log_level
    # mbedTLS (tf-psa-crypto builtin must come before mbedtls/include so
    # mbedtls/private/* are found)
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/port/include
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/port/include/aes
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/port/psa_driver/include
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/drivers/builtin/include
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/include
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/core
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/include
    # Newlib / esp_libc
    ${ESP_HAL_3RDPARTY_REPO}/components/newlib/priv_include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_libc/priv_include
    ${ESP_HAL_3RDPARTY_REPO}/nuttx/src/components/esp_libc/platform_include
    # RISC-V
    ${ESP_HAL_3RDPARTY_REPO}/components/riscv/include
    # SOC
    ${ESP_HAL_3RDPARTY_REPO}/components/soc/include
    ${ESP_HAL_3RDPARTY_REPO}/components/soc/${CHIP_SERIES}/include
    ${ESP_HAL_3RDPARTY_REPO}/components/soc/${CHIP_SERIES}/register
    # SPI Flash
    ${ESP_HAL_3RDPARTY_REPO}/components/spi_flash/include
    ${ESP_HAL_3RDPARTY_REPO}/components/spi_flash/include/spi_flash
    ${ESP_HAL_3RDPARTY_REPO}/components/spi_flash/include/esp_flash_chips
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_blockdev/include)

# Additional includes for simple boot
if(CONFIG_ESPRESSIF_SIMPLE_BOOT)
  target_include_directories(
    arch PRIVATE ${ESP_HAL_3RDPARTY_REPO}/components/esp_app_format/include)
endif()

# ##############################################################################
# Linker Scripts
# ##############################################################################

set(ESP_ROM_LD_DIR
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_rom/${CHIP_SERIES}/ld)
set(ESP_SOC_LD_DIR ${ESP_HAL_3RDPARTY_REPO}/components/soc/${CHIP_SERIES}/ld)
set(ESP_RISCV_LD_DIR ${ESP_HAL_3RDPARTY_REPO}/components/riscv/ld)

set(_esp32c3_rom_ld_files
    ${ESP_ROM_LD_DIR}/${CHIP_SERIES}.rom.api.ld
    ${ESP_ROM_LD_DIR}/${CHIP_SERIES}.rom.bt_funcs.ld
    ${ESP_ROM_LD_DIR}/${CHIP_SERIES}.rom.eco3_bt_funcs.ld
    ${ESP_ROM_LD_DIR}/${CHIP_SERIES}.rom.eco3.ld
    ${ESP_ROM_LD_DIR}/${CHIP_SERIES}.rom.ld
    ${ESP_ROM_LD_DIR}/${CHIP_SERIES}.rom.libc.ld
    ${ESP_ROM_LD_DIR}/${CHIP_SERIES}.rom.libc-suboptimal_for_misaligned_mem.ld
    ${ESP_ROM_LD_DIR}/${CHIP_SERIES}.rom.libgcc.ld
    ${ESP_ROM_LD_DIR}/${CHIP_SERIES}.rom.newlib.ld
    ${ESP_ROM_LD_DIR}/${CHIP_SERIES}.rom.version.ld
    ${ESP_SOC_LD_DIR}/${CHIP_SERIES}.peripherals.ld
    ${ESP_RISCV_LD_DIR}/rom.api.ld)

# Add these files to the GLOBAL PROPERTY LD_SCRIPT
set_property(GLOBAL APPEND PROPERTY LD_SCRIPT ${_esp32c3_rom_ld_files})

# ##############################################################################
# HAL Source Files
# ##############################################################################

set(HAL_SRCS)

# ESP ADC sources
list(
  APPEND
  HAL_SRCS
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_adc/adc_cali_curve_fitting.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_adc/adc_cali.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_adc/${CHIP_SERIES}/curve_fitting_coefficients.c
)

# Bootloader support sources
list(
  APPEND HAL_SRCS
  ${ESP_HAL_3RDPARTY_REPO}/components/bootloader_support/src/bootloader_init.c
  ${ESP_HAL_3RDPARTY_REPO}/components/bootloader_support/src/bootloader_efuse.c)

# EFUSE sources
list(
  APPEND
  HAL_SRCS
  ${ESP_HAL_3RDPARTY_REPO}/components/efuse/src/esp_efuse_api.c
  ${ESP_HAL_3RDPARTY_REPO}/components/efuse/src/esp_efuse_utility.c
  ${ESP_HAL_3RDPARTY_REPO}/components/efuse/src/esp_efuse_fields.c
  ${ESP_HAL_3RDPARTY_REPO}/components/efuse/src/esp_efuse_startup.c
  ${ESP_HAL_3RDPARTY_REPO}/components/efuse/${CHIP_SERIES}/esp_efuse_fields.c
  ${ESP_HAL_3RDPARTY_REPO}/components/efuse/${CHIP_SERIES}/esp_efuse_rtc_calib.c
  ${ESP_HAL_3RDPARTY_REPO}/components/efuse/${CHIP_SERIES}/esp_efuse_table.c
  ${ESP_HAL_3RDPARTY_REPO}/components/efuse/${CHIP_SERIES}/esp_efuse_utility.c
  ${ESP_HAL_3RDPARTY_REPO}/components/efuse/src/efuse_controller/keys/with_key_purposes/esp_efuse_api_key.c
)

# ESP Common sources
list(APPEND HAL_SRCS
     ${ESP_HAL_3RDPARTY_REPO}/components/esp_common/src/esp_err_to_name.c)

# ESP HW Support sources
list(
  APPEND
  HAL_SRCS
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/adc_share_hw_ctrl.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/clk_ctrl_os.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/clk_utils.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/cpu.c
  ${ESP_HAL_3RDPARTY_REPO}/components/upper_hal_dma/src/gdma.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/esp_clk.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/esp_gpio_reserve.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/esp_memory_utils.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/hw_random.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/intr_alloc.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/lowpower/port/${CHIP_SERIES}/sleep_cpu.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/mac_addr.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/periph_ctrl.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/regi2c_ctrl.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/rtc_module.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/sleep_modes.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/sleep_uart.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/sleep_modem.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/port/esp_clk_tree_common.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/port/${CHIP_SERIES}/adc2_init_cal.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/port/${CHIP_SERIES}/esp_clk_tree.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/port/${CHIP_SERIES}/esp_cpu_intr.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/port/${CHIP_SERIES}/cpu_region_protect.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/port/${CHIP_SERIES}/rtc_clk.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/port/${CHIP_SERIES}/rtc_clk_init.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/port/${CHIP_SERIES}/rtc_init.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/port/${CHIP_SERIES}/rtc_sleep.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/port/${CHIP_SERIES}/rtc_time.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/port/${CHIP_SERIES}/sar_periph_ctrl.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/port/${CHIP_SERIES}/systimer.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/power_supply/brownout.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/sleep_console.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/sleep_event.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/sleep_gpio.c)

# ESP MM sources
list(
  APPEND
  HAL_SRCS
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_mm/esp_cache_msync.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_mm/esp_mmu_map.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_mm/esp_cache_utils.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_mm/port/${CHIP_SERIES}/ext_mem_layout.c
  ${ESP_HAL_3RDPARTY_REPO}/components/heap/port/${CHIP_SERIES}/memory_layout.c)

# ESP PHY sources
list(
  APPEND
  HAL_SRCS
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_phy/src/lib_printf.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_phy/src/phy_common.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_phy/src/phy_init.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_phy/${CHIP_SERIES}/phy_init_data.c)

# ESP PM sources
list(APPEND HAL_SRCS ${ESP_HAL_3RDPARTY_REPO}/components/esp_pm/pm_locks.c
     ${ESP_HAL_3RDPARTY_REPO}/components/esp_pm/pm_impl.c)

# ESP ROM sources
list(
  APPEND
  HAL_SRCS
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_rom/patches/esp_rom_sys.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_rom/patches/esp_rom_print.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_rom/patches/esp_rom_crc.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_rom/patches/esp_rom_serial_output.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_rom/patches/esp_rom_spiflash.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_rom/patches/esp_rom_efuse.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_rom/patches/esp_rom_gpio.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_rom/patches/esp_rom_systimer.c)

# ESP System sources
list(
  APPEND
  HAL_SRCS
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_system/esp_err.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_system/esp_system.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_system/startup.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_system/startup_funcs.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_system/system_time.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_system/port/soc/${CHIP_SERIES}/clk.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_system/port/soc/${CHIP_SERIES}/system_internal.c
)

# ESP Timer
list(
  APPEND
  HAL_SRCS
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_timer/src/esp_timer.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_timer/src/esp_timer_init.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_timer/src/esp_timer_impl_systimer.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_timer/src/system_time.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_timer/src/esp_timer_impl_common.c)

# Make) HAL sources (paths from hal_esp32c3.mk: esp_hal_* and hal components)
list(
  APPEND
  HAL_SRCS
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_ana_conv/adc_hal_common.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_ana_conv/adc_oneshot_hal.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_pmu/brownout_hal.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_pmu/${CHIP_SERIES}/rtc_cntl_hal.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_security/hmac_hal.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_security/aes_hal.c
  ${ESP_HAL_3RDPARTY_REPO}/components/hal/cache_hal.c
  ${ESP_HAL_3RDPARTY_REPO}/components/hal/efuse_hal.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_gpio/gpio_hal.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_dma/gdma_hal_ahb_v1.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_dma/gdma_hal_top.c
  ${ESP_HAL_3RDPARTY_REPO}/components/hal/hal_utils.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_ledc/ledc_hal.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_ledc/ledc_hal_iram.c
  ${ESP_HAL_3RDPARTY_REPO}/components/hal/systimer_hal.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_timg/timer_hal.c
  ${ESP_HAL_3RDPARTY_REPO}/components/hal/mmu_hal.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_rmt/rmt_hal.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_gpio/sdm_hal.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_i2c/i2c_hal.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_i2s/i2s_hal.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_security/sha_hal.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_gpspi/spi_hal.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_gpspi/spi_hal_iram.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_twai/twai_hal_v1.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_uart/uart_hal.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_uart/uart_hal_iram.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_wdt/wdt_hal_iram.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_mspi/spi_flash_hal.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_mspi/spi_flash_hal_iram.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_mspi/spi_flash_hal_gpspi.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_mspi/spi_flash_encrypt_hal_iram.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_wdt/xt_wdt_hal.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_clock/${CHIP_SERIES}/clk_tree_hal.c
  ${ESP_HAL_3RDPARTY_REPO}/components/hal/${CHIP_SERIES}/efuse_hal.c)

# Log sources
list(
  APPEND
  HAL_SRCS
  ${ESP_HAL_3RDPARTY_REPO}/components/log/src/log.c
  ${ESP_HAL_3RDPARTY_REPO}/components/log/src/log_level/log_level.c
  ${ESP_HAL_3RDPARTY_REPO}/components/log/src/log_level/tag_log_level/tag_log_level.c
  ${ESP_HAL_3RDPARTY_REPO}/components/log/src/log_level/tag_log_level/linked_list/log_linked_list.c
  ${ESP_HAL_3RDPARTY_REPO}/components/log/src/noos/log_lock.c
  ${ESP_HAL_3RDPARTY_REPO}/components/log/src/noos/log_timestamp.c
  ${ESP_HAL_3RDPARTY_REPO}/components/log/src/os/log_write.c
  ${ESP_HAL_3RDPARTY_REPO}/components/log/src/os/util.c)

# RISC-V sources
list(
  APPEND
  HAL_SRCS
  ${ESP_HAL_3RDPARTY_REPO}/components/riscv/instruction_decode.c
  ${ESP_HAL_3RDPARTY_REPO}/components/riscv/interrupt.c
  ${ESP_HAL_3RDPARTY_REPO}/components/riscv/interrupt_intc.c
  ${ESP_HAL_3RDPARTY_REPO}/components/riscv/rv_utils.c)

# SPI Flash sources
list(
  APPEND
  HAL_SRCS
  ${ESP_HAL_3RDPARTY_REPO}/components/spi_flash/spi_flash_chip_generic.c
  ${ESP_HAL_3RDPARTY_REPO}/components/spi_flash/spi_flash_chip_boya.c
  ${ESP_HAL_3RDPARTY_REPO}/components/spi_flash/spi_flash_chip_gd.c
  ${ESP_HAL_3RDPARTY_REPO}/components/spi_flash/spi_flash_chip_winbond.c
  ${ESP_HAL_3RDPARTY_REPO}/components/spi_flash/spi_flash_chip_issi.c
  ${ESP_HAL_3RDPARTY_REPO}/components/spi_flash/spi_flash_chip_mxic_opi.c
  ${ESP_HAL_3RDPARTY_REPO}/components/spi_flash/spi_flash_chip_mxic.c
  ${ESP_HAL_3RDPARTY_REPO}/components/spi_flash/spi_flash_chip_th.c
  ${ESP_HAL_3RDPARTY_REPO}/components/spi_flash/spi_flash_chip_drivers.c
  ${ESP_HAL_3RDPARTY_REPO}/components/spi_flash/memspi_host_driver.c
  ${ESP_HAL_3RDPARTY_REPO}/components/spi_flash/flash_brownout_hook.c)

# Cache (relates to SPI Flash)

set(CACHE_SRCS)
list(
  APPEND
  CACHE_SRCS
  ${ESP_HAL_3RDPARTY_REPO}/components/spi_flash/cache_utils.c
  ${ESP_HAL_3RDPARTY_REPO}/components/spi_flash/flash_mmap.c
  ${ESP_HAL_3RDPARTY_REPO}/components/spi_flash/flash_ops.c
  ${ESP_HAL_3RDPARTY_REPO}/components/spi_flash/esp_flash_api.c
  ${ESP_HAL_3RDPARTY_REPO}/components/spi_flash/esp_flash_spi_init.c
  ${ESP_HAL_3RDPARTY_REPO}/components/spi_flash/spi_flash_os_func_app.c
  ${ESP_HAL_3RDPARTY_REPO}/components/spi_flash/spi_flash_os_func_noos.c
  ${ESP_HAL_3RDPARTY_REPO}/components/spi_flash/spi_flash_wrap.c)

list(APPEND HAL_SRCS ${CACHE_SRCS})

# Avoid cache miss by unexpected inlineing when built by -Os
set_source_files_properties(
  ${CACHE_SRCS} DIRECTORY ../../../../
  PROPERTIES
    COMPILE_FLAGS
    "-fno-inline-functions -fno-inline-small-functions -fno-inline-functions-called-once"
)

# SOC sources (paths from hal_esp32c3.mk: esp_hal_* periph and soc)
list(
  APPEND
  HAL_SRCS
  ${ESP_HAL_3RDPARTY_REPO}/components/soc/lldesc.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_ana_conv/${CHIP_SERIES}/adc_periph.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_gpio/${CHIP_SERIES}/dedic_gpio_periph.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_dma/${CHIP_SERIES}/gdma_periph.c
  ${ESP_HAL_3RDPARTY_REPO}/components/soc/${CHIP_SERIES}/gpio_periph.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_ledc/${CHIP_SERIES}/ledc_periph.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_rmt/${CHIP_SERIES}/rmt_periph.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_gpio/${CHIP_SERIES}/sdm_periph.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_i2c/${CHIP_SERIES}/i2c_periph.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_i2s/${CHIP_SERIES}/i2s_periph.c
  ${ESP_HAL_3RDPARTY_REPO}/components/soc/${CHIP_SERIES}/interrupts.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_gpspi/${CHIP_SERIES}/spi_periph.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_ana_conv/${CHIP_SERIES}/temperature_sensor_periph.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_timg/${CHIP_SERIES}/timer_periph.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_twai/${CHIP_SERIES}/twai_periph.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_uart/${CHIP_SERIES}/uart_periph.c)

# ESP Security sources
list(
  APPEND
  HAL_SRCS
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_security/src/esp_hmac.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_security/src/esp_crypto_lock.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_security/src/esp_crypto_periph_clk.c)

# NuttX platform sources (upper_hal_gpio, upper_hal_rmt, os, heap_caps, newlib
# init)
list(
  APPEND
  HAL_SRCS
  ${ESP_HAL_3RDPARTY_REPO}/nuttx/src/platform/os.c
  ${ESP_HAL_3RDPARTY_REPO}/nuttx/src/heap_caps.c
  ${ESP_HAL_3RDPARTY_REPO}/nuttx/src/components/newlib/newlib/libc/misc/init.c
  ${ESP_HAL_3RDPARTY_REPO}/components/upper_hal_gpio/src/gpio.c
  ${ESP_HAL_3RDPARTY_REPO}/components/upper_hal_gpio/src/rtc_io.c
  ${ESP_HAL_3RDPARTY_REPO}/components/upper_hal_rmt/src/rmt_common.c
  ${ESP_HAL_3RDPARTY_REPO}/components/upper_hal_rmt/src/rmt_encoder.c
  ${ESP_HAL_3RDPARTY_REPO}/components/upper_hal_rmt/src/rmt_encoder_bytes.c
  ${ESP_HAL_3RDPARTY_REPO}/components/upper_hal_rmt/src/rmt_encoder_copy.c
  ${ESP_HAL_3RDPARTY_REPO}/components/upper_hal_rmt/src/rmt_encoder_simple.c
  ${ESP_HAL_3RDPARTY_REPO}/components/upper_hal_rmt/src/rmt_rx.c
  ${ESP_HAL_3RDPARTY_REPO}/components/upper_hal_rmt/src/rmt_tx.c
  ${ESP_HAL_3RDPARTY_REPO}/components/upper_hal_uart/src/uart_wakeup.c)

if(CONFIG_ESPRESSIF_WIFI OR CONFIG_ESPRESSIF_EMAC)
  list(APPEND HAL_SRCS ${ESP_HAL_3RDPARTY_REPO}/nuttx/src/esp_event.c)
endif()

# Bootloader flash encrypt source
list(APPEND HAL_SRCS
     ${ESP_HAL_3RDPARTY_REPO}/components/bootloader_support/src/flash_encrypt.c)

# Bootloader common
list(
  APPEND HAL_SRCS
  ${ESP_HAL_3RDPARTY_REPO}/components/bootloader_support/src/bootloader_mem.c)

# ##############################################################################
# Simple Boot Sources
# ##############################################################################

if(CONFIG_ESPRESSIF_SIMPLE_BOOT)
  list(
    APPEND
    HAL_SRCS
    ${ESP_HAL_3RDPARTY_REPO}/nuttx/src/bootloader_banner_wrap.c
    ${ESP_HAL_3RDPARTY_REPO}/components/bootloader_support/src/bootloader_console.c
    ${ESP_HAL_3RDPARTY_REPO}/components/bootloader_support/src/bootloader_console_loader.c
    ${ESP_HAL_3RDPARTY_REPO}/components/bootloader_support/src/${CHIP_SERIES}/bootloader_${CHIP_SERIES}.c
    ${ESP_HAL_3RDPARTY_REPO}/components/bootloader_support/src/bootloader_common.c
    ${ESP_HAL_3RDPARTY_REPO}/components/bootloader_support/src/bootloader_common_loader.c
    ${ESP_HAL_3RDPARTY_REPO}/components/bootloader_support/bootloader_flash/src/bootloader_flash.c
    ${ESP_HAL_3RDPARTY_REPO}/components/bootloader_support/bootloader_flash/src/bootloader_flash_config_${CHIP_SERIES}.c
    ${ESP_HAL_3RDPARTY_REPO}/components/bootloader_support/bootloader_flash/src/flash_qio_mode.c
    ${ESP_HAL_3RDPARTY_REPO}/components/bootloader_support/src/bootloader_clock_init.c
    ${ESP_HAL_3RDPARTY_REPO}/components/bootloader_support/src/bootloader_random.c
    ${ESP_HAL_3RDPARTY_REPO}/components/bootloader_support/src/bootloader_random_${CHIP_SERIES}.c
    ${ESP_HAL_3RDPARTY_REPO}/components/bootloader_support/src/esp_image_format.c
    ${ESP_HAL_3RDPARTY_REPO}/components/bootloader_support/src/bootloader_sha.c
    ${ESP_HAL_3RDPARTY_REPO}/components/bootloader_support/src/${CHIP_SERIES}/bootloader_soc.c
  )

  target_link_options(nuttx PRIVATE -Wl,--wrap=bootloader_print_banner)

elseif(CONFIG_ESPRESSIF_EFUSE_VIRTUAL_KEEP_IN_FLASH)
  # Special case for bootloader_flash when using efuse virtual mode
  list(
    APPEND
    HAL_SRCS
    ${ESP_HAL_3RDPARTY_REPO}/components/bootloader_support/bootloader_flash/src/bootloader_flash.c
    ${ESP_HAL_3RDPARTY_REPO}/components/bootloader_support/bootloader_flash/src/bootloader_flash_config_${CHIP_SERIES}.c
  )
endif()

# ##############################################################################
# Add HAL sources to arch target
# ##############################################################################

target_sources(arch PRIVATE ${HAL_SRCS})
