# ##############################################################################
# arch/risc-v/src/esp32h2/hal_esp32h2.cmake
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
# Include Paths (from hal_esp32h2.mk)
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
    ${ESP_HAL_3RDPARTY_REPO}/components/bootloader_support/bootloader_flash/include
    ${ESP_HAL_3RDPARTY_REPO}/components/bootloader_support/include
    ${ESP_HAL_3RDPARTY_REPO}/components/bootloader_support/private_include
    ${ESP_HAL_3RDPARTY_REPO}/components/driver/twai/include
    ${ESP_HAL_3RDPARTY_REPO}/components/driver/spi/include
    ${ESP_HAL_3RDPARTY_REPO}/components/efuse/include
    ${ESP_HAL_3RDPARTY_REPO}/components/efuse/private_include
    ${ESP_HAL_3RDPARTY_REPO}/components/efuse/${CHIP_SERIES}/include
    ${ESP_HAL_3RDPARTY_REPO}/components/efuse/${CHIP_SERIES}/private_include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_adc/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_adc/interface
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_adc/${CHIP_SERIES}/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_blockdev/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_common/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_event/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_ana_conv/${CHIP_SERIES}/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_ana_conv/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_clock/${CHIP_SERIES}/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_clock/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_dma/${CHIP_SERIES}/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_dma/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_gpio/${CHIP_SERIES}/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_gpio/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_gpspi/${CHIP_SERIES}/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_gpspi/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_i2c/${CHIP_SERIES}/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_i2c/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_i2s/${CHIP_SERIES}/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_i2s/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_ledc/${CHIP_SERIES}/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_ledc/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_mcpwm/${CHIP_SERIES}/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_mcpwm/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_mspi/${CHIP_SERIES}/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_mspi/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_parlio/${CHIP_SERIES}/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_parlio/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_pcnt/${CHIP_SERIES}/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_pcnt/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_pmu/${CHIP_SERIES}/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_pmu/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_rmt/${CHIP_SERIES}/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_rmt/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_rtc_timer/${CHIP_SERIES}/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_rtc_timer/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_security/${CHIP_SERIES}/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_security/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_timg/${CHIP_SERIES}/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_timg/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_touch_sens/${CHIP_SERIES}/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_touch_sens/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_twai/${CHIP_SERIES}/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_twai/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_uart/${CHIP_SERIES}/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_uart/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_usb/${CHIP_SERIES}/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_usb/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_wdt/${CHIP_SERIES}/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_wdt/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/dma/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/etm/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/include/esp_private
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/include/soc
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/include/soc/${CHIP_SERIES}
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/mspi/mspi_intr/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/mspi/mspi_timing_tuning/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/mspi/mspi_timing_tuning/port/${CHIP_SERIES}
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/mspi/mspi_timing_tuning/tuning_scheme_impl/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/ldo/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/port/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/port/${CHIP_SERIES}/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/port/${CHIP_SERIES}/private_include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/power_supply/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_mm/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_pm
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_pm/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_rom
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_rom/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_rom/${CHIP_SERIES}
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_rom/${CHIP_SERIES}/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_rom/${CHIP_SERIES}/include/${CHIP_SERIES}
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_security/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_system/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_system/port/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_system/port/include/private
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_system/port/public_compat
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_timer/include
    ${ESP_HAL_3RDPARTY_REPO}/components/hal/${CHIP_SERIES}/include
    ${ESP_HAL_3RDPARTY_REPO}/components/hal/include
    ${ESP_HAL_3RDPARTY_REPO}/components/hal/platform_port/include
    ${ESP_HAL_3RDPARTY_REPO}/components/heap/include
    ${ESP_HAL_3RDPARTY_REPO}/components/log
    ${ESP_HAL_3RDPARTY_REPO}/components/log/include
    ${ESP_HAL_3RDPARTY_REPO}/components/log/src/log_level/tag_log_level
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/port/include
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/port/include/aes
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/port/psa_driver/include
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/include
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/core
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/drivers/builtin/include
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_libc/priv_include
    ${ESP_HAL_3RDPARTY_REPO}/components/riscv/include
    ${ESP_HAL_3RDPARTY_REPO}/components/soc/include
    ${ESP_HAL_3RDPARTY_REPO}/components/soc/${CHIP_SERIES}/include
    ${ESP_HAL_3RDPARTY_REPO}/components/soc/${CHIP_SERIES}/register
    ${ESP_HAL_3RDPARTY_REPO}/components/spi_flash/include
    ${ESP_HAL_3RDPARTY_REPO}/components/spi_flash/include/esp_flash_chips
    ${ESP_HAL_3RDPARTY_REPO}/components/ulp
    ${ESP_HAL_3RDPARTY_REPO}/components/ulp/lp_core
    ${ESP_HAL_3RDPARTY_REPO}/components/ulp/lp_core/shared
    ${ESP_HAL_3RDPARTY_REPO}/components/ulp/lp_core/shared/include
    ${ESP_HAL_3RDPARTY_REPO}/components/ulp/lp_core/include
    ${ESP_HAL_3RDPARTY_REPO}/components/ulp/ulp_common
    ${ESP_HAL_3RDPARTY_REPO}/components/ulp/ulp_common/include
    ${ESP_HAL_3RDPARTY_REPO}/components/upper_hal_dma/include
    ${ESP_HAL_3RDPARTY_REPO}/components/upper_hal_dma/src
    ${ESP_HAL_3RDPARTY_REPO}/components/upper_hal_gpio/include
    ${ESP_HAL_3RDPARTY_REPO}/components/upper_hal_rmt/include
    ${ESP_HAL_3RDPARTY_REPO}/components/upper_hal_rmt/src
    ${ESP_HAL_3RDPARTY_REPO}/components/upper_hal_uart/include
    ${ESP_HAL_3RDPARTY_REPO}/nuttx/src/components/esp_libc/platform_include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/mspi_timing_tuning/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_timer/private_include
    ${ESP_HAL_3RDPARTY_REPO}/components/heap/private_include)

if(CONFIG_ESPRESSIF_SIMPLE_BOOT)
  target_include_directories(
    arch PRIVATE ${ESP_HAL_3RDPARTY_REPO}/components/esp_app_format/include)
endif()

# ##############################################################################
# Linker Scripts (from hal_esp32h2.mk)
# ##############################################################################

set(ESP_ROM_LD_DIR
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_rom/${CHIP_SERIES}/ld)
set(ESP_SOC_LD_DIR ${ESP_HAL_3RDPARTY_REPO}/components/soc/${CHIP_SERIES}/ld)
set(ESP_RISCV_LD_DIR ${ESP_HAL_3RDPARTY_REPO}/components/riscv/ld)
set(ESP_WDT_LD_DIR
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_wdt/${CHIP_SERIES})

set(_esp32h2_rom_ld_files
    ${ESP_ROM_LD_DIR}/${CHIP_SERIES}.rom.api.ld
    ${ESP_ROM_LD_DIR}/${CHIP_SERIES}.rom.ld
    ${ESP_ROM_LD_DIR}/${CHIP_SERIES}.rom.libc.ld
    ${ESP_ROM_LD_DIR}/${CHIP_SERIES}.rom.libc-suboptimal_for_misaligned_mem.ld
    ${ESP_ROM_LD_DIR}/${CHIP_SERIES}.rom.libgcc.ld
    ${ESP_ROM_LD_DIR}/${CHIP_SERIES}.rom.newlib.ld
    ${ESP_ROM_LD_DIR}/${CHIP_SERIES}.rom.version.ld
    ${ESP_WDT_LD_DIR}/rom.wdt.ld
    ${ESP_ROM_LD_DIR}/${CHIP_SERIES}.rom.heap.ld
    ${ESP_RISCV_LD_DIR}/rom.api.ld
    ${ESP_SOC_LD_DIR}/${CHIP_SERIES}.peripherals.ld)

if(CONFIG_ESPRESSIF_USE_LP_CORE)
  list(APPEND _esp32h2_rom_ld_files
       ${NUTTX_DIR}/arch/${CONFIG_ARCH}/src/board/scripts/ulp_aliases.ld)
endif()

if(CONFIG_ESPRESSIF_SPI_FLASH_USE_ROM_CODE)
  list(APPEND _esp32h2_rom_ld_files
       ${ESP_ROM_LD_DIR}/${CHIP_SERIES}.rom.spiflash.ld)
endif()

# Add these files to the GLOBAL PROPERTY LD_SCRIPT
set_property(GLOBAL APPEND PROPERTY LD_SCRIPT ${_esp32h2_rom_ld_files})

# ##############################################################################
# HAL Source Files (from hal_esp32h2.mk CHIP_CSRCS and CHIP_ASRCS)
# ##############################################################################

set(HAL_SRCS)

# ESP ADC
list(
  APPEND
  HAL_SRCS
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_adc/adc_cali_curve_fitting.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_adc/adc_cali.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_adc/${CHIP_SERIES}/curve_fitting_coefficients.c
)

# Bootloader support
list(
  APPEND HAL_SRCS
  ${ESP_HAL_3RDPARTY_REPO}/components/bootloader_support/src/bootloader_init.c
  ${ESP_HAL_3RDPARTY_REPO}/components/bootloader_support/src/bootloader_efuse.c)

# EFUSE
list(
  APPEND
  HAL_SRCS
  ${ESP_HAL_3RDPARTY_REPO}/components/efuse/src/esp_efuse_api.c
  ${ESP_HAL_3RDPARTY_REPO}/components/efuse/src/esp_efuse_utility.c
  ${ESP_HAL_3RDPARTY_REPO}/components/efuse/src/esp_efuse_startup.c
  ${ESP_HAL_3RDPARTY_REPO}/components/efuse/${CHIP_SERIES}/esp_efuse_fields.c
  ${ESP_HAL_3RDPARTY_REPO}/components/efuse/${CHIP_SERIES}/esp_efuse_rtc_calib.c
  ${ESP_HAL_3RDPARTY_REPO}/components/efuse/${CHIP_SERIES}/esp_efuse_table.c
  ${ESP_HAL_3RDPARTY_REPO}/components/efuse/${CHIP_SERIES}/esp_efuse_table_v0.0_v1.1.c
  ${ESP_HAL_3RDPARTY_REPO}/components/efuse/${CHIP_SERIES}/esp_efuse_utility.c
  ${ESP_HAL_3RDPARTY_REPO}/components/efuse/src/efuse_controller/keys/with_key_purposes/esp_efuse_api_key.c
)

# ESP Common
list(APPEND HAL_SRCS
     ${ESP_HAL_3RDPARTY_REPO}/components/esp_common/src/esp_err_to_name.c)

# ESP HW Support
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
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/mac_addr.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/modem_clock.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/periph_ctrl.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/regi2c_ctrl.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/rtc_module.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/sleep_console.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/sleep_event.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/sleep_gpio.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/sleep_modes.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/sleep_modem.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/sleep_uart.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/sleep_retention.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/sleep_system_peripheral.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/lowpower/port/${CHIP_SERIES}/sleep_cpu.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/lowpower/port/${CHIP_SERIES}/sleep_clock.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/port/esp_clk_tree_common.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/port/pau_regdma.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/port/regdma_link.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/port/${CHIP_SERIES}/esp_clk_tree.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/port/${CHIP_SERIES}/esp_cpu_intr.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/port/${CHIP_SERIES}/cpu_region_protect.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/port/${CHIP_SERIES}/peripheral_domain_pd.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/port/${CHIP_SERIES}/pmu_init.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/port/${CHIP_SERIES}/pmu_param.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/port/${CHIP_SERIES}/pmu_sleep.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/port/${CHIP_SERIES}/rtc_clk.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/port/${CHIP_SERIES}/rtc_clk_init.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/port/${CHIP_SERIES}/rtc_time.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/port/${CHIP_SERIES}/sar_periph_ctrl.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/port/${CHIP_SERIES}/systimer.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/power_supply/brownout.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/power_supply/vbat.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/mspi/mspi_timing_tuning/mspi_timing_tuning.c
)

# ESP MM
list(
  APPEND
  HAL_SRCS
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_mm/esp_cache_msync.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_mm/esp_mmu_map.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_mm/esp_cache_utils.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_mm/port/${CHIP_SERIES}/ext_mem_layout.c
)

# ESP PM
list(APPEND HAL_SRCS ${ESP_HAL_3RDPARTY_REPO}/components/esp_pm/pm_locks.c
     ${ESP_HAL_3RDPARTY_REPO}/components/esp_pm/pm_impl.c)

# ESP ROM
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
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_rom/patches/esp_rom_regi2c_${CHIP_SERIES}.c
)

# ESP System
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

# HAL components
list(
  APPEND
  HAL_SRCS
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_ana_conv/adc_hal_common.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_ana_conv/adc_oneshot_hal.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_security/apm_hal.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_security/aes_hal.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_security/hmac_hal.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_pmu/brownout_hal.c
  ${ESP_HAL_3RDPARTY_REPO}/components/hal/efuse_hal.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_gpio/gpio_hal.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_dma/gdma_hal_ahb_v1.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_dma/gdma_hal_top.c
  ${ESP_HAL_3RDPARTY_REPO}/components/hal/hal_utils.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_ledc/ledc_hal.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_ledc/ledc_hal_iram.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_pcnt/pcnt_hal.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_rmt/rmt_hal.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_gpio/sdm_hal.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_i2c/i2c_hal.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_i2s/i2s_hal.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_security/sha_hal.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_gpspi/spi_hal.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_gpspi/spi_hal_iram.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_timg/timer_hal.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_twai/twai_hal_v1.c
  ${ESP_HAL_3RDPARTY_REPO}/components/hal/cache_hal.c
  ${ESP_HAL_3RDPARTY_REPO}/components/hal/mmu_hal.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_mcpwm/mcpwm_hal.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_uart/uart_hal.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_uart/uart_hal_iram.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_wdt/wdt_hal_iram.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_mspi/spi_flash_hal.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_mspi/spi_flash_hal_iram.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_mspi/spi_flash_hal_gpspi.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_mspi/spi_flash_encrypt_hal_iram.c
  ${ESP_HAL_3RDPARTY_REPO}/components/hal/systimer_hal.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_gpio/rtc_io_hal.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_gpspi/spi_slave_hal_iram.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_gpspi/spi_slave_hal.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_clock/${CHIP_SERIES}/clk_tree_hal.c
  ${ESP_HAL_3RDPARTY_REPO}/components/hal/${CHIP_SERIES}/efuse_hal.c
  ${ESP_HAL_3RDPARTY_REPO}/components/hal/${CHIP_SERIES}/modem_clock_hal.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_pmu/${CHIP_SERIES}/pau_hal.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_pmu/${CHIP_SERIES}/pmu_hal.c)

# Log
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

# RISC-V (C6 uses interrupt_plic)
list(
  APPEND
  HAL_SRCS
  ${ESP_HAL_3RDPARTY_REPO}/components/riscv/instruction_decode.c
  ${ESP_HAL_3RDPARTY_REPO}/components/riscv/interrupt.c
  ${ESP_HAL_3RDPARTY_REPO}/components/riscv/interrupt_plic.c
  ${ESP_HAL_3RDPARTY_REPO}/components/riscv/rv_utils.c)

# SOC / periph
list(
  APPEND
  HAL_SRCS
  ${ESP_HAL_3RDPARTY_REPO}/components/soc/lldesc.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_ana_conv/${CHIP_SERIES}/adc_periph.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_gpio/${CHIP_SERIES}/dedic_gpio_periph.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_dma/${CHIP_SERIES}/gdma_periph.c
  ${ESP_HAL_3RDPARTY_REPO}/components/soc/${CHIP_SERIES}/interrupts.c
  ${ESP_HAL_3RDPARTY_REPO}/components/soc/${CHIP_SERIES}/gpio_periph.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_ledc/${CHIP_SERIES}/ledc_periph.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_pcnt/${CHIP_SERIES}/pcnt_periph.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_rmt/${CHIP_SERIES}/rmt_periph.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_gpio/${CHIP_SERIES}/sdm_periph.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_i2c/${CHIP_SERIES}/i2c_periph.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_i2s/${CHIP_SERIES}/i2s_periph.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_mcpwm/${CHIP_SERIES}/mcpwm_periph.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_gpio/${CHIP_SERIES}/rtc_io_periph.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_gpspi/${CHIP_SERIES}/spi_periph.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_ana_conv/${CHIP_SERIES}/temperature_sensor_periph.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_timg/${CHIP_SERIES}/timer_periph.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_twai/${CHIP_SERIES}/twai_periph.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hal_uart/${CHIP_SERIES}/uart_periph.c)

# ESP Security
list(
  APPEND
  HAL_SRCS
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_security/src/esp_hmac.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_security/src/esp_crypto_lock.c
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_security/src/esp_crypto_periph_clk.c)

# SPI Flash (C6: generic, gd, winbond only)
list(
  APPEND
  HAL_SRCS
  ${ESP_HAL_3RDPARTY_REPO}/components/spi_flash/spi_flash_os_func_noos.c
  ${ESP_HAL_3RDPARTY_REPO}/components/spi_flash/spi_flash_os_func_app.c
  ${ESP_HAL_3RDPARTY_REPO}/components/spi_flash/spi_flash_chip_generic.c
  ${ESP_HAL_3RDPARTY_REPO}/components/spi_flash/spi_flash_chip_gd.c
  ${ESP_HAL_3RDPARTY_REPO}/components/spi_flash/spi_flash_chip_winbond.c
  ${ESP_HAL_3RDPARTY_REPO}/components/spi_flash/esp_flash_api.c
  ${ESP_HAL_3RDPARTY_REPO}/components/spi_flash/flash_ops.c
  ${ESP_HAL_3RDPARTY_REPO}/components/spi_flash/spi_flash_chip_drivers.c
  ${ESP_HAL_3RDPARTY_REPO}/components/spi_flash/memspi_host_driver.c
  ${ESP_HAL_3RDPARTY_REPO}/components/spi_flash/esp_flash_spi_init.c
  ${ESP_HAL_3RDPARTY_REPO}/components/spi_flash/flash_mmap.c
  ${ESP_HAL_3RDPARTY_REPO}/components/spi_flash/flash_brownout_hook.c
  ${ESP_HAL_3RDPARTY_REPO}/components/spi_flash/cache_utils.c)

# Upper HAL RMT / GPIO
list(
  APPEND
  HAL_SRCS
  ${ESP_HAL_3RDPARTY_REPO}/components/upper_hal_rmt/src/rmt_common.c
  ${ESP_HAL_3RDPARTY_REPO}/components/upper_hal_rmt/src/rmt_encoder.c
  ${ESP_HAL_3RDPARTY_REPO}/components/upper_hal_rmt/src/rmt_encoder_bytes.c
  ${ESP_HAL_3RDPARTY_REPO}/components/upper_hal_rmt/src/rmt_encoder_copy.c
  ${ESP_HAL_3RDPARTY_REPO}/components/upper_hal_rmt/src/rmt_encoder_simple.c
  ${ESP_HAL_3RDPARTY_REPO}/components/upper_hal_rmt/src/rmt_rx.c
  ${ESP_HAL_3RDPARTY_REPO}/components/upper_hal_rmt/src/rmt_tx.c
  ${ESP_HAL_3RDPARTY_REPO}/components/upper_hal_gpio/src/gpio.c
  ${ESP_HAL_3RDPARTY_REPO}/components/upper_hal_gpio/src/rtc_io.c
  ${ESP_HAL_3RDPARTY_REPO}/components/upper_hal_uart/src/uart_wakeup.c)

# Sleep ASM
list(
  APPEND
  HAL_SRCS
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_hw_support/lowpower/port/esp32h2/sleep_cpu_asm.S
)

# NuttX platform
list(
  APPEND HAL_SRCS ${ESP_HAL_3RDPARTY_REPO}/nuttx/src/platform/os.c
  ${ESP_HAL_3RDPARTY_REPO}/nuttx/src/heap_caps.c
  ${ESP_HAL_3RDPARTY_REPO}/nuttx/src/components/newlib/newlib/libc/misc/init.c)

if(CONFIG_ESPRESSIF_WIFI OR CONFIG_ESPRESSIF_EMAC)
  list(APPEND HAL_SRCS ${ESP_HAL_3RDPARTY_REPO}/nuttx/src/esp_event.c)
endif()

# Bootloader flash encrypt
list(APPEND HAL_SRCS
     ${ESP_HAL_3RDPARTY_REPO}/components/bootloader_support/src/flash_encrypt.c)

# Bootloader common
list(
  APPEND HAL_SRCS
  ${ESP_HAL_3RDPARTY_REPO}/components/bootloader_support/src/bootloader_mem.c)

# ##############################################################################
# Simple Boot
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
    ${ESP_HAL_3RDPARTY_REPO}/components/efuse/src/esp_efuse_fields.c)
  target_link_options(nuttx PRIVATE -Wl,--wrap=bootloader_print_banner)
elseif(CONFIG_ESPRESSIF_EFUSE_VIRTUAL_KEEP_IN_FLASH)
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

# ##############################################################################
# This override is required to allow linker script to include "sdkconfig.h" and
# "ld.common". Store path in a GLOBAL property (scope when hal runs); the
# overridden function runs later from top-level and reads it.
# ##############################################################################

set_property(
  GLOBAL PROPERTY LD_SCRIPT_ESPRESSIF_HAL_INCLUDE_DIR
                  ${ESP_HAL_3RDPARTY_REPO}/nuttx/${CHIP_SERIES}/include)

set_property(GLOBAL PROPERTY LD_SCRIPT_ESPRESSIF_ADDITIONAL_INCLUDE_DIR
                             ${ESP_HAL_3RDPARTY_REPO}/components/esp_system/ld)

function(nuttx_generate_preprocess_target)
  nuttx_parse_function_args(
    FUNC
    nuttx_generate_preprocess_target
    ONE_VALUE
    SOURCE_FILE
    TARGET_FILE
    MULTI_VALUE
    DEPENDS
    REQUIRED
    SOURCE_FILE
    TARGET_FILE
    ARGN
    ${ARGN})

  get_property(LD_SCRIPT_HAL_DIR GLOBAL
               PROPERTY LD_SCRIPT_ESPRESSIF_HAL_INCLUDE_DIR)
  get_property(LD_SCRIPT_ADDITIONAL_DIR GLOBAL
               PROPERTY LD_SCRIPT_ESPRESSIF_ADDITIONAL_INCLUDE_DIR)
  set(LD_SCRIPT_HAL_INCLUDE)
  set(LD_SCRIPT_ADDITIONAL_INCLUDE)
  if(LD_SCRIPT_HAL_DIR)
    set(LD_SCRIPT_HAL_INCLUDE -I${LD_SCRIPT_HAL_DIR})
  endif()
  if(LD_SCRIPT_ADDITIONAL_DIR)
    set(LD_SCRIPT_ADDITIONAL_INCLUDE -I${LD_SCRIPT_ADDITIONAL_DIR})
  endif()
  add_custom_command(
    OUTPUT ${TARGET_FILE}
    COMMAND
      ${PREPROCESS} -I${CMAKE_BINARY_DIR}/include -I${NUTTX_DIR}/include
      -I${NUTTX_CHIP_ABS_DIR} ${LD_SCRIPT_HAL_INCLUDE}
      ${LD_SCRIPT_ADDITIONAL_INCLUDE} -D__NuttX__ ${SOURCE_FILE} >
      ${TARGET_FILE}
    DEPENDS ${SOURCE_FILE} ${DEPENDS})
endfunction()
