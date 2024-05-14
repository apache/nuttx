############################################################################
# arch/risc-v/src/esp32c6/esp32c6.mk
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
#
############################################################################

# Include header paths

INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)nuttx$(DELIM)include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)nuttx$(DELIM)$(CHIP_SERIES)$(DELIM)include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)driver$(DELIM)twai$(DELIM)include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)efuse$(DELIM)include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)efuse$(DELIM)private_include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)efuse$(DELIM)$(CHIP_SERIES)$(DELIM)include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)efuse$(DELIM)$(CHIP_SERIES)$(DELIM)private_include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_common$(DELIM)include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_event$(DELIM)include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_hw_support$(DELIM)include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_hw_support$(DELIM)include$(DELIM)esp_private
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_hw_support$(DELIM)include$(DELIM)soc
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_hw_support$(DELIM)port$(DELIM)include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_hw_support$(DELIM)port$(DELIM)$(CHIP_SERIES)
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_hw_support$(DELIM)port$(DELIM)$(CHIP_SERIES)$(DELIM)private_include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_phy$(DELIM)$(CHIP_SERIES)$(DELIM)include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_phy$(DELIM)include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_rom$(DELIM)include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_rom$(DELIM)include$(DELIM)$(CHIP_SERIES)
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_rom$(DELIM)$(CHIP_SERIES)
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_system$(DELIM)include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_system$(DELIM)port$(DELIM)include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_system$(DELIM)port$(DELIM)include$(DELIM)private
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_system$(DELIM)port$(DELIM)public_compat
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_timer$(DELIM)include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_wifi$(DELIM)include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)hal$(DELIM)$(CHIP_SERIES)$(DELIM)include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)hal$(DELIM)include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)hal$(DELIM)platform_port$(DELIM)include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)log
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)log$(DELIM)include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)riscv$(DELIM)include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)soc$(DELIM)include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)soc$(DELIM)$(CHIP_SERIES)$(DELIM)include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)spi_flash$(DELIM)include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)spi_flash$(DELIM)include$(DELIM)spi_flash

ifeq ($(CONFIG_ESPRESSIF_SIMPLE_BOOT),y)
  INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)bootloader_support$(DELIM)include
  INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)bootloader_support$(DELIM)private_include
  INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)bootloader_support$(DELIM)bootloader_flash$(DELIM)include
  INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)spi_flash$(DELIM)include
  INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)spi_flash$(DELIM)include$(DELIM)spi_flash
  INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_app_format$(DELIM)include
endif

# Linker scripts

ARCHSCRIPT += $(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_rom$(DELIM)$(CHIP_SERIES)$(DELIM)ld$(DELIM)$(CHIP_SERIES).rom.api.ld
ARCHSCRIPT += $(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_rom$(DELIM)$(CHIP_SERIES)$(DELIM)ld$(DELIM)$(CHIP_SERIES).rom.coexist.ld
ARCHSCRIPT += $(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_rom$(DELIM)$(CHIP_SERIES)$(DELIM)ld$(DELIM)$(CHIP_SERIES).rom.ld
ARCHSCRIPT += $(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_rom$(DELIM)$(CHIP_SERIES)$(DELIM)ld$(DELIM)$(CHIP_SERIES).rom.net80211.ld
ARCHSCRIPT += $(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_rom$(DELIM)$(CHIP_SERIES)$(DELIM)ld$(DELIM)$(CHIP_SERIES).rom.newlib.ld
ARCHSCRIPT += $(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_rom$(DELIM)$(CHIP_SERIES)$(DELIM)ld$(DELIM)$(CHIP_SERIES).rom.phy.ld
ARCHSCRIPT += $(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_rom$(DELIM)$(CHIP_SERIES)$(DELIM)ld$(DELIM)$(CHIP_SERIES).rom.pp.ld
ARCHSCRIPT += $(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_rom$(DELIM)$(CHIP_SERIES)$(DELIM)ld$(DELIM)$(CHIP_SERIES).rom.spiflash.ld
ARCHSCRIPT += $(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_rom$(DELIM)$(CHIP_SERIES)$(DELIM)ld$(DELIM)$(CHIP_SERIES).rom.version.ld
ARCHSCRIPT += $(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_rom$(DELIM)$(CHIP_SERIES)$(DELIM)ld$(DELIM)$(CHIP_SERIES).rom.wdt.ld
ARCHSCRIPT += $(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)soc$(DELIM)$(CHIP_SERIES)$(DELIM)ld$(DELIM)$(CHIP_SERIES).peripherals.ld

# Source files

CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)efuse$(DELIM)src$(DELIM)esp_efuse_api.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)efuse$(DELIM)src$(DELIM)esp_efuse_utility.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)efuse$(DELIM)$(CHIP_SERIES)$(DELIM)esp_efuse_fields.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)efuse$(DELIM)$(CHIP_SERIES)$(DELIM)esp_efuse_table.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)efuse$(DELIM)$(CHIP_SERIES)$(DELIM)esp_efuse_utility.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_common$(DELIM)src$(DELIM)esp_err_to_name.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_hw_support$(DELIM)clk_ctrl_os.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_hw_support$(DELIM)cpu.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_hw_support$(DELIM)esp_clk.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_hw_support$(DELIM)hw_random.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_hw_support$(DELIM)mac_addr.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_hw_support$(DELIM)modem_clock.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_hw_support$(DELIM)periph_ctrl.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_hw_support$(DELIM)regi2c_ctrl.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_hw_support$(DELIM)sleep_modem.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_hw_support$(DELIM)port$(DELIM)esp_clk_tree_common.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_hw_support$(DELIM)port$(DELIM)$(CHIP_SERIES)$(DELIM)esp_clk_tree.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_hw_support$(DELIM)port$(DELIM)$(CHIP_SERIES)$(DELIM)cpu_region_protect.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_hw_support$(DELIM)port$(DELIM)$(CHIP_SERIES)$(DELIM)ocode_init.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_hw_support$(DELIM)port$(DELIM)$(CHIP_SERIES)$(DELIM)pmu_init.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_hw_support$(DELIM)port$(DELIM)$(CHIP_SERIES)$(DELIM)pmu_param.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_hw_support$(DELIM)port$(DELIM)$(CHIP_SERIES)$(DELIM)pmu_sleep.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_hw_support$(DELIM)port$(DELIM)$(CHIP_SERIES)$(DELIM)rtc_clk.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_hw_support$(DELIM)port$(DELIM)$(CHIP_SERIES)$(DELIM)rtc_clk_init.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_hw_support$(DELIM)port$(DELIM)$(CHIP_SERIES)$(DELIM)rtc_time.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_hw_support$(DELIM)port$(DELIM)$(CHIP_SERIES)$(DELIM)sar_periph_ctrl.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_hw_support$(DELIM)port$(DELIM)$(CHIP_SERIES)$(DELIM)systimer.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_phy$(DELIM)src$(DELIM)lib_printf.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_phy$(DELIM)src$(DELIM)phy_init.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_rom$(DELIM)patches$(DELIM)esp_rom_regi2c_$(CHIP_SERIES).c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_rom$(DELIM)patches$(DELIM)esp_rom_systimer.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_rom$(DELIM)patches$(DELIM)esp_rom_wdt.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_system$(DELIM)port$(DELIM)brownout.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_system$(DELIM)port$(DELIM)soc$(DELIM)$(CHIP_SERIES)$(DELIM)clk.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_system$(DELIM)port$(DELIM)soc$(DELIM)$(CHIP_SERIES)$(DELIM)system_internal.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)hal$(DELIM)brownout_hal.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)hal$(DELIM)efuse_hal.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)hal$(DELIM)gpio_hal.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)hal$(DELIM)ledc_hal.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)hal$(DELIM)ledc_hal_iram.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)hal$(DELIM)lp_timer_hal.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)hal$(DELIM)rmt_hal.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)hal$(DELIM)timer_hal.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)hal$(DELIM)timer_hal_iram.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)hal$(DELIM)twai_hal_iram.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)hal$(DELIM)twai_hal.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)hal$(DELIM)cache_hal.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)hal$(DELIM)mpu_hal.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)hal$(DELIM)mmu_hal.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)hal$(DELIM)uart_hal.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)hal$(DELIM)uart_hal_iram.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)hal$(DELIM)wdt_hal_iram.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)hal$(DELIM)$(CHIP_SERIES)$(DELIM)clk_tree_hal.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)hal$(DELIM)$(CHIP_SERIES)$(DELIM)efuse_hal.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)hal$(DELIM)$(CHIP_SERIES)$(DELIM)modem_clock_hal.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)log$(DELIM)log.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)log$(DELIM)log_noos.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)riscv$(DELIM)interrupt.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)soc$(DELIM)$(CHIP_SERIES)$(DELIM)gpio_periph.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)soc$(DELIM)$(CHIP_SERIES)$(DELIM)ledc_periph.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)soc$(DELIM)$(CHIP_SERIES)$(DELIM)rmt_periph.c

ifeq ($(CONFIG_ESPRESSIF_SIMPLE_BOOT),y)
  CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)nuttx$(DELIM)src$(DELIM)bootloader_banner_wrap.c
  CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)bootloader_support$(DELIM)src$(DELIM)bootloader_console.c
  CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)bootloader_support$(DELIM)src$(DELIM)bootloader_console_loader.c
  CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)bootloader_support$(DELIM)src$(DELIM)${CHIP_SERIES}$(DELIM)bootloader_${CHIP_SERIES}.c
  CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)bootloader_support$(DELIM)src$(DELIM)bootloader_init.c
  CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)bootloader_support$(DELIM)src$(DELIM)bootloader_common.c
  CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)bootloader_support$(DELIM)src$(DELIM)bootloader_common_loader.c
  CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)bootloader_support$(DELIM)bootloader_flash$(DELIM)src$(DELIM)bootloader_flash.c
  CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)bootloader_support$(DELIM)bootloader_flash$(DELIM)src$(DELIM)bootloader_flash_config_${CHIP_SERIES}.c
  CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)bootloader_support$(DELIM)src$(DELIM)bootloader_clock_init.c
  CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)bootloader_support$(DELIM)src$(DELIM)bootloader_clock_loader.c
  CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)bootloader_support$(DELIM)src$(DELIM)bootloader_efuse.c
  CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)bootloader_support$(DELIM)src$(DELIM)bootloader_mem.c
  CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)bootloader_support$(DELIM)src$(DELIM)bootloader_random.c
  CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)bootloader_support$(DELIM)src$(DELIM)bootloader_random_${CHIP_SERIES}.c
  CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)bootloader_support$(DELIM)src$(DELIM)esp_image_format.c
  CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)bootloader_support$(DELIM)src$(DELIM)${CHIP_SERIES}$(DELIM)bootloader_soc.c
  CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)bootloader_support$(DELIM)src$(DELIM)${CHIP_SERIES}$(DELIM)bootloader_sha.c
  CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)bootloader_support$(DELIM)src$(DELIM)flash_encrypt.c
  CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)soc$(DELIM)${CHIP_SERIES}$(DELIM)uart_periph.c
  CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_rom$(DELIM)patches$(DELIM)esp_rom_uart.c
  CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_rom$(DELIM)patches$(DELIM)esp_rom_sys.c
  CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_rom$(DELIM)patches$(DELIM)esp_rom_spiflash.c
  CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)efuse$(DELIM)src$(DELIM)esp_efuse_fields.c
  CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)efuse$(DELIM)src$(DELIM)efuse_controller$(DELIM)keys$(DELIM)with_key_purposes$(DELIM)esp_efuse_api_key.c

  LDFLAGS += --wrap=bootloader_print_banner
endif
