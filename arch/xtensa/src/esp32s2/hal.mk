############################################################################
# arch/xtensa/src/esp32s2/hal.mk
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

INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)bootloader_support$(DELIM)$(DELIM)include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)efuse$(DELIM)$(CHIP_SERIES)$(DELIM)include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)efuse$(DELIM)$(CHIP_SERIES)$(DELIM)private_include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)efuse$(DELIM)include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)efuse$(DELIM)private_include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_common$(DELIM)include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_hw_support$(DELIM)include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_hw_support$(DELIM)include$(DELIM)esp_private
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_hw_support$(DELIM)include$(DELIM)soc
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_hw_support$(DELIM)port$(DELIM)$(CHIP_SERIES)
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_hw_support$(DELIM)port$(DELIM)$(CHIP_SERIES)$(DELIM)private_include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_hw_support$(DELIM)port$(DELIM)include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_rom$(DELIM)$(CHIP_SERIES)
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_rom$(DELIM)include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_rom$(DELIM)include$(DELIM)$(CHIP_SERIES)
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_system$(DELIM)include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_system$(DELIM)port$(DELIM)include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_system$(DELIM)port$(DELIM)include$(DELIM)private
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_system$(DELIM)port$(DELIM)public_compat
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_timer$(DELIM)include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)hal$(DELIM)$(CHIP_SERIES)$(DELIM)include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)hal$(DELIM)include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)hal$(DELIM)platform_port$(DELIM)include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)log
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)log$(DELIM)include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)soc$(DELIM)$(CHIP_SERIES)$(DELIM)include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)soc$(DELIM)include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)xtensa$(DELIM)$(CHIP_SERIES)$(DELIM)include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)xtensa$(DELIM)include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)nuttx$(DELIM)$(CHIP_SERIES)$(DELIM)include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)nuttx$(DELIM)include

# Linker scripts

ARCHSCRIPT += $(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_rom$(DELIM)$(CHIP_SERIES)$(DELIM)ld$(DELIM)$(CHIP_SERIES).rom.api.ld
ARCHSCRIPT += $(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_rom$(DELIM)$(CHIP_SERIES)$(DELIM)ld$(DELIM)$(CHIP_SERIES).rom.ld
ARCHSCRIPT += $(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_rom$(DELIM)$(CHIP_SERIES)$(DELIM)ld$(DELIM)$(CHIP_SERIES).rom.spiflash.ld
ARCHSCRIPT += $(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)soc$(DELIM)$(CHIP_SERIES)$(DELIM)ld$(DELIM)$(CHIP_SERIES).peripherals.ld

# Source files

CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_hw_support$(DELIM)clk_ctrl_os.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_hw_support$(DELIM)cpu.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_hw_support$(DELIM)esp_clk.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_hw_support$(DELIM)hw_random.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_hw_support$(DELIM)periph_ctrl.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_hw_support$(DELIM)port$(DELIM)$(CHIP_SERIES)$(DELIM)cpu_region_protect.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_hw_support$(DELIM)port$(DELIM)$(CHIP_SERIES)$(DELIM)esp_clk_tree.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_hw_support$(DELIM)port$(DELIM)$(CHIP_SERIES)$(DELIM)rtc_clk.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_hw_support$(DELIM)port$(DELIM)$(CHIP_SERIES)$(DELIM)rtc_time.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_hw_support$(DELIM)port$(DELIM)esp_clk_tree_common.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_hw_support$(DELIM)regi2c_ctrl.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_rom$(DELIM)patches$(DELIM)esp_rom_regi2c_$(CHIP_SERIES).c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_rom$(DELIM)patches$(DELIM)esp_rom_wdt.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_system$(DELIM)port$(DELIM)soc$(DELIM)$(CHIP_SERIES)$(DELIM)clk.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_system$(DELIM)port$(DELIM)soc$(DELIM)$(CHIP_SERIES)$(DELIM)system_internal.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)hal$(DELIM)$(CHIP_SERIES)$(DELIM)clk_tree_hal.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)hal$(DELIM)$(CHIP_SERIES)$(DELIM)efuse_hal.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)hal$(DELIM)brownout_hal.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)hal$(DELIM)efuse_hal.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)hal$(DELIM)gpio_hal.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)hal$(DELIM)ledc_hal_iram.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)hal$(DELIM)ledc_hal.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)hal$(DELIM)rmt_hal.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)hal$(DELIM)timer_hal_iram.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)hal$(DELIM)timer_hal.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)hal$(DELIM)uart_hal_iram.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)hal$(DELIM)uart_hal.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)log$(DELIM)log_noos.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)log$(DELIM)log.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)soc$(DELIM)$(CHIP_SERIES)$(DELIM)gpio_periph.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)soc$(DELIM)$(CHIP_SERIES)$(DELIM)ledc_periph.c
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)soc$(DELIM)$(CHIP_SERIES)$(DELIM)rmt_periph.c

CFLAGS += ${DEFINE_PREFIX}ESP_PLATFORM=1
