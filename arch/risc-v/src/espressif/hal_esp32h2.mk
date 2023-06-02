############################################################################
# arch/risc-v/src/espressif/esp32h2.mk
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

INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)/chip/$(ESP_HAL_3RDPARTY_REPO)/nuttx/include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)/chip/$(ESP_HAL_3RDPARTY_REPO)/nuttx/$(CHIP_SERIES)/include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)/chip/$(ESP_HAL_3RDPARTY_REPO)/components/efuse/include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)/chip/$(ESP_HAL_3RDPARTY_REPO)/components/efuse/private_include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)/chip/$(ESP_HAL_3RDPARTY_REPO)/components/efuse/$(CHIP_SERIES)/include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)/chip/$(ESP_HAL_3RDPARTY_REPO)/components/efuse/$(CHIP_SERIES)/private_include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)/chip/$(ESP_HAL_3RDPARTY_REPO)/components/esp_common/include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)/chip/$(ESP_HAL_3RDPARTY_REPO)/components/esp_hw_support/include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)/chip/$(ESP_HAL_3RDPARTY_REPO)/components/esp_hw_support/include/esp_private
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)/chip/$(ESP_HAL_3RDPARTY_REPO)/components/esp_hw_support/include/soc
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)/chip/$(ESP_HAL_3RDPARTY_REPO)/components/esp_hw_support/port/include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)/chip/$(ESP_HAL_3RDPARTY_REPO)/components/esp_hw_support/port/$(CHIP_SERIES)
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)/chip/$(ESP_HAL_3RDPARTY_REPO)/components/esp_hw_support/port/$(CHIP_SERIES)/private_include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)/chip/$(ESP_HAL_3RDPARTY_REPO)/components/esp_rom/include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)/chip/$(ESP_HAL_3RDPARTY_REPO)/components/esp_rom/include/$(CHIP_SERIES)
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)/chip/$(ESP_HAL_3RDPARTY_REPO)/components/esp_rom/$(CHIP_SERIES)
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)/chip/$(ESP_HAL_3RDPARTY_REPO)/components/esp_system/include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)/chip/$(ESP_HAL_3RDPARTY_REPO)/components/esp_system/port/include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)/chip/$(ESP_HAL_3RDPARTY_REPO)/components/esp_system/port/include/private
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)/chip/$(ESP_HAL_3RDPARTY_REPO)/components/esp_system/port/public_compat
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)/chip/$(ESP_HAL_3RDPARTY_REPO)/components/hal/$(CHIP_SERIES)/include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)/chip/$(ESP_HAL_3RDPARTY_REPO)/components/hal/include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)/chip/$(ESP_HAL_3RDPARTY_REPO)/components/hal/platform_port/include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)/chip/$(ESP_HAL_3RDPARTY_REPO)/components/log
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)/chip/$(ESP_HAL_3RDPARTY_REPO)/components/log/include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)/chip/$(ESP_HAL_3RDPARTY_REPO)/components/riscv/include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)/chip/$(ESP_HAL_3RDPARTY_REPO)/components/soc/include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)/chip/$(ESP_HAL_3RDPARTY_REPO)/components/soc/$(CHIP_SERIES)/include

# Linker scripts

ARCHSCRIPT += $(ARCH_SRCDIR)/chip/$(ESP_HAL_3RDPARTY_REPO)/components/esp_rom/$(CHIP_SERIES)/ld/$(CHIP_SERIES).rom.ld
ARCHSCRIPT += $(ARCH_SRCDIR)/chip/$(ESP_HAL_3RDPARTY_REPO)/components/esp_rom/$(CHIP_SERIES)/ld/$(CHIP_SERIES).rom.api.ld
ARCHSCRIPT += $(ARCH_SRCDIR)/chip/$(ESP_HAL_3RDPARTY_REPO)/components/esp_rom/$(CHIP_SERIES)/ld/$(CHIP_SERIES).rom.newlib.ld
ARCHSCRIPT += $(ARCH_SRCDIR)/chip/$(ESP_HAL_3RDPARTY_REPO)/components/esp_rom/$(CHIP_SERIES)/ld/$(CHIP_SERIES).rom.spiflash.ld
ARCHSCRIPT += $(ARCH_SRCDIR)/chip/$(ESP_HAL_3RDPARTY_REPO)/components/esp_rom/$(CHIP_SERIES)/ld/$(CHIP_SERIES).rom.version.ld
ARCHSCRIPT += $(ARCH_SRCDIR)/chip/$(ESP_HAL_3RDPARTY_REPO)/components/esp_rom/$(CHIP_SERIES)/ld/$(CHIP_SERIES).rom.wdt.ld
ARCHSCRIPT += $(ARCH_SRCDIR)/chip/$(ESP_HAL_3RDPARTY_REPO)/components/soc/$(CHIP_SERIES)/ld/$(CHIP_SERIES).peripherals.ld

# Source files

CHIP_CSRCS += chip/$(ESP_HAL_3RDPARTY_REPO)/components/efuse/src/esp_efuse_api.c
CHIP_CSRCS += chip/$(ESP_HAL_3RDPARTY_REPO)/components/efuse/src/esp_efuse_utility.c
CHIP_CSRCS += chip/$(ESP_HAL_3RDPARTY_REPO)/components/efuse/$(CHIP_SERIES)/esp_efuse_fields.c
CHIP_CSRCS += chip/$(ESP_HAL_3RDPARTY_REPO)/components/efuse/$(CHIP_SERIES)/esp_efuse_table.c
CHIP_CSRCS += chip/$(ESP_HAL_3RDPARTY_REPO)/components/efuse/$(CHIP_SERIES)/esp_efuse_utility.c
CHIP_CSRCS += chip/$(ESP_HAL_3RDPARTY_REPO)/components/esp_hw_support/cpu.c
CHIP_CSRCS += chip/$(ESP_HAL_3RDPARTY_REPO)/components/esp_hw_support/esp_clk.c
CHIP_CSRCS += chip/$(ESP_HAL_3RDPARTY_REPO)/components/esp_hw_support/hw_random.c
CHIP_CSRCS += chip/$(ESP_HAL_3RDPARTY_REPO)/components/esp_hw_support/periph_ctrl.c
CHIP_CSRCS += chip/$(ESP_HAL_3RDPARTY_REPO)/components/esp_hw_support/regi2c_ctrl.c
CHIP_CSRCS += chip/$(ESP_HAL_3RDPARTY_REPO)/components/esp_hw_support/port/esp_clk_tree_common.c
CHIP_CSRCS += chip/$(ESP_HAL_3RDPARTY_REPO)/components/esp_hw_support/port/$(CHIP_SERIES)/esp_clk_tree.c
CHIP_CSRCS += chip/$(ESP_HAL_3RDPARTY_REPO)/components/esp_hw_support/port/$(CHIP_SERIES)/cpu_region_protect.c
CHIP_CSRCS += chip/$(ESP_HAL_3RDPARTY_REPO)/components/esp_hw_support/port/$(CHIP_SERIES)/pmu_init.c
CHIP_CSRCS += chip/$(ESP_HAL_3RDPARTY_REPO)/components/esp_hw_support/port/$(CHIP_SERIES)/pmu_param.c
CHIP_CSRCS += chip/$(ESP_HAL_3RDPARTY_REPO)/components/esp_hw_support/port/$(CHIP_SERIES)/rtc_clk.c
CHIP_CSRCS += chip/$(ESP_HAL_3RDPARTY_REPO)/components/esp_hw_support/port/$(CHIP_SERIES)/rtc_time.c
CHIP_CSRCS += chip/$(ESP_HAL_3RDPARTY_REPO)/components/esp_hw_support/port/$(CHIP_SERIES)/sar_periph_ctrl.c
CHIP_CSRCS += chip/$(ESP_HAL_3RDPARTY_REPO)/components/esp_hw_support/port/$(CHIP_SERIES)/systimer.c
CHIP_CSRCS += chip/$(ESP_HAL_3RDPARTY_REPO)/components/esp_rom/patches/esp_rom_regi2c_$(CHIP_SERIES).c
CHIP_CSRCS += chip/$(ESP_HAL_3RDPARTY_REPO)/components/esp_rom/patches/esp_rom_systimer.c
CHIP_CSRCS += chip/$(ESP_HAL_3RDPARTY_REPO)/components/esp_rom/patches/esp_rom_wdt.c
CHIP_CSRCS += chip/$(ESP_HAL_3RDPARTY_REPO)/components/esp_system/port/brownout.c
CHIP_CSRCS += chip/$(ESP_HAL_3RDPARTY_REPO)/components/esp_system/port/soc/$(CHIP_SERIES)/clk.c
CHIP_CSRCS += chip/$(ESP_HAL_3RDPARTY_REPO)/components/esp_system/port/soc/$(CHIP_SERIES)/system_internal.c
CHIP_CSRCS += chip/$(ESP_HAL_3RDPARTY_REPO)/components/hal/brownout_hal.c
CHIP_CSRCS += chip/$(ESP_HAL_3RDPARTY_REPO)/components/hal/cache_hal.c
CHIP_CSRCS += chip/$(ESP_HAL_3RDPARTY_REPO)/components/hal/efuse_hal.c
CHIP_CSRCS += chip/$(ESP_HAL_3RDPARTY_REPO)/components/hal/timer_hal.c
CHIP_CSRCS += chip/$(ESP_HAL_3RDPARTY_REPO)/components/hal/timer_hal_iram.c
CHIP_CSRCS += chip/$(ESP_HAL_3RDPARTY_REPO)/components/hal/uart_hal.c
CHIP_CSRCS += chip/$(ESP_HAL_3RDPARTY_REPO)/components/hal/uart_hal_iram.c
CHIP_CSRCS += chip/$(ESP_HAL_3RDPARTY_REPO)/components/hal/$(CHIP_SERIES)/clk_tree_hal.c
CHIP_CSRCS += chip/$(ESP_HAL_3RDPARTY_REPO)/components/hal/$(CHIP_SERIES)/efuse_hal.c
CHIP_CSRCS += chip/$(ESP_HAL_3RDPARTY_REPO)/components/log/log.c
CHIP_CSRCS += chip/$(ESP_HAL_3RDPARTY_REPO)/components/log/log_noos.c
CHIP_CSRCS += chip/$(ESP_HAL_3RDPARTY_REPO)/components/riscv/interrupt.c
CHIP_CSRCS += chip/$(ESP_HAL_3RDPARTY_REPO)/components/soc/$(CHIP_SERIES)/gpio_periph.c
