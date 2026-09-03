/****************************************************************************
 * boards/risc-v/esp32c2/scripts/esp32c2_flat_memory.ld
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * ESP32-C2 Linker Script Memory Layout
 *
 * This file describes the memory layout (memory blocks) as virtual
 * memory addresses.
 *
 * esp32c2_<legacy/mcuboot>_sections.ld contains output sections to link
 * compiler output into these memory blocks.
 *
 ****************************************************************************/

#include "common.ld"
#include "esp32c2_aliases.ld"

#define SRAM_IRAM_START     0x4037C000
#define SRAM_DRAM_START     0x3FCA0000

/* ICache size is fixed to 16KB on ESP32-C2 */

#define ICACHE_SIZE         0x4000
#define I_D_SRAM_OFFSET     (SRAM_IRAM_START - SRAM_DRAM_START + ICACHE_SIZE)

/* 2nd stage bootloader iram_loader_seg start address */

#define SRAM_DRAM_END       (0x403AEB70 - I_D_SRAM_OFFSET)

#define SRAM_IRAM_ORG       (SRAM_IRAM_START + ICACHE_SIZE)
#define SRAM_DRAM_ORG       (SRAM_DRAM_START)

#define I_D_SRAM_SIZE       (SRAM_DRAM_END - SRAM_DRAM_ORG)

#ifdef CONFIG_ESPRESSIF_FLASH_2M
#  define FLASH_SIZE        0x200000
#elif defined (CONFIG_ESPRESSIF_FLASH_4M)
#  define FLASH_SIZE        0x400000
#elif defined (CONFIG_ESPRESSIF_FLASH_8M)
#  define FLASH_SIZE        0x800000
#elif defined (CONFIG_ESPRESSIF_FLASH_16M)
#  define FLASH_SIZE        0x1000000
#endif

MEMORY
{
#ifdef CONFIG_ESPRESSIF_BOOTLOADER_MCUBOOT
  metadata (RX) :        org = CONFIG_ESPRESSIF_APP_MCUBOOT_HEADER_SIZE, len = 0x60
  ROM (RX) :             org = ORIGIN(metadata) + LENGTH(metadata),
                         len = FLASH_SIZE - ORIGIN(ROM)
#elif defined (CONFIG_ESPRESSIF_SIMPLE_BOOT)
  ROM (RX) :             org = ORIGIN(ROM),
                         len = FLASH_SIZE - ORIGIN(ROM)
#endif

  iram0_0_seg (RX) :      org = SRAM_IRAM_ORG, len = I_D_SRAM_SIZE

#ifdef CONFIG_ESPRESSIF_BOOTLOADER_MCUBOOT
  irom0_0_seg (RX) :      org = 0x42000000, len = FLASH_SIZE
#else
  irom0_0_seg (RX) :      org = 0x42000000, len = FLASH_SIZE
#endif

  dram0_0_seg (RW) :      org = SRAM_DRAM_ORG, len = I_D_SRAM_SIZE

#ifdef CONFIG_ESPRESSIF_BOOTLOADER_MCUBOOT
  drom0_0_seg (R) :      org = 0x3c000000 + ORIGIN(ROM),
                         len = FLASH_SIZE - ORIGIN(ROM)
#else
  drom0_0_seg (R) :      org = 0x3c000000, len = FLASH_SIZE
#endif

  rtc_iram_seg(RWX) :    org = 0x50000000, len = 0x2000 - RESERVE_RTC_MEM
  rtc_reserved_seg(RW) : org = 0x50000000 + 0x2000 - RESERVE_RTC_MEM, len = RESERVE_RTC_MEM
}

REGION_ALIAS("rtc_data_seg", rtc_iram_seg );
REGION_ALIAS("rtc_slow_seg", rtc_iram_seg );
REGION_ALIAS("rtc_data_location", rtc_iram_seg );

#if CONFIG_ESPRESSIF_RUN_IRAM
  REGION_ALIAS("default_rodata_seg", dram0_0_seg);
  REGION_ALIAS("default_code_seg", iram0_0_seg);
#else
  REGION_ALIAS("default_rodata_seg", drom0_0_seg);
  REGION_ALIAS("default_code_seg", irom0_0_seg);
#endif /* CONFIG_ESPRESSIF_RUN_IRAM */
