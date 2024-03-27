/****************************************************************************
 * boards/risc-v/esp32c6/common/scripts/esp32c6_flat_memory.ld
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
 * ESP32-C6 Linker Script Memory Layout
 *
 * This file describes the memory layout (memory blocks) as virtual
 * memory addresses.
 *
 * esp32c6_<legacy/mcuboot>_sections.ld contains output sections to link compiler
 * output into these memory blocks.
 *
 ****************************************************************************/

#include "common.ld"

#define SRAM_IRAM_START       0x40800000
#define SRAM_DRAM_START       0x40800000

/* 2nd stage bootloader iram_loader_seg start address */

#define SRAM_DRAM_END         (0x4086e610)

#define SRAM_IRAM_ORG         (SRAM_IRAM_START)
#define SRAM_DRAM_ORG         (SRAM_DRAM_START)

#define I_D_SRAM_SIZE         (SRAM_DRAM_END - SRAM_DRAM_ORG)

/* IDRAM0_2_SEG_SIZE_DEFAULT is used when page size is 64KB */

#define CONFIG_MMU_PAGE_SIZE  0x10000
#define IDRAM0_2_SEG_SIZE     (CONFIG_MMU_PAGE_SIZE << 8)

#define DRAM0_0_SEG_LEN       I_D_SRAM_SIZE

MEMORY
{
#ifdef CONFIG_ESPRESSIF_SIMPLE_BOOT
  ROM (R) :              org = ORIGIN(ROM),
                         len = IDRAM0_2_SEG_SIZE
#endif

  /* Below values assume the flash cache is on, and have the blocks this
   * uses subtracted from the length of the various regions. The 'data access
   * port' dram/drom regions map to the same iram/irom regions but are
   * connected to the data port of the CPU and e.g. allow bytewise access.
   */

  iram0_0_seg (RX) :      org = SRAM_IRAM_ORG, len = I_D_SRAM_SIZE

  /* Flash mapped instruction data.
   *
   * The 0x20 offset is a convenience for the app binary image generation.
   * Flash cache has 64KB pages. The .bin file which is flashed to the chip
   * has a 0x18 byte file header, and each segment has a 0x08 byte segment
   * header. Setting this offset makes it simple to meet the flash cache MMU's
   * constraint that (paddr % 64KB == vaddr % 64KB).
   */

  irom_seg    (RX) :      org = 0x42000000, len = IDRAM0_2_SEG_SIZE

  /* Shared data RAM, excluding memory reserved for ROM bss/data/stack.
   * Enabling Bluetooth & Trace Memory features in menuconfig will decrease
   * the amount of RAM available.
   */

  dram0_0_seg (RW) :      org = SRAM_DRAM_ORG, len = DRAM0_0_SEG_LEN

  /* Flash mapped constant data.
   *
   * The 0x20 offset is a convenience for the app binary image generation.
   * Flash cache has 64KB pages. The .bin file which is flashed to the chip
   * has a 0x18 byte file header, and each segment has a 0x08 byte segment
   * header. Setting this offset makes it simple to meet the flash cache MMU's
   * constraint that (paddr % 64KB == vaddr % 64KB).
   */

  drom_seg    (R) :       org = 0x42000000, len = IDRAM0_2_SEG_SIZE

  /* RTC fast memory (executable). Persists over deep sleep. */

  lp_ram_seg  (RWX) :     org = 0x50000000, len = 0x4000 - RESERVE_RTC_MEM

  /* We reduced the size of lp_ram_seg by RESERVE_RTC_MEM value.
   * It reserves the amount of LP memory that we use for this memory segment.
   * This segment is intended for keeping:
   *   - (lower addr) rtc timer data (s_rtc_timer_retain_mem, see esp_clk.c files).
   *   - (higher addr) bootloader rtc data (s_bootloader_retain_mem, when a Kconfig option is on).
   * The aim of this is to keep data that will not be moved around and have a fixed address.
   */

  lp_reserved_seg (RW):   org = 0x50000000 + 0x4000 - RESERVE_RTC_MEM, len = RESERVE_RTC_MEM
}

/* The lines below define location alias for .rtc.data section
 * ESP32-C6 has no distinguished LP(RTC) fast and slow memory sections,
 * instead, there is a unified LP_RAM section.
 * Thus, the following region segments are not configurable like on other targets
 */

REGION_ALIAS("rtc_iram_seg", lp_ram_seg);
REGION_ALIAS("rtc_data_seg", rtc_iram_seg);
REGION_ALIAS("rtc_slow_seg", rtc_iram_seg);
REGION_ALIAS("rtc_data_location", rtc_iram_seg);
REGION_ALIAS("rtc_reserved_seg", lp_reserved_seg );

#if CONFIG_ESPRESSIF_RUN_IRAM
  REGION_ALIAS("default_rodata_seg", dram0_0_seg);
  REGION_ALIAS("default_code_seg", iram0_0_seg);
#else
  REGION_ALIAS("default_rodata_seg", drom_seg);
  REGION_ALIAS("default_code_seg", irom_seg);
#endif /* CONFIG_ESPRESSIF_RUN_IRAM */
