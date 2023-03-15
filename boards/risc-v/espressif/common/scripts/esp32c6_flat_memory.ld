/****************************************************************************
 * boards/risc-v/espressif/common/scripts/esp32c6_flat_memory.ld
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

#include <nuttx/config.h>

#define SRAM_IRAM_START       0x40800000
#define SRAM_DRAM_START       0x40800000

#define I_D_SRAM_OFFSET       (SRAM_IRAM_START - SRAM_DRAM_START)

/* 2nd stage bootloader iram_loader_seg start address */

#define SRAM_DRAM_END         (0x4086e610 - I_D_SRAM_OFFSET)

#define SRAM_IRAM_ORG         (SRAM_IRAM_START)
#define SRAM_DRAM_ORG         (SRAM_DRAM_START)

#define I_D_SRAM_SIZE         (SRAM_DRAM_END - SRAM_DRAM_ORG)

/* IDRAM0_2_SEG_SIZE_DEFAULT is used when page size is 64KB */

#define CONFIG_MMU_PAGE_SIZE  0x10000
#define IDRAM0_2_SEG_SIZE     (CONFIG_MMU_PAGE_SIZE << 8)

#define DRAM0_0_SEG_LEN       I_D_SRAM_SIZE

MEMORY
{
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

  irom_seg    (RX) :      org = 0x42000020, len = IDRAM0_2_SEG_SIZE - 0x20

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

  drom_seg    (R) :       org = 0x42000020, len = IDRAM0_2_SEG_SIZE - 0x20

  /* RTC fast memory (executable). Persists over deep sleep. */

  lp_ram_seg  (RWX) :     org = 0x50000000, len = 0x4000
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

#if CONFIG_ESPRESSIF_RUN_IRAM
  REGION_ALIAS("default_rodata_seg", dram0_0_seg);
  REGION_ALIAS("default_code_seg", iram0_0_seg);
#else
  REGION_ALIAS("default_rodata_seg", drom_seg);
  REGION_ALIAS("default_code_seg", irom_seg);
#endif /* CONFIG_ESPRESSIF_RUN_IRAM */
