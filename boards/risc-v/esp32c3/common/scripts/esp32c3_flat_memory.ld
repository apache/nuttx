/****************************************************************************
 * boards/risc-v/esp32c3/common/scripts/esp32c3_flat_memory.ld
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
 * ESP32-C3 Linker Script Memory Layout
 *
 * This file describes the memory layout (memory blocks) as virtual
 * memory addresses.
 *
 * esp32c3_<legacy/mcuboot>_sections.ld contains output sections to link compiler
 * output into these memory blocks.
 *
 ****************************************************************************/

#include "common.ld"

#define SRAM_IRAM_START     0x4037c000
#define SRAM_DRAM_START     0x3fc7c000

/* ICache size is fixed to 16KB on ESP32-C3 */

#define ICACHE_SIZE         0x4000
#define I_D_SRAM_OFFSET     (SRAM_IRAM_START - SRAM_DRAM_START)

/* 2nd stage bootloader iram_loader_seg start address */

#define SRAM_DRAM_END       (0x403ce710 - I_D_SRAM_OFFSET)

#define SRAM_IRAM_ORG       (SRAM_IRAM_START + ICACHE_SIZE)
#define SRAM_DRAM_ORG       (SRAM_DRAM_START + ICACHE_SIZE)

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
  /* The origin values for "metadata" and "ROM" memory regions are the actual
   * load addresses.
   *
   * NOTE: The memory region starting from 0x0 with length represented by
   * CONFIG_ESPRESSIF_APP_MCUBOOT_HEADER_SIZE is reserved for the MCUboot header,
   * which will be prepended to the binary file by the "imgtool" during the
   * signing of firmware image.
   */

  metadata (RX) :        org = CONFIG_ESPRESSIF_APP_MCUBOOT_HEADER_SIZE, len = 0x20
  ROM (RX) :             org = ORIGIN(metadata) + LENGTH(metadata),
                         len = FLASH_SIZE - ORIGIN(ROM)
#elif defined (CONFIG_ESPRESSIF_SIMPLE_BOOT)
  ROM (RX) :             org = ORIGIN(ROM),
                         len = FLASH_SIZE - ORIGIN(ROM)
#endif

  /* Below values assume the flash cache is on, and have the blocks this
   * uses subtracted from the length of the various regions. The 'data access
   * port' dram/drom regions map to the same iram/irom regions but are
   * connected to the data port of the CPU and e.g. allow bytewise access.
   */

  iram0_0_seg (RX) :      org = SRAM_IRAM_ORG, len = I_D_SRAM_SIZE

  /* Flash mapped instruction data. */

#ifdef CONFIG_ESPRESSIF_BOOTLOADER_MCUBOOT
  irom0_0_seg (RX) :      org = 0x42000000, len = FLASH_SIZE
#else
  /* The 0x20 offset is a convenience for the app binary image generation.
   * Flash cache has 64KB pages. The .bin file which is flashed to the chip
   * has a 0x18 byte file header, and each segment has a 0x08 byte segment
   * header. Setting this offset makes it simple to meet the flash cache MMU's
   * constraint that (paddr % 64KB == vaddr % 64KB).
   */

  irom0_0_seg (RX) :      org = 0x42000000, len = FLASH_SIZE
#endif

  /* Shared data RAM, excluding memory reserved for ROM bss/data/stack. */

  dram0_0_seg (RW) :      org = SRAM_DRAM_ORG, len = I_D_SRAM_SIZE

  /* Flash mapped constant data. */

#ifdef CONFIG_ESPRESSIF_BOOTLOADER_MCUBOOT
  /* The DROM segment origin is offset by 0x40 for mirroring the actual ROM
   * image layout:
   *    0x0  - 0x1F : MCUboot header
   *    0x20 - 0x3F : Application image metadata section
   *    0x40 onwards: ROM code and data
   * This is required to meet the following constraint from the external
   * flash MMU:
   *    VMA % 64KB == LMA % 64KB
   * i.e. the lower 16 bits of both the virtual address (address seen by the
   * CPU) and the load address (physical address of the external flash) must
   * be equal.
   */

  drom0_0_seg (R) :      org = 0x3c000000 + ORIGIN(ROM),
                         len = FLASH_SIZE - ORIGIN(ROM)
#else
  /* The 0x20 offset is a convenience for the app binary image generation.
   * Flash cache has 64KB pages. The .bin file which is flashed to the chip
   * has a 0x18 byte file header, and each segment has a 0x08 byte segment
   * header. Setting this offset makes it simple to meet the flash cache MMU's
   * constraint that (paddr % 64KB == vaddr % 64KB).
   */

  drom0_0_seg (R) :      org = 0x3c000000, len = FLASH_SIZE
#endif

  /* RTC fast memory (executable). Persists over deep sleep. */

  rtc_iram_seg(RWX) :    org = 0x50000000, len = 0x2000 - RESERVE_RTC_MEM

  /* We reduced the size of rtc_iram_seg by RESERVE_RTC_MEM value.
   * It reserves the amount of RTC fast memory that we use for this memory segment.
   * This segment is intended for keeping:
   *   - (lower addr) rtc timer data (s_rtc_timer_retain_mem, see esp_clk.c files).
   *   - (higher addr) bootloader rtc data (s_bootloader_retain_mem, when a Kconfig option is on).
   * The aim of this is to keep data that will not be moved around and have a fixed address.
   */
  rtc_reserved_seg(RW) : org = 0x50000000 + 0x2000 - RESERVE_RTC_MEM, len = RESERVE_RTC_MEM
}

/*  The lines below define location alias for .rtc.data section
 *  As C3 only has RTC fast memory, this is not configurable like on other targets
 */

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
