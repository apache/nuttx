/****************************************************************************
 * boards/xtensa/esp32s3/common/scripts/flat_memory.ld
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
 * ESP32-S3 Linker Script Memory Layout
 *
 * This file describes the memory layout (memory blocks) as virtual
 * memory addresses.
 *
 * <legacy/mcuboot>_sections.ld contains output sections to link compiler
 * output into these memory blocks.
 *
 ****************************************************************************/

#include <nuttx/config.h>

#define SRAM_IRAM_START     0x40370000
#define SRAM_DIRAM_I_START  0x40378000

/* The memory region starting from SRAM_IRAM_END up to 0x403dffff is
 * reserved to the 2nd stage bootloader for actually loading the NuttX
 * Application Image code and data into IRAM and DRAM. Otherwise the
 * Bootloader could end up overwriting itself and failing to load the NuttX
 * Application Image properly.
 *
 * For more information, refer to the bootloader linker scripts:
 * https://github.com/espressif/esp-idf/blob/v4.4.2/components/bootloader/subproject/main/ld/esp32s3/bootloader.ld#L41-L47
 */

#define SRAM_IRAM_END       0x403cc700
#define I_D_SRAM_OFFSET     (SRAM_DIRAM_I_START - SRAM_DRAM_START)

#define SRAM_DRAM_START     0x3fc88000

#define SRAM_DRAM_END       (SRAM_IRAM_END - I_D_SRAM_OFFSET) /* 2nd stage bootloader iram_loader_seg start address */
#define I_D_SRAM_SIZE       (SRAM_DRAM_END - SRAM_DRAM_START)

#define ICACHE_SIZE         0x8000

#define SRAM_IRAM_ORG       (SRAM_IRAM_START + CONFIG_ESP32S3_INSTRUCTION_CACHE_SIZE)
#define SRAM_IRAM_SIZE      (I_D_SRAM_SIZE + ICACHE_SIZE - CONFIG_ESP32S3_INSTRUCTION_CACHE_SIZE)

#define DCACHE_SIZE         0x10000
#define SRAM_DRAM_ORG       (SRAM_DRAM_START)

#ifdef CONFIG_ESP32S3_FLASH_4M
#  define FLASH_SIZE        0x400000
#elif defined (CONFIG_ESP32S3_FLASH_8M)
#  define FLASH_SIZE        0x800000
#elif defined (CONFIG_ESP32S3_FLASH_16M)
#  define FLASH_SIZE        0x1000000
#endif

MEMORY
{
#ifdef CONFIG_ESP32S3_APP_FORMAT_MCUBOOT
  /* The origin values for "metadata" and "ROM" memory regions are the actual
   * load addresses.
   *
   * NOTE: The memory region starting from 0x0 with length represented by
   * CONFIG_ESP32S3_APP_MCUBOOT_HEADER_SIZE is reserved for the MCUboot header,
   * which will be prepended to the binary file by the "imgtool" during the
   * signing of firmware image.
   */

  metadata (RX) :        org = CONFIG_ESP32S3_APP_MCUBOOT_HEADER_SIZE, len = 0x20
  ROM (RX) :             org = ORIGIN(metadata) + LENGTH(metadata),
                         len = FLASH_SIZE - ORIGIN(ROM)
#endif

  /* Below values assume the flash cache is on, and have the blocks this
   * uses subtracted from the length of the various regions. The 'data access
   * port' dram/drom regions map to the same iram/irom regions but are
   * connected to the data port of the CPU and eg allow bytewise access.
   */

  /* IRAM for CPU */

  iram0_0_seg (RX) :                 org = SRAM_IRAM_ORG, len = SRAM_IRAM_SIZE

  /* Flash mapped instruction data. */

#ifdef CONFIG_ESP32S3_APP_FORMAT_MCUBOOT
  irom0_0_seg (RX) :                 org = 0x42000000, len = FLASH_SIZE
#else
  /* The 0x20 offset is a convenience for the app binary image generation.
   * Flash cache has 64KB pages. The .bin file which is flashed to the chip
   * has a 0x18 byte file header, and each segment has a 0x08 byte segment
   * header. Setting this offset makes it simple to meet the flash cache MMU's
   * constraint that (paddr % 64KB == vaddr % 64KB).
   */

  irom0_0_seg (RX) :                 org = 0x42000020, len = FLASH_SIZE - 0x20
#endif

  /* Shared data RAM, excluding memory reserved for bootloader and ROM
   * bss/data/stack.
   */

  dram0_0_seg (RW) :                 org = SRAM_DRAM_ORG, len = I_D_SRAM_SIZE

  /* Flash mapped constant data */

#ifdef CONFIG_ESP32S3_APP_FORMAT_MCUBOOT
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

  drom0_0_seg (R) :                  org = 0x3c000020, len = FLASH_SIZE - 0x20
#endif

  /* RTC fast memory (executable). Persists over deep sleep. */

  rtc_iram_seg(RWX) :                org = 0x600fe000, len = 0x2000

  /* RTC fast memory (same block as above), viewed from data bus */

  rtc_data_seg(RW)  :                org = 0x600fe000, len = 0x2000

  /* RTC slow memory (data accessible). Persists over deep sleep.
   * Start of RTC slow memory is reserved for ULP co-processor code + data,
   * if enabled.
   */

  rtc_slow_seg(RW)  :    org = 0x50000000 + CONFIG_ESP32S3_ULP_COPROC_RESERVE_MEM,
                         len = 0x2000 - CONFIG_ESP32S3_ULP_COPROC_RESERVE_MEM
}

#ifdef CONFIG_ESP32S3_RUN_IRAM
  REGION_ALIAS("default_rodata_seg", dram0_0_seg);
  REGION_ALIAS("default_code_seg", iram0_0_seg);
#else
  REGION_ALIAS("default_rodata_seg", drom0_0_seg);
  REGION_ALIAS("default_code_seg", irom0_0_seg);
#endif /* CONFIG_ESP32S3_RUN_IRAM */

