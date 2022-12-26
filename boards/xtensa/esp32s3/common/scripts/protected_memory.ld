/****************************************************************************
 * boards/xtensa/esp32s3/common/scripts/protected_memory.ld
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
 * ESP32-S3 Linker Script Memory Layout for Protected Mode
 *
 * This file describes the memory layout (memory blocks) as virtual
 * memory addresses.
 *
 * kernel-space.ld and user-space.ld contain output sections to link compiler
 * output into these memory blocks for the Kernel and User images,
 * respectively.
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
  metadata (RX) :   org = 0x0, len = 0x30
  ROM (RX) :        org = ORIGIN(metadata) + LENGTH(metadata),
                    len = 0x100000 - ORIGIN(ROM)

  /* Instruction RAM */

  UIRAM (RWX) :     org = SRAM_IRAM_ORG, len = 16K
  KIRAM (RWX) :     org = ORIGIN(UIRAM) + LENGTH(UIRAM), len = 32K

  /* Flash mapped instruction data. */

  /* The 0x20 offset for the KIROM region is a convenience for the Kernel
   * binary image generation in Espressif Application Image format.
   * Flash cache has 64KB pages. The .bin file which is flashed to the chip
   * has a 0x18 byte file header, and each segment has a 0x08 byte segment
   * header. Setting this offset makes it simple to meet the flash cache MMU's
   * constraint that (paddr % 64KB == vaddr % 64KB).
   */

  KIROM (RX) :      org = 0x42000020, len = 0x80000 - 0x20
  UIROM (RX) :      org = 0x42080000, len = 0x180000

  /* Shared data RAM, excluding memory reserved for ROM bss/data/stack. */

  KDRAM (RW) :      org = ORIGIN(KIRAM) + LENGTH(KIRAM) - I_D_SRAM_OFFSET, len = 64K
  UDRAM (RW) :      org = ORIGIN(KDRAM) + LENGTH(KDRAM), len = 256K

  /* Flash mapped constant data */

  /* See KIROM region documentation above for the meaning of the 0x20 offset.
   *
   * The 0x30 offset for the UDROM region is a convenience for the User
   * binary image generation following a custom image format, which defines
   * a "metadata" output section containing some information that the Kernel
   * needs for properly configuring the External Flash MMU and initializing
   * SRAM contents when loading the User application image.
   */

  KDROM (R)  :      org = 0x3c000020, len = 0x80000 - 0x20
  UDROM (R)  :      org = 0x3c080030, len = 0x180000 - ORIGIN(ROM)
}

