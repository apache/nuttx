/****************************************************************************
 * boards/risc-v/esp32c3/esp32c3-devkit/scripts/protected.template.ld
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
 * This file describes the memory layout (memory blocks) as virtual
 * memory addresses.
 *
 * kernel-space.ld and user-space.ld contain output sections to link compiler
 * output into these memory blocks for the Kernel and User images,
 * respectively.
 *
 * NOTE: That this is not the actual linker script but rather a "template"
 * for the esp32c3_out.ld script. This template script is passed through
 * the C preprocessor to include selected configuration options.
 *
 ****************************************************************************/

#include <nuttx/config.h>

#include "esp32c3_aliases.ld"

#define SRAM_IRAM_START     0x4037c000
#define SRAM_DRAM_START     0x3fc7c000

/* ICache size is fixed to 16KB on ESP32-C3 */

#define ICACHE_SIZE         0x4000
#define I_D_SRAM_OFFSET     (SRAM_IRAM_START - SRAM_DRAM_START)

/* 2nd stage bootloader iram_loader_seg start address */

#define SRAM_IRAM_END       0x403d0000
#define SRAM_DRAM_END       SRAM_IRAM_END - I_D_SRAM_OFFSET

#define SRAM_IRAM_ORG       (SRAM_IRAM_START + ICACHE_SIZE)
#define SRAM_DRAM_ORG       (SRAM_DRAM_START + ICACHE_SIZE)

#define I_D_SRAM_SIZE       SRAM_DRAM_END - SRAM_DRAM_ORG

MEMORY
{
  metadata (RX) :   org = 0x0, len = 0x18
  ROM (RX) :        org = 0x18, len = 0x100000

  /* Below values assume the flash cache is on, and have the blocks this
   * uses subtracted from the length of the various regions. The 'data access
   * port' dram/drom regions map to the same iram/irom regions but are
   * connected to the data port of the CPU and e.g. allow bytewise access.
   */

  KIRAM (RWX) :     org = SRAM_IRAM_ORG, len = 64K
  UIRAM (RWX) :     org = SRAM_IRAM_ORG + 64K, len = 256K

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

  KDRAM (RW) :      org = SRAM_DRAM_ORG, len = 64K
  UDRAM (RW) :      org = SRAM_DRAM_ORG + 64K, len = 256K

  /* Flash mapped constant data */

  /* See KIROM region documentation above for the meaning of the 0x20 offset.
   *
   * The 0x18 offset for the UDROM region is a convenience for the User
   * binary image generation following a custom image format, which defines
   * a "metadata" output section containing some information that the Kernel
   * needs for properly configuring the External Flash MMU when loading the
   * User application image.
   */

  KDROM (R)  :      org = 0x3c000020, len = 0x80000 - 0x20
  UDROM (R)  :      org = 0x3c080018, len = 0x180000 - 0x18
}
