/****************************************************************************
 * boards/risc-v/esp32p4/common/scripts/esp32p4_protected_memory.ld
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
 * ESP32-P4 memory layout for the protected build.
 *
 * kernel-space.ld and user-space.ld link the kernel and user images into
 * these blocks respectively.
 *
 * Three properties of this SoC shape the layout:
 *
 * 1. Instruction and data share one virtual window for external flash
 *    (SOC_MMU_DI_VADDR_SHARED).  Unlike parts with separate IROM and DROM
 *    windows, all four flash regions are carved out of 0x40000000 upwards
 *    and are distinguished only by address range.
 *
 * 2. On silicon older than v3.0 the internal SRAM is two banks with a
 *    hardware gap.  That maps neatly onto the kernel/user split: the kernel
 *    takes sram_low, user data takes sram_high, and the boundary needs no
 *    artificial alignment because the banks are already far apart.
 *
 * 3. PMP granularity is 128 bytes, so every region boundary that the PMP
 *    describes must be 128-byte aligned.  This is why the metadata block
 *    below is 0x80 rather than the 0x18 used by parts with 4-byte
 *    granularity: UDROM starts immediately after it, and that start address
 *    becomes a PMP TOR boundary.
 *
 * Flash offsets must also satisfy the cache MMU constraint
 * paddr % 64KB == vaddr % 64KB.  All four flash regions are placed on 64 KB
 * boundaries, with the metadata offset applied on top for UDROM.
 ****************************************************************************/

#include <nuttx/config.h>

/* common.ld would supply RESERVE_RTC_MEM and MSPI_WORKAROUND_SIZE, but it
 * also pulls in the HAL's sdkconfig.h, which is not on the include path when
 * the user image is linked in pass 1.  This file is shared by both links, so
 * the two constants are reproduced here instead.  Values are the ESP32-P4
 * ones: 24 bytes of RTC timer retention, and no MSPI workaround because that
 * applies to revision v3 silicon only.
 */

#ifndef RESERVE_RTC_MEM
#  define RESERVE_RTC_MEM       24
#endif

#ifndef MSPI_WORKAROUND_SIZE
#  define MSPI_WORKAROUND_SIZE  0x0
#endif

/* Internal SRAM.  Bank boundaries come from the flat layout so that the two
 * stay in step; see esp32p4_flat_memory.ld.
 */

#define SRAM_LOW_START      0x4ff00000
#define SRAM_LOW_END        0x4ff2cbd0  /* 2nd stage bootloader iram_loader_seg */
#define SRAM_LOW_SIZE       (SRAM_LOW_END - SRAM_LOW_START)

#define SRAM_HIGH_START     0x4ff40000
/* CONFIG_CACHE_L2_CACHE_SIZE is a HAL sdkconfig alias that is not visible
 * here; the NuttX-side symbol it is derived from is used directly instead.
 */

#define SRAM_HIGH_SIZE      (0x80000 - CONFIG_ESPRESSIF_CACHE_L2_CACHE_SIZE)

/* Flash, mapped into the shared instruction/data window.  The 0x20 offset on
 * KIROM is the usual Espressif convenience: the image has a 0x18 byte file
 * header and each segment an 0x08 byte header, so starting 0x20 in keeps
 * paddr % 64KB == vaddr % 64KB satisfied for the first segment.
 */

#define IDROM_SEG_SIZE      (CONFIG_MMU_PAGE_SIZE << 10)

#define FLASH_WINDOW        0x40000000

#define KIROM_ORG           (FLASH_WINDOW + 0x20)
#define KIROM_LEN           (0x180000 - 0x20)

#define KDROM_ORG           (FLASH_WINDOW + 0x180000)
#define KDROM_LEN           0x080000

#define UIROM_ORG           (FLASH_WINDOW + 0x200000)
#define UIROM_LEN           0x100000

/* The user image begins with a metadata block that the kernel reads to learn
 * where this image expects to be mapped.  UDROM starts just past it, and
 * that address is a PMP boundary, hence the 128-byte size.
 */

#define UDROM_METADATA_LEN  0x80
#define UDROM_ORG           (FLASH_WINDOW + 0x300000 + UDROM_METADATA_LEN)
#define UDROM_LEN           (0x080000 - UDROM_METADATA_LEN)

MEMORY
{
  /* Load-only block carrying the user image metadata.  It is emitted at the
   * start of the user image file, not mapped at run time.
   */

  metadata (RX) :   org = 0x0, len = UDROM_METADATA_LEN
  ROM (RX) :        org = ORIGIN(metadata) + LENGTH(metadata),
                    len = 0x400000 - ORIGIN(ROM)

  /* TCM, kernel only */

  tcm_idram_seg (RX) : org = 0x30100000, len = 0x2000

  /* Kernel internal SRAM: code, data, bss and the kernel heap.  This is one
   * region rather than separate IRAM and DRAM blocks because SRAM is
   * unified on this SoC -- there is no second view of it at another
   * address, as there is on the ESP32-C3.
   */

  KRAM (RWX) :      org = SRAM_LOW_START, len = SRAM_LOW_SIZE

  /* User internal SRAM: data, bss and the user heap.  User code executes in
   * place from flash, so there is no UIRAM region.
   */

  UDRAM (RW) :      org = SRAM_HIGH_START, len = SRAM_HIGH_SIZE

  /* Flash-mapped kernel code and read-only data */

  KIROM (RX) :      org = KIROM_ORG, len = KIROM_LEN
  KDROM (R) :       org = KDROM_ORG, len = KDROM_LEN

  /* Flash-mapped user code and read-only data */

  UIROM (RX) :      org = UIROM_ORG, len = UIROM_LEN
  UDROM (R) :       org = UDROM_ORG, len = UDROM_LEN

  /* PSRAM */

  extern_ram_seg (RWX) : org = 0x48000000, len = IDROM_SEG_SIZE

  /* LP/RTC memory, kernel only */

  rev3_mspi_workaround_seg (RWX) : org = 0x50108000, len = MSPI_WORKAROUND_SIZE
  lp_reserved_seg (RW) : org = 0x50108000 + MSPI_WORKAROUND_SIZE,
                    len = RESERVE_RTC_MEM
  lp_ram_seg (RW) : org = 0x50108000 + MSPI_WORKAROUND_SIZE + RESERVE_RTC_MEM,
                    len = 0x8000 - RESERVE_RTC_MEM - MSPI_WORKAROUND_SIZE
}

/* The flat layout's alias names are reused so that the kernel sections can
 * be taken from esp32p4_sections.ld unchanged.
 *
 * Both sram_low and sram_high alias the single kernel RAM region: the flat
 * script spills some sections into sram_high, and with both names pointing
 * at KRAM those sections simply flow on sequentially instead of landing in
 * memory that now belongs to user space.
 */

REGION_ALIAS("rtc_iram_seg", lp_ram_seg);
REGION_ALIAS("rtc_data_seg", rtc_iram_seg);
REGION_ALIAS("rtc_slow_seg", rtc_iram_seg);
REGION_ALIAS("rtc_data_location", rtc_iram_seg);
REGION_ALIAS("rtc_reserved_seg", lp_reserved_seg);
REGION_ALIAS("ext_ram_seg", extern_ram_seg);

REGION_ALIAS("sram_low", KRAM);
REGION_ALIAS("sram_high", KRAM);
REGION_ALIAS("text_seg_low", KIROM);
REGION_ALIAS("rodata_seg_low", KDROM);
