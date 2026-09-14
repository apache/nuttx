/****************************************************************************
 * arch/arm/src/rtl8730e/rtl8730e_memorymap.c
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
 * Included Files
 ****************************************************************************/

#include <sys/param.h>

#include "mmu.h"

#include "hardware/rtl8730e_memorymap.h"
#include "rtl8730e_memorymap.h"

/****************************************************************************
 * Macro Definitions
 ****************************************************************************/

#define _NSECTIONS(b)                 (((b) + 0x000fffff) >> 20)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct section_mapping_s g_section_mapping[] =
{
  /* SPI NOR flash XIP window (0x08000000-0x0fffffff, 128 MiB).
   * Normal-cacheable / read-only / executable: FLASH_ReadStream uses memcpy
   * from this window.  xlat_flash_region_device() remaps it to MMU_IOFLAGS
   * around erase/program, then xlat_flash_region_xip() restores it.
   */

  {
    VIRT_FLASH_PSECTION, VIRT_FLASH_VSECTION,
    MMU_ROMFLAGS, _NSECTIONS(VIRT_FLASH_SECSIZE)
  },
  /* KM4 HP SRAM NS (0x20000000, 1 MiB).  KM4 stores flash_init_para here;
   * CA32 copies it at startup via rtl8730e_flash_init_para().
   */

  {
    VIRT_KM4_SRAM_PSECTION, VIRT_KM4_SRAM_PSECTION,
    MMU_IOFLAGS, _NSECTIONS(VIRT_KM4_SRAM_SECSIZE)
  },
  {
    VIRT_KM0_PSECTION, VIRT_KM0_VSECTION,
    MMU_IOFLAGS, _NSECTIONS(VIRT_KM0_SECSIZE)
  },
  {
    VIRT_IO_PSECTION, VIRT_IO_VSECTION,
    MMU_IOFLAGS, _NSECTIONS(VIRT_IO_SECSIZE)
  },
  {
    VIRT_SEC_IO_PSECTION, VIRT_SEC_IO_VSECTION,
    MMU_IOFLAGS, _NSECTIONS(VIRT_SEC_IO_SECSIZE)
  },
  {
    VIRT_DDR_PSECTION, VIRT_DDR_VSECTION,
    MMU_MEMFLAGS, _NSECTIONS(VIRT_DDR_SECSIZE)
  },
  {
    VIRT_GIC_PSECTION, VIRT_GIC_VSECTION,
    MMU_IOFLAGS, _NSECTIONS(VIRT_GIC_SECSIZE)
  },
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rtl8730e_setupmappings
 *
 * Description:
 *   Initializes the non-code area page table
 *
 ****************************************************************************/

int rtl8730e_setupmappings(void)
{
  mmu_l1_map_regions(g_section_mapping, nitems(g_section_mapping));

  return 0;
}
