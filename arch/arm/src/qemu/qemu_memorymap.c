/****************************************************************************
 * arch/arm/src/qemu/qemu_memorymap.c
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
#include "qemu_memorymap.h"

/****************************************************************************
 * Macro Definitions
 ****************************************************************************/

#define _NSECTIONS(b)                 (((b) + 0x000fffff) >> 20)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct section_mapping_s g_section_mapping[] =
{
  {
    VIRT_FLASH_PSECTION, VIRT_FLASH_VSECTION,
    MMU_MEMFLAGS, _NSECTIONS(VIRT_FLASH_SECSIZE)
  },
  {
    VIRT_IO_PSECTION, VIRT_IO_VSECTION,
    MMU_IOFLAGS, _NSECTIONS(VIRT_IO_SECSIZE)
  },
  {
    VIRT_SEC_MEM_PSECTION, VIRT_SEC_MEM_VSECTION,
    MMU_MEMFLAGS, _NSECTIONS(VIRT_SEC_MEM_SECSIZE)
  },
  {
    VIRT_PCIE_PSECTION, VIRT_PCIE_VSECTION,
    MMU_IOFLAGS, _NSECTIONS(VIRT_PCIE_SECSIZE)
  },
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: qemu_setupmappings
 *
 * Description:
 *   Initializes the non-code area page table
 *
 ****************************************************************************/

int qemu_setupmappings(void)
{
  mmu_l1_map_regions(g_section_mapping, nitems(g_section_mapping));

  return 0;
}
