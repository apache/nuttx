/****************************************************************************
 * arch/x86_64/src/common/x86_64_mmu.h
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

#ifndef __ARCH_X86_64_SRC_COMMON_X86_64_MMU_H
#define __ARCH_X86_64_SRC_COMMON_X86_64_MMU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/arch.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* 4 level paging, 4K pages */

#define X86_MMU_PT_LEVELS       4
#define X86_MMU_PAGE_SHIFT      12
#define X86_MMU_PAGE_SIZE       (1 << X86_MMU_PAGE_SHIFT)
#define X86_MMU_ENTRIES_PER_PGT (X86_MMU_PAGE_SIZE / 8)

/* Get virtual address shift for a given paging level.
 * NOTE: in this implementation PTL4 has index 0, PT has index 3 !
 */

#define X86_MMU_PADDR_SHIFT     12
#define X86_MMU_VPN_WIDTH       9
#define X86_MMU_VPN_MASK        ((1 << X86_MMU_VPN_WIDTH) - 1)
#define X86_MMU_VADDR_SHIFT(n)  (X86_MMU_PADDR_SHIFT + X86_MMU_VPN_WIDTH * \
                                 (X86_MMU_PT_LEVELS - ((n) + 1)))

/* Get index in a given page table level for a given virtual address */

#define X86_MMU_VADDR_INDEX(vaddr, ptlevel) \
  ((vaddr >> X86_MMU_VADDR_SHIFT(ptlevel)) & X86_MMU_VPN_MASK)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: mmu_cr3_reg
 *
 * Description:
 *   Utility function to build cr3 register value for input parameters
 *
 * Input Parameters:
 *   pgbase - The physical base address of the translation table base
 *   asid - Address space identifier. Not used now.
 *
 ****************************************************************************/

static inline uintptr_t mmu_cr3_reg(uintptr_t pgbase, uint16_t asid)
{
  uintptr_t reg;

  UNUSED(asid);

  reg = pgbase;
  return reg;
}

/****************************************************************************
 * Name: mmu_pte_to_paddr
 *
 * Description:
 *   Extract physical address from PTE
 *
 * Input Parameters:
 *   pte - Page table entry
 *
 * Returned Value:
 *   Physical address from PTE
 *
 ****************************************************************************/

static inline uintptr_t mmu_pte_to_paddr(uintptr_t pte)
{
  uintptr_t paddr = 0;

  if (pte & X86_PAGE_PRESENT)
    {
      paddr = pte;

      /* Get page address - remove flags */

      paddr &= 0x0dfffffffffff000;
    }

  return paddr;
}

/****************************************************************************
 * Name: mmu_ln_setentry
 *
 * Description:
 *   Set a level n translation table entry.
 *
 * Input Parameters:
 *   ptlevel - The translation table level, amount of levels is
 *     MMU implementation specific
 *   lnvaddr - The virtual address of the beginning of the page table at
 *     level n
 *   paddr - The physical address to be mapped. Must be aligned to a PPN
 *     address boundary which is dependent on the level of the entry
 *   vaddr - The virtual address to be mapped. Must be aligned to a PPN
 *     address boundary which is dependent on the level of the entry
 *   mmuflags - The MMU flags to use in the mapping.
 *
 ****************************************************************************/

void mmu_ln_setentry(uint32_t ptlevel, uintptr_t lnvaddr, uintptr_t paddr,
                     uintptr_t vaddr, uint32_t mmuflags);

/****************************************************************************
 * Name: mmu_ln_getentry
 *
 * Description:
 *   Get a level n translation table entry.
 *
 * Input Parameters:
 *   ptlevel - The translation table level, amount of levels is
 *     MMU implementation specific
 *   lnvaddr - The virtual address of the beginning of the page table at
 *     level n
 *   vaddr - The virtual address to get pte for. Must be aligned to a PPN
 *     address boundary which is dependent on the level of the entry
 *
 ****************************************************************************/

uintptr_t mmu_ln_getentry(uint32_t ptlevel, uintptr_t lnvaddr,
                          uintptr_t vaddr);

/****************************************************************************
 * Name: mmu_ln_map_region
 *
 * Description:
 *   Set a translation table region for level n
 *
 * Input Parameters:
 *   ptlevel - The translation table level, amount of levels is
 *     MMU implementation specific
 *   lnvaddr - The virtual address of the beginning of the page table at
 *     level n
 *   paddr - The physical address to be mapped. Must be aligned to a PPN
 *     address boundary which is dependent on the level of the entry
 *   vaddr - The virtual address to be mapped. Must be aligned to a PPN
 *     address boundary which is dependent on the level of the entry
 *   size - The size of the region in bytes
 *   mmuflags - The MMU flags to use in the mapping.
 *
 ****************************************************************************/

void mmu_ln_map_region(uint32_t ptlevel, uintptr_t lnvaddr, uintptr_t paddr,
                       uintptr_t vaddr, size_t size, uint32_t mmuflags);

#endif  /* __ARCH_X86_64_SRC_COMMON_X86_64_MMU_H */
