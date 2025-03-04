/****************************************************************************
 * arch/x86_64/src/common/x86_64_mmu.c
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

#include <nuttx/config.h>

#include <nuttx/arch.h>

#include "x86_64_internal.h"
#include "x86_64_mmu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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
 *   paddr - The physical address to be mapped
 *   vaddr - The virtual address to be mapped
 *   mmuflags - The MMU flags to use in the mapping.
 *
 ****************************************************************************/

void mmu_ln_setentry(uint32_t ptlevel, uintptr_t lnvaddr, uintptr_t paddr,
                     uintptr_t vaddr, uint32_t mmuflags)
{
  uintptr_t *lntable = (uintptr_t *)lnvaddr;
  uint32_t   index;

  DEBUGASSERT(ptlevel >= 0 && ptlevel < X86_MMU_PT_LEVELS);

  /* Make sure the entry is valid */

  mmuflags |= X86_PAGE_PRESENT;

  /* Calculate index for lntable */

  index = X86_MMU_VADDR_INDEX(vaddr, ptlevel);

  /* Save it */

  lntable[index] = (paddr | mmuflags);

  /* Update with memory by flushing the cache(s) */

  up_invalid_tlb(vaddr, vaddr + 1);
}

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
 *   vaddr - The virtual address to get pte for
 *
 ****************************************************************************/

uintptr_t mmu_ln_getentry(uint32_t ptlevel, uintptr_t lnvaddr,
                          uintptr_t vaddr)
{
  uintptr_t *lntable = (uintptr_t *)lnvaddr;
  uint32_t  index;

  DEBUGASSERT(ptlevel >= 0 && ptlevel < X86_MMU_PT_LEVELS);

  index = X86_MMU_VADDR_INDEX(vaddr, ptlevel);

  return lntable[index];
}

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
 *   paddr - The physical address to be mapped
 *   vaddr - The virtual address to be mapped
 *   size - The size of the region in bytes
 *   mmuflags - The MMU flags to use in the mapping.
 *
 ****************************************************************************/

void mmu_ln_map_region(uint32_t ptlevel, uintptr_t lnvaddr, uintptr_t paddr,
                       uintptr_t vaddr, size_t size, uint32_t mmuflags)
{
  uintptr_t end_paddr = paddr + size;
  size_t    page_size = X86_MMU_PAGE_SIZE;

  DEBUGASSERT(ptlevel >= 0 && ptlevel < X86_MMU_PT_LEVELS);

  while (paddr < end_paddr)
    {
      mmu_ln_setentry(ptlevel, lnvaddr, paddr, vaddr, mmuflags);
      paddr += page_size;
      vaddr += page_size;
    }
}
