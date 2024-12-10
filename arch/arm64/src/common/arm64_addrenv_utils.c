/****************************************************************************
 * arch/arm64/src/common/arm64_addrenv_utils.c
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
#include <nuttx/addrenv.h>
#include <nuttx/irq.h>
#include <nuttx/pgalloc.h>
#include <nuttx/sched.h>

#include <arch/barriers.h>

#include "pgalloc.h"
#include "arm64_mmu.h"

#ifdef CONFIG_BUILD_KERNEL

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm64_get_pgtable
 *
 * Description:
 *   Get the physical address of the final level page table corresponding to
 *   'vaddr'. If one does not exist, it will be allocated.
 *
 * Input Parameters:
 *   addrenv - Pointer to a structure describing the address environment
 *   vaddr - Virtual address to query for
 *
 * Returned Value:
 *   The physical address of the corresponding final level page table, or
 *   NULL if one does not exist, and there is no free memory to allocate one
 *
 ****************************************************************************/

uintptr_t arm64_get_pgtable(arch_addrenv_t *addrenv, uintptr_t vaddr)
{
  uintptr_t paddr;
  uintptr_t ptprev;
  uint32_t  ptlevel;
  uint32_t  flags;

  /* Get the current level MAX_LEVELS-1 entry corresponding to this vaddr */

  ptlevel = MMU_PGT_LEVEL_MAX - 1;
  ptprev  = arm64_pgvaddr(addrenv->spgtables[ptlevel]);
  if (!ptprev)
    {
      /* Something is very wrong */

      return 0;
    }

  /* Find the physical address of the final level page table */

  paddr = mmu_pte_to_paddr(mmu_ln_getentry(ptlevel, ptprev, vaddr));
  if (!paddr)
    {
      /* No page table has been allocated... allocate one now */

      paddr = mm_pgalloc(1);
      if (paddr)
        {
          /* Determine page table flags */

          if (arm64_uservaddr(vaddr))
            {
              flags = MMU_UPGT_FLAGS;
            }
          else
            {
              flags = MMU_KPGT_FLAGS;
            }

          /* Wipe the page and assign it */

          arm64_pgwipe(paddr);
          mmu_ln_setentry(ptlevel, ptprev, paddr, vaddr, flags);
        }
    }

  return paddr;
}

/****************************************************************************
 * Name: arm64_map_pages
 *
 * Description:
 *   Map physical pages into a continuous virtual memory block.
 *
 * Input Parameters:
 *   addrenv - Pointer to a structure describing the address environment.
 *   pages - A pointer to the first element in a array of physical address,
 *     each corresponding to one page of memory.
 *   npages - The number of pages in the list of physical pages to be mapped.
 *   vaddr - The virtual address corresponding to the beginning of the
 *     (continuous) virtual address region.
 *   prot - MMU flags to use.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int arm64_map_pages(arch_addrenv_t *addrenv, uintptr_t *pages,
                    unsigned int npages, uintptr_t vaddr, uint64_t prot)
{
  uintptr_t ptlast;
  uintptr_t ptlevel;
  uintptr_t paddr;

  ptlevel = MMU_PGT_LEVEL_MAX;

  /* Add the references to pages[] into the caller's address environment */

  for (; npages > 0; npages--)
    {
      /* Get the address of the last level page table */

      ptlast = arm64_pgvaddr(arm64_get_pgtable(addrenv, vaddr));
      if (!ptlast)
        {
          return -ENOMEM;
        }

      /* Then add the reference */

      paddr = *pages++;
      mmu_ln_setentry(ptlevel, ptlast, paddr, vaddr, prot);
      vaddr += MM_PGSIZE;
    }

  return OK;
}

/****************************************************************************
 * Name: arm64_unmap_pages
 *
 * Description:
 *   Unmap a previously mapped virtual memory region.
 *
 * Input Parameters:
 *   addrenv - Pointer to a structure describing the address environment.
 *   vaddr - The virtual address corresponding to the beginning of the
 *     (continuous) virtual address region.
 *   npages - The number of pages to be unmapped
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int arm64_unmap_pages(arch_addrenv_t *addrenv, uintptr_t vaddr,
                      unsigned int npages)
{
  uintptr_t ptlast;
  uintptr_t ptprev;
  uintptr_t ptlevel;
  uintptr_t paddr;

  /* Get the current level MAX_LEVELS-1 entry corresponding to this vaddr */

  ptlevel = MMU_PGT_LEVEL_MAX - 1;
  ptprev  = arm64_pgvaddr(addrenv->spgtables[ptlevel]);
  if (!ptprev)
    {
      /* Something is very wrong */

      return -EFAULT;
    }

  /* Remove the references from the caller's address environment */

  for (; npages > 0; npages--)
    {
      /* Get the current final level entry corresponding to this vaddr */

      paddr = mmu_pte_to_paddr(mmu_ln_getentry(ptlevel, ptprev, vaddr));
      ptlast = arm64_pgvaddr(paddr);
      if (!ptlast)
        {
          return -EFAULT;
        }

      /* Then wipe the reference */

      mmu_ln_clear(ptlevel + 1, ptlast, vaddr);
      vaddr += MM_PGSIZE;
    }

  return OK;
}

#endif /* CONFIG_BUILD_KERNEL */
