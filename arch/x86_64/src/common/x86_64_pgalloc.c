/****************************************************************************
 * arch/x86_64/src/common/x86_64_pgalloc.c
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

#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/sched.h>
#include <nuttx/arch.h>
#include <nuttx/addrenv.h>
#include <nuttx/pgalloc.h>
#include <nuttx/spinlock.h>

#include "addrenv.h"
#include "pgalloc.h"
#include "x86_64_mmu.h"

#ifdef CONFIG_BUILD_KERNEL

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Last PGT level */

#define PGT_LAST    (X86_MMU_PT_LEVELS)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: x86_64_get_pgtable
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

uintptr_t x86_64_get_pgtable(arch_addrenv_t *addrenv, uintptr_t vaddr)
{
  uintptr_t paddr;
  uintptr_t ptprev;
  uint32_t  ptlevel;
  uint32_t  flags;

  /* Get the current level MAX_LEVELS-1 entry corresponding to this vaddr */

  ptlevel = ARCH_SPGTS;
  ptprev  = x86_64_pgvaddr(addrenv->spgtables[ARCH_SPGTS - 1]);
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

          if (x86_64_uservaddr(vaddr))
            {
              flags = MMU_UPGT_FLAGS;
            }
          else
            {
              flags = MMU_KPGT_FLAGS;
            }

          /* Wipe the page and assign it */

          x86_64_pgwipe(paddr);
          mmu_ln_setentry(ptlevel, ptprev, paddr, vaddr, flags);
        }
    }

  /* Flush the data cache, so the changes are committed to memory */

  UP_DMB();

  return paddr;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pgalloc
 *
 * Description:
 *   If there is a page allocator in the configuration and if and MMU is
 *   available to map physical addresses to virtual address, then function
 *   must be provided by the platform-specific code.  This is part of the
 *   implementation of sbrk().  This function will allocate the requested
 *   number of pages using the page allocator and map them into consecutive
 *   virtual addresses beginning with 'brkaddr'
 *
 *   NOTE:  This function does not use the up_ naming standard because it
 *   is indirectly callable from user-space code via a system trap.
 *   Therefore, it is a system interface and follows a different naming
 *   convention.
 *
 * Input Parameters:
 *   brkaddr - The heap break address.  The next page will be allocated and
 *     mapped to this address.  Must be page aligned.  If the memory manager
 *     has not yet been initialized and this is the first block requested for
 *     the heap, then brkaddr should be zero.  pgalloc will then assigned the
 *     well-known virtual address of the beginning of the heap.
 *   npages - The number of pages to allocate and map.  Mapping of pages
 *     will be contiguous beginning beginning at 'brkaddr'
 *
 * Returned Value:
 *   The (virtual) base address of the mapped page will returned on success.
 *   Normally this will be the same as the 'brkaddr' input. However, if
 *   the 'brkaddr' input was zero, this will be the virtual address of the
 *   beginning of the heap.  Zero is returned on any failure.
 *
 ****************************************************************************/

uintptr_t pgalloc(uintptr_t brkaddr, unsigned int npages)
{
  struct tcb_s          *tcb = nxsched_self();
  struct arch_addrenv_s *addrenv;
  uintptr_t              ptlast;
  uintptr_t              paddr;
  uintptr_t              vaddr;

  DEBUGASSERT(tcb && tcb->addrenv_own);
  addrenv = &tcb->addrenv_own->addrenv;

  /* The current implementation only supports extending the user heap
   * region as part of the implementation of user sbrk().  This function
   * needs to be expanded to also handle (1) extending the user stack
   * space and (2) extending the kernel memory regions as well.
   */

  /* brkaddr = 0 means that no heap has yet been allocated */

  if (!brkaddr)
    {
      brkaddr = addrenv->heapvbase;
    }

  /* Start mapping from the old heap break address */

  vaddr = brkaddr;

  /* Sanity checks */

  DEBUGASSERT(brkaddr >= addrenv->heapvbase);
  DEBUGASSERT(MM_ISALIGNED(brkaddr));

  for (; npages > 0; npages--)
    {
      /* Get the address of the last level page table */

      ptlast = x86_64_pgvaddr(x86_64_get_pgtable(addrenv, vaddr));
      if (!ptlast)
        {
          return 0;
        }

      /* Allocate physical memory for the new heap */

      paddr = mm_pgalloc(1);
      if (!paddr)
        {
          return 0;
        }

      /* Wipe the memory */

      x86_64_pgwipe(paddr);

      /* Then add the reference */

      mmu_ln_setentry(PGT_LAST, ptlast, paddr, vaddr, MMU_UDATA_FLAGS);
      vaddr += MM_PGSIZE;
    }

  /* Flush the data cache, so the changes are committed to memory */

  UP_DMB();

  return brkaddr;
}

#endif /* CONFIG_BUILD_KERNEL */
