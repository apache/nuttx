/****************************************************************************
 * arch/arm/src/armv7-a/arm_addrenv_utils.c
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
#include <nuttx/debug.h>

#include <nuttx/pgalloc.h>
#include <nuttx/irq.h>
#include <nuttx/cache.h>

#include "mmu.h"
#include "pgalloc.h"
#include "addrenv.h"

#ifdef CONFIG_ARCH_ADDRENV

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_addrenv_create_region
 *
 * Description:
 *   Create one memory region.
 *
 * Returned Value:
 *   On success, the number of pages allocated is returned.  Otherwise, a
 *   negated errno value is returned.
 *
 ****************************************************************************/

int arm_addrenv_create_region(uintptr_t *l1table, unsigned int listlen,
                              uintptr_t vaddr, size_t regionsize,
                              uint32_t mmuflags)
{
  uintptr_t paddr;
  uintptr_t *l2table;
  size_t nmapped;
  unsigned int npages;
  unsigned int nlist;
  unsigned int i;
  unsigned int j;

  binfo("listlen=%d vaddr=%08lx regionsize=%ld, mmuflags=%08x\n",
        listlen, (unsigned long)vaddr, (unsigned long)regionsize,
        (unsigned int)mmuflags);

  /* Verify that we are configured with enough virtual address space to
   * support this memory region.
   *
   *   npages pages corresponds to (npages << MM_PGSHIFT) bytes
   *   listlen sections corresponds to (listlen << 20) bytes
   */

  npages = MM_NPAGES(regionsize);
  if (npages > (listlen << (20 - MM_PGSHIFT)))
    {
      berr("ERROR: npages=%u listlen=%u\n", npages, listlen);
      return -E2BIG;
    }

  /* Back the allocation up with physical pages and set up the level mapping
   * (which of course does nothing until the L2 page table is hooked into
   * the L1 page table).
   */

  nlist = (npages + ENTRIES_PER_L2TABLE - 1) / ENTRIES_PER_L2TABLE;
  nmapped = 0;
  for (i = 0; i < nlist; i++)
    {
      /* Allocate one physical page for the L2 page table */

      paddr = mm_pgalloc(1);
      binfo("a new l2 page table (paddr=%x)\n", paddr);
      if (!paddr)
        {
          return -ENOMEM;
        }

      DEBUGASSERT(MM_ISALIGNED(paddr));

      mmu_l1table_setentry(l1table, paddr, vaddr, MMU_L1_PGTABFLAGS);

      /* Get the virtual address corresponding to the physical page address */

      l2table = (uintptr_t *)arm_pgvaddr(paddr);

      /* Initialize the page table */

      memset(l2table, 0, ENTRIES_PER_L2TABLE * sizeof(uintptr_t));

      /* Back up L2 entries with physical memory */

      for (j = 0; j < ENTRIES_PER_L2TABLE && nmapped < regionsize; j++)
        {
          /* Allocate one physical page for region data */

          paddr = mm_pgalloc(1);
          binfo("a new page (paddr=%x)\n", paddr);
          if (!paddr)
            {
              return -ENOMEM;
            }

          /* Map the virtual address to this physical address */

          set_l2_entry(l2table, paddr, vaddr, mmuflags);
          nmapped += MM_PGSIZE;
          vaddr   += MM_PGSIZE;
        }

      /* Make sure that the initialized L2 table is flushed to physical
       * memory.
       */

      up_flush_dcache((uintptr_t)l2table,
                      (uintptr_t)l2table +
                      ENTRIES_PER_L2TABLE * sizeof(uintptr_t));
    }

  return npages;
}

#ifdef CONFIG_ARCH_HAVE_FORK
/****************************************************************************
 * Name: arm_addrenv_fork_region
 *
 * Description:
 *   Duplicate one memory region of an address environment for fork().
 *
 *   The destination gets its own L2 page tables and, unless `share' is
 *   requested, its own physical pages holding a copy of the source's
 *   contents.  Both are mapped at the same virtual addresses as the source,
 *   which is what makes the duplicate an exact copy:  a pointer the parent
 *   held into its own memory means the same thing in the child.
 *
 *   Only the sections the source actually has mapped are duplicated, so the
 *   copy is the size of the parent, not the size of the region.
 *
 *   `share' maps the source's own pages into the destination rather than
 *   copying them.  It is used for the shared memory region, which POSIX says
 *   remains shared across fork().
 *
 * Input Parameters:
 *   srcl1    - The L1 page table of the address environment to duplicate
 *   destl1   - The L1 page table receiving the duplicate
 *   listlen  - Number of sections in the region
 *   vaddr    - Virtual base address of the region
 *   mmuflags - L2 flags for the duplicated pages
 *   share    - Map the source's pages rather than copying them
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.  On failure the
 *   caller is expected to tear the destination down with
 *   up_addrenv_destroy(), which copes with a partially built environment.
 *
 ****************************************************************************/

int arm_addrenv_fork_region(uintptr_t *srcl1, uintptr_t *destl1,
                            unsigned int listlen, uintptr_t vaddr,
                            uint32_t mmuflags, bool share)
{
  uintptr_t l1entry;
  uintptr_t *srcl2;
  uintptr_t *destl2;
  uintptr_t paddr;
  unsigned int i;
  unsigned int j;

  binfo("listlen=%d vaddr=%08lx share=%d\n", listlen,
        (unsigned long)vaddr, share);

  for (i = 0; i < listlen; i++, vaddr += SECTION_SIZE)
    {
      /* Skip sections the source never populated */

      l1entry = mmu_l1table_getentry(srcl1, vaddr);
      if (l1entry == 0)
        {
          continue;
        }

      srcl2 = (uintptr_t *)arm_pgvaddr(l1entry & PTE_SMALL_PADDR_MASK);

      /* Allocate one physical page for the destination L2 page table */

      paddr = mm_pgalloc(1);
      if (!paddr)
        {
          return -ENOMEM;
        }

      DEBUGASSERT(MM_ISALIGNED(paddr));

      mmu_l1table_setentry(destl1, paddr, vaddr, MMU_L1_PGTABFLAGS);

      destl2 = (uintptr_t *)arm_pgvaddr(paddr);
      memset(destl2, 0, ENTRIES_PER_L2TABLE * sizeof(uintptr_t));

      for (j = 0; j < ENTRIES_PER_L2TABLE; j++)
        {
          uintptr_t srcpage = srcl2[j];
          uintptr_t destpage;

          if (srcpage == 0)
            {
              continue;
            }

          if (share)
            {
              /* Map the very same page.  Note that up_addrenv_destroy()
               * tears a shared region down with keep=true, so this does not
               * hand the same page to the page allocator twice.
               */

              destl2[j] = srcpage;
              continue;
            }

          srcpage &= PTE_SMALL_PADDR_MASK;

          destpage = mm_pgalloc(1);
          if (!destpage)
            {
              return -ENOMEM;
            }

          memcpy((void *)arm_pgvaddr(destpage),
                 (const void *)arm_pgvaddr(srcpage), MM_PGSIZE);

          up_flush_dcache(arm_pgvaddr(destpage),
                          arm_pgvaddr(destpage) + MM_PGSIZE);

          set_l2_entry(destl2, destpage, vaddr + (j << MM_PGSHIFT),
                       mmuflags);
        }

      /* Make sure the initialized L2 table is visible to the MMU */

      up_flush_dcache((uintptr_t)destl2,
                      (uintptr_t)destl2 +
                      ENTRIES_PER_L2TABLE * sizeof(uintptr_t));
    }

  return OK;
}
#endif /* CONFIG_ARCH_HAVE_FORK */

/****************************************************************************
 * Name: arm_addrenv_destroy_region
 *
 * Description:
 *   Destroy one memory region.
 *
 ****************************************************************************/

void arm_addrenv_destroy_region(uintptr_t *l1table, unsigned int listlen,
                                uintptr_t vaddr, bool keep)
{
  uintptr_t l1entry;
  uintptr_t *l2table;
  int i;
  int j;

  binfo("listlen=%d vaddr=%08lx\n", listlen, (unsigned long)vaddr);

  for (i = 0; i < listlen; vaddr += SECTION_SIZE, i++)
    {
      /* Has this page table been allocated? */

      l1entry = mmu_l1table_getentry(l1table, vaddr);
      if (l1entry != 0)
        {
          l1entry &= PTE_SMALL_PADDR_MASK;

          /* Get the virtual address corresponding to the physical page
           * address
           */

          l2table = (uintptr_t *)arm_pgvaddr(l1entry);

          /* Return the allocated pages to the page allocator unless we were
           * asked to keep the page data.  We keep the page data only for
           * the case of shared memory.  In that case, we need to tear down
           * the mapping and page table entries, but keep the raw page data
           * will still may be mapped by other user processes.
           */

          if (!keep)
            {
              for (j = 0; j < ENTRIES_PER_L2TABLE; j++)
                {
                  uintptr_t paddr = *l2table++;

                  if (paddr != 0)
                    {
                      paddr &= PTE_SMALL_PADDR_MASK;
                      mm_pgfree(paddr, 1);
                    }
                }
            }

          /* And free the L2 page table itself.
           * The l1table will be entire free, so don't need to set 0,
           */

          mm_pgfree((uintptr_t)l1entry, 1);
        }
    }
}

#endif /* CONFIG_ARCH_ADDRENV */
