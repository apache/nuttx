/****************************************************************************
 * arch/arm/src/armv7/arm_addrenv.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/pgalloc.h>

#include <nuttx/irq.h>

#include "cache.h"
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

int arm_addrenv_create_region(FAR uintptr_t **list, unsigned int listlen,
                              uintptr_t vaddr, size_t regionsize,
                              uint32_t mmuflags)
{
  irqstate_t flags;
  uintptr_t paddr;
  FAR uint32_t *l2table;
#ifndef CONFIG_ARCH_PGPOOL_MAPPING
  uint32_t l1save;
#endif
  size_t nmapped;
  unsigned int npages;
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

  nmapped = 0;
  for (i = 0; i < npages; i += ENTRIES_PER_L2TABLE)
    {
      /* Allocate one physical page for the L2 page table */

      paddr = mm_pgalloc(1);
      if (!paddr)
        {
          return -ENOMEM;
        }

      DEBUGASSERT(MM_ISALIGNED(paddr));
      list[i] = (FAR uintptr_t *)paddr;

      flags = enter_critical_section();

#ifdef CONFIG_ARCH_PGPOOL_MAPPING
      /* Get the virtual address corresponding to the physical page address */

      l2table = (FAR uint32_t *)arm_pgvaddr(paddr);
#else
      /* Temporarily map the page into the virtual address space */

      l1save = mmu_l1_getentry(ARCH_SCRATCH_VBASE);
      mmu_l1_setentry(paddr & ~SECTION_MASK, ARCH_SCRATCH_VBASE, MMU_MEMFLAGS);
      l2table = (FAR uint32_t *)(ARCH_SCRATCH_VBASE | (paddr & SECTION_MASK));
#endif

      /* Initialize the page table */

      memset(l2table, 0, ENTRIES_PER_L2TABLE * sizeof(uint32_t));

      /* Back up L2 entries with physical memory */

      for (j = 0; j < ENTRIES_PER_L2TABLE && nmapped < regionsize; j++)
        {
          /* Allocate one physical page for region data */

          paddr = mm_pgalloc(1);
          if (!paddr)
            {
#ifndef CONFIG_ARCH_PGPOOL_MAPPING
              mmu_l1_restore(ARCH_SCRATCH_VBASE, l1save);
#endif
              leave_critical_section(flags);
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

      arch_flush_dcache((uintptr_t)l2table,
                        (uintptr_t)l2table +
                        ENTRIES_PER_L2TABLE * sizeof(uint32_t));

#ifndef CONFIG_ARCH_PGPOOL_MAPPING
      /* Restore the scratch section L1 page table entry */

      mmu_l1_restore(ARCH_SCRATCH_VBASE, l1save);
#endif
      leave_critical_section(flags);
    }

  return npages;
}

/****************************************************************************
 * Name: arm_addrenv_destroy_region
 *
 * Description:
 *   Destroy one memory region.
 *
 ****************************************************************************/

void arm_addrenv_destroy_region(FAR uintptr_t **list, unsigned int listlen,
                                uintptr_t vaddr, bool keep)
{
  irqstate_t flags;
  uintptr_t paddr;
  FAR uint32_t *l2table;
#ifndef CONFIG_ARCH_PGPOOL_MAPPING
  uint32_t l1save;
#endif
  int i;
  int j;

  binfo("listlen=%d vaddr=%08lx\n", listlen, (unsigned long)vaddr);

  for (i = 0; i < listlen; vaddr += SECTION_SIZE, list++, i++)
    {
      /* Unhook the L2 page table from the L1 page table */

      mmu_l1_clrentry(vaddr);

      /* Has this page table been allocated? */

      paddr = (uintptr_t)list[i];
      if (paddr != 0)
        {
          flags = enter_critical_section();

#ifdef CONFIG_ARCH_PGPOOL_MAPPING
          /* Get the virtual address corresponding to the physical page address */

          l2table = (FAR uint32_t *)arm_pgvaddr(paddr);
#else
          /* Temporarily map the page into the virtual address space */

          l1save = mmu_l1_getentry(ARCH_SCRATCH_VBASE);
          mmu_l1_setentry(paddr & ~SECTION_MASK, ARCH_SCRATCH_VBASE, MMU_MEMFLAGS);
          l2table = (FAR uint32_t *)(ARCH_SCRATCH_VBASE | (paddr & SECTION_MASK));
#endif

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
                  paddr = *l2table++;
                  if (paddr != 0)
                    {
                      paddr &= PTE_SMALL_PADDR_MASK;
                      mm_pgfree(paddr, 1);
                    }
                }
            }

#ifndef CONFIG_ARCH_PGPOOL_MAPPING
          /* Restore the scratch section L1 page table entry */

          mmu_l1_restore(ARCH_SCRATCH_VBASE, l1save);
#endif
          leave_critical_section(flags);

          /* And free the L2 page table itself */

          mm_pgfree((uintptr_t)list[i], 1);
        }
    }
}

#endif /* CONFIG_ARCH_ADDRENV */
