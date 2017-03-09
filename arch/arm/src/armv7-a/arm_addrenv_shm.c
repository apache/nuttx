/****************************************************************************
 * arch/arm/src/armv7/arm_addrenv_shm.c
 *
 *   Copyright (C) 2014, 2016 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <string.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/sched.h>
#include <nuttx/addrenv.h>

#include "mmu.h"
#include "cache.h"
#include "addrenv.h"
#include "pgalloc.h"

#if defined(CONFIG_BUILD_KERNEL) && defined(CONFIG_MM_SHM)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_shmat
 *
 * Description:
 *   Attach, i.e, map, on shared memory region to a user virtual address
 *
 * Input Parameters:
 *   pages - A pointer to the first element in a array of physical address,
 *     each corresponding to one page of memory.
 *   npages - The number of pages in the list of physical pages to be mapped.
 *   vaddr - The virtual address corresponding to the beginning of the
 *     (contiguous) virtual address region.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int up_shmat(FAR uintptr_t *pages, unsigned int npages, uintptr_t vaddr)
{
  FAR struct tcb_s *tcb = sched_self();
  FAR struct task_group_s *group;
  FAR uintptr_t *l1entry;
  FAR uint32_t *l2table;
  irqstate_t flags;
  uintptr_t paddr;
#ifndef CONFIG_ARCH_PGPOOL_MAPPING
  uint32_t l1save;
#endif
  unsigned int nmapped;
  unsigned int shmndx;

  shminfo("pages=%p npages=%d vaddr=%08lx\n",
          pages, npages, (unsigned long)vaddr);

  /* Sanity checks */

  DEBUGASSERT(pages && npages > 0 && tcb && tcb->group);
  DEBUGASSERT(vaddr >= CONFIG_ARCH_SHM_VBASE && vaddr < ARCH_SHM_VEND);
  DEBUGASSERT(MM_ISALIGNED(vaddr));

  group = tcb->group;

  /* Loop until all pages have been mapped into the caller's address space. */

  for (nmapped = 0; nmapped < npages; )
    {
      /* Get the shm[] index associated with the virtual address */

      shmndx = (vaddr - CONFIG_ARCH_SHM_VBASE) >> SECTION_SHIFT;

      /* Has a level 1 page table entry been created for this virtual address */

      l1entry = group->tg_addrenv.shm[shmndx];
      if (l1entry == NULL)
        {
          /* No.. Allocate one physical page for the L2 page table */

          paddr = mm_pgalloc(1);
          if (!paddr)
            {
              return -ENOMEM;
            }

          DEBUGASSERT(MM_ISALIGNED(paddr));

          /* We need to be more careful after we begin modifying
           * global resources.
           */

          flags = enter_critical_section();
          group->tg_addrenv.shm[shmndx] = (FAR uintptr_t *)paddr;

#ifdef CONFIG_ARCH_PGPOOL_MAPPING
          /* Get the virtual address corresponding to the physical page
           * address.
           */

          l2table = (FAR uint32_t *)arm_pgvaddr(paddr);
#else
          /* Temporarily map the page into the virtual address space */

          l1save = mmu_l1_getentry(ARCH_SCRATCH_VBASE);
          mmu_l1_setentry(paddr & ~SECTION_MASK, ARCH_SCRATCH_VBASE,
                          MMU_MEMFLAGS);
          l2table = (FAR uint32_t *)
            (ARCH_SCRATCH_VBASE | (paddr & SECTION_MASK));
#endif

          /* Initialize the page table */

          memset(l2table, 0, ENTRIES_PER_L2TABLE * sizeof(uint32_t));
        }
      else
        {
          /* Get the physical address of the L2 page table from the L1 page
           * table entry.
           */

          paddr = (uintptr_t)l1entry & ~SECTION_MASK;
          flags = enter_critical_section();

#ifdef CONFIG_ARCH_PGPOOL_MAPPING
          /* Get the virtual address corresponding to the physical page\
           * address.
           */

          l2table = (FAR uint32_t *)arm_pgvaddr(paddr);
#else
          /* Temporarily map the page into the virtual address space */

          l1save = mmu_l1_getentry(ARCH_SCRATCH_VBASE);
          mmu_l1_setentry(paddr & ~SECTION_MASK, ARCH_SCRATCH_VBASE,
                          MMU_MEMFLAGS);
          l2table = (FAR uint32_t *)
            (ARCH_SCRATCH_VBASE | (paddr & SECTION_MASK));
#endif
        }

      /* Map the virtual address to this physical address */

      DEBUGASSERT(get_l2_entry(l2table, vaddr) == 0);

      paddr = *pages++;
      set_l2_entry(l2table, paddr, vaddr, MMU_MEMFLAGS);
      nmapped++;
      vaddr += MM_PGSIZE;

      /* Make sure that the initialized L2 table is flushed to physical
       * memory.
       *
       * REVISIT: We could be smarter in doing this.  Currently, we may
       * flush the entire L2 page table numerous times.
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

  return OK;
}

/****************************************************************************
 * Name: up_shmdt
 *
 * Description:
 *   Detach, i.e, unmap, on shared memory region from a user virtual address
 *
 * Input Parameters:
 *   vaddr - The virtual address corresponding to the beginning of the
 *     (contiguous) virtual address region.
 *   npages - The number of pages to be unmapped
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int up_shmdt(uintptr_t vaddr, unsigned int npages)
{
  FAR struct tcb_s *tcb = sched_self();
  FAR struct task_group_s *group;
  FAR uintptr_t *l1entry;
  FAR uint32_t *l2table;
  irqstate_t flags;
  uintptr_t paddr;
#ifndef CONFIG_ARCH_PGPOOL_MAPPING
  uint32_t l1save;
#endif
  unsigned int nunmapped;
  unsigned int shmndx;

  shminfo("npages=%d vaddr=%08lx\n", npages, (unsigned long)vaddr);

  /* Sanity checks */

  DEBUGASSERT(npages > 0 && tcb && tcb->group);
  DEBUGASSERT(vaddr >= CONFIG_ARCH_SHM_VBASE && vaddr < ARCH_SHM_VEND);
  DEBUGASSERT(MM_ISALIGNED(vaddr));

  group = tcb->group;

  /* Loop until all pages have been unmapped from the caller's address
   * space.
   */

  for (nunmapped = 0; nunmapped < npages; )
    {
      /* Get the shm[] index associated with the virtual address */

      shmndx = (vaddr - CONFIG_ARCH_SHM_VBASE) >> SECTION_SHIFT;

      /* Get the level 1 page table entry for this virtual address */

      l1entry = group->tg_addrenv.shm[shmndx];
      DEBUGASSERT(l1entry != NULL);

      /* Get the physical address of the L2 page table from the L1 page
       * table entry.
       */

       paddr = (uintptr_t)l1entry & ~SECTION_MASK;
       flags = enter_critical_section();

#ifdef CONFIG_ARCH_PGPOOL_MAPPING
      /* Get the virtual address corresponding to the physical page
       * address.
       */

      l2table = (FAR uint32_t *)arm_pgvaddr(paddr);
#else
      /* Temporarily map the page into the virtual address space */

      l1save = mmu_l1_getentry(ARCH_SCRATCH_VBASE);
      mmu_l1_setentry(paddr & ~SECTION_MASK, ARCH_SCRATCH_VBASE,
                      MMU_MEMFLAGS);
      l2table = (FAR uint32_t *)
        (ARCH_SCRATCH_VBASE | (paddr & SECTION_MASK));
#endif

      /* Unmap this virtual page address.
       *
       * REVISIT: Note that the page allocated for the level 2 page table
       * is not freed nor is the level 1 page table entry ever cleared.
       * This means that the 4KiB page is still allocated to the process
       * even though it may not contain any mappings and that the it will
       * persist until the process terminates.  That is not all bad because
       * it means that we will be able to re-instantiate the shared memory
       * mapping very quickly.
       */

      DEBUGASSERT(get_l2_entry(l2table, vaddr) != 0);

      clr_l2_entry(l2table, vaddr);
      nunmapped++;
      vaddr += MM_PGSIZE;

      /* Make sure that the modified L2 table is flushed to physical
       * memory.
       *
       * REVISIT: We could be smarter in doing this.  Currently, we may
       * flush the entire L2 page table numerous times.
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

  return OK;
}

#endif /* CONFIG_BUILD_KERNEL && CONFIG_MM_SHM */
