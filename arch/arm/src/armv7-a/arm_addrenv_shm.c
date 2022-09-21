/****************************************************************************
 * arch/arm/src/armv7-a/arm_addrenv_shm.c
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

#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/sched.h>
#include <nuttx/addrenv.h>

#include "mmu.h"
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

int up_shmat(uintptr_t *pages, unsigned int npages, uintptr_t vaddr)
{
  struct tcb_s *tcb = nxsched_self();
  struct task_group_s *group;
  uintptr_t *l1entry;
  uint32_t *l2table;
  irqstate_t flags;
  uintptr_t paddr;
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

      /* Has a level 1 page table entry been created for this virtual
       * address.
       */

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
          group->tg_addrenv.shm[shmndx] = (uintptr_t *)paddr;

          /* Get the virtual address corresponding to the physical page
           * address.
           */

          l2table = (uint32_t *)arm_pgvaddr(paddr);

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

          /* Get the virtual address corresponding to the physical page\
           * address.
           */

          l2table = (uint32_t *)arm_pgvaddr(paddr);
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

      up_flush_dcache((uintptr_t)l2table,
                      (uintptr_t)l2table +
                      ENTRIES_PER_L2TABLE * sizeof(uint32_t));

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
  struct tcb_s *tcb = nxsched_self();
  struct task_group_s *group;
  uintptr_t *l1entry;
  uint32_t *l2table;
  irqstate_t flags;
  uintptr_t paddr;
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

      /* Get the virtual address corresponding to the physical page
       * address.
       */

      l2table = (uint32_t *)arm_pgvaddr(paddr);

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

      up_flush_dcache((uintptr_t)l2table,
                      (uintptr_t)l2table +
                      ENTRIES_PER_L2TABLE * sizeof(uint32_t));

      leave_critical_section(flags);
    }

  return OK;
}

#endif /* CONFIG_BUILD_KERNEL && CONFIG_MM_SHM */
