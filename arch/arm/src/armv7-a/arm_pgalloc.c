/****************************************************************************
 * arch/arm/src/armv7-a/arm_pgalloc.c
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

#include "mmu.h"
#include "pgalloc.h"

#ifdef CONFIG_BUILD_KERNEL

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: alloc_pgtable
 *
 * Description:
 *   Add one page table to a memory region.
 *
 ****************************************************************************/

static uintptr_t alloc_pgtable(void)
{
  irqstate_t flags;
  uintptr_t paddr;
  FAR uint32_t *l2table;
#ifndef CONFIG_ARCH_PGPOOL_MAPPING
  uint32_t l1save;
#endif

  /* Allocate one physical page for the L2 page table */

  paddr = mm_pgalloc(1);
  binfo("a new l2 page table (paddr=%x)\n", paddr);
  if (paddr)
    {
      DEBUGASSERT(MM_ISALIGNED(paddr));

      flags = enter_critical_section();

#ifdef CONFIG_ARCH_PGPOOL_MAPPING
      /* Get the virtual address corresponding to the physical page address */

      l2table = (FAR uint32_t *)arm_pgvaddr(paddr);
#else
      /* Temporarily map the page into the virtual address space */

      l1save = mmu_l1_getentry(ARCH_SCRATCH_VBASE);
      mmu_l1_setentry(paddr & ~SECTION_MASK, ARCH_SCRATCH_VBASE,
                      MMU_MEMFLAGS);
      l2table = (FAR uint32_t *)(ARCH_SCRATCH_VBASE |
                                 (paddr & SECTION_MASK));
#endif

      /* Initialize the page table */

      memset(l2table, 0, MM_PGSIZE);

      /* Make sure that the initialized L2 table is flushed to physical
       * memory.
       */

      up_flush_dcache((uintptr_t)l2table,
                      (uintptr_t)l2table + MM_PGSIZE);

#ifndef CONFIG_ARCH_PGPOOL_MAPPING
      /* Restore the scratch section page table entry */

      mmu_l1_restore(ARCH_SCRATCH_VBASE, l1save);
#endif
      leave_critical_section(flags);
    }

  return paddr;
}

/****************************************************************************
 * Name: get_pgtable
 *
 * Description:
 *   Return the physical address of the L2 page table corresponding to
 *   'vaddr'
 *
 ****************************************************************************/

static int get_pgtable(FAR group_addrenv_t *addrenv, uintptr_t vaddr)
{
  uint32_t l1entry;
  uintptr_t paddr;
  unsigned int hpoffset;
  unsigned int hpndx;

  /* The current implementation only supports extending the user heap
   * region as part of the implementation of user sbrk().
   */

  DEBUGASSERT(vaddr >= CONFIG_ARCH_HEAP_VBASE && vaddr < ARCH_HEAP_VEND);

  /* Get the current level 1 entry corresponding to this vaddr */

  hpoffset = vaddr - CONFIG_ARCH_HEAP_VBASE;
  if (hpoffset >= ARCH_HEAP_SIZE)
    {
      return 0;
    }

  hpndx   = hpoffset >> 20;
  l1entry = (uintptr_t)addrenv->heap[hpndx];
  if (l1entry == 0)
    {
      /* No page table has been allocated... allocate one now */

      paddr = alloc_pgtable();
      if (paddr != 0)
        {
          /* Set the new level 1 page table entry in the address
           * environment.
           */

          l1entry = paddr | MMU_L1_PGTABFLAGS;
          addrenv->heap[hpndx] = (FAR uintptr_t *)l1entry;

          /* And instantiate the modified environment */

          up_addrenv_select(addrenv, NULL);
        }
    }

  return l1entry & PMD_PTE_PADDR_MASK;
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
  FAR struct tcb_s *tcb = nxsched_self();
  FAR struct task_group_s *group;
  FAR uint32_t *l2table;
  irqstate_t flags;
  uintptr_t paddr;
#ifndef CONFIG_ARCH_PGPOOL_MAPPING
  uint32_t l1save;
#endif
  unsigned int index;

  binfo("tcb->pid=%d tcb->group=%p\n", tcb->pid, tcb->group);
  binfo("brkaddr=%x npages=%d\n", brkaddr, npages);
  DEBUGASSERT(tcb && tcb->group);
  group = tcb->group;

  /* The current implementation only supports extending the user heap
   * region as part of the implementation of user sbrk().  This function
   * needs to be expanded to also handle (1) extending the user stack
   * space and (2) extending the kernel memory regions as well.
   */

  DEBUGASSERT((group->tg_flags & GROUP_FLAG_ADDRENV) != 0);

  /* brkaddr = 0 means that no heap has yet been allocated */

  if (brkaddr == 0)
    {
      brkaddr = CONFIG_ARCH_HEAP_VBASE;
    }

  DEBUGASSERT(brkaddr >= CONFIG_ARCH_HEAP_VBASE && brkaddr < ARCH_HEAP_VEND);
  DEBUGASSERT(MM_ISALIGNED(brkaddr));

  for (; npages > 0; npages--)
    {
      /* Get the physical address of the level 2 page table */

      paddr = get_pgtable(&group->tg_addrenv, brkaddr);
      binfo("l2 page table (paddr=%x)\n", paddr);
      binfo("brkaddr=%x\n", brkaddr);
      if (paddr == 0)
        {
          return 0;
        }

      flags = enter_critical_section();

#ifdef CONFIG_ARCH_PGPOOL_MAPPING
      /* Get the virtual address corresponding to the physical page address */

      l2table = (FAR uint32_t *)arm_pgvaddr(paddr);
#else
      /* Temporarily map the level 2 page table into the "scratch" virtual
       * address space
       */

      l1save = mmu_l1_getentry(ARCH_SCRATCH_VBASE);
      mmu_l1_setentry(paddr & ~SECTION_MASK, ARCH_SCRATCH_VBASE,
                      MMU_MEMFLAGS);
      l2table = (FAR uint32_t *)(ARCH_SCRATCH_VBASE |
                                 (paddr & SECTION_MASK));
#endif

      /* Back up L2 entry with physical memory */

      paddr = mm_pgalloc(1);
      binfo("a new page (paddr=%x)\n", paddr);
      if (paddr == 0)
        {
#ifndef CONFIG_ARCH_PGPOOL_MAPPING
          mmu_l1_restore(ARCH_SCRATCH_VBASE, l1save);
#endif
          leave_critical_section(flags);
          return 0;
        }

      /* The table divides a 1Mb address space up into 256 entries, each
       * corresponding to 4Kb of address space.  The page table index is
       * related to the offset from the beginning of 1Mb region.
       */

      index = (brkaddr & 0x000ff000) >> 12;

      /* Map the .text region virtual address to this physical address */

      DEBUGASSERT(l2table[index] == 0);
      l2table[index] = paddr | MMU_L2_UDATAFLAGS;
      brkaddr += MM_PGSIZE;

      /* Make sure that the modified L2 table is flushed to physical
       * memory.
       */

      up_flush_dcache((uintptr_t)&l2table[index],
                      (uintptr_t)&l2table[index] + sizeof(uint32_t));

#ifndef CONFIG_ARCH_PGPOOL_MAPPING
      /* Restore the scratch L1 page table entry */

      mmu_l1_restore(ARCH_SCRATCH_VBASE, l1save);
#endif
      leave_critical_section(flags);
    }

  return brkaddr;
}

#endif /* CONFIG_BUILD_KERNEL */
