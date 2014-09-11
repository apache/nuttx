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
 * Address Environment Interfaces
 *
 * Low-level interfaces used in binfmt/ to instantiate tasks with address
 * environments.  These interfaces all operate on type group_addrenv_t which
 * is an abstract representation of a task group's address environment and
 * must be defined in arch/arch.h if CONFIG_ARCH_ADDRENV is defined.
 *
 *   up_addrenv_create   - Create an address environment
 *   up_addrenv_destroy  - Destroy an address environment.
 *   up_addrenv_vtext    - Returns the virtual base address of the .text
 *                         address environment
 *   up_addrenv_vdata    - Returns the virtual base address of the .bss/.data
 *                         address environment
 *   up_addrenv_heapsize - Returns the size of the initial heap allocation.
 *   up_addrenv_select   - Instantiate an address environment
 *   up_addrenv_restore  - Restore an address environment
 *   up_addrenv_clone    - Copy an address environment from one location to
 *                         another.
 *
 * Higher-level interfaces used by the tasking logic.  These interfaces are
 * used by the functions in sched/ and all operate on the thread which whose
 * group been assigned an address environment by up_addrenv_clone().
 *
 *   up_addrenv_attach   - Clone the address environment assigned to one TCB
 *                         to another.  This operation is done when a pthread
 *                         is created that share's the same address
 *                         environment.
 *   up_addrenv_detach   - Release the threads reference to an address
 *                         environment when a task/thread exits.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/addrenv.h>

#include <arch/arch.h>
#include <arch/irq.h>

#include "cache.h"
#include "mmu.h"
#include "pginline.h"

#ifdef CONFIG_ARCH_ADDRENV

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration */

#if (CONFIG_ARCH_TEXT_VBASE & SECTION_MASK) != 0
#  error CONFIG_ARCH_TEXT_VBASE not aligned to section boundary
#endif

#if (CONFIG_ARCH_DATA_VBASE & SECTION_MASK) != 0
#  error CONFIG_ARCH_DATA_VBASE not aligned to section boundary
#endif

#if (CONFIG_ARCH_HEAP_VBASE & SECTION_MASK) != 0
#  error CONFIG_ARCH_HEAP_VBASE not aligned to section boundary
#endif

#if (CONFIG_ARCH_STACK_VBASE & SECTION_MASK) != 0
#  error CONFIG_ARCH_STACK_VBASE not aligned to section boundary
#endif

/* Using a 4KiB page size, each 1MiB section maps to a PTE containing
 * 256*2KiB entries
 */

#define ENTRIES_PER_L2TABLE 256

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: set_l2_entry
 *
 * Description:
 *   Set the L2 table entry as part of the initialization of the L2 Page
 *   table.
 *
 ****************************************************************************/

static void set_l2_entry(FAR uint32_t *l2table, uintptr_t paddr,
                         uintptr_t vaddr, uint32_t mmuflags)
{
  uint32_t index;

  /* The table divides a 1Mb address space up into 256 entries, each
   * corresponding to 4Kb of address space.  The page table index is
   * related to the offset from the beginning of 1Mb region.
   */

  index = (vaddr & 0x000ff000) >> 12;

  /* Save the table entry */

  l2table[index] = (paddr | mmuflags);
}

/****************************************************************************
 * Name: up_addrenv_create_region
 *
 * Description:
 *   Create one memory region.
 *
 * Returned Value:
 *   On success, the number of pages allocated is returned.  Otherwise, a
 *   negated errno value is returned.
 *
 ****************************************************************************/

static int up_addrenv_create_region(FAR uintptr_t **list,
                                    unsigned int listlen, uintptr_t vaddr,
                                    size_t regionsize, uint32_t mmuflags)
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

  bvdbg("listlen=%d vaddr=%08lx regionsize=%ld, mmuflags=%08x\n",
        listlen, (unsigned long)vaddr, (unsigned long)regionsize,
        (unsigned int)mmuflags);

  /* Verify that we are configured with enough virtual address space to
   * support this memory region.
   *
   *   npages pages correspondes to (npages << MM_PGSHIFT) bytes
   *   listlen sections corresponds to (listlen << 20) bytes
   */

  npages = MM_NPAGES(regionsize);
  if (npages > (listlen << (20 - MM_PGSHIFT)))
    {
      bdbg("ERROR: npages=%u listlen=%u\n", npages, listlen);
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

      flags = irqsave();

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
              irqrestore(flags);
              return -ENOMEM;
            }

          /* Map the .text region virtual address to this physical address */

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
      irqrestore(flags);
    }

  return npages;
}

/****************************************************************************
 * Name: up_addrenv_initdata
 *
 * Description:
 *   Initialize the region of memory at the the beginning of the .bss/.data
 *   region that is shared between the user process and the kernel.
 *
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
static int up_addrenv_initdata(uintptr_t l2table)
{
  irqstate_t flags;
  FAR uint32_t *virtptr;
  uintptr_t paddr;
#ifndef CONFIG_ARCH_PGPOOL_MAPPING
  uint32_t l1save;
#endif

  DEBUGASSERT(l2table);
  flags = irqsave();

#ifdef CONFIG_ARCH_PGPOOL_MAPPING
  /* Get the virtual address corresponding to the physical page table address */

  virtptr = (FAR uint32_t *)arm_pgvaddr(l2table);
#else
  /* Temporarily map the page into the virtual address space */

  l1save = mmu_l1_getentry(ARCH_SCRATCH_VBASE);
  mmu_l1_setentry(l2table & ~SECTION_MASK, ARCH_SCRATCH_VBASE, MMU_MEMFLAGS);
  virtptr = (FAR uint32_t *)(ARCH_SCRATCH_VBASE | (l2table & SECTION_MASK));
#endif

  /* Invalidate D-Cache so that we read from the physical memory */

  arch_invalidate_dcache((uintptr_t)virtptr,
                         (uintptr_t)virtptr + sizeof(uint32_t));

  /* Get the physical address of the first page of of .bss/.data */

  paddr = (uintptr_t)(*virtptr) & PTE_SMALL_PADDR_MASK;
  DEBUGASSERT(paddr);

#ifdef CONFIG_ARCH_PGPOOL_MAPPING
  /* Get the virtual address corresponding to the physical page address */

  virtptr = (FAR uint32_t *)arm_pgvaddr(paddr);
#else
  /* Temporarily map the page into the virtual address space */

  mmu_l1_setentry(paddr & ~SECTION_MASK, ARCH_SCRATCH_VBASE, MMU_MEMFLAGS);
  virtptr = (FAR uint32_t *)(ARCH_SCRATCH_VBASE | (paddr & SECTION_MASK));
#endif

  /* Finally, after of all of that, we can initialize the tiny region at
   * the beginning of .bss/.data by setting it to zero.
   */

  memset(virtptr, 0, ARCH_DATA_RESERVE_SIZE);

  /* Make sure that the initialized data is flushed to physical memory. */

  arch_flush_dcache((uintptr_t)virtptr,
                    (uintptr_t)virtptr + ARCH_DATA_RESERVE_SIZE);

#ifndef CONFIG_ARCH_PGPOOL_MAPPING
  /* Restore the scratch section L1 page table entry */

  mmu_l1_restore(ARCH_SCRATCH_VBASE, l1save);
#endif
  irqrestore(flags);
  return OK;
}
#endif /* CONFIG_BUILD_KERNEL */

/****************************************************************************
 * Name: up_addrenv_destroy_region
 *
 * Description:
 *   Destroy one memory region.
 *
 ****************************************************************************/

static void up_addrenv_destroy_region(FAR uintptr_t **list,
                                      unsigned int listlen, uintptr_t vaddr)
{
  irqstate_t flags;
  uintptr_t paddr;
  FAR uint32_t *l2table;
#ifndef CONFIG_ARCH_PGPOOL_MAPPING
  uint32_t l1save;
#endif
  int i;
  int j;

  bvdbg("listlen=%d vaddr=%08lx\n", listlen, (unsigned long)vaddr);

  for (i = 0; i < listlen; vaddr += SECTION_SIZE, list++, i++)
    {
      /* Unhook the L2 page table from the L1 page table */

      mmu_l1_clrentry(vaddr);

      /* Has this page table been allocated? */

      paddr = (uintptr_t)list[i];
      if (paddr != 0)
        {
          flags = irqsave();

#ifdef CONFIG_ARCH_PGPOOL_MAPPING
          /* Get the virtual address corresponding to the physical page address */

          l2table = (FAR uint32_t *)arm_pgvaddr(paddr);
#else
          /* Temporarily map the page into the virtual address space */

          l1save = mmu_l1_getentry(ARCH_SCRATCH_VBASE);
          mmu_l1_setentry(paddr & ~SECTION_MASK, ARCH_SCRATCH_VBASE, MMU_MEMFLAGS);
          l2table = (FAR uint32_t *)(ARCH_SCRATCH_VBASE | (paddr & SECTION_MASK));
#endif

          /* Return the allocated pages to the page allocator */

          for (j = 0; j < ENTRIES_PER_L2TABLE; j++)
            {
              paddr = *l2table++;
              if (paddr != 0)
                {
                  paddr &= PTE_SMALL_PADDR_MASK;
                  mm_pgfree(paddr, 1);
                }
            }

#ifndef CONFIG_ARCH_PGPOOL_MAPPING
          /* Restore the scratch section L1 page table entry */

          mmu_l1_restore(ARCH_SCRATCH_VBASE, l1save);
#endif
          irqrestore(flags);

          /* And free the L2 page table itself */

          mm_pgfree((uintptr_t)list[i], 1);
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_addrenv_create
 *
 * Description:
 *   This function is called when a new task is created in order to
 *   instantiate an address environment for the new task group.
 *   up_addrenv_create() is essentially the allocator of the physical
 *   memory for the new task.
 *
 * Input Parameters:
 *   textsize - The size (in bytes) of the .text address environment needed
 *     by the task.  This region may be read/execute only.
 *   datasize - The size (in bytes) of the .data/.bss address environment
 *     needed by the task.  This region may be read/write only.  NOTE: The
 *     actual size of the data region that is allocated will include a
 *     OS private reserved region at the beginning.  The size of the
 *     private, reserved region is give by ARCH_DATA_RESERVE_SIZE.
 *   heapsize - The initial size (in bytes) of the heap address environment
 *     needed by the task.  This region may be read/write only.
 *   addrenv - The location to return the representation of the task address
 *     environment.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_addrenv_create(size_t textsize, size_t datasize, size_t heapsize,
                      FAR group_addrenv_t *addrenv)
{
  int ret;

  bvdbg("addrenv=%p textsize=%lu datasize=%lu\n",
        addrenv, (unsigned long)textsize, (unsigned long)datasize);

  DEBUGASSERT(addrenv);

  /* Initialize the address environment structure to all zeroes */

  memset(addrenv, 0, sizeof(group_addrenv_t));

  /* Back the allocation up with physical pages and set up the level mapping
   * (which of course does nothing until the L2 page table is hooked into
   * the L1 page table).
   */

  /* Allocate .text space pages */

  ret = up_addrenv_create_region(addrenv->text, ARCH_TEXT_NSECTS,
                                 CONFIG_ARCH_TEXT_VBASE, textsize,
                                 MMU_L2_UTEXTFLAGS);
  if (ret < 0)
    {
      bdbg("ERROR: Failed to create .text region: %d\n", ret);
      goto errout;
    }

  /* Allocate .bss/.data space pages.  NOTE that a configurable offset is
   * added to the allocted size.  This is matched by the offset that is
   * used when reporting the virtual data address in up_addrenv_vdata().
   */

  ret = up_addrenv_create_region(addrenv->data, ARCH_DATA_NSECTS,
                                 CONFIG_ARCH_DATA_VBASE,
                                 datasize + ARCH_DATA_RESERVE_SIZE,
                                 MMU_L2_UDATAFLAGS);
  if (ret < 0)
    {
      bdbg("ERROR: Failed to create .bss/.data region: %d\n", ret);
      goto errout;
    }

#ifdef CONFIG_BUILD_KERNEL
  /* Initialize the shared data are at the beginning of the .bss/.data
   * region.
   */

  ret = up_addrenv_initdata((uintptr_t)addrenv->data[0] & PMD_PTE_PADDR_MASK);
  if (ret < 0)
    {
      bdbg("ERROR: Failed to initialize .bss/.data region: %d\n", ret);
      goto errout;
    }
#endif

#ifdef CONFIG_BUILD_KERNEL
  /* Allocate heap space pages */

  ret = up_addrenv_create_region(addrenv->heap, ARCH_HEAP_NSECTS,
                                 CONFIG_ARCH_HEAP_VBASE, heapsize,
                                 MMU_L2_UDATAFLAGS);
  if (ret < 0)
    {
      bdbg("ERROR: Failed to create heap region: %d\n", ret);
      goto errout;
    }

  /* Save the initial heap size allocated.  This will be needed when
   * the heap data structures are initialized.
   */

  addrenv->heapsize = (size_t)ret << MM_PGSHIFT;
#endif
  return OK;

errout:
  up_addrenv_destroy(addrenv);
  return ret;
}

/****************************************************************************
 * Name: up_addrenv_destroy
 *
 * Description:
 *   This function is called when a final thread leaves the task group and
 *   the task group is destroyed.  This function then destroys the defunct
 *   address environment, releasing the underlying physical memory.
 *
 * Input Parameters:
 *   addrenv - The address environment to be destroyed.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_addrenv_destroy(FAR group_addrenv_t *addrenv)
{
  bvdbg("addrenv=%p\n", addrenv);
  DEBUGASSERT(addrenv);

  /* Destroy the .text region */

  up_addrenv_destroy_region(addrenv->text, ARCH_TEXT_NSECTS,
                            CONFIG_ARCH_TEXT_VBASE);

  /* Destroy the .bss/.data region */

  up_addrenv_destroy_region(addrenv->data, ARCH_DATA_NSECTS,
                            CONFIG_ARCH_DATA_VBASE);

#ifdef CONFIG_BUILD_KERNEL
  /* Destroy the heap region */

  up_addrenv_destroy_region(addrenv->heap, ARCH_HEAP_NSECTS,
                            CONFIG_ARCH_HEAP_VBASE);

  memset(addrenv, 0, sizeof(group_addrenv_t));
#endif
  return OK;
}

/****************************************************************************
 * Name: up_addrenv_vtext
 *
 * Description:
 *   Return the virtual address associated with the newly create .text
 *   address environment.  This function is used by the binary loaders in
 *   order get an address that can be used to initialize the new task.
 *
 * Input Parameters:
 *   addrenv - The representation of the task address environment previously
 *      returned by up_addrenv_create.
 *   vtext - The location to return the virtual address.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_addrenv_vtext(FAR group_addrenv_t *addrenv, FAR void **vtext)
{
  bvdbg("return=%p\n", (FAR void *)CONFIG_ARCH_TEXT_VBASE);

  /* Not much to do in this case */

  DEBUGASSERT(addrenv && vtext);
  *vtext = (FAR void *)CONFIG_ARCH_TEXT_VBASE;
  return OK;
}

/****************************************************************************
 * Name: up_addrenv_vdata
 *
 * Description:
 *   Return the virtual address associated with the newly create .text
 *   address environment.  This function is used by the binary loaders in
 *   order get an address that can be used to initialize the new task.
 *
 * Input Parameters:
 *   addrenv - The representation of the task address environment previously
 *      returned by up_addrenv_create.
 *   textsize - For some implementations, the text and data will be saved
 *      in the same memory region (read/write/execute) and, in this case,
 *      the virtual address of the data just lies at this offset into the
 *      common region.
 *   vdata - The location to return the virtual address.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_addrenv_vdata(FAR group_addrenv_t *addrenv, uintptr_t textsize,
                     FAR void **vdata)
{
  bvdbg("return=%p\n", (FAR void *)CONFIG_ARCH_DATA_VBASE);
  /* Not much to do in this case */

  DEBUGASSERT(addrenv && vdata);
  *vdata = (FAR void *)(CONFIG_ARCH_DATA_VBASE + ARCH_DATA_RESERVE_SIZE);
  return OK;
}

/****************************************************************************
 * Name: up_addrenv_heapsize
 *
 * Description:
 *   Return the initial heap allocation size.  That is the amount of memory
 *   allocated by up_addrenv_create() when the heap memory region was first
 *   created.  This may or may not differ from the heapsize parameter that
 *   was passed to up_addrenv_create()
 *
 * Input Parameters:
 *   addrenv - The representation of the task address environment previously
 *     returned by up_addrenv_create.
 *
 * Returned Value:
 *   The initial heap size allocated is returned on success; a negated
 *   errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
ssize_t up_addrenv_heapsize(FAR const group_addrenv_t *addrenv)
{
  DEBUGASSERT(addrenv);
  return (ssize_t)addrenv->heapsize;
}
#endif

/****************************************************************************
 * Name: up_addrenv_select
 *
 * Description:
 *   After an address environment has been established for a task group (via
 *   up_addrenv_create().  This function may be called to to instantiate
 *   that address environment in the virtual address space.  this might be
 *   necessary, for example, to load the code for the task group from a file or
 *   to access address environment private data.
 *
 * Input Parameters:
 *   addrenv - The representation of the task address environment previously
 *     returned by up_addrenv_create.
 *   oldenv
 *     The address environment that was in place before up_addrenv_select().
 *     This may be used with up_addrenv_restore() to restore the original
 *     address environment that was in place before up_addrenv_select() was
 *     called.  Note that this may be a task agnostic, platform-specific
 *     representation that may or may not be different from group_addrenv_t.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_addrenv_select(FAR const group_addrenv_t *addrenv,
                      FAR save_addrenv_t *oldenv)
{
  uintptr_t vaddr;
  uintptr_t paddr;
  int i;

  DEBUGASSERT(addrenv);

  for (vaddr = CONFIG_ARCH_TEXT_VBASE, i = 0;
       i < ARCH_TEXT_NSECTS;
       vaddr += SECTION_SIZE, i++)
    {
      /* Save the old L1 page table entry */

      if (oldenv)
        {
          oldenv->text[i] = mmu_l1_getentry(vaddr);
        }

      /* Set (or clear) the new page table entry */

      paddr = (uintptr_t)addrenv->text[i];
      if (paddr)
        {
          mmu_l1_setentry(paddr, vaddr, MMU_L1_PGTABFLAGS);
        }
      else
        {
          mmu_l1_clrentry(vaddr);
        }
    }

  for (vaddr = CONFIG_ARCH_DATA_VBASE, i = 0;
       i < ARCH_DATA_NSECTS;
       vaddr += SECTION_SIZE, i++)
    {
      /* Save the old L1 page table entry */

      if (oldenv)
        {
          oldenv->data[i] = mmu_l1_getentry(vaddr);
        }

      /* Set (or clear) the new page table entry */

      paddr = (uintptr_t)addrenv->data[i];
      if (paddr)
        {
          mmu_l1_setentry(paddr, vaddr, MMU_L1_PGTABFLAGS);
        }
      else
        {
          mmu_l1_clrentry(vaddr);
        }
    }

#ifdef CONFIG_BUILD_KERNEL
  for (vaddr = CONFIG_ARCH_HEAP_VBASE, i = 0;
       i < ARCH_HEAP_NSECTS;
       vaddr += SECTION_SIZE, i++)
    {
      /* Save the old L1 page table entry */

      if (oldenv)
        {
          oldenv->heap[i] = mmu_l1_getentry(vaddr);
        }

      /* Set (or clear) the new page table entry */

      paddr = (uintptr_t)addrenv->heap[i];
      if (paddr)
        {
          mmu_l1_setentry(paddr, vaddr, MMU_L1_PGTABFLAGS);
        }
      else
        {
          mmu_l1_clrentry(vaddr);
        }
    }
#endif

  return OK;
}

/****************************************************************************
 * Name: up_addrenv_restore
 *
 * Description:
 *   After an address environment has been temporarily instantiated by
 *   up_addrenv_select(), this function may be called to to restore the
 *   original address environment.
 *
 * Input Parameters:
 *   oldenv - The platform-specific representation of the address environment
 *     previously returned by up_addrenv_select.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_addrenv_restore(FAR const save_addrenv_t *oldenv)
{
  uintptr_t vaddr;
  int i;

  bvdbg("oldenv=%p\n", oldenv);
  DEBUGASSERT(oldenv);

  for (vaddr = CONFIG_ARCH_TEXT_VBASE, i = 0;
       i < ARCH_TEXT_NSECTS;
       vaddr += SECTION_SIZE, i++)
    {
      /* Restore the L1 page table entry */

      mmu_l1_restore(vaddr, oldenv->text[i]);
    }

  for (vaddr = CONFIG_ARCH_DATA_VBASE, i = 0;
       i < ARCH_DATA_NSECTS;
       vaddr += SECTION_SIZE, i++)
    {
      /* Restore the L1 page table entry */

      mmu_l1_restore(vaddr, oldenv->data[i]);
    }

#ifdef CONFIG_BUILD_KERNEL
  for (vaddr = CONFIG_ARCH_HEAP_VBASE, i = 0;
       i < ARCH_HEAP_NSECTS;
       vaddr += SECTION_SIZE, i++)
    {
      /* Restore the L1 page table entry */

      mmu_l1_restore(vaddr, oldenv->heap[i]);
    }
#endif

  return OK;
}

/****************************************************************************
 * Name: up_addrenv_coherent
 *
 * Description:
 *   Flush D-Cache and invalidate I-Cache in preparation for a change in
 *   address environments.  This should immediately precede a call to
 *   up_addrenv_select();
 *
 * Input Parameters:
 *   addrenv - Describes the address environment to be made coherent.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_addrenv_coherent(FAR const group_addrenv_t *addrenv)
{
  DEBUGASSERT(addrenv);

  /* Invalidate I-Cache */

  cp15_invalidate_icache();

  /* Clean D-Cache in each region. */

#warning REVISIT... causes crashes
#if 0
  arch_clean_dcache(CONFIG_ARCH_TEXT_VBASE,
                    CONFIG_ARCH_TEXT_VBASE +
                    CONFIG_ARCH_TEXT_NPAGES * MM_PGSIZE - 1);

  arch_clean_dcache(CONFIG_ARCH_DATA_VBASE,
                    CONFIG_ARCH_DATA_VBASE +
                    CONFIG_ARCH_DATA_NPAGES * MM_PGSIZE - 1);

#if 0 /* Not yet implemented */
  arch_clean_dcache(CONFIG_ARCH_HEAP_VBASE,
                    CONFIG_ARCH_HEAP_VBASE +
                    CONFIG_ARCH_HEAP_NPAGES * MM_PGSIZE - 1);
#endif
#endif

  return OK;
}

/****************************************************************************
 * Name: up_addrenv_clone
 *
 * Description:
 *   Duplicate an address environment.  This does not copy the underlying
 *   memory, only the representation that can be used to instantiate that
 *   memory as an address environment.
 *
 * Input Parameters:
 *   src - The address environment to be copied.
 *   dest - The location to receive the copied address environment.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_addrenv_clone(FAR const group_addrenv_t *src,
                     FAR group_addrenv_t *dest)
{
  bvdbg("src=%p dest=%p\n", src, dest);
  DEBUGASSERT(src && dest);

  /* Just copy the address environment from the source to the destination */

  memcpy(dest, src, sizeof(group_addrenv_t));
  return OK;
}

/****************************************************************************
 * Name: up_addrenv_attach
 *
 * Description:
 *   This function is called from the core scheduler logic when a thread
 *   is created that needs to share the address environment of its task
 *   group.
 *
 *   NOTE: In some platforms, nothing will need to be done in this case.
 *   Simply being a member of the group that has the address environment
 *   may be sufficient.
 *
 * Input Parameters:
 *   group - The task group to which the new thread belongs.
 *   tcb   - The TCB of the thread needing the address environment.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_addrenv_attach(FAR struct task_group_s *group, FAR struct tcb_s *tcb)
{
  bvdbg("group=%p tcb=%p\n", group, tcb);

  /* Nothing needs to be done in this implementation */

  return OK;
}

/****************************************************************************
 * Name: up_addrenv_detach
 *
 * Description:
 *   This function is called when a task or thread exits in order to release
 *   its reference to an address environment.  The address environment,
 *   however, should persist until up_addrenv_destroy() is called when the
 *   task group is itself destroyed.  Any resources unique to this thread
 *   may be destroyed now.
 *
 *   NOTE: In some platforms, nothing will need to be done in this case.
 *   Simply being a member of the group that has the address environment
 *   may be sufficient.
 *
 * Input Parameters:
 *   group - The group to which the thread belonged.
 *   tcb - The TCB of the task or thread whose the address environment will
 *     be released.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_addrenv_detach(FAR struct task_group_s *group, FAR struct tcb_s *tcb)
{
  bvdbg("group=%p tcb=%p\n", group, tcb);

  /* Nothing needs to be done in this implementation */

  return OK;
}

#endif /* CONFIG_ARCH_ADDRENV */
