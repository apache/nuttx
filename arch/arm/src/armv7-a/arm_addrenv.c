/****************************************************************************
 * arch/arm/src/armv7-a/arm_addrenv.c
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
 * Address Environment Interfaces
 *
 * Low-level interfaces used in binfmt/ to instantiate tasks with address
 * environments.  These interfaces all operate on type arch_addrenv_t which
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
 *   up_addrenv_clone    - Copy an address environment from one location to
 *                         another.
 *
 * Higher-level interfaces used by the tasking logic.  These interfaces are
 * used by the functions in sched/ and all operate on the thread which whose
 * group been assigned an address environment by up_addrenv_clone().
 *
 *   up_addrenv_attach   - Clone the address environment assigned to one TCB
 *                         to another.  This operation is done when a pthread
 *                         is created that share's the same group address
 *                         environment.
 *   up_addrenv_detach   - Release the thread's reference to an address
 *                         environment when a task/thread exits.
 *
 * CONFIG_ARCH_STACK_DYNAMIC=y indicates that the user process stack resides
 * in its own address space.  This options is also *required* if
 * CONFIG_BUILD_KERNEL and CONFIG_LIBC_EXECFUNCS are selected.  Why?
 * Because the caller's stack must be preserved in its own address space
 * when we instantiate the environment of the new process in order to
 * initialize it.
 *
 * NOTE: The naming of the CONFIG_ARCH_STACK_DYNAMIC selection implies that
 * dynamic stack allocation is supported.  Certainly this option must be set
 * if dynamic stack allocation is supported by a platform.  But the more
 * general meaning of this configuration environment is simply that the
 * stack has its own address space.
 *
 * If CONFIG_ARCH_STACK_DYNAMIC=y is selected then the platform specific
 * code must export these additional interfaces:
 *
 *   up_addrenv_ustackalloc  - Create a stack address environment
 *   up_addrenv_ustackfree   - Destroy a stack address environment.
 *   up_addrenv_vustack      - Returns the virtual base address of the stack
 *   up_addrenv_ustackselect - Instantiate a stack address environment
 *
 * If CONFIG_ARCH_KERNEL_STACK is selected, then each user process will have
 * two stacks:  (1) a large (and possibly dynamic) user stack and (2) a
 * smaller kernel stack.  However, this option is *required* if both
 * CONFIG_BUILD_KERNEL and CONFIG_LIBC_EXECFUNCS are selected.  Why?  Because
 * when we instantiate and initialize the address environment of the new
 * user process, we will temporarily lose the address environment of the old
 * user process, including its stack contents.  The kernel C logic will crash
 * immediately with no valid stack in place.
 *
 * If CONFIG_ARCH_KERNEL_STACK=y is selected then the platform specific
 * code must export these additional interfaces:
 *
 *   up_addrenv_kstackalloc  - Create a stack in the kernel address
 *                             environment
 *   up_addrenv_kstackfree   - Destroy the kernel stack.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <string.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/pgalloc.h>
#include <nuttx/irq.h>
#include <nuttx/cache.h>

#include "pgalloc.h"
#include "mmu.h"
#include "cp15_cacheops.h"
#include "addrenv.h"

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

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_addrenv_initdata
 *
 * Description:
 *   Initialize the region of memory at the beginning of the .bss/.data
 *   region that is shared between the user process and the kernel.
 *
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
static int up_addrenv_initdata(uintptr_t l2table)
{
  irqstate_t flags;
  uint32_t *virtptr;
  uintptr_t paddr;

  DEBUGASSERT(l2table);
  flags = enter_critical_section();

  /* Get the virtual address corresponding to the physical page table
   * address
   */

  virtptr = (uint32_t *)arm_pgvaddr(l2table);

  /* Invalidate D-Cache so that we read from the physical memory */

  up_invalidate_dcache((uintptr_t)virtptr,
                       (uintptr_t)virtptr + sizeof(uint32_t));

  /* Get the physical address of the first page of .bss/.data */

  paddr = (uintptr_t)(*virtptr) & PTE_SMALL_PADDR_MASK;
  DEBUGASSERT(paddr);

  /* Get the virtual address corresponding to the physical page address */

  virtptr = (uint32_t *)arm_pgvaddr(paddr);

  /* Finally, after of all of that, we can initialize the tiny region at
   * the beginning of .bss/.data by setting it to zero.
   */

  binfo("*** clear .bss/data (virtptr=%p, size=%d)\n",
        virtptr, ARCH_DATA_RESERVE_SIZE);
  memset(virtptr, 0, ARCH_DATA_RESERVE_SIZE);

  /* Make sure that the initialized data is flushed to physical memory. */

  up_flush_dcache((uintptr_t)virtptr,
                  (uintptr_t)virtptr + ARCH_DATA_RESERVE_SIZE);

  leave_critical_section(flags);
  return OK;
}
#endif /* CONFIG_BUILD_KERNEL */

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
                      arch_addrenv_t *addrenv)
{
  int ret;

  binfo("addrenv=%p textsize=%lu datasize=%lu heapsize=%lu\n",
        addrenv,
        (unsigned long)textsize,
        (unsigned long)datasize,
        (unsigned long)heapsize);

  DEBUGASSERT(addrenv);

  /* Initialize the address environment structure to all zeroes */

  memset(addrenv, 0, sizeof(arch_addrenv_t));

  /* Back the allocation up with physical pages and set up the level 2
   * mapping (which of course does nothing until the L2 page table is hooked
   * into the L1 page table).
   */

  /* Allocate .text space pages */

  ret = arm_addrenv_create_region(addrenv->text, ARCH_TEXT_NSECTS,
                                  CONFIG_ARCH_TEXT_VBASE, textsize,
                                  MMU_L2_UTEXTFLAGS);
  if (ret < 0)
    {
      berr("ERROR: Failed to create .text region: %d\n", ret);
      goto errout;
    }

  /* Allocate .bss/.data space pages.  NOTE that a configurable offset is
   * added to the allocated size.  This is matched by the offset that is
   * used when reporting the virtual data address in up_addrenv_vdata().
   */

  ret = arm_addrenv_create_region(addrenv->data, ARCH_DATA_NSECTS,
                                  CONFIG_ARCH_DATA_VBASE,
                                  datasize + ARCH_DATA_RESERVE_SIZE,
                                  MMU_L2_UDATAFLAGS);
  if (ret < 0)
    {
      berr("ERROR: Failed to create .bss/.data region: %d\n", ret);
      goto errout;
    }

#ifdef CONFIG_BUILD_KERNEL
  /* Initialize the shared data are at the beginning of the .bss/.data
   * region.
   */

  ret = up_addrenv_initdata((uintptr_t)addrenv->data[0] &
                            PMD_PTE_PADDR_MASK);
  if (ret < 0)
    {
      berr("ERROR: Failed to initialize .bss/.data region: %d\n", ret);
      goto errout;
    }
#endif

#ifdef CONFIG_BUILD_KERNEL
  /* Allocate heap space pages */

  ret = arm_addrenv_create_region(addrenv->heap, ARCH_HEAP_NSECTS,
                                  CONFIG_ARCH_HEAP_VBASE, heapsize,
                                  MMU_L2_UDATAFLAGS);
  if (ret < 0)
    {
      berr("ERROR: Failed to create heap region: %d\n", ret);
      goto errout;
    }

  /* Save the initial heap size allocated.  This will be needed when
   * the heap data structures are initialized.
   */

  addrenv->heapsize = (size_t)ret << MM_PGSHIFT;
  binfo("addrenv->heapsize=%d\n", addrenv->heapsize);
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

int up_addrenv_destroy(arch_addrenv_t *addrenv)
{
  binfo("addrenv=%p\n", addrenv);
  DEBUGASSERT(addrenv);

  /* Destroy the .text region */

  arm_addrenv_destroy_region(addrenv->text, ARCH_TEXT_NSECTS,
                             CONFIG_ARCH_TEXT_VBASE, false);

  /* Destroy the .bss/.data region */

  arm_addrenv_destroy_region(addrenv->data, ARCH_DATA_NSECTS,
                             CONFIG_ARCH_DATA_VBASE, false);

#ifdef CONFIG_BUILD_KERNEL
  /* Destroy the heap region */

  arm_addrenv_destroy_region(addrenv->heap, ARCH_HEAP_NSECTS,
                             CONFIG_ARCH_HEAP_VBASE, false);
#ifdef CONFIG_ARCH_VMA_MAPPING
  /* Destroy the shared memory region (without freeing the physical page
   * data).
   */

  arm_addrenv_destroy_region(addrenv->heap, ARCH_SHM_NSECTS,
                             CONFIG_ARCH_SHM_VBASE, true);
#endif
#endif

  memset(addrenv, 0, sizeof(arch_addrenv_t));
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

int up_addrenv_vtext(arch_addrenv_t *addrenv, void **vtext)
{
  binfo("return=%p\n", (void *)CONFIG_ARCH_TEXT_VBASE);

  /* Not much to do in this case */

  DEBUGASSERT(addrenv && vtext);
  *vtext = (void *)CONFIG_ARCH_TEXT_VBASE;
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

int up_addrenv_vdata(arch_addrenv_t *addrenv, uintptr_t textsize,
                     void **vdata)
{
  binfo("return=%p\n",
        (void *)(CONFIG_ARCH_DATA_VBASE + ARCH_DATA_RESERVE_SIZE));

  /* Not much to do in this case */

  DEBUGASSERT(addrenv && vdata);
  *vdata = (void *)(CONFIG_ARCH_DATA_VBASE + ARCH_DATA_RESERVE_SIZE);
  return OK;
}

/****************************************************************************
 * Name: up_addrenv_vheap
 *
 * Description:
 *   Return the heap virtual address associated with the newly created
 *   address environment.  This function is used by the binary loaders in
 *   order get an address that can be used to initialize the new task.
 *
 * Input Parameters:
 *   addrenv - The representation of the task address environment previously
 *      returned by up_addrenv_create.
 *   vheap - The location to return the virtual address.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int up_addrenv_vheap(const arch_addrenv_t *addrenv, void **vheap)
{
  DEBUGASSERT(addrenv && vheap);
  *vheap = (void *)CONFIG_ARCH_HEAP_VBASE;
  return OK;
}
#endif

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
ssize_t up_addrenv_heapsize(const arch_addrenv_t *addrenv)
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
 *   up_addrenv_create().  This function may be called to instantiate
 *   that address environment in the virtual address space.  this might be
 *   necessary, for example, to load the code for the task group from a file
 *   or to access address environment private data.
 *
 * Input Parameters:
 *   addrenv - The representation of the task address environment previously
 *     returned by up_addrenv_create.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_addrenv_select(const arch_addrenv_t *addrenv)
{
  uintptr_t vaddr;
  uintptr_t paddr;
  int i;

  binfo("addrenv=%p\n", addrenv);
  DEBUGASSERT(addrenv);

  for (vaddr = CONFIG_ARCH_TEXT_VBASE, i = 0;
       i < ARCH_TEXT_NSECTS;
       vaddr += SECTION_SIZE, i++)
    {
      /* Set (or clear) the new page table entry */

      paddr = (uintptr_t)addrenv->text[i];
      if (paddr)
        {
          binfo("text: set l1 entry (paddr=%x vaddr=%x)\n", paddr, vaddr);
          mmu_l1_setentry(paddr, vaddr, MMU_L1_PGTABFLAGS);
        }
      else
        {
          binfo("text: clear l1 (vaddr=%x)\n", vaddr);
          mmu_l1_clrentry(vaddr);
        }
    }

  for (vaddr = CONFIG_ARCH_DATA_VBASE, i = 0;
       i < ARCH_DATA_NSECTS;
       vaddr += SECTION_SIZE, i++)
    {
      /* Set (or clear) the new page table entry */

      paddr = (uintptr_t)addrenv->data[i];
      if (paddr)
        {
          binfo("data: set l1 entry (paddr=%x vaddr=%x)\n", paddr, vaddr);
          mmu_l1_setentry(paddr, vaddr, MMU_L1_PGTABFLAGS);
        }
      else
        {
          binfo("data: clear l1 (vaddr=%x)\n", vaddr);
          mmu_l1_clrentry(vaddr);
        }
    }

#ifdef CONFIG_BUILD_KERNEL
  for (vaddr = CONFIG_ARCH_HEAP_VBASE, i = 0;
       i < ARCH_HEAP_NSECTS;
       vaddr += SECTION_SIZE, i++)
    {
      /* Set (or clear) the new page table entry */

      paddr = (uintptr_t)addrenv->heap[i];
      if (paddr)
        {
          binfo("heap: set l1 entry (paddr=%x vaddr=%x)\n", paddr, vaddr);
          mmu_l1_setentry(paddr, vaddr, MMU_L1_PGTABFLAGS);
        }
      else
        {
          binfo("data: clear l1 (vaddr=%x)\n", vaddr);
          mmu_l1_clrentry(vaddr);
        }
    }

#ifdef CONFIG_ARCH_VMA_MAPPING
  for (vaddr = CONFIG_ARCH_SHM_VBASE, i = 0;
       i < ARCH_SHM_NSECTS;
       vaddr += SECTION_SIZE, i++)
    {
      /* Set (or clear) the new page table entry */

      paddr = (uintptr_t)addrenv->shm[i];
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

int up_addrenv_coherent(const arch_addrenv_t *addrenv)
{
  DEBUGASSERT(addrenv);

  /* Invalidate I-Cache */

  up_invalidate_icache_all();

  /* Clean D-Cache in each region.
   * REVISIT:  Cause crashes when trying to clean unmapped memory.  In order
   * for this to work, we need to know the exact size of each region (as we
   * do now for the heap region).
   */

#warning REVISIT... causes crashes
#if 0
  up_clean_dcache(CONFIG_ARCH_TEXT_VBASE,
                  CONFIG_ARCH_TEXT_VBASE +
                  CONFIG_ARCH_TEXT_NPAGES * MM_PGSIZE - 1);

  up_clean_dcache(CONFIG_ARCH_DATA_VBASE,
                  CONFIG_ARCH_DATA_VBASE +
                  CONFIG_ARCH_DATA_NPAGES * MM_PGSIZE - 1);
#endif

#ifdef CONFIG_BUILD_KERNEL
  up_clean_dcache(CONFIG_ARCH_HEAP_VBASE,
                  CONFIG_ARCH_HEAP_VBASE + addrenv->heapsize);
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

int up_addrenv_clone(const arch_addrenv_t *src,
                     arch_addrenv_t *dest)
{
  binfo("src=%p dest=%p\n", src, dest);
  DEBUGASSERT(src && dest);

  /* Just copy the address environment from the source to the destination */

  memcpy(dest, src, sizeof(arch_addrenv_t));
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
 * Input Parameters:
 *   ptcb  - The tcb of the parent task.
 *   tcb   - The tcb of the thread needing the address environment.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_addrenv_attach(struct tcb_s *ptcb, struct tcb_s *tcb)
{
  binfo("parent=%p tcb=%p\n", ptcb, tcb);

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
 * Input Parameters:
 *   tcb - The TCB of the task or thread whose the address environment will
 *     be released.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_addrenv_detach(struct tcb_s *tcb)
{
  binfo("tcb=%p\n", tcb);

  /* Nothing needs to be done in this implementation */

  return OK;
}

#endif /* CONFIG_ARCH_ADDRENV */
