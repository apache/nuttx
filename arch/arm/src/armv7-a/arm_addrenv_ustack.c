/****************************************************************************
 * arch/arm/src/armv7-a/arm_addrenv_ustack.c
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
 * If CONFIG_ARCH_KERNEL_STACK=y is selected then the platform specific
 * code must export these additional interfaces:
 *
 *   up_addrenv_ustackalloc  - Create a stack address environment
 *   up_addrenv_ustackfree   - Destroy a stack address environment.
 *   up_addrenv_vustack      - Returns the virtual base address of the stack
 *   up_addrenv_ustackselect - Instantiate a stack address environment
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
#include <nuttx/sched.h>
#include <nuttx/addrenv.h>

#include <arch/irq.h>

#include "mmu.h"
#include "addrenv.h"

#if defined(CONFIG_ARCH_ADDRENV) && defined(CONFIG_ARCH_STACK_DYNAMIC)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration */

#if (CONFIG_ARCH_STACK_VBASE & SECTION_MASK) != 0
#  error CONFIG_ARCH_STACK_VBASE not aligned to section boundary
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_addrenv_ustackalloc
 *
 * Description:
 *   This function is called when a new thread is created in order to
 *   instantiate an address environment for the new thread's stack.
 *   up_addrenv_ustackalloc() is essentially the allocator of the physical
 *   memory for the new task's stack.
 *
 * Input Parameters:
 *   tcb - The TCB of the thread that requires the stack address environment.
 *   stacksize - The size (in bytes) of the initial stack address
 *     environment needed by the task.  This region may be read/write only.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_addrenv_ustackalloc(struct tcb_s *tcb, size_t stacksize)
{
  int ret;

  binfo("tcb=%p stacksize=%lu\n", tcb, (unsigned long)stacksize);

  DEBUGASSERT(tcb);

  /* Initialize the address environment list to all zeroes */

  memset(tcb->xcp.ustack, 0, ARCH_STACK_NSECTS * sizeof(uintptr_t *));

  /* Back the allocation up with physical pages and set up the level 2
   * mapping (which of course does nothing until the L2 page table is hooked
   * into the L1 page table).
   */

  /* Allocate .text space pages */

  ret = arm_addrenv_create_region(tcb->xcp.ustack, ARCH_STACK_NSECTS,
                                  CONFIG_ARCH_STACK_VBASE, stacksize,
                                  MMU_L2_UDATAFLAGS);
  if (ret < 0)
    {
      berr("ERROR: Failed to create stack region: %d\n", ret);
      up_addrenv_ustackfree(tcb);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: up_addrenv_ustackfree
 *
 * Description:
 *   This function is called when any thread exits.  This function then
 *   destroys the defunct address environment for the thread's stack,
 *   releasing the underlying physical memory.
 *
 * Input Parameters:
 *   tcb - The TCB of the thread that no longer requires the stack address
 *     environment.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_addrenv_ustackfree(struct tcb_s *tcb)
{
  binfo("tcb=%p\n", tcb);
  DEBUGASSERT(tcb);

  /* Destroy the stack region */

  arm_addrenv_destroy_region(tcb->xcp.ustack, ARCH_STACK_NSECTS,
                             CONFIG_ARCH_STACK_VBASE, false);

  memset(tcb->xcp.ustack, 0, ARCH_STACK_NSECTS * sizeof(uintptr_t *));
  return OK;
}

/****************************************************************************
 * Name: up_addrenv_vustack
 *
 * Description:
 *   Return the virtual address associated with the newly created stack
 *   address environment.
 *
 * Input Parameters:
 *   tcb - The TCB of the thread with the stack address environment of
 *     interest.
 *   vstack - The location to return the stack virtual base address.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_addrenv_vustack(const struct tcb_s *tcb, void **vstack)
{
  binfo("Return=%p\n", (void *)CONFIG_ARCH_STACK_VBASE);

  /* Not much to do in this case */

  DEBUGASSERT(tcb);
  *vstack = (void *)CONFIG_ARCH_STACK_VBASE;
  return OK;
}

/****************************************************************************
 * Name: up_addrenv_ustackselect
 *
 * Description:
 *   After an address environment has been established for a task's stack
 *   (via up_addrenv_ustackalloc().  This function may be called to
 *   instantiate that address environment in the virtual address space.
 *   This is a necessary step before each context switch to the newly created
 *   thread (including the initial thread startup).
 *
 * Input Parameters:
 *   tcb - The TCB of the thread with the stack address environment to be
 *     instantiated.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_addrenv_ustackselect(const struct tcb_s *tcb)
{
  uintptr_t vaddr;
  uintptr_t paddr;
  int i;

  DEBUGASSERT(tcb);

  for (vaddr = CONFIG_ARCH_STACK_VBASE, i = 0;
       i < ARCH_TEXT_NSECTS;
       vaddr += ARCH_STACK_NSECTS, i++)
    {
      /* Set (or clear) the new page table entry */

      paddr = (uintptr_t)tcb->xcp.ustack[i];
      if (paddr)
        {
          mmu_l1_setentry(paddr, vaddr, MMU_L1_PGTABFLAGS);
        }
      else
        {
          mmu_l1_clrentry(vaddr);
        }
    }

  return OK;
}

#endif /* CONFIG_ARCH_ADDRENV && CONFIG_ARCH_STACK_DYNAMIC */
