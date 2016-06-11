/****************************************************************************
 * arch/arm/src/armv7/arm_addrenv_kstack.c
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
 *   up_addrenv_kstackalloc  - Create a stack in the kernel address environment
 *   up_addrenv_kstackfree   - Destroy the kernel stack.
 *   up_addrenv_vkstack      - Return the base address of the kernel stack
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/sched.h>
#include <nuttx/kmalloc.h>
#include <nuttx/addrenv.h>
#include <nuttx/arch.h>

#include "addrenv.h"

#if defined(CONFIG_ARCH_ADDRENV) && defined(CONFIG_ARCH_KERNEL_STACK)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_addrenv_kstackalloc
 *
 * Description:
 *   This function is called when a new thread is created to allocate
 *   the new thread's kernel stack.   This function may be called for certain
 *   terminating threads which have no kernel stack.  It must be tolerant of
 *   that case.
 *
 * Input Parameters:
 *   tcb - The TCB of the thread that requires the kernel stack.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_addrenv_kstackalloc(FAR struct tcb_s *tcb)
{
  binfo("tcb=%p stacksize=%u\n", tcb, ARCH_KERNEL_STACKSIZE);

  DEBUGASSERT(tcb && tcb->xcp.kstack == 0);

  /* Allocate the kernel stack */

  tcb->xcp.kstack = (FAR uint32_t *)kmm_memalign(8, ARCH_KERNEL_STACKSIZE);
  if (!tcb->xcp.kstack)
    {
      berr("ERROR: Failed to allocate the kernel stack\n");
      return -ENOMEM;
    }

  return OK;
}

/****************************************************************************
 * Name: up_addrenv_kstackfree
 *
 * Description:
 *   This function is called when any thread exits.  This function frees
 *   the kernel stack.
 *
 * Input Parameters:
 *   tcb - The TCB of the thread that no longer requires the kernel stack.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_addrenv_kstackfree(FAR struct tcb_s *tcb)
{
  binfo("tcb=%p\n", tcb);
  DEBUGASSERT(tcb);

  /* Does the exiting thread have a kernel stack? */

  if (tcb->xcp.kstack)
    {
      /* Yes.. Free the kernel stack */

      kmm_free(tcb->xcp.kstack);
      tcb->xcp.kstack = NULL;
    }

  return OK;
}

#endif /* CONFIG_ARCH_ADDRENV && CONFIG_ARCH_KERNEL_STACK */
