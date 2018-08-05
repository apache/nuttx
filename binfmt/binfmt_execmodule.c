/****************************************************************************
 * binfmt/binfmt_execmodule.c
 *
 *   Copyright (C) 2009, 2013-2014, 2017 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <sched.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mm/mm.h>
#include <nuttx/mm/shm.h>
#include <nuttx/binfmt/binfmt.h>

#include "sched/sched.h"
#include "binfmt.h"

#ifndef CONFIG_BINFMT_DISABLE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* If C++ constructors are used, then CONFIG_SCHED_STARTHOOK must also be
 * selected be the start hook is used to schedule execution of the
 * constructors.
 */

#if defined(CONFIG_BINFMT_CONSTRUCTORS) && !defined(CONFIG_SCHED_STARTHOOK)
#  errror "CONFIG_SCHED_STARTHOOK must be defined to use constructors"
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: exec_ctors
 *
 * Description:
 *   Execute C++ static constructors.  This function is registered as a
 *   start hook and runs on the thread of the newly created task before
 *   the new task's main function is called.
 *
 * Input Parameters:
 *   arg - Argument is instance of load state info structure cast to void *.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

#ifdef CONFIG_BINFMT_CONSTRUCTORS
static void exec_ctors(FAR void *arg)
{
  FAR const struct binary_s *binp = (FAR const struct binary_s *)arg;
  binfmt_ctor_t *ctor = binp->ctors;
  int i;

  /* Execute each constructor */

  for (i = 0; i < binp->nctors; i++)
    {
      binfo("Calling ctor %d at %p\n", i, (FAR void *)ctor);

      (*ctor)();
      ctor++;
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: exec_module
 *
 * Description:
 *   Execute a module that has been loaded into memory by load_module().
 *
 * Returned Value:
 *   This is a NuttX internal function so it follows the convention that
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int exec_module(FAR const struct binary_s *binp)
{
  FAR struct task_tcb_s *tcb;
#if defined(CONFIG_ARCH_ADDRENV) && defined(CONFIG_BUILD_KERNEL)
  save_addrenv_t oldenv;
#endif
  FAR uint32_t *stack;
  pid_t pid;
  int ret;

  /* Sanity checking */

#ifdef CONFIG_DEBUG_FEATURES
  if (!binp || !binp->entrypt || binp->stacksize <= 0)
    {
      return -EINVAL;
    }
#endif

  binfo("Executing %s\n", binp->filename);

  /* Allocate a TCB for the new task. */

  tcb = (FAR struct task_tcb_s *)kmm_zalloc(sizeof(struct task_tcb_s));
  if (!tcb)
    {
      return -ENOMEM;
    }

#if defined(CONFIG_ARCH_ADDRENV) && defined(CONFIG_BUILD_KERNEL)
  /* Instantiate the address environment containing the user heap */

  ret = up_addrenv_select(&binp->addrenv, &oldenv);
  if (ret < 0)
    {
      berr("ERROR: up_addrenv_select() failed: %d\n", ret);
      goto errout_with_tcb;
    }

  /* Initialize the user heap */

  umm_initialize((FAR void *)CONFIG_ARCH_HEAP_VBASE,
                 up_addrenv_heapsize(&binp->addrenv));
#endif

  /* Allocate the stack for the new task.
   *
   * REVISIT:  This allocation is currently always from the user heap.  That
   * will need to change if/when we want to support dynamic stack allocation.
   */

  stack = (FAR uint32_t *)kumm_malloc(binp->stacksize);
  if (!stack)
    {
      ret = -ENOMEM;
      goto errout_with_addrenv;
    }

  /* Initialize the task */

  ret = task_init((FAR struct tcb_s *)tcb, binp->filename, binp->priority,
                  stack, binp->stacksize, binp->entrypt, binp->argv);
  if (ret < 0)
    {
      ret = -get_errno();
      berr("task_init() failed: %d\n", ret);
      goto errout_with_addrenv;
    }

  /* We can free the argument buffer now.
   * REVISIT:  It is good to free up memory as soon as possible, but
   * unfortunately here 'binp' is 'const'.  So to do this properly, we will
   * have to make some more extensive changes.
   */

  binfmt_freeargv((FAR struct binary_s *)binp);

  /* Note that tcb->flags are not modified.  0=normal task */
  /* tcb->flags |= TCB_FLAG_TTYPE_TASK; */

#if defined(CONFIG_ARCH_ADDRENV) && defined(CONFIG_BUILD_KERNEL)
  /* Allocate the kernel stack */

  ret = up_addrenv_kstackalloc(&tcb->cmn);
  if (ret < 0)
    {
      berr("ERROR: up_addrenv_select() failed: %d\n", ret);
      goto errout_with_tcbinit;
    }
#endif

#if defined(CONFIG_BUILD_KERNEL) && defined(CONFIG_MM_SHM)
  /* Initialize the shared memory virtual page allocator */

  ret = shm_group_initialize(tcb->cmn.group);
  if (ret < 0)
    {
      berr("ERROR: shm_group_initialize() failed: %d\n", ret);
      goto errout_with_tcbinit;
    }
#endif

#ifdef CONFIG_PIC
  /* Add the D-Space address as the PIC base address.  By convention, this
   * must be the first allocated address space.
   */

  tcb->cmn.dspace = binp->alloc[0];

  /* Re-initialize the task's initial state to account for the new PIC base */

  up_initial_state(&tcb->cmn);
#endif

#ifdef CONFIG_ARCH_ADDRENV
  /* Assign the address environment to the new task group */

  ret = up_addrenv_clone(&binp->addrenv, &tcb->cmn.group->tg_addrenv);
  if (ret < 0)
    {
      berr("ERROR: up_addrenv_clone() failed: %d\n", ret);
      goto errout_with_tcbinit;
    }

  /* Mark that this group has an address environment */

  tcb->cmn.group->tg_flags |= GROUP_FLAG_ADDRENV;
#endif

#ifdef CONFIG_BINFMT_CONSTRUCTORS
  /* Setup a start hook that will execute all of the C++ static constructors
   * on the newly created thread.  The struct binary_s must persist at least
   * until the new task has been started.
   */

  if (binp->nctors > 0)
    {
      task_starthook(tcb, exec_ctors, (FAR void *)binp);
    }
#endif

  /* Get the assigned pid before we start the task */

  pid = tcb->cmn.pid;

  /* Then activate the task at the provided priority */

  ret = task_activate((FAR struct tcb_s *)tcb);
  if (ret < 0)
    {
      ret = -get_errno();
      berr("task_activate() failed: %d\n", ret);
      goto errout_with_tcbinit;
    }

#if defined(CONFIG_ARCH_ADDRENV) && defined(CONFIG_BUILD_KERNEL)
  /* Restore the address environment of the caller */

  ret = up_addrenv_restore(&oldenv);
  if (ret < 0)
    {
      berr("ERROR: up_addrenv_select() failed: %d\n", ret);
      goto errout_with_tcbinit;
    }
#endif

  return (int)pid;

errout_with_tcbinit:
  tcb->cmn.stack_alloc_ptr = NULL;
  sched_releasetcb(&tcb->cmn, TCB_FLAG_TTYPE_TASK);
  kumm_free(stack);
  return ret;

errout_with_addrenv:
#if defined(CONFIG_ARCH_ADDRENV) && defined(CONFIG_BUILD_KERNEL)
  (void)up_addrenv_restore(&oldenv);

errout_with_tcb:
#endif
  kmm_free(tcb);
  return ret;
}

#endif /* CONFIG_BINFMT_DISABLE */
