/****************************************************************************
 * sched/task/task_init.c
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

#include <sys/types.h>
#include <stdint.h>
#include <sched.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/queue.h>
#include <nuttx/sched.h>

#include "sched/sched.h"
#include "environ/environ.h"
#include "group/group.h"
#include "task/task.h"
#include "tls/tls.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxtask_init
 *
 * Description:
 *   This function initializes a Task Control Block (TCB) in preparation for
 *   starting a new thread.  It performs a subset of the functionality of
 *   task_create()
 *
 *   Unlike task_create():
 *     1. Allocate the TCB.  The pre-allocated TCB is passed in argv.
 *     2. Allocate the stack.  The pre-allocated stack is passed in argv.
 *     3. Activate the task. This must be done by calling nxtask_activate().
 *
 *   Certain fields of the pre-allocated TCB may be set to change the
 *   nature of the created task.  For example:
 *
 *     - Task type may be set in the TCB flags to create kernel thread
 *
 * Input Parameters:
 *   tcb        - Address of the new task's TCB
 *   name       - Name of the new task (not used)
 *   priority   - Priority of the new task
 *   stack      - Start of the pre-allocated stack
 *   stack_size - Size (in bytes) of the stack allocated
 *   entry      - Application start point of the new task
 *   argv       - A pointer to an array of input parameters.  The array
 *                should be terminated with a NULL argv[] value. If no
 *                parameters are required, argv may be NULL.
 *
 * Returned Value:
 *   OK on success; negative error value on failure appropriately.  (See
 *   nxtask_setup_scheduler() for possible failure conditions).  On failure,
 *   the caller is responsible for freeing the stack memory and for calling
 *   nxsched_release_tcb() to free the TCB (which could be in most any
 *   state).
 *
 ****************************************************************************/

int nxtask_init(FAR struct task_tcb_s *tcb, const char *name, int priority,
                FAR void *stack, uint32_t stack_size,
                main_t entry, FAR char * const argv[],
                FAR char * const envp[])
{
  uint8_t ttype = tcb->cmn.flags & TCB_FLAG_TTYPE_MASK;
  int ret;

#ifndef CONFIG_DISABLE_PTHREAD
  /* Only tasks and kernel threads can be initialized in this way */

  DEBUGASSERT(tcb && ttype != TCB_FLAG_TTYPE_PTHREAD);
#endif

#ifdef CONFIG_ARCH_ADDRENV
  /* Allocate address environment for the task */

  ret = addrenv_allocate(&tcb->cmn, tcb->cmn.flags);
  if (ret < 0)
    {
      return ret;
    }
#endif

  /* Create a new task group */

  ret = group_allocate(tcb, tcb->cmn.flags);
  if (ret < 0)
    {
      goto errout_with_addrenv;
    }

  /* Duplicate the parent tasks environment */

  ret = env_dup(tcb->cmn.group, envp);
  if (ret < 0)
    {
      goto errout_with_group;
    }

  /* Associate file descriptors with the new task */

  ret = group_setuptaskfiles(tcb);
  if (ret < 0)
    {
      goto errout_with_group;
    }

  if (stack)
    {
      /* Use pre-allocated stack */

      ret = up_use_stack(&tcb->cmn, stack, stack_size);
    }
  else
    {
      /* Allocate the stack for the TCB */

      ret = up_create_stack(&tcb->cmn, stack_size, ttype);
    }

  if (ret < OK)
    {
      goto errout_with_group;
    }

  /* Initialize thread local storage */

  ret = tls_init_info(&tcb->cmn);
  if (ret < OK)
    {
      goto errout_with_group;
    }

  /* Initialize the task control block */

  ret = nxtask_setup_scheduler(tcb, priority, nxtask_start,
                               entry, ttype);
  if (ret < OK)
    {
      goto errout_with_group;
    }

  /* Setup to pass parameters to the new task */

  ret = nxtask_setup_arguments(tcb, name, argv);
  if (ret < OK)
    {
      goto errout_with_group;
    }

  /* Now we have enough in place that we can join the group */

  group_initialize(tcb);
  return ret;

errout_with_group:
  if (!stack && tcb->cmn.stack_alloc_ptr)
    {
#ifdef CONFIG_BUILD_KERNEL
      /* If the exiting thread is not a kernel thread, then it has an
       * address environment.  Don't bother to release the stack memory
       * in this case... There is no point since the memory lies in the
       * user memory region that will be destroyed anyway (and the
       * address environment has probably already been destroyed at
       * this point.. so we would crash if we even tried it).  But if
       * this is a privileged group, when we still have to release the
       * memory using the kernel allocator.
       */

      if (ttype == TCB_FLAG_TTYPE_KERNEL)
#endif
        {
          up_release_stack(&tcb->cmn, ttype);
        }
    }

  group_leave(&tcb->cmn);

errout_with_addrenv:
#ifdef CONFIG_ARCH_ADDRENV
  addrenv_free(&tcb->cmn);
#endif
  return ret;
}

/****************************************************************************
 * Name: nxtask_uninit
 *
 * Description:
 *   Undo all operations on a TCB performed by task_init() and release the
 *   TCB by calling kmm_free().  This is intended primarily to support
 *   error recovery operations after a successful call to task_init() such
 *   was when a subsequent call to task_activate fails.
 *
 *   Caution:  Freeing of the TCB itself might be an unexpected side-effect.
 *
 * Input Parameters:
 *   tcb - Address of the TCB initialized by task_init()
 *
 * Returned Value:
 *   OK on success; negative error value on failure appropriately.
 *
 ****************************************************************************/

void nxtask_uninit(FAR struct task_tcb_s *tcb)
{
  /* The TCB was added to the inactive task list by
   * nxtask_setup_scheduler().
   */

  dq_rem((FAR dq_entry_t *)tcb, &g_inactivetasks);

  /* Release all resources associated with the TCB... Including the TCB
   * itself.
   */

  nxsched_release_tcb((FAR struct tcb_s *)tcb,
                      tcb->cmn.flags & TCB_FLAG_TTYPE_MASK);
}
