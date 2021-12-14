/****************************************************************************
 * sched/task/task_vfork.c
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

#include <sys/wait.h>
#include <stdint.h>
#include <sched.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <queue.h>
#include <debug.h>

#include <nuttx/sched.h>
#include <nuttx/tls.h>

#include "sched/sched.h"
#include "group/group.h"
#include "task/task.h"

/* vfork() requires architecture-specific support as well as waipid(). */

#if defined(CONFIG_ARCH_HAVE_VFORK) && defined(CONFIG_SCHED_WAITPID)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxtask_setup_vfork
 *
 * Description:
 *   The vfork() function has the same effect as fork(), except that the
 *   behavior is undefined if the process created by vfork() either modifies
 *   any data other than a variable of type pid_t used to store the return
 *   value from vfork(), or returns from the function in which vfork() was
 *   called, or calls any other function before successfully calling _exit()
 *   or one of the exec family of functions.
 *
 *   This function provides one step in the overall vfork() sequence:  It
 *   Allocates and initializes the child task's TCB.  The overall sequence
 *   is:
 *
 *   1) User code calls vfork().  vfork() is provided in
 *      architecture-specific code.
 *   2) vfork()and calls nxtask_setup_vfork().
 *   3) nxtask_setup_vfork() allocates and configures the child task's TCB.
 *      This consists of:
 *      - Allocation of the child task's TCB.
 *      - Initialization of file descriptors and streams
 *      - Configuration of environment variables
 *      - Allocate and initialize the stack
 *      - Setup the input parameters for the task.
 *      - Initialization of the TCB (including call to up_initial_state())
 *   4) up_vfork() provides any additional operating context. up_vfork must:
 *      - Initialize special values in any CPU registers that were not
 *        already configured by up_initial_state()
 *   5) up_vfork() then calls nxtask_start_vfork()
 *   6) nxtask_start_vfork() then executes the child thread.
 *
 * Input Parameters:
 *   retaddr - Return address
 *   argsize - Location to return the argument size
 *
 * Returned Value:
 *   Upon successful completion, nxtask_setup_vfork() returns a pointer to
 *   newly allocated and initialized child task's TCB.  NULL is returned
 *   on any failure and the errno is set appropriately.
 *
 ****************************************************************************/

FAR struct task_tcb_s *nxtask_setup_vfork(start_t retaddr)
{
  FAR struct tcb_s *ptcb = this_task();
  FAR struct tcb_s *parent;
  FAR struct task_tcb_s *child;
  FAR struct tls_info_s *info;
  size_t stack_size;
  uint8_t ttype;
  int priority;
  int ret;

  DEBUGASSERT(retaddr != NULL);

  /* Get the type of the fork'ed task (kernel or user) */

  if ((ptcb->flags & TCB_FLAG_TTYPE_MASK) == TCB_FLAG_TTYPE_KERNEL)
    {
      /* Fork'ed from a kernel thread */

      ttype = TCB_FLAG_TTYPE_KERNEL;
      parent = ptcb;
    }
  else
    {
      /* Fork'ed from a user task or pthread */

      ttype = TCB_FLAG_TTYPE_TASK;
      if ((ptcb->flags & TCB_FLAG_TTYPE_MASK) == TCB_FLAG_TTYPE_TASK)
        {
          parent = ptcb;
        }
      else
        {
          parent = nxsched_get_tcb(ptcb->group->tg_pid);
          if (parent == NULL)
            {
              ret = -ENOENT;
              goto errout;
            }
        }
    }

  /* Allocate a TCB for the child task. */

  child = (FAR struct task_tcb_s *)kmm_zalloc(sizeof(struct task_tcb_s));
  if (!child)
    {
      serr("ERROR: Failed to allocate TCB\n");
      ret = -ENOMEM;
      goto errout;
    }

  /* Allocate a new task group with the same privileges as the parent */

  ret = group_allocate(child, ttype);
  if (ret < 0)
    {
      goto errout_with_tcb;
    }

  /* Associate file descriptors with the new task */

  ret = group_setuptaskfiles(child);
  if (ret < OK)
    {
      goto errout_with_tcb;
    }

  /* Allocate the stack for the TCB */

  stack_size = (uintptr_t)ptcb->stack_base_ptr -
      (uintptr_t)ptcb->stack_alloc_ptr + ptcb->adj_stack_size;

  ret = up_create_stack(&child->cmn, stack_size, ttype);
  if (ret < OK)
    {
      goto errout_with_tcb;
    }

  /* Setup thread local storage */

  info = up_stack_frame(&child->cmn, up_tls_size());
  if (info == NULL)
    {
      ret = -ENOMEM;
      goto errout_with_tcb;
    }

  DEBUGASSERT(info == child->cmn.stack_alloc_ptr);
  memcpy(info, parent->stack_alloc_ptr, sizeof(struct tls_info_s));
  info->tl_task = child->cmn.group->tg_info;

  up_tls_initialize(info);

  /* Get the priority of the parent task */

#ifdef CONFIG_PRIORITY_INHERITANCE
  priority = ptcb->base_priority;   /* "Normal," unboosted priority */
#else
  priority = ptcb->sched_priority;  /* Current priority */
#endif

  /* Initialize the task control block.  This calls up_initial_state() */

  sinfo("Child priority=%d start=%p\n", priority, retaddr);
  ret = nxtask_setup_scheduler(child, priority, retaddr,
                               ptcb->entry.main, ttype);
  if (ret < OK)
    {
      goto errout_with_tcb;
    }

  /* Setup to pass parameters to the new task */

  nxtask_setup_arguments(child, parent->group->tg_info->argv[0],
                         &parent->group->tg_info->argv[1]);

  /* Now we have enough in place that we can join the group */

  ret = group_initialize(child);
  if (ret < OK)
    {
      goto errout_with_list;
    }

  sinfo("parent=%p, returning child=%p\n", parent, child);
  return child;

errout_with_list:
  dq_rem((FAR dq_entry_t *)child, (FAR dq_queue_t *)&g_inactivetasks);
errout_with_tcb:
  nxsched_release_tcb((FAR struct tcb_s *)child, ttype);
errout:
  set_errno(-ret);
  return NULL;
}

/****************************************************************************
 * Name: nxtask_start_vfork
 *
 * Description:
 *   The vfork() function has the same effect as fork(), except that the
 *   behavior is undefined if the process created by vfork() either modifies
 *   any data other than a variable of type pid_t used to store the return
 *   value from vfork(), or returns from the function in which vfork() was
 *   called, or calls any other function before successfully calling _exit()
 *   or one of the exec family of functions.
 *
 *   This function provides one step in the overall vfork() sequence:  It
 *   starts execution of the previously initialized TCB.  The overall
 *   sequence is:
 *
 *   1) User code calls vfork()
 *   2) Architecture-specific code provides vfork()and calls
 *      nxtask_setup_vfork().
 *   3) nxtask_setup_vfork() allocates and configures the child task's TCB.
 *      This consists of:
 *      - Allocation of the child task's TCB.
 *      - Initialization of file descriptors and streams
 *      - Configuration of environment variables
 *      - Allocate and initialize the stack
 *      - Setup the input parameters for the task.
 *      - Initialization of the TCB (including call to up_initial_state())
 *   4) vfork() provides any additional operating context. vfork must:
 *      - Initialize special values in any CPU registers that were not
 *        already configured by up_initial_state()
 *   5) vfork() then calls nxtask_start_vfork()
 *   6) nxtask_start_vfork() then executes the child thread.
 *
 * Input Parameters:
 *   retaddr - The return address from vfork() where the child task
 *     will be started.
 *
 * Returned Value:
 *   Upon successful completion, vfork() returns 0 to the child process and
 *   returns the process ID of the child process to the parent process.
 *   Otherwise, -1 is returned to the parent, no child process is created,
 *   and errno is set to indicate the error.
 *
 ****************************************************************************/

pid_t nxtask_start_vfork(FAR struct task_tcb_s *child)
{
  pid_t pid;
  int rc = 0;
  int ret;

  sinfo("Starting Child TCB=%p\n", child);
  DEBUGASSERT(child);

  /* Get the assigned pid before we start the task */

  pid = (int)child->cmn.pid;

  /* Eliminate a race condition by disabling pre-emption.  The child task
   * can be instantiated, but cannot run until we call waitpid().  This
   * assures us that we cannot miss the death-of-child signal (only
   * needed in the SMP case).
   */

  sched_lock();

  /* Activate the task */

  nxtask_activate((FAR struct tcb_s *)child);

  /* The child task has not yet ran because pre-emption is disabled.
   * The child task has the same priority as the parent task, so that
   * would typically be the case anyway.  However, in the SMP
   * configuration, the child thread might have already ran on
   * another CPU if pre-emption were not disabled.
   *
   * It is a requirement that the parent environment be stable while
   * vfork runs; the child thread is still dependent on things in the
   * parent thread... like the pointers into parent thread's stack
   * which will still appear in the child's registers and environment.
   *
   * We assure that by waiting for the child thread to exit before
   * returning to the parent thread.  NOTE that pre-emption will be
   * re-enabled while we are waiting, giving the child thread the
   * opportunity to run.
   */

  ret = waitpid(pid, &rc, 0);
  if (ret < 0)
    {
      serr("ERROR: waitpid failed: %d\n", get_errno());
    }

  sched_unlock();
  return pid;
}

/****************************************************************************
 * Name: nxtask_abort_vfork
 *
 * Description:
 *   Recover from any errors after nxtask_setup_vfork() was called.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxtask_abort_vfork(FAR struct task_tcb_s *child, int errcode)
{
  /* The TCB was added to the active task list by nxtask_setup_scheduler() */

  dq_rem((FAR dq_entry_t *)child, (FAR dq_queue_t *)&g_inactivetasks);

  /* Release the TCB */

  nxsched_release_tcb((FAR struct tcb_s *)child,
                      child->cmn.flags & TCB_FLAG_TTYPE_MASK);
  set_errno(errcode);
}

#endif /* CONFIG_ARCH_HAVE_VFORK && CONFIG_SCHED_WAITPID */
