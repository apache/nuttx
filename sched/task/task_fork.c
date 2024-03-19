/****************************************************************************
 * sched/task/task_fork.c
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
#include <debug.h>

#include <nuttx/queue.h>

#include "sched/sched.h"
#include "environ/environ.h"
#include "group/group.h"
#include "task/task.h"
#include "tls/tls.h"

/* fork() requires architecture-specific support as well as waipid(). */

#ifdef CONFIG_ARCH_HAVE_FORK

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxtask_setup_fork
 *
 * Description:
 *   The fork() function has the same effect as posix fork(), except that the
 *   behavior is undefined if the process created by fork() either modifies
 *   any data other than a variable of type pid_t used to store the return
 *   value from fork(), or returns from the function in which fork() was
 *   called, or calls any other function before successfully calling _exit()
 *   or one of the exec family of functions.
 *
 *   This function provides one step in the overall fork() sequence:  It
 *   Allocates and initializes the child task's TCB.  The overall sequence
 *   is:
 *
 *   1) User code calls fork().  fork() is provided in
 *      architecture-specific code.
 *   2) fork()and calls nxtask_setup_fork().
 *   3) nxtask_setup_fork() allocates and configures the child task's TCB.
 *      This consists of:
 *      - Allocation of the child task's TCB.
 *      - Initialization of file descriptors and streams
 *      - Configuration of environment variables
 *      - Allocate and initialize the stack
 *      - Setup the input parameters for the task.
 *      - Initialization of the TCB (including call to up_initial_state())
 *   4) up_fork() provides any additional operating context. up_fork must:
 *      - Initialize special values in any CPU registers that were not
 *        already configured by up_initial_state()
 *   5) up_fork() then calls nxtask_start_fork()
 *   6) nxtask_start_fork() then executes the child thread.
 *
 * Input Parameters:
 *   retaddr - Return address
 *   argsize - Location to return the argument size
 *
 * Returned Value:
 *   Upon successful completion, nxtask_setup_fork() returns a pointer to
 *   newly allocated and initialized child task's TCB.  NULL is returned
 *   on any failure and the errno is set appropriately.
 *
 ****************************************************************************/

FAR struct task_tcb_s *nxtask_setup_fork(start_t retaddr)
{
  FAR struct tcb_s *ptcb = this_task();
  FAR struct tcb_s *parent;
  FAR struct task_tcb_s *child;
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

  child = kmm_zalloc(sizeof(struct task_tcb_s));
  if (!child)
    {
      serr("ERROR: Failed to allocate TCB\n");
      ret = -ENOMEM;
      goto errout;
    }

  child->cmn.flags |= TCB_FLAG_FREE_TCB;

  /* Initialize the task join */

  nxtask_joininit(&child->cmn);

  /* Allocate a new task group with the same privileges as the parent */

  ret = group_initialize(child, ttype);
  if (ret < 0)
    {
      goto errout_with_tcb;
    }

  /* Duplicate the parent tasks environment */

  ret = env_dup(child->cmn.group, environ);
  if (ret < 0)
    {
      goto errout_with_tcb;
    }

  /* Associate file descriptors with the new task */

  ret = group_setuptaskfiles(child, NULL, false);
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

  ret = tls_dup_info(&child->cmn, parent);
  if (ret < OK)
    {
      goto errout_with_tcb;
    }

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

  ret = nxtask_setup_arguments(child, parent->group->tg_info->ta_argv[0],
                               &parent->group->tg_info->ta_argv[1]);
  if (ret < OK)
    {
      goto errout_with_tcb;
    }

  /* Now we have enough in place that we can join the group */

  group_postinitialize(child);
  sinfo("parent=%p, returning child=%p\n", parent, child);
  return child;

errout_with_tcb:
  nxsched_release_tcb((FAR struct tcb_s *)child, ttype);
errout:
  set_errno(-ret);
  return NULL;
}

/****************************************************************************
 * Name: nxtask_start_fork
 *
 * Description:
 *   The fork() function has the same effect as fork(), except that the
 *   behavior is undefined if the process created by fork() either modifies
 *   any data other than a variable of type pid_t used to store the return
 *   value from fork(), or returns from the function in which fork() was
 *   called, or calls any other function before successfully calling _exit()
 *   or one of the exec family of functions.
 *
 *   This function provides one step in the overall fork() sequence:  It
 *   starts execution of the previously initialized TCB.  The overall
 *   sequence is:
 *
 *   1) User code calls fork()
 *   2) Architecture-specific code provides fork()and calls
 *      nxtask_setup_fork().
 *   3) nxtask_setup_fork() allocates and configures the child task's TCB.
 *      This consists of:
 *      - Allocation of the child task's TCB.
 *      - Initialization of file descriptors and streams
 *      - Configuration of environment variables
 *      - Allocate and initialize the stack
 *      - Setup the input parameters for the task.
 *      - Initialization of the TCB (including call to up_initial_state())
 *   4) fork() provides any additional operating context. fork must:
 *      - Initialize special values in any CPU registers that were not
 *        already configured by up_initial_state()
 *   5) fork() then calls nxtask_start_fork()
 *   6) nxtask_start_fork() then executes the child thread.
 *
 * Input Parameters:
 *   child - The task_tcb_s struct instance that created by
 *           nxtask_setup_fork() method
 *   wait_child - whether need to wait until the child is running finished
 *
 * Returned Value:
 *   Upon successful completion, fork() returns 0 to the child process and
 *   returns the process ID of the child process to the parent process.
 *   Otherwise, -1 is returned to the parent, no child process is created,
 *   and errno is set to indicate the error.
 *
 ****************************************************************************/

pid_t nxtask_start_fork(FAR struct task_tcb_s *child)
{
  pid_t pid;

  sinfo("Starting Child TCB=%p\n", child);
  DEBUGASSERT(child);

  /* Get the assigned pid before we start the task */

  pid = child->cmn.pid;

  /* Eliminate a race condition by disabling pre-emption.  The child task
   * can be instantiated, but cannot run until we call waitpid().  This
   * assures us that we cannot miss the death-of-child signal (only
   * needed in the SMP case).
   */

  sched_lock();

  /* Activate the task */

  nxtask_activate((FAR struct tcb_s *)child);

  sched_unlock();
  return pid;
}

/****************************************************************************
 * Name: nxtask_abort_fork
 *
 * Description:
 *   Recover from any errors after nxtask_setup_fork() was called.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxtask_abort_fork(FAR struct task_tcb_s *child, int errcode)
{
  /* The TCB was added to the active task list by nxtask_setup_scheduler() */

  dq_rem((FAR dq_entry_t *)child, list_inactivetasks());

  /* Release the TCB */

  nxsched_release_tcb((FAR struct tcb_s *)child,
                      child->cmn.flags & TCB_FLAG_TTYPE_MASK);
  set_errno(errcode);
}

#endif /* CONFIG_ARCH_HAVE_FORK */
