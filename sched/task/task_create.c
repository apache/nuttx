/****************************************************************************
 * sched/task/task_create.c
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
#include <sched.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/sched.h>
#include <nuttx/kthread.h>

#include "sched/sched.h"
#include "group/group.h"
#include "task/task.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxthread_create
 *
 * Description:
 *   This function creates and activates a new thread of the specified type
 *   with a specified priority and returns its system-assigned ID.  It is the
 *   internal, common implementation of task_create() and kthread_create().
 *   See comments with task_create() for further information.
 *
 * Input Parameters:
 *   name       - Name of the new task
 *   ttype      - Type of the new task
 *   priority   - Priority of the new task
 *   stack_size - size (in bytes) of the stack needed
 *   entry      - Entry point of a new task
 *   arg        - A pointer to an array of input parameters.  The array
 *                should be terminated with a NULL argv[] value. If no
 *                parameters are required, argv may be NULL.
 *
 * Returned Value:
 *   Returns the positive, non-zero process ID of the new task or a negated
 *   errno value to indicate the nature of any failure.  If memory is
 *   insufficient or the task cannot be created -ENOMEM will be returned.
 *
 ****************************************************************************/

static int nxthread_create(FAR const char *name, uint8_t ttype,
                           int priority, int stack_size, main_t entry,
                           FAR char * const argv[])
{
  FAR struct task_tcb_s *tcb;
  pid_t pid;
  int ret;

  /* Allocate a TCB for the new task. */

  tcb = (FAR struct task_tcb_s *)kmm_zalloc(sizeof(struct task_tcb_s));
  if (!tcb)
    {
      serr("ERROR: Failed to allocate TCB\n");
      return -ENOMEM;
    }

  /* Allocate a new task group with privileges appropriate for the parent
   * thread type.
   */

  ret = group_allocate(tcb, ttype);
  if (ret < 0)
    {
      goto errout_with_tcb;
    }

#if 0 /* No... there are side effects */
  /* Associate file descriptors with the new task.  Exclude kernel threads;
   * kernel threads do not have file or socket descriptors.  They must use
   * SYSLOG for output and the low-level psock interfaces for network I/O.
   */

  if (ttype != TCB_FLAG_TTYPE_KERNEL)
#endif
    {
      ret = group_setuptaskfiles(tcb);
      if (ret < OK)
        {
          goto errout_with_tcb;
        }
    }

  /* Allocate the stack for the TCB */

  ret = up_create_stack((FAR struct tcb_s *)tcb, stack_size, ttype);
  if (ret < OK)
    {
      goto errout_with_tcb;
    }

  /* Initialize the task control block */

  ret = nxtask_setup_scheduler(tcb, priority, nxtask_start, entry, ttype);
  if (ret < OK)
    {
      goto errout_with_tcb;
    }

  /* Setup to pass parameters to the new task */

  nxtask_setup_arguments(tcb, name, argv);

  /* Now we have enough in place that we can join the group */

  ret = group_initialize(tcb);
  if (ret < 0)
    {
      goto errout_with_tcb;
    }

  /* Get the assigned pid before we start the task */

  pid = (int)tcb->cmn.pid;

  /* Activate the task */

  ret = task_activate((FAR struct tcb_s *)tcb);
  if (ret < OK)
    {
      /* The TCB was added to the active task list by nxtask_setup_scheduler() */

      dq_rem((FAR dq_entry_t *)tcb, (FAR dq_queue_t *)&g_inactivetasks);
      goto errout_with_tcb;
    }

  return pid;

errout_with_tcb:
  nxsched_release_tcb((FAR struct tcb_s *)tcb, ttype);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxtask_create
 *
 * Description:
 *   This function creates and activates a new task with a specified
 *   priority and returns its system-assigned ID.
 *
 *   The entry address entry is the address of the "main" function of the
 *   task.  This function will be called once the C environment has been
 *   set up.  The specified function will be called with four arguments.
 *   Should the specified routine return, a call to exit() will
 *   automatically be made.
 *
 *   Note that four (and only four) arguments must be passed for the spawned
 *   functions.
 *
 *   nxtask_create() is identical to the function task_create(), differing
 *   only in its return value:  This function does not modify the errno
 *   variable.  This is a non-standard, internal OS function and is not
 *   intended for use by application logic.  Applications should use
 *   task_create().
 *
 * Input Parameters:
 *   name       - Name of the new task
 *   priority   - Priority of the new task
 *   stack_size - size (in bytes) of the stack needed
 *   entry      - Entry point of a new task
 *   arg        - A pointer to an array of input parameters.  The array
 *                should be terminated with a NULL argv[] value. If no
 *                parameters are required, argv may be NULL.
 *
 * Returned Value:
 *   Returns the positive, non-zero process ID of the new task or a negated
 *   errno value to indicate the nature of any failure.  If memory is
 *   insufficient or the task cannot be created -ENOMEM will be returned.
 *
 ****************************************************************************/

int nxtask_create(FAR const char *name, int priority,
                  int stack_size, main_t entry, FAR char * const argv[])
{
  return nxthread_create(name, TCB_FLAG_TTYPE_TASK, priority, stack_size,
                         entry, argv);
}

/****************************************************************************
 * Name: task_create
 *
 * Description:
 *   This function creates and activates a new task with a specified
 *   priority and returns its system-assigned ID.
 *
 *   The entry address entry is the address of the "main" function of the
 *   task.  This function will be called once the C environment has been
 *   set up.  The specified function will be called with four arguments.
 *   Should the specified routine return, a call to exit() will
 *   automatically be made.
 *
 *   Note that four (and only four) arguments must be passed for the spawned
 *   functions.
 *
 * Input Parameters:
 *   name       - Name of the new task
 *   priority   - Priority of the new task
 *   stack_size - size (in bytes) of the stack needed
 *   entry      - Entry point of a new task
 *   arg        - A pointer to an array of input parameters.  The array
 *                should be terminated with a NULL argv[] value. If no
 *                parameters are required, argv may be NULL.
 *
 * Returned Value:
 *   Returns the non-zero process ID of the new task or ERROR if memory is
 *   insufficient or the task cannot be created.  The errno will be set in
 *   the failure case to indicate the nature of the error.
 *
 ****************************************************************************/

#ifndef CONFIG_BUILD_KERNEL
int task_create(FAR const char *name, int priority,
                int stack_size, main_t entry, FAR char * const argv[])
{
  int ret = nxtask_create(name, priority, stack_size, entry, argv);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: kthread_create
 *
 * Description:
 *   This function creates and activates a kernel thread task with kernel-
 *   mode privileges.  It is identical to task_create() except that it
 *   configures the newly started thread to run in kernel model.
 *
 * Input Parameters:
 *   name       - Name of the new task
 *   priority   - Priority of the new task
 *   stack_size - size (in bytes) of the stack needed
 *   entry      - Entry point of a new task
 *   arg        - A pointer to an array of input parameters.  The array
 *                should be terminated with a NULL argv[] value. If no
 *                parameters are required, argv may be NULL.
 *
 * Returned Value:
 *   Returns the positive, non-zero process ID of the new task or a negated
 *   errno value to indicate the nature of any failure.  If memory is
 *   insufficient or the task cannot be created -ENOMEM will be returned.
 *
 ****************************************************************************/

int kthread_create(FAR const char *name, int priority,
                   int stack_size, main_t entry, FAR char * const argv[])
{
  return nxthread_create(name, TCB_FLAG_TTYPE_KERNEL, priority, stack_size,
                         entry, argv);
}
