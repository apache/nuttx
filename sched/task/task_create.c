/****************************************************************************
 * sched/task/task_create.c
 *
 *   Copyright (C) 2007-2010, 2013-2014, 2016, 2018 Gregory Nutt. All rights
 *     reserved.
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
#include <sched.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>

#include "sched/sched.h"
#include "group/group.h"
#include "task/task.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: thread_create
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

static int thread_create(FAR const char *name, uint8_t ttype, int priority,
                         int stack_size, main_t entry,
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

#ifdef HAVE_TASK_GROUP
  /* Allocate a new task group with privileges appropriate for the parent
   * thread type.
   */

  ret = group_allocate(tcb, ttype);
  if (ret < 0)
    {
      goto errout_with_tcb;
    }
#endif

#if CONFIG_NFILE_DESCRIPTORS > 0 || CONFIG_NSOCKET_DESCRIPTORS > 0
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
#endif

  /* Allocate the stack for the TCB */

  ret = up_create_stack((FAR struct tcb_s *)tcb, stack_size, ttype);
  if (ret < OK)
    {
      goto errout_with_tcb;
    }

  /* Initialize the task control block */

  ret = task_schedsetup(tcb, priority, task_start, entry, ttype);
  if (ret < OK)
    {
      goto errout_with_tcb;
    }

  /* Setup to pass parameters to the new task */

  (void)task_argsetup(tcb, name, argv);

#ifdef HAVE_TASK_GROUP
  /* Now we have enough in place that we can join the group */

  ret = group_initialize(tcb);
  if (ret < 0)
    {
      goto errout_with_tcb;
    }
#endif

  /* Get the assigned pid before we start the task */

  pid = (int)tcb->cmn.pid;

  /* Activate the task */

  ret = task_activate((FAR struct tcb_s *)tcb);
  if (ret < OK)
    {
      ret = -get_errno();
      DEBUGASSERT(ret < 0);

      /* The TCB was added to the active task list by task_schedsetup() */

      dq_rem((FAR dq_entry_t *)tcb, (FAR dq_queue_t *)&g_inactivetasks);
      goto errout_with_tcb;
    }

  return pid;

errout_with_tcb:
  sched_releasetcb((FAR struct tcb_s *)tcb, ttype);
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
  return thread_create(name, TCB_FLAG_TTYPE_TASK, priority, stack_size,
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
                   int stack_size, main_t entry, FAR char *const argv[])
{
  return thread_create(name, TCB_FLAG_TTYPE_KERNEL, priority, stack_size,
                       entry, argv);
}
