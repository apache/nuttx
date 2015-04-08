/****************************************************************************
 * sched/task/task_create.c
 *
 *   Copyright (C) 2007-2010, 2013-2014 Gregory Nutt. All rights reserved.
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
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Global Variables
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: thread_create
 *
 * Description:
 *   This function creates and activates a new thread of the specified type
 *   with a specified priority and returns its system-assigned ID.  It is the
 *   internal, common implementation of task_create() and kernel_thread().
 *   See comments with task_create() for further information.
 *
 * Input Parameters:
 *   name       - Name of the new task
 *   ttype      - Type of the new task
 *   priority   - Priority of the new task
 *   stack_size - size (in bytes) of the stack needed
 *   entry      - Entry point of a new task
 *   arg        - A pointer to an array of input parameters. Up to
 *                CONFIG_MAX_TASK_ARG parameters may be provided.  If fewer
 *                than CONFIG_MAX_TASK_ARG parameters are passed, the list
 *                should be terminated with a NULL argv[] value. If no
 *                parameters are required, argv may be NULL.
 *
 * Return Value:
 *   Returns the non-zero process ID of the new task or ERROR if memory is
 *   insufficient or the task cannot be created.  The errno will be set to
 *   indicate the nature of the error (always ENOMEM).
 *
 ****************************************************************************/

static int thread_create(FAR const char *name, uint8_t ttype, int priority,
                         int stack_size, main_t entry, FAR char * const argv[])
{
  FAR struct task_tcb_s *tcb;
  pid_t pid;
  int errcode;
  int ret;

  /* Allocate a TCB for the new task. */

  tcb = (FAR struct task_tcb_s *)kmm_zalloc(sizeof(struct task_tcb_s));
  if (!tcb)
    {
      sdbg("ERROR: Failed to allocate TCB\n");
      errcode = ENOMEM;
      goto errout;
    }

  /* Allocate a new task group with privileges appropriate for the parent
   * thread type.
   */

#ifdef HAVE_TASK_GROUP
  ret = group_allocate(tcb, ttype);
  if (ret < 0)
    {
      errcode = -ret;
      goto errout_with_tcb;
    }
#endif

  /* Associate file descriptors with the new task */

#if CONFIG_NFILE_DESCRIPTORS > 0 || CONFIG_NSOCKET_DESCRIPTORS > 0
  ret = group_setuptaskfiles(tcb);
  if (ret < OK)
    {
      errcode = -ret;
      goto errout_with_tcb;
    }
#endif

  /* Allocate the stack for the TCB */

  ret = up_create_stack((FAR struct tcb_s *)tcb, stack_size, ttype);
  if (ret < OK)
    {
      errcode = -ret;
      goto errout_with_tcb;
    }

  /* Initialize the task control block */

  ret = task_schedsetup(tcb, priority, task_start, entry, ttype);
  if (ret < OK)
    {
      errcode = -ret;
      goto errout_with_tcb;
    }

  /* Setup to pass parameters to the new task */

  (void)task_argsetup(tcb, name, argv);

  /* Now we have enough in place that we can join the group */

#ifdef HAVE_TASK_GROUP
  ret = group_initialize(tcb);
  if (ret < 0)
    {
      errcode = -ret;
      goto errout_with_tcb;
    }
#endif

  /* Get the assigned pid before we start the task */

  pid = (int)tcb->cmn.pid;

  /* Activate the task */

  ret = task_activate((FAR struct tcb_s *)tcb);
  if (ret < OK)
    {
      errcode = get_errno();

      /* The TCB was added to the active task list by task_schedsetup() */

      dq_rem((FAR dq_entry_t*)tcb, (dq_queue_t*)&g_inactivetasks);
      goto errout_with_tcb;
    }

  return pid;

errout_with_tcb:
  sched_releasetcb((FAR struct tcb_s *)tcb, ttype);

errout:
  set_errno(errcode);
  return ERROR;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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
 *   arg        - A pointer to an array of input parameters. Up to
 *                CONFIG_MAX_TASK_ARG parameters may be provided.  If fewer
 *                than CONFIG_MAX_TASK_ARG parameters are passed, the list
 *                should be terminated with a NULL argv[] value. If no
 *                parameters are required, argv may be NULL.
 *
 * Return Value:
 *   Returns the non-zero process ID of the new task or ERROR if memory is
 *   insufficient or the task cannot be created.  The errno will be set to
 *   indicate the nature of the error (always ENOMEM).
 *
 ****************************************************************************/

#ifndef CONFIG_BUILD_KERNEL
int task_create(FAR const char *name, int priority,
                int stack_size, main_t entry, FAR char * const argv[])
{
  return thread_create(name, TCB_FLAG_TTYPE_TASK, priority, stack_size, entry, argv);
}
#endif

/****************************************************************************
 * Name: kernel_thread
 *
 * Description:
 *   This function creates and activates a kernel thread task with kernel-
 *   mode privileges.  It is identical to task_create() except that it
 *   configures the newly started thread to run in kernel model.
 *
 * Input Parameters:
 *   (same as task_create())
 *
 * Return Value:
 *   (same as task_create())
 *
 ****************************************************************************/

int kernel_thread(FAR const char *name, int priority,
                  int stack_size, main_t entry, FAR char * const argv[])
{
  return thread_create(name, TCB_FLAG_TTYPE_KERNEL, priority, stack_size, entry, argv);
}
