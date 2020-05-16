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
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/sched.h>

#include "sched/sched.h"
#include "group/group.h"
#include "task/task.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: task_init
 *
 * Description:
 *   This function initializes a Task Control Block (TCB) in preparation for
 *   starting a new thread.  It performs a subset of the functionality of
 *   task_create()
 *
 *   Unlike task_create():
 *     1. Allocate the TCB.  The pre-allocated TCB is passed in argv.
 *     2. Allocate the stack.  The pre-allocated stack is passed in argv.
 *     3. Activate the task. This must be done by calling task_activate().
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

int task_init(FAR struct tcb_s *tcb, const char *name, int priority,
              FAR uint32_t *stack, uint32_t stack_size,
              main_t entry, FAR char * const argv[])
{
  FAR struct task_tcb_s *ttcb = (FAR struct task_tcb_s *)tcb;
  int ret;

  /* Only tasks and kernel threads can be initialized in this way */

#ifndef CONFIG_DISABLE_PTHREAD
  DEBUGASSERT(tcb &&
              (tcb->flags & TCB_FLAG_TTYPE_MASK) != TCB_FLAG_TTYPE_PTHREAD);
#endif

  /* Create a new task group */

  ret = group_allocate(ttcb, tcb->flags);
  if (ret < 0)
    {
      goto errout;
    }

  /* Associate file descriptors with the new task */

  ret = group_setuptaskfiles(ttcb);
  if (ret < 0)
    {
      goto errout_with_group;
    }

  /* Configure the user provided stack region */

  up_use_stack(tcb, stack, stack_size);

  /* Initialize the task control block */

  ret = nxtask_setup_scheduler(ttcb, priority, nxtask_start, entry,
                          TCB_FLAG_TTYPE_TASK);
  if (ret < OK)
    {
      goto errout_with_group;
    }

  /* Setup to pass parameters to the new task */

  nxtask_setup_arguments(ttcb, name, argv);

  /* Now we have enough in place that we can join the group */

  ret = group_initialize(ttcb);
  if (ret < 0)
    {
      goto errout_with_group;
    }

  return OK;

errout_with_group:
  group_leave(tcb);

errout:
  return ret;
}
