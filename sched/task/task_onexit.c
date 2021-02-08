/****************************************************************************
 * sched/task/task_onexit.c
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

#include <stdlib.h>
#include <assert.h>
#include <unistd.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/fs/fs.h>

#include "sched/sched.h"
#include "task/task.h"

#ifdef CONFIG_SCHED_ONEXIT

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: on_exit
 *
 * Description:
 *    Registers a function to be called at program exit.
 *    The on_exit() function registers the given function to be called
 *    at normal process termination, whether via exit or via return from
 *    the program's main(). The function is passed the status argument
 *    given to the last call to exit and the arg argument from on_exit().
 *
 *    NOTE 1: This function comes from SunOS 4, but is also present in
 *    libc4, libc5 and glibc. It no longer occurs in Solaris (SunOS 5).
 *    Avoid this function, and use the standard atexit() instead.
 *
 *    NOTE 2: CONFIG_SCHED_ONEXIT must be defined to enable this function
 *
 *    Limitations in the current implementation:
 *
 *      1. Only a single on_exit function can be registered unless
 *         CONFIG_SCHED_ONEXIT_MAX defines a larger number.
 *      2. on_exit functions are not inherited when a new task is
 *         created.
 *
 * Input Parameters:
 *   func - A pointer to the function to be called when the task exits.
 *   arg -  An argument that will be provided to the on_exit() function when
 *          the task exits.
 *
 * Returned Value:
 *   Zero on success. Non-zero on failure.
 *
 ****************************************************************************/

int on_exit(CODE void (*func)(int, FAR void *), FAR void *arg)
{
  FAR struct tcb_s *tcb = this_task();
  FAR struct task_group_s *group = tcb->group;
  int index;
  int ret = ENOSPC;

  DEBUGASSERT(group);

  /* The following must be atomic */

  if (func)
    {
      sched_lock();

      /* Search for the first available slot.  on_exit() functions are
       * registered from lower to higher array indices; they must be called
       * in the reverse order of registration when task exists, i.e.,
       * from higher to lower indices.
       */

      for (index = 0; index < CONFIG_SCHED_EXIT_MAX; index++)
        {
          if (!group->tg_exit[index].func.on)
            {
              group->tg_exit[index].func.on = func;
              group->tg_exit[index].arg     = arg;
              ret = OK;
              break;
            }
        }

      sched_unlock();
    }

  return ret;
}

#endif /* CONFIG_SCHED_ONEXIT */
