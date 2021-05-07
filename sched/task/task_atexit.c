/****************************************************************************
 * sched/task/task_atexit.c
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

#ifdef CONFIG_SCHED_ATEXIT

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_SCHED_ONEXIT
static void exitfunc(int exitcode, FAR void *arg)
{
  (*(atexitfunc_t)arg)();
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: atexit
 *
 * Description:
 *    Registers a function to be called at program exit.
 *    The atexit() function registers the given function to be called
 *    at normal process termination, whether via exit or via return from
 *    the program's main().
 *
 *    NOTE: CONFIG_SCHED_ATEXIT must be defined to enable this function
 *
 *    Limitations in the current implementation:
 *
 *      1. Only a single atexit function can be registered unless
 *         CONFIG_SCHED_ATEXIT_MAX defines a larger number.
 *      2. atexit functions are not inherited when a new task is
 *         created.
 *      3. If both SCHED_ONEXIT and SCHED_ATEXIT are selected, then atexit()
 *         is built on top of the on_exit() implementation.  In that case,
 *         CONFIG_SCHED_ONEXIT_MAX determines the size of the combined
 *         number of atexit() and on_exit() calls and SCHED_ATEXIT_MAX is
 *         not used.
 *
 * Input Parameters:
 *   func - A pointer to the function to be called when the task exits.
 *
 * Returned Value:
 *   Zero on success. Non-zero on failure.
 *
 ****************************************************************************/

int atexit(void (*func)(void))
{
#if defined(CONFIG_SCHED_ONEXIT)
  /* atexit is equivalent to on_exit() with no argument (Assuming that the
   * ABI can handle a callback function that receives more parameters than
   * it expects).
   */

  return on_exit(exitfunc, func);

#else
  FAR struct tcb_s *tcb = this_task();
  FAR struct task_group_s *group = tcb->group;
  int index;
  int ret = ERROR;

  DEBUGASSERT(group);

  /* The following must be atomic */

  if (func)
    {
      sched_lock();

      /* Search for the first available slot.  atexit() functions are
       * registered from lower to higher array indices; they must be called
       * in the reverse order of registration when task exists, i.e., from
       * higher to lower indices.
       */

      for (index = 0; index < CONFIG_SCHED_EXIT_MAX; index++)
        {
          if (!group->tg_exit[index].func.at)
            {
              group->tg_exit[index].func.at = func;
              ret = OK;
              break;
            }
        }

      sched_unlock();
    }

  return ret;
#endif
}

#endif /* CONFIG_SCHED_ATEXIT */
