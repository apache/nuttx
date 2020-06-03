/****************************************************************************
 * sched/task/task_atexit.c
 *
 *   Copyright (C) 2007, 2009, 2011-2013 Gregory Nutt. All rights reserved.
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
