/****************************************************************************
 * sched/task/task_onexit.c
 *
 *   Copyright (C) 2012-2013 Gregory Nutt. All rights reserved.
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
