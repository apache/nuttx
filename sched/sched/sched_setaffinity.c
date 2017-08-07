/****************************************************************************
 * sched/sched/sched_setaffinity.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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
#include <assert.h>
#include <errno.h>

#include <nuttx/arch.h>

#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sched_getscheduler
 *
 * Description:
 *   sched_setaffinity() sets the CPU affinity mask of the thread whose ID
 *   is pid to the value specified by mask.  If pid is zero, then the
 *   calling thread is used.  The argument cpusetsize is the length (i
 *   bytes) of the data pointed to by mask.  Normally this argument would
 *   be specified as sizeof(cpu_set_t).
 *
 *  If the thread specified by pid is not currently running on one of the
 *  CPUs specified in mask, then that thread is migrated to one of the
 *   CPUs specified in mask.
 *
 * Inputs:
 *   pid       - The ID of thread whose affinity set will be modified.
 *   cpusetsize - Size of cpuset.  MUST be sizeofcpu_set_t().
 *   cpuset     - The location to return the thread's new affinity set.
 *
 * Return Value:
 *   0 if successful.  Otherwise, ERROR (-1) is returned, and errno is
 *   set appropriately:
 *
 *      ESRCH  The task whose ID is pid could not be found.
 *
 ****************************************************************************/

int sched_setaffinity(pid_t pid, size_t cpusetsize, FAR const cpu_set_t *mask)
{
  FAR struct tcb_s *tcb;
  irqstate_t flags;
  int errcode = 0;
  int ret;

  DEBUGASSERT(cpusetsize == sizeof(cpu_set_t) && mask != NULL);

  /* Verify that the PID corresponds to a real task */

  sched_lock();
  if (!pid)
    {
      tcb = this_task();
    }
  else
    {
      tcb = sched_gettcb(pid);
    }

  if (tcb == NULL)
    {
      errcode = ESRCH;
      goto errout_with_lock;
    }

  /* Don't permit changing the affinity mask of any task locked to a CPU
   * (i.e., an IDLE task)
   */

  flags = enter_critical_section();
  if ((tcb->flags & TCB_FLAG_CPU_LOCKED) != 0)
    {
      errcode = EINVAL;
      goto errout_with_csection;
    }

  /* Set the new affinity mask. */

  tcb->affinity = *mask;

  /* Is the task still executing a a CPU in its affinity mask? Will this
   * change cause the task to be removed from its current assigned task
   * list?
   *
   * First... is the task in an assigned task list?
   */

  if (tcb->task_state >= FIRST_ASSIGNED_STATE &&
      tcb->task_state <= LAST_ASSIGNED_STATE)
    {
      /* Yes... is the CPU associated with the assigned task in the new
       * affinity mask?
       */

      if ((tcb->affinity & (1 << tcb->cpu)) == 0)
        {
          /* No.. then we will need to move the task from the assigned
           * task list to some other ready to run list.
           *
           * sched_setpriority() will do just what we want... it will remove
           * the task from its current position in the some assigned task list
           * and then simply put it back in the right place.  This works even
           * if the task is this task.
           */

          ret = sched_setpriority(tcb, tcb->sched_priority);
          if (ret < 0)
            {
              errcode = get_errno();
            }
        }
    }

errout_with_csection:
  leave_critical_section(flags);

errout_with_lock:
  sched_unlock();
  if (errcode != 0)
    {
      set_errno(errcode);
      return ERROR;
    }

  return OK;
}
