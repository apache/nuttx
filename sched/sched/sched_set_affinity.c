/****************************************************************************
 * sched/sched/sched_set_affinity.c
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
#include <assert.h>
#include <errno.h>

#include <nuttx/sched.h>

#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsched_set_affinity
 *
 * Description:
 *   nxsched_set_affinity() sets the CPU affinity mask of the thread whose ID
 *   is pid to the value specified by mask.  If pid is zero, then the
 *   calling thread is used.  The argument cpusetsize is the length (i
 *   bytes) of the data pointed to by mask.  Normally this argument would
 *   be specified as sizeof(cpu_set_t).
 *
 *   If the thread specified by pid is not currently running on one of the
 *   CPUs specified in mask, then that thread is migrated to one of the
 *   CPUs specified in mask.
 *
 *   nxsched_set_affinity() is identical to the function nxsched_set_param(),
 *   differing only in its return value:  This function does not modify
 *   the errno variable.  This is a non-standard, internal OS function and
 *   is not intended for use by application logic.  Applications should
 *   use the standard nxsched_set_param().
 *
 * Input Parameters:
 *   pid        - The ID of thread whose affinity set will be modified.
 *   cpusetsize - Size of mask.  MUST be sizeofcpu_set_t().
 *   mask       - The location to return the thread's new affinity set.
 *
 * Returned Value:
 *   Zero (OK) if successful.  Otherwise, a negated errno value is returned:
 *
 *     ESRCH  The task whose ID is pid could not be found.
 *
 ****************************************************************************/

int nxsched_set_affinity(pid_t pid, size_t cpusetsize,
                         FAR const cpu_set_t *mask)
{
  FAR struct tcb_s *tcb;
  irqstate_t flags;
  int ret = OK;

  DEBUGASSERT(cpusetsize == sizeof(cpu_set_t) && mask != NULL);

  /* Verify that the PID corresponds to a real task */

  if (!pid)
    {
      tcb = this_task();
    }
  else
    {
      tcb = nxsched_get_tcb(pid);
    }

  if (tcb == NULL)
    {
      ret = -ESRCH;
      goto errout;
    }

  /* Don't permit changing the affinity mask of any task locked to a CPU
   * (i.e., an IDLE task)
   */

  flags = enter_critical_section();
  if ((tcb->flags & TCB_FLAG_CPU_LOCKED) != 0)
    {
      ret = -EINVAL;
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
           * nxsched_set_priority() will do just what we want... it will
           * remove the task from its current position in the some assigned
           * task list and then simply put it back in the right place.  This
           * works even if the task is this task.
           */

          ret = nxsched_set_priority(tcb, tcb->sched_priority);
        }
    }

errout_with_csection:
  leave_critical_section(flags);

errout:
  return ret;
}
