/****************************************************************************
 * sched/sched/sched_cpuselect.c
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
#include <assert.h>

#include <nuttx/sched.h>

#include "sched/sched.h"

#ifdef CONFIG_SMP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IMPOSSIBLE_CPU 0xff

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  nxsched_select_cpu
 *
 * Description:
 *   Return the index to the CPU with the lowest priority running task,
 *   possibly its IDLE task.
 *
 * Input Parameters:
 *   affinity - The set of CPUs on which the thread is permitted to run.
 *
 * Returned Value:
 *   Index of the CPU with the lowest priority running task
 *
 * Assumptions:
 *   Called from within a critical section.
 *
 ****************************************************************************/

int nxsched_select_cpu(cpu_set_t affinity)
{
  uint8_t minprio;
  int cpu;
  int i;

  /* Otherwise, find the CPU that is executing the lowest priority task
   * (possibly its IDLE task).
   */

  minprio = SCHED_PRIORITY_MAX;
  cpu     = IMPOSSIBLE_CPU;

  for (i = 0; i < CONFIG_SMP_NCPUS; i++)
    {
      /* If the thread permitted to run on this CPU? */

      if ((affinity & (1 << i)) != 0)
        {
          FAR struct tcb_s *rtcb = (FAR struct tcb_s *)
                                   g_assignedtasks[i].head;

          /* If this thread is executing its IDLE task, the use it.  The
           * IDLE task is always the last task in the assigned task list.
           */

          if (rtcb->flink == NULL)
            {
              /* The IDLE task should always be assigned to this CPU and have
               * a priority of zero.
               */

              DEBUGASSERT(rtcb->sched_priority == 0);
              return i;
            }
          else if (rtcb->sched_priority < minprio)
            {
              DEBUGASSERT(rtcb->sched_priority > 0);
              minprio = rtcb->sched_priority;
              cpu = i;
            }
        }
    }

  DEBUGASSERT(cpu != IMPOSSIBLE_CPU);
  return cpu;
}

#endif /* CONFIG_SMP */
