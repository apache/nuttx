/****************************************************************************
 * sched/sched/sched_cpuselect.c
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

  minprio = SCHED_PRIORITY_MAX;
  cpu     = IMPOSSIBLE_CPU;

  for (i = 0; i < CONFIG_SMP_NCPUS; i++)
    {
      /* Is the thread permitted to run on this CPU? */

      if ((affinity & (1 << i)) != 0)
        {
          FAR struct tcb_s *rtcb = (FAR struct tcb_s *)
                                   g_assignedtasks[i].head;

          /* If this CPU is executing its IDLE task, then use it.  The
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
