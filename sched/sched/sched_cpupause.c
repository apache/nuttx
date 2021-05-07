/****************************************************************************
 * sched/sched/sched_cpupause.c
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
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/sched_note.h>

#include "sched/sched.h"

#ifdef CONFIG_SMP

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsched_pause_cpu
 *
 * Description:
 *   Check if task associated with 'tcb' is running on a different CPU.  If
 *   so then pause that CPU and return its CPU index.
 *
 * Input Parameters:
 *   tcb - The TCB of the task to be conditionally paused.
 *
 * Returned Value:
 *   If a CPU is pauses its non-negative CPU index is returned.  This index
 *   may then be used to resume the CPU.  If the task is not running at all
 *   (or if an error occurs), then a negated errno value is returned.  -ESRCH
 *   is returned in the case where the task is not running on any CPU.
 *
 * Assumptions:
 *   This function was called in a critical section.  In that case, no tasks
 *   may started or may exit until the we leave the critical section.  This
 *   critical section should extend until up_cpu_resume() is called in the
 *   typical case.
 *
 ****************************************************************************/

int nxsched_pause_cpu(FAR struct tcb_s *tcb)
{
  int cpu;
  int ret;

  DEBUGASSERT(tcb != NULL);

  /* If the task is not running at all then our job is easy */

  cpu = tcb->cpu;
  if (tcb->task_state != TSTATE_TASK_RUNNING)
    {
      return -ESRCH;
    }

  /* Check the CPU that the task is running on */

  DEBUGASSERT(cpu != this_cpu() && (unsigned int)cpu < CONFIG_SMP_NCPUS);
  if (cpu == this_cpu())
    {
      /* We can't pause ourself */

      return -EACCES;
    }

  /* Pause the CPU that the task is running on */

  ret = up_cpu_pause(cpu);
  if (ret < 0)
    {
      return ret;
    }

  /* Return the CPU that the task is running on */

  return cpu;
}

#endif /* CONFIG_SMP */
