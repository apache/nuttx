/****************************************************************************
 * sched/sched/sched_cpupause.c
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
