/****************************************************************************
 * sched/irq/irq_restorelock.c
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

#include "sched/sched.h"
#include "irq/irq.h"

#ifdef CONFIG_SMP

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: irq_restore_cpulock
 *
 * Description:
 *   Restore the state of g_cpu_schedlock and g_cpu_irqlock.  This function
 *   is called after a context switch on another CPU.  A consequence of
 *   the context switch is that the global spinlocks may need to change
 *   states.
 *
 * Input Parameters:
 *   cpu  - The CPU on which the task was started
 *   rtcb - The TCB of the task that was started
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void irq_restore_cpulock(int cpu, FAR struct tcb_s *rtcb)
{
  /* Adjust global pre-emption controls.  If the lockcount is greater than
   * zero, then this task/this CPU holds the scheduler lock.
   */

  if (rtcb->irqcount > 0)
    {
      spin_setbit(&g_cpu_irqset, cpu, &g_cpu_irqsetlock, &g_cpu_irqlock);
    }
  else
    {
      spin_clrbit(&g_cpu_irqset, cpu, &g_cpu_irqsetlock, &g_cpu_irqlock);
    }
}

/****************************************************************************
 * Name: irq_restore_lock
 *
 * Description:
 *   Restore the state of g_cpu_schedlock and g_cpu_irqlock.  This function
 *   is called after a context switch on the current CPU.  A consequence of
 *   the context switch is that the global spinlocks may need to change
 *   states.  However, the actual realization of that change cannot occur
 *   until all context switching operations have completed.  This function
 *   implements the deferred setting of g_cpu_irqlock.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   g_cpu_irqlock is set upon entry.  It may or may not be set upon return.
 *
 ****************************************************************************/

void irq_restore_lock(void)
{
  FAR struct tcb_s *rtcb;
  int cpu;

  cpu  = this_cpu();
  rtcb = current_task(cpu);

  /* Adjust global pre-emption and IRQ controls. */

  irq_restore_cpulock(cpu, rtcb);
}

#endif /* CONFIG_SMP */
