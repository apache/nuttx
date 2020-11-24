/****************************************************************************
 * arch/arm/src/lc823450/lc823450_cpupause.c
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

#include <stdint.h>
#include <assert.h>
#include <debug.h>
#include <string.h>
#include <stdio.h>

#include <nuttx/arch.h>
#include <nuttx/spinlock.h>
#include <nuttx/sched_note.h>

#include "arm_arch.h"
#include "sched/sched.h"
#include "arm_internal.h"
#include "lc823450_intc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if 0
#define DPRINTF(fmt, args...) llinfo(fmt, ##args)
#else
#define DPRINTF(fmt, args...) do {} while (0)
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* These spinlocks are used in the SMP configuration in order to implement
 * up_cpu_pause().  The protocol for CPUn to pause CPUm is as follows
 *
 * 1. The up_cpu_pause() implementation on CPUn locks both g_cpu_wait[m]
 *    and g_cpu_paused[m].  CPUn then waits spinning on g_cpu_paused[m].
 * 2. CPUm receives the interrupt it (1) unlocks g_cpu_paused[m] and
 *    (2) locks g_cpu_wait[m].  The first unblocks CPUn and the second
 *    blocks CPUm in the interrupt handler.
 *
 * When CPUm resumes, CPUn unlocks g_cpu_wait[m] and the interrupt handler
 * on CPUm continues.  CPUm must, of course, also then unlock g_cpu_wait[m]
 * so that it will be ready for the next pause operation.
 */

volatile spinlock_t g_cpu_wait[CONFIG_SMP_NCPUS];
volatile spinlock_t g_cpu_paused[CONFIG_SMP_NCPUS];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_cpu_pausereq
 *
 * Description:
 *   Return true if a pause request is pending for this CPU.
 *
 * Input Parameters:
 *   cpu - The index of the CPU to be queried
 *
 * Returned Value:
 *   true   = a pause request is pending.
 *   false = no pasue request is pending.
 *
 ****************************************************************************/

bool up_cpu_pausereq(int cpu)
{
  return spin_islocked(&g_cpu_paused[cpu]);
}

/****************************************************************************
 * Name: up_cpu_paused
 *
 * Description:
 *   Handle a pause request from another CPU.  Normally, this logic is
 *   executed from interrupt handling logic within the architecture-specific
 *   However, it is sometimes necessary necessary to perform the pending
 *   pause operation in other contexts where the interrupt cannot be taken
 *   in order to avoid deadlocks.
 *
 *   This function performs the following operations:
 *
 *   1. It saves the current task state at the head of the current assigned
 *      task list.
 *   2. It waits on a spinlock, then
 *   3. Returns from interrupt, restoring the state of the new task at the
 *      head of the ready to run list.
 *
 * Input Parameters:
 *   cpu - The index of the CPU to be paused
 *
 * Returned Value:
 *   On success, OK is returned.  Otherwise, a negated errno value indicating
 *   the nature of the failure is returned.
 *
 ****************************************************************************/

int up_cpu_paused(int cpu)
{
  FAR struct tcb_s *tcb = this_task();

  /* Update scheduler parameters */

  nxsched_suspend_scheduler(tcb);

#ifdef CONFIG_SCHED_INSTRUMENTATION
  /* Notify that we are paused */

  sched_note_cpu_paused(tcb);
#endif

  /* Save the current context at CURRENT_REGS into the TCB at the head
   * of the assigned task list for this CPU.
   */

  arm_savestate(tcb->xcp.regs);

  /* Wait for the spinlock to be released */

  spin_unlock(&g_cpu_paused[cpu]);
  spin_lock(&g_cpu_wait[cpu]);

  /* Restore the exception context of the tcb at the (new) head of the
   * assigned task list.
   */

  tcb = this_task();

#ifdef CONFIG_SCHED_INSTRUMENTATION
  /* Notify that we have resumed */

  sched_note_cpu_resumed(tcb);
#endif

  /* Reset scheduler parameters */

  nxsched_resume_scheduler(tcb);

  /* Then switch contexts.  Any necessary address environment changes
   * will be made when the interrupt returns.
   */

  arm_restorestate(tcb->xcp.regs);

  spin_unlock(&g_cpu_wait[cpu]);

  return OK;
}

/****************************************************************************
 * Name: lc823450_pause_handler
 *
 * Description:
 *   Inter-CPU interrupt handler
 *
 * Input Parameters:
 *   Standard interrupt handler inputs
 *
 * Returned Value:
 *   Should always return OK
 *
 ****************************************************************************/

int lc823450_pause_handler(int irq, void *c, FAR void *arg)
{
  int cpu = up_cpu_index();

  /* Clear : Pause IRQ */

  if (irq == LC823450_IRQ_CTXM3_01)
    {
      DPRINTF("CPU0 -> CPU1\n");
      putreg32(IPICLR_INTISR0_CLR_1, IPICLR);
    }
  else
    {
      DPRINTF("CPU1 -> CPU0\n");
      putreg32(IPICLR_INTISR1_CLR_1, IPICLR);
    }

  /* Check for false alarms.  Such false could occur as a consequence of
   * some deadlock breaking logic that might have already serviced the SG2
   * interrupt by calling up_cpu_paused.
   */

  if (spin_islocked(&g_cpu_paused[cpu]))
    {
      return up_cpu_paused(cpu);
    }

  return OK;
}

/****************************************************************************
 * Name: up_cpu_pause
 *
 * Description:
 *   Save the state of the current task at the head of the
 *   g_assignedtasks[cpu] task list and then pause task execution on the
 *   CPU.
 *
 *   This function is called by the OS when the logic executing on one CPU
 *   needs to modify the state of the g_assignedtasks[cpu] list for another
 *   CPU.
 *
 * Input Parameters:
 *   cpu - The index of the CPU to be stopped/
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_cpu_pause(int cpu)
{
  DPRINTF("cpu=%d\n", cpu);

#ifdef CONFIG_SCHED_INSTRUMENTATION
  /* Notify of the pause event */

  sched_note_cpu_pause(this_task(), cpu);
#endif

  DEBUGASSERT(cpu >= 0 && cpu < CONFIG_SMP_NCPUS && cpu != this_cpu());

  /* Take the both spinlocks.  The g_cpu_wait spinlock will prevent the SGI2
   * handler from returning until up_cpu_resume() is called; g_cpu_paused
   * is a handshake that will prefent this function from returning until
   * the CPU is actually paused.
   */

  DEBUGASSERT(!spin_islocked(&g_cpu_wait[cpu]) &&
              !spin_islocked(&g_cpu_paused[cpu]));

  spin_lock(&g_cpu_wait[cpu]);
  spin_lock(&g_cpu_paused[cpu]);

  /* Execute Pause IRQ to CPU(cpu) */

  if (cpu == 1)
    {
      putreg32(IPIREG_INTISR0_1, IPIREG);
    }
  else
    {
      putreg32(IPIREG_INTISR1_1, IPIREG);
    }

  /* Wait for the other CPU to unlock g_cpu_paused meaning that
   * it is fully paused and ready for up_cpu_resume();
   */

  spin_lock(&g_cpu_paused[cpu]);

  spin_unlock(&g_cpu_paused[cpu]);

  /* On successful return g_cpu_wait will be locked, the other CPU will be
   * spinning on g_cpu_wait and will not continue until g_cpu_resume() is
   * called.  g_cpu_paused will be unlocked in any case.
   */

  return 0;
}

/****************************************************************************
 * Name: up_cpu_resume
 *
 * Description:
 *   Restart the cpu after it was paused via up_cpu_pause(), restoring the
 *   state of the task at the head of the g_assignedtasks[cpu] list, and
 *   resume normal tasking.
 *
 *   This function is called after up_cpu_pause in order resume operation of
 *   the CPU after modifying its g_assignedtasks[cpu] list.
 *
 * Input Parameters:
 *   cpu - The index of the CPU being re-started.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_cpu_resume(int cpu)
{
  DPRINTF("cpu=%d\n", cpu);

#ifdef CONFIG_SCHED_INSTRUMENTATION
  /* Notify of the resume event */

  sched_note_cpu_resume(this_task(), cpu);
#endif

  DEBUGASSERT(cpu >= 0 && cpu < CONFIG_SMP_NCPUS && cpu != this_cpu());

  /* Release the spinlock.  Releasing the spinlock will cause the SGI2
   * handler on 'cpu' to continue and return from interrupt to the newly
   * established thread.
   */

  DEBUGASSERT(spin_islocked(&g_cpu_wait[cpu]) &&
              !spin_islocked(&g_cpu_paused[cpu]));

  spin_unlock(&g_cpu_wait[cpu]);

  return 0;
}
