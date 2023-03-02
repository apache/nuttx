/****************************************************************************
 * arch/arm/src/armv7-r/arm_cpupause.c
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

#include <nuttx/arch.h>
#include <nuttx/sched.h>
#include <nuttx/spinlock.h>
#include <nuttx/sched_note.h>

#include "arm_internal.h"
#include "gic.h"
#include "sched/sched.h"

#ifdef CONFIG_SMP

/****************************************************************************
 * Private Data
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

static volatile spinlock_t g_cpu_wait[CONFIG_SMP_NCPUS];
static volatile spinlock_t g_cpu_paused[CONFIG_SMP_NCPUS];

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
  struct tcb_s *tcb = this_task();

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

  /* Release the g_cpu_paused spinlock to synchronize with the
   * requesting CPU.
   */

  spin_unlock(&g_cpu_paused[cpu]);

  /* Wait for the spinlock to be released.  The requesting CPU will release
   * the spinlock when the CPU is resumed.
   */

  spin_lock(&g_cpu_wait[cpu]);

  /* This CPU has been resumed. Restore the exception context of the TCB at
   * the (new) head of the assigned task list.
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
 * Name: arm_pause_handler
 *
 * Description:
 *   This is the handler for SGI2.  It performs the following operations:
 *
 *   1. It saves the current task state at the head of the current assigned
 *      task list.
 *   2. It waits on a spinlock, then
 *   3. Returns from interrupt, restoring the state of the new task at the
 *      head of the ready to run list.
 *
 * Input Parameters:
 *   Standard interrupt handling
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int arm_pause_handler(int irq, void *context, void *arg)
{
  int cpu = this_cpu();

  /* Check for false alarms.  Such false could occur as a consequence of
   * some deadlock breaking logic that might have already serviced the SG2
   * interrupt by calling up_cpu_paused().  If the pause event has already
   * been processed then g_cpu_paused[cpu] will not be locked.
   */

  if (up_cpu_pausereq(cpu))
    {
      /* NOTE: The following enter_critical_section() will call
       * up_cpu_paused() to process a pause request to break a deadlock
       * because the caller held a critical section. Once up_cpu_paused()
       * finished, the caller will proceed and release the g_cpu_irqlock.
       * Then this CPU will acquire g_cpu_irqlock in the function.
       */

      irqstate_t flags = enter_critical_section();

      /* NOTE: the pause request should not exist here */

      DEBUGVERIFY(!up_cpu_pausereq(cpu));

      leave_critical_section(flags);
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
 *   cpu - The index of the CPU to be stopped
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_cpu_pause(int cpu)
{
  DEBUGASSERT(cpu >= 0 && cpu < CONFIG_SMP_NCPUS && cpu != this_cpu());

#ifdef CONFIG_SCHED_INSTRUMENTATION
  /* Notify of the pause event */

  sched_note_cpu_pause(this_task(), cpu);
#endif

  /* Take the both spinlocks.  The g_cpu_wait spinlock will prevent the SGI2
   * handler from returning until up_cpu_resume() is called; g_cpu_paused
   * is a handshake that will prefent this function from returning until
   * the CPU is actually paused.
   * Note that we might spin before getting g_cpu_wait, this just means that
   * the other CPU still hasn't finished responding to the previous resume
   * request.
   */

  DEBUGASSERT(!spin_islocked(&g_cpu_paused[cpu]));

  spin_lock(&g_cpu_wait[cpu]);
  spin_lock(&g_cpu_paused[cpu]);

  /* Execute SGI2 */

  arm_cpu_sgi(GIC_IRQ_SGI2, (1 << cpu));

  /* Wait for the other CPU to unlock g_cpu_paused meaning that
   * it is fully paused and ready for up_cpu_resume();
   */

  spin_lock(&g_cpu_paused[cpu]);
  spin_unlock(&g_cpu_paused[cpu]);

  /* On successful return g_cpu_wait will be locked, the other CPU will be
   * spinning on g_cpu_wait and will not continue until g_cpu_resume() is
   * called.  g_cpu_paused will be unlocked in any case.
   */

  return OK;
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
  DEBUGASSERT(cpu >= 0 && cpu < CONFIG_SMP_NCPUS && cpu != this_cpu());

#ifdef CONFIG_SCHED_INSTRUMENTATION
  /* Notify of the resume event */

  sched_note_cpu_resume(this_task(), cpu);
#endif

  /* Release the spinlock.  Releasing the spinlock will cause the SGI2
   * handler on 'cpu' to continue and return from interrupt to the newly
   * established thread.
   */

  DEBUGASSERT(spin_islocked(&g_cpu_wait[cpu]) &&
              !spin_islocked(&g_cpu_paused[cpu]));

  spin_unlock(&g_cpu_wait[cpu]);
  return OK;
}

#endif /* CONFIG_SMP */
