/****************************************************************************
 * arch/xtensa/src/common/xtensa_cpupause.c
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

#include "xtensa.h"
#include "sched/sched.h"

#ifdef CONFIG_SMP

/****************************************************************************
 * Private Data
 ****************************************************************************/

static spinlock_t g_cpu_wait[CONFIG_SMP_NCPUS];
static spinlock_t g_cpu_paused[CONFIG_SMP_NCPUS];

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
 *   true  = a pause request is pending.
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
 *   However, it is sometimes necessary to perform the pending pause
 *   operation in other contexts where the interrupt cannot be taken in
 *   order to avoid deadlocks.
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

  /* Copy the CURRENT_REGS into the OLD TCB (otcb).  The co-processor state
   * will be saved as part of the return from xtensa_irq_dispatch().
   */

  xtensa_savestate(tcb->xcp.regs);

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

  xtensa_restorestate(tcb->xcp.regs);

  spin_unlock(&g_cpu_wait[cpu]);
  return OK;
}

/****************************************************************************
 * Name: xtensa_pause_handler
 *
 * Description:
 *   This is the handler for CPU_INTCODE_PAUSE CPU interrupt.  This
 *   implements up_cpu_pause() by performing the following operations:
 *
 *   1. The current task state at the head of the current assigned task
 *      list was saved when the interrupt was entered.
 *   2. This function simply waits on a spinlock, then returns.
 *   3. Upon return, the interrupt exit logic will restore the state of
 *      the new task at the head of the ready to run list.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void xtensa_pause_handler(void)
{
  int cpu = up_cpu_index();

  /* Check for false alarms.  Such false could occur as a consequence of
   * some deadlock breaking logic that might have already serviced the
   * interrupt by calling up_cpu_paused.
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
 *   cpu - The index of the CPU to be stopped.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_cpu_pause(int cpu)
{
  int ret;

#ifdef CONFIG_SCHED_INSTRUMENTATION
  /* Notify of the pause event */

  sched_note_cpu_pause(this_task(), cpu);
#endif

  DEBUGASSERT(cpu >= 0 && cpu < CONFIG_SMP_NCPUS && cpu != this_cpu());

  /* Take both spinlocks.  The g_cpu_wait spinlock will prevent the interrupt
   * handler from returning until up_cpu_resume() is called; g_cpu_paused
   * is a handshake that will prevent this function from returning until
   * the CPU is actually paused.
   * Note that we might spin before getting g_cpu_wait, this just means that
   * the other CPU still hasn't finished responding to the previous resume
   * request.
   */

  DEBUGASSERT(!spin_islocked(&g_cpu_paused[cpu]));

  spin_lock(&g_cpu_wait[cpu]);
  spin_lock(&g_cpu_paused[cpu]);

  /* Execute the intercpu interrupt */

  ret = xtensa_intercpu_interrupt(cpu, CPU_INTCODE_PAUSE);
  if (ret < 0)
    {
      /* What happened?  Unlock the g_cpu_wait spinlock */

      spin_unlock(&g_cpu_wait[cpu]);
    }
  else
    {
      /* Wait for the other CPU to unlock g_cpu_paused meaning that
       * it is fully paused and ready for up_cpu_resume();
       */

      spin_lock(&g_cpu_paused[cpu]);
    }

  spin_unlock(&g_cpu_paused[cpu]);

  /* On successful return g_cpu_wait will be locked, the other CPU will be
   * spinning on g_cpu_wait and will not continue until g_cpu_resume() is
   * called.  g_cpu_paused will be unlocked in any case.
   */

  return ret;
}

/****************************************************************************
 * Name: up_cpu_resume
 *
 * Description:
 *   Restart the cpu after it was paused via up_cpu_pause(), restoring the
 *   state of the task at the head of the g_assignedtasks[cpu] list, and
 *   resume normal tasking.
 *
 *   This function is called after up_cpu_pause in order to resume operation
 *   of the CPU after modifying its g_assignedtasks[cpu] list.
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
#ifdef CONFIG_SCHED_INSTRUMENTATION
  /* Notify of the resume event */

  sched_note_cpu_resume(this_task(), cpu);
#endif

  DEBUGASSERT(cpu >= 0 && cpu < CONFIG_SMP_NCPUS && cpu != this_cpu());

  /* Release the spinlock.  Releasing the spinlock will cause the interrupt
   * handler on 'cpu' to continue and return from interrupt to the newly
   * established thread.
   */

  DEBUGASSERT(spin_islocked(&g_cpu_wait[cpu]) &&
              !spin_islocked(&g_cpu_paused[cpu]));

  spin_unlock(&g_cpu_wait[cpu]);
  return OK;
}

#endif /* CONFIG_SMP */
