/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_cpupause.c
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

#include "sched/sched.h"
#include "arm_internal.h"
#include "hardware/cxd5602_memorymap.h"

#ifdef CONFIG_SMP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if 0
#  define DPRINTF(fmt, args...) _err(fmt, ##args)
#else
#  define DPRINTF(fmt, args...) do {} while (0)
#endif

#define CXD56_CPU_P2_INT        (CXD56_SWINT_BASE + 0x8)  /* for APP_DSP0 */

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
static volatile spinlock_t g_cpu_resumed[CONFIG_SMP_NCPUS];

static volatile int g_irq_to_handle[CONFIG_SMP_NCPUS][2];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: handle_irqreq
 *
 * Description:
 *   If an irq handling request is found on cpu, call up_enable_irq() or
 *   up_disable_irq(), then return true.
 *
 * Input Parameters:
 *   cpu - The index of the CPU to be queried
 *
 * Returned Value:
 *   true  = an irq handling request is found
 *   false = no irq handling request is found
 *
 ****************************************************************************/

static bool handle_irqreq(int cpu)
{
  int i;
  bool handled = false;

  /* Check both cases */

  for (i = 0; i < 2; i++)
    {
      int irqreq = g_irq_to_handle[cpu][i];

      if (irqreq)
        {
          /* Unlock the spinlock first */

          spin_unlock(&g_cpu_paused[cpu]);

          /* Then wait for the spinlock to be released */

          spin_lock(&g_cpu_wait[cpu]);

          /* Clear g_irq_to_handle[cpu][i] */

          g_irq_to_handle[cpu][i] = 0;

          if (0 == i)
            {
              up_enable_irq(irqreq);
            }
          else
            {
              up_disable_irq(irqreq);
            }

          /* Finally unlock the spinlock */

          spin_unlock(&g_cpu_wait[cpu]);
          handled = true;

          break;
        }
    }

  return handled;
}

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
  /* Fistly, check if this IPI is to enable/disable IRQ */

  if (handle_irqreq(cpu))
    {
      return OK;
    }

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

  /* Wait for the spinlock to be released */

  spin_unlock(&g_cpu_paused[cpu]);

  /* Ensure the CPU has been resumed to avoid causing a deadlock */

  spin_lock(&g_cpu_resumed[cpu]);

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
  spin_unlock(&g_cpu_resumed[cpu]);

  return OK;
}

/****************************************************************************
 * Name: arm_pause_handler
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

int arm_pause_handler(int irq, void *c, void *arg)
{
  int cpu = up_cpu_index();
  int ret = OK;

  DPRINTF("cpu%d will be paused\n", cpu);

  /* Clear SW_INT for APP_DSP(cpu) */

  putreg32(0, CXD56_CPU_P2_INT + (4 * cpu));

  /* Check for false alarms.  Such false could occur as a consequence of
   * some deadlock breaking logic that might have already serviced the SG2
   * interrupt by calling up_cpu_paused.
   */

  if (up_cpu_pausereq(cpu))
    {
      /* NOTE: The following enter_critical_section() would call
       * up_cpu_paused() to process a pause request to break a deadlock
       * because the caller held a critical section. Once up_cpu_paused()
       * finished, the caller will proceed and release the g_cpu_irqlock.
       * Then this CPU will acquire g_cpu_irqlock in the function.
       */

      irqstate_t flags = enter_critical_section();

      /* NOTE: Normally, we do not call up_cpu_paused() here because
       * the above enter_critical_setion() would call up_cpu_paused()
       * inside because the caller holds a crtical section.
       * However, cxd56's remote IRQ control logic also uses this handler
       * and a caller might not take a critical section to avoid a deadlock
       * during up_enable_irq() and up_disable_irq(). This is allowed
       * because IRQ control logic does not interact wtih the scheduler.
       * This means that if the request was not handled above, we need
       * to call up_cpu_paused() here again.
       */

      if (up_cpu_pausereq(cpu))
        {
          ret = up_cpu_paused(cpu);
        }

      leave_critical_section(flags);
    }

  return ret;
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

  DEBUGASSERT(cpu >= 0 && cpu < CONFIG_SMP_NCPUS && cpu != this_cpu());

#ifdef CONFIG_SCHED_INSTRUMENTATION
  /* Notify of the pause event */

  sched_note_cpu_pause(this_task(), cpu);
#endif

  /* Take the both spinlocks.  The g_cpu_wait spinlock will prevent the
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

  /* Generate IRQ for CPU(cpu) */

  putreg32(1, CXD56_CPU_P2_INT + (4 * cpu));

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
  DPRINTF("cpu=%d\n", cpu);

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

  /* Ensure the CPU has been resumed to avoid causing a deadlock */

  spin_lock(&g_cpu_resumed[cpu]);

  spin_unlock(&g_cpu_resumed[cpu]);
  return OK;
}

/****************************************************************************
 * Name: up_send_irqreq()
 *
 * Description:
 *   Send up_enable_irq() / up_disable_irq() request to the specified cpu
 *
 *   This function is called from up_enable_irq() or up_disable_irq()
 *   to be handled on specified CPU. Locking protocol in the sequence is
 *   the same as up_pause_cpu() plus up_resume_cpu().
 *
 * Input Parameters:
 *   idx - The request index (0: enable, 1: disable)
 *   irq - The IRQ number to be handled
 *   cpu - The index of the CPU which will handle the request
 *
 ****************************************************************************/

void up_send_irqreq(int idx, int irq, int cpu)
{
  DEBUGASSERT(cpu >= 0 && cpu < CONFIG_SMP_NCPUS && cpu != this_cpu());

  /* Wait for the spinlocks to be released */

  spin_lock(&g_cpu_wait[cpu]);
  spin_lock(&g_cpu_paused[cpu]);

  /* Set irq for the cpu */

  g_irq_to_handle[cpu][idx] = irq;

  /* Generate IRQ for CPU(cpu) */

  putreg32(1, CXD56_CPU_P2_INT + (4 * cpu));

  /* Wait for the handler is executed on cpu */

  spin_lock(&g_cpu_paused[cpu]);
  spin_unlock(&g_cpu_paused[cpu]);

  /* Finally unlock the spinlock to proceed the handler */

  spin_unlock(&g_cpu_wait[cpu]);

  /* Ensure the CPU has been resumed to avoid causing a deadlock */

  spin_lock(&g_cpu_resumed[cpu]);

  spin_unlock(&g_cpu_resumed[cpu]);
}

#endif /* CONFIG_SMP */
