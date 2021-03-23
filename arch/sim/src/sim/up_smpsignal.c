/****************************************************************************
 * arch/sim/src/sim/up_smpsignal.c
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

#include <nuttx/sched.h>
#include <nuttx/sched_note.h>
#include <nuttx/spinlock.h>

#include "sched/sched.h"
#include "up_internal.h"

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
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sim_cpupause_handler
 *
 * Description:
 *   This is the SIGUSR signal handler.  It implements the core logic of
 *   up_cpu_pause() on the thread of execution the simulated CPU.
 *
 * Input Parameters:
 *   irq - the interrupt number
 *   context  - not used
 *   arg      - not used
 *
 * Returned Value:
 *   In case of success OK (0) is returned otherwise a negative value.
 *
 ****************************************************************************/

static int sim_cpupause_handler(int irq, FAR void *context, FAR void *arg)
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
  else
    {
      /* NOTE: sim specific logic
       * In the case of no pause request, call sim_timer_handler()
       */

      sim_timer_handler();
    }

  return OK;
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
  struct tcb_s *tcb = current_task(cpu);

  /* Update scheduler parameters */

  nxsched_suspend_scheduler(tcb);

#ifdef CONFIG_SCHED_INSTRUMENTATION
  /* Notify that we are paused */

  sched_note_cpu_paused(tcb);
#endif

  /* Save the current context at CURRENT_REGS into the TCB at the head
   * of the assigned task list for this CPU.
   */

  up_savestate(tcb->xcp.regs);

  /* Wait for the spinlock to be released */

  spin_unlock(&g_cpu_paused[cpu]);
  spin_lock(&g_cpu_wait[cpu]);

  /* Restore the exception context of the tcb at the (new) head of the
   * assigned task list.
   */

  tcb = current_task(cpu);

  /* The way that we handle signals in the simulation is kind of a
   * kludge.  This would be unsafe in a truly multi-threaded,
   * interrupt driven environment.
   */

#ifdef CONFIG_SCHED_INSTRUMENTATION
  /* Notify that we have resumed */

  sched_note_cpu_resumed(tcb);
#endif

  /* Reset scheduler parameters */

  nxsched_resume_scheduler(tcb);

  /* Then switch contexts.  Any necessary address environment changes
   * will be made when the interrupt returns.
   */

  up_restorestate(tcb->xcp.regs);
  spin_unlock(&g_cpu_wait[cpu]);

  return OK;
}

/****************************************************************************
 * Name: up_cpu_started
 *
 * Description:
 *   Notify the current cpu start successfully.
 *
 ****************************************************************************/

void up_cpu_started(void)
{
#ifdef CONFIG_SCHED_INSTRUMENTATION
  FAR struct tcb_s *tcb = this_task();

  /* Notify that this CPU has started */

  sched_note_cpu_started(tcb);

  /* Announce that the IDLE task has started */

  sched_note_start(tcb);
#endif
}

/****************************************************************************
 * Name: up_this_task
 *
 * Description:
 *   Return the currrent task tcb.
 *
 ****************************************************************************/

struct tcb_s *up_this_task(void)
{
  return this_task();
}

/****************************************************************************
 * Name: up_cpu_set_pause_handler
 *
 * Description:
 *   Attach the CPU pause request interrupt to the NuttX logic.
 *
 * Input Parameters:
 *   irq - the SIGUSR1 interrupt number
 *
 * Returned Value:
 *   On success returns OK (0), otherwise a negative value.
 ****************************************************************************/

int up_cpu_set_pause_handler(int irq)
{
  up_enable_irq(irq);
  return irq_attach(irq, sim_cpupause_handler, NULL);
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

  /* Take the both spinlocks.  The g_cpu_wait spinlock will prevent the
   * handler from returning until up_cpu_resume() is called; g_cpu_paused
   * is a handshake that will prefent this function from returning until
   * the CPU is actually paused.
   */

  DEBUGASSERT(!spin_islocked(&g_cpu_wait[cpu]) &&
              !spin_islocked(&g_cpu_paused[cpu]));

  spin_lock(&g_cpu_wait[cpu]);
  spin_lock(&g_cpu_paused[cpu]);

  /* Generate IRQ for CPU(cpu) */

  sim_send_ipi(cpu);

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

/****************************************************************************
 * Name: sim_sigdeliver
 ****************************************************************************/

void sim_sigdeliver(void)
{
  int cpu = this_cpu();
  struct tcb_s *rtcb = current_task(cpu);

  if (NULL == (rtcb->xcp.sigdeliver))
    {
      return;
    }

  /* NOTE: we enter critical section here for sim instead of
   * up_irq_enable() up_irq_save() used in other architectures
   */

  irqstate_t flags = enter_critical_section();

  /* Save the errno.  This must be preserved throughout the signal handling
   * so that the user code final gets the correct errno value (probably
   * EINTR).
   */

  int saved_errno = get_errno();

#ifdef CONFIG_SMP
  /* In the SMP case, we must terminate the critical section while the signal
   * handler executes, but we also need to restore the irqcount when the
   * we resume the main thread of the task.
   */

  int16_t saved_irqcount;
#endif

  sinfo("rtcb=%p sigdeliver=%p sigpendactionq.head=%p\n",
        rtcb, rtcb->xcp.sigdeliver, rtcb->sigpendactionq.head);
  DEBUGASSERT(rtcb->xcp.sigdeliver != NULL);

  /* NOTE: we do not save the return state for sim */

#ifdef CONFIG_SMP
  /* In the SMP case, up_schedule_sigaction(0) will have incremented
   * 'irqcount' in order to force us into a critical section.  Save the
   * pre-incremented irqcount.
   */

  saved_irqcount = rtcb->irqcount;
  DEBUGASSERT(saved_irqcount >= 0);

  /* Now we need call leave_critical_section() repeatedly to get the irqcount
   * to zero, freeing all global spinlocks that enforce the critical section.
   */

  do
    {
      leave_critical_section(flags);
    }
  while (rtcb->irqcount > 0);
#endif /* CONFIG_SMP */

  /* Deliver the signal */

  ((sig_deliver_t)rtcb->xcp.sigdeliver)(rtcb);

  /* Output any debug messages BEFORE restoring errno (because they may
   * alter errno), then disable interrupts again and restore the original
   * errno that is needed by the user logic (it is probably EINTR).
   *
   * I would prefer that all interrupts are disabled when
   * up_fullcontextrestore() is called, but that may not be necessary.
   */

  sinfo("Resuming\n");

#ifdef CONFIG_SMP
  /* Restore the saved 'irqcount' and recover the critical section
   * spinlocks.
   */

  DEBUGASSERT(rtcb->irqcount == 0);
  while (rtcb->irqcount < saved_irqcount)
    {
      enter_critical_section();
    }
#endif

  /* Restore the saved errno value */

  set_errno(saved_errno);

  /* Allows next handler to be scheduled */

  rtcb->xcp.sigdeliver = NULL;

  /* NOTE: we leave a critical section here for sim */

  leave_critical_section(flags);

  /* NOTE: we do not restore the return state for sim */
}
