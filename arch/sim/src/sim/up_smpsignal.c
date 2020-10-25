/****************************************************************************
 * arch/sim/src/sim/up_simsmp.c
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
  struct tcb_s *rtcb = current_task(cpu);

  /* Update scheduler parameters */

  nxsched_suspend_scheduler(rtcb);

#ifdef CONFIG_SCHED_INSTRUMENTATION
  /* Notify that we are paused */

  sched_note_cpu_paused(rtcb);
#endif

  /* Copy the exception context into the TCB at the (old) head of the
   * CPUs assigned task list. if up_setjmp returns a non-zero value, then
   * this is really the previously running task restarting!
   */

  if (up_setjmp(rtcb->xcp.regs) == 0)
    {
      /* Unlock the g_cpu_paused spinlock to indicate that we are in the
       * paused state
       */

      spin_unlock(&g_cpu_paused[cpu]);

      /* Spin until we are asked to resume.  When we resume, we need to
       * inicate that we are not longer paused.
       */

      spin_lock(&g_cpu_wait[cpu]);
      spin_unlock(&g_cpu_wait[cpu]);

      /* While we were paused, logic on a different CPU probably changed
       * the task as that head of the assigned task list.  So now we need
       * restore the exception context of the rtcb at the (new) head
       * of the assigned list in order to instantiate the new task.
       */

      rtcb = current_task(cpu);

      /* The way that we handle signals in the simulation is kind of a
       * kludge.  This would be unsafe in a truly multi-threaded,
       * interrupt driven environment.
       */

      if (rtcb->xcp.sigdeliver)
        {
          sinfo("CPU%d: Delivering signals TCB=%p\n", cpu, rtcb);
          ((sig_deliver_t)rtcb->xcp.sigdeliver)(rtcb);
          rtcb->xcp.sigdeliver = NULL;
        }

#ifdef CONFIG_SCHED_INSTRUMENTATION
      /* Notify that we have resumed */

      sched_note_cpu_resumed(rtcb);
#endif

      /* Reset scheduler parameters */

      nxsched_resume_scheduler(rtcb);

      /* Then switch contexts */

      up_longjmp(rtcb->xcp.regs, 1);
    }

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
