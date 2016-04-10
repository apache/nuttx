/****************************************************************************
 * arch/sim/src/up_simsmp.c
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
#include <nuttx/spinlock.h>

#include "sched/sched.h"
#include "up_internal.h"

#ifdef CONFIG_SMP

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sim_cpu_pause
 *
 * Description:
 *   This is the SIGUSR1 signal handling logic.  It implements the core
 *   logic of up_cpu_pause() on the thread of execution the simulated CPU.
 *   This is the part of the implementation that must be performed in the
 *   NuttX vs. the host domain.
 *
 * Input Parameters:
 *   cpu    - The CPU being paused.
 *   wait   - Spinlock to wait on to be un-paused
 *   paused - A boolean to set when we are in the paused state.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sim_cpu_pause(int cpu, volatile spinlock_t *wait,
                   volatile unsigned char *paused)
{
  struct tcb_s *rtcb = current_task(cpu);

  /* Update scheduler parameters */

  sched_suspend_scheduler(rtcb);

  /* Copy the exception context into the TCB at the (old) head of the
   * CPUs assigned task list. if up_setjmp returns a non-zero value, then
   * this is really the previously running task restarting!
   */

  if (up_setjmp(rtcb->xcp.regs) == 0)
    {
      /* Indicate that we are in the paused state */

      *paused = 1;

      /* Spin until we are asked to resume.  When we resume, we need to
       * inicate that we are not longer paused.
       */

      spin_lock(wait);
      *paused = 0;

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
          sdbg("CPU%d: Delivering signals TCB=%p\n", cpu, rtcb);
          ((sig_deliver_t)rtcb->xcp.sigdeliver)(rtcb);
          rtcb->xcp.sigdeliver = NULL;
        }

      /* Reset scheduler parameters */

      sched_resume_scheduler(rtcb);

      /* Then switch contexts */

      up_longjmp(rtcb->xcp.regs, 1);
    }
}

#endif /* CONFIG_SMP */

