/****************************************************************************
 * sched/sched/sched_unlock.c
 *
 *   Copyright (C) 2007, 2009, 2014, 2016, 2018 Gregory Nutt. All rights
 *     reserved.
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

#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/arch.h>
#include <nuttx/sched_note.h>

#include "irq/irq.h"
#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  sched_unlock
 *
 * Description:
 *   This function decrements the preemption lock count.  Typically this
 *   is paired with sched_lock() and concludes a critical section of
 *   code.  Preemption will not be unlocked until sched_unlock() has
 *   been called as many times as sched_lock().  When the lockcount is
 *   decremented to zero, any tasks that were eligible to preempt the
 *   current task will execute.
 *
 ****************************************************************************/

#ifdef CONFIG_SMP

int sched_unlock(void)
{
  FAR struct tcb_s *rtcb;
  int cpu;

  /* This operation is safe because the scheduler is locked and no context
   * switch may occur.
   */

  cpu  = this_cpu();
  rtcb = current_task(cpu);

  /* Check for some special cases:  (1) rtcb may be NULL only during
   * early boot-up phases, and (2) sched_unlock() should have no
   * effect if called from the interrupt level.
   */

  if (rtcb != NULL && !up_interrupt_context())
    {
      /* Prevent context switches throughout the following. */

      irqstate_t flags = enter_critical_section();

      /* Decrement the preemption lock counter */

      if (rtcb->lockcount > 0)
        {
          rtcb->lockcount--;
        }

      /* Check if the lock counter has decremented to zero.  If so,
       * then pre-emption has been re-enabled.
       */

      if (rtcb->lockcount <= 0)
        {
#ifdef CONFIG_SCHED_INSTRUMENTATION_PREEMPTION
          /* Note that we no longer have pre-emption disabled. */

          sched_note_premption(rtcb, false);
#endif
          /* Set the lock count to zero */

          rtcb->lockcount = 0;

          /* The lockcount has decremented to zero and we need to perform
           * release our hold on the lock.
           */

          DEBUGASSERT(g_cpu_schedlock == SP_LOCKED &&
                      (g_cpu_lockset & (1 << cpu)) != 0);

          spin_clrbit(&g_cpu_lockset, cpu, &g_cpu_locksetlock,
                      &g_cpu_schedlock);

          /* Release any ready-to-run tasks that have collected in
           * g_pendingtasks.
           *
           * NOTE: This operation has a very high likelihood of causing
           * this task to be switched out!
           */

          /* In the SMP case, the tasks remains pend(1) if we are
           * in a critical section, i.e., g_cpu_irqlock is locked by other
           * CPUs, or (2) other CPUs still have pre-emption disabled, i.e.,
           * g_cpu_schedlock is locked.  In those cases, the release of the
           * pending tasks must be deferred until those conditions are met.
           *
           * There are certain conditions that we must avoid by preventing
           * releasing the pending tasks while within the critical section
           * of other CPUs.  This logic does that and there is matching
           * logic in sched_addreadytorun to avoid starting new tasks within
           * the critical section (unless the CPU is the holder of the lock).
           *
           * REVISIT: If this CPU is only one that holds the IRQ lock, then
           * we should go ahead and release the pending tasks.  See the logic
           * leave_critical_section():  It will call up_release_pending()
           * BEFORE it clears IRQ lock.
           */

          if (!sched_islocked_global() && !irq_cpu_locked(cpu) &&
              g_pendingtasks.head != NULL)
            {
              up_release_pending();
            }

#if CONFIG_RR_INTERVAL > 0
          /* If (1) the task that was running supported round-robin
           * scheduling and (2) if its time slice has already expired, but
           * (3) it could not slice out because pre-emption was disabled,
           * then we need to swap the task out now and reassess the interval
           * timer for the next time slice.
           */

          if ((rtcb->flags & TCB_FLAG_POLICY_MASK) == TCB_FLAG_SCHED_RR &&
              rtcb->timeslice == 0)
            {
              /* Yes.. that is the situation.  But one more thing.  The call
               * to up_release_pending() above may have actually replaced
               * the task at the head of the read-to-run list.  In that case,
               * we need only to reset the timeslice value back to the
               * maximum.
               */

              if (rtcb != current_task(cpu))
                {
                  rtcb->timeslice = MSEC2TICK(CONFIG_RR_INTERVAL);
                }
#ifdef CONFIG_SCHED_TICKLESS
              else
                {
                  sched_timer_reassess();
                }
#endif
            }
#endif

#ifdef CONFIG_SCHED_SPORADIC
#if CONFIG_RR_INTERVAL > 0
          else
#endif
          /* If (1) the task that was running supported sporadic scheduling
           * and (2) if its budget slice has already expired, but (3) it
           * could not slice out because pre-emption was disabled, then we
           * need to swap the task out now and reassess the interval timer
           * for the next time slice.
           */

          if ((rtcb->flags & TCB_FLAG_POLICY_MASK) == TCB_FLAG_SCHED_SPORADIC &&
              rtcb->timeslice < 0)
            {
              /* Yes.. that is the situation.  Force the low-priority state
               * now
               */

              sched_sporadic_lowpriority(rtcb);

#ifdef CONFIG_SCHED_TICKLESS
              /* Make sure that the call to up_release_pending() did not
               * change the currently active task.
               */

              if (rtcb == current_task(cpu))
                {
                  sched_timer_reassess();
                }
#endif
            }
#endif
        }

      leave_critical_section(flags);
    }

  return OK;
}

#else /* CONFIG_SMP */

int sched_unlock(void)
{
  FAR struct tcb_s *rtcb = this_task();

  /* Check for some special cases:  (1) rtcb may be NULL only during
   * early boot-up phases, and (2) sched_unlock() should have no
   * effect if called from the interrupt level.
   */

  if (rtcb != NULL && !up_interrupt_context())
    {
      /* Prevent context switches throughout the following. */

      irqstate_t flags = enter_critical_section();

      /* Decrement the preemption lock counter */

      if (rtcb->lockcount > 0)
        {
          rtcb->lockcount--;
        }

      /* Check if the lock counter has decremented to zero.  If so,
       * then pre-emption has been re-enabled.
       */

      if (rtcb->lockcount <= 0)
        {
#ifdef CONFIG_SCHED_INSTRUMENTATION_PREEMPTION
          /* Note that we no longer have pre-emption disabled. */

          sched_note_premption(rtcb, false);
#endif
          /* Set the lock count to zero */

          rtcb->lockcount = 0;

          /* Release any ready-to-run tasks that have collected in
           * g_pendingtasks.
           *
           * NOTE: This operation has a very high likelihood of causing
           * this task to be switched out!
           *
           * In the single CPU case, decrementing irqcount to zero is
           * sufficient to release the pending tasks.  Further, in that
           * configuration, critical sections and pre-emption can operate
           * fully independently.
           */

          if (g_pendingtasks.head != NULL)
            {
              up_release_pending();
            }

#if CONFIG_RR_INTERVAL > 0
          /* If (1) the task that was running supported round-robin
           * scheduling and (2) if its time slice has already expired, but
           * (3) it could not slice out because pre-emption was disabled,
           * then we need to swap the task out now and reassess the interval
           * timer for the next time slice.
           */

          if ((rtcb->flags & TCB_FLAG_POLICY_MASK) == TCB_FLAG_SCHED_RR &&
              rtcb->timeslice == 0)
            {
              /* Yes.. that is the situation.  But one more thing.  The call
               * to up_release_pending() above may have actually replaced
               * the task at the head of the read-to-run list.  In that case,
               * we need only to reset the timeslice value back to the
               * maximum.
               */

              if (rtcb != this_task())
                {
                  rtcb->timeslice = MSEC2TICK(CONFIG_RR_INTERVAL);
                }
#ifdef CONFIG_SCHED_TICKLESS
              else
                {
                  sched_timer_reassess();
                }
#endif
            }
#endif

#ifdef CONFIG_SCHED_SPORADIC
#if CONFIG_RR_INTERVAL > 0
          else
#endif
          /* If (1) the task that was running supported sporadic scheduling
           * and (2) if its budget slice has already expired, but (3) it
           * could not slice out because pre-emption was disabled, then we
           * need to swap the task out now and reassess the interval timer
           * for the next time slice.
           */

          if ((rtcb->flags & TCB_FLAG_POLICY_MASK) == TCB_FLAG_SCHED_SPORADIC &&
              rtcb->timeslice < 0)
            {
              /* Yes.. that is the situation.  Force the low-priority state
               * now
               */

              sched_sporadic_lowpriority(rtcb);

#ifdef CONFIG_SCHED_TICKLESS
              /* Make sure that the call to up_release_pending() did not
               * change the currently active task.
               */

              if (rtcb == this_task())
                {
                  sched_timer_reassess();
                }
#endif
            }
#endif
        }

      leave_critical_section(flags);
    }

  return OK;
}

#endif /* CONFIG_SMP */
