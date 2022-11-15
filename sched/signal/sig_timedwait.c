/****************************************************************************
 * sched/signal/sig_timedwait.c
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
#include <nuttx/compiler.h>

#include <stdint.h>
#include <string.h>
#include <signal.h>
#include <time.h>
#include <assert.h>
#include <debug.h>
#include <sched.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/wdog.h>
#include <nuttx/signal.h>
#include <nuttx/cancelpt.h>
#include <nuttx/queue.h>

#include "sched/sched.h"
#include "signal/signal.h"
#include "clock/clock.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* These are special values of si_signo that mean that either the wait was
 * awakened with a timeout, or the wait was canceled... not the receipt of a
 * signal.
 */

#define SIG_CANCEL_TIMEOUT 0xfe
#define SIG_WAIT_TIMEOUT   0xff

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsig_timeout
 *
 * Description:
 *   A timeout elapsed while waiting for signals to be queued.
 *
 * Assumptions:
 *   This function executes in the context of the timer interrupt handler.
 *   Local interrupts are assumed to be disabled on entry.
 *
 ****************************************************************************/

static void nxsig_timeout(wdparm_t arg)
{
  FAR struct tcb_s *wtcb = (FAR struct tcb_s *)(uintptr_t)arg;
#ifdef CONFIG_SMP
  irqstate_t flags;

  /* We must be in a critical section in order to call up_switch_context()
   * below.  If we are running on a single CPU architecture, then we know
   * interrupts a disabled an there is no need to explicitly call
   * enter_critical_section().  However, in the SMP case,
   * enter_critical_section() does much more than just disable interrupts on
   * the local CPU; it also manages spinlocks to assure the stability of the
   * TCB that we are manipulating.
   */

  flags = enter_critical_section();
#endif

  /* There may be a race condition -- make sure the task is
   * still waiting for a signal
   */

  if (wtcb->task_state == TSTATE_WAIT_SIG)
    {
      FAR struct tcb_s *rtcb = this_task();

      wtcb->sigunbinfo.si_signo           = SIG_WAIT_TIMEOUT;
      wtcb->sigunbinfo.si_code            = SI_TIMER;
      wtcb->sigunbinfo.si_errno           = ETIMEDOUT;
      wtcb->sigunbinfo.si_value.sival_int = 0;
#ifdef CONFIG_SCHED_HAVE_PARENT
      wtcb->sigunbinfo.si_pid             = 0;  /* Not applicable */
      wtcb->sigunbinfo.si_status          = OK;
#endif

      /* Remove the task from waitting list */

      dq_rem((FAR dq_entry_t *)wtcb, &g_waitingforsignal);

      /* Add the task to ready-to-run task list, and
       * perform the context switch if one is needed
       */

      if (nxsched_add_readytorun(wtcb))
        {
          up_switch_context(wtcb, rtcb);
        }
    }

#ifdef CONFIG_SMP
  leave_critical_section(flags);
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsig_wait_irq
 *
 * Description:
 *   An error event has occurred and the signal wait must be terminated with
 *   an error.
 *
 ****************************************************************************/

#ifdef CONFIG_CANCELLATION_POINTS
void nxsig_wait_irq(FAR struct tcb_s *wtcb, int errcode)
{
#ifdef CONFIG_SMP
  irqstate_t flags;

  /* We must be in a critical section in order to call up_switch_context()
   * below.  If we are running on a single CPU architecture, then we know
   * interrupts a disabled an there is no need to explicitly call
   * enter_critical_section().  However, in the SMP case,
   * enter_critical_section() does much more than just disable interrupts on
   * the local CPU; it also manages spinlocks to assure the stability of the
   * TCB that we are manipulating.
   */

  flags = enter_critical_section();
#endif

  /* There may be a race condition -- make sure the task is
   * still waiting for a signal
   */

  if (wtcb->task_state == TSTATE_WAIT_SIG)
    {
      FAR struct tcb_s *rtcb = this_task();

      wtcb->sigunbinfo.si_signo           = SIG_CANCEL_TIMEOUT;
      wtcb->sigunbinfo.si_code            = SI_USER;
      wtcb->sigunbinfo.si_errno           = errcode;
      wtcb->sigunbinfo.si_value.sival_int = 0;
#ifdef CONFIG_SCHED_HAVE_PARENT
      wtcb->sigunbinfo.si_pid             = 0;  /* Not applicable */
      wtcb->sigunbinfo.si_status          = OK;
#endif

      /* Remove the task from waitting list */

      dq_rem((FAR dq_entry_t *)wtcb, &g_waitingforsignal);

      /* Add the task to ready-to-run task list, and
       * perform the context switch if one is needed
       */

      if (nxsched_add_readytorun(wtcb))
        {
          up_switch_context(wtcb, rtcb);
        }
    }

#ifdef CONFIG_SMP
  leave_critical_section(flags);
#endif
}
#endif /* CONFIG_CANCELLATION_POINTS */

/****************************************************************************
 * Name: nxsig_timedwait
 *
 * Description:
 *   This function selects the pending signal set specified by the argument
 *   set.  If multiple signals are pending in set, it will remove and return
 *   the lowest numbered one.  If no signals in set are pending at the time
 *   of the call, the calling process will be suspended until one of the
 *   signals in set becomes pending, OR until the process is interrupted by
 *   an unblocked signal, OR until the time interval specified by timeout
 *   (if any), has expired. If timeout is NULL, then the timeout interval
 *   is forever.
 *
 *   If the info argument is non-NULL, the selected signal number is stored
 *   in the si_signo member and the cause of the signal is stored in the
 *   si_code member.  The content of si_value is only meaningful if the
 *   signal was generated by sigqueue() (or nxsig_queue).
 *
 *   This is an internal OS interface.  It is functionally equivalent to
 *   sigtimedwait() except that:
 *
 *   - It is not a cancellation point, and
 *   - It does not modify the errno value.
 *
 * Input Parameters:
 *   set     - The pending signal set.
 *   info    - The returned value (may be NULL).
 *   timeout - The amount of time to wait (may be NULL)
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 *   EAGAIN - No signal specified by set was generated within the specified
 *            timeout period.
 *   EINTR  - The wait was interrupted by an unblocked, caught signal.
 *
 ****************************************************************************/

int nxsig_timedwait(FAR const sigset_t *set, FAR struct siginfo *info,
                    FAR const struct timespec *timeout)
{
  FAR struct tcb_s *rtcb = this_task();
  sigset_t intersection;
  FAR sigpendq_t *sigpend;
  irqstate_t flags;
  int32_t waitticks;
  bool switch_needed;
  int ret;

  DEBUGASSERT(set != NULL);

  /* Several operations must be performed below:  We must determine if any
   * signal is pending and, if not, wait for the signal.  Since signals can
   * be posted from the interrupt level, there is a race condition that
   * can only be eliminated by disabling interrupts!
   */

  flags = enter_critical_section();

  /* Check if there is a pending signal corresponding to one of the
   * signals in the pending signal set argument.
   */

  intersection = *set & nxsig_pendingset(rtcb);
  if (intersection != NULL_SIGNAL_SET)
    {
      /* One or more of the signals in intersections is sufficient to cause
       * us to not wait.  Pick the lowest numbered signal and mark it not
       * pending.
       */

      sigpend = nxsig_remove_pendingsignal(rtcb,
                                           nxsig_lowest(&intersection));
      DEBUGASSERT(sigpend);

      /* Return the signal info to the caller if so requested */

      if (info != NULL)
        {
          memcpy(info, &sigpend->info, sizeof(struct siginfo));
        }

      /* The return value is the number of the signal that awakened us */

      ret = sigpend->info.si_signo;

      /* Then dispose of the pending signal structure properly */

      nxsig_release_pendingsignal(sigpend);
      leave_critical_section(flags);
    }

  /* We will have to wait for a signal to be posted to this task. */

  else
    {
#ifdef CONFIG_CANCELLATION_POINTS
      /* nxsig_timedwait() is not a cancellation point, but it may be called
       * from a cancellation point.  So if a cancellation is pending, we
       * must exit immediately without waiting.
       */

      if (check_cancellation_point())
        {
          /* If there is a pending cancellation, then do not perform
           * the wait.  Exit now with ECANCELED.
           */

          leave_critical_section(flags);
          return -ECANCELED;
        }
#endif

      /* Check if we should wait for the timeout */

      if (timeout != NULL)
        {
          /* Convert the timespec to system clock ticks, making sure that
           * the resulting delay is greater than or equal to the requested
           * time in nanoseconds.
           */

#ifdef CONFIG_HAVE_LONG_LONG
          uint64_t waitticks64 = ((uint64_t)timeout->tv_sec * NSEC_PER_SEC +
                                  (uint64_t)timeout->tv_nsec +
                                  NSEC_PER_TICK - 1) /
                                 NSEC_PER_TICK;
          DEBUGASSERT(waitticks64 <= UINT32_MAX);
          waitticks = (uint32_t)waitticks64;
#else
          uint32_t waitmsec;

          DEBUGASSERT(timeout->tv_sec < UINT32_MAX / MSEC_PER_SEC);
          waitmsec = timeout->tv_sec * MSEC_PER_SEC +
                     (timeout->tv_nsec + NSEC_PER_MSEC - 1) / NSEC_PER_MSEC;
          waitticks = MSEC2TICK(waitmsec);
#endif

          if (waitticks > 0)
            {
              /* Save the set of pending signals to wait for */

              rtcb->sigwaitmask = *set;

              /* Start the watchdog */

              wd_start(&rtcb->waitdog, waitticks,
                       nxsig_timeout, (uintptr_t)rtcb);

              /* Now wait for either the signal or the watchdog, but
               * first, make sure this is not the idle task,
               * descheduling that isn't going to end well.
               */

              DEBUGASSERT(!is_idle_task(rtcb));

              /* Remove the tcb task from the ready-to-run list. */

              switch_needed = nxsched_remove_readytorun(rtcb, true);

              /* Add the task to the specified blocked task list */

              rtcb->task_state = TSTATE_WAIT_SIG;
              dq_addlast((FAR dq_entry_t *)rtcb, &g_waitingforsignal);

              /* Now, perform the context switch if one is needed */

              if (switch_needed)
                {
                  up_switch_context(this_task(), rtcb);
                }

              /* We no longer need the watchdog */

              wd_cancel(&rtcb->waitdog);
            }
          else
            {
              leave_critical_section(flags);
              return -EAGAIN;
            }
        }

      /* No timeout, just wait */

      else
        {
          /* Save the set of pending signals to wait for */

          rtcb->sigwaitmask = *set;

          /* And wait until one of the unblocked signals is posted,
           * but first make sure this is not the idle task,
           * descheduling that isn't going to end well.
           */

          DEBUGASSERT(!is_idle_task(rtcb));

          /* Remove the tcb task from the ready-to-run list. */

          switch_needed = nxsched_remove_readytorun(rtcb, true);

          /* Add the task to the specified blocked task list */

          rtcb->task_state = TSTATE_WAIT_SIG;
          dq_addlast((FAR dq_entry_t *)rtcb, &g_waitingforsignal);

          /* Now, perform the context switch if one is needed */

          if (switch_needed)
            {
              up_switch_context(this_task(), rtcb);
            }
        }

      /* We are running again, clear the sigwaitmask */

      rtcb->sigwaitmask = NULL_SIGNAL_SET;

      /* When we awaken, the cause will be in the TCB.  Get the signal number
       * or timeout) that awakened us.
       */

      if (GOOD_SIGNO(rtcb->sigunbinfo.si_signo))
        {
          /* We were awakened by a signal... but is it one of the signals
           * that we were waiting for?
           */

          if (nxsig_ismember(set, rtcb->sigunbinfo.si_signo))
            {
              /* Return the signal info to the caller if so requested */

              if (info != NULL)
                {
                  memcpy(info, &rtcb->sigunbinfo, sizeof(struct siginfo));
                }

              /* Yes.. the return value is the number of the signal that
               * awakened us.
               */

              ret = rtcb->sigunbinfo.si_signo;
            }
          else
            {
              /* No... then report the EINTR error */

              ret = -EINTR;
            }
        }
      else
        {
          /* Otherwise, we must have been awakened by the timeout or,
           * perhaps, the wait was cancelled.
           */

#ifdef CONFIG_CANCELLATION_POINTS
          if (rtcb->sigunbinfo.si_signo == SIG_CANCEL_TIMEOUT)
            {
              /* The wait was canceled */

              ret = -rtcb->sigunbinfo.si_errno;
              DEBUGASSERT(ret < 0);
            }
          else
#endif
            {
              /* We were awakened by a timeout.  Set EAGAIN and return an
               * error.
               */

              DEBUGASSERT(rtcb->sigunbinfo.si_signo == SIG_WAIT_TIMEOUT);
              ret = -EAGAIN;
            }
        }

      leave_critical_section(flags);
    }

  return ret;
}

/****************************************************************************
 * Name: sigtimedwait
 *
 * Description:
 *   This function selects the pending signal set specified by the argument
 *   set.  If multiple signals are pending in set, it will remove and return
 *   the lowest numbered one.  If no signals in set are pending at the time
 *   of the call, the calling process will be suspended until one of the
 *   signals in set becomes pending, OR until the process is interrupted by
 *   an unblocked signal, OR until the time interval specified by timeout
 *   (if any), has expired. If timeout is NULL, then the timeout interval
 *   is forever.
 *
 *   If the info argument is non-NULL, the selected signal number is stored
 *   in the si_signo member and the cause of the signal is stored in the
 *   si_code member.  The content of si_value is only meaningful if the
 *   signal was generated by sigqueue().
 *
 *   The following values for si_code are defined in signal.h:
 *     SI_USER    - Signal sent from kill, raise, or abort
 *     SI_QUEUE   - Signal sent from sigqueue
 *     SI_TIMER   - Signal is result of timer expiration
 *     SI_ASYNCIO - Signal is the result of asynch IO completion
 *     SI_MESGQ   - Signal generated by arrival of a message on an
 *                  empty message queue.
 *
 * Input Parameters:
 *   set     - The pending signal set.
 *   info    - The returned value (may be NULL).
 *   timeout - The amount of time to wait (may be NULL)
 *
 * Returned Value:
 *   Signal number that cause the wait to be terminated, otherwise -1 (ERROR)
 *   is returned with errno set to either:
 *
 *   EAGAIN - No signal specified by set was generated within the specified
 *            timeout period.
 *   EINTR  - The wait was interrupted by an unblocked, caught signal.
 *
 ****************************************************************************/

int sigtimedwait(FAR const sigset_t *set, FAR struct siginfo *info,
                 FAR const struct timespec *timeout)
{
  int ret;

  /* sigtimedwait() is a cancellation point */

  enter_cancellation_point();

  /* Let nxsig_timedwait() do the work. */

  ret = nxsig_timedwait(set, info, timeout);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  leave_cancellation_point();
  return ret;
}
