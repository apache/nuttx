/****************************************************************************
 * sched/signal/sig_timedwait.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

      if (wtcb->sigunbinfo != NULL)
        {
          wtcb->sigunbinfo->si_signo           = SIG_WAIT_TIMEOUT;
          wtcb->sigunbinfo->si_code            = SI_TIMER;
          wtcb->sigunbinfo->si_errno           = ETIMEDOUT;
          wtcb->sigunbinfo->si_value.sival_int = 0;
#ifdef CONFIG_SCHED_HAVE_PARENT
          wtcb->sigunbinfo->si_pid             = 0;  /* Not applicable */
          wtcb->sigunbinfo->si_status          = OK;
#endif
        }

      /* Remove the task from waitting list */

      dq_rem((FAR dq_entry_t *)wtcb, list_waitingforsignal());

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

      if (wtcb->sigunbinfo != NULL)
        {
          wtcb->sigunbinfo->si_signo           = SIG_CANCEL_TIMEOUT;
          wtcb->sigunbinfo->si_code            = SI_USER;
          wtcb->sigunbinfo->si_errno           = errcode;
          wtcb->sigunbinfo->si_value.sival_int = 0;
#ifdef CONFIG_SCHED_HAVE_PARENT
          wtcb->sigunbinfo->si_pid             = 0;  /* Not applicable */
          wtcb->sigunbinfo->si_status          = OK;
#endif
        }

      /* Remove the task from waitting list */

      dq_rem((FAR dq_entry_t *)wtcb, list_waitingforsignal());

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
 * Name: nxsig_clockwait
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
 *   clockid - The ID of the clock to be used to measure the timeout.
 *   flags   - Open flags.  TIMER_ABSTIME  is the only supported flag.
 *   rqtp - The amount of time to be suspended from execution.
 *   rmtp - If the rmtp argument is non-NULL, the timespec structure
 *          referenced by it is updated to contain the amount of time
 *          remaining in the interval (the requested time minus the time
 *          actually slept)
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   A negated errno value is returned on failure.
 *
 *   EAGAIN - wait time is zero.
 *   EINTR  - The wait was interrupted by an unblocked, caught signal.
 *
 * Notes:
 *  This function should be called with critical section set.
 *
 ****************************************************************************/

int nxsig_clockwait(int clockid, int flags,
                    FAR const struct timespec *rqtp,
                    FAR struct timespec *rmtp)
{
  FAR struct tcb_s *rtcb = this_task();
  irqstate_t iflags;
  clock_t expect = 0;
  clock_t stop;

  if (rqtp && (rqtp->tv_nsec < 0 || rqtp->tv_nsec >= 1000000000))
    {
      return -EINVAL;
    }

  /* If rqtp is zero, yield CPU and return
   * Notice: The behavior of sleep(0) is not defined in POSIX, so there are
   * different implementations:
   * 1. In Linux, nanosleep(0) will call schedule() to yield CPU:
   *    https://elixir.bootlin.com/linux/latest/source/kernel/time/
   *    hrtimer.c#L2038
   * 2. In BSD, nanosleep(0) will return immediately:
   *    https://github.com/freebsd/freebsd-src/blob/
   *    475fa89800086718bd9249fd4dc3f862549f1f78/crypto/openssh/
   *    openbsd-compat/bsd-misc.c#L243
   */

  if (rqtp && rqtp->tv_sec == 0 && rqtp->tv_nsec == 0)
    {
      sched_yield();
      return -EAGAIN;
    }

#ifdef CONFIG_CANCELLATION_POINTS
  /* nxsig_clockwait() is not a cancellation point, but it may be called
   * from a cancellation point.  So if a cancellation is pending, we
   * must exit immediately without waiting.
   */

  if (check_cancellation_point())
    {
      /* If there is a pending cancellation, then do not perform
       * the wait.  Exit now with ECANCELED.
       */

      return -ECANCELED;
    }
#endif

  iflags = enter_critical_section();

  if (rqtp)
    {
      /* Start the watchdog timer */

      if ((flags & TIMER_ABSTIME) == 0)
        {
          expect = clock_systime_ticks() + clock_time2ticks(rqtp);
          wd_start_abstick(&rtcb->waitdog, expect,
                           nxsig_timeout, (uintptr_t)rtcb);
        }
      else if (clockid == CLOCK_REALTIME)
        {
          wd_start_realtime(&rtcb->waitdog, rqtp,
                            nxsig_timeout, (uintptr_t)rtcb);
        }
      else
        {
          wd_start_abstime(&rtcb->waitdog, rqtp,
                           nxsig_timeout, (uintptr_t)rtcb);
        }
    }

  /* Remove the tcb task from the ready-to-run list. */

  nxsched_remove_self(rtcb);

  /* Add the task to the specified blocked task list */

  rtcb->task_state = TSTATE_WAIT_SIG;
  dq_addlast((FAR dq_entry_t *)rtcb, &g_waitingforsignal);

  /* Now, perform the context switch if one is needed */

  up_switch_context(this_task(), rtcb);

  /* We no longer need the watchdog */

  if (rqtp)
    {
      wd_cancel(&rtcb->waitdog);
      stop = clock_systime_ticks();
    }

  leave_critical_section(iflags);

  if (rqtp && rmtp && expect)
    {
      clock_ticks2time(rmtp, expect > stop ? expect - stop : 0);
    }

  return 0;
}

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
  siginfo_t unbinfo;
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

  intersection = nxsig_pendingset(rtcb);
  sigandset(&intersection, &intersection, set);
  if (!sigisemptyset(&intersection))
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
    }

  /* We will have to wait for a signal to be posted to this task. */

  else
    {
      rtcb->sigunbinfo = (info == NULL) ? &unbinfo : info;

      /* Save the set of pending signals to wait for */

      rtcb->sigwaitmask = *set;

      leave_critical_section(flags);

      ret = nxsig_clockwait(CLOCK_REALTIME, 0, timeout, NULL);
      if (ret < 0)
        {
          rtcb->sigunbinfo = NULL;
          return ret;
        }

      flags = enter_critical_section();

      /* We are running again, clear the sigwaitmask */

      sigemptyset(&rtcb->sigwaitmask);

      /* When we awaken, the cause will be in the TCB.  Get the signal number
       * or timeout) that awakened us.
       */

      if (GOOD_SIGNO(rtcb->sigunbinfo->si_signo))
        {
          /* We were awakened by a signal... but is it one of the signals
           * that we were waiting for?
           */

          if (nxsig_ismember(set, rtcb->sigunbinfo->si_signo))
            {
              /* Yes.. the return value is the number of the signal that
               * awakened us.
               */

              ret = rtcb->sigunbinfo->si_signo;
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
          if (rtcb->sigunbinfo->si_signo == SIG_CANCEL_TIMEOUT)
            {
              /* The wait was canceled */

              ret = -rtcb->sigunbinfo->si_errno;
              DEBUGASSERT(ret < 0);
            }
          else
#endif
            {
              /* We were awakened by a timeout.  Set EAGAIN and return an
               * error.
               */

              DEBUGASSERT(rtcb->sigunbinfo->si_signo == SIG_WAIT_TIMEOUT);
              ret = -EAGAIN;
            }
        }

      rtcb->sigunbinfo = NULL;
    }

  leave_critical_section(flags);

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
