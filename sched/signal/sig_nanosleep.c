/****************************************************************************
 * sched/signal/sig/nanosleep.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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

#include <time.h>
#include <signal.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/clock.h>
#include <arch/irq.h>

#include "clock/clock.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Global Variables
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nanosleep
 *
 * Description:
 *   The nanosleep() function causes the current thread to be suspended from
 *   execution until either the time interval specified by the rqtp argument
 *   has elapsed or a signal is delivered to the calling thread and its
 *   action is to invoke a signal-catching function or to terminate the
 *   process. The suspension time may be longer than requested because the
 *   argument value is rounded up to an integer multiple of the sleep
 *   resolution or because of the scheduling of other activity by the
 *   system. But, except for the case of being interrupted by a signal, the
 *   suspension time will not be less than the time specified by rqtp, as
 *   measured by the system clock, CLOCK_REALTIME.
 *
 *   The use of the nanosleep() function has no effect on the action or
 *   blockage of any signal.
 *
 * Parameters:
 *   rqtp - The amount of time to be suspended from execution.
 *   rmtp - If the rmtp argument is non-NULL, the timespec structure
 *          referenced by it is updated to contain the amount of time
 *          remaining in the interval (the requested time minus the time
 *          actually slept)
 *
 * Returned Value:
 *   If the nanosleep() function returns because the requested time has
 *   elapsed, its return value is zero.
 *
 *   If the nanosleep() function returns because it has been interrupted by
 *   a signal, the function returns a value of -1 and sets errno to indicate
 *   the interruption. If the rmtp argument is non-NULL, the timespec
 *   structure referenced by it is updated to contain the amount of time
 *   remaining in the interval (the requested time minus the time actually
 *   slept). If the rmtp argument is NULL, the remaining time is not
 *   returned.
 *
 *   If nanosleep() fails, it returns a value of -1 and sets errno to
 *   indicate the error. The nanosleep() function will fail if:
 *
 *     EINTR - The nanosleep() function was interrupted by a signal.
 *     EINVAL - The rqtp argument specified a nanosecond value less than
 *       zero or greater than or equal to 1000 million.
 *     ENOSYS - The nanosleep() function is not supported by this
 *       implementation.
 *
 ****************************************************************************/

int nanosleep(FAR const struct timespec *rqtp, FAR struct timespec *rmtp)
{
  irqstate_t flags;
  uint32_t starttick;
  sigset_t set;
  struct siginfo value;
  int errval;
#ifdef CONFIG_DEBUG /* Warning avoidance */
  int ret;
#endif

  if (!rqtp || rqtp->tv_nsec < 0 || rqtp->tv_nsec >= 1000000000)
    {
      errval = EINVAL;
      goto errout;
    }

  /* Get the start time of the wait.  Interrupts are disabled to prevent
   * timer interrupts while we do tick-related calculations before and
   * after the wait.
   */

  flags     = irqsave();
  starttick = clock_systimer();

  /* Set up for the sleep.  Using the empty set means that we are not
   * waiting for any particular signal.  However, any unmasked signal can
   * still awaken sigtimedwait().
   */

  (void)sigemptyset(&set);

  /* nanosleep is a simple application of sigtimedwait. */

#ifdef CONFIG_DEBUG /* Warning avoidance */
  ret = sigtimedwait(&set, &value, rqtp);
#else
  (void)sigtimedwait(&set, &value, rqtp);
#endif

  /* sigtimedwait() cannot succeed.  It should always return error with
   * either (1) EAGAIN meaning that the timeout occurred, or (2) EINTR
   * meaning that some other unblocked signal was caught.
   */

  errval = get_errno();
  DEBUGASSERT(ret < 0 && (errval == EAGAIN || errval == EINTR));

  if (errval == EAGAIN)
    {
      /* The timeout "error" is the normal, successful result */

      irqrestore(flags);
      return OK;
    }

  /* If we get there, the wait has failed because we were awakened by a
   * signal.  Return the amount of "unwaited" time if rmtp is non-NULL.
   */

  if (rmtp)
    {
      uint32_t elapsed;
      uint32_t remaining;
      int ticks;

      /* First get the number of clock ticks that we were requested to
       * wait.
       */

      (void)clock_time2ticks(rqtp, &ticks);

      /* Get the number of ticks that we actually waited */

      elapsed = clock_systimer() - starttick;

      /* The difference between the number of ticks that we were requested
       * to wait and the number of ticks that we actualy waited is that
       * amount of time that we failed to wait.
       */

      if (elapsed >= (uint32_t)ticks)
        {
          remaining = 0;
        }
      else
        {
          remaining = (uint32_t)ticks - elapsed;
        }

      (void)clock_ticks2time((int)remaining, rmtp);
    }

  irqrestore(flags);

errout:
  set_errno(errval);
  return ERROR;
}
