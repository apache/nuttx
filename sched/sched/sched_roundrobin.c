/****************************************************************************
 * sched/sched/sched_roundrobin.c
 *
 *   Copyright (C) 2007, 2009, 2014-2016 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <stdbool.h>
#include <sched.h>
#include <assert.h>

#include <nuttx/sched.h>
#include <nuttx/clock.h>

#include "sched/sched.h"

#if CONFIG_RR_INTERVAL > 0

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef MIN
#  define MIN(a,b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef MAX
#  define MAX(a,b) (((a) > (b)) ? (a) : (b))
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  sched_roundrobin_process
 *
 * Description:
 *   Check if the currently executing task has exceeded its time slice.
 *
 * Input Parameters:
 *   tcb - The TCB of the currently executing task
 *   ticks - The number of ticks that have elapsed on the interval timer.
 *   noswitches - True: Can't do context switches now.
 *
 * Returned Value:
 *   The number if ticks remaining until the next time slice expires.
 *   Zero is returned if there is no time slicing (i.e., the task at the
 *   head of the ready-to-run list does not support round robin
 *   scheduling).
 *
 *   The value one may returned under certain circumstances that probably
 *   can't happen.  The value one is the minimal timer setup and it means
 *   that a context switch is needed now, but cannot be performed because
 *   noswitches == true.
 *
 * Assumptions:
 *   - Interrupts are disabled
 *   - The task associated with TCB uses the round robin scheduling
 *     policy
 *
 ****************************************************************************/

uint32_t sched_roundrobin_process(FAR struct tcb_s *tcb, uint32_t ticks,
                                  bool noswitches)
{
  uint32_t ret;
  int decr;

  /* How much can we decrement the timeslice delay?  If 'ticks' is greater
   * than the timeslice value, then we ignore any excess amount.
   *
   * 'ticks' should not be greater than the remaining timeslice.  But that
   * event seems to be possible, perhaps in cases where pre-emption has been
   * disabled or the noswitches flag is set.  This might cause jitter of a
   * few ticks in the slicing because the excess amount is not handled.
   */

  DEBUGASSERT(tcb != NULL);
  decr = MIN(tcb->timeslice, ticks);

  /* Decrement the timeslice counter */

  tcb->timeslice -= decr;

  /* Did decrementing the timeslice counter cause the timeslice to expire? */

  ret = tcb->timeslice;
  if (tcb->timeslice <= 0 && !sched_islocked_tcb(tcb))
    {
      /* We will also suppress context switches if we were called via one
       * of the unusual cases handled by sched_timer_reasses().  In that
       * case, we will return a value of one so that the timer will expire
       * as soon as possible and we can perform this action in the normal
       * timer expiration context.
       *
       * This is kind of kludge, but I am not to concerned because I hope
       * that the situation is impossible or at least could only occur on
       * rare corner-cases.
       */

      if (noswitches)
        {
          ret = 1;
        }
      else
        {
          /* Reset the timeslice. */

          tcb->timeslice = MSEC2TICK(CONFIG_RR_INTERVAL);
          ret = tcb->timeslice;

          /* We know we are at the head of the ready to run prioritized
           * list.  We must be the highest priority task eligible for
           * execution.  Check the next task in the ready to run list.  If
           * it is the same priority, then we need to relinquish the CPU and
           * give that task a shot.
           */

          if (tcb->flink &&
              tcb->flink->sched_priority >= tcb->sched_priority)
            {
              /* Just resetting the task priority to its current value.
               * This this will cause the task to be rescheduled behind any
               * other tasks at the same priority.
               */

              up_reprioritize_rtr(tcb, tcb->sched_priority);
            }
        }
    }

  return ret;
}

#endif /* CONFIG_RR_INTERVAL > 0 */
