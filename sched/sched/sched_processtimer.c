/****************************************************************************
 * sched/sched/sched_processtimer.c
 *
 *   Copyright (C) 2007, 2009, 2014-2015 Gregory Nutt. All rights reserved.
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
#include <nuttx/compiler.h>
#include <time.h>

#if CONFIG_RR_INTERVAL > 0
# include <sched.h>
# include <nuttx/arch.h>
#endif

#include "sched/sched.h"
#include "wdog/wdog.h"
#include "clock/clock.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_SCHED_CPULOAD_TIMECONSTANT
#  define CONFIG_SCHED_CPULOAD_TIMECONSTANT 2
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  sched_process_scheduler
 *
 * Description:
 *   Check for operations specific to scheduling policy of the currently
 *   active task.
 *
 * Inputs:
 *   None
 *
 * Return Value:
 *   None
 *
 ****************************************************************************/

#if CONFIG_RR_INTERVAL > 0 || defined(CONFIG_SCHED_SPORADIC)
static inline void sched_process_scheduler(void)
{
  FAR struct tcb_s *rtcb  = this_task();

#if CONFIG_RR_INTERVAL > 0
  /* Check if the currently executing task uses round robin scheduling. */

  if ((rtcb->flags & TCB_FLAG_POLICY_MASK) == TCB_FLAG_SCHED_RR)
    {
      /* Yes, check if the currently executing task has exceeded its
       * timeslice.
       */

      sched_roundrobin_process(rtcb, 1, false);
    }
#endif

#ifdef CONFIG_SCHED_SPORADIC
  /* Check if the currently executing task uses sporadic scheduling. */

  if ((rtcb->flags & TCB_FLAG_POLICY_MASK) == TCB_FLAG_SCHED_SPORADIC)
    {
      /* Yes, check if the currently executing task has exceeded its
       * budget.
       */

      (void)sched_sporadic_process(rtcb, 1, false);
    }
#endif
}
#else
#  define sched_process_scheduler()
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * System Timer Hooks
 *
 * These are standard interfaces that are exported by the OS
 * for use by the architecture specific logic
 *
 ****************************************************************************/

/****************************************************************************
 * Name:  sched_process_timer
 *
 * Description:
 *   This function handles system timer events.
 *   The timer interrupt logic itself is implemented in the
 *   architecture specific code, but must call the following OS
 *   function periodically -- the calling interval must be
 *   USEC_PER_TICK
 *
 * Inputs:
 *   None
 *
 * Return Value:
 *   None
 *
 ****************************************************************************/

void sched_process_timer(void)
{
  /* Increment the system time (if in the link) */

#ifdef CONFIG_HAVE_WEAKFUNCTIONS
  if (clock_timer != NULL)
#endif
    {
      clock_timer();
    }

#if defined(CONFIG_SCHED_CPULOAD) && !defined(CONFIG_SCHED_CPULOAD_EXTCLK)
  /* Perform CPU load measurements (before any timer-initiated context
   * switches can occur)
   */

#ifdef CONFIG_HAVE_WEAKFUNCTIONS
  if (sched_process_cpuload != NULL)
#endif
    {
      sched_process_cpuload();
    }
#endif

  /* Process watchdogs */

  wd_timer();

  /* Check if the currently executing task has exceeded its
   * timeslice.
   */

  sched_process_scheduler();
}
