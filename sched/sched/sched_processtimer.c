/****************************************************************************
 * sched/sched/sched_processtimer.c
 *
 *   Copyright (C) 2007, 2009, 2014-2019 Gregory Nutt. All rights reserved.
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
#  include <sched.h>
#  include <nuttx/arch.h>
#endif

#ifdef CONFIG_SYSTEMTICK_HOOK
#  include <nuttx/board.h>
#endif

#include "sched/sched.h"
#include "wdog/wdog.h"
#include "clock/clock.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  nxsched_cpu_scheduler
 *
 * Description:
 *   Check for operations specific to scheduling policy of the currently
 *   active task on one CPU.
 *
 * Input Parameters:
 *   cpu - The CPU that we are performing the scheduler operations on.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if CONFIG_RR_INTERVAL > 0 || defined(CONFIG_SCHED_SPORADIC)
static inline void nxsched_cpu_scheduler(int cpu)
{
  FAR struct tcb_s *rtcb = current_task(cpu);

#if CONFIG_RR_INTERVAL > 0
  /* Check if the currently executing task uses round robin scheduling. */

  if ((rtcb->flags & TCB_FLAG_POLICY_MASK) == TCB_FLAG_SCHED_RR)
    {
      /* Yes, check if the currently executing task has exceeded its
       * timeslice.
       */

      nxsched_process_roundrobin(rtcb, 1, false);
    }
#endif

#ifdef CONFIG_SCHED_SPORADIC
  /* Check if the currently executing task uses sporadic scheduling. */

  if ((rtcb->flags & TCB_FLAG_POLICY_MASK) == TCB_FLAG_SCHED_SPORADIC)
    {
      /* Yes, check if the currently executing task has exceeded its
       * budget.
       */

      nxsched_process_sporadic(rtcb, 1, false);
    }
#endif
}
#endif

/****************************************************************************
 * Name:  nxsched_process_scheduler
 *
 * Description:
 *   Check for operations specific to scheduling policy of the currently
 *   active task on all configured CPUs.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if CONFIG_RR_INTERVAL > 0 || defined(CONFIG_SCHED_SPORADIC)
static inline void nxsched_process_scheduler(void)
{
#ifdef CONFIG_SMP
  irqstate_t flags;
  int i;

  /* If we are running on a single CPU architecture, then we know interrupts
   * a disabled an there is no need to explicitly call
   * enter_critical_section().  However, in the SMP case,
   * enter_critical_section() does much more than just disable interrupts on
   * the local CPU; it also manages spinlocks to assure the stability of the
   * TCB that we are manipulating.
   */

  flags = enter_critical_section();

  /* Perform scheduler operations on all CPUs */

  for (i = 0; i < CONFIG_SMP_NCPUS; i++)
    {
      nxsched_cpu_scheduler(i);
    }

  leave_critical_section(flags);

#else
  /* Perform scheduler operations on the single CPUs */

  nxsched_cpu_scheduler(0);
#endif
}
#else
#  define nxsched_process_scheduler()
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
 * Name:  nxsched_process_timer
 *
 * Description:
 *   This function handles system timer events.
 *   The timer interrupt logic itself is implemented in the
 *   architecture specific code, but must call the following OS
 *   function periodically -- the calling interval must be
 *   USEC_PER_TICK
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxsched_process_timer(void)
{
#ifdef CONFIG_CLOCK_TIMEKEEPING
  /* Process wall time */

  clock_update_wall_time();
#endif

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
  if (nxsched_process_cpuload != NULL)
#endif
    {
      nxsched_process_cpuload();
    }
#endif

  /* Check if the currently executing task has exceeded its
   * timeslice.
   */

  nxsched_process_scheduler();

  /* Process watchdogs */

  wd_timer();

#ifdef CONFIG_SYSTEMTICK_HOOK
  /* Call out to a user-provided function in order to perform board-specific,
   * custom timer operations.
   */

  board_timerhook();
#endif
}
