/****************************************************************************
 * sched/sched/sched_processtimer.c
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
   * are disabled and there is no need to explicitly call
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
 * Name: nxsched_process_wdtimer
 *
 * Description:
 *   Wdog timer process, should with critical_section when SMP mode.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SMP
static inline void nxsched_process_wdtimer(void)
{
  irqstate_t flags;

  /* We are in an interrupt handler and, as a consequence, interrupts are
   * disabled.  But in the SMP case, interrupts MAY be disabled only on
   * the local CPU since most architectures do not permit disabling
   * interrupts on other CPUS.
   *
   * Hence, we must follow rules for critical sections even here in the
   * SMP case.
   */

  flags = enter_critical_section();
  wd_timer();
  leave_critical_section(flags);
}
#else
#  define nxsched_process_wdtimer() wd_timer()
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

  nxsched_process_wdtimer();

#ifdef CONFIG_SYSTEMTICK_HOOK
  /* Call out to a user-provided function in order to perform board-specific,
   * custom timer operations.
   */

  board_timerhook();
#endif
}
