/****************************************************************************
 * sched/sched/sched_processtimer.c
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
#include <time.h>

#if CONFIG_RR_INTERVAL > 0
#  include <sched.h>
#  include <nuttx/arch.h>
#endif

#ifdef CONFIG_SYSTEMTICK_HOOK
#  include <nuttx/board.h>
#endif

#ifdef CONFIG_CLOCK_TIMEKEEPING
#  include "clock/clock_timekeeping.h"
#endif

#include "sched/sched.h"
#include "wdog/wdog.h"
#include "clock/clock.h"
#include "hrtimer/hrtimer.h"

#ifdef CONFIG_HRTIMER
/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static uint64_t
nxsched_hrtimer_callback(FAR const hrtimer_t *hrtimer,
                         uint64_t expired);

static void nxsched_process_tick(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Scheduler-owned high-resolution timer instance.
 *
 * This timer acts as the time source for scheduler-related events:
 *
 *  - Periodic scheduler ticks in non-tickless mode
 *  - Dynamic expiration points in tickless mode
 *
 * The timer is initialized lazily to avoid unnecessary setup when
 * CONFIG_HRTIMER is enabled but not used immediately.
 */

static hrtimer_t g_nxsched_hrtimer =
{
  .func = nxsched_hrtimer_callback,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsched_hrtimer_callback
 *
 * Description:
 *   Callback invoked by the high-resolution timer framework when the
 *   scheduler timer expires.
 *
 *   Behavior depends on scheduler configuration:
 *
 *   CONFIG_SCHED_TICKLESS:
 *     - Query current high-resolution time
 *     - Convert time to scheduler ticks
 *     - Notify scheduler via nxsched_tick_expiration()
 *
 *   !CONFIG_SCHED_TICKLESS:
 *     - Re-arm the next periodic tick
 *     - Process a single scheduler tick
 *
 * Input Parameters:
 *   hrtimer - Pointer to the expired high-resolution timer
 *
 * Returned Value:
 *   In non-tickless mode, returns the interval until the next expiration.
 *   In tickless mode, the return value is ignored.
 *
 ****************************************************************************/

static uint64_t
nxsched_hrtimer_callback(FAR hrtimer_t *hrtimer, uint64_t expired)
{
  UNUSED(hrtimer);
  UNUSED(expired);

#ifdef CONFIG_SCHED_TICKLESS
  nxsched_tick_expiration();
#else
  nxsched_process_tick();
  return NSEC_PER_TICK;
#endif
}

/****************************************************************************
 * Name: nxsched_process_hrtimer
 *
 * Description:
 *   Entry point for scheduler-related high-resolution timer processing.
 *
 *   Responsibilities:
 *     - Process expired hrtimer events
 *     - Perform one-time initialization of the scheduler hrtimer
 *     - Arm the initial scheduler timer event
 *
 *   This function is expected to be called from architecture-specific
 *   timer interrupt handling code.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void nxsched_process_hrtimer(void)
{
  uint64_t now = hrtimer_gettime();

  /* Process any expired high-resolution timers */

  hrtimer_process(now);
}
#endif /* CONFIG_HRTIMER */

#ifndef CONFIG_SCHED_TICKLESS
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
  irqstate_t flags;
  int i;

  /* Single CPU case:
   * For nested interrupts, higher IRQs may interrupt nxsched_cpu_scheduler()
   * but nxsched_cpu_scheduler() requires that interrupts be disabled.
   * We are in ISR context, no meaning we are disabled the interrupts.
   *
   * SMP case:
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
}
#else
#  define nxsched_process_scheduler()
#endif

/****************************************************************************
 * Name:  nxsched_process_tick
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

static void nxsched_process_tick(void)
{
#ifdef CONFIG_CLOCK_TIMEKEEPING
  /* Process wall time */

  clock_update_wall_time();
#endif

  /* Increment the system time (if in the link) */

  clock_increase_sched_ticks(1);

  /* Check if the currently executing task has exceeded its
   * timeslice.
   */

  nxsched_process_scheduler();

  /* Process watchdogs */

  wd_timer(clock_systime_ticks());

#ifdef CONFIG_SYSTEMTICK_HOOK
  /* Call out to a user-provided function in order to perform board-specific,
   * custom timer operations.
   */

  board_timerhook();
#endif
}
#endif

/****************************************************************************
 * System Timer Hooks
 *
 * These are standard interfaces that are exported by the OS
 * for use by the architecture specific logic
 *
 ****************************************************************************/

void nxsched_process_timer(void)
{
#if defined(CONFIG_HRTIMER)
  /* High-resolution timer-based scheduling */

  nxsched_process_hrtimer();

#elif defined(CONFIG_SCHED_TICKLESS)
  /* Tickless scheduling */

  nxsched_tick_expiration();

#else
  /* Periodic tick-based scheduling */

  nxsched_process_tick();

#endif
}
