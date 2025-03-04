/****************************************************************************
 * sched/sched/sched_critmonitor.c
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

#include <sys/types.h>
#include <sched.h>
#include <assert.h>
#include <debug.h>
#include <time.h>

#include "sched/sched.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_PREEMPTION > 0
#  define CHECK_PREEMPTION(pid, elapsed) \
     do \
       { \
         if (pid > 0 && \
             elapsed > CONFIG_SCHED_CRITMONITOR_MAXTIME_PREEMPTION) \
           { \
             CRITMONITOR_PANIC("PID %d hold sched lock too long %"PRIu32"\n", \
                               pid, elapsed); \
           } \
       } \
     while (0)
#else
#  define CHECK_PREEMPTION(pid, elapsed)
#endif

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_CSECTION > 0
#  define CHECK_CSECTION(pid, elapsed) \
     do \
       { \
         if (pid > 0 && \
             elapsed > CONFIG_SCHED_CRITMONITOR_MAXTIME_CSECTION) \
           { \
             CRITMONITOR_PANIC("PID %d hold critical section too long %" \
                               PRIu32 "\n", pid, elapsed); \
           } \
       } \
     while (0)
#else
#  define CHECK_CSECTION(pid, elapsed)
#endif

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_THREAD > 0
#  define CHECK_THREAD(pid, elapsed) \
     do \
       { \
         if (pid > 0 && \
             elapsed > CONFIG_SCHED_CRITMONITOR_MAXTIME_THREAD) \
           { \
             CRITMONITOR_PANIC("PID %d execute too long %"PRIu32"\n", \
                               pid, elapsed); \
           } \
       } \
     while (0)
#else
#  define CHECK_THREAD(pid, elapsed)
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Maximum time with pre-emption disabled or within critical section. */

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_PREEMPTION >= 0
clock_t g_preemp_max[CONFIG_SMP_NCPUS];
#endif

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_CSECTION >= 0
clock_t g_crit_max[CONFIG_SMP_NCPUS];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsched_critmon_cpuload
 *
 * Description:
 *   Update the running time of all running threads when switching threads
 *
 * Input Parameters:
 *   tcb   - The task that we are performing the load operations on.
 *   current - The current time
 *   tick - The ticks that we process in this cpuload.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SCHED_CPULOAD_CRITMONITOR
static void nxsched_critmon_cpuload(FAR struct tcb_s *tcb, clock_t current,
                                    clock_t tick)
{
  int i;
  UNUSED(i);

  /* Update the cpuload of the thread ready to be suspended */

  nxsched_process_taskload_ticks(tcb, tick);

  /* Update the cpuload of threads running on other CPUs */

#  ifdef CONFIG_SMP
  for (i = 0; i < CONFIG_SMP_NCPUS; i++)
    {
      FAR struct tcb_s *rtcb = current_task(i);

      if (tcb->cpu == rtcb->cpu)
        {
          continue;
        }

      nxsched_process_taskload_ticks(rtcb, tick);

      /* Update start time, avoid repeated statistics when the next call */

      rtcb->run_start = current;
    }
#  endif
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsched_critmon_preemption
 *
 * Description:
 *   Called when there is any change in pre-emptible state of a thread.
 *
 * Assumptions:
 *   - Called within a critical section.
 *   - Never called from an interrupt handler
 *   - Caller is the address of the function that is changing the pre-emption
 *
 ****************************************************************************/

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_PREEMPTION >= 0
void nxsched_critmon_preemption(FAR struct tcb_s *tcb, bool state,
                                FAR void *caller)
{
  clock_t current = perf_gettime();

  /* Are we enabling or disabling pre-emption */

  if (state)
    {
      /* Disabling.. Save the thread start time */

      tcb->preemp_start  = current;
      tcb->preemp_caller = caller;
    }
  else
    {
      /* Re-enabling.. Check for the max elapsed time */

      clock_t elapsed = current - tcb->preemp_start;
      int cpu         = this_cpu();

      if (elapsed > tcb->preemp_max)
        {
          tcb->preemp_max        = elapsed;
          tcb->preemp_max_caller = tcb->preemp_caller;
          CHECK_PREEMPTION(tcb->pid, elapsed);
        }

      /* Check for the global max elapsed time */

      if (elapsed > g_preemp_max[cpu])
        {
          g_preemp_max[cpu] = elapsed;
        }
    }
}
#endif /* CONFIG_SCHED_CRITMONITOR_MAXTIME_PREEMPTION >= 0 */

/****************************************************************************
 * Name: nxsched_critmon_csection
 *
 * Description:
 *   Called when a thread enters or leaves a critical section.
 *
 * Assumptions:
 *   - Called within a critical section.
 *   - Never called from an interrupt handler
 *   - Caller is the address of the function that is entering the critical
 *
 ****************************************************************************/

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_CSECTION >= 0
void nxsched_critmon_csection(FAR struct tcb_s *tcb, bool state,
                              FAR void *caller)
{
  clock_t current = perf_gettime();

  /* Are we entering or leaving the critical section? */

  if (state)
    {
      /* Entering... Save the start time. */

      tcb->crit_start  = current;
      tcb->crit_caller = caller;
    }
  else
    {
      /* Leaving .. Check for the max elapsed time */

      clock_t elapsed = current - tcb->crit_start;
      int cpu         = this_cpu();

      if (elapsed > tcb->crit_max)
        {
          tcb->crit_max        = elapsed;
          tcb->crit_max_caller = tcb->crit_caller;
          CHECK_CSECTION(tcb->pid, elapsed);
        }

      /* Check for the global max elapsed time */

      if (elapsed > g_crit_max[cpu])
        {
          g_crit_max[cpu] = elapsed;
        }
    }
}
#endif /* CONFIG_SCHED_CRITMONITOR_MAXTIME_CSECTION >= 0 */

/****************************************************************************
 * Name: nxsched_resume_critmon
 *
 * Description:
 *   Called when a thread resumes execution, perhaps re-establishing a
 *   critical section or a non-pre-emptible state.
 *
 * Assumptions:
 *   - Called within a critical section.
 *   - Might be called from an interrupt handler
 *
 ****************************************************************************/

void nxsched_resume_critmon(FAR struct tcb_s *tcb)
{
  clock_t current = perf_gettime();

  UNUSED(current);

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_THREAD >= 0
  tcb->run_start = current;
#endif

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_PREEMPTION >= 0
  /* Did this task disable pre-emption? */

  if (nxsched_islocked_tcb(tcb))
    {
      /* Yes.. Save the start time */

      tcb->preemp_start = current;
    }
#endif /* CONFIG_SCHED_CRITMONITOR_MAXTIME_PREEMPTION */

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_CSECTION >= 0
  /* Was this task in a critical section? */

  if (tcb->irqcount > 0)
    {
      /* Yes.. Save the start time */

      tcb->crit_start = current;
    }
#endif /* CONFIG_SCHED_CRITMONITOR_MAXTIME_CSECTION */
}

/****************************************************************************
 * Name: nxsched_suspend_critmon
 *
 * Description:
 *   Called when a thread suspends execution, perhaps terminating a
 *   critical section or a non-preemptible state.
 *
 * Assumptions:
 *   - Called within a critical section.
 *   - Might be called from an interrupt handler
 *
 ****************************************************************************/

void nxsched_suspend_critmon(FAR struct tcb_s *tcb)
{
  clock_t current = perf_gettime();
  clock_t elapsed = current - tcb->run_start;
  int cpu = this_cpu();

#ifdef CONFIG_SCHED_CPULOAD_CRITMONITOR
  clock_t tick = elapsed * CLOCKS_PER_SEC / perf_getfreq();
  nxsched_critmon_cpuload(tcb, current, tick);
#endif

  UNUSED(cpu);

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_THREAD >= 0
  tcb->run_time += elapsed;
  if (elapsed > tcb->run_max)
    {
      tcb->run_max = elapsed;
      CHECK_THREAD(tcb->pid, elapsed);
    }
#endif

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_PREEMPTION >= 0
  /* Did this task disable preemption? */

  if (nxsched_islocked_tcb(tcb))
    {
      /* Possibly re-enabling.. Check for the max elapsed time */

      elapsed = current - tcb->preemp_start;
      if (elapsed > tcb->preemp_max)
        {
          tcb->preemp_max        = elapsed;
          tcb->preemp_max_caller = tcb->preemp_caller;
          CHECK_PREEMPTION(tcb->pid, elapsed);
        }

      /* Suspend percore preemptible statistic and if necessary will
       * re-open in nxsched_resume_critmon.
       */

      if (elapsed > g_preemp_max[cpu])
        {
          g_preemp_max[cpu] = elapsed;
        }
    }
#endif /* CONFIG_SCHED_CRITMONITOR_MAXTIME_PREEMPTION */

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_CSECTION >= 0
  /* Is this task in a critical section? */

  if (tcb->irqcount > 0)
    {
      /* Possibly leaving .. Check for the max elapsed time */

      elapsed = current - tcb->crit_start;
      if (elapsed > tcb->crit_max)
        {
          tcb->crit_max        = elapsed;
          tcb->crit_max_caller = tcb->crit_caller;
          CHECK_CSECTION(tcb->pid, elapsed);
        }

      /* Check for the global max elapsed time */

      if (elapsed > g_crit_max[cpu])
        {
          g_crit_max[cpu] = elapsed;
        }
    }
#endif /* CONFIG_SCHED_CRITMONITOR_MAXTIME_CSECTION */
}

void nxsched_update_critmon(FAR struct tcb_s *tcb)
{
  clock_t current = perf_gettime();
  clock_t elapsed = current - tcb->run_start;

  if (tcb->task_state != TSTATE_TASK_RUNNING)
    {
      return;
    }

#ifdef CONFIG_SCHED_CPULOAD_CRITMONITOR
  clock_t tick = elapsed * CLOCKS_PER_SEC / perf_getfreq();
  nxsched_process_taskload_ticks(tcb, tick);
#endif

  tcb->run_start = current;
  tcb->run_time += elapsed;
  if (elapsed > tcb->run_max)
    {
      tcb->run_max = elapsed;
      CHECK_THREAD(tcb->pid, elapsed);
    }
}
