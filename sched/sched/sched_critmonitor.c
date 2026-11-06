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
#include <nuttx/debug.h>
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

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_BUSYWAIT > 0
#  define CHECK_BUSYWAIT(pid, elapsed) \
     do \
       { \
         if (pid > 0 && \
             elapsed > CONFIG_SCHED_CRITMONITOR_MAXTIME_BUSYWAIT) \
           { \
             CRITMONITOR_PANIC("PID %d wait for critical section or spin" \
                               "lock too long %" PRIu32 "\n", pid, elapsed); \
           } \
       } \
     while (0)
#else
#  define CHECK_BUSYWAIT(pid, elapsed)
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
 * Private Data
 ****************************************************************************/

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_PREEMPTION >= 0
static spinlock_t g_crimonitor_lock = SP_UNLOCKED;
#endif

/* The time of the last CPU load update of each CPU.  The run time of the
 * threads (run_start, run_time) is only changed by their own CPU.
 */

#ifdef CONFIG_SCHED_CPULOAD_CRITMONITOR
static clock_t g_cpuload_start[CONFIG_SMP_NCPUS];
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

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_BUSYWAIT >= 0
clock_t g_busywait_max[CONFIG_SMP_NCPUS];
clock_t g_busywait_total[CONFIG_SMP_NCPUS];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsched_critmon_cpuload
 *
 * Description:
 *   Give the running thread of a CPU the CPU load ticks since the last
 *   update of that CPU.
 *
 * Input Parameters:
 *   tcb     - The thread that runs on the CPU.
 *   cpu     - The CPU.
 *   current - The current time.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SCHED_CPULOAD_CRITMONITOR
static void nxsched_critmon_cpuload(FAR struct tcb_s *tcb, int cpu,
                                    clock_t current)
{
  clock_t elapsed = current - g_cpuload_start[cpu];

  g_cpuload_start[cpu] = current;
  nxsched_process_taskload_ticks(tcb, elapsed * CLK_TCK / perf_getfreq());
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
  irqstate_t flags;

  flags = spin_lock_irqsave_notrace(&g_crimonitor_lock);

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

  spin_unlock_irqrestore_notrace(&g_crimonitor_lock, flags);
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
 * Name: nxsched_critmon_busywait
 *
 * Description:
 *   Called when a thread try to enter critical section（get spinlock） or
 *   successfully entered cirtical section.
 *
 * Assumptions:
 *   - Called before a critical section or within a critical section.
 *   - Might be called from an interrupt handler.
 *
 ****************************************************************************/

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_BUSYWAIT >= 0
void nxsched_critmon_busywait(bool state, FAR void *caller)
{
  FAR struct tcb_s *tcb = this_task();
  clock_t current       = perf_gettime();

  /* Are we busy waiting for critical section or spinlock? */

  if (state)
    {
      /* Waiting... Save the start time. */

      tcb->busywait_start  = current;
      tcb->busywait_caller = caller;
    }
  else
    {
      /* Entered critical section... Check for the max elapsed time */

      clock_t elapsed = current - tcb->busywait_start;
      int cpu         = this_cpu();

      if (elapsed > tcb->busywait_max)
        {
          tcb->busywait_max        = elapsed;
          tcb->busywait_max_caller = tcb->busywait_caller;
          CHECK_BUSYWAIT(tcb->pid, elapsed);
        }

      /* Check for the global max elapsed time */

      if (elapsed > g_busywait_max[cpu])
        {
          g_busywait_max[cpu] = elapsed;
        }

      /* Update thread-level and cpu-level busywait */

      tcb->busywait_total   += elapsed;
      g_busywait_total[cpu] += elapsed;
    }
}
#endif /* CONFIG_SCHED_CRITMONITOR_MAXTIME_BUSYWAIT >= 0 */

/****************************************************************************
 * Name: nxsched_switch_critmon
 *
 * Description:
 *   Called when a thread is switched, update the critical monitor data.
 *
 * Input Parameters:
 *   from - The thread that is being switched out.
 *   to   - The thread that is being switched in.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxsched_switch_critmon(FAR struct tcb_s *from, FAR struct tcb_s *to)
{
  clock_t current = perf_gettime();
#if CONFIG_SCHED_CRITMONITOR_MAXTIME_THREAD >= 0
  clock_t elapsed = current - from->run_start;
#else
  clock_t elapsed = 0;

  UNUSED(current);
  UNUSED(elapsed);
#endif

#ifdef CONFIG_SCHED_CPULOAD_CRITMONITOR
  int thiscpu = this_cpu();
  int i;

  /* Update the CPU load of the running threads of all CPUs */

  for (i = 0; i < CONFIG_SMP_NCPUS; i++)
    {
      nxsched_critmon_cpuload(i == thiscpu ? from : current_task(i), i,
                              current);
    }
#endif

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_THREAD >= 0
  /* A zero run_start tells nxsched_critmon_runtime() that the thread does
   * not run.  Clear it before run_time changes.
   */

  from->run_start = 0;
  SMP_WMB();
  from->run_time += elapsed;
  to->run_start = current;
  if (elapsed > from->run_max)
    {
      from->run_max = elapsed;
      CHECK_THREAD(from->pid, elapsed);
    }
#endif

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_PREEMPTION >= 0

  /* Did this task disable preemption? */

  if (nxsched_islocked_tcb(from))
    {
      int cpu = this_cpu();

      /* Possibly re-enabling.. Check for the max elapsed time */

      elapsed = current - from->preemp_start;
      if (elapsed > from->preemp_max)
        {
          from->preemp_max        = elapsed;
          from->preemp_max_caller = from->preemp_caller;
          CHECK_PREEMPTION(from->pid, elapsed);
        }

      /* Suspend percore preemptible statistic and if necessary will
       * re-open in nxsched_resume_critmon.
       */

      if (elapsed > g_preemp_max[cpu])
        {
          g_preemp_max[cpu] = elapsed;
        }
    }

  if (to->lockcount > 0)
    {
      to->preemp_start = current;
    }
#endif /* CONFIG_SCHED_CRITMONITOR_MAXTIME_PREEMPTION */

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_CSECTION >= 0

  /* Is this task in a critical section? */

  if (from->irqcount > 0)
    {
      int cpu = this_cpu();

      /* Possibly leaving .. Check for the max elapsed time */

      elapsed = current - from->crit_start;
      if (elapsed > from->crit_max)
        {
          from->crit_max        = elapsed;
          from->crit_max_caller = from->crit_caller;
          CHECK_CSECTION(from->pid, elapsed);
        }

      /* Check for the global max elapsed time */

      if (elapsed > g_crit_max[cpu])
        {
          g_crit_max[cpu] = elapsed;
        }
    }

  if (to->irqcount > 0)
    {
      to->crit_start = current;
    }

#endif /* CONFIG_SCHED_CRITMONITOR_MAXTIME_CSECTION */
}

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_THREAD >= 0
void nxsched_update_critmon(FAR struct tcb_s *tcb)
{
  clock_t current = perf_gettime();
  clock_t start = tcb->run_start;

  /* Only the CPU of the thread changes run_start and run_time */

  if (tcb->task_state == TSTATE_TASK_RUNNING && start != 0)
    {
#ifdef CONFIG_SCHED_CPULOAD_CRITMONITOR
#  ifdef CONFIG_SMP
      nxsched_critmon_cpuload(tcb, tcb->cpu, current);
#  else
      nxsched_critmon_cpuload(tcb, 0, current);
#  endif
#endif

      if (current - start > tcb->run_max)
        {
          tcb->run_max = current - start;
          CHECK_THREAD(tcb->pid, current - start);
        }
    }
}
#endif

/****************************************************************************
 * Name: nxsched_critmon_runtime
 *
 * Description:
 *   Return the run time of a thread.  If the thread runs now, add the time
 *   since it started to run.
 *
 ****************************************************************************/

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_THREAD >= 0
clock_t nxsched_critmon_runtime(FAR struct tcb_s *tcb)
{
  irqstate_t flags = up_irq_save();
  clock_t runtime = tcb->run_time;
  clock_t start;

  /* Read run_time before run_start: nxsched_switch_critmon() clears
   * run_start before it adds the time slice to run_time.
   */

  SMP_RMB();
  start = tcb->run_start;
  if (start != 0 && tcb->task_state == TSTATE_TASK_RUNNING)
    {
      runtime += perf_gettime() - start;
    }

  up_irq_restore(flags);
  return runtime;
}
#endif
