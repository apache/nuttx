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
#include <nuttx/sched_note.h>

#include <sys/types.h>
#include <sched.h>
#include <assert.h>
#include <nuttx/debug.h>
#include <nuttx/seqlock.h>
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
             CRITMONITOR_PANIC("PID %d hold sched lock too long %ju\n", \
                               pid, (uintmax_t)elapsed); \
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
             CRITMONITOR_PANIC("PID %d hold critical section too long " \
                               "%ju\n", pid, (uintmax_t)elapsed); \
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
                               "lock too long %ju\n", \
                               pid, (uintmax_t)elapsed); \
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
             CRITMONITOR_PANIC("PID %d execute too long %ju\n", \
                               pid, (uintmax_t)elapsed); \
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

/* g_thread_tcb[cpu] is the thread that runs on the CPU (NULL during a
 * context switch), and g_thread_start[cpu] is the time since which the
 * run time of the CPU is not counted.  A context switch changes the values
 * of its own CPU, and clock_cpuload() changes the values of all CPUs.
 * They read the time and change the values under g_thread_lock[cpu].
 * nxsched_critmon_runtime() reads them without a lock and reads again if
 * they changed.
 */

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_THREAD >= 0
static seqcount_t g_thread_lock[CONFIG_SMP_NCPUS];
static FAR struct tcb_s *g_thread_tcb[CONFIG_SMP_NCPUS];
static clock_t g_thread_start[CONFIG_SMP_NCPUS];
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
 * Name: nxsched_critmon_account
 *
 * Description:
 *   Add the time since the last update of a CPU to the run time and to the
 *   CPU load of the thread that runs on the CPU.
 *
 * Input Parameters:
 *   tcb     - The thread that runs on the CPU.
 *   cpu     - The CPU.
 *   current - The current time.
 *
 * Returned Value:
 *   The time added.
 *
 * Assumptions:
 *   The caller holds g_thread_lock[cpu].
 *
 ****************************************************************************/

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_THREAD >= 0
static clock_t nxsched_critmon_account(FAR struct tcb_s *tcb, int cpu,
                                       clock_t current)
{
  clock_t elapsed = current - g_thread_start[cpu];

  g_thread_start[cpu] = current;
  tcb->run_time += elapsed;
#  ifdef CONFIG_SCHED_CPULOAD_CRITMONITOR
  nxsched_process_taskload_ticks(tcb, elapsed);
#  endif
  return elapsed;
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
#ifdef CONFIG_SCHED_INSTRUMENTATION_THREADTIME
  static clock_t threshold = CLOCK_MAX;
#endif
#if CONFIG_SCHED_CRITMONITOR_MAXTIME_THREAD >= 0
  irqstate_t flags = write_seqlock_irqsave(&g_thread_lock[this_cpu()]);
#endif
  clock_t current = perf_gettime();
#if CONFIG_SCHED_CRITMONITOR_MAXTIME_THREAD >= 0
  clock_t elapsed = nxsched_critmon_account(tcb, this_cpu(), current);

  g_thread_tcb[this_cpu()] = NULL;
  write_sequnlock_irqrestore(&g_thread_lock[this_cpu()], flags);
#else
  clock_t elapsed = 0;

  UNUSED(current);
  UNUSED(elapsed);
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_THREADTIME
  if (threshold == CLOCK_MAX)
    {
      threshold =
        CONFIG_SCHED_INSTRUMENTATION_THREAD_RUNTIME_DURATION *
        (clock_t)perf_getfreq() / USEC_PER_SEC;
    }

  if (!is_idle_task(tcb) && elapsed > threshold)
    {
      clock_t us = elapsed * USEC_PER_SEC / perf_getfreq();

      sched_note_threadtime(us);
    }
#endif

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_THREAD >= 0
  /* Check run_max after the unlock: a panic reads the CPU load */

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
      int cpu = this_cpu();

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
      int cpu = this_cpu();

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
#if CONFIG_SCHED_CRITMONITOR_MAXTIME_THREAD >= 0
  irqstate_t flags;
#endif
  clock_t current = perf_gettime();

  UNUSED(current);

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_THREAD >= 0
  /* Its run time counts from g_thread_start[], the suspend of the previous
   * thread.
   */

  flags = write_seqlock_irqsave(&g_thread_lock[this_cpu()]);
  g_thread_tcb[this_cpu()] = tcb;
  write_sequnlock_irqrestore(&g_thread_lock[this_cpu()], flags);
#endif

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_PREEMPTION >= 0

  /* Did this task disable pre-emption? */

  if (tcb->lockcount > 0)
    {
      /* Yes.. Save the start time */

      tcb->preemp_start = current;
    }
#endif /* CONFIG_SCHED_CRITMONITOR_MAXTIME_PREEMPTION */

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_CSECTION >= 0
  if (tcb->irqcount > 0)
    {
      tcb->crit_start = current;
    }
#endif /* CONFIG_SCHED_CRITMONITOR_MAXTIME_CSECTION */
}

/****************************************************************************
 * Name: nxsched_update_critmon
 *
 * Description:
 *   Add the time since the last update to the run time and to the CPU load
 *   of the running thread of each CPU, also of a thread in a busy loop.
 *   All CPUs, so that g_cpuload_total has the time of all CPUs.
 *
 ****************************************************************************/

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_THREAD >= 0
void nxsched_update_critmon(void)
{
  FAR struct tcb_s *tcb;
  irqstate_t flags;
  clock_t elapsed;
  int cpu;

  for (cpu = 0; cpu < CONFIG_SMP_NCPUS; cpu++)
    {
      flags = write_seqlock_irqsave(&g_thread_lock[cpu]);
      tcb   = g_thread_tcb[cpu];

      /* Skip a CPU in a context switch or before its first one */

      if (tcb == NULL)
        {
          write_sequnlock_irqrestore(&g_thread_lock[cpu], flags);
          continue;
        }

      elapsed = nxsched_critmon_account(tcb, cpu, perf_gettime());
      write_sequnlock_irqrestore(&g_thread_lock[cpu], flags);

      if (elapsed > tcb->run_max)
        {
          tcb->run_max = elapsed;
          CHECK_THREAD(tcb->pid, elapsed);
        }
    }
}
#endif

/****************************************************************************
 * Name: nxsched_critmon_runtime
 *
 * Description:
 *   Return the run time of a thread.  If the thread runs now, add the time
 *   since the last update of its CPU.
 *
 ****************************************************************************/

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_THREAD >= 0
clock_t nxsched_critmon_runtime(FAR struct tcb_s *tcb)
{
  uint32_t seq[CONFIG_SMP_NCPUS];
  clock_t runtime;
  bool retry;
  int cpu;

  /* Read again if a CPU changed the values, so that the run time does not
   * go back.  The scheduler changes tcb->cpu before the context switch, and
   * the thread can move to another CPU: look for the thread on all CPUs,
   * and check the sequence of all CPUs.
   */

  do
    {
      for (cpu = 0; cpu < CONFIG_SMP_NCPUS; cpu++)
        {
          seq[cpu] = read_seqbegin(&g_thread_lock[cpu]);
        }

      runtime = tcb->run_time;
      for (cpu = 0; cpu < CONFIG_SMP_NCPUS; cpu++)
        {
          if (g_thread_tcb[cpu] == tcb)
            {
              runtime += perf_gettime() - g_thread_start[cpu];
              break;
            }
        }

      retry = false;
      for (cpu = 0; cpu < CONFIG_SMP_NCPUS; cpu++)
        {
          retry |= read_seqretry(&g_thread_lock[cpu], seq[cpu]);
        }
    }
  while (retry);

  return runtime;
}
#endif
