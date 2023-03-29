/****************************************************************************
 * sched/sched/sched_critmonitor.c
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

#include "sched/sched.h"

#ifdef CONFIG_SCHED_CRITMONITOR

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_SCHED_CRITMONITOR_MAXTIME_PREEMPTION
#  define CONFIG_SCHED_CRITMONITOR_MAXTIME_PREEMPTION 0
#endif

#ifndef CONFIG_SCHED_CRITMONITOR_MAXTIME_CSECTION
#  define CONFIG_SCHED_CRITMONITOR_MAXTIME_CSECTION 0
#endif

#ifndef CONFIG_SCHED_CRITMONITOR_MAXTIME_THREAD
#  define CONFIG_SCHED_CRITMONITOR_MAXTIME_THREAD 0
#endif

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_PREEMPTION > 0
#  define CHECK_PREEMPTION(pid, elapsed) \
     do \
       { \
         if (pid > 0 && \
             elapsed > CONFIG_SCHED_CRITMONITOR_MAXTIME_PREEMPTION) \
           { \
             serr("PID %d hold sched lock too long %"PRIu32"\n", \
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
             serr("PID %d hold critical section too long %"PRIu32"\n", \
                   pid, elapsed); \
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
             serr("PID %d execute too long %"PRIu32"\n", \
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

/* Start time when pre-emption disabled or critical section entered. */

static uint32_t g_premp_start[CONFIG_SMP_NCPUS];
static uint32_t g_crit_start[CONFIG_SMP_NCPUS];

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Maximum time with pre-emption disabled or within critical section. */

uint32_t g_premp_max[CONFIG_SMP_NCPUS];
uint32_t g_crit_max[CONFIG_SMP_NCPUS];

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
 *
 ****************************************************************************/

void nxsched_critmon_preemption(FAR struct tcb_s *tcb, bool state)
{
  int cpu = this_cpu();

  /* Are we enabling or disabling pre-emption */

  if (state)
    {
      /* Disabling.. Save the thread start time */

      tcb->premp_start   = up_perf_gettime();
      g_premp_start[cpu] = tcb->premp_start;
    }
  else
    {
      /* Re-enabling.. Check for the max elapsed time */

      uint32_t now     = up_perf_gettime();
      uint32_t elapsed = now - tcb->premp_start;

      if (elapsed > tcb->premp_max)
        {
          tcb->premp_max = elapsed;
          CHECK_PREEMPTION(tcb->pid, elapsed);
        }

      /* Check for the global max elapsed time */

      elapsed = now - g_premp_start[cpu];
      if (elapsed > g_premp_max[cpu])
        {
          g_premp_max[cpu] = elapsed;
        }
    }
}

/****************************************************************************
 * Name: nxsched_critmon_csection
 *
 * Description:
 *   Called when a thread enters or leaves a critical section.
 *
 * Assumptions:
 *   - Called within a critical section.
 *   - Never called from an interrupt handler
 *
 ****************************************************************************/

void nxsched_critmon_csection(FAR struct tcb_s *tcb, bool state)
{
  int cpu = this_cpu();

  /* Are we entering or leaving the critical section? */

  if (state)
    {
      /* Entering... Save the start time. */

      tcb->crit_start   = up_perf_gettime();
      g_crit_start[cpu] = tcb->crit_start;
    }
  else
    {
      /* Leaving .. Check for the max elapsed time */

      uint32_t now     = up_perf_gettime();
      uint32_t elapsed = now - tcb->crit_start;

      if (elapsed > tcb->crit_max)
        {
          tcb->crit_max = elapsed;
          CHECK_CSECTION(tcb->pid, elapsed);
        }

      /* Check for the global max elapsed time */

      elapsed = now - g_crit_start[cpu];
      if (elapsed > g_crit_max[cpu])
        {
          g_crit_max[cpu] = elapsed;
        }
    }
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
  uint32_t current = up_perf_gettime();
  int cpu = this_cpu();
  uint32_t elapsed;

  tcb->run_start = current;

  /* Did this task disable pre-emption? */

  if (tcb->lockcount > 0)
    {
      /* Yes.. Save the start time */

      tcb->premp_start   = current;
      g_premp_start[cpu] = current;
    }
  else
    {
      /* Check for the global max elapsed time */

      elapsed = current - g_premp_start[cpu];
      if (elapsed > g_premp_max[cpu])
        {
          g_premp_max[cpu] = elapsed;
          CHECK_PREEMPTION(tcb->pid, elapsed);
        }
    }

  /* Was this task in a critical section? */

  if (tcb->irqcount > 0)
    {
      /* Yes.. Save the start time */

      tcb->crit_start   = current;
      g_crit_start[cpu] = current;
    }
  else
    {
      /* Check for the global max elapsed time */

      elapsed = current - g_crit_start[cpu];
      if (elapsed > g_crit_max[cpu])
        {
          g_crit_max[cpu] = elapsed;
          CHECK_CSECTION(tcb->pid, elapsed);
        }
    }
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
  uint32_t current = up_perf_gettime();
  uint32_t elapsed = current - tcb->run_start;

  if (elapsed > tcb->run_max)
    {
      tcb->run_max = elapsed;
      CHECK_THREAD(tcb->pid, elapsed);
    }

  /* Did this task disable preemption? */

  if (tcb->lockcount > 0)
    {
      /* Possibly re-enabling.. Check for the max elapsed time */

      elapsed = current - tcb->premp_start;
      if (elapsed > tcb->premp_max)
        {
          tcb->premp_max = elapsed;
          CHECK_PREEMPTION(tcb->pid, elapsed);
        }
    }

  /* Is this task in a critical section? */

  if (tcb->irqcount > 0)
    {
      /* Possibly leaving .. Check for the max elapsed time */

      elapsed = current - tcb->crit_start;
      if (elapsed > tcb->crit_max)
        {
          tcb->crit_max = elapsed;
          CHECK_CSECTION(tcb->pid, elapsed);
        }
    }
}

#endif
