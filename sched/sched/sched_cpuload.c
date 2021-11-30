/****************************************************************************
 * sched/sched/sched_cpuload.c
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

#include <errno.h>
#include <assert.h>

#include <nuttx/clock.h>
#include <nuttx/irq.h>

#include "sched/sched.h"

#ifdef CONFIG_SCHED_CPULOAD

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Are we using the system timer, or an external clock?  Get the rate
 * of the sampling in ticks per second for the selected timer.
 */

#ifdef CONFIG_SCHED_CPULOAD_EXTCLK
#  ifndef CONFIG_SCHED_CPULOAD_TICKSPERSEC
#    error CONFIG_SCHED_CPULOAD_TICKSPERSEC is not defined
#  endif
#  define CPULOAD_TICKSPERSEC CONFIG_SCHED_CPULOAD_TICKSPERSEC
#else
#  define CPULOAD_TICKSPERSEC CLOCKS_PER_SEC
#endif

/* When g_cpuload_total exceeds the following time constant, the load and
 * the counts will be scaled back by two.  In the CONFIG_SMP, g_cpuload_total
 * will be incremented multiple times per tick.
 */

#ifdef CONFIG_SMP
#  define CPULOAD_TIMECONSTANT \
     (CONFIG_SMP_NCPUS * \
      CONFIG_SCHED_CPULOAD_TIMECONSTANT * \
      CPULOAD_TICKSPERSEC)
#else
#  define CPULOAD_TIMECONSTANT \
     (CONFIG_SCHED_CPULOAD_TIMECONSTANT * \
      CPULOAD_TICKSPERSEC)
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is the total number of clock tick counts.  Essentially the
 * 'denominator' for all CPU load calculations.
 *
 * For a single CPU, this value is increment once per sample interval.  So,
 * for example, if nothing is running but the IDLE thread, that IDLE thread
 * will get 100% of the load.
 *
 * But for the case of multiple CPUs (with CONFIG_SMP=y), this value is
 * incremented for each CPU on each sample interval. So, as an example, if
 * there are four CPUs and is nothing is running but the IDLE threads, then
 * each would have a load of 25% of the total.
 */

volatile uint32_t g_cpuload_total;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsched_cpu_process_cpuload
 *
 * Description:
 *   Collect data that can be used for CPU load measurements.
 *
 * Input Parameters:
 *   cpu - The CPU that we are performing the load operations on.
 *
 * Returned Value:
 *   None
 *
 * Assumptions/Limitations:
 *   This function is called from a timer interrupt handler with all
 *   interrupts disabled.
 *
 ****************************************************************************/

static inline void nxsched_cpu_process_cpuload(int cpu)
{
  FAR struct tcb_s *rtcb = current_task(cpu);

  /* Increment the count on the currently executing thread */

  rtcb->ticks++;

  /* Increment tick count.  NOTE that the count is increment once for each
   * CPU on each sample interval.
   */

  g_cpuload_total++;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsched_process_cpuload
 *
 * Description:
 *   Collect data that can be used for CPU load measurements.  When
 *   CONFIG_SCHED_CPULOAD_EXTCLK is defined, this is an exported interface,
 *   use the the external clock logic.  Otherwise, it is an OS Internal
 *   interface.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions/Limitations:
 *   This function is called from a timer interrupt handler with all
 *   interrupts disabled.
 *
 ****************************************************************************/

void weak_function nxsched_process_cpuload(void)
{
  int i;
  irqstate_t flags;

  /* Perform scheduler operations on all CPUs. */

  flags = enter_critical_section();

#ifdef CONFIG_SMP
  for (i = 0; i < CONFIG_SMP_NCPUS; i++)
    {
      nxsched_cpu_process_cpuload(i);
    }

#else
  /* Perform scheduler operations on the single CPU. */

  nxsched_cpu_process_cpuload(0);

#endif

  /* If the accumulated tick value exceed a time constant, then shift the
   * accumulators and recalculate the total.
   */

  if (g_cpuload_total > CPULOAD_TIMECONSTANT)
    {
      uint32_t total = 0;

      /* Divide the tick count for every task by two and recalculate the
       * total.
       */

      for (i = 0; i < g_npidhash; i++)
        {
          if (g_pidhash[i])
            {
              g_pidhash[i]->ticks >>= 1;
              total += g_pidhash[i]->ticks;
            }
        }

      /* Save the new total. */

      g_cpuload_total = total;
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name:  clock_cpuload
 *
 * Description:
 *   Return load measurement data for the select PID.
 *
 * Input Parameters:
 *   pid - The task ID of the thread of interest.  pid == 0 is the IDLE
 *         thread.
 *   cpuload - The location to return the CPU load
 *
 * Returned Value:
 *   OK (0) on success; a negated errno value on failure.  The only reason
 *   that this function can fail is if 'pid' no longer refers to a valid
 *   thread.
 *
 * Assumptions:
 *
 ****************************************************************************/

int clock_cpuload(int pid, FAR struct cpuload_s *cpuload)
{
  irqstate_t flags;
  int hash_index;
  int ret = -ESRCH;

  DEBUGASSERT(cpuload);

  /* Momentarily disable interrupts.  We need (1) the task to stay valid
   * while we are doing these operations and (2) the tick counts to be
   * synchronized when read.
   */

  flags = enter_critical_section();
  hash_index = PIDHASH(pid);

  /* Make sure that the entry is valid (TCB field is not NULL) and matches
   * the requested PID.  The first check is needed if the thread has exited.
   * The second check is needed for the case where the task associated with
   * the requested PID has exited and the slot has been taken by another
   * thread with a different PID.
   *
   * NOTE also that CPU load measurement data is retained in the g_pidhash
   * table vs. in the TCB which would seem to be the more logic place.  It
   * is place in the hash table, instead, to facilitate CPU load adjustments
   * on all threads during timer interrupt handling. nxsched_foreach() could
   * do this too, but this would require a little more overhead.
   */

  if (g_pidhash[hash_index] && g_pidhash[hash_index]->pid == pid)
    {
      cpuload->total  = g_cpuload_total;
      cpuload->active = g_pidhash[hash_index]->ticks;
      ret = OK;
    }

  leave_critical_section(flags);
  return ret;
}

#endif /* CONFIG_SCHED_CPULOAD */
