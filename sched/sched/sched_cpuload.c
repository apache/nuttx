/****************************************************************************
 * sched/sched/sched_cpuload.c
 *
 *   Copyright (C) 2014, 2019 Gregory Nutt. All rights reserved.
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
  FAR struct tcb_s *rtcb  = current_task(cpu);
  int hash_index;

  /* Increment the count on the currently executing thread
   *
   * NOTE also that CPU load measurement data is retained in the g_pidhash
   * table vs. in the TCB which would seem to be the more logic place.  It
   * is place in the hash table, instead, to facilitate CPU load adjustments
   * on all threads during timer interrupt handling. nxsched_foreach() could
   * do this too, but this would require a little more overhead.
   */

  hash_index = PIDHASH(rtcb->pid);
  g_pidhash[hash_index].ticks++;

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

#ifdef CONFIG_SMP
  irqstate_t flags;

  /* Perform scheduler operations on all CPUs. */

  flags = enter_critical_section();
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

      for (i = 0; i < CONFIG_MAX_TASKS; i++)
        {
          g_pidhash[i].ticks >>= 1;
          total += g_pidhash[i].ticks;
        }

      /* Save the new total. */

      g_cpuload_total = total;
    }

#ifdef CONFIG_SMP
  leave_critical_section(flags);
#endif
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
  int hash_index = PIDHASH(pid);
  int ret = -ESRCH;

  DEBUGASSERT(cpuload);

  /* Momentarily disable interrupts.  We need (1) the task to stay valid
   * while we are doing these operations and (2) the tick counts to be
   * synchronized when read.
   */

  flags = enter_critical_section();

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

  if (g_pidhash[hash_index].tcb && g_pidhash[hash_index].pid == pid)
    {
      cpuload->total  = g_cpuload_total;
      cpuload->active = g_pidhash[hash_index].ticks;
      ret = OK;
    }

  leave_critical_section(flags);
  return ret;
}

#endif /* CONFIG_SCHED_CPULOAD */
