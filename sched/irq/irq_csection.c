/****************************************************************************
 * sched/irq/irq_csection.c
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
#include <assert.h>

#include <nuttx/init.h>
#include <nuttx/spinlock.h>
#include <nuttx/sched_note.h>
#include <arch/irq.h>

#include "sched/sched.h"
#include "irq/irq.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef CONFIG_SMP
/* This is the spinlock that enforces critical sections when interrupts are
 * disabled.
 */

volatile spinlock_t g_cpu_irqlock = SP_UNLOCKED;

/* Used to keep track of which CPU(s) hold the IRQ lock. */

volatile cpu_set_t g_cpu_irqset;

#endif

/* Handles nested calls to enter_critical section from interrupt handlers */

volatile uint8_t g_cpu_nestcount[CONFIG_SMP_NCPUS];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: enter_critical_section_notrace
 ****************************************************************************/

#ifdef CONFIG_SMP
irqstate_t enter_critical_section_notrace(void)
{
  irqstate_t ret;
  int cpu;

  ret = up_irq_save();
  cpu = this_cpu();
  if (g_cpu_nestcount[cpu]++ == 0)
    {
      spin_lock_notrace(&g_cpu_irqlock);
      DEBUGASSERT(spin_is_locked(&g_cpu_irqlock));
    }

  return ret;
}

#else

irqstate_t enter_critical_section_notrace(void)
{
  irqstate_t ret;

  /* Disable interrupts */

  ret = up_irq_save();
  g_cpu_nestcount[this_cpu()]++;

  /* Return interrupt status */

  return ret;
}
#endif

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_CSECTION >= 0 || \
    defined(CONFIG_SCHED_INSTRUMENTATION_CSECTION) || defined(CONFIG_SMP)
void restore_critical_section(uint16_t count)
{
  /* If CONFIG_SCHED_CRITMONITOR_MAXTIME_BUSYWAIT >= 0,
   * start counting time of busy-waiting.
   */

  nxsched_critmon_busywait(true, return_address(0));

#  ifdef CONFIG_SMP
  spin_lock_notrace(&g_cpu_irqlock);
#  endif

  g_cpu_nestcount[this_cpu()] = count;

  /* Get the lock, end counting busy-waiting */

  nxsched_critmon_busywait(false, return_address(0));

#  if CONFIG_SCHED_CRITMONITOR_MAXTIME_CSECTION >= 0 || \
      defined(CONFIG_SCHED_INSTRUMENTATION_CSECTION)
  if (!up_interrupt_context())
    {
      FAR struct tcb_s *rtcb = this_task();

#    ifdef CONFIG_SCHED_INSTRUMENTATION_CSECTION
      sched_note_csection(rtcb, true);
#    endif
#    if CONFIG_SCHED_CRITMONITOR_MAXTIME_CSECTION >= 0
      nxsched_critmon_csection(rtcb, true, return_address(0));
#    endif
    }
#  endif
}

void break_critical_section(void)
{
  int cpu = this_cpu();

#  if CONFIG_SCHED_CRITMONITOR_MAXTIME_CSECTION >= 0 || \
    defined(CONFIG_SCHED_INSTRUMENTATION_CSECTION)
  if (!up_interrupt_context())
    {
      FAR struct tcb_s *rtcb = this_task();

#    ifdef CONFIG_SCHED_INSTRUMENTATION_CSECTION
      sched_note_csection(rtcb, false);
#    endif
#    if CONFIG_SCHED_CRITMONITOR_MAXTIME_CSECTION >= 0
      nxsched_critmon_csection(rtcb, false, return_address(0));
#    endif
    }
#  endif

  g_cpu_nestcount[cpu] = 0;

#  ifdef CONFIG_SMP
  spin_unlock_notrace(&g_cpu_irqlock);
#  endif
}
#endif

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_CSECTION >= 0 || \
    CONFIG_SCHED_CRITMONITOR_MAXTIME_BUSYWAIT >= 0 || \
    defined(CONFIG_SCHED_INSTRUMENTATION_CSECTION)
irqstate_t enter_critical_section(void)
{
  irqstate_t flags;

  /* If CONFIG_SCHED_CRITMONITOR_MAXTIME_BUSYWAIT >= 0,
   * start counting time of busy-waiting.
   */

  nxsched_critmon_busywait(true, return_address(0));

  flags = enter_critical_section_notrace();

  /* Get the lock, end counting busy-waiting */

  nxsched_critmon_busywait(false, return_address(0));

  if (!up_interrupt_context())
    {
      FAR struct tcb_s *rtcb = this_task();
      if (g_cpu_nestcount[this_cpu()] == 1)
        {
#if CONFIG_SCHED_CRITMONITOR_MAXTIME_CSECTION >= 0
          nxsched_critmon_csection(rtcb, true, return_address(0));
#endif
#ifdef CONFIG_SCHED_INSTRUMENTATION_CSECTION
          sched_note_csection(rtcb, true);
#endif
        }
    }

  return flags;
}
#endif

/****************************************************************************
 * Name: leave_critical_section_notrace
 ****************************************************************************/

#ifdef CONFIG_SMP
void leave_critical_section_notrace(irqstate_t flags)
{
  int cpu;

  cpu = this_cpu();
  if (g_cpu_nestcount[cpu]-- == 1)
    {
      spin_unlock_notrace(&g_cpu_irqlock);
    }

  up_irq_restore(flags);
}
#else
void leave_critical_section_notrace(irqstate_t flags)
{
  /* Restore the previous interrupt state. */

  g_cpu_nestcount[this_cpu()]--;
  up_irq_restore(flags);
}
#endif

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_CSECTION >= 0 || \
    CONFIG_SCHED_CRITMONITOR_MAXTIME_BUSYWAIT >= 0 || \
    defined(CONFIG_SCHED_INSTRUMENTATION_CSECTION)
void leave_critical_section(irqstate_t flags)
{
  if (!up_interrupt_context())
    {
      FAR struct tcb_s *rtcb = this_task();
      if (g_cpu_nestcount[this_cpu()] == 1)
        {
#  if CONFIG_SCHED_CRITMONITOR_MAXTIME_CSECTION >= 0
          nxsched_critmon_csection(rtcb, false, return_address(0));
#  endif
#  ifdef CONFIG_SCHED_INSTRUMENTATION_CSECTION
          sched_note_csection(rtcb, false);
#  endif
        }
    }

  leave_critical_section_notrace(flags);
}
#endif

/****************************************************************************
 * Name: critical_section_count
 *
 * Description:
 *   get critical section count
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Critical section count
 *
 ****************************************************************************/

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_CSECTION >= 0 || \
    defined(CONFIG_SCHED_INSTRUMENTATION_CSECTION) || defined(CONFIG_SMP)
uint16_t critical_section_count(void)
{
  return g_cpu_nestcount[this_cpu()];
}
#endif
