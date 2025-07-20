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
 * Public Data
 ****************************************************************************/

rspinlock_t g_schedlock = RSPINLOCK_INITIALIZER;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_CSECTION >= 0 || \
    CONFIG_SCHED_CRITMONITOR_MAXTIME_BUSYWAIT >= 0 || \
    defined(CONFIG_SCHED_INSTRUMENTATION_CSECTION)
void restore_critical_section(uint16_t count)
{
  /* If CONFIG_SCHED_CRITMONITOR_MAXTIME_BUSYWAIT >= 0,
   * start counting time of busy-waiting.
   */

  nxsched_critmon_busywait(true, return_address(0));

  rspin_restorelock(&g_schedlock, count);

  /* Get the lock, end counting busy-waiting */

  nxsched_critmon_busywait(false, return_address(0));

  if (!up_interrupt_context())
    {
      FAR struct tcb_s *rtcb = this_task();

#  ifdef CONFIG_SCHED_INSTRUMENTATION_CSECTION
      sched_note_csection(rtcb, true);
#  endif
#  if CONFIG_SCHED_CRITMONITOR_MAXTIME_CSECTION >= 0
      nxsched_critmon_csection(rtcb, true, return_address(0));
#  endif
    }
}

void break_critical_section(void)
{
  if (!up_interrupt_context())
    {
      FAR struct tcb_s *rtcb = this_task();

#  ifdef CONFIG_SCHED_INSTRUMENTATION_CSECTION
      sched_note_csection(rtcb, false);
#  endif
#  if CONFIG_SCHED_CRITMONITOR_MAXTIME_CSECTION >= 0
      nxsched_critmon_csection(rtcb, false, return_address(0));
#  endif
    }

  rspin_breaklock(&g_schedlock);
}

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
      if (rspin_lock_count(&g_schedlock) == 1)
        {
#  ifdef CONFIG_SCHED_INSTRUMENTATION_CSECTION
          sched_note_csection(rtcb, true);
#  endif
#  if CONFIG_SCHED_CRITMONITOR_MAXTIME_CSECTION >= 0
          nxsched_critmon_csection(rtcb, true, return_address(0));
#  endif
        }
    }

  return flags;
}

void leave_critical_section(irqstate_t flags)
{
  if (!up_interrupt_context())
    {
      FAR struct tcb_s *rtcb = this_task();
      if (rspin_lock_count(&g_schedlock) == 1)
        {
#  ifdef CONFIG_SCHED_INSTRUMENTATION_CSECTION
          sched_note_csection(rtcb, false);
#  endif
#  if CONFIG_SCHED_CRITMONITOR_MAXTIME_CSECTION >= 0
          nxsched_critmon_csection(rtcb, false, return_address(0));
#  endif
        }
    }

  leave_critical_section_notrace(flags);
}
#endif
