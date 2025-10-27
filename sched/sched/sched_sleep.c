/****************************************************************************
 * sched/sched/sched_sleep.c
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

#include <nuttx/irq.h>
#include <nuttx/clock.h>

#include "sched/sched.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsched_timeout
 *
 * Description:
 *   A timeout elapsed while waiting for timeout.
 *
 * Assumptions:
 *   This function executes in the context of the timer interrupt handler.
 *   Local interrupts are assumed to be disabled on entry.
 *
 * Input Parameters:
 *   arg - Parameter to pass to wdentry.
 *
 ****************************************************************************/

static void nxsched_timeout(wdparm_t arg)
{
  nxsched_wakeup((FAR struct tcb_s *)(uintptr_t)arg);
}

/****************************************************************************
 * Name: nxsched_ticksleep
 *
 * Description:
 *   The nxsched_ticksleep() function will cause the calling thread to be
 *   suspended from execution for the specified number of system ticks.
 *
 *   It can only be resumed through scheduler operations.
 *
 * Input Parameters:
 *   ticks - The number of system ticks to sleep.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxsched_ticksleep(unsigned int ticks)
{
  FAR struct tcb_s *rtcb;
  irqstate_t flags;

  if (ticks == 0)
    {
      sched_yield();
      return;
    }

  flags = enter_critical_section();

  rtcb = this_task();

  wd_start(&rtcb->waitdog, ticks, nxsched_timeout, (uintptr_t)rtcb);

  /* Remove the tcb task from the ready-to-run list. */

  nxsched_remove_self(rtcb);

  /* Add the task to the specified blocked task list */

  rtcb->task_state = TSTATE_SLEEPING;
  dq_addlast((FAR dq_entry_t *)rtcb, list_waitingforsignal());

  /* Now, perform the context switch if one is needed */

  up_switch_context(this_task(), rtcb);

  wd_cancel(&rtcb->waitdog);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: nxsched_wakeup
 *
 * Description:
 *   The nxsched_wakeup() function is used to wake up a task that is
 *   currently in the sleeping state before its timeout expires.
 *
 *   This function can be used by internal scheduler logic or by
 *   system-level components that need to resume a sleeping task early.
 *
 * Input Parameters:
 *   tcb - Pointer to the TCB of the task to be awakened.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxsched_wakeup(FAR struct tcb_s *tcb)
{
  irqstate_t flags;

  DEBUGASSERT(tcb != NULL);

  flags = enter_critical_section();

  if (tcb->task_state == TSTATE_SLEEPING)
    {
      FAR struct tcb_s *rtcb = this_task();

      /* Remove the task from sleeping list */

      dq_rem((FAR dq_entry_t *)tcb, list_waitingforsignal());

      wd_cancel(&tcb->waitdog);

      /* Add the task to ready-to-run task list, and
       * perform the context switch if one is needed
       */

      if (nxsched_add_readytorun(tcb))
        {
          up_switch_context(this_task(), rtcb);
        }
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: nxsched_usleep
 *
 * Description:
 *   The nxsched_usleep() function will cause the calling thread to be
 *   suspended from execution for the specified number of microseconds.
 *
 *   It can only be resumed through scheduler.
 *
 * Input Parameters:
 *   usec - The number of microseconds to sleep.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxsched_usleep(useconds_t usec)
{
  nxsched_ticksleep(USEC2TICK(usec));
}

/****************************************************************************
 * Name: nxsched_msleep
 *
 * Description:
 *   The nxsched_msleep() function will cause the calling thread to be
 *   suspended from execution for the specified number of milliseconds.
 *
 *   It can only be resumed through scheduler.
 *
 * Input Parameters:
 *   msec - The number of milliseconds to sleep.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxsched_msleep(unsigned int msec)
{
  nxsched_ticksleep(MSEC2TICK(msec));
}

/****************************************************************************
 * Name: nxsched_sleep
 *
 * Description:
 *   The nxsched_sleep() function will cause the calling thread to be
 *   suspended from execution for the specified number of seconds.
 *
 *   It can only be resumed through scheduler.
 *
 * Input Parameters:
 *   sec - The number of seconds to sleep.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxsched_sleep(unsigned int sec)
{
  nxsched_ticksleep(SEC2TICK(sec));
}
