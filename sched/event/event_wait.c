/****************************************************************************
 * sched/event/event_wait.c
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

#include "event.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxevent_timeout
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

static void nxevent_timeout(wdparm_t arg)
{
  FAR struct tcb_s *wtcb;
  irqstate_t flags;

  /* Get waiting tcb from parameter */

  wtcb = (FAR struct tcb_s *)(uintptr_t)arg;

  /* We must be in a critical section in order to call up_switch_context()
   * below.
   */

  flags = enter_critical_section();

  /* There may be a race condition -- make sure the task is
   * still waiting for a signal
   */

  if (wtcb->task_state == TSTATE_WAIT_EVENT)
    {
      nxevent_wait_irq(wtcb, ETIMEDOUT);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxevent_tickwait
 *
 * Description:
 *   Wait for all of the specified events for the specified tick time.
 *
 *   This routine waits on event object event until all of the specified
 *   events have been delivered to the event object, or the maximum wait time
 *   timeout has expired. A thread may wait on up to 32 distinctly numbered
 *   events that are expressed as bits in a single 32-bit word.
 *
 * Input Parameters:
 *   event  - Address of the event object
 *   events - Set of events to wait
 *          - Set events to 0 will indicate wait from any events
 *   eflags - Events flags
 *   delay  - Ticks to wait from the start time until the event is
 *            posted.  If ticks is zero, then this function is equivalent
 *            to nxevent_trywait().
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   Return of matching events upon success.
 *   0 if matching events were not received within the specified time.
 *
 ****************************************************************************/

nxevent_mask_t nxevent_tickwait(FAR nxevent_t *event, nxevent_mask_t events,
                                nxevent_flags_t eflags, uint32_t delay)
{
  FAR struct tcb_s *rtcb = this_task();
  irqstate_t flags;
  bool waitany;

  DEBUGASSERT(event != NULL && up_interrupt_context() == false);

  waitany = ((eflags & NXEVENT_WAIT_ALL) == 0);

  if (events == 0)
    {
      events = ~0;
    }

  flags = enter_critical_section();

  if ((eflags & NXEVENT_WAIT_RESET) != 0)
    {
      event->events = 0;
    }

  /* Fetch for any event */

  if (waitany && ((events & event->events) != 0))
    {
      events &= event->events;
      if ((eflags & NXEVENT_WAIT_NOCLEAR) == 0)
        {
          event->events &= ~events;
        }
    }

  /* Events we desire here ? */

  else if (!waitany && (event->events & events) == events)
    {
      if ((eflags & NXEVENT_WAIT_NOCLEAR) == 0)
        {
          event->events &= ~events;
        }
    }

  /* Return 0 if no event expect in try wait case */

  else if (delay == 0)
    {
      events = 0;
    }

  /* Let's wait for the event to arrive */

  else
    {
      /* Start the watchdog with interrupts still disabled */

      wd_start(&rtcb->waitdog, delay, nxevent_timeout, (uintptr_t)rtcb);

      /* First, verify that the task is not already waiting on a
       * event
       */

      DEBUGASSERT(rtcb->waitobj == NULL);

      /* Save the waited on event in the TCB */

      rtcb->waitobj = event;
      rtcb->expect = events;
      rtcb->eflags = eflags;

      /* Set the errno value to zero (preserving the original errno)
       * value).  We reuse the per-thread errno to pass information
       * between nxevent_wait_irq() and this functions.
       */

      rtcb->errcode = OK;

      /* Add the TCB to the prioritized event wait queue, after
       * checking this is not the idle task - descheduling that
       * isn't going to end well.
       */

      DEBUGASSERT(!is_idle_task(rtcb));

      nxsched_remove_self(rtcb);

      rtcb->task_state = TSTATE_WAIT_EVENT;
      nxsched_add_prioritized(rtcb, EVENT_WAITLIST(event));

      /* Now, perform the context switch if one is needed */

      up_switch_context(this_task(), rtcb);

      if (rtcb->errcode == OK)
        {
          events = rtcb->expect;
        }
      else
        {
          events = 0;
        }
    }

  leave_critical_section(flags);

  return events;
}

/****************************************************************************
 * Name: nxevent_wait
 *
 * Description:
 *   Wait for all of the specified events.
 *
 *   This routine waits on event object event until all of the specified
 *   events have been delivered to the event object. A thread may wait on
 *   up to 32 distinctly numbered events that are expressed as bits in a
 *   single 32-bit word.
 *
 * Input Parameters:
 *   event  - Address of the event object
 *   events - Set of events to wait, 0 will indicate wait from any events
 *   eflags - Events flags
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   Return of matching events upon success, Otherwise, 0 is returned if OS
 *   internal error.
 *
 ****************************************************************************/

nxevent_mask_t nxevent_wait(FAR nxevent_t *event, nxevent_mask_t events,
                            nxevent_flags_t eflags)
{
  return nxevent_tickwait(event, events, eflags, UINT32_MAX);
}

/****************************************************************************
 * Name: nxevent_trywait
 *
 * Description:
 *   Try wait for all of the specified events.
 *
 *   This routine try to waits on event object event if any of the specified
 *   events have been delivered to the event object. A thread may wait on
 *   up to 32 distinctly numbered events that are expressed as bits in a
 *   single 32-bit word.
 *
 * Input Parameters:
 *   event  - Address of the event object
 *   events - Set of events to wait
 *          - Set events to 0 will indicate wait from any events
 *   eflags - Events flags
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   Return of matching events upon success.
 *   0 if matching events were not received.
 *
 ****************************************************************************/

nxevent_mask_t nxevent_trywait(FAR nxevent_t *event,
                               nxevent_mask_t events,
                               nxevent_flags_t eflags)
{
  return nxevent_tickwait(event, events, eflags, 0);
}
