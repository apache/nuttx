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

#include <nuttx/sched.h>

#include "event.h"

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
  nxevent_wait_t wait;
  irqstate_t flags;
  bool waitany;
  int ret;

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
      /* Initialize event wait */

      nxsem_init(&wait.sem, 0, 0);
      wait.expect = events;
      wait.eflags = eflags;

      list_add_tail(&event->list, &wait.node);

      /* Wait for the event */

      if (delay == UINT32_MAX)
        {
          ret = nxsem_wait_uninterruptible(&wait.sem);
        }
      else
        {
          ret = nxsem_tickwait_uninterruptible(&wait.sem, delay);
        }

      /* Destroy local variables */

      nxsem_destroy(&wait.sem);
      list_delete(&wait.node);

      if (ret == 0)
        {
          events = wait.expect;
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
