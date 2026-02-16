/****************************************************************************
 * sched/event/event_post.c
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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxevent_post
 *
 * Description:
 *   Post one or more events to the specified event object.  This function
 *   wakes up any tasks that are waiting for the posted events, depending
 *   on their wait condition (any/all).
 *
 * Input Parameters:
 *   event  - Pointer to the event object.
 *   events - Bitmask of events to post.
 *   eflags - Posting behavior flags, such as:
 *              NXEVENT_POST_SET: overwrite existing events.
 *              NXEVENT_POST_ALL: wake all waiting tasks.
 * Returned Value:
 *   OK on success; -EINVAL if the event pointer is NULL.
 *
 ****************************************************************************/

int nxevent_post(FAR nxevent_t *event, nxevent_mask_t events,
                 nxevent_flags_t eflags)
{
  FAR struct tcb_s *wtcb;
  FAR struct tcb_s *rtcb;
  nxevent_mask_t clear = 0;
  FAR dq_entry_t *entry;
  FAR dq_entry_t *tmp;
  irqstate_t flags;
  dq_queue_t *waitlist;
  bool waitall;
  bool postall;
  bool need_switch = false;

  if (event == NULL)
    {
      return -EINVAL;
    }

  flags = enter_critical_section();

  if ((eflags & NXEVENT_POST_SET) != 0)
    {
      event->events = events ? events : ~0;
    }
  else
    {
      event->events |= events ? events : ~0;
    }

  postall = ((eflags & NXEVENT_POST_ALL) != 0);
  rtcb    = this_task();
  waitlist = EVENT_WAITLIST(event);

  dq_for_every_safe(waitlist, entry, tmp)
    {
      wtcb = (FAR struct tcb_s *)entry;
      waitall = ((wtcb->eflags & NXEVENT_WAIT_ALL) != 0);

      /* Check if this task's wait condition is satisfied */

      if ((!waitall && (wtcb->expect & event->events) != 0) ||
          (waitall && ((wtcb->expect & event->events) == wtcb->expect)))
        {
          dq_rem((FAR dq_entry_t *)wtcb, waitlist);

          /* Stop timeout watchdog */

          wd_cancel(&wtcb->waitdog);
          wtcb->waitobj = NULL;

          /* Make task ready-to-run */

          if (nxsched_add_readytorun(wtcb) && !need_switch)
            {
              /* Higher priority task is ready */

              need_switch = true;
            }

          /* If waiting for any event, mark received ones */

          if (!waitall)
            {
              wtcb->expect &= event->events;
            }

          if ((wtcb->eflags & NXEVENT_WAIT_NOCLEAR) == 0)
            {
              clear |= wtcb->expect;
            }

          if (!postall && (event->events & ~clear) == 0)
            {
              break;
            }
        }
    }

  if (clear != 0)
    {
      event->events &= ~clear;
    }

  if (need_switch)
    {
      /* Switch context to the highest priority ready-to-run task */

      up_switch_context(this_task(), rtcb);
    }

  leave_critical_section(flags);

  return OK;
}
