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

#include <nuttx/sched.h>

#include "event.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxevent_sem_post
 ****************************************************************************/

static inline_function int nxevent_sem_post(FAR sem_t *sem)
{
  int semcount;

  nxsem_get_value(sem, &semcount);
  if (semcount < 1)
    {
      return nxsem_post(sem);
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxevent_post
 *
 * Description:
 *   Post one or more events to an event object.
 *
 *   This routine posts one or more events to an event object. All tasks
 *   waiting on the event object event whose waiting conditions become
 *   met by this posting immediately unpend.
 *
 *   Posting differs from setting in that posted events are merged together
 *   with the current set of events tracked by the event object.
 *
 * Input Parameters:
 *   event  - Address of the event object
 *   events - Set of events to post to event
 *          - Set events to 0 will be considered as any,
 *            waking up the waiting thread immediately.
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 * Assumptions:
 *   This function may be called from an interrupt handler.
 *
 ****************************************************************************/

int nxevent_post(FAR nxevent_t *event, nxevent_mask_t events,
                 nxevent_flags_t eflags)
{
  nxevent_mask_t clear = 0;
  FAR nxevent_wait_t *wait;
  FAR nxevent_wait_t *tmp;
  irqstate_t flags;
  bool waitall;
  bool postall;
  int ret = 0;

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

  if (!list_is_empty(&event->list))
    {
      postall = ((eflags & NXEVENT_POST_ALL) != 0);

      /* Hold schedule lock here to avoid context switch if post high
       * priority task.
       */

      sched_lock();

      list_for_every_entry_safe(&event->list, wait, tmp,
                                nxevent_wait_t, node)
        {
          waitall = ((wait->eflags & NXEVENT_WAIT_ALL) != 0);

          if ((!waitall && ((wait->expect & event->events) != 0)) ||
              (waitall && ((wait->expect & event->events) == wait->expect)))
            {
              list_delete_init(&wait->node);

              ret = nxevent_sem_post(&wait->sem);
              if (ret < 0)
                {
                  continue;
                }

              if (!waitall)
                {
                  wait->expect &= event->events;
                }

              if ((wait->eflags & NXEVENT_WAIT_NOCLEAR) == 0)
                {
                  clear |= wait->expect;
                }

              if (!postall && (event->events & ~clear) == 0)
                {
                  break;
                }
            }
        }

      if (clear)
        {
          event->events &= ~clear;
        }

      sched_unlock();
    }

  leave_critical_section(flags);

  return ret;
}
