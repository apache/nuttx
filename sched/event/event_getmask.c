/****************************************************************************
 * sched/event/event_getmask.c
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
 * Name: nxevent_getmask
 *
 * Description:
 *   Get the event mask of the given event object.
 *
 * Input Parameters:
 *   event - Address of the event object
 *
 * Returned Value:
 *   Returns the event mask value of the event object.
 *
 * Notes:
 *   - This is an internal OS interface and must not be invoked directly
 *     by user applications.
 *   - This function is safe to call from an interrupt handler.
 *
 ****************************************************************************/

nxevent_mask_t nxevent_getmask(FAR nxevent_t *event)
{
  nxevent_mask_t events;
  irqstate_t flags;

  DEBUGASSERT(event != NULL);

  flags = spin_lock_irqsave(&event->lock);

  events = event->events;

  spin_unlock_irqrestore(&event->lock, flags);

  return events;
}
