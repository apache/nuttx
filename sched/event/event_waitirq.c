/****************************************************************************
 * sched/event/event_waitirq.c
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

#include <sched.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "sched/sched.h"
#include "event/event.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxevent_wait_irq
 *
 * Description:
 *   An error event has occurred and the event wait must be terminated with
 *   an error.
 *
 ****************************************************************************/

void nxevent_wait_irq(FAR struct tcb_s *wtcb, int errcode)
{
  FAR struct tcb_s *rtcb = this_task();
  FAR nxevent_t *event = wtcb->waitobj;

  /* Remove the task from the event waiting list */

  dq_rem((FAR dq_entry_t *)wtcb, EVENT_WAITLIST(event));

  /* Indicate that the wait is over */

  wtcb->waitobj = NULL;

  /* Store the error code for the thread */

  wtcb->errcode = errcode;

  /* Add the task to the ready-to-run list and perform a context
   * switch if one is needed.
   */

  if (nxsched_add_readytorun(wtcb))
    {
      up_switch_context(this_task(), rtcb);
    }
}
