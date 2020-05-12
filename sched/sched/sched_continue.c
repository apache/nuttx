/****************************************************************************
 * sched/sched/sched_continue.c
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
#include <sched.h>
#include <assert.h>

#include <nuttx/irq.h>
#include <nuttx/sched.h>
#include <nuttx/arch.h>

#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsched_continue
 *
 * Description:
 *   Resume the specified thread.  This is normally calling indirectly
 *   via group_continue();
 *
 ****************************************************************************/

void nxsched_continue(FAR struct tcb_s *tcb)
{
  irqstate_t flags;

  DEBUGASSERT(tcb != NULL && tcb->task_state == TSTATE_TASK_STOPPED);

  flags = enter_critical_section();

  /* Simply restart the thread.  If is was blocked before, it will awaken
   * with errcode = EINTR and will appears as if it were awakened by a
   * signal.  If pre-emption is not disabled this action could block this
   * task here!
   */

  up_unblock_task(tcb);
  leave_critical_section(flags);
}
