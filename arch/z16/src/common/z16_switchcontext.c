/****************************************************************************
 * arch/z16/src/common/z16_switchcontext.c
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
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/sched.h>

#include "chip.h"
#include "sched/sched.h"
#include "clock/clock.h"
#include "z16_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_switch_context
 *
 * Description:
 *   A task is currently in the ready-to-run list but has been prepped
 *   to execute. Restore its context, and start execution.
 *
 * Input Parameters:
 *   tcb: Refers to the head task of the ready-to-run list
 *     which will be executed.
 *   rtcb: Refers to the running task which will be blocked.
 *
 ****************************************************************************/

void up_switch_context(FAR struct tcb_s *tcb, FAR struct tcb_s *rtcb)
{
  /* Update scheduler parameters */

  nxsched_suspend_scheduler(rtcb);

  /* Are we in an interrupt handler? */

  if (IN_INTERRUPT)
    {
      /* Yes, then we have to do things differently.
       * Just copy the current context into the OLD rtcb.
       */

      SAVE_IRQCONTEXT(rtcb);

      /* Update scheduler parameters */

      nxsched_resume_scheduler(tcb);

      /* Then setup so that the context will be performed on exit
       * from the interrupt.
       */

      SET_IRQCONTEXT(tcb);
    }

  /* We are not in an interrupt handler.  Copy the user C context
   * into the TCB of the task that was previously active.  if
   * SAVE_USERCONTEXT returns a non-zero value, then this is really the
   * previously running task restarting!
   */

  else if (!SAVE_USERCONTEXT(rtcb))
    {
      /* Update scheduler parameters */

      nxsched_resume_scheduler(tcb);

      /* Then switch contexts */

      RESTORE_USERCONTEXT(tcb);
    }
}
