/****************************************************************************
 * arch/xtensa/src/common/xtensa_switchcontext.c
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
#include <arch/chip/core-isa.h>

#include "sched/sched.h"
#include "group/group.h"
#include "clock/clock.h"
#include "xtensa.h"

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

void up_switch_context(struct tcb_s *tcb, struct tcb_s *rtcb)
{
  /* Update scheduler parameters */

  nxsched_suspend_scheduler(rtcb);

  /* Are we in an interrupt handler? */

  if (CURRENT_REGS)
    {
      /* Yes, then we have to do things differently.
       * Just copy the CURRENT_REGS into the OLD rtcb.
       */

      xtensa_savestate(rtcb->xcp.regs);

      /* Update scheduler parameters */

      nxsched_resume_scheduler(tcb);

      /* Then switch contexts.  Any necessary address environment
       * changes will be made when the interrupt returns.
       */

      xtensa_restorestate(tcb->xcp.regs);
    }

  /* No, then we will need to perform the user context switch */

  else
    {
      /* Reset scheduler parameters */

      nxsched_resume_scheduler(tcb);

      /* Switch context to the context of the task at the head of the
       * ready to run list.
       */

      xtensa_switchcontext(&rtcb->xcp.regs, tcb->xcp.regs);

      /* xtensa_switchcontext forces a context switch to the task at the
       * head of the ready-to-run list.  It does not 'return' in the
       * normal sense.  When it does return, it is because the blocked
       * task is again ready to run and has execution priority.
       */
    }
}
