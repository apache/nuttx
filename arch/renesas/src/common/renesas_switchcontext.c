/****************************************************************************
 * arch/renesas/src/common/renesas_switchcontext.c
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

#include "sched/sched.h"
#include "group/group.h"
#include "clock/clock.h"
#include "renesas_internal.h"

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

  if (g_current_regs)
    {
      /* Yes, then we have to do things differently.
       * Just copy the g_current_regs into the OLD rtcb.
       */

      renesas_savestate(rtcb->xcp.regs);

      /* Update scheduler parameters */

      nxsched_resume_scheduler(tcb);

      /* Then switch contexts.  Any necessary address environment
       * changes will be made when the interrupt returns.
       */

      g_current_regs = tcb->xcp.regs;
    }

  /* We are not in an interrupt handler.  Copy the user C context
   * into the TCB of the task that was previously active.  if
   * up_saveusercontext returns a non-zero value, then this
   * is really the previously running task restarting!
   */

  else if (!up_saveusercontext(rtcb->xcp.regs))
    {
#ifdef CONFIG_ARCH_ADDRENV
      /* Make sure that the address environment for the previously
       * running task is closed down gracefully (data caches dump,
       * MMU flushed) and set up the address environment for the new
       * thread at the head of the ready-to-run list.
       */

      group_addrenv(tcb);
#endif
      /* Update scheduler parameters */

      nxsched_resume_scheduler(tcb);

      /* Then switch contexts */

      renesas_fullcontextrestore(tcb->xcp.regs);
    }
}
