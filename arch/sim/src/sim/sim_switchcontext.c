/****************************************************************************
 * arch/sim/src/sim/sim_switchcontext.c
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
#include <debug.h>
#include <nuttx/arch.h>
#include <nuttx/sched.h>

#include "clock/clock.h"
#include "sim_internal.h"
#include "sched/sched.h"

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
  int ret;

  sinfo("Unblocking TCB=%p\n", tcb);

  /* Update scheduler parameters */

  nxsched_suspend_scheduler(rtcb);

  /* Are we in an interrupt handler? */

  if (up_interrupt_context())
    {
      /* Yes, then we have to do things differently.
       * Just copy the current_regs into the OLD rtcb.
       */

      sim_savestate(rtcb->xcp.regs);

      /* Update scheduler parameters */

      nxsched_resume_scheduler(tcb);

      /* Restore the cpu lock */

      restore_critical_section(tcb, this_cpu());

      /* Then switch contexts */

      sim_restorestate(tcb->xcp.regs);

      return;
    }

  /* Copy the exception context into the TCB of the task that was
   * previously active.  if sim_saveusercontext returns a non-zero value,
   * then this is really the previously running task restarting!
   */

  sim_saveusercontext(rtcb->xcp.regs, ret);
  if (ret == 0)
    {
      sinfo("New Active Task TCB=%p\n", tcb);

      /* Update scheduler parameters */

      nxsched_resume_scheduler(tcb);

      /* Restore the cpu lock */

      restore_critical_section(tcb, this_cpu());

      /* Record the new "running" task */

      g_running_tasks[this_cpu()] = tcb;

      /* Then switch contexts */

      sim_fullcontextrestore(tcb->xcp.regs);
    }
  else
    {
      /* The way that we handle signals in the simulation is kind of
       * a kludge.  This would be unsafe in a truly multi-threaded,
       * interrupt driven environment.
       */

      sim_sigdeliver();
    }
}
