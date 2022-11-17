/****************************************************************************
 * arch/sim/src/sim/sim_blocktask.c
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

#include <stdbool.h>
#include <sched.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/sched.h>

#include "sched/sched.h"
#include "sim_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_block_task
 *
 * Description:
 *   The currently executing task has already removed from ready-to-run list.
 *   Save its context and switch to the next running task at the head of the
 *   ready-to-run list.
 *
 * Input Parameters:
 *   rtcb: Reference to the running task which is different to the
 *     task (next running task) at the head of the list.
 *
 ****************************************************************************/

void up_block_task(struct tcb_s *rtcb)
{
  /* Update scheduler parameters */

  nxsched_suspend_scheduler(rtcb);

  /* TODO */

  if (CURRENT_REGS)
    {
      ASSERT(false);
    }

  /* Copy the exception context into the TCB at the (old) head of the
   * ready-to-run Task list. if setjmp returns a non-zero
   * value, then this is really the previously running task restarting!
   */

  else if (!setjmp(rtcb->xcp.regs))
    {
      /* Restore the exception context of the rtcb at the (new) head
       * of the ready-to-run task list.
       */

      rtcb = this_task();
      sinfo("New Active Task TCB=%p\n", rtcb);

      /* Reset scheduler parameters */

      nxsched_resume_scheduler(rtcb);

      /* Restore the cpu lock */

      restore_critical_section();

      /* Then switch contexts */

      longjmp(rtcb->xcp.regs, 1);
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
