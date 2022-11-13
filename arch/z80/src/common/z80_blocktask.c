/****************************************************************************
 * arch/z80/src/common/z80_blocktask.c
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

#include "chip/switch.h"
#include "sched/sched.h"
#include "group/group.h"
#include "z80_internal.h"

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

void up_block_task(FAR struct tcb_s *rtcb)
{
  /* Update scheduler parameters */

  nxsched_suspend_scheduler(rtcb);

  /* Are we in an interrupt handler? */

  if (IN_INTERRUPT())
    {
      /* Yes, then we have to do things differently.
       * Just copy the current registers into the OLD rtcb.
       */

      SAVE_IRQCONTEXT(rtcb);

      /* Restore the exception context of the rtcb at the (new) head
       * of the ready-to-run task list.
       */

      rtcb = this_task();

      /* Reset scheduler parameters */

      nxsched_resume_scheduler(rtcb);

      /* Then setup so that the context will be performed on exit
       * from the interrupt.  Any necessary address environment
       * changes will be made when the interrupt returns.
       */

      SET_IRQCONTEXT(rtcb);
    }

  /* Copy the user C context into the TCB at the (old) head of the
   * ready-to-run Task list. if SAVE_USERCONTEXT returns a non-zero
   * value, then this is really the previously running task restarting!
   */

  else if (!SAVE_USERCONTEXT(rtcb))
    {
      /* Restore the exception context of the rtcb at the (new) head
       * of the ready-to-run task list.
       */

      rtcb = this_task();

#ifdef CONFIG_ARCH_ADDRENV
      /* Make sure that the address environment for the previously
       * running task is closed down gracefully (data caches dump,
       * MMU flushed) and set up the address environment for the new
       * thread at the head of the ready-to-run list.
       */

      group_addrenv(rtcb);
#endif
      /* Reset scheduler parameters */

      nxsched_resume_scheduler(rtcb);

      /* Then switch contexts */

      RESTORE_USERCONTEXT(rtcb);
    }
}
