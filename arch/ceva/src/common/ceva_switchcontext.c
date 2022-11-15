/****************************************************************************
 * arch/ceva/src/common/ceva_switchcontext.c
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

#include <nuttx/arch.h>
#include <arch/syscall.h>

#include "sched/sched.h"
#include "ceva_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ceva_switchcontext
 *
 * Description:
 *   Save the current thread context and restore the specified context.
 *   Full prototype is:
 *
 *   void ceva_switchcontext(uint32_t **saveregs, uint32_t *restoreregs);
 *
 * Return:
 *   None
 *
 ****************************************************************************/

void ceva_switchcontext(uint32_t **saveregs, uint32_t *restoreregs)
{
  /* Let sys_call2() do all of the work */

  sys_call2(SYS_switch_context, (uintptr_t)saveregs, (uintptr_t)restoreregs);
}

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

  sched_suspend_scheduler(rtcb);

  /* Are we in an interrupt handler? */

  if (CURRENT_REGS)
    {
      /* Yes, then we have to do things differently.
       * Just copy the CURRENT_REGS into the OLD rtcb.
       */

      rtcb->xcp.regs = CURRENT_REGS;

      /* Update scheduler parameters */

      sched_resume_scheduler(tcb);

      /* Then switch contexts */

      CURRENT_REGS = tcb->xcp.regs;
    }

  /* No, then we will need to perform the user context switch */

  else
    {
      /* Update scheduler parameters */

      sched_resume_scheduler(tcb);

      /* Switch context to the context of the task at the head of the
       * ready to run list.
       */

      ceva_switchcontext(&rtcb->xcp.regs, tcb->xcp.regs);

      /* ceva_switchcontext forces a context switch to the task at the
       * head of the ready-to-run list.  It does not 'return' in the
       * normal sense.  When it does return, it is because the blocked
       * task is again ready to run and has execution priority.
       */
    }
}
