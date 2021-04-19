/****************************************************************************
 * arch/sim/src/sim/up_initialstate.c
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

#include <stdint.h>
#include <string.h>

#include <nuttx/arch.h>

#include "up_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_initial_state
 *
 * Description:
 *   A new thread is being started and a new TCB
 *   has been created. This function is called to initialize
 *   the processor specific portions of the new TCB.
 *
 *   This function must setup the initial architecture registers
 *   and/or  stack so that execution will begin at tcb->start
 *   on the next context switch.
 *
 ****************************************************************************/

void up_initial_state(struct tcb_s *tcb)
{
  if (tcb->pid == 0)
    {
      tcb->stack_alloc_ptr = (void *)(sim_getsp() -
                                      CONFIG_IDLETHREAD_STACKSIZE);
      tcb->stack_base_ptr   = tcb->stack_alloc_ptr;
      tcb->adj_stack_size  = CONFIG_IDLETHREAD_STACKSIZE;
    }

  memset(&tcb->xcp, 0, sizeof(struct xcptcontext));

  /* Note: The amd64 ABI requires 16-bytes alignment _before_ a function
   * call.
   * On the other hand, our way to set up and switch to a new context
   * is basically a JUMP.
   * Thus, we need to emulate the effect of a CALL here, by subtracting
   * sizeof(xcpt_reg_t), which is the amount a CALL would move RSP to store
   * the return address.
   */

  tcb->xcp.regs[JB_SP] = (xcpt_reg_t)tcb->stack_base_ptr +
                                     tcb->adj_stack_size -
                                     sizeof(xcpt_reg_t);
  tcb->xcp.regs[JB_PC] = (xcpt_reg_t)tcb->start;
}
