/****************************************************************************
 * arch/z80/src/z180/z180_initialstate.c
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

#include <string.h>
#include <nuttx/arch.h>

#include "chip.h"
#include "z80_internal.h"

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
  struct xcptcontext *xcp = &tcb->xcp;

  /* Initialize the idle thread stack */

  if (tcb->pid == IDLE_PROCESS_ID)
    {
      char *stack_ptr = (char *)CONFIG_STACK_BASE;
#ifdef CONFIG_STACK_COLORATION
      char *stack_end = (char *)up_getsp();

      /* If stack debug is enabled, then fill the stack with a
       * recognizable value that we can use later to test for high
       * water marks.
       */

      while (stack_ptr < stack_end)
        {
          *--stack_end = 0xaa;
        }
#endif /* CONFIG_STACK_COLORATION */

      tcb->stack_alloc_ptr = stack_ptr;
      tcb->stack_base_ptr  = stack_ptr;
      tcb->adj_stack_size  = CONFIG_IDLETHREAD_STACKSIZE;
    }

  /* Initialize the initial exception register context structure */

  memset(xcp, 0, sizeof(struct xcptcontext));
#ifndef CONFIG_SUPPRESS_INTERRUPTS
  xcp->regs[XCPT_I]  = Z180_PV_FLAG; /* Parity flag will enable interrupts */
#endif
  xcp->regs[XCPT_SP] = (chipreg_t)tcb->stack_base_ptr +
                                  tcb->adj_stack_size;
  xcp->regs[XCPT_PC] = (chipreg_t)tcb->start;
}
