/****************************************************************************
 * arch/avr/src/avr/up_initialstate.c
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
#include <stdint.h>
#include <string.h>

#include <nuttx/arch.h>
#include <arch/irq.h>
#include <avr/io.h>

#include "up_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_initial_state
 *
 * Description:
 *   A new thread is being started and a new TCB has been created. This
 *   function is called to initialize the processor specific portions of the
 *   new TCB.
 *
 *   This function must setup the initial architecture registers and/or stack
 *   so that execution will begin at tcb->start  on the next context switch.
 *
 ****************************************************************************/

void up_initial_state(struct tcb_s *tcb)
{
  struct xcptcontext *xcp = &tcb->xcp;
  uintptr_t sp;

  /* Initialize the idle thread stack */

  if (tcb->pid == IDLE_PROCESS_ID)
    {
      char *stack_ptr = (char *)(g_idle_topstack -
                                 CONFIG_IDLETHREAD_STACKSIZE);
#ifdef CONFIG_STACK_COLORATION
      char *stack_end = (char *)avr_getsp();

      /* If stack debug is enabled, then fill the stack with a
       * recognizable value that we can use later to test for high
       * water marks.
       */

      while (stack_ptr < stack_end)
        {
          *--stack_end = STACK_COLOR;
        }
#endif /* CONFIG_STACK_COLORATION */

      tcb->stack_alloc_ptr = stack_ptr;
      tcb->stack_base_ptr  = stack_ptr;
      tcb->adj_stack_size  = CONFIG_IDLETHREAD_STACKSIZE;
    }

  /* Initialize the initial exception register context structure.  Zeroing
   * all registers is a good debug helper, but should not be necessary.
   */

  memset(xcp, 0, sizeof(struct xcptcontext));

  /* Set the initial stack pointer to the top of the allocated stack */

  sp                   = (uintptr_t)tcb->stack_base_ptr +
                                    tcb->adj_stack_size;
  xcp->regs[REG_SPH]   = (uint8_t)(sp >> 8);
  xcp->regs[REG_SPL]   = (uint8_t)(sp & 0xff);

  /* Save the task entry point */

#if !defined(REG_PC2)
  xcp->regs[REG_PC0]   = (uint8_t)((uintptr_t)tcb->start >> 8);
  xcp->regs[REG_PC1]   = (uint8_t)((uintptr_t)tcb->start & 0xff);
#else
  xcp->regs[REG_PC0]   = (uint8_t)((uint32_t)(uintptr_t)tcb->start >> 16);
  xcp->regs[REG_PC1]   = (uint8_t)((uintptr_t)tcb->start >> 8);
  xcp->regs[REG_PC2]   = (uint8_t)((uintptr_t)tcb->start & 0xff);
#endif

  /* Enable or disable interrupts, based on user configuration */

#ifdef CONFIG_SUPPRESS_INTERRUPTS
  xcp->regs[REG_SREG]  = getsreg() & ~(1 << SREG_I);
#else
  xcp->regs[REG_SREG]  = getsreg() | (1 << SREG_I);
#endif
}
