/****************************************************************************
 * arch/mips/src/mips32/mips_initialstate.c
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
#include <arch/mips32/cp0.h>

#include "mips_internal.h"

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
 *   and/or stack so that execution will begin at tcb->start
 *   on the next context switch.
 *
 ****************************************************************************/

void up_initial_state(struct tcb_s *tcb)
{
  struct xcptcontext *xcp = &tcb->xcp;
  uint32_t regval;

  /* Initialize the idle thread stack */

  if (tcb->pid == IDLE_PROCESS_ID)
    {
      char *stack_ptr = (char *)(g_idle_topstack -
                                 CONFIG_IDLETHREAD_STACKSIZE);
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

  /* Save the initial stack pointer.  Hmmm.. the stack is set to the very
   * beginning of the stack region.  Some functions may want to store data on
   * the caller's stack and it might be good to reserve some space.  However,
   * only the start function would do that and we have control over that one.
   */

  xcp->regs[REG_SP] = (uint32_t)tcb->stack_base_ptr +
                                tcb->adj_stack_size;

  /* Save the task entry point */

  xcp->regs[REG_EPC] = (uint32_t)tcb->start;

  /* If this task is running PIC, then set the PIC base register to the
   * address of the allocated D-Space region.
   */

#ifdef CONFIG_PIC
#  warning "Missing logic"
#endif

  /* Set privileged- or unprivileged-mode, depending on how NuttX is
   * configured and what kind of thread is being started.
   *
   * If the kernel build is not selected, then all threads run in
   * privileged thread mode.
   */

#ifdef CONFIG_BUILD_KERNEL
#  warning "Missing logic"
#endif

  /* Set the initial value of the status register.  It will be the same
   * as the current status register with some changes:
   *
   * 1. Make sure the IE is set
   * 2. Clear the BEV bit (This bit should already be clear)
   * 3. Clear the UM bit so that the new task executes in kernel mode
   *   (This bit should already be clear)
   * 4. Set the interrupt mask bits (depending on configuration)
   * 5. Set the EXL bit (This will not be set)
   *
   * The EXL bit is set because this new STATUS register will be
   * instantiated in kernel mode inside of an interrupt handler. EXL
   * will be automatically cleared by the eret instruction.
   */

  regval  = cp0_getstatus();
#ifdef CONFIG_SUPPRESS_INTERRUPTS
  regval &= ~(CP0_STATUS_INT_MASK | CP0_STATUS_BEV | CP0_STATUS_UM);
  regval |=  (CP0_STATUS_IE | CP0_STATUS_EXL | CP0_STATUS_INT_SW0);
#else
  regval &= ~(CP0_STATUS_BEV | CP0_STATUS_UM);
  regval &= ~CP0_STATUS_INT_MASK;
  regval |=  (CP0_STATUS_IE | CP0_STATUS_EXL);
#endif
  xcp->regs[REG_STATUS] = regval;
}
