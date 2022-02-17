/****************************************************************************
 * arch/ceva/src/xm6/up_initialstate.c
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
#include <nuttx/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define REG_MODE_DEFAULT          0x40004000 /* PR14H and PR14L */

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
 *   This function must setup the intial architecture registers
 *   and/or  stack so that execution will begin at tcb->start
 *   on the next context switch.
 *
 ****************************************************************************/

void up_initial_state(struct tcb_s *tcb)
{
  struct xcptcontext *xcp = &tcb->xcp;

  /* Initialize the initial exception register context structure */

  memset(xcp, 0, sizeof(struct xcptcontext));

  if (tcb->stack_base_ptr)
    {
      xcp->regs = tcb->stack_base_ptr - XCPTCONTEXT_SIZE;
      memset(xcp->regs, 0, XCPTCONTEXT_SIZE);

      /* Save the initial stack pointer */

      xcp->regs[REG_SP]   = (uint32_t)xcp->regs;

      /* Save the task entry point */

      xcp->regs[REG_PC]   = (uint32_t)tcb->start;

      /* Initialize all no zero registers */

      xcp->regs[REG_MODA] = REG_MODA_DEFAULT;

      /* All tasks start via a stub function in kernel space.
       * So all tasks must start in privileged thread mode.
       * If CONFIG_BUILD_PROTECTED is defined,
       * then that stub function will switch to unprivileged
       * mode before transferring control to the user task.
       */

      xcp->regs[REG_OM]   = REG_OM_DEFAULT;

#ifdef CONFIG_ARCH_XM6_BUG001
      /* See BUG 001 in CEVA-XM6 Bug List */

      xcp->regs[REG_MODE] = REG_MODE_DEFAULT;
#endif
    }
}
