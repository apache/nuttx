/****************************************************************************
 * arch/xtensa/src/common/xtensa_initialstate.c
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
#include <arch/xtensa/core.h>
#include <arch/chip/core-isa.h>
#include <arch/xtensa/xtensa_corebits.h>
#include <arch/xtensa/xtensa_coproc.h>

#include "xtensa.h"

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
 *   so that execution will begin at tcb->start on the next context switch.
 *
 ****************************************************************************/

void up_initial_state(struct tcb_s *tcb)
{
  struct xcptcontext *xcp = &tcb->xcp;

  /* Initialize the idle thread stack */

  if (tcb->pid == 0)
    {
      tcb->stack_alloc_ptr = g_idlestack;
      tcb->stack_base_ptr  = tcb->stack_alloc_ptr;
      tcb->adj_stack_size  = CONFIG_IDLETHREAD_STACKSIZE;

#ifdef CONFIG_STACK_COLORATION
      /* If stack debug is enabled, then fill the stack with a
       * recognizable value that we can use later to test for high
       * water marks.
       */

      xtensa_stack_color(tcb->stack_alloc_ptr, 0);
#endif /* CONFIG_STACK_COLORATION */
    }

  /* Initialize the initial exception register context structure */

  memset(xcp, 0, sizeof(struct xcptcontext));

  /* Set initial values of registers */

  xcp->regs[REG_PC]   = (uint32_t)tcb->start;           /* Task entrypoint                */
  xcp->regs[REG_A0]   = 0;                              /* To terminate GDB backtrace     */
  xcp->regs[REG_A1]   = (uint32_t)tcb->stack_base_ptr + /* Physical top of stack frame    */
                                  tcb->adj_stack_size;

  /* Set initial PS to int level 0, user mode. */

#ifdef __XTENSA_CALL0_ABI__
  xcp->regs[REG_PS]   = PS_UM;

#else
  /* For windowed ABI set WOE and CALLINC (pretend task was 'call4'd). */

  xcp->regs[REG_PS]   = PS_UM | PS_WOE | PS_CALLINC(1);
#endif

#if XCHAL_CP_NUM > 0
  /* Set up the co-processors that will be enabled initially when the thread
   * starts (see xtensa_coproc.h).  If the lazy co-processor state restore
   * logic is selected, that would be the empty set.
   */

#ifdef CONFIG_XTENSA_CP_LAZY
  xcp->cpstate.cpenable = 0;  /* No co-processors are enabled */
#else
  xcp->cpstate.cpenable = (CONFIG_XTENSA_CP_INITSET & XTENSA_CP_ALLSET);
#endif
  xcp->cpstate.cpstored = 0;  /* No co-processors haved state saved */
#endif
}
