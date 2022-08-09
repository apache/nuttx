/****************************************************************************
 * arch/risc-v/src/common/riscv_initialstate.c
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
#include <nuttx/tls.h>
#include <arch/irq.h>

#include "riscv_internal.h"

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
  uintptr_t regval;

  /* Initialize the initial exception register context structure */

  memset(xcp, 0, sizeof(struct xcptcontext));

  /* Initialize the idle thread stack */

  if (tcb->pid == IDLE_PROCESS_ID)
    {
      tcb->stack_alloc_ptr = (void *)(g_idle_topstack -
                                      CONFIG_IDLETHREAD_STACKSIZE);
      tcb->stack_base_ptr  = tcb->stack_alloc_ptr;
      tcb->adj_stack_size  = CONFIG_IDLETHREAD_STACKSIZE;

#ifdef CONFIG_STACK_COLORATION
      /* If stack debug is enabled, then fill the stack with a
       * recognizable value that we can use later to test for high
       * water marks.
       */

      riscv_stack_color(tcb->stack_alloc_ptr, 0);
#endif /* CONFIG_STACK_COLORATION */
      return;
    }

  xcp->regs = (uintptr_t *)(
    (uintptr_t)tcb->stack_base_ptr + tcb->adj_stack_size - XCPTCONTEXT_SIZE);

  memset(xcp->regs, 0, XCPTCONTEXT_SIZE);

  /* Save the initial stack pointer.  Hmmm.. the stack is set to the very
   * beginning of the stack region.  Some functions may want to store data on
   * the caller's stack and it might be good to reserve some space.  However,
   * only the start function would do that and we have control over that one
   */

  xcp->regs[REG_SP]      = (uintptr_t)tcb->stack_base_ptr +
                                      tcb->adj_stack_size;

  /* Save the task entry point */

  xcp->regs[REG_EPC]     = (uintptr_t)tcb->start;

  /* Setup thread local storage pointer */

#ifdef CONFIG_SCHED_THREAD_LOCAL
  xcp->regs[REG_TP]      = (uintptr_t)tcb->stack_alloc_ptr +
                                     sizeof(struct tls_info_s);
#endif

  /* Set the initial value of the interrupt context register.
   *
   * Since various RISC-V platforms use different interrupt
   * methodologies, the value of the interrupt context is
   * part specific.
   *
   */

  regval = riscv_get_newintctx();
  xcp->regs[REG_INT_CTX] = regval;
}
