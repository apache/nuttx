/****************************************************************************
 * arch/arm/src/armv7-a/arm_initialstate.c
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

#include "arm.h"
#include "arm_internal.h"
#include "arm_arch.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_initial_state
 *
 * Description:
 *   A new thread is being started and a new TCB has been created. This
 *   function is called to initialize the processor specific portions of
 *   the new TCB.
 *
 *   This function must setup the initial architecture registers and/or
 *   stack so that execution will begin at tcb->start on the next context
 *   switch.
 *
 ****************************************************************************/

void up_initial_state(struct tcb_s *tcb)
{
  struct xcptcontext *xcp = &tcb->xcp;
  uint32_t cpsr;

  /* Initialize the idle thread stack */

  if (tcb->pid == 0)
    {
      tcb->stack_alloc_ptr = (void *)(g_idle_topstack -
                                      CONFIG_IDLETHREAD_STACKSIZE);
      tcb->adj_stack_ptr   = (void *)g_idle_topstack;
      tcb->adj_stack_size  = CONFIG_IDLETHREAD_STACKSIZE;
    }

  /* Initialize the initial exception register context structure */

  memset(xcp, 0, sizeof(struct xcptcontext));

  /* Save the initial stack pointer */

  xcp->regs[REG_SP] = (uint32_t)tcb->adj_stack_ptr;

  /* Save the task entry point */

  xcp->regs[REG_PC] = (uint32_t)tcb->start;

  /* If this task is running PIC, then set the PIC base register to the
   * address of the allocated D-Space region.
   */

#ifdef CONFIG_PIC
  if (tcb->dspace != NULL)
    {
      /* Set the PIC base register (probably R10) to the address of the
       * alloacated D-Space region.
       */

      xcp->regs[REG_PIC] = (uint32_t)tcb->dspace->region;
    }
#endif

  /* Set supervisor-mode and disable FIQs, regardless of how NuttX is
   * configured and of what kind of thread is being started.  That is
   * because all threads, even user-mode threads will start in kernel
   * trampoline at nxtask_start() or pthread_start().  The thread's
   * privileges will be dropped before transitioning to user code.
   */

  cpsr = PSR_MODE_SVC;

  /* Enable or disable interrupts, based on user configuration */

#ifdef CONFIG_SUPPRESS_INTERRUPTS
  /* Disable interrupts (both IRQs and FIQs) */

  cpsr |= (PSR_I_BIT | PSR_F_BIT);

#else /* CONFIG_SUPPRESS_INTERRUPTS */
  /* Leave IRQs enabled (Also FIQs if CONFIG_ARMV7A_DECODEFIQ is selected) */

#ifndef CONFIG_ARMV7A_DECODEFIQ

  cpsr |= PSR_F_BIT;

#endif /* !CONFIG_ARMV7A_DECODEFIQ */
#endif /* CONFIG_SUPPRESS_INTERRUPTS */

#ifdef CONFIG_ARM_THUMB
  cpsr |= PSR_T_BIT;
#endif

  xcp->regs[REG_CPSR] = cpsr;
}
