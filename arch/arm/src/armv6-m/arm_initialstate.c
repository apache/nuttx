/****************************************************************************
 * arch/arm/src/armv6-m/arm_initialstate.c
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

#include "arm_internal.h"
#include "arm_arch.h"

#include "psr.h"
#include "exc_return.h"

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

  xcp->regs[REG_SP]      = (uint32_t)tcb->adj_stack_ptr;

  /* Save the task entry point (stripping off the thumb bit) */

  xcp->regs[REG_PC]      = (uint32_t)tcb->start & ~1;

  /* Specify thumb mode */

  xcp->regs[REG_XPSR]    = ARMV6M_XPSR_T;

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

#ifdef CONFIG_NXFLAT
  /* Make certain that bit 0 is set in the main entry address.  This is
   * only an issue when NXFLAT is enabled.  NXFLAT doesn't know anything
   * about thumb; the addresses that NXFLAT sets are based on file header
   * info and won't have bit 0 set.
   */

  tcb->entry.main = (main_t)((uint32_t)tcb->entry.main | 1);
#endif
#endif /* CONFIG_PIC */

#ifdef CONFIG_BUILD_PROTECTED
  /* All tasks start via a stub function in kernel space.  So all
   * tasks must start in privileged thread mode.  If CONFIG_BUILD_PROTECTED
   * is defined, then that stub function will switch to unprivileged
   * mode before transferring control to the user task.
   */

  xcp->regs[REG_EXC_RETURN] = EXC_RETURN_PRIVTHR;
#endif

  /* Enable or disable interrupts, based on user configuration */

#ifdef CONFIG_SUPPRESS_INTERRUPTS
  xcp->regs[REG_PRIMASK] = 1;
#endif /* CONFIG_SUPPRESS_INTERRUPTS */
}
