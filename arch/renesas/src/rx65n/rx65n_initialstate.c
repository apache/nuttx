/****************************************************************************
 * arch/renesas/src/rx65n/rx65n_initialstate.c
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

#include "chip/chip.h"
#include "up_internal.h"
#include "up_arch.h"
#include "arch/rx65n/irq.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define       up_getsr()      __getsr()

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
      up_use_stack(tcb, (void *)(g_idle_topstack -
        CONFIG_IDLETHREAD_STACKSIZE), CONFIG_IDLETHREAD_STACKSIZE);
    }

  /* Initialize the initial exception register context structure */

  memset(xcp, 0, sizeof(struct xcptcontext));

  xcp->regs[REG_SP] = (uint32_t)tcb->adj_stack_ptr;
  xcp->regs[REG_PC] = (uint32_t)tcb->start;

  /* Enable or disable interrupts, based on user configuration */

#ifdef CONFIG_SUPPRESS_INTERRUPTS
  xcp->regs[REG_PSW] = up_getsr() & ~0x00010000;
#else
  xcp->regs[REG_PSW] = up_getsr() | 0x00010000;
#endif
}
