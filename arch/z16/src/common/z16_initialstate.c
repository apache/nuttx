/****************************************************************************
 * arch/z16/src/common/z16_initialstate.c
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

#include "chip.h"
#include "z16_internal.h"

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
  uint32_t *reg32          = (uint32_t *)tcb->xcp.regs;

  /* Initialize the initial exception register context structure */

  memset(&tcb->xcp, 0, sizeof(struct xcptcontext));
#ifndef CONFIG_SUPPRESS_INTERRUPTS
  tcb->xcp.regs[REG_FLAGS] = (uint16_t)Z16F_CNTRL_FLAGS_IRQE; /* IRQE flag will enable interrupts */
#endif
  reg32[REG_SP / 2]        = (uint32_t)tcb->adj_stack_ptr;
  reg32[REG_PC / 2]        = (uint32_t)tcb->start;
}
