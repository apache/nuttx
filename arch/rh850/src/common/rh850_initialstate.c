/****************************************************************************
 * arch/rh850/src/common/rh850_initialstate.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <nuttx/sched.h>

#include "rh850_internal.h"
#include "rh850_sysreg.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PSW of a new task: EI level interrupts enabled */

#ifdef CONFIG_ARCH_FPU
#  define RH850_PSW_NEWTASK  (RH850_PSW_EBV | RH850_PSW_CU0)
#  define RH850_FPSR_NEWTASK RH850_FPSR_FS
#else
#  define RH850_PSW_NEWTASK  RH850_PSW_EBV
#  define RH850_FPSR_NEWTASK 0
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_initial_state
 *
 * Description:
 *   The first frame of a new task sits at the top of its stack; EIRET
 *   enters tcb->start.  The idle task already runs on the idle stack.
 *
 ****************************************************************************/

void up_initial_state(FAR struct tcb_s *tcb)
{
  struct xcptcontext *xcp = &tcb->xcp;
  uintptr_t topstack;

  memset(xcp, 0, sizeof(struct xcptcontext));

  if (tcb->pid == IDLE_PROCESS_ID)
    {
      tcb->stack_alloc_ptr = (FAR void *)((uintptr_t)g_idle_topstack -
                                          CONFIG_IDLETHREAD_STACKSIZE);
      tcb->stack_base_ptr  = tcb->stack_alloc_ptr;
      tcb->adj_stack_size  = CONFIG_IDLETHREAD_STACKSIZE;

#ifdef CONFIG_STACK_COLORATION
      rh850_stack_color(tcb->stack_alloc_ptr, 0);
#endif
      return;
    }

  topstack  = (uintptr_t)tcb->stack_base_ptr + tcb->adj_stack_size;
  xcp->regs = (uintptr_t *)(topstack - XCPTCONTEXT_SIZE);
  memset(xcp->regs, 0, XCPTCONTEXT_SIZE);

  xcp->regs[REG_SP]    = topstack;
  xcp->regs[REG_EIPC]  = (uintptr_t)tcb->start;
  xcp->regs[REG_EIPSW] = RH850_PSW_NEWTASK;
  xcp->regs[REG_FPSR]  = RH850_FPSR_NEWTASK;
}
