/****************************************************************************
 * arch/tricore/src/common/tricore_svcall.c
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
#include <assert.h>
#include <debug.h>
#include <syscall.h>

#include <arch/irq.h>
#include <sched/sched.h>
#include <nuttx/sched.h>

#include "tricore_internal.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tricore_svcall
 *
 * Description:
 *   This is SVCall exception handler that performs context switching
 *
 ****************************************************************************/

void tricore_svcall(volatile void *trap)
{
  struct tcb_s **running_task = &g_running_tasks[this_cpu()];
  struct tcb_s *tcb = this_task();
  uintptr_t *regs;
  uint32_t cmd;

  regs = (uintptr_t *)__mfcr(CPU_PCXI);

  /* DSYNC instruction should be executed immediately prior to the MTCR */

  __dsync();

  regs = tricore_csa2addr((uintptr_t)regs);

  /* Set irq flag */

  up_set_interrupt_context(true);

  cmd = regs[REG_D8];

  if (cmd != SYS_restore_context)
    {
      (*running_task)->xcp.regs = tricore_csa2addr(regs[REG_UPCXI]);
    }
  else
    {
      tricore_reclaim_csa(regs[REG_UPCXI]);
    }

  /* Handle the SVCall according to the command in R0 */

  switch (cmd)
    {
      case SYS_switch_context:
        nxsched_switch_context(*running_task, tcb);

      case SYS_restore_context:
        *running_task = tcb;
        regs[REG_UPCXI] = tricore_addr2csa(tcb->xcp.regs);
        __isync();
        break;

      default:
        svcerr("ERROR: Bad SYS call: %d\n", (int)regs[REG_D0]);
        break;
    }

  /* Set irq flag */

  up_set_interrupt_context(false);
}
