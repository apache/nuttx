/****************************************************************************
 * arch/rh850/src/common/rh850_doirq.c
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

#include <assert.h>
#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "irq/irq.h"
#include "sched/sched.h"
#include "rh850_internal.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* EIINT nesting depth, changed only with PSW.ID = 1 */

volatile uint8_t g_rh850_irqnest;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rh850_doirq
 *
 * Description:
 *   Interrupts are enabled around the dispatch.  Only the outermost level
 *   switches tasks; inner levels return their own frame.
 *
 ****************************************************************************/

uintptr_t *rh850_doirq(int irq, uintptr_t *regs)
{
  struct tcb_s **running_task = &g_running_tasks[this_cpu()];
  struct tcb_s *tcb;

  if (g_rh850_irqnest++ == 0)
    {
      (*running_task)->xcp.regs = regs;
      up_set_interrupt_context(true);
    }

  /* Higher priorities nest from here; ISPR holds equal and lower ones */

  up_irq_enable();
  irq_dispatch(irq, regs);
  up_irq_save();

  if (--g_rh850_irqnest > 0)
    {
      return regs;
    }

  tcb = this_task();
  if (*running_task != tcb)
    {
      nxsched_switch_context(*running_task, tcb);
      *running_task = tcb;
    }

  up_set_interrupt_context(false);

  regs = tcb->xcp.regs;
  (*running_task)->xcp.regs = NULL;
  return regs;
}
