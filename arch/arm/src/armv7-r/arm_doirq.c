/****************************************************************************
 * arch/arm/src/armv7-r/arm_doirq.c
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
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <assert.h>

#include <nuttx/board.h>
#include <arch/board/board.h>
#include <sched/sched.h>

#include "arm_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

uint32_t *arm_doirq(int irq, uint32_t *regs)
{
  struct tcb_s *tcb = this_task();

  board_autoled_on(LED_INIRQ);

#ifdef CONFIG_SUPPRESS_INTERRUPTS
  PANIC();
#else
  /* Nested interrupts are not supported */

  DEBUGASSERT(!up_interrupt_context());

  /* if irq == GIC_SMP_CPUSTART
   * We are initiating the multi-core jumping state to up_idle,
   * and we will use this_task(). Therefore, it cannot be overridden.
   */

#ifdef CONFIG_SMP
  if (irq != GIC_SMP_CPUSTART)
#endif
    {
      tcb->xcp.regs = regs;
    }

  /* Set irq flag */

  up_set_interrupt_context(true);

  /* Deliver the IRQ */

  irq_dispatch(irq, regs);
  tcb = this_task();

  if (regs != tcb->xcp.regs)
    {
      /* Update scheduler parameters */

      nxsched_suspend_scheduler(g_running_tasks[this_cpu()]);
      nxsched_resume_scheduler(tcb);

      /* Record the new "running" task when context switch occurred.
       * g_running_tasks[] is only used by assertion logic for reporting
       * crashes.
       */

      g_running_tasks[this_cpu()] = tcb;
      regs = tcb->xcp.regs;
    }

  /* Set irq flag */

  up_set_interrupt_context(false);
  board_autoled_off(LED_INIRQ);
#endif
  return regs;
}
