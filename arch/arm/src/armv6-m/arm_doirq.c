/****************************************************************************
 * arch/arm/src/armv6-m/arm_doirq.c
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
#include <assert.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/board/board.h>
#include <sched/sched.h>

#include "arm_internal.h"
#include "exc_return.h"
#include "nvic.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void exception_direct(void)
{
  int irq = getipsr();

  arm_ack_irq(irq);
  irq_dispatch(irq, NULL);

  if (g_running_tasks[this_cpu()] != this_task())
    {
      up_trigger_irq(NVIC_IRQ_PENDSV, 0);
    }
}

uint32_t *arm_doirq(int irq, uint32_t *regs)
{
  struct tcb_s *tcb = this_task();

  board_autoled_on(LED_INIRQ);
#ifdef CONFIG_SUPPRESS_INTERRUPTS
  PANIC();
#else

  /* Acknowledge the interrupt */

  arm_ack_irq(irq);

  /* Set current regs for crash dump */

  up_set_current_regs(regs);

  if (irq == NVIC_IRQ_PENDSV)
    {
#ifdef CONFIG_ARCH_HIPRI_INTERRUPT
      /* Dispatch the PendSV interrupt */

      irq_dispatch(irq, regs);
#endif

      up_irq_save();
      g_running_tasks[this_cpu()]->xcp.regs = regs;
    }
  else
    {
      /* Dispatch irq */

      tcb->xcp.regs = regs;
      irq_dispatch(irq, regs);
    }

  /* If a context switch occurred while processing the interrupt then
   * current_regs may have change value.  If we return any value different
   * from the input regs, then the lower level will know that a context
   * switch occurred during interrupt processing.
   */

  tcb = this_task();

  /* Update scheduler parameters */

  nxsched_suspend_scheduler(g_running_tasks[this_cpu()]);
  nxsched_resume_scheduler(tcb);

  /* Record the new "running" task when context switch occurred.
   * g_running_tasks[] is only used by assertion logic for reporting
   * crashes.
   */

  g_running_tasks[this_cpu()] = tcb;
  regs = tcb->xcp.regs;
#endif

  /* Clear current regs */

  up_set_current_regs(NULL);

  board_autoled_off(LED_INIRQ);

  return regs;
}
