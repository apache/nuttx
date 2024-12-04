/****************************************************************************
 * arch/z16/src/common/z16_doirq.c
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

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/board/board.h>
#include <sched/sched.h>

#include "chip.h"
#include "z16_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: z16_doirq
 *
 * Description:
 *   Interface between low-level IRQ decode logic and the NuttX IRQ dispatch
 *   logic.
 *
 ****************************************************************************/

FAR chipreg_t *z16_doirq(int irq, FAR chipreg_t *regs)
{
  FAR chipreg_t *ret = regs;

  board_autoled_on(LED_INIRQ);
#ifdef CONFIG_SUPPRESS_INTERRUPTS
  PANIC();
#else
  if ((unsigned)irq < NR_IRQS)
    {
      struct tcb_s **running_task = &g_running_tasks[this_cpu()];
      FAR chipreg_t *savestate;

      z16_copystate((*running_task)->xcp.regs, regs)

      /* Nested interrupts are not supported in this implementation.  If
       * you want to implement nested interrupts, you would have to (1)
       * change the way that g_current_regs is handled and (2) the design
       * ssociated with CONFIG_ARCH_INTERRUPTSTACK.  The savestate variable
       * will not work for that purpose as implemented here because only
       * the outermost nested interrupt can result in a context switch (it
       * can probably be deleted).
       */

      /* Current regs non-zero indicates that we are processing
       * an interrupt; g_current_regs is also used to manage
       * interrupt level context switches.
       */

      savestate = up_current_regs();
      up_set_current_regs(regs);

      /* Acknowledge the interrupt */

      z16_ack_irq(irq);

      /* Deliver the IRQ */

      irq_dispatch(irq, regs);

      if (regs != up_current_regs())
        {
          /* Record the new "running" task when context switch occurred.
           * g_running_tasks[] is only used by assertion logic for reporting
           * crashes.
           */

          *running_task = this_task();
        }

      /* Restore the previous value of g_current_regs.  NULL would indicate
       * that we are no longer in an interrupt handler.  It will be non-NULL
       * if we are returning from a nested interrupt.
       */

      ret = up_current_regs();
      up_set_current_regs(savestate);
    }

  board_autoled_off(LED_INIRQ);
#endif

  return ret;
}
