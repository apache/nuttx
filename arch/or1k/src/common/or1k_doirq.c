/****************************************************************************
 * arch/or1k/src/common/or1k_doirq.c
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
#include <assert.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/board/board.h>
#include <sched/sched.h>

#include "or1k_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

uint32_t *or1k_doirq(int irq, uint32_t *regs)
{
  struct tcb_s **running_task = &g_running_tasks[this_cpu()];

  or1k_copyfullstate((*running_task)->xcp.regs, regs);

  board_autoled_on(LED_INIRQ);
#ifdef CONFIG_SUPPRESS_INTERRUPTS
  PANIC();
#else
  uint32_t *savestate;

  regs = NULL;

  /* Current regs non-zero indicates that we are processing an interrupt;
   * current_regs is also used to manage interrupt level context switches.
   */

  savestate = up_current_regs();
  up_set_current_regs(regs);

  /* Acknowledge the interrupt */

  or1k_ack_irq(irq);

  /* Deliver the IRQ */

  irq_dispatch(irq, regs);

  /* If a context switch occurred while processing the interrupt then
   * current_regs may have changed value.  If we return any value different
   * from the input regs, then the lower level will know that a context
   * switch occurred during interrupt processing.
   */

  if (regs != up_current_regs())
    {
      /* Record the new "running" task when context switch occurred.
       * g_running_tasks[] is only used by assertion logic for reporting
       * crashes.
       */

      *running_task = this_task();
    }

  regs = up_current_regs();

  /* Restore the previous value of current_regs.  NULL would indicate that
   * we are no longer in an interrupt handler.  It will be non-NULL if we
   * are returning from a nested interrupt.
   */

  up_set_current_regs(savestate);
#endif
  board_autoled_off(LED_INIRQ);
  return regs;
}
