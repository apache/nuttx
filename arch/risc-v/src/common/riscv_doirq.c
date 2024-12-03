/****************************************************************************
 * arch/risc-v/src/common/riscv_doirq.c
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
#include <nuttx/addrenv.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/board/board.h>
#include <sched/sched.h>

#include "riscv_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

uintreg_t *riscv_doirq(int irq, uintreg_t *regs)
{
  struct tcb_s **running_task = &g_running_tasks[this_cpu()];
  bool restore_context = false;
  struct tcb_s *tcb = this_task();

  board_autoled_on(LED_INIRQ);
#ifdef CONFIG_SUPPRESS_INTERRUPTS
  PANIC();
#else

  /* NOTE: In case of ecall, we need to adjust mepc in the context */

  if (irq >= RISCV_IRQ_ECALLU && irq <= RISCV_IRQ_ECALLM)
    {
      regs[REG_EPC] += 4;
      if (regs[REG_A0] != SYS_restore_context)
        {
          (*running_task)->xcp.regs = regs;
        }
      else
        {
          restore_context = true;
        }
    }
  else
    {
      (*running_task)->xcp.regs = regs;
    }

  /* Nested interrupts are not supported */

  DEBUGASSERT(!up_interrupt_context());

  /* Set irq flag */

  up_set_interrupt_context(true);

  /* Deliver the IRQ */

  irq_dispatch(irq, regs);
  tcb = this_task();

  /* Check for a context switch. */

  if (*running_task != tcb || restore_context)
    {
#ifdef CONFIG_ARCH_ADDRENV
      /* Make sure that the address environment for the previously
       * running task is closed down gracefully (data caches dump,
       * MMU flushed) and set up the address environment for the new
       * thread at the head of the ready-to-run list.
       */

      addrenv_switch(tcb);
#endif

      /* Update scheduler parameters */

      if (!restore_context)
        {
          nxsched_suspend_scheduler(*running_task);
        }

      nxsched_resume_scheduler(tcb);

      /* Record the new "running" task when context switch occurred.
       * g_running_tasks[] is only used by assertion logic for reporting
       * crashes.
       */

      *running_task = tcb;
    }

  /* Set irq flag */

  up_set_interrupt_context(false);

#endif
  board_autoled_off(LED_INIRQ);
  return tcb->xcp.regs;
}
