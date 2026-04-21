/****************************************************************************
 * arch/tricore/src/common/tricore_doirq.c
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
#include <nuttx/addrenv.h>
#include <nuttx/board.h>

#include <arch/barriers.h>
#include <arch/board/board.h>

#include <sched/sched.h>

#include "tricore_internal.h"

#include "IfxCpu.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tricore_irq_direct
 *
 * Description:
 *   Fast interrupt path (analogous to ARM-M exception_direct).
 *   Only dispatches the IRQ handler, no context switch.
 *   If a context switch is needed, triggers the scheduling GPSR which
 *   will execute after all higher-priority interrupts have returned.
 *
 ****************************************************************************/

static void tricore_irq_direct(uintptr_t *regs, int ccpn)
{
  struct tcb_s **running_task = &g_running_tasks[this_cpu()];
  int irq;

#ifdef CONFIG_ARCH_IRQPRIO
  up_irq_enable();
#endif

  irq = tricore_srpn_to_irq(ccpn);
  tricore_ack_irq(irq);
  irq_dispatch(irq, regs);

  if (*running_task != this_task())
    {
      cpu_set_t cpuset;

      CPU_ZERO(&cpuset);
      CPU_SET(up_cpu_index(), &cpuset);
      up_trigger_irq(TRICORE_SCHEDIRQ, cpuset);
    }
}

/****************************************************************************
 * Name: tricore_irq_sched
 *
 * Description:
 *   Scheduling GPSR path for context switch (analogous to arm_doirq).
 *   This is triggered by tricore_irq_direct when it detects that a
 *   context switch is needed.
 *
 *   Because the GPSR SRPN is the lowest priority, this only executes
 *   after all normal interrupts have completed, guaranteeing safe
 *   context switch without race conditions.
 *
 ****************************************************************************/

static void tricore_irq_sched(uintptr_t *regs)
{
  struct tcb_s **running_task = &g_running_tasks[this_cpu()];
  struct tcb_s *tcb;

#ifdef CONFIG_ARCH_IRQPRIO
  up_irq_enable();
#endif

  if (*running_task != NULL)
    {
      (*running_task)->xcp.regs = regs;
    }

  board_autoled_on(LED_INIRQ);

  tcb = this_task();

  /* Check for a context switch. */

  if (*running_task != tcb)
    {
#ifdef CONFIG_ARCH_ADDRENV
      /* Make sure that the address environment for the previously
       * running task is closed down gracefully (data caches dump,
       * MMU flushed) and set up the address environment for the new
       * thread at the head of the ready-to-run list.
       */

      addrenv_switch(tcb);
      tcb = this_task();
#endif

      /* Update scheduler parameters */

      nxsched_switch_context(*running_task, tcb);

      /* Record the new "running" task when context switch occurred.
       * g_running_tasks[] is only used by assertion logic for reporting
       * crashes.
       */

      *running_task = tcb;

      tricore_mtcr(CPU_PCXI, tricore_addr2csa(tcb->xcp.regs));
      UP_ISB();
    }

  /* running_task->xcp.regs is about to become invalid
   * and will be marked as NULL to avoid misusage.
   */

  (*running_task)->xcp.regs = NULL;
  board_autoled_off(LED_INIRQ);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

IFX_INTERRUPT_INTERNAL(tricore_doirq, 0, 255)
{
  Ifx_CPU_ICR icr;
  uintptr_t *regs;

  icr.U = tricore_mfcr(CPU_ICR);
  regs = tricore_csa2addr(tricore_mfcr(CPU_PCXI));

  if (icr.B.CCPN == TRICORE_IRQ_MIN_PRIORITY)
    {
      tricore_irq_sched(regs);
    }
  else
    {
      tricore_irq_direct(regs, icr.B.CCPN);
    }
}
