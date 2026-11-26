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
#include <nuttx/tls.h>

#include <arch/barriers.h>
#include <arch/board/board.h>

#include <sched/sched.h>

#include "tricore_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQPRIO

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

void tricore_irq_direct(void)
{
  struct tcb_s **running_task = &g_running_tasks[this_cpu()];
  uintptr_t *regs = tricore_csa2addr(tricore_mfcr(TRICORE_CPU_PCXI));
  int irq = tricore_irq_from_lasr();

  up_irq_enable();

  tricore_ack_irq(irq);
  irq_dispatch(irq, regs);

  if (*running_task != this_task())
    {
      cpu_set_t cpuset;

      CPU_ZERO(&cpuset);
      CPU_SET(up_cpu_index(), &cpuset);
      up_trigger_irq(TRICORE_SCHEDIRQ, cpuset);
    }

  tricore_jump_backto_link();
}

#endif /* CONFIG_ARCH_IRQPRIO */

/****************************************************************************
 * Name: tricore_irq_sched
 *
 * Description:
 *   Context-switch interrupt path. Two dispatch schemes share this path
 *   with different roles, selected by CONFIG_ARCH_IRQPRIO.
 *
 *   CONFIG_ARCH_IRQPRIO=y:
 *     SRPN 1 -> tricore_irq_sched (lowest priority, task switch only),
 *     SRPN 2~MAX -> tricore_irq_direct (fast dispatch, nestable).
 *   CONFIG_ARCH_IRQPRIO=n:
 *     tricore_irq_sched does dispatch and task switch for all IRQs.
 *     No nesting support and no scheduling GPSR.
 *
 ****************************************************************************/

void tricore_irq_sched(void)
{
  struct tcb_s **running_task = &g_running_tasks[this_cpu()];
  uintptr_t *regs = tricore_csa2addr(tricore_mfcr(TRICORE_CPU_PCXI));
  struct tcb_s *tcb;
#ifndef CONFIG_ARCH_IRQPRIO
  int irq = tricore_irq_from_lasr();
#endif

  if (*running_task != NULL)
    {
      (*running_task)->xcp.regs = regs;
      tricore_store_pprs(*running_task);

#ifdef CONFIG_ARCH_IRQPRIO
      if ((*running_task)->sigdeliver && !(*running_task)->xcp.saved_regs)
        {
          up_schedule_sigaction(*running_task);
        }
#endif
    }

#ifdef CONFIG_ARCH_HIPRI_INTERRUPT
  up_irq_save();
  up_irq_enable();
#endif

  board_autoled_on(LED_INIRQ);

#ifndef CONFIG_ARCH_IRQPRIO
  tricore_ack_irq(irq);
  irq_dispatch(irq, regs);
#endif

  tcb = this_task();

  /* Check for a context switch. */

  if (*running_task != tcb)
    {
      /* Update scheduler parameters */

      nxsched_switch_context(*running_task, tcb);

#ifdef CONFIG_ARCH_ADDRENV
      /* Make sure that the address environment for the previously
       * running task is closed down gracefully (data caches dump,
       * MMU flushed) and set up the address environment for the new
       * thread at the head of the ready-to-run list.
       */

      addrenv_switch(tcb);
      tcb = this_task();
#endif

      /* Update the TLS pointer */

      tricore_set_tls_info((uintptr_t)tcb->stack_alloc_ptr +
                           sizeof(struct tls_info_s));

      /* Record the new "running" task when context switch occurred.
       * g_running_tasks[] is only used by assertion logic for reporting
       * crashes.
       */

      *running_task = tcb;
    }

  /* The saved context may also reference a signal-delivery CSA. */

  tricore_restore_pprs(tcb);
  tricore_mtcr(TRICORE_CPU_PCXI, tricore_addr2csa(tcb->xcp.regs));
  UP_ISB();

  /* (*running_task)->xcp.regs is about to become invalid and
   * will be marked as NULL to avoid misusage. the same applies
   * to (*running_task)->xcp.pprs.
   */

  (*running_task)->xcp.regs = NULL;
  tricore_change_pprs(*running_task, UINT32_MAX);

  board_autoled_off(LED_INIRQ);
  tricore_jump_backto_link();
}
