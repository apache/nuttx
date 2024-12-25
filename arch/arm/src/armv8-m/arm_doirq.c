/****************************************************************************
 * arch/arm/src/armv8-m/arm_doirq.c
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

#include "arm_internal.h"
#include "exc_return.h"
#include "nvic.h"
#include "psr.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void exception_direct(void)
{
  int irq = getipsr();

#ifdef CONFIG_ARCH_FPU
  __asm__ __volatile__
    (
      "mov r0, %0\n"
      "vmsr fpscr, r0\n"
      :
      : "i" (ARMV8M_FPSCR_LTPSIZE_NONE)
    );
#endif

  arm_ack_irq(irq);
  irq_dispatch(irq, NULL);

  if (g_running_tasks[this_cpu()] != this_task())
    {
      up_trigger_irq(NVIC_IRQ_PENDSV, 0);
    }
}

uint32_t *arm_doirq(int irq, uint32_t *regs)
{
  struct tcb_s **running_task = &g_running_tasks[this_cpu()];
  struct tcb_s *tcb           = *running_task;

  /* This judgment proves that (*running_task)->xcp.regs
   * is invalid, and we can safely overwrite it.
   */

  if (!(NVIC_IRQ_SVCALL == irq && regs[REG_R0] == SYS_restore_context))
    {
      tcb->xcp.regs = regs;
    }

  board_autoled_on(LED_INIRQ);
#ifdef CONFIG_SUPPRESS_INTERRUPTS
  PANIC();
#else

  /* Acknowledge the interrupt */

  arm_ack_irq(irq);

  if (irq == NVIC_IRQ_PENDSV)
    {
#ifdef CONFIG_ARCH_HIPRI_INTERRUPT
      /* Dispatch the PendSV interrupt */

      irq_dispatch(irq, regs);
#endif
      if ((tcb->flags & TCB_FLAG_SIGDELIVER) != 0)
        {
          /* Pendsv able to access running tcb with no critical section */

          up_schedule_sigaction(tcb);
        }

      up_irq_save();
    }
  else
    {
      irq_dispatch(irq, regs);
    }

  tcb = this_task();

  /* Update scheduler parameters */

  nxsched_suspend_scheduler(*running_task);
  nxsched_resume_scheduler(tcb);

  /* Record the new "running" task when context switch occurred.
   * g_running_tasks[] is only used by assertion logic for reporting
   * crashes.
   */

  *running_task = tcb;
  regs = tcb->xcp.regs;
#endif

  board_autoled_off(LED_INIRQ);

#ifdef CONFIG_ARMV8M_TRUSTZONE_HYBRID
  if (((1 << this_cpu()) & CONFIG_ARMV8M_TRUSTZONE_CPU_BITMASK) == 0)
    {
      regs[REG_EXC_RETURN] &=
        ~(EXC_RETURN_EXC_SECURE | EXC_RETURN_SECURE_STACK);
    }
  else
    {
      regs[REG_EXC_RETURN] |=
        (EXC_RETURN_EXC_SECURE | EXC_RETURN_SECURE_STACK);
    }
#endif

  return regs;
}
