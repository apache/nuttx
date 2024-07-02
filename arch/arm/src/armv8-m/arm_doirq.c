/****************************************************************************
 * arch/arm/src/armv8-m/arm_doirq.c
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

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_from_thread
 *
 * Description:
 *   If not defined CONFIG_ARCH_HAVE_TRUSTZONE
 *   Return true if interrupt return to thread mode, false otherwise.
 *
 *   If defined CONFIG_ARCH_HAVE_TRUSTZONE
 *   Return true if interrupt return to thread mode, or if it is the first
 *   interrupt from TEE to REE, or REE to TEE, false otherwise.
 *
 *   Interrupt nesting between TEE and REE can be determined based
 *   on the S and ES bits of EXC_RETURN
 *   If TEE interrupts REE, then EXC_RETURN.S=0, EXC_RETURN.ES=1;
 *   Conversely, EXC_RETURN.S=1, EXC_RETURN.ES=0.
 *
 *   But only one level nesting between TEE and REE is supported, and
 *   recursive nesting between TEE and REE is not supported.
 *
 ****************************************************************************/

static inline bool arm_from_thread(uint32_t excret)
{
  if (excret & EXC_RETURN_THREAD_MODE)
    {
      return true;
    }

#if defined(CONFIG_ARCH_TRUSTZONE_SECURE)
  if (!(excret & EXC_RETURN_SECURE_STACK) &&
      (excret & EXC_RETURN_EXC_SECURE))
    {
      return true;
    }

  if (!(excret & EXC_RETURN_EXC_SECURE) &&
      (excret & EXC_RETURN_SECURE_STACK))
    {
      return true;
    }
#endif

  return false;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

uint32_t *arm_doirq(int irq, uint32_t *regs)
{
  board_autoled_on(LED_INIRQ);
#ifdef CONFIG_SUPPRESS_INTERRUPTS
  PANIC();
#else

  if (arm_from_thread(regs[REG_EXC_RETURN]))
    {
      CURRENT_REGS = regs;
    }

  /* Acknowledge the interrupt */

  arm_ack_irq(irq);

  /* Deliver the IRQ */

  irq_dispatch(irq, regs);

  /* If a context switch occurred while processing the interrupt then
   * CURRENT_REGS may have change value.  If we return any value different
   * from the input regs, then the lower level will know that a context
   * switch occurred during interrupt processing.
   */

  if (arm_from_thread(regs[REG_EXC_RETURN]))
    {
      /* Restore the cpu lock */

      if (regs != CURRENT_REGS)
        {
          /* Record the new "running" task when context switch occurred.
           * g_running_tasks[] is only used by assertion logic for reporting
           * crashes.
           */

          g_running_tasks[this_cpu()] = this_task();

          restore_critical_section();
          regs = (uint32_t *)CURRENT_REGS;
        }

      /* Update the CURRENT_REGS to NULL. */

      CURRENT_REGS = NULL;
    }
#endif

  board_autoled_off(LED_INIRQ);

#ifdef CONFIG_ARMV8M_TRUSTZONE_HYBRID
  if (((1 << up_cpu_index()) & CONFIG_ARMV8M_TRUSTZONE_CPU_BITMASK) == 0)
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
