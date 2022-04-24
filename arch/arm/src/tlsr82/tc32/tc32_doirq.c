/****************************************************************************
 * arch/arm/src/tlsr82/tc32/tc32_doirq.c
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

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/compiler.h>
#include <nuttx/board.h>

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <arch/board/board.h>

#include "arm_internal.h"

#include "group/group.h"

#include "hardware/tlsr82_irq.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint8_t tc32_lowbit_bitmap[] =
{
  0, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0, /* 00 */
  4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0, /* 10 */
  5, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0, /* 20 */
  4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0, /* 30 */
  6, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0, /* 40 */
  4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0, /* 50 */
  5, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0, /* 60 */
  4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0, /* 70 */
  7, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0, /* 80 */
  4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0, /* 90 */
  5, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0, /* A0 */
  4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0, /* B0 */
  6, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0, /* C0 */
  4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0, /* D0 */
  5, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0, /* E0 */
  4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0, /* F0 */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tc32_ffs
 *
 * Description:
 *   This function finds the first bit set (beginning with the least
 *   significant bit) in value and return the index of that bit.
 *   TC32 archtecture does not support clz instruction.
 *
 * Parameters:
 *   uint32_t - value
 *
 * Return Value:
 *   [1, 32] - On Success.
 *   0       - No set bit in value (value = 0).
 *
 ****************************************************************************/

static inline int locate_code(".ram_code") tc32_ffs(uint32_t value)
{
  int ret;

  if (value == 0)
    {
      ret = 0;
    }
  else if (value & 0xff)
    {
      ret = (int)tc32_lowbit_bitmap[value & 0xff] + 1;
    }
  else if (value & 0xff00)
    {
      ret = (int)tc32_lowbit_bitmap[(value & 0xff00) >> 8] + 9;
    }
  else if (value & 0xff0000)
    {
      ret = (int)tc32_lowbit_bitmap[(value & 0xff0000) >> 16] + 17;
    }
  else
    {
      ret = (int)tc32_lowbit_bitmap[(value & 0xff000000) >> 24] + 25;
    }

  return ret;
}

/****************************************************************************
 * Name: tc32_getirq
 *
 * Description:
 *   This function is used to get the interrupt number based on the
 *   interrupt source flag register.
 *
 * Parameters:
 *   void
 *
 * Return Value:
 *   [0, NR_IRQS-1] - On success, found interrupt number, success.
 *   NR_IRQS        - Error, invalid interrupt number.
 *
 ****************************************************************************/

static int locate_code(".ram_code") tc32_getirq(void)
{
  int irq;

  /* Only detect the enable interrupt */

  irq = tc32_ffs(IRQ_SRC_REG & IRQ_MASK_REG);

  if (irq > 0 && irq <= NR_IRQS)
    {
      /* Minus one to obatin the correct irq number */

      irq = irq - 1;
    }
  else
    {
      /* Invalid irq number */

      irq = NR_IRQS;
    }

  return irq;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/**
 * @brief     This function serves to handle the interrupt of MCU
 * @param[in] none
 * @return    none
 */

uint32_t *arm_doirq(int irq, uint32_t *regs)
{
  board_autoled_on(LED_INIRQ);
#ifdef CONFIG_SUPPRESS_INTERRUPTS
  PANIC();
#else
  /* Current regs non-zero indicates that we are processing an interrupt;
   * g_current_regs is also used to manage interrupt level context switches.
   *
   * Nested interrupts are not supported
   */

  DEBUGASSERT(CURRENT_REGS == NULL);
  CURRENT_REGS = regs;

  /* Disable further occurrences of this interrupt (until the interrupt
   * sources have been clear by the driver).
   */

  /* Deliver the IRQ */

  irq_dispatch(irq, regs);

#if defined(CONFIG_ARCH_FPU) || defined(CONFIG_ARCH_ADDRENV)
  /* Check for a context switch.  If a context switch occurred, then
   * g_current_regs will have a different value than it did on entry.  If an
   * interrupt level context switch has occurred, then restore the floating
   * point state and the establish the correct address environment before
   * returning from the interrupt.
   */

  if (regs != CURRENT_REGS)
    {
#ifdef CONFIG_ARCH_FPU
      /* Restore floating point registers */

      riscv_restorefpu(CURRENT_REGS);
#endif

#ifdef CONFIG_ARCH_ADDRENV
      /* Make sure that the address environment for the previously
       * running task is closed down gracefully (data caches dump,
       * MMU flushed) and set up the address environment for the new
       * thread at the head of the ready-to-run list.
       */

      group_addrenv(NULL);
#endif
    }
#endif

  /* If a context switch occurred while processing the interrupt then
   * g_current_regs may have change value.  If we return any value different
   * from the input regs, then the lower level will know that a context
   * switch occurred during interrupt processing.
   */

  regs = (uint32_t *)CURRENT_REGS;

  /* Set g_current_regs to NULL to indicate that we are no longer in an
   * interrupt handler.
   */

  CURRENT_REGS = NULL;

  /* Unmask the last interrupt (global interrupts are still disabled) */

#endif
  board_autoled_off(LED_INIRQ);
  return regs;
}

/****************************************************************************
 * Name: irq_handler
 *
 * Description:
 *   This function is the common interrupt handler for all interrupts.
 *
 * Parameters:
 *   uint32_t *regs - the saved context array pointer of interrpted task,
 *                    size = XCPTCONTEXT_REGS.
 *
 * Return Value:
 *   uint32_t *regs - if occur context switch, regs = the saved context
 *                    array pointer of next task, if not, regs = input regs,
 *                    size = XCPTCONTEXT_REGS.
 *
 ****************************************************************************/

uint32_t * locate_code(".ram_code") irq_handler(uint32_t *regs)
{
  int irq = tc32_getirq();

  return arm_doirq(irq, regs);
}

