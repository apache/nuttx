/****************************************************************************
 * arch/arm/src/common/stm32/stm32_timerisr.c
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
#include <time.h>

#include <nuttx/debug.h>
#include <nuttx/arch.h>
#include <nuttx/timers/arch_timer.h>
#include <arch/board/board.h>

#include "nvic.h"
#include "clock/clock.h"
#include "arm_internal.h"
#include "systick.h"

#include "chip.h"
#include "stm32_systick.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The desired timer interrupt frequency is provided by the definition
 * CLK_TCK (see include/time.h).  CLK_TCK defines the desired number of
 * system clock ticks per second.  That value is a user configurable setting
 * that defaults to 100 (100 ticks per second = 10 MS interval).
 */

#define SYSTICK_RELOAD ((STM32_SYSTICK_CLOCK / CLK_TCK) - 1)

/* The size of the reload field is 24 bits.  Verify that the reload value
 * will fit in the reload register.
 */

#define SYSTICK_MAX 0x00ffffff
#if SYSTICK_RELOAD > SYSTICK_MAX
#  error SYSTICK_RELOAD exceeds the range of the RELOAD register
#endif

#if (defined(CONFIG_ARMV7M_SYSTICK) || defined(CONFIG_ARMV8M_SYSTICK)) && \
    defined(CONFIG_TIMER_ARCH)
#  define HAVE_SYSTICK_LOWERHALF 1
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  stm32_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the systems.
 *
 ****************************************************************************/

#ifndef HAVE_SYSTICK_LOWERHALF
static int stm32_timerisr(int irq, uint32_t *regs, void *arg)
{
  /* Process timer interrupt */

  nxsched_process_timer();
  return 0;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  up_adjtime
 *
 * Description:
 *   Adjusts timer period. This call is used when adjusting timer period as
 *   defined in adjtime() function.
 *
 * Input Parameters:
 *   ppb - Adjustment in parts per billion (nanoseconds per second).
 *         Zero is default rate, positive value makes clock run faster
 *         and negative value slower.
 *
 * Assumptions:
 *   Called from within critical section or interrupt context.
 ****************************************************************************/

#ifdef CONFIG_CLOCK_ADJTIME
void up_adjtime(long ppb)
{
  uint32_t period = SYSTICK_RELOAD;

  if (ppb != 0)
    {
      period -= (long long)ppb * SYSTICK_RELOAD / 1000000000;
    }

  /* Check whether period is at maximum value. */

  if (period > SYSTICK_MAX)
    {
      period = SYSTICK_MAX;
    }

  putreg32(period, NVIC_SYSTICK_RELOAD);
}
#endif

/****************************************************************************
 * Function:  up_timer_initialize
 *
 * Description:
 *   This function is called during start-up to initialize
 *   the timer interrupt.
 *
 ****************************************************************************/

void up_timer_initialize(void)
{
  uint32_t regval;

  /* Set the SysTick interrupt to the default priority */

  regval = getreg32(NVIC_SYSH12_15_PRIORITY);
  regval &= ~NVIC_SYSH_PRIORITY_PR15_MASK;
  regval |= (NVIC_SYSH_PRIORITY_DEFAULT << NVIC_SYSH_PRIORITY_PR15_SHIFT);
  putreg32(regval, NVIC_SYSH12_15_PRIORITY);

#ifdef HAVE_SYSTICK_LOWERHALF
  up_timer_set_lowerhalf(systick_initialize(STM32_SYSTICK_CORECLK,
                                            STM32_SYSTICK_CLOCK, -1));
#else
  /* Configure SysTick to interrupt at the requested rate */

  putreg32(SYSTICK_RELOAD, NVIC_SYSTICK_RELOAD);
  putreg32(0, NVIC_SYSTICK_CURRENT);

  /* Attach the timer interrupt vector */

  irq_attach(STM32_IRQ_SYSTICK, (xcpt_t)stm32_timerisr, NULL);

  /* Enable SysTick interrupts */

  regval = NVIC_SYSTICK_CTRL_TICKINT | NVIC_SYSTICK_CTRL_ENABLE;
  if (STM32_SYSTICK_CORECLK)
    {
      regval |= NVIC_SYSTICK_CTRL_CLKSOURCE;
    }

  putreg32(regval, NVIC_SYSTICK_CTRL);

  /* And enable the timer interrupt */

  up_enable_irq(STM32_IRQ_SYSTICK);
#endif
}
