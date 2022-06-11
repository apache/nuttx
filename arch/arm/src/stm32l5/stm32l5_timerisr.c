/****************************************************************************
 * arch/arm/src/stm32l5/stm32l5_timerisr.c
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
#include <debug.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "nvic.h"
#include "clock/clock.h"
#include "arm_internal.h"
#include "chip.h"
#include "stm32l5.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The desired timer interrupt frequency is provided by the definition
 * CLK_TCK (see include/time.h).  CLK_TCK defines the desired number of
 * system clock ticks per second.  That value is a user configurable setting
 * that defaults to 100 (100 ticks per second = 10 MS interval).
 *
 * The RCC feeds the Cortex System Timer (SysTick) with the AHB clock (HCLK)
 * divided by 8.  The SysTick can work either with this clock or with the
 * Cortex clock (HCLK), configurable in the SysTick Control and Status
 * register.
 */

/* Power up default is HCLK, not HCLK/8.
 * And I don't know now to re-configure it yet
 */

#undef CONFIG_STM32L5_SYSTICK_HCLKd8

#ifdef CONFIG_STM32L5_SYSTICK_HCLKd8
#  define SYSTICK_RELOAD ((STM32L5_HCLK_FREQUENCY / 8 / CLK_TCK) - 1)
#else
#  define SYSTICK_RELOAD ((STM32L5_HCLK_FREQUENCY / CLK_TCK) - 1)
#endif

/* The size of the reload field is 24 bits.  Verify that the reload value
 * will fit in the reload register.
 */

#if SYSTICK_RELOAD > 0x00ffffff
#  error SYSTICK_RELOAD exceeds the range of the RELOAD register
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  stm32l5_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the systems.
 *
 ****************************************************************************/

static int stm32l5_timerisr(int irq, uint32_t *regs, void *arg)
{
  /* Process timer interrupt */

  nxsched_process_timer();
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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

  /* Make sure that the SYSTICK clock source is set correctly */

#if 0 /* Does not work.  Comes up with HCLK source and I can't change it */
  regval = getreg32(NVIC_SYSTICK_CTRL);
#ifdef CONFIG_STM32L5_SYSTICK_HCLKd8
  regval &= ~NVIC_SYSTICK_CTRL_CLKSOURCE;
#else
  regval |= NVIC_SYSTICK_CTRL_CLKSOURCE;
#endif
  putreg32(regval, NVIC_SYSTICK_CTRL);
#endif

  /* Configure SysTick to interrupt at the requested rate */

  putreg32(SYSTICK_RELOAD, NVIC_SYSTICK_RELOAD);

  /* Attach the timer interrupt vector */

  irq_attach(STM32L5_IRQ_SYSTICK, (xcpt_t)stm32l5_timerisr, NULL);

  /* Enable SysTick interrupts */

  putreg32((NVIC_SYSTICK_CTRL_CLKSOURCE | NVIC_SYSTICK_CTRL_TICKINT |
            NVIC_SYSTICK_CTRL_ENABLE), NVIC_SYSTICK_CTRL);

  /* And enable the timer interrupt */

  up_enable_irq(STM32L5_IRQ_SYSTICK);
}
