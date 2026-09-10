/****************************************************************************
 * arch/arm/src/n32h7/n32_timerisr.c
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
#include <debug.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "nvic.h"
#include "clock/clock.h"
#include "arm_internal.h"
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Select MCU-specific settings
 *
 * The RCC feeds the external clock of the Cortex System Timer (SysTick) with
 * the AHB clock (HCLK) divided by 8. The SysTick can work either with this
 * clock or with the Cortex clock (HCLK), configurable in the SysTick control
 * and status register.
 *
 * The SysTick calibration value is fixed to 75000, which gives a reference
 * time base of 1 ms with the SysTick clock set to 75 MHz (HCLK/8, with
 * HCLK set to 600 MHz).
 *
 * REVISIT:  Per ES0392 Rev 4: N32H743xI Errata sheet N32H743xI device
 * limitations
 *
 *   SysTick external clock is not HCLK/8
 *   Description
 *     The SysTick external clock is the system clock, instead of the system
 *     clock divided by 8 (HCLK/8).
 *   Workaround
 *     Use the system clock (HCLK) as external clock and multiply the reload
 *     value by 8 in STK_LOAD register (take care that the maximum value is
 *     224-1).
 */

#undef CONFIG_N32H7_SYSTICK_HCLKd8

/* REVISIT:
 *   It looks like SYSTICK for H7 is always clocked from CPUCLK and doesn't
 *   depend on the SYSTICK_CTRL_CLKSOURCE bit settings.
 */

#ifdef CONFIG_N32H7_SYSTICK_HCLKd8
#  define N32_SYSTICK_CLOCK  (N32_M7CPU_FREQUENCY / 8)
#else
#  define N32_SYSTICK_CLOCK  (N32_M7CPU_FREQUENCY)
#endif

/* The desired timer interrupt frequency is provided by the definition
 * CLK_TCK (see include/time.h).  CLK_TCK defines the desired number of
 * system clock ticks per second.  That value is a user configurable setting
 * that defaults to 1000 (1000 ticks per second = 1 MS interval).
 *
 * For example, suppose HCLK = 600 MHz and CLK_TCK = 1000, then:
 *
 *   N32_SYSTICK_CLOCK = 600 MHz
 *   SYSTICK_RELOAD      = (600,000,000 / 1000) - 1 = 599,999
 */

#define SYSTICK_RELOAD ((N32_SYSTICK_CLOCK / CLK_TCK) - 1)

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
 * Function:  n32_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the systems.
 *
 ****************************************************************************/

static int n32_timerisr(int irq, uint32_t *regs, void *arg)
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
 *   This function is called during start-up to initialize the timer
 *   interrupt.
 *
 ****************************************************************************/

void up_timer_initialize(void)
{
  uint32_t regval;

  /* Configure SysTick to interrupt at the requested rate */

  putreg32(SYSTICK_RELOAD, NVIC_SYSTICK_RELOAD);
  putreg32(0, NVIC_SYSTICK_CURRENT);

  /* Attach the timer interrupt vector */

  irq_attach(N32_IRQ_SYSTICK, (xcpt_t)n32_timerisr, NULL);

  /* Enable SysTick interrupts:
   *
   *   NVIC_SYSTICK_CTRL_CLKSOURCE   : Configurable, 0=HCLK/8, 1=CPU
   *   NVIC_SYSTICK_CTRL_TICKINT=1   : Generate interrupts
   *   NVIC_SYSTICK_CTRL_ENABLE      : Enable the counter
   */

  regval  = (NVIC_SYSTICK_CTRL_TICKINT | NVIC_SYSTICK_CTRL_ENABLE);
#ifndef CONFIG_N32H7_SYSTICK_HCLKd8
  regval |= NVIC_SYSTICK_CTRL_CLKSOURCE;
#else
  regval &= ~NVIC_SYSTICK_CTRL_CLKSOURCE;
#endif
  putreg32(regval, NVIC_SYSTICK_CTRL);

  /* And enable the timer interrupt */

  up_enable_irq(N32_IRQ_SYSTICK);
}
