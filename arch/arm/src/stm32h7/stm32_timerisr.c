/****************************************************************************
 * arch/arm/src/stm32h7/stm32_timerisr.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
#include "arm_arch.h"

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
 * The SysTick calibration value is fixed to 18750, which gives a reference
 * time base of 1 ms with the SysTick clock set to 18.75 MHz (HCLK/8, with
 * HCLK set to 150 MHz).
 *
 * REVISIT:  Per ES0392 Rev 4: STM32H743xI Errata sheet STM32H743xI device
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

#undef CONFIG_STM32H7_SYSTICK_HCLKd8

/* REVISIT:
 *   It looks like SYSTICK for H7 is always clocked from CPUCLK and doesn't
 *   depend on the SYSTICK_CTRL_CLKSOURCE bit settings.
 */

#ifdef CONFIG_STM32H7_SYSTICK_HCLKd8
#  define STM32_SYSTICK_CLOCK  (STM32_HCLK_FREQUENCY / 8)
#else
#  define STM32_SYSTICK_CLOCK  (STM32_CPUCLK_FREQUENCY)
#endif

/* The desired timer interrupt frequency is provided by the definition
 * CLK_TCK (see include/time.h).  CLK_TCK defines the desired number of
 * system clock ticks per second.  That value is a user configurable setting
 * that defaults to 100 (100 ticks per second = 10 MS interval).
 *
 * For example, suppose HCLK = 216 MHz and CLK_TCK = 100, then:
 *
 *   STM32_SYSTICK_CLOCK = 216 MHz / 8 = 27 MHz
 *   SYSTICK_RELOAD      = (27,000,000 / 100) - 1 = 269,999
 */

#define SYSTICK_RELOAD ((STM32_SYSTICK_CLOCK / CLK_TCK) - 1)

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
 * Function:  stm32_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the systems.
 *
 ****************************************************************************/

static int stm32_timerisr(int irq, uint32_t *regs, void *arg)
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

  irq_attach(STM32_IRQ_SYSTICK, (xcpt_t)stm32_timerisr, NULL);

  /* Enable SysTick interrupts:
   *
   *   NVIC_SYSTICK_CTRL_CLKSOURCE   : Configurable, 0=HCLK/8, 1=CPU
   *   NVIC_SYSTICK_CTRL_TICKINT=1   : Generate interrupts
   *   NVIC_SYSTICK_CTRL_ENABLE      : Enable the counter
   */

  regval  = (NVIC_SYSTICK_CTRL_TICKINT | NVIC_SYSTICK_CTRL_ENABLE);
#ifndef CONFIG_STM32H7_SYSTICK_HCLKd8
  regval |= NVIC_SYSTICK_CTRL_CLKSOURCE;
#else
  regval &= ~NVIC_SYSTICK_CTRL_CLKSOURCE;
#endif
  putreg32(regval, NVIC_SYSTICK_CTRL);

  /* And enable the timer interrupt */

  up_enable_irq(STM32_IRQ_SYSTICK);
}
