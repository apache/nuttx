/****************************************************************************
 * arch/arm/src/lpc54xx/lpc54_timerisr.c
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
#include "hardware/lpc54_syscon.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The SysTick clock may be clocked internally either by the by the system
 * clock (CLKSOURCE==1) or by the SysTick function clock (CLKSOURCE==0).
 * The SysTick Function clock is equal to:
 *
 *   Fsystick = Fmainclk / SYSTICKCLKDIV
 *
 * Both the divider value (BOARD_SYSTICKCLKDIV) and the resulting SysTick
 * function clock frequency (Fsystick, BOARD_SYSTICK_CLOCK)
 *
 * The desired timer interrupt frequency is provided by the definition
 * CLK_TCK (see include/time.h).  CLK_TCK defines the desired number of
 * system clock ticks per second.  That value is a user configurable setting
 * that defaults to 100 (100 ticks per second = 10 MS interval).
 *
 *    reload = (Fsystick / CLK_TICK) - 1
 *
 * Tips for selecting BOARD_SYSTICKCLKDIV:  The resulting reload value
 * should be as large as possible, but must be less than 2^24:
 *
 *   SYSTICKDIV > Fmainclk / CLK_TCK / 2^24
 */

#define SYSTICK_RELOAD ((BOARD_SYSTICK_CLOCK / CLK_TCK) - 1)

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
 * Function:  lpc54_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the systems.
 *
 ****************************************************************************/

static int lpc54_timerisr(int irq, uint32_t *regs, void *arg)
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

  /* May be clocked internally by the system clock or the SysTick function
   * clock.  Set the SysTick clock divider in the SYSCON_SYSTICK register.
   * Since this function is called early after reset, it is safe to assume
   * that the SysTick is disabled and so that no reset or halt actions are
   * necessary.
   */

  regval = (SYSCON_SYSTICKCLKDIV_DIV(BOARD_SYSTICKCLKDIV) |
            SYSCON_SYSTICKCLKDIV_REQFLAG);
  putreg32(regval, LPC54_SYSCON_SYSTICKCLKDIV);

  /* The request flag will be cleared when the divider change is complete */

  while ((getreg32(LPC54_SYSCON_SYSTICKCLKDIV) &
          SYSCON_SYSTICKCLKDIV_REQFLAG) != 0)
    {
    }

  /* Make sure that the SYSTICK clock source is set to use the SysTick
   * function clock (CLKSOURCE==0).
   *
   * REVISIT: This is over-writted with CLKSOURCE==1 below.
   */

  regval  = getreg32(NVIC_SYSTICK_CTRL);
  regval &= ~NVIC_SYSTICK_CTRL_CLKSOURCE;
  putreg32(regval, NVIC_SYSTICK_CTRL);

  /* Configure SysTick to interrupt at the requested rate */

  putreg32(SYSTICK_RELOAD, NVIC_SYSTICK_RELOAD);

  /* Attach the timer interrupt vector */

  irq_attach(LPC54_IRQ_SYSTICK, (xcpt_t)lpc54_timerisr, NULL);

  /* Enable SysTick interrupts */

  putreg32((NVIC_SYSTICK_CTRL_CLKSOURCE | NVIC_SYSTICK_CTRL_TICKINT |
            NVIC_SYSTICK_CTRL_ENABLE), NVIC_SYSTICK_CTRL);

  /* And enable the timer interrupt */

  up_enable_irq(LPC54_IRQ_SYSTICK);
}
