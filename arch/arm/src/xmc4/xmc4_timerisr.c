/****************************************************************************
 * arch/arm/src/xmc4/xmc4_timerisr.c
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

/* The SysTick counter runs on the clock selected by SYST_CSR.CLKSOURCE.
 * That selection may be either:
 *
 *    CLKSOURCE=0: fSTDBY / 2
 *    CLKSOURCE=1: fCPU
 *
 * In the first case, the SysTick counter would run at 16.384Khz.  The most
 * common system clock of 10 msec/tick cannot be exactly represented with
 * that value.
 *
 * In the second case, the SysTick counter may run too rapidly to support
 * longer timer tick intervals.  For example, if the CPU clock is 144Mhz,
 * then that 10 msec interval would correspond to a reload value of 1,440,000
 * or 0x0015f900.
 */

/* The desired timer interrupt frequency is provided by the definition
 * CLK_TCK (see include/time.h).  CLK_TCK defines the desired number of
 * system clock ticks per second.  That value is a user configurable setting
 * that defaults to 100 (100 ticks per second = 10 MS interval).
 *
 * Lets try fCPU first:
 */

#define SYSTICK_RELOAD ((BOARD_CPU_FREQUENCY / CLK_TCK) - 1)
#undef  USE_STDBY_CLOCK

/* Verify that the reload value will fit in the reload register. */

#if SYSTICK_RELOAD > 0x00ffffff
  /* No, then revert to fSTDBY */

#  undef SYSTICK_RELOAD
#  define SYSTICK_RELOAD ((BOARD_STDBY_FREQUENCY / CLK_TCK) - 1)
#  define USE_STDBY_CLOCK 1
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  xmc4_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the systems.
 *
 ****************************************************************************/

static int xmc4_timerisr(int irq, uint32_t *regs, void *arg)
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

  regval  = getreg32(NVIC_SYSH12_15_PRIORITY);
  regval &= ~NVIC_SYSH_PRIORITY_PR15_MASK;
  regval |= (NVIC_SYSH_PRIORITY_DEFAULT << NVIC_SYSH_PRIORITY_PR15_SHIFT);
  putreg32(regval, NVIC_SYSH12_15_PRIORITY);

#ifndef USE_STDBY_CLOCK
  /* Note that it should not be necessary to set the SYSTICK clock source:
   * "The CLKSOURCE bit in SysTick Control and Status register is always set
   *  to select the core clock."
   *
   * For the XMC4xx, fhat selection may be either:
   *
   *   CLKSOURCE=0: fSTDBY / 2
   *   CLKSOURCE=1: fCPU
   */

  regval  = getreg32(NVIC_SYSTICK_CTRL);
  regval |= NVIC_SYSTICK_CTRL_CLKSOURCE;
  putreg32(regval, NVIC_SYSTICK_CTRL);
#endif

  /* Configure SysTick to interrupt at the requested rate */

  putreg32(SYSTICK_RELOAD, NVIC_SYSTICK_RELOAD);

  /* Attach the timer interrupt vector */

  irq_attach(XMC4_IRQ_SYSTICK, (xcpt_t)xmc4_timerisr, NULL);

  /* Enable SysTick interrupts */

  putreg32((NVIC_SYSTICK_CTRL_CLKSOURCE | NVIC_SYSTICK_CTRL_TICKINT |
            NVIC_SYSTICK_CTRL_ENABLE),
           NVIC_SYSTICK_CTRL);

  /* And enable the timer interrupt */

  up_enable_irq(XMC4_IRQ_SYSTICK);
}
