/****************************************************************************
 * arch/arm/src/str71x/str71x_timerisr.c
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

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "chip.h"
#include "clock/clock.h"
#include "arm_internal.h"
#include "str71x.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The desired timer interrupt frequency is provided by the definition
 * CLK_TCK (see include/time.h).  CLK_TCK defines the desired number of
 * system clock ticks per second.  That value is a user configurable setting
 * that defaults to 100 (100 ticks per second = 10 MS interval).
 *
 * The best accuracy would be obtained by using the largest value in the
 * the output compare register (OCAR), i.e., 0xffff = 65,535:
 */

#define MAX_OCAR    65535

/* In this case, the desired, maximum clocking would be MAX_TIM0CLK.  For
 * example if CLK_TCK is the default of 100Hz, then the ideal clocking for
 * timer0 would be 6,553,500
 */

#define MAX_TIM0CLK (MAX_OCAR * CLK_TCK)

/* The best divider then would be the one that reduces PCLK2 to MAX_TIM0CLK.
 * Note that the following calculation forces an integer divisor to the next
 * integer above the optimal.  So, for example, if MAX_TIM0CLK is 6,553,500
 * and PCLK2 is 32MHz, then ideal PCLK2_DIVIDER would be 4.88 but 5 is used
 * instead.  The value 5 would give an actual TIM0CLK of 6,400,000, less
 * than the maximum.
 */

#if STR71X_PCLK2 > MAX_TIM0CLK
#  define PCLK2_DIVIDER (((STR71X_PCLK2) + (MAX_TIM0CLK+1)) / MAX_TIM0CLK)
#else
#  define PCLK2_DIVIDER (1)
#endif

#if PCLK2_DIVIDER > 255
#  error "PCLK2 is too fast for any divisor"
#endif

/* Then we can get the actual OCAR value from the selected divider value.
 * For example, if PCLK2 is 32MHz and PCLK2_DIVIDER is 5, then the actual
 * TIM0CLK would 6,4000,000 and the final OCAR_VALUE would be 64,000.
 */

#define ACTUAL_TIM0CLK (STR71X_PCLK2 / PCLK2_DIVIDER)
#define OCAR_VALUE     (ACTUAL_TIM0CLK / CLK_TCK)

#if OCAR_VALUE > 65535
#  error "PCLK2 is too fast for the configured CLK_TCK"
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  str71x_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the systems.
 *
 ****************************************************************************/

static int str71x_timerisr(int irq, uint32_t *regs, void *arg)
{
  uint16_t ocar;

  /* Clear all the output compare A interrupt status bit */

  putreg16(~STR71X_TIMERSR_OCFA, STR71X_TIMER0_SR);

  /* Set up for the next compare match.  We could either reset
   * the OCAR and CNTR to restart, or simply update the OCAR as
   * follows to that the match occurs later without resetting:
   */

  ocar = getreg16(STR71X_TIMER0_OCAR);
  ocar += OCAR_VALUE;
  putreg16(ocar, STR71X_TIMER0_OCAR);

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
  irqstate_t flags;

  /* Make sure that timer0 is disabled */

  flags = enter_critical_section();
  putreg16(0x0000, STR71X_TIMER0_CR1);
  putreg16(0x0000, STR71X_TIMER0_CR2);
  putreg16(0x0000, STR71X_TIMER0_SR);

  /* Configure TIM0 so that it is clocked by the internal APB2 frequency
   * (PCLK2) divided by the above prescaler value (1) -- versus an external
   * Clock.
   * -- Nothing to do because  STR71X_TIMERCR1_ECKEN is already cleared.
   *
   * Select a divisor to reduce the frequency of clocking.
   * This must be done so that the entire timer interval can fit in the
   * 16-bit OCAR register. (see the discussion above).
   */

  putreg16(STR71X_TIMERCR2_OCAIE | (PCLK2_DIVIDER - 1), STR71X_TIMER0_CR2);

  /* Start The TIM0 Counter and enable the output comparison A */

  putreg16(STR71X_TIMERCR1_EN | STR71X_TIMERCR1_OCAE, STR71X_TIMER0_CR1);

  /* Setup output compare A for desired interrupt frequency.  Note that
   * the OCAE and OCBE bits are cleared and the pins are available for other
   * functions.
   */

  putreg16(OCAR_VALUE, STR71X_TIMER0_OCAR);
  putreg16(0xfffc, STR71X_TIMER0_CNTR);

  /* Attach the timer interrupt vector */

  irq_attach(STR71X_IRQ_SYSTIMER, (xcpt_t)str71x_timerisr, NULL);

  /* And enable the timer interrupt */

  up_enable_irq(STR71X_IRQ_SYSTIMER);
  leave_critical_section(flags);
}
