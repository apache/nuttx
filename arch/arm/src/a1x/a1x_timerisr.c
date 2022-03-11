/****************************************************************************
 * arch/arm/src/a1x/a1x_timerisr.c
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
#include <assert.h>

#include <nuttx/arch.h>

#include <arch/irq.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "hardware/a1x_timer.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Timer 0 will run at the rate of OSC24M with no division */

#define TMR0_CLOCK (24000000)

/* The desired timer interrupt frequency is provided by the definition
 * CLK_TCK (see include/time.h).  CLK_TCK defines the desired number of
 * system clock ticks per second.  That value is a user configurable setting
 * that defaults to 100 (100 ticks per second = 10 MS interval).
 *
 * Timer 0 counts down from the interval reload value to zero, generating
 * an interrupt (and reload) when the counts decrements to zero.
 */

#define TMR_INTERVAL ((TMR0_CLOCK + (CLK_TCK >> 1)) / CLK_TCK)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  a1x_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the systems.
 *
 ****************************************************************************/

static int a1x_timerisr(int irq, uint32_t *regs, void *arg)
{
  /* Only a TIMER0 interrupt is expected here */

  DEBUGASSERT((getreg32(A1X_TMR_IRQ_STA) & TMR_IRQ_TMR0) != 0);

  /* Clear the pending interrupt by writing a '1' to the status register */

  putreg32(TMR_IRQ_TMR0, A1X_TMR_IRQ_STA);

  /* Process timer interrupt */

  nxsched_process_timer();
  return OK;
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

  /* Set the timer reload interval value */

  putreg32(TMR_INTERVAL, A1X_TMR0_INTV_VALUE);

  /* Configure timer 0:
   *
   *   ENABLE    - Enable counting
   *   RELOAD    - Reload timer interval value
   *   CLKSRC    - OSC24M
   *   PRESCALER - No division
   *   MODE      - Continuous mode
   */

  regval = (TMR_CTRL_EN | TMR_CTRL_RELOAD | TMR_CTRL_SRC_OSC24M |
            TMR_CTRL_CLK_PRES_DIV1 | TMR_CTRL_MODE_CONTINUOUS);
  putreg32(regval, A1X_TMR0_CTRL);

  /* Make sure that interrupts from the Timer 0 are disabled */

  up_disable_irq(A1X_IRQ_TIMER0);

  /* Attach the timer interrupt vector */

  irq_attach(A1X_IRQ_TIMER0, (xcpt_t)a1x_timerisr, NULL);

  /* Enable interrupts from the TIMER 0 port */

  regval = getreg32(A1X_TMR_IRQ_EN);
  regval |= TMR_IRQ_TMR0;
  putreg32(regval, A1X_TMR_IRQ_EN);

  /* And enable the timer interrupt */

  up_enable_irq(A1X_IRQ_TIMER0);
}
