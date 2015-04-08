/****************************************************************************
 * arch/arm/src/a1x/a1x_timerisr.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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
#include <assert.h>

#include <nuttx/arch.h>

#include <arch/irq.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "chip/a1x_timer.h"

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
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Global Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  up_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the systems.
 *
 ****************************************************************************/

int up_timerisr(int irq, uint32_t *regs)
{
  /* Only a TIMER0 interrupt is expected here */

  DEBUGASSERT((getreg32(A1X_TMR_IRQ_STA) & TMR_IRQ_TMR0) != 0);

  /* Clear the pending interrupt by writing a '1' to the status register */

  putreg32(TMR_IRQ_TMR0, A1X_TMR_IRQ_STA);

  /* Process timer interrupt */

  sched_process_timer();
  return OK;
}

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

  (void)irq_attach(A1X_IRQ_TIMER0, (xcpt_t)up_timerisr);

  /* Enable interrupts from the TIMER 0 port */

  regval = getreg32(A1X_TMR_IRQ_EN);
  regval |= TMR_IRQ_TMR0;
  putreg32(regval, A1X_TMR_IRQ_EN);

  /* And enable the timer interrupt */

  up_enable_irq(A1X_IRQ_TIMER0);
}
