/***************************************************************************
 * arch/z80/src/z8/z8_timerisr.c
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 ***************************************************************************/

/***************************************************************************
 * Included Files
 ***************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <ez8.h>

#include "chip/chip.h"
#include "clock_internal.h"
#include "up_internal.h"

/***************************************************************************
 * Definitions
 ***************************************************************************/

/* System clock frequency value from ZDS target settings */

extern ROM uint32 __user_frequency;
#define _DEFCLK ((uint32)&__user_frequency)

/***************************************************************************
 * Private Types
 ***************************************************************************/

/***************************************************************************
 * Private Functions
 ***************************************************************************/

/***************************************************************************
 * Public Functions
 ***************************************************************************/

/***************************************************************************
 * Function:  up_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the system.
 *
 ***************************************************************************/

int up_timerisr(int irq, uint32 *regs)
{
   /* Process timer interrupt */

   sched_process_timer();
   return 0;
}

/***************************************************************************
 * Function:  up_timerinit
 *
 * Description:
 *   This function is called during start-up to initialize the timer
 *   interrupt.
 *
 ***************************************************************************/

void up_timerinit(void)
{
  up_disable_irq(Z8_IRQ_SYSTIMER);

  /* Write to the timer control register to disable the timer, configure
   * the timer for continuous mode, and set up the pre-scale value for
   * divide by 4.
   */

  putreg8( Z8_TIMERSCTL_DIV4 | Z8_TIMERSCTL_CONT, Z8_TIMER0_CTL);

  /* Write to the timer high and low byte registers to set a starting
   * count value (this effects only the first pass in continuous mode)
   */

  putreg16(0x0001, Z8_TIMER0_HL);

  /* Write to the timer reload register to set the reload value.
   *
   * In continuous mode:
   *
   *   timer_period = reload_value x prescale / system_clock_frequency
   * or
   *   reload_value = (timer_period * system_clock_frequency) / prescale
   *
   * For system_clock_frequency=18.432MHz, timer_period=10mS, and prescale=4,
   * then reload_value=46,080 - OR:
   *
   *   reload_value = system_clock_frequency / 400
   */

  putreg16(((uint32)_DEFCLK / 400), Z8_TIMER0_R);

  /* Write to the timer control register to enable the timer and to
   * initiate counting
   */

  putreg8((getreg8(Z8_TIMER0_CTL) | Z8_TIMERCTL_TEN), Z8_TIMER0_CTL);

  /* Set the timer priority */

  /* Attach and enable the timer interrupt (leaving at priority 0 */

  irq_attach(Z8_IRQ_SYSTIMER, (xcpt_t)up_timerisr);
  up_enable_irq(Z8_IRQ_SYSTIMER);
}

