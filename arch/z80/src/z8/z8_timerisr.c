/****************************************************************************
 * arch/z80/src/z8/z8_timerisr.c
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
#include <debug.h>
#include <ez8.h>

#include <nuttx/arch.h>

#include "chip.h"
#include "clock/clock.h"
#include "z80_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* This function is normally prototyped int the ZiLOG header file sio.h */

extern uint32_t get_freq(void);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  z8_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the system.
 *
 ****************************************************************************/

static int z8_timerisr(int irq, uint32_t *regs, void *arg)
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
  uint32_t reload;

  up_disable_irq(Z8_IRQ_SYSTIMER);

  /* Write to the timer control register to disable the timer, configure
   * the timer for continuous mode, and set up the pre-scale value for
   * divide by 4.
   */

  putreg8((Z8_TIMERCTL_DIV4 | Z8_TIMERCTL_CONT), T0CTL);

  /* Write to the timer high and low byte registers to set a starting
   * count value (this effects only the first pass in continuous mode)
   */

  putreg16(0x0001, T0);

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

  reload = get_freq() / 400;
  putreg16((uint16_t)reload, T0R);

  /* Write to the timer control register to enable the timer and to
   * initiate counting
   */

  putreg8((getreg8(T0CTL) | Z8_TIMERCTL_TEN), T0CTL);

  /* Set the timer priority */

  /* Attach and enable the timer interrupt (leaving at priority 0 */

  irq_attach(Z8_IRQ_SYSTIMER, (xcpt_t)z8_timerisr, NULL);
  up_enable_irq(Z8_IRQ_SYSTIMER);
}
