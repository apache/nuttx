/****************************************************************************
 * arch/arm/src/dm320/dm320_timerisr.c
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
#include <nuttx/arch.h>

#include "chip.h"
#include "clock/clock.h"
#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DM320 Timers
 *
 * Each of the general-purpose timers can run in one of two modes:  one-
 * shot mode and free-run mode.  In one-shot mode, an interrupt only
 * occurs once and then the timer must be explicitly reset to begin the
 * timing operation again.  In free-run mode, when the timer generates an
 * interrupt, the timer counter is automatically reloaded to start the count
 * operation again.  Use the bit field MODE in TMMDx to configure the
 * timer for one-shot more or free-run mode. The bit field MODE in TMMDx
 * also allows you to stop the timer.
 *
 * Either the ARM clock divided by 2 (CLK_ARM/2) or an external clock
 * connected to the M27XI pin can be selected as the clock source of the
 * timer.
 *
 * The actual clock frequency used in the timer count operation is the input
 * clock divided by: 1 plus the value set in the bit field PRSCL of the
 * register TMPRSCLx (10 bits).  The timer expires when it reaches the
 * value set in the bit field DIV of the register TMDIVx (16 bits) plus 1.
 * PRSCL+1 is the source clock frequency divide factor and DIV+1 is the
 * timer count value.  The frequency of a timer interrupt is given by the
 * following equation:
 *
 * Interrupt Frequency = (Source Clock Frequency) / (PRSCL+1) / (DIV+1)
 */

/* System Timer
 *
 * Timer0 is dedicated as the system timer.  The rate of system timer
 * interrupts is assumed to be 10MS per tick / 100Hz. The following
 * register settings are used for timer 0
 *
 * System clock formula:
 *   Interrupt Frequency = (Source Clock Frequency) / (PRSCL+1) / (DIV+1)
 *   Source Clock Frequency = 27MHz  (PLL clock)
 *   DIV                    = 26,999 (Yields 1Khz timer clock)
 *   PRSCL                  = 9      (Produces 100Hz interrupts)
 */

#define DM320_TMR0_MODE  DM320_TMR_MODE_FREERUN /* Free running */
#define DM320_TMR0_DIV   26999                  /* (see above) */
#define DM320_TMR0_PRSCL 9                      /* (see above) */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  dm320_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the systems.
 *
 ****************************************************************************/

static int dm320_timerisr(int irq, uint32_t *regs, void *arg)
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
  up_disable_irq(DM320_IRQ_SYSTIMER);

  /* Start timer0 running so that an interrupt is generated at
   * the rate USEC_PER_TICK.
   */

  putreg16(DM320_TMR0_PRSCL, DM320_TIMER0_TMPRSCL); /* Timer 0 Prescalar */
  putreg16(DM320_TMR0_DIV, DM320_TIMER0_TMDIV);     /* Timer 0 Divisor (count) */

  /* Start the timer */

  putreg16(DM320_TMR0_MODE, DM320_TIMER0_TMMD); /* Timer 0 Mode */

  /* Attach and enable the timer interrupt */

  irq_attach(DM320_IRQ_SYSTIMER, (xcpt_t)dm320_timerisr, NULL);
  up_enable_irq(DM320_IRQ_SYSTIMER);
}
