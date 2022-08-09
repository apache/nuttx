/****************************************************************************
 * arch/arm/src/imx1/imx_timerisr.c
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
#include <errno.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "chip.h"
#include "clock/clock.h"
#include "arm_internal.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  imx_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the systems.
 *
 ****************************************************************************/

static int imx_timerisr(int irq, uint32_t *regs, void *arg)
{
  uint32_t tstat;
  int    ret = -EIO;

  /* Get and clear the interrupt status */

  tstat = getreg32(IMX_TIMER1_TSTAT);
  putreg32(0, IMX_TIMER1_TSTAT);

  /* Verify that this is a timer interrupt */

  if ((tstat & TIMER_TSTAT_COMP) != 0)
    {
      /* Process timer interrupt */

      nxsched_process_timer();
      ret = OK;
    }

  return ret;
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
  uint32_t tctl;

  /* Make sure the timer interrupts are disabled */

  up_disable_irq(IMX_IRQ_SYSTIMER);

  /* Make sure that timer1 is disabled */

  putreg32(0, IMX_TIMER1_TCTL);
  putreg32(0, IMX_TIMER1_TPRER);

  /* Select restart mode with source = PERCLK1. In restart mode, after
   * the compare value is reached, the counter resets to 0x00000000, the
   * compare event (COMP) bit of the timer status register is set, an
   * interrupt is issued if the interrupt request enable (IRQEN) bit of
   * the corresponding TCTL register is set, and the counter resumes
   * counting.
   */

  tctl = TCTL_CLKSOURCE_PERCLK1;
  putreg32(tctl, IMX_TIMER1_TCTL);

  /* The timer is driven by PERCLK1.  Set prescaler for division by one
   * so that the clock is driven at PERCLK1.
   *
   * putreg(0, IMX_TIMER1_TPRER); -- already the case
   *
   * Set the compare register so that the COMP interrupt is generated
   * with a period of USEC_PER_TICK.  The value IMX_PERCLK1_FREQ/1000
   * (defined in board.h) is the number of counts in millisecond, so:
   */

  putreg32(MSEC2TICK(IMX_PERCLK1_FREQ / 1000), IMX_TIMER1_TCMP);

  /* Configure to provide timer COMP interrupts when TCN increments
   * to TCMP.
   */

  tctl |= TIMER_TCTL_IRQEN;
  putreg32(tctl, IMX_TIMER1_TCTL);

  /* Finally, enable the timer (be be the last operation on TCTL) */

  tctl |= TIMER_TCTL_TEN;
  putreg32(tctl, IMX_TIMER1_TCTL);

  /* Attach and enable the timer interrupt */

  irq_attach(IMX_IRQ_SYSTIMER, (xcpt_t)imx_timerisr, NULL);
  up_enable_irq(IMX_IRQ_SYSTIMER);
}
