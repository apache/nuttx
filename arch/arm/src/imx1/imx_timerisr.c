/****************************************************************************
 * arch/arm/src/imx1/imx_timerisr.c
 * arch/arm/src/chip/imx_timerisr.c
 *
 *   Copyright (C) 2009, 2017 Gregory Nutt. All rights reserved.
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
#include <debug.h>
#include <errno.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_arch.h"
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

static int imx_timerisr(int irq, uint32_t *regs, FAR void *arg)
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
