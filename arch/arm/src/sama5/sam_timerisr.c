/****************************************************************************
 * arch/arm/src/sama5/sam_timerisr.c
 *
 *   Copyright (C) 2013, 2017 Gregory Nutt. All rights reserved.
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

#include <nuttx/arch.h>

#include <arch/irq.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "sam_periphclks.h"
#include "chip/sam_pit.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* The PIT counter runs at a rate of the main clock (MCK) divided by 16.
 *
 * On the SAMA5D4, the clocking to the PIC may be divided down from MCK.
 * Perhaps because of H32MXDIV?  We will let the board.h tell us the correct
 * PIT include clock by defining BOARD_PIT_FREQUENCY.
 */

#define PIT_CLOCK (BOARD_PIT_FREQUENCY >> 4)

/* The desired timer interrupt frequency is provided by the definition
 * CLK_TCK (see include/time.h).  CLK_TCK defines the desired number of
 * system clock ticks per second.  That value is a user configurable setting
 * that defaults to 100 (100 ticks per second = 10 MS interval).
 *
 * The PIT counts from zero and up until it reaches the overflow value set
 * in the field PIV of the Mode Register (PIT MR).  So an PIV value of n
 * corresponds a duration of n * PIT_CLOCK
 */

#define PIT_PIV ((PIT_CLOCK + (CLK_TCK >> 1)) / CLK_TCK)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  sam_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the systems.
 *
 ****************************************************************************/

static int sam_timerisr(int irq, uint32_t *regs, void *arg)
{
  /* "When CPIV and PICNT values are obtained by reading the Periodic
   *  Interval Value Register (PIT_PIVR), the overflow counter (PICNT) is
   *  reset and the PITS is cleared, thus acknowledging the interrupt. The
   *  value of PICNT gives the number of periodic intervals elapsed since the
   *  last read of PIT_PIVR."
   */

  uint32_t picnt = getreg32(SAM_PIT_PIVR) >> PIT_PICNT_SHIFT;

  /* Process timer interrupt (multiple times if we missed an interrupt) */

  while (picnt-- > 0)
    {
      sched_process_timer();
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  arm_timer_initialize
 *
 * Description:
 *   This function is called during start-up to initialize
 *   the timer interrupt.
 *
 ****************************************************************************/

void arm_timer_initialize(void)
{
  uint32_t regval;

  /* Enable the PIT peripheral */

  sam_pit_enableclk();

  /* Make sure that interrupts from the PIT are disabled */

  up_disable_irq(SAM_IRQ_PIT);

  /* Attach the timer interrupt vector */

  (void)irq_attach(SAM_IRQ_PIT, (xcpt_t)sam_timerisr, NULL);

  /* Set the PIT overflow value (PIV), enable the PIT, and enable
   * interrupts from the PIT.
   */

  regval  = PIT_PIV;
  DEBUGASSERT(regval <= PIT_MR_PIV_MASK);

  regval |= (PIT_MR_PITEN | PIT_MR_PITIEN);
  putreg32(regval, SAM_PIT_MR);

  /* And enable the timer interrupt */

  up_enable_irq(SAM_IRQ_PIT);
}
