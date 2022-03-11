/****************************************************************************
 * arch/arm/src/sama5/sam_timerisr.c
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
#include "sam_periphclks.h"
#include "hardware/sam_pit.h"

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
      nxsched_process_timer();
    }

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

  /* Enable the PIT peripheral */

  sam_pit_enableclk();

  /* Make sure that interrupts from the PIT are disabled */

  up_disable_irq(SAM_IRQ_PIT);

  /* Attach the timer interrupt vector */

  irq_attach(SAM_IRQ_PIT, (xcpt_t)sam_timerisr, NULL);

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
