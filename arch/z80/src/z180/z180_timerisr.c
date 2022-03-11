/****************************************************************************
 * arch/z80/src/z180/z180_timerisr.c
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

#include <arch/board/board.h>

#include "clock/clock.h"
#include "z80_internal.h"
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* "The Z180 contains a two channel 16-bit Programmable Reload Timer. Each
 * PRT channel contains a 16-bit down counter and a 16-bit reload register."
 * Channel 0 is dedicated as the system timer.
 */

/* "The PRT input clock for both channels is equal to the system clock
 * divided by 20."
 */

#define Z180_PRT_CLOCK   (Z180_SYSCLOCK / 20)

/* The data Register "(TMDR) is decremented once every twenty clocks. When
 * TMDR counts down to 0, it is automatically reloaded with the value
 * contained in the Reload Register (RLDR)."
 */

#define A180_PRT0_RELOAD (Z180_PRT_CLOCK / CLK_TCK)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: z180_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the systems.
 *
 ****************************************************************************/

static int z180_timerisr(int irq, chipreg_t *regs, void *arg)
{
  /* "When TMDR0 decrements to 0, TIF0 is set to 1. This generates an
   * interrupt request if enabled by TIE0 = 1. TIF0 is reset to 0 when TCR
   * is read and the higher or lower byte of TMDR0 is read."
   */

  inp(Z180_PRT_TCR);
  inp(Z180_PRT0_DRL);
  inp(Z180_PRT0_DRH);

  /* Process timer interrupt */

  nxsched_process_timer();
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: up_timer_initialize
 *
 * Description:
 *   This function is called during start-up to initialize the timer
 *   interrupt.
 *
 ****************************************************************************/

void up_timer_initialize(void)
{
  uint8_t regval;

  /* Configure PRT0 to interrupt at the requested rate */

  /* First stop PRT0 and disable interrupts */

  regval  = inp(Z180_PRT_TCR);
  regval &= (PRT_TCR_TIF0 | PRT_TCR_TIE0 | PRT_TCR_TDE0);
  outp(Z180_PRT_TCR, regval);

  /* Set the timer reload value so that the timer will interrupt at the
   * desired frequency.  "For writing, the TMDR down counting must be
   * inhibited using the TDE (Timer Down Count Enable) bits in the TCR
   * (Timer Control Register). Then, any or both higher and lower bytes of
   * TMDR can be freely written (and read) in any order."
   */

  outp(Z180_PRT0_RLDRL, (A180_PRT0_RELOAD & 0xff));
  outp(Z180_PRT0_RLDRH, (A180_PRT0_RELOAD >> 8));

  /* Enable down-counting */

  regval |= PRT_TCR_TDE0;
  outp(Z180_PRT_TCR, regval);

  /* Attach the timer interrupt vector */

  irq_attach(Z180_PRT0, (xcpt_t)z180_timerisr, NULL);

  /* And enable the timer interrupt */

  regval |= PRT_TCR_TIE0;
  outp(Z180_PRT_TCR, regval);
}
