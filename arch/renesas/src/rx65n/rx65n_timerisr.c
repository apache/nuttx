/****************************************************************************
 * arch/renesas/src/rx65n/rx65n_timerisr.c
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
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>
#include "arch/rx65n/irq.h"
#include "clock/clock.h"
#include "renesas_internal.h"
#include "arch/rx65n/iodefine.h"
#include "chip.h"
#include "rx65n_cmt.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The desired timer interrupt frequency is provided by the definition
 * CLK_TCK (see include/time.h).  CLK_TCK defines the desired number of
 * system clock ticks per second.  That value is a user configurable setting
 * that defaults to 100 (100 ticks per second = 10 MS interval).
 *
 * ITU1 operates in periodic timer mode.  TCNT counts up until it matches
 * the value of GRA0, then an interrupt is generated.  Two values must be
 * computed:
 *
 * (1) The divider that determines the rate at which TCNT increments, and
 * (2) The value of GRA0 that cause the interrupt to occur.
 *
 * These must be selected so that the frequency of interrupt generation is
 * CLK_TCK.  Ideally, we would like to use the full range of GRA0 for better
 * timing accuracy:
 */

/* The ideal divider would be one that generates exactly 65535 ticks in
 * 1/CLK_TCK seconds.  For example, if RX_CLOCK is 10MHz and CLK_TCK is
 * 100, then the ideal divider must be less greater than or equal to:
 *
 *   (10,000,000 / CLK_TCK) / 65535 = 1.525
 *
 * The actual selected divider would then have to be 2, resulting in a
 * counting rate of 5,000,0000 and a GRA0 setting of 50,000.
 */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  rx65n_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the systems.
 *
 ****************************************************************************/

static int rx65n_timerisr(int irq, uint32_t *regs, void *arg)
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
 *   This function is called during start-up to initialize
 *   the timer interrupt.
 *
 ****************************************************************************/

void up_timer_initialize(void)
{
  uint16_t reg16;
  uint32_t reg32;

  /* Enable write to System registers */

  putreg16(RX65N_PRCR_VALUE, RX65N_PRCR_ADDR);

  /* Start CMT module */

  reg32 = getreg32(RX65N_MSTPCRA_ADDR);
  reg32 &= (~RX65N_CMT_MSTPCRA_STOP);
  putreg32(reg32, RX65N_MSTPCRA_ADDR);

  /* Disable CMT interrupt */

  putreg16(RX65N_CMT_CMCR_DEFAULT, RX65N_CMT0_CMCR_ADDR);
  IEN(CMT0, CMI0) = 0;

  /* Set CMI0 priority level */

  IPR(CMT0, CMI0) = _0F_CMT_PRIORITY_LEVEL15;

  /* Set Counter to 0 initially */

  putreg16(0, RX65N_CMT0_CMCNT_ADDR);

  /* Set the CMCOR match value.  The interrupt will be generated when TNCT
   * increments to this value
   */

  putreg16(RX65N_CMT0_COUNT_VALUE, RX65N_CMT0_CMCOR_ADDR);

  /* Attach the IMIA0 IRQ */

  irq_attach(RX65N_CMI0_IRQ, (xcpt_t)rx65n_timerisr, NULL);

  /* Set control registers */

  putreg16(RX65N_CMT_CMCR_INIT, RX65N_CMT0_CMCR_ADDR);
  IEN(CMT0, CMI0) = 1;

  /* Start the timer */

  reg16  = getreg16(RX65N_CMT_CMSTR0_ADDR);
  reg16 |= RX65N_CMTCMSTR0_STR0;           /* Enable TCNT0 */
  putreg16(reg16, RX65N_CMT_CMSTR0_ADDR);  /* TCNT0 is counting */
}
