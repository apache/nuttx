/****************************************************************************
 * arch/arm/src/rm57/rm57_timerisr.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

/* Adapted from tms570_timerisr.c, using RM57's RTI1 register layout
 * (hardware/rm57_rti.h) which matches TMS570's RTI/DWWD module -
 * same peripheral IP, same base offset pattern.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/arch.h>

#include "arm_internal.h"
#include "hardware/rm57_rti.h"

#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The input clock to the RTI is RTICLK, sourced from VCLK.  The correct
 * RTICLK frequency must be provided by board.h as BOARD_RTICLK_FREQUENCY.
 */

#ifndef BOARD_RTICLK_FREQUENCY
#  error BOARD_RTICLK_FREQUENCY not defined
#endif

/* Timing Calculations:
 *
 *   FRC0CLK = RTICLK / (CPUC0 + 1)      Hz
 *   Tcount  = 1,000,000 / FRC0CLK       Microseconds
 *   CMP0    = CONFIG_USEC_PER_TICK * FRC0CLK / 1,000,000
 *
 * (identical derivation to the sibling tms570_timerisr.c)
 */

#if BOARD_RTICLK_FREQUENCY > 10000000
#  define RTI_FRC0CLK  (1000000)
#elif BOARD_RTICLK_FREQUENCY > 5000000
#  define RTI_FRC0CLK  (500000)
#elif BOARD_RTICLK_FREQUENCY > 1000000
#  define RTI_FRC0CLK  (100000)
#else
#  error No logic for this value of RTICLK
#endif

#define RTI_CPUC0 (((BOARD_RTICLK_FREQUENCY) / RTI_FRC0CLK) - 1)

#define RTI_CMP0  ((CONFIG_USEC_PER_TICK * (RTI_FRC0CLK / 100000) + 50) / 10)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rm57_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the systems.
 *
 ****************************************************************************/

static int rm57_timerisr(int irq, uint32_t *regs, void *arg)
{
  /* Clear the RTI Compare 0 interrupt */

  putreg32(RTI_INT_COMPARE0, RM57_RTI_INTFLAG);

  /* Process timer interrupt */

  nxsched_process_timer();
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_timer_initialize
 *
 * Description:
 *   This function is called during start-up to initialize the timer
 *   interrupt.
 *
 ****************************************************************************/

void up_timer_initialize(void)
{
  /* Disable all RTI interrupts */

  up_disable_irq(RM57_REQ_RTICOMPARE0);
  putreg32(0x0, RM57_RTI_GCTRL);
  putreg32(0xffffffff, RM57_RTI_CLEARINTENA);

  /* Configure RTICOMP0 and RTIUDCP0 with the calculated compare value */

  putreg32(RTI_CMP0, RM57_RTI_COMP0);
  putreg32(RTI_CMP0, RM57_RTI_UDCP0);

  /* Configure the FRC0CLK clock via RTICPUC0 */

  putreg32(RTI_CPUC0, RM57_RTI_CPUC0);

  /* Initialize the free-running counter and the RTI up-counter */

  putreg32(0, RM57_RTI_FRC0);
  putreg32(0, RM57_RTI_UC0);

  /* Clear any pending interrupts */

  putreg32(0xffffffff, RM57_RTI_INTFLAG);

  /* Enable the RTI Compare 0 interrupt (still disabled at the VIM) */

  putreg32(RTI_INT_COMPARE0, RM57_RTI_SETINTENA);

  /* Enable counter 0 */

  putreg32(RTI_GCTRL_CNT0EN, RM57_RTI_GCTRL);

  /* Attach the interrupt handler to the RTI Compare 0 interrupt */

  DEBUGVERIFY(irq_attach(RM57_REQ_RTICOMPARE0, (xcpt_t)rm57_timerisr, NULL));

  /* Enable RTI compare 0 interrupts at the VIM */

  up_enable_irq(RM57_REQ_RTICOMPARE0);
}
