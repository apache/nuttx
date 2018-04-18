/****************************************************************************
 * arch/arm/src/tms570/tms570_timerisr.c
 *
 *   Copyright (C) 2015, 2017-2018 Gregory Nutt. All rights reserved.
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

#include <nuttx/arch.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip/tms570_rti.h"

#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* The input clock to the RTI is the RTICLK.  The RTI source is always VCLK
 * which may be divided down by 2.  The correct RTICLK frequency must be
 * provided by board.h file as BOARD_RTICLK_FREQUENCY.
 */

#ifndef BOARD_RTICLK_FREQUENCY
#  error BOARD_RTICLK_FREQUENCY not defined
#endif

/* Timing Calculations:
 *
 *   FRC0CLK = RTICLK / (CPUC0 + 1)      Hz
 *   Tcount  = 1 / FRC0CLK               Seconds
 *           = 1,000,000 / FRC0CLK       Microseconds
 *   CMP0    = Period / Tcount
 *           = CONFIG_USEC_PER_TICK * FRC0CLK / 1,000,000
 *           = CONFIG_USEC_PER_TICK * RTICLK / (CPUC0 + 1) / 1,000,000
 *
 * For Example:
 *   VCLK    = 80,000,000                Hz
 *   RTICLK  = VCLK / 2                  Hz
 *           = 40,000,000                Hz
 *   CPUC0   = 39
 *   FR0CLK  = 1,000,000                 Hz
 *   Tcount  = 1                         Microsecond
 *   CONFIG_USEC_PER_TICK = 10,000       Microseconds
 *   CMP0    = 10,000 * 40,000,000 / 40 / 1,000,000
 *           = 10, 000 = CONFIG_USEC_PER_TICK
 */

#if BOARD_RTICLK_FREQUENCY > 10000000
  /* Use FR0CLK = 1MHz with CPUC0 at least 9 */

#  define RTI_FRC0CLK  (1000000)
#elif BOARD_RTICLK_FREQUENCY > 5000000
  /* Use FR0CLK = 500KHz with CPUC0 at least 9 */

#  define RTI_FRC0CLK  (500000)
#elif BOARD_RTICLK_FREQUENCY > 1000000
  /* Use FR0CLK = 100KHz with CPUC0 at least 9 */

#  define RTI_FRC0CLK  (100000)
#else
#  error No logic for this value of RTICLK
#endif

/* CPUC0 = RTICLK / FRC0CLK - 1
 *
 * NOTES:
 *   - The following calculation performs rounding.
 */

#define RTI_CPUC0 (((BOARD_RTICLK_FREQUENCY ) / RTI_FRC0CLK) - 1)

/* CMP0 = CONFIG_USEC_PER_TICK * FRC0CLK / 1,000,000
 *
 * NOTES:
 *   - The following calculation performs rounding.
 *   - The following calculation avoids integer overflow by depending on
 *     FRCLK being a multiple of 100,000
 */

#define RTI_CMP0  ((CONFIG_USEC_PER_TICK * (RTI_FRC0CLK / 100000) + 50) / 10)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  tms570_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the systems.
 *
 ****************************************************************************/

static int tms570_timerisr(int irq, uint32_t *regs, void *arg)
{
  /* Clear the RTI Compare 0 interrupts */

  putreg32(RTI_INT0, TMS570_RTI_INTFLAG);

  /* Process timer interrupt */

  sched_process_timer();
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  arm_timer_initialize
 *
 * Description:
 *   This function is called during start-up to initialize the timer
 *   interrupt.
 *
 ****************************************************************************/

void arm_timer_initialize(void)
{
  /* Disable all RTI interrupts */

  up_disable_irq(TMS570_REQ_RTICMP0);
  putreg32(0x0, TMS570_RTI_GCTRL);
  putreg32(RTI_ALLINTS, TMS570_RTI_CLEARINTENA);

  /* Configure RTICOMP0 register and the RTIUDCP0 Register to initialize with
   * the calculated compare value.
   */

  putreg32(RTI_CMP0, TMS570_RTI_COMP0);
  putreg32(RTI_CMP0, TMS570_RTI_UDCP0);

  /* Configure the FRC0CLK clock by setting the RTICPUC0 register to the
   * calculated value.
   */

  putreg32(RTI_CPUC0, TMS570_RTI_CPUC0);

  /* Initialize the free-running counter and the RTI up-counter */

  putreg32(0, TMS570_RTI_FRC0);
  putreg32(0, TMS570_RTI_UC0);

  /* Clear any pending interrupts */

  putreg32(RTI_ALLINTS, TMS570_RTI_INTFLAG);

  /* Enable the RTI Compare 0 interrupts (still disabled at the VIM) */

  putreg32(RTI_INT0, TMS570_RTI_SETINTENA);

  /* Enable counter 0 */

  putreg32(RTI_GCTRL_CNT0EN, TMS570_RTI_GCTRL);

  /* Attach the interrupt handler to the RTI Compare 0 interrupt */

  DEBUGVERIFY(irq_attach(TMS570_REQ_RTICMP0, (xcpt_t)tms570_timerisr, NULL));

  /* Enable RTI compare 0 interrupts at the VIM */

  up_enable_irq(TMS570_REQ_RTICMP0);
}
