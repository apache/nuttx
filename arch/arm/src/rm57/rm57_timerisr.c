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
#include "rm57_dma.h"

#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RTI compare 0 provides the system tick.  When any SCI receives with DMA,
 * RTI compare 1 is dedicated to flushing the RX DMA rings: the SCI has no
 * idle-line interrupt, so without a periodic flush the bytes of a burst
 * shorter than half a ring buffer are not handed to the upper half until
 * further bytes arrive.  Both compares run off free-running counter 0.
 */

#if defined(USE_SERIALDRIVER) && defined(CONFIG_RM57_SERIAL_RXDMA_POLL)
#  define HAVE_SERIAL_RXDMA 1
#endif

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

/* Compare 1 period, in FRC0 counts, for the RX DMA flush */

#ifdef HAVE_SERIAL_RXDMA
#  define RTI_CMP1 (RTI_FRC0CLK / CONFIG_RM57_SERIAL_RXDMA_POLL_HZ)
#  if RTI_CMP1 < 1
#    error CONFIG_RM57_SERIAL_RXDMA_POLL_HZ is faster than the RTI counter
#  endif
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rm57_rxdma_pollisr
 *
 * Description:
 *   RTI Compare 1 interrupt handler.  Flushes any bytes that the SCI RX
 *   DMA has written to a ring buffer but that have not yet reached the
 *   DMA half/full interrupt threshold.
 *
 ****************************************************************************/

#ifdef HAVE_SERIAL_RXDMA
static int rm57_rxdma_pollisr(int irq, uint32_t *regs, void *arg)
{
  /* Clear the RTI Compare 1 interrupt */

  putreg32(RTI_INT_COMPARE1, RM57_RTI_INTFLAG);

  /* Each enabled port costs one pointer comparison when it has received
   * nothing since the previous poll.
   */

  rm57_serial_dma_poll();
  return 0;
}
#endif

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

#ifdef HAVE_SERIAL_RXDMA
  /* Same for compare 1, which drives the RX DMA flush.  Both compares are
   * driven by free-running counter 0, which is the reset state of the
   * COMPSEL fields but is made explicit here.
   */

  putreg32(RTI_CMP1, RM57_RTI_COMP1);
  putreg32(RTI_CMP1, RM57_RTI_UDCP1);

  putreg32(getreg32(RM57_RTI_COMPCTRL) &
           ~(RTI_COMPCTRL_COMPSEL0 | RTI_COMPCTRL_COMPSEL1),
           RM57_RTI_COMPCTRL);
#endif

  /* Configure the FRC0CLK clock via RTICPUC0 */

  putreg32(RTI_CPUC0, RM57_RTI_CPUC0);

  /* Initialize the free-running counter and the RTI up-counter */

  putreg32(0, RM57_RTI_FRC0);
  putreg32(0, RM57_RTI_UC0);

  /* Clear any pending interrupts */

  putreg32(0xffffffff, RM57_RTI_INTFLAG);

  /* Enable the compare interrupts in use (still disabled at the VIM) */

#ifdef HAVE_SERIAL_RXDMA
  putreg32(RTI_INT_COMPARE0 | RTI_INT_COMPARE1, RM57_RTI_SETINTENA);
#else
  putreg32(RTI_INT_COMPARE0, RM57_RTI_SETINTENA);
#endif

  /* Enable counter 0 */

  putreg32(RTI_GCTRL_CNT0EN, RM57_RTI_GCTRL);

  /* Attach the interrupt handler to the RTI Compare 0 interrupt */

  DEBUGVERIFY(irq_attach(RM57_REQ_RTICOMPARE0, (xcpt_t)rm57_timerisr, NULL));

  /* Enable RTI compare 0 interrupts at the VIM */

  up_enable_irq(RM57_REQ_RTICOMPARE0);

#ifdef HAVE_SERIAL_RXDMA
  /* Same for compare 1.  The handler tolerates ports whose DMA has not
   * been set up yet, so it can be enabled before the serial driver is
   * initialized.
   */

  DEBUGVERIFY(irq_attach(RM57_REQ_RTICOMPARE1, (xcpt_t)rm57_rxdma_pollisr,
                         NULL));
  up_enable_irq(RM57_REQ_RTICOMPARE1);
#endif
}
