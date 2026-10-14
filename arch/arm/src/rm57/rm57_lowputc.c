/****************************************************************************
 * arch/arm/src/rm57/rm57_lowputc.c
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

/* Adapted from tms570_lowputc.c, using RM57's SCI register layout
 * (hardware/rm57_sci.h). Supports SCI1-SCI4; SCI1/LIN1 and SCI2/LIN2 are
 * dual-role SCI/LIN modules, driven here in SCI-compatibility mode (the
 * LIN_MODE and MBUF_MODE bits in GCR1 are never set by
 * rm57_sci_configure()).
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>
#include <nuttx/spinlock.h>

#include "arm_internal.h"
#include "hardware/rm57_sci.h"
#include "rm57_lowputc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Select SCI parameters for the selected console. At most one of
 * CONFIG_SCIn_SERIAL_CONSOLE will be set (see the "choice" in
 * drivers/serial/Kconfig); if none is set, the board has no serial
 * console (HAVE_SERIAL_CONSOLE stays undefined) rather than a build
 * error, since a console is no longer mandatory now that SCI2-4 are
 * available as plain /dev/ttySx devices.
 */

#if defined(CONFIG_SCI1_SERIAL_CONSOLE) && defined(CONFIG_RM57_SCI1)
#  define RM57_CONSOLE_BASE     RM57_SCI1_BASE
#  define RM57_CONSOLE_BAUD     CONFIG_SCI1_BAUD
#  define RM57_CONSOLE_BITS     8
#  define RM57_CONSOLE_PARITY   0
#  define RM57_CONSOLE_2STOP    CONFIG_SCI1_2STOP
#  define HAVE_SERIAL_CONSOLE   1
#elif defined(CONFIG_SCI2_SERIAL_CONSOLE) && defined(CONFIG_RM57_SCI2)
#  define RM57_CONSOLE_BASE     RM57_SCI2_BASE
#  define RM57_CONSOLE_BAUD     CONFIG_SCI2_BAUD
#  define RM57_CONSOLE_BITS     8
#  define RM57_CONSOLE_PARITY   0
#  define RM57_CONSOLE_2STOP    CONFIG_SCI2_2STOP
#  define HAVE_SERIAL_CONSOLE   1
#elif defined(CONFIG_SCI3_SERIAL_CONSOLE) && defined(CONFIG_RM57_SCI3)
#  define RM57_CONSOLE_BASE     RM57_SCI3_BASE
#  define RM57_CONSOLE_BAUD     CONFIG_SCI3_BAUD
#  define RM57_CONSOLE_BITS     8
#  define RM57_CONSOLE_PARITY   0
#  define RM57_CONSOLE_2STOP    CONFIG_SCI3_2STOP
#  define HAVE_SERIAL_CONSOLE   1
#elif defined(CONFIG_SCI4_SERIAL_CONSOLE) && defined(CONFIG_RM57_SCI4)
#  define RM57_CONSOLE_BASE     RM57_SCI4_BASE
#  define RM57_CONSOLE_BAUD     CONFIG_SCI4_BAUD
#  define RM57_CONSOLE_BITS     8
#  define RM57_CONSOLE_PARITY   0
#  define RM57_CONSOLE_2STOP    CONFIG_SCI4_2STOP
#  define HAVE_SERIAL_CONSOLE   1
#else
#  undef HAVE_SERIAL_CONSOLE
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static spinlock_t g_rm57_lowputc_lock = SP_UNLOCKED;

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef HAVE_SERIAL_CONSOLE
static const struct sci_config_s g_console_config =
{
  .baud       = RM57_CONSOLE_BAUD,
  .parity     = RM57_CONSOLE_PARITY,
  .bits       = RM57_CONSOLE_BITS,
  .stopbits2  = RM57_CONSOLE_2STOP,
};
#endif /* HAVE_SERIAL_CONSOLE */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rm57_sci_initialize
 *
 * Description:
 *   Perform one-time initialization of an SCI module: bring it out of
 *   reset and configure its RX/TX pins.
 *
 ****************************************************************************/

static void rm57_sci_initialize(uint32_t base)
{
  /* Bring the SCI out of reset */

  putreg32(0x0, base + RM57_SCI_GCR0_OFFSET);
  putreg32(SCI_GCR0_RESET, base + RM57_SCI_GCR0_OFFSET);

  /* Pin Function Register: RX is receive pin, TX is transmit pin */

  putreg32(SCI_PIO_RX | SCI_PIO_TX, base + RM57_SCI_PIO0_OFFSET);

  /* Pin Direction Register: general purpose inputs (irrelevant, TX/RX
   * function bits above take priority)
   */

  putreg32(0, base + RM57_SCI_PIO1_OFFSET);

  /* Pin Open Drain Output Enable Register: disabled */

  putreg32(0, base + RM57_SCI_PIO6_OFFSET);

  /* Pin Pullup/Pulldown Disable Register: pull control enabled */

  putreg32(0, base + RM57_SCI_PIO7_OFFSET);

  /* Pin Pullup/Pulldown Selection Register: pulled up */

  putreg32(SCI_PIO_RX | SCI_PIO_TX, base + RM57_SCI_PIO8_OFFSET);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_lowputc
 *
 * Description:
 *   Output one byte on the serial console
 *
 ****************************************************************************/

void arm_lowputc(char ch)
{
#ifdef HAVE_SERIAL_CONSOLE
  irqstate_t flags;

  /* Wait for the transmitter to be available */

  flags = spin_lock_irqsave(&g_rm57_lowputc_lock);

  while ((getreg32(RM57_CONSOLE_BASE + RM57_SCI_FLR_OFFSET) &
          SCI_FLR_TXRDY) == 0)
    {
    }

  /* Send the character */

  putreg32((uint32_t)ch, RM57_CONSOLE_BASE + RM57_SCI_TD_OFFSET);

  spin_unlock_irqrestore(&g_rm57_lowputc_lock, flags);
#endif
}

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes
 *
 ****************************************************************************/

void up_putc(int ch)
{
#ifdef HAVE_SERIAL_CONSOLE
  arm_lowputc(ch);
#endif
}

/****************************************************************************
 * Name: rm57_lowflush
 *
 * Description:
 *   Wait until everything written with arm_lowputc() has left the shift
 *   register.  The serial driver resets the SCI when it takes over the
 *   console, which would otherwise cut off the last characters.
 *
 ****************************************************************************/

void rm57_lowflush(void)
{
#ifdef HAVE_SERIAL_CONSOLE
  while ((getreg32(RM57_CONSOLE_BASE + RM57_SCI_FLR_OFFSET) &
          SCI_FLR_TXEMPTY) == 0)
    {
    }
#endif
}

/****************************************************************************
 * Name: rm57_lowsetup
 *
 * Description:
 *   This performs basic initialization of the SCI used for the serial
 *   console.  Its purpose is to get the console output available as soon
 *   as possible.
 *
 ****************************************************************************/

void rm57_lowsetup(void)
{
#ifdef CONFIG_RM57_SCI1
  rm57_sci_initialize(RM57_SCI1_BASE);
#endif

#ifdef CONFIG_RM57_SCI2
  rm57_sci_initialize(RM57_SCI2_BASE);
#endif

#ifdef CONFIG_RM57_SCI3
  rm57_sci_initialize(RM57_SCI3_BASE);
#endif

#ifdef CONFIG_RM57_SCI4
  rm57_sci_initialize(RM57_SCI4_BASE);
#endif

#if defined(HAVE_SERIAL_CONSOLE) && !defined(CONFIG_SUPPRESS_SCI_CONFIG)
  /* Configure the console (only) */

  rm57_sci_configure(RM57_CONSOLE_BASE, &g_console_config);
#endif
}

/****************************************************************************
 * Name: rm57_sci_configure
 *
 * Description:
 *   Configure an SCI for non-interrupt driven operation
 *
 ****************************************************************************/

int rm57_sci_configure(uint32_t base, const struct sci_config_s *config)
{
  float divb7;
  uint32_t intpart;
  uint32_t p;
  uint32_t regval;
  uint32_t gcr1;
  uint32_t nbits;

  /* Pre-calculate the baud-rate divisor (integer part only - see note
   * below). The input clock to the baud rate generator is VCLK (asynchronous
   * timing).
   */

  divb7 = (float)BOARD_VCLK_FREQUENCY / (config->baud * 16);
  intpart = (uint32_t)divb7;

  /* Disable all interrupts */

  putreg32(SCI_INT_ALLINTS, base + RM57_SCI_CLEARINT_OFFSET);
  putreg32(SCI_INT_ALLINTS, base + RM57_SCI_CLEARINTLVL_OFFSET);

  /* Global Control 1: async timing, internal clock, parity/stop-bits per
   * config, receiver+transmitter enabled
   */

  gcr1 = SCI_GCR1_TIMING_MODE | SCI_GCR1_CLOCK |
         SCI_GCR1_RXENA | SCI_GCR1_TXENA;

  DEBUGASSERT(config->parity <= 2);
  if (config->parity == 1)
    {
      gcr1 |= SCI_GCR1_PARITY_ENA;
    }
  else if (config->parity == 2)
    {
      gcr1 |= (SCI_GCR1_PARITY_ENA | SCI_GCR1_PARITY);
    }

  if (config->stopbits2)
    {
      gcr1 |= SCI_GCR1_STOP;
    }

  /* Internal self test: connect the transmitter to the receiver inside the
   * module so that a port can be exercised without any external wiring
   * (TRM SPNU562A 29.4, "Set LOOP BACK bit in SCIGCR1 to 1 to connect the
   * transmitter to the receiver internally").
   */

  if (config->loopback)
    {
      gcr1 |= SCI_GCR1_LOOPBACK;
    }

  putreg32(gcr1, base + RM57_SCI_GCR1_OFFSET);

  /* Baud rate divisor. Only the integer P divisor is used (M=0), matching
   * TI's HALCoGen-generated sciInit() for this SCI IP, e.g.
   * BRS=487 for a 9600 target at BOARD_VCLK_FREQUENCY yields the
   * documented 9606 actual baud. A fractional M term should not be added
   * without validating it against real hardware first.
   */

  p = intpart - 1;

  regval = SCI_BRS_P(p);
  putreg32(regval, base + RM57_SCI_BRS_OFFSET);

  /* Transmission length */

  nbits = config->bits;
  DEBUGASSERT(nbits >= 1 && nbits <= 8);

  regval = SCI_FORMAT_CHAR(nbits - 1);
  putreg32(regval, base + RM57_SCI_FORMAT_OFFSET);

  /* Put the SCI in its operational state */

  gcr1 |= SCI_GCR1_SWRST;
  putreg32(gcr1, base + RM57_SCI_GCR1_OFFSET);
  return OK;
}
