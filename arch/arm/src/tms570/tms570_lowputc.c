/****************************************************************************
 * arch/arm/src/tms570/tms570_lowputc.c
 *
 *   Copyright (C) 2015, 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Includes some logic from TI sample which has a compatible three-clause
 * BSD license and:
 *
 *   Copyright (c) 2012, Texas Instruments Incorporated
 *   All rights reserved.
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
#include <errno.h>

#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "arm_arch.h"

#include "hardware/tms570_sci.h"
#include "hardware/tms570_iomm.h"
#include "tms570_lowputc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Select SCI parameters for the selected console */

#if defined(CONFIG_SCI1_SERIAL_CONSOLE) && defined(CONFIG_TMS570_SCI1)
#  define TMS570_CONSOLE_BASE     TMS570_SCI1_BASE
#  define TMS570_CONSOLE_BAUD     CONFIG_SCI1_BAUD
#  define TMS570_CONSOLE_BITS     CONFIG_SCI1_BITS
#  define TMS570_CONSOLE_PARITY   CONFIG_SCI1_PARITY
#  define TMS570_CONSOLE_2STOP    CONFIG_SCI1_2STOP
#  define HAVE_SERIAL_CONSOLE     1
#elif defined(CONFIG_SCI2_SERIAL_CONSOLE) && defined(CONFIG_TMS570_SCI2)
#  define TMS570_CONSOLE_BASE     TMS570_SCI2_BASE
#  define TMS570_CONSOLE_BAUD     CONFIG_SCI2_BAUD
#  define TMS570_CONSOLE_BITS     CONFIG_SCI2_BITS
#  define TMS570_CONSOLE_PARITY   CONFIG_SCI2_PARITY
#  define TMS570_CONSOLE_2STOP    CONFIG_SCI2_2STOP
#  define HAVE_SERIAL_CONSOLE     1
#else
#  error "No CONFIG_SCIn_SERIAL_CONSOLE Setting"
#  undef HAVE_SERIAL_CONSOLE
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef HAVE_SERIAL_CONSOLE
static const struct sci_config_s g_console_config =
{
  .baud       = TMS570_CONSOLE_BAUD,
  .parity     = TMS570_CONSOLE_PARITY,
  .bits       = TMS570_CONSOLE_BITS,
  .stopbits2  = TMS570_CONSOLE_2STOP,
};
#endif /* HAVE_SERIAL_CONSOLE */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tms570_sci_initialize
 *
 * Description:
 *   Perform one-time initialization of the SCI module.
 *
 ****************************************************************************/

static void tms570_sci_initialize(uint32_t base)
{
#if 0
  uint32_t reg;

  reg = 0x83e70b13u;
  putreg32(reg, TMS570_IOMM_KICK0);

  reg = 0x95a4f1e0u;
  putreg32(reg, TMS570_IOMM_KICK1);

  reg = (2 << 16);
  putreg32(reg, TMS570_IOMM_PINMMR7);

  reg = (2 << 0);
  putreg32(reg, TMS570_IOMM_PINMMR8);

  reg = 0;
  putreg32(reg, TMS570_IOMM_KICK0);

  reg = 0;
  putreg32(reg, TMS570_IOMM_KICK1);
#endif

  /* Bring SCI1 out of reset */

  putreg32(0x0, base + TMS570_SCI_GCR0_OFFSET);
  putreg32(SCI_GCR0_RESET, base + TMS570_SCI_GCR0_OFFSET);

  /* Configure pins */

  /* Pin Function Register: RX is receive pin, TX is transmit pin. */

  putreg32(SCI_PIO_RX | SCI_PIO_TX, base + TMS570_SCI_FUN_OFFSET);

  /* Pin Data Out Register: Output values are logic low.  Irrelevant because
   * TX FUNC != 0 and RX FUNC != 0
   */

  putreg32(0, base + TMS570_SCI_DOUT_OFFSET);

  /* Pin Direction Register: General purpose inputs.  Irrelevant because
   * TX FUNC != 0 and RX FUNC != 0.
   */

  putreg32(0, base + TMS570_SCI_DIR_OFFSET);

  /* Set SCI pins open drain enable: ODR functionality disabled.  Irrelevant
   * because TX FUNC != 0 and RX FUNC != 0
   */

  putreg32(0, base + TMS570_SCI_ODR_OFFSET);

  /* Set SCI pins pullup/pulldown enable: Pull control enabled */

  putreg32(0, base + TMS570_SCI_PD_OFFSET);

  /* Set SCI pins pullup/pulldown select: Pulled up */

  putreg32(SCI_PIO_RX | SCI_PIO_TX, base + TMS570_SCI_PSL_OFFSET);
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

  for (; ; )
    {
      /* Wait for the transmitter to be available */

      while ((getreg32(TMS570_CONSOLE_BASE + TMS570_SCI_FLR_OFFSET) &
        SCI_FLR_TXRDY) == 0);

      /* Disable interrupts so that the test and the transmission are
       * atomic.
       */

      flags = enter_critical_section();
      if ((getreg32(TMS570_CONSOLE_BASE + TMS570_SCI_FLR_OFFSET) &
        SCI_FLR_TXRDY) != 0)
        {
          /* Send the character */

          putreg32((uint32_t)ch, TMS570_CONSOLE_BASE + TMS570_SCI_TD_OFFSET);
          leave_critical_section(flags);
          return;
        }

      leave_critical_section(flags);
    }
#endif
}

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes
 *
 ****************************************************************************/

int up_putc(int ch)
{
#ifdef HAVE_SERIAL_CONSOLE
  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      arm_lowputc('\r');
    }

  arm_lowputc(ch);
#endif
  return ch;
}

/****************************************************************************
 * Name: tms570_lowsetup
 *
 * Description:
 *   This performs basic initialization of the SCI used for the serial
 *   console.  Its purpose is to get the console output available as soon
 *   as possible.
 *
 ****************************************************************************/

void tms570_lowsetup(void)
{
#ifdef CONFIG_TMS570_SCI1
  /* Perform one-time SCI initialization */

  tms570_sci_initialize(TMS570_SCI1_BASE);
#endif

#ifdef CONFIG_TMS570_SCI2
  /* Perform one-time SCI initialization */

  tms570_sci_initialize(TMS570_SCI2_BASE);
#endif

#if defined(HAVE_SERIAL_CONSOLE) && !defined(CONFIG_SUPPRESS_SCI_CONFIG)
  /* Configure the console (only) */

  tms570_sci_configure(TMS570_CONSOLE_BASE, &g_console_config);
#endif
}

/****************************************************************************
 * Name: tms570_sci_configure
 *
 * Description:
 *   Configure an SCI for non-interrupt driven operation
 *
 ****************************************************************************/

int tms570_sci_configure(uint32_t base,
                         FAR const struct sci_config_s *config)
{
  float    divb7;
  uint32_t intpart;
  uint32_t p;
  uint32_t m;
  uint32_t u;
  uint32_t nbits;
  uint32_t regval;
  uint32_t gcr1;

  /* Pre-calculate the baudrate divisor with 7 bits of fraction
   *
   * The input clock to the baud rate generator is VCLK.
   * Asynchronous timing is assumed.
   */

  divb7 = BOARD_VCLK_FREQUENCY / (config->baud * 16) ;

  /* Break out the integer and fractional parts */

  intpart = (uint32_t)divb7;

  /* Disable all interrupts and map them all to INT0 */

  putreg32(SCI_INT_ALLINTS, base + TMS570_SCI_CLEARINT_OFFSET);
  putreg32(SCI_INT_ALLINTS, base + TMS570_SCI_CLEARINTLVL_OFFSET);

  /* Global control 1:
   * COMM=0        Idle line mode is used.
   * TIMING=1      Asynchronous timing is used.
   * PARENA=?      Depends on configuration settings
   * PARITY=?      Depends on configuration settings
   * STOP=?        Depends on configuration settings
   * CLOCK=1       The internal SCICLK is the clock source
   * LIN=0         LIN mode is disabled
   * SWRST=0       SCI is initiailized and held in reset state
   * SLEEP=0       Sleep mode is disabled
   * ADAPT=0       Automatic baud rate adjustment is disabled
   * MBUF=0        The multi-buffer mode is disabled.
   * CTYPE=0       Classic checksum is used.
   * HGEN=0        (Effective in LIN mode only)
   * STOPEXT=0     (Effective in LIN mode only)
   * LOOPBACK=0    Loop back mode is disabled
   * CONT=0        Freeze SCI when debug mode is entered
   * RXENA=1       Receiver is enabled
   * TXENA=1       Transmitter is enabled
   */

  gcr1 = (SCI_GCR1_TIMING | SCI_GCR1_CLOCK |
          SCI_GCR1_RXENA | SCI_GCR1_TXENA);

  DEBUGASSERT(config->parity >= 0 && config->parity <= 2);
  if (config->parity == 1)
    {
      gcr1 |= SCI_GCR1_PARENA;
    }
  else if (config->parity == 2)
    {
      gcr1 |= (SCI_GCR1_PARENA | SCI_GCR1_PARITY);
    }

  if (config->stopbits2)
    {
      gcr1 |= SCI_GCR1_STOP;
    }

  gcr1 = 0;
  gcr1 = (SCI_GCR1_TIMING | SCI_GCR1_CLOCK |
          SCI_GCR1_RXENA | SCI_GCR1_TXENA);
  putreg32(gcr1, base + TMS570_SCI_GCR1_OFFSET);

  p    = (uint32_t)intpart - 1;
  m    = (divb7 - intpart) * 16;
  u    = 0;

  regval = SCI_BRS_P(p) | SCI_BRS_M(m) | SCI_BRS_U(u);
  putreg32(regval, base + TMS570_SCI_BRS_OFFSET);

  /* Transmission length */

  nbits = config->bits;
  DEBUGASSERT(nbits >= 1 && nbits <= 8);

  if (nbits < 1)
    {
      nbits = 1;
    }
  else if (nbits > 8)
    {
      nbits = 8;
    }

  regval = SCI_FORMAT_CHAR(nbits - 1);
  putreg32(regval, base + TMS570_SCI_FORMAT_OFFSET);

  /* Put the SCI in its operational state. */

  gcr1 |= SCI_GCR1_SWRST;
  putreg32(gcr1, base + TMS570_SCI_GCR1_OFFSET);
  return OK;
}
