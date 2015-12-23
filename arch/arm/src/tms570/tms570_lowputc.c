/****************************************************************************
 * arch/arm/src/tms570/tms570_lowputc.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Includes some logic from TI sample which has a compatibile three-clause
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

#include <arch/irq.h>
#include <arch/board/board.h>

#include "up_internal.h"
#include "up_arch.h"

#include "tms570_config.h"
#include "tms570_lowputc.h"

#include "chip/tms570_sci.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration **********************************************************/

#ifdef HAVE_SERIAL_CONSOLE

/* BAUD definitions
 *
 * The source clock is selectable and could be one of:
 *
 *   - The peripheral clock
 *   - A division of the peripheral clock, where the divider is product-
 *     dependent, but generally set to 8
 *   - A processor/peripheral independent clock source fully programmable
 *      provided by PMC (PCK)
 *   - The external clock, available on the SCK pin
 *
 * Only the first two options are supported by this driver.  The divided
 * peripheral clock is only used for very low BAUD selections.
 */

#define FAST_SCI_CLOCK   BOARD_MCK_FREQUENCY
#define SLOW_SCI_CLOCK   (BOARD_MCK_FREQUENCY >> 3)

/* Select SCI parameters for the selected console */

#  if defined(CONFIG_SCI1_SERIAL_CONSOLE)
#    define TMS570_CONSOLE_BASE     TMS570_SCI1_BASE
#    define TMS570_CONSOLE_BAUD     CONFIG_SCI1_BAUD
#    define TMS570_CONSOLE_BITS     CONFIG_SCI1_BITS
#    define TMS570_CONSOLE_PARITY   CONFIG_SCI1_PARITY
#    define TMS570_CONSOLE_2STOP    CONFIG_SCI1_2STOP
#  elif defined(CONFIG_SCI2_SERIAL_CONSOLE)
#    define TMS570_CONSOLE_BASE     TMS570_SCI2_BASE
#    define TMS570_CONSOLE_BAUD     CONFIG_SCI2_BAUD
#    define TMS570_CONSOLE_BITS     CONFIG_SCI2_BITS
#    define TMS570_CONSOLE_PARITY   CONFIG_SCI2_PARITY
#    define TMS570_CONSOLE_2STOP    CONFIG_SCI2_2STOP
#  else
#    error "No CONFIG_SCIn_SERIAL_CONSOLE Setting"
#  endif

/* Select the settings for the mode register */

#  if TMS570_CONSOLE_BITS == 5
#    define MR_CHRL_VALUE SCI_MR_CHRL_5BITS /* 5 bits */
#  elif TMS570_CONSOLE_BITS == 6
#    define MR_CHRL_VALUE SCI_MR_CHRL_6BITS  /* 6 bits */
#  elif TMS570_CONSOLE_BITS == 7
#    define MR_CHRL_VALUE SCI_MR_CHRL_7BITS /* 7 bits */
#  elif TMS570_CONSOLE_BITS == 8
#    define MR_CHRL_VALUE SCI_MR_CHRL_8BITS /* 8 bits */
#  elif TMS570_CONSOLE_BITS == 9 && !defined(CONFIG_SCI1_SERIAL_CONSOLE) && \
       !defined(CONFIG_SCI2_SERIAL_CONSOLE)
#    define MR_CHRL_VALUE SCI_MR_MODE9
#  else
#    error "Invalid number of bits"
#  endif

#  if TMS570_CONSOLE_PARITY == 1
#    define MR_PAR_VALUE SCI_MR_PAR_ODD
#  elif TMS570_CONSOLE_PARITY == 2
#    define MR_PAR_VALUE SCI_MR_PAR_EVEN
#  else
#    define MR_PAR_VALUE SCI_MR_PAR_NONE
#  endif

#  if TMS570_CONSOLE_2STOP != 0
#    define MR_NBSTOP_VALUE SCI_MR_NBSTOP_2
#  else
#    define MR_NBSTOP_VALUE SCI_MR_NBSTOP_1
#  endif

#  define MR_VALUE (SCI_MR_MODE_NORMAL | SCI_MR_USCLKS_MCK | \
                    MR_CHRL_VALUE | MR_PAR_VALUE | MR_NBSTOP_VALUE)

#endif /* HAVE_SERIAL_CONSOLE */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_lowputc
 *
 * Description:
 *   Output one byte on the serial console
 *
 ****************************************************************************/

void up_lowputc(char ch)
{
#ifdef HAVE_SERIAL_CONSOLE
  irqstate_t flags;

  for (; ; )
    {
      /* Wait for the transmitter to be available */

      while ((getreg32(TMS570_CONSOLE_BASE + TMS570_SCI_SR_OFFSET) &
        SCI_INT_TXEMPTY) == 0);

      /* Disable interrupts so that the test and the transmission are
       * atomic.
       */

      flags = irqsave();
      if ((getreg32(TMS570_CONSOLE_BASE + TMS570_SCI_SR_OFFSET) &
        SCI_INT_TXEMPTY) != 0)
        {
          /* Send the character */

          putreg32((uint32_t)ch, TMS570_CONSOLE_BASE + TMS570_SCI_THR_OFFSET);
          irqrestore(flags);
          return;
        }

      irqrestore(flags);
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

      up_lowputc('\r');
    }

  up_lowputc(ch);
#endif
  return ch;
}

/****************************************************************************
 * Name: tms570_lowsetup
 *
 * Description:
 *   This performs basic initialization of the SCI used for the serial
 *   console.  Its purpose is to get the console output availabe as soon
 *   as possible.
 *
 ****************************************************************************/

void tms570_lowsetup(void)
{
#if defined(HAVE_SERIAL_CONSOLE) && !defined(CONFIG_SUPPRESS_SCI_CONFIG)
  uint64_t divb3;
  uint32_t intpart;
  uint32_t fracpart;
  uint32_t regval;
#endif

  /* Configure the console (only) */
#if defined(HAVE_SERIAL_CONSOLE) && !defined(CONFIG_SUPPRESS_SCI_CONFIG)
  /* Reset and disable receiver and transmitter */

  putreg32((SCI_CR_RSTRX | SCI_CR_RSTTX | SCI_CR_RXDIS | SCI_CR_TXDIS),
           TMS570_CONSOLE_BASE + TMS570_SCI_CR_OFFSET);

  /* Disable all interrupts */

  putreg32(0xffffffff, TMS570_CONSOLE_BASE + TMS570_SCI_IDR_OFFSET);

  /* Set up the mode register */

  putreg32(MR_VALUE, TMS570_CONSOLE_BASE + TMS570_SCI_MR_OFFSET);

  /* Configure the console baud:
   *
   *   Fbaud   = SCI_CLOCK / (16 * divisor)
   *   divisor = SCI_CLOCK / (16 * Fbaud)
   *
   * NOTE: Oversampling by 8 is not supported. This may limit BAUD rates
   * for lower SCI clocks.
   */

  divb3    = ((FAST_SCI_CLOCK + (TMS570_CONSOLE_BAUD << 3)) << 3) /
             (TMS570_CONSOLE_BAUD << 4);
  intpart  = (divb3 >> 3);
  fracpart = (divb3 & 7);

  /* Retain the fast MR peripheral clock UNLESS unless using that clock
   * would result in an excessively large divider.
   *
   * REVISIT: The fractional divider is not used.
   */

  if ((intpart & ~SCI_BRGR_CD_MASK) != 0)
    {
      /* Use the divided SCI clock */

      divb3    = ((SLOW_SCI_CLOCK + (TMS570_CONSOLE_BAUD << 3)) << 3) /
                 (TMS570_CONSOLE_BAUD << 4);
      intpart  = (divb3 >> 3);
      fracpart = (divb3 & 7);

      /* Re-select the clock source */

      regval  = getreg32(TMS570_CONSOLE_BASE + TMS570_SCI_MR_OFFSET);
      regval &= ~SCI_MR_USCLKS_MASK;
      regval |= SCI_MR_USCLKS_MCKDIV;
      putreg32(regval, TMS570_CONSOLE_BASE + TMS570_SCI_MR_OFFSET);
    }

  /* Save the BAUD divider (the fractional part is not used for SCIs) */

  regval = SCI_BRGR_CD(intpart) | SCI_BRGR_FP(fracpart);
  putreg32(regval, TMS570_CONSOLE_BASE + TMS570_SCI_BRGR_OFFSET);

  /* Enable receiver & transmitter */

  putreg32((SCI_CR_RXEN | SCI_CR_TXEN),
           TMS570_CONSOLE_BASE + TMS570_SCI_CR_OFFSET);
#endif
}
