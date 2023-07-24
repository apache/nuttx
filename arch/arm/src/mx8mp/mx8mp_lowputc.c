/****************************************************************************
 * arch/arm/src/mx8mp/mx8mp_lowputc.c
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
#include <assert.h>
#include <fixedmath.h>

#include "arm_internal.h"

#include "mx8mp_config.h"
#include "mx8mp_lowputc.h"
#include "mx8mp_iomuxc.h"
#include "mx8mp_ccm.h"
#include "hardware/mx8mp_pinmux.h"
#include "hardware/mx8mp_uart.h"
#include <arch/board/board.h> /* Include last:  has dependencies */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#if defined(HAVE_UART_CONSOLE)
#  if defined(CONFIG_UART1_SERIAL_CONSOLE)
#    define CONSOLE_BASE     MX8M_UART1
#    define CONSOLE_FREQ     UART1_CLK_ROOT
#    define CONSOLE_BAUD     CONFIG_UART1_BAUD
#    define CONSOLE_BITS     CONFIG_UART1_BITS
#    define CONSOLE_2STOP    CONFIG_UART1_2STOP
#    define CONSOLE_PARITY   CONFIG_UART1_PARITY
#  elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#    define CONSOLE_BASE     MX8M_UART2
#    define CONSOLE_FREQ     UART2_CLK_ROOT
#    define CONSOLE_BAUD     CONFIG_UART2_BAUD
#    define CONSOLE_BITS     CONFIG_UART2_BITS
#    define CONSOLE_2STOP    CONFIG_UART2_2STOP
#    define CONSOLE_PARITY   CONFIG_UART2_PARITY
#  elif defined(CONFIG_UART3_SERIAL_CONSOLE)
#    define CONSOLE_BASE     MX8M_UART3
#    define CONSOLE_FREQ     UART3_CLK_ROOT
#    define CONSOLE_BAUD     CONFIG_UART3_BAUD
#    define CONSOLE_BITS     CONFIG_UART3_BITS
#    define CONSOLE_2STOP    CONFIG_UART3_2STOP
#    define CONSOLE_PARITY   CONFIG_UART3_PARITY
#  elif defined(CONFIG_UART4_SERIAL_CONSOLE)
#    define CONSOLE_BASE     MX8M_UART4
#    define CONSOLE_CLK      UART4_CLK_ROOT
#    define CONSOLE_BAUD     CONFIG_UART4_BAUD
#    define CONSOLE_BITS     CONFIG_UART4_BITS
#    define CONSOLE_2STOP    CONFIG_UART4_2STOP
#    define CONSOLE_PARITY   CONFIG_UART4_PARITY
#  elif defined(HAVE_UART_CONSOLE)
#    error "No CONFIG_UARTn_SERIAL_CONSOLE Setting"
#  endif
#endif /* HAVE_UART_CONSOLE */

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef HAVE_UART_CONSOLE
static const struct uart_config_s g_console_config =
{
  .baud       = CONSOLE_BAUD,
  .parity     = CONSOLE_PARITY,
  .bits       = CONSOLE_BITS,
  .stopbits2  = CONSOLE_2STOP
};
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int mx8mp_uart_configure(uint32_t base,
                         uint32_t clk_frequency,
                         const struct uart_config_s *config)
{
#ifndef CONFIG_SUPPRESS_UART_CONFIG
  uint64_t tmp;
  uint32_t regval;
  uint32_t ucr2;
  uint32_t refclk;
  uint32_t div;
  uint32_t num;
  uint32_t den;
  b16_t ratio;

  /* Disable the UART */

  putreg32(0, base + UART_UCR1_OFFSET);
  putreg32(0, base + UART_UCR2_OFFSET);
  putreg32(0, base + UART_UCR3_OFFSET);
  putreg32(0, base + UART_UCR4_OFFSET);

  /* Wait for the UART to come out of reset */

  while ((getreg32(base + UART_UCR2_OFFSET) & UART_UCR2_SRST) == 0);

  /* Set up UCR2, Clearing all bits that will be configured below. */

  ucr2  = getreg32(base + UART_UCR2_OFFSET);
  ucr2 &= ~(UART_UCR2_WS   | UART_UCR2_STPB | UART_UCR2_PREN |
            UART_UCR2_PROE | UART_UCR2_IRTS | UART_UCR2_CTSC);

  /* Select the number of data bits */

  DEBUGASSERT(config->bits == 7 || config->bits == 8);
  if (config->bits == 8)
    {
      ucr2 |= UART_UCR2_WS;
    }

  /* Select the number of stop bits */

  if (config->stopbits2)
    {
      ucr2 |= UART_UCR2_STPB;
    }

  /* Select even/odd parity */

  if (config->parity != 0)
    {
      DEBUGASSERT(config->parity == 1 || config->parity == 2);
      ucr2 |= UART_UCR2_PREN;
      if (config->parity == 1)
        {
          ucr2 |= UART_UCR2_PROE;
        }
    }

  /* Setup hardware flow control */

  regval = 0;

#if 0
  if (config->hwfc)
    {
      /* CTS controlled by Rx FIFO */

      ucr2 |= UART_UCR2_CTSC;

      /* Set CTS trigger level */

      regval |= 30 << UART_UCR4_CTSTL_SHIFT;

      /* REVISIT:  There are other relevant bits that must be managed in
       * UCR1 and UCR3.
       */
    }
  else
#endif
    {
      /* Ignore RTS */

      ucr2 |= UART_UCR2_IRTS;
    }

  putreg32(regval, base + UART_UCR4_OFFSET);

  /* Setup the new UART configuration */

  putreg32(ucr2, base + UART_UCR2_OFFSET);

  /* Select a reference clock divider.
   * REVISIT:  For now we just use a divider of 2.  That might not be
   * optimal for very high or very low baud settings.
   */

  div    = 2;
  refclk = (clk_frequency >> 1);

  /* Set the baud.
   *
   *   baud    = REFFREQ / (16 * NUM/DEN)
   *   baud    = REFFREQ / 16 / RATIO
   *   RATIO   = REFREQ / 16 / baud;
   *
   *   NUM     = SCALE * RATIO
   *   DEN     = SCALE
   *
   *   UMBR    = NUM-1
   *   UBIR    = DEN-1;
   */

  tmp   = ((uint64_t)refclk << (16 - 4)) / config->baud;
  DEBUGASSERT(tmp < 0x0000000100000000ll);
  ratio = (b16_t)tmp;

  /* Pick a scale factor that gives us about 14 bits of accuracy.
   * REVISIT:  Why not go all the way to 16-bits?
   */

  if (ratio < b16HALF)
    {
      den = (1 << 15);
      num = b16toi(ratio << 15);
      DEBUGASSERT(num > 0);
    }
  else if (ratio < b16ONE)
    {
      den = (1 << 14);
      num = b16toi(ratio << 14);
    }
  else if (ratio < itob16(2))
    {
      den = (1 << 13);
      num = b16toi(ratio << 13);
    }
  else if (ratio < itob16(4))
    {
      den = (1 << 12);
      num = b16toi(ratio << 12);
    }
  else if (ratio < itob16(8))
    {
      den = (1 << 11);
      num = b16toi(ratio << 11);
    }
  else if (ratio < itob16(16))
    {
      den = (1 << 10);
      num = b16toi(ratio << 10);
    }
  else if (ratio < itob16(32))
    {
      den = (1 << 9);
      num = b16toi(ratio << 9);
    }
  else if (ratio < itob16(64))
    {
      den = (1 << 8);
      num = b16toi(ratio << 8);
    }
  else if (ratio < itob16(128))
    {
      den = (1 << 7);
      num = b16toi(ratio << 7);
    }
  else if (ratio < itob16(256))
    {
      den = (1 << 6);
      num = b16toi(ratio << 6);
    }
  else if (ratio < itob16(512))
    {
      den = (1 << 5);
      num = b16toi(ratio << 5);
    }
  else if (ratio < itob16(1024))
    {
      den = (1 << 4);
      num = b16toi(ratio << 4);
    }
  else if (ratio < itob16(2048))
    {
      den = (1 << 3);
      num = b16toi(ratio << 3);
    }
  else if (ratio < itob16(4096))
    {
      den = (1 << 2);
      num = b16toi(ratio << 2);
    }
  else if (ratio < itob16(8192))
    {
      den = (1 << 1);
      num = b16toi(ratio << 1);
    }
  else /* if (ratio < itob16(16384)) */
    {
      DEBUGASSERT(ratio < itob16(16384));
      den = (1 << 0);
      num = b16toi(ratio);
    }

  /* Reduce if possible without losing accuracy. */

  while ((num & 1) == 0 && (den & 1) == 0)
    {
      num >>= 1;
      den >>= 1;
    }

  /* The actual values are we write to the registers need to be
   * decremented by 1.  NOTE that the UBIR must be set before
   * the UBMR.
   */

  putreg32(den - 1, base + UART_UBIR_OFFSET);
  putreg32(num - 1, base + UART_UBMR_OFFSET);

  /* Fixup the divisor, the value in the UFCR register is
   *
   *   000 = Divide input clock by 6
   *   001 = Divide input clock by 5
   *   010 = Divide input clock by 4
   *   011 = Divide input clock by 3
   *   100 = Divide input clock by 2
   *   101 = Divide input clock by 1
   *   110 = Divide input clock by 7
   */

  if (div == 7)
    {
      div = 6;
    }
  else
    {
      div = 6 - div;
    }

  regval = div << UART_UFCR_RFDIV_SHIFT;

  /* Set the TX trigger level to interrupt when the TxFIFO has 2 or fewer
   * characters.  Set the RX trigger level to interrupt when the RxFIFO has
   * 1 character.
   */

  regval |= ((2 << UART_UFCR_TXTL_SHIFT) | (1 << UART_UFCR_RXTL_SHIFT));
  putreg32(regval, base + UART_UFCR_OFFSET);

  /* Selected. Selects proper input pins for serial and Infrared input
   * signal.  NOTE: In this chip, UARTs are used in MUXED mode, so that this
   * bit should always be set.
   */

  putreg32(UART_UCR3_RXDMUXSEL, base + UART_UCR3_OFFSET);

  /* Enable the TX and RX */

  ucr2 |= (UART_UCR2_TXEN | UART_UCR2_RXEN);
  putreg32(ucr2, base + UART_UCR2_OFFSET);

  /* Enable the UART */

  regval  = getreg32(base + UART_UCR1_OFFSET);
  regval |= UART_UCR1_UARTEN;
  putreg32(regval, base + UART_UCR1_OFFSET);
#endif

  return OK;
}

/****************************************************************************
 * Name: mx8mp_lowsetup
 *
 * Description:
 *   Called at the very beginning of _start.  Performs low level
 *   initialization including setup of the console UART.  This UART done
 *   early so that the serial console is available for debugging very early
 *   in the boot sequence.
 *
 ****************************************************************************/

void mx8mp_lowsetup(void)
{
#ifdef HAVE_UART_CONSOLE
  mx8mp_iomuxc_config(IOMUX_CONSOLE_UART_RX);
  mx8mp_iomuxc_config(IOMUX_CONSOLE_UART_TX);

  uint32_t clk = mx8mp_ccm_get_clock(CONSOLE_CLK);
  mx8mp_uart_configure(CONSOLE_BASE, clk, &g_console_config);
#endif
}

/****************************************************************************
 * Name: arm_lowputc
 *
 * Description:
 *   Output a byte with as few system dependencies as possible.  This will
 *   even work BEFORE the console is initialized if we are booting from
 *   U-Boot (and the same UART is used for the console, of course.)
 *
 ****************************************************************************/

void arm_lowputc(char ch)
{
#ifdef HAVE_UART_CONSOLE
  /* Poll the TX fifo trigger level bit of the UART status register. When the
   * TXFE bit is non-zero, the TX Buffer FIFO is empty.
   */

  while ((getreg32(CONSOLE_BASE + UART_USR2_OFFSET) & UART_USR2_TXFE) == 0);

  /* Send the character by writing it into the UART_TXD register. */

  putreg32((uint32_t)ch, CONSOLE_BASE + UART_TXD_OFFSET);
#endif
}
