/****************************************************************************
 * arch/arm/src/s32k1xx/s32k1xx_lowputc.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author:  Gregory Nutt <gnutt@nuttx.org>
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
#include <fixedmath.h>
#include <assert.h>
#include <errno.h>

#include "arm_arch.h"

#include "hardware/s32k1xx_pinmux.h"
#include "hardware/s32k1xx_lpuart.h"

#include "s32k1xx_config.h"
#include "s32k1xx_pin.h"
#include "s32k1xx_lowputc.h"
#include "s32k1xx_periphclocks.h"

#include "arm_internal.h"

#include <arch/board/board.h> /* Include last:  has dependencies */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifdef HAVE_LPUART_CONSOLE
#  if defined(CONFIG_LPUART0_SERIAL_CONSOLE)
#    define S32K1XX_CONSOLE_BASE     S32K1XX_LPUART0_BASE
#    define S32K1XX_CONSOLE_BAUD     CONFIG_LPUART0_BAUD
#    define S32K1XX_CONSOLE_BITS     CONFIG_LPUART0_BITS
#    define S32K1XX_CONSOLE_PARITY   CONFIG_LPUART0_PARITY
#    define S32K1XX_CONSOLE_2STOP    CONFIG_LPUART0_2STOP
#  elif defined(CONFIG_LPUART1_SERIAL_CONSOLE)
#    define S32K1XX_CONSOLE_BASE     S32K1XX_LPUART1_BASE
#    define S32K1XX_CONSOLE_BAUD     CONFIG_LPUART1_BAUD
#    define S32K1XX_CONSOLE_BITS     CONFIG_LPUART1_BITS
#    define S32K1XX_CONSOLE_PARITY   CONFIG_LPUART1_PARITY
#    define S32K1XX_CONSOLE_2STOP    CONFIG_LPUART1_2STOP
#  elif defined(CONFIG_LPUART2_SERIAL_CONSOLE)
#    define S32K1XX_CONSOLE_BASE     S32K1XX_LPUART2_BASE
#    define S32K1XX_CONSOLE_BAUD     CONFIG_LPUART2_BAUD
#    define S32K1XX_CONSOLE_BITS     CONFIG_LPUART2_BITS
#    define S32K1XX_CONSOLE_PARITY   CONFIG_LPUART2_PARITY
#    define S32K1XX_CONSOLE_2STOP    CONFIG_LPUART2_2STOP
#  endif
#endif

/* Clocking *****************************************************************/

/* Functional clocking is provided via the  PCC.  The PCC clocking must
 * be configured by board-specific logic prior to using the LPUART.
 */

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef HAVE_LPUART_CONSOLE
static const struct uart_config_s g_console_config =
{
  .baud      = S32K1XX_CONSOLE_BAUD,    /* Configured baud */
  .parity    = S32K1XX_CONSOLE_PARITY,  /* 0=none, 1=odd, 2=even */
  .bits      = S32K1XX_CONSOLE_BITS,    /* Number of bits (5-9) */
  .stopbits2 = S32K1XX_CONSOLE_2STOP,   /* true: Configure with 2 stop bits instead of 1 */
};
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s32k1xx_lowsetup
 *
 * Description:
 *   Called at the very beginning of _start.  Performs low level
 *   initialization including setup of the console UART.  This UART done
 *   early so that the serial console is available for debugging very early
 *   in the boot sequence.
 *
 ****************************************************************************/

void s32k1xx_lowsetup(void)
{
#ifndef CONFIG_SUPPRESS_LPUART_CONFIG
#ifdef HAVE_LPUART_DEVICE

#ifdef CONFIG_S32K1XX_LPUART0

  /* Configure LPUART0 pins: RXD and TXD.  Also configure RTS and CTS if flow
   * control is enabled.
   */

  s32k1xx_pinconfig(PIN_LPUART0_RX);
  s32k1xx_pinconfig(PIN_LPUART0_TX);
#ifdef CONFIG_LPUART0_OFLOWCONTROL
  s32k1xx_pinconfig(PIN_LPUART0_CTS);
#endif
#if ((defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART0_RS485RTSCONTROL)) || \
     (defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART0_IFLOWCONTROL)))
  s32k1xx_pinconfig(PIN_LPUART0_RTS);
#endif
#endif

#ifdef CONFIG_S32K1XX_LPUART1

  /* Configure LPUART1 pins: RXD and TXD.  Also configure RTS and CTS if flow
   * control is enabled.
   */

  s32k1xx_pinconfig(PIN_LPUART1_RX);
  s32k1xx_pinconfig(PIN_LPUART1_TX);
#ifdef CONFIG_LPUART1_OFLOWCONTROL
  s32k1xx_pinconfig(PIN_LPUART1_CTS);
#endif
#if ((defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART1_RS485RTSCONTROL)) || \
     (defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART1_IFLOWCONTROL)))
  s32k1xx_pinconfig(PIN_LPUART1_RTS);
#endif
#endif

#ifdef CONFIG_S32K1XX_LPUART2

  /* Configure LPUART2 pins: RXD and TXD.  Also configure RTS and CTS if flow
   * control is enabled.
   */

  s32k1xx_pinconfig(PIN_LPUART2_RX);
  s32k1xx_pinconfig(PIN_LPUART2_TX);
#ifdef CONFIG_LPUART2_OFLOWCONTROL
  s32k1xx_pinconfig(PIN_LPUART2_CTS);
#endif
#if ((defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART2_RS485RTSCONTROL)) || \
     (defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART2_IFLOWCONTROL)))
  s32k1xx_pinconfig(PIN_LPUART2_RTS);
#endif
#endif

#ifdef HAVE_LPUART_CONSOLE
  /* Configure the serial console for initial, non-interrupt driver mode */

  s32k1xx_lpuart_configure(S32K1XX_CONSOLE_BASE, &g_console_config);
#endif
#endif /* HAVE_LPUART_DEVICE */
#endif /* CONFIG_SUPPRESS_LPUART_CONFIG */
}

/****************************************************************************
 * Name: s32k1xx_lpuart_configure
 *
 * Description:
 *   Configure a UART for non-interrupt driven operation
 *
 ****************************************************************************/

#ifdef HAVE_LPUART_DEVICE
int s32k1xx_lpuart_configure(uint32_t base,
                             FAR const struct uart_config_s *config)
{
  enum clock_names_e clkname;
  uint32_t lpuart_freq = 0;
  uint16_t sbr;
  uint16_t temp_sbr;
  uint32_t osr;
  uint32_t temp_osr;
  uint32_t temp_diff;
  uint32_t calculated_baud;
  uint32_t baud_diff;
  uint32_t regval;
  int ret;

  /* Functional clocking is provided via the  PCC.  The PCC clocking must
   * be configured by board-specific logic prior to using the LPUART.
   */

  /* Get the PCC source clock */

#ifdef CONFIG_S32K1XX_LPUART0
  if (base == S32K1XX_LPUART0_BASE)
    {
      clkname = LPUART0_CLK;
    }
  else
#endif
#ifdef CONFIG_S32K1XX_LPUART1
  if (base == S32K1XX_LPUART1_BASE)
    {
      clkname = LPUART1_CLK;
    }
  else
#endif
#ifdef CONFIG_S32K1XX_LPUART2
  if (base == S32K1XX_LPUART2_BASE)
    {
      clkname = LPUART2_CLK;
    }
  else
#endif
    {
      DEBUGPANIC();
      return -EINVAL;
    }

  ret = s32k1xx_get_pclkfreq(clkname, &lpuart_freq);
  DEBUGASSERT(ret >= 0);
  if (ret < 0)
    {
      return ret;
    }

  /* This LPUART instantiation uses a slightly different baud rate
   * calculation.  The idea is to use the best OSR (over-sampling rate)
   * possible.
   *
   * NOTE: OSR is typically hard-set to 16 in other LPUART instantiations
   * loop to find the best OSR value possible, one that generates minimum
   * baud_diff iterate through the rest of the supported values of OSR
   */

  baud_diff = config->baud;
  osr       = 0;
  sbr       = 0;

  for (temp_osr = 4; temp_osr <= 32; temp_osr++)
    {
      /* Calculate the temporary sbr value   */

      temp_sbr = (lpuart_freq / (config->baud * temp_osr));

      /* Set temp_sbr to 1 if the sourceClockInHz can not satisfy the
       * desired baud rate.
       */

      if (temp_sbr == 0)
        {
          temp_sbr = 1;
        }

      /* Calculate the baud rate based on the temporary OSR and SBR values */

      calculated_baud = (lpuart_freq / (temp_osr * temp_sbr));
      temp_diff       = calculated_baud - config->baud;

      /* Select the better value between srb and (sbr + 1) */

      if (temp_diff > (config->baud - (lpuart_freq / (temp_osr * (temp_sbr + 1)))))
        {
          temp_diff = config->baud - (lpuart_freq / (temp_osr * (temp_sbr + 1)));
          temp_sbr++;
        }

      if (temp_diff <= baud_diff)
        {
          baud_diff = temp_diff;
          osr       = temp_osr;
          sbr       = temp_sbr;
        }
    }

  if (baud_diff > ((config->baud / 100) * 3))
    {
      /* Unacceptable baud rate difference of more than 3% */

      return ERROR;
    }

  /* Reset all internal logic and registers, except the Global Register */

  regval  = getreg32(base + S32K1XX_LPUART_GLOBAL_OFFSET);
  regval |= LPUART_GLOBAL_RST;
  putreg32(regval, base + S32K1XX_LPUART_GLOBAL_OFFSET);

  regval &= ~LPUART_GLOBAL_RST;
  putreg32(regval, base + S32K1XX_LPUART_GLOBAL_OFFSET);

  /* Construct MODIR register */

  regval = 0;

  if (config->userts)
    {
      regval |= LPUART_MODIR_RXRTSE;
    }
  else if (config->users485)
    {
      /* Both TX and RX side can't control RTS, so this gives
       * the RX side precedence. This should have been filtered
       * in layers above anyway, but it's just a precaution.
       */

      regval |= LPUART_MODIR_TXRTSE;
    }

  if (config->usects)
    {
      regval |= LPUART_MODIR_TXCTSE;
    }

  if (config->invrts)
    {
      regval |= LPUART_MODIR_TXRTSPOL;
    }

  putreg32(regval, base + S32K1XX_LPUART_MODIR_OFFSET);

  regval = 0;

  if ((osr > 3) && (osr < 8))
    {
      regval |= LPUART_BAUD_BOTHEDGE;
    }

  if (config->stopbits2)
    {
      regval |= LPUART_BAUD_SBNS;
    }

  regval |= LPUART_BAUD_OSR(osr) | LPUART_BAUD_SBR(sbr);
  putreg32(regval, base + S32K1XX_LPUART_BAUD_OFFSET);

  regval = 0;
  if (config->parity == 1)
    {
      regval |= LPUART_CTRL_PE | LPUART_CTRL_PT_ODD;
    }
  else if (config->parity == 2)
    {
      regval |= LPUART_CTRL_PE | LPUART_CTRL_PT_EVEN;
    }

  if (config->bits == 8)
    {
      regval &= ~LPUART_CTRL_M;
    }
  else if (config->bits == 9)
    {
      regval |= LPUART_CTRL_M;
    }
  else
    {
      /* REVISIT: Here should be added support of other bit modes. */

      return -ENOSYS;
    }

  regval |= LPUART_CTRL_RE | LPUART_CTRL_TE;
  putreg32(regval, base + S32K1XX_LPUART_CTRL_OFFSET);

  return OK;
}
#endif /* HAVE_LPUART_DEVICE */

/****************************************************************************
 * Name: s32k1xx_lowputc
 *
 * Description:
 *   Output a byte with as few system dependencies as possible.  This will
 *   even work BEFORE the console is initialized if we are booting from U-
 *   Boot (and the same UART is used for the console, of course.)
 *
 ****************************************************************************/

#if defined(HAVE_LPUART_DEVICE) && defined(CONFIG_DEBUG_FEATURES)
void s32k1xx_lowputc(int ch)
{
  while ((getreg32(S32K1XX_CONSOLE_BASE + S32K1XX_LPUART_STAT_OFFSET) &
         LPUART_STAT_TDRE) == 0)
    {
    }

  /* If the character to output is a newline, then pre-pend a carriage return */

  if (ch == '\n')
    {
      /* Send the carriage return by writing it into the UART_TXD register. */

      putreg32((uint32_t)'\r', S32K1XX_CONSOLE_BASE + S32K1XX_LPUART_DATA_OFFSET);

      /* Wait for the transmit register to be emptied. When the TXFE bit is
       * non-zero, the TX Buffer FIFO is empty.
       */

      while ((getreg32(S32K1XX_CONSOLE_BASE + S32K1XX_LPUART_STAT_OFFSET) &
             LPUART_STAT_TDRE) == 0)
        {
        }
    }

  /* Send the character by writing it into the UART_TXD register. */

  putreg32((uint32_t)ch, S32K1XX_CONSOLE_BASE + S32K1XX_LPUART_DATA_OFFSET);

  /* Wait for the transmit register to be emptied. When the TXFE bit is
   * non-zero, the TX Buffer FIFO is empty.
   */

  while ((getreg32(S32K1XX_CONSOLE_BASE + S32K1XX_LPUART_STAT_OFFSET) &
         LPUART_STAT_TDRE) == 0)
    {
    }
}
#endif
