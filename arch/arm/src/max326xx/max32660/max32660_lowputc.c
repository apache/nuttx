/****************************************************************************
 * arch/arm/src/max326xx/max32660_lowputc.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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

#include <stdbool.h>
#include <fixedmath.h>

#include "arm_arch.h"
#include "arm_internal.h"

#include "hardware/max326_memorymap.h"
#include "hardware/max326_pinmux.h"
#include "hardware/max326_uart.h"

#include "max326_config.h"
#include "max326_periphclks.h"
#include "max326_clockconfig.h"
#include "max326_gpio.h"
#include "max326_lowputc.h"

#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_UART0_SERIAL_CONSOLE)
#  define CONSOLE_BASE        MAX326_UART0_BASE
#  define CONSOLE_BAUD        CONFIG_UART0_BAUD
#  define CONSOLE_PARITY      CONFIG_UART0_PARITY
#  define CONSOLE_BITS        CONFIG_UART0_BITS
#  define CONSOLE_STOPBITS2   CONFIG_UART0_2STOP
#  ifdef CONFIG_UART0_IFLOWCONTROL
#    define CONSOLE_IFLOW     true
#  else
#    define CONSOLE_IFLOW     false
#  endif
#  ifdef CONFIG_UART0_OFLOWCONTROL
#    define CONSOLE_OFLOW     true
#  else
#    define CONSOLE_OFLOW     false
#  endif
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#  define CONSOLE_BASE        MAX326_UART1_BASE
#  define CONSOLE_BAUD        CONFIG_UART1_BAUD
#  define CONSOLE_PARITY      CONFIG_UART1_PARITY
#  define CONSOLE_BITS        CONFIG_UART1_BITS
#  define CONSOLE_STOPBITS2   CONFIG_UART1_2STOP
#  ifdef CONFIG_UART1_IFLOWCONTROL
#    define CONSOLE_IFLOW     true
#  else
#    define CONSOLE_IFLOW     false
#  endif
#  ifdef CONFIG_UART1_OFLOWCONTROL
#    define CONSOLE_OFLOW     true
#  else
#    define CONSOLE_OFLOW     false
#  endif
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef HAVE_UART_CONSOLE
/* UART console configuration */

static const struct uart_config_s g_console_config =
{
  .baud      = CONSOLE_BAUD,
  .parity    = CONSOLE_PARITY,
  .bits      = CONSOLE_BITS,
  .txlevel   = MAX326_UART_TXFIFO_DEPTH / 2,
  .rxlevel   = MAX326_UART_RXFIFO_DEPTH / 4,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rtslevel  = 3 * MAX326_UART_RXFIFO_DEPTH / 4,
#endif
  .stopbits2 = CONSOLE_STOPBITS2,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .iflow     = CONSOLE_IFLOW,
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  .oflow     = CONSOLE_OFLOW,
#endif
};
#endif /* HAVE_UART_CONSOLE */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: max326_setbaud
 *
 * Description:
 *   Configure the UART BAUD.
 *
 *   The UART peripheral clock, Fpclk , is used as the input clock to the
 *   UART bit rate generator. The following fields are used to set the
 *   target bit rate for the UART.
 *
 *     UARTn_BAUD0.CLKDIV selects the bit rate clock divisor.
 *     UARTn_BAUD0.IBAUD sets the integer portion of the bit rate divisor.
 *     UARTn_BAUD1.DBAUD sets the decimal portion of the bit rate divisor.
 *
 *   UART Bit Rate Divisor Equation:
 *
 *     DIV = Fpclk / (CLKDIV x Fbaud)
 *
 *   Where CLKDIV must selected to get the most accurat value and also to
 *   keep the IBAUD value within range.
 *
 *   Bit Rate Integer Calculation:
 *
 *     IBAUD = floor(DIV)
 *
 *   Bit Rate Remainder Calculation:
 *
 *     DBAUD = (DIV - IBAUD) * 128
 *
 ****************************************************************************/

#ifdef HAVE_UART_DEVICE
static void max326_setbaud(uintptr_t base,
                           FAR const struct uart_config_s *config)
{
  ub32_t div;
  ub32_t pclk;
  uint32_t regval;
  uint32_t ibaud;
  unsigned int divider;
  int clkdiv;
  int i;

  /* Try all possible values of CLKDIV to find the smallest divider that
   * results in an integer divider >= 1.
   */

  pclk   = itob32(max326_pclk_frequency());
  clkdiv = -1;

  for (i = 0; i < 5; i++)
    {
      /* Calculate the divider associated with this value of clkdiv:
       * 128, 64, 32, 16, 8
       */

      divider = (1 << (7 - i));

      /* Calculate the temporary IBAUD value */

      div = pclk / (divider * config->baud);

      /* We are traversing from larger to larger smaller values.  Hence, the
       * 'div' value will be increasing with the smaller values.  So break
       * out of the loop as soon as we encounter a divider greater than or
       * equal to one.
       */

      if (div >= b32ONE)
        {
          DEBUGASSERT(div < itob32(4096));

          clkdiv = i;
          break;
        }
    }

  /* Then set up the BAUD-related registers */

  DEBUGASSERT(clkdiv >= 0);

  ibaud  = b32toi(div);
  regval = UART_BAUD0_IBAUD(ibaud) | UART_BAUD0_CLKDIV(clkdiv);
  putreg32(regval, base + MAX326_UART_BAUD0_OFFSET);

  regval = b32frac(div) >> (32 - 12);
  putreg32(regval, base + MAX326_UART_BAUD1_OFFSET);
}
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: max326_lowsetup
 *
 * Description:
 *   Called at the very beginning of _start.  Performs low level initialization
 *   including setup of the console UART.  This UART initialization is done
 *   early so that the serial console is available for debugging very early in
 *   the boot sequence.
 *
 ************************************************************************************/

void max326_lowsetup(void)
{
  /* Enable clocking to GPIO0 */

  max326_gpio0_enableclk();

#ifdef HAVE_UART_DEVICE
#ifdef CONFIG_MAX326XX_UART0
  /* Enable clocking to UART0 */

  max326_uart0_enableclk();

  /* Configure UART0 pins */

  max326_gpio_config(GPIO_UART0_RX);
  max326_gpio_config(GPIO_UART0_TX);
#ifdef CONFIG_UART0_OFLOWCONTROL
  max326_gpio_config(GPIO_UART0_CTS);
#endif
#ifdef CONFIG_UART0_IFLOWCONTROL
  max326_gpio_config(GPIO_UART0_RTS);
#endif
#endif /* CONFIG_MAX326XX_UART0 */

#ifdef CONFIG_MAX326XX_UART1
  /* Enable clocking to UART1 */

  max326_uart1_enableclk();

   /* Configure UART1 pins */

  max326_gpio_config(GPIO_UART1_RX);
  max326_gpio_config(GPIO_UART1_TX);
#ifdef CONFIG_UART1_OFLOWCONTROL
  max326_gpio_config(GPIO_UART1_CTS);
#endif
#ifdef CONFIG_UART1_IFLOWCONTROL
  max326_gpio_config(GPIO_UART1_RTS);
#endif
#endif /* CONFIG_MAX326XX_UART1 */

#ifdef HAVE_UART_CONSOLE
  /* Configure the console UART (if any) */

  max326_uart_configure(CONSOLE_BASE, &g_console_config);

#endif /* HAVE_UART_CONSOLE */
#endif /* HAVE_UART_DEVICE */
}

/****************************************************************************
 * Name: max326_uart_configure
 *
 * Description:
 *   Configure a UART for non-interrupt driven operation
 *
 ****************************************************************************/

#ifdef HAVE_UART_DEVICE
void max326_uart_configure(uintptr_t base,
                           FAR const struct uart_config_s *config)
{
  uint32_t regval;
  uint32_t ctrl0;

  /* Disable interrupts */

  putreg32(0, base + MAX326_UART_INTEN_OFFSET);

  /* Configure baud */

  max326_setbaud(CONSOLE_BASE, config);

  /* Configure RX and TX FIFOs */
  /* Flush FIFOs */

  ctrl0  = getreg32(base + MAX326_UART_CTRL0_OFFSET);
  ctrl0 |= (UART_CTRL0_TXFLUSH | UART_CTRL0_RXFLUSH);
  putreg32(ctrl0, base + MAX326_UART_CTRL0_OFFSET);

  /* Wait for the flush to complete */

  do
    {
      ctrl0  = getreg32(base + MAX326_UART_CTRL0_OFFSET);
      regval = ctrl0 & (UART_CTRL0_TXFLUSH | UART_CTRL0_RXFLUSH);
    }
  while (regval != 0);

  /* Setup trigger level */

  regval  = UART_CTRL1_RXFIFOLVL(config->rxlevel) |
            UART_CTRL1_TXFIFOLVL(config->txlevel);
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  regval |= UART_CTRL1_RTSFIFOLVL(config->rtslevel);
#endif
  putreg32(regval, base + MAX326_UART_CTRL1_OFFSET);

  /* Configure the RXFIFO timeout */

  ctrl0 &= ~UART_CTRL0_TOCNT_MASK;
  ctrl0 |= UART_CTRL0_TOCNT(4);
  putreg32(ctrl0, base + MAX326_UART_CTRL0_OFFSET);

  /* Setup configuration and enable UART */

  ctrl0 &= ~UART_CTRL0_SIZE_MASK;
  switch (config->bits)
    {
      case 5:
        ctrl0 |= UART_CTRL0_SIZE_5BITS;
        break;

      case 6:
        ctrl0 |= UART_CTRL0_SIZE_6BITS;
        break;

      case 7:
        ctrl0 |= UART_CTRL0_SIZE_7BITS;
        break;

      default:
      case 8:
        ctrl0 |= UART_CTRL0_SIZE_8BITS;
        break;
    }

  putreg32(ctrl0, base + MAX326_UART_CTRL0_OFFSET);

  ctrl0 &= ~(UART_CTRL0_PARITYEN | UART_CTRL0_PARITYMODE_MASK);
  ctrl0 |= UART_CTRL0_PARITYLVL;
  switch (config->parity)
    {
      default:
      case 0:
        break;

      case 1:
        ctrl0 |= (UART_CTRL0_PARITYEN | UART_CTRL0_PARITY_ODD);
        break;

      case 2:
        ctrl0 |= (UART_CTRL0_PARITYEN | UART_CTRL0_PARITY_EVEN);
        break;
    }

  putreg32(ctrl0, base + MAX326_UART_CTRL0_OFFSET);

  ctrl0 &= ~UART_CTRL0_STOP;
  if (config->stopbits2)
    {
      ctrl0 |= UART_CTRL0_STOP;  /* Only 1.5 stop bits */
    }

  putreg32(ctrl0, base + MAX326_UART_CTRL0_OFFSET);

#if defined(CONFIG_SERIAL_IFLOWCONTROL) || defined(CONFIG_SERIAL_OFLOWCONTROL)
  /* Enable flow control.  REVISIT! */

  ctrl0 |= UART_CTRL0_FLOW;
  putreg32(ctrl0, base + MAX326_UART_CTRL0_OFFSET);
#endif

  /* Enable the UART */

  ctrl0 |= UART_CTRL0_ENABLE;
  putreg32(ctrl0, base + MAX326_UART_CTRL0_OFFSET);
}
#endif

/****************************************************************************
 * Name: max326_uart_disable
 *
 * Description:
 *   Disable a UART.  it will be necessary to again call
 *   max326_uart_configure() in order to use this UART channel again.
 *
 ****************************************************************************/

#ifdef HAVE_UART_DEVICE
void max326_uart_disable(uintptr_t base)
{
  uint32_t regval;

  /* Disable interrupts */

  putreg32(0, base + MAX326_UART_INTEN_OFFSET);

  /* Disable the UART (which disables the FIFOs and baud rate generation) */

  regval  = getreg32(base + MAX326_UART_CTRL0_OFFSET);
  regval &= ~UART_CTRL0_ENABLE;
  putreg32(regval, base + MAX326_UART_CTRL0_OFFSET);
}
#endif

/****************************************************************************
 * Name: arm_lowputc
 *
 * Description:
 *   Output one byte on the serial console
 *
 ****************************************************************************/

void arm_lowputc(char ch)
{
#ifdef HAVE_UART_CONSOLE
  irqstate_t flags;

  for (; ; )
    {
      /* Wait for the transmit FIFO to be not full */

      while ((getreg32(CONSOLE_BASE + MAX326_UART_STAT_OFFSET) &
             UART_STAT_TXFULL) != 0)
        {
        }

      /* Disable interrupts so that the test and the transmission are
       * atomic.
       */

      flags = spin_lock_irqsave();
      if ((getreg32(CONSOLE_BASE + MAX326_UART_STAT_OFFSET) &
           UART_STAT_TXFULL) == 0)
        {
          /* Send the character */

          putreg32((uint32_t)ch, CONSOLE_BASE + MAX326_UART_FIFO_OFFSET);
          spin_unlock_irqrestore(flags);
          return;
        }

      spin_unlock_irqrestore(flags);
    }
#endif
}
