/************************************************************************************
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

#include "up_arch.h"
#include "up_internal.h"

#include "chip/max326_memorymap.h"
#include "chip/max326_pinmux.h"
#include "chip/max326_uart.h"

#include "max326_config.h"
#include "max326_periphclks.h"
#include "max326_clockconfig.h"
#include "max326_gpio.h"
#include "max326_lowputc.h"

#include <arch/board/board.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#if defined(CONFIG_UART0_SERIAL_CONSOLE)
#  define CONSOLE_BASE        MAX326_UART0_BASE
#  define CONSOLE_BAUD        CONFIG_UART0_BAUD
#  define CONSOLE_PARITY      CONFIG_UART0_PARITY
#  define CONSOLE_BITS        CONFIG_UART0_BITS
#  ifdef CONFIG_UART0_2STOP
#    define CONSOLE_STOPBITS2 true
#  else
#    define CONSOLE_STOPBITS2 false
#  endif
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
#  ifdef CONFIG_UART1_2STOP
#    define CONSOLE_STOPBITS2 true
#  else
#    define CONSOLE_STOPBITS2 false
#  endif
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

/************************************************************************************
 * Private Data
 ************************************************************************************/

#ifdef HAVE_UART_CONSOLE
/* UART console configuration */

static const struct uart_config_s g_console_config=
{
  .baud      = CONSOLE_BAUD,
  .parity    = CONSOLE_PARITY,
  .bits      = CONSOLE_BITS,
  .txlevel   = MAX326_UART_TXFIFO_DEPTH / 2,
  .rxlevel   = MAX326_UART_RXFIFO_DEPTH / 4,
  .stopbits2 = CONSOLE_STOPBITS2,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .iflow     = CONSOLE_IFLOW,
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  .oflow     = CONSOLE_OFLOW,
#endif
};
#endif /* HAVE_UART_CONSOLE */

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Name: max326_setbaud
 *
 * Description:
 *   Configure the UART BAUD.
 *
 ************************************************************************************/

#ifdef HAVE_UART_DEVICE
static void max326_setbaud(uintptr_t base, FAR const struct uart_config_s *config)
{
#warning Missing logic
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

/************************************************************************************
 * Name: max326_uart_configure
 *
 * Description:
 *   Configure a UART for non-interrupt driven operation
 *
 ************************************************************************************/

#ifdef HAVE_UART_DEVICE
void max326_uart_configure(uintptr_t base, FAR const struct uart_config_s *config)
{
  uint32_t regval;

  /* Configure baud */

  max326_setbaud(CONSOLE_BASE, config);

  /* Configure RX and TX FIFOs */
  /* Empty and enable FIFOs */


  /* Setup trigger level */


  /* Enable trigger events */


  /* Setup configuration and enable UART */

  switch (config->bits)
    {
      case 7:
        break;

      default:
      case 8:
        break;

      case 9:
        break;
    }

  switch (config->parity)
    {
      default:
      case 0:
        break;

      case 1:
        break;

      case 2:
        break;
    }

  if (config->stopbits2)
    {
    }

#warning Missing logic
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
  /* Disable interrupts */


  /* Disable the UART */


  /* Disable the FIFOs */

}
#endif

/****************************************************************************
 * Name: up_lowputc
 *
 * Description:
 *   Output one byte on the serial console
 *
 ****************************************************************************/

void up_lowputc(char ch)
{
#ifdef HAVE_UART_CONSOLE
  irqstate_t flags;

  for (; ; )
    {
      /* Wait for the transmit FIFO to be not full */
#warning Missing logic

      /* Disable interrupts so that the test and the transmission are
       * atomic.
       */

      flags = spin_lock_irqsave();
#warning Missing logic
        {
          /* Send the character */

          spin_unlock_irqrestore(flags);
          return;
        }

      spin_unlock_irqrestore(flags);
    }
#endif
}
