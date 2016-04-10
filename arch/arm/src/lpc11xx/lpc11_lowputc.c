/****************************************************************************
 * arch/arm/src/lpc11xx/lpc11_lowputc.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

#include <arch/irq.h>
#include <arch/board/board.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip/lpc11_syscon.h"
#include "chip/lpc11_uart.h"

#include "lpc11_gpio.h"
#include "lpc11_lowputc.h"
#include "lpc11_serial.h"

/****************************************************************************
 * Private Definitions
 ****************************************************************************/

/* Select UART parameters for the selected console */

#if defined(CONFIG_UART0_SERIAL_CONSOLE)
#  define CONSOLE_BASE     LPC11_UART0_BASE
#  define CONSOLE_BAUD     CONFIG_UART0_BAUD
#  define CONSOLE_BITS     CONFIG_UART0_BITS
#  define CONSOLE_PARITY   CONFIG_UART0_PARITY
#  define CONSOLE_2STOP    CONFIG_UART0_2STOP
#elif defined(HAVE_SERIAL_CONSOLE)
#  error "No CONFIG_UART0_SERIAL_CONSOLE Setting"
#endif

/* Get word length setting for the console */

#if CONSOLE_BITS == 5
#  define CONSOLE_LCR_WLS UART_LCR_WLS_5BIT
#elif CONSOLE_BITS == 6
#  define CONSOLE_LCR_WLS UART_LCR_WLS_6BIT
#elif CONSOLE_BITS == 7
#  define CONSOLE_LCR_WLS UART_LCR_WLS_7BIT
#elif CONSOLE_BITS == 8
#  define CONSOLE_LCR_WLS UART_LCR_WLS_8BIT
#elif defined(HAVE_SERIAL_CONSOLE)
#  error "Invalid CONFIG_UARTn_BITS setting for console "
#endif

/* Get parity setting for the console */

#if CONSOLE_PARITY == 0
#  define CONSOLE_LCR_PAR 0
#elif CONSOLE_PARITY == 1
#  define CONSOLE_LCR_PAR (UART_LCR_PE|UART_LCR_PS_ODD)
#elif CONSOLE_PARITY == 2
#  define CONSOLE_LCR_PAR (UART_LCR_PE|UART_LCR_PS_EVEN)
#elif CONSOLE_PARITY == 3
#  define CONSOLE_LCR_PAR (UART_LCR_PE|UART_LCR_PS_STICK1)
#elif CONSOLE_PARITY == 4
#  define CONSOLE_LCR_PAR (UART_LCR_PE|UART_LCR_PS_STICK0)
#elif defined(HAVE_SERIAL_CONSOLE)
#    error "Invalid CONFIG_UARTn_PARITY setting for CONSOLE"
#endif

/* Get stop-bit setting for the console and UART0-3 */

#if CONSOLE_2STOP != 0
#  define CONSOLE_LCR_STOP UART_LCR_STOP
#else
#  define CONSOLE_LCR_STOP 0
#endif

/* LCR and FCR values for the console */

#define CONSOLE_LCR_VALUE (CONSOLE_LCR_WLS | CONSOLE_LCR_PAR | CONSOLE_LCR_STOP)
#define CONSOLE_FCR_VALUE (UART_FCR_RXTRIGGER_8 | UART_FCR_TXRST |\
                           UART_FCR_RXRST | UART_FCR_FIFOEN)

/****************************************************************************
 * This Baud Rate configuration is based on idea suggested at LPCWare:
 * www.lpcware.com/content/blog/lpc17xx-uart-simpler-way-calculate-baudrate-timming
 *
 * The original code is for LPC17xx but with few modifications it worked
 * fine in the LPC11xx as well.
 *
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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
#if defined HAVE_UART && defined HAVE_SERIAL_CONSOLE
  /* Wait for the transmitter to be available */

  while ((getreg32(CONSOLE_BASE+LPC11_UART_LSR_OFFSET) & UART_LSR_THRE) == 0);

  /* Send the character */

  putreg32((uint32_t)ch, CONSOLE_BASE+LPC11_UART_THR_OFFSET);
#endif
}

/****************************************************************************
 * Name: lpc11_lowsetup
 *
 * Description:
 *   This performs basic initialization of the UART used for the serial
 *   console.  Its purpose is to get the console output available as soon
 *   as possible.
 *
 *   The UART peripheral is configured using the following registers:
 *   1. Pins: For the LPC111x/101/201/301 parts, the UART pins must be
 *      configured in the IOCONFIG register block before the UART clocks can
 *      be enabled in the SYSAHBCLKCTRL register. For all other parts, no
 *      special enabling sequence is required.
 *   2. Power: In the SYSAHBCLKCTRL register, set bit 12.
 *      On reset, UART is disabled.
 *   3. Peripheral clock: Enable the UART peripheral clock by writing to the
 *      UARTCLKDIV register.
 *
 ****************************************************************************/

void lpc11_lowsetup(void)
{
#ifdef HAVE_UART
  uint32_t regval;
  uint32_t coreclk = LPC11_MCLK;
  uint32_t rate16 = 16 * CONSOLE_BAUD;
  uint32_t dval;
  uint32_t mval;
  uint32_t dl;

  /* Enable clock for GPIO and I/O block */

  regval = getreg32(LPC11_SYSCON_SYSAHBCLKCTRL);
  regval |= (SYSCON_SYSAHBCLKCTRL_GPIO | SYSCON_SYSAHBCLKCTRL_IOCON);
  putreg32(regval, LPC11_SYSCON_SYSAHBCLKCTRL);

#if defined(CONFIG_UART0_SERIAL_CONSOLE)
  /* Step 1: Pins configuration */

  lpc11_configgpio(GPIO_UART0_TXD);
  lpc11_configgpio(GPIO_UART0_RXD);
#endif

  /* Step 2: Enable power for all console UART and disable power for
   * other UARTs.
   */

  regval = getreg32(LPC11_SYSCON_SYSAHBCLKCTRL);
#if defined(CONFIG_UART0_SERIAL_CONSOLE)
  regval |= SYSCON_SYSAHBCLKCTRL_UART;
#endif
  putreg32(regval, LPC11_SYSCON_SYSAHBCLKCTRL);

  /* Step 3: Enable peripheral clocking for the console UART and disable
   * clocking for all other UARTs
   */

  /* Don't divide the UART Clock it is be equal to Peripheral Clock */

  putreg32(1, LPC11_SYSCON_UARTCLKDIV);

  /* Configure the console (only) */

#if defined(HAVE_SERIAL_CONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)

  /* Clear fifos */

  putreg32(UART_FCR_RXRST | UART_FCR_TXRST,
          CONSOLE_BASE + LPC11_UART_FCR_OFFSET);

  /* Set trigger */

  putreg32(UART_FCR_FIFOEN | UART_FCR_RXTRIGGER_8,
           CONSOLE_BASE + LPC11_UART_FCR_OFFSET);

  /* Set up the LCR and set DLAB=1 */

  putreg32(CONSOLE_LCR_VALUE | UART_LCR_DLAB,
           CONSOLE_BASE + LPC11_UART_LCR_OFFSET);

  /* Configure the Baud rate
   *
   * The fractional is calculated as
   * (PCLK % (16 * Baudrate)) / (16 * Baudrate)
   */

  dval = coreclk % rate16;

  /* The PCLK / (16 * Baudrate) is fractional
   * dval = pclk % rate16
   * mval = rate16
   * now normalize the ratio
   * dval / mval = 1 / new_mval
   * new_mval = mval / dval;
   * new_dval = 1
   */

  if (dval > 0)
    {
      mval = rate16 / dval;
      dval = 1;

      if (mval > 12)
        {
          dval = 0;
        }
    }

  dval &= 0xf;
  mval &= 0xf;

  dl = coreclk / (rate16 + rate16 * dval / mval);

  /* Set the BAUD divisor */

  putreg32(dl & 0xff, CONSOLE_BASE + LPC11_UART_DLL_OFFSET);
  putreg32(dl >> 8,   CONSOLE_BASE + LPC11_UART_DLM_OFFSET);

  /* Set the BAUD fractional */

  putreg32((mval << UART_FDR_MULVAL_SHIFT) |
           (dval << UART_FDR_DIVADDVAL_SHIFT),
           CONSOLE_BASE + LPC11_UART_FDR_OFFSET);

  /* Clear DLAB */

  putreg32(CONSOLE_LCR_VALUE, CONSOLE_BASE + LPC11_UART_LCR_OFFSET);

  /* Configure the FIFOs */

  putreg32(UART_FCR_RXTRIGGER_8 | UART_FCR_TXRST | UART_FCR_RXRST |
           UART_FCR_FIFOEN,
           CONSOLE_BASE + LPC11_UART_FCR_OFFSET);
#endif
#endif /* HAVE_UART */
}
