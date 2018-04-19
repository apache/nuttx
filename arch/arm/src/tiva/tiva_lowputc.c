/****************************************************************************
 * arch/arm/src/tiva/tiva_lowputc.c
 *
 *   Copyright (C) 2009-2010, 2014, 2018 Gregory Nutt. All rights reserved.
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

#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"

#include "tiva_enablepwr.h"
#include "tiva_enableclks.h"
#include "tiva_periphrdy.h"
#include "tiva_gpio.h"
#include "chip/tiva_pinmap.h"

#include "tiva_lowputc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration **********************************************************/

/* Select UART parameters for the selected console */

#if defined(CONFIG_UART0_SERIAL_CONSOLE)
#  define TIVA_CONSOLE_UART     0
#  define TIVA_CONSOLE_BASE     TIVA_UART0_BASE
#  define TIVA_CONSOLE_BAUD     CONFIG_UART0_BAUD
#  define TIVA_CONSOLE_BITS     CONFIG_UART0_BITS
#  define TIVA_CONSOLE_PARITY   CONFIG_UART0_PARITY
#  define TIVA_CONSOLE_2STOP    CONFIG_UART0_2STOP
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#  define TIVA_CONSOLE_UART     1
#  define TIVA_CONSOLE_BASE     TIVA_UART1_BASE
#  define TIVA_CONSOLE_BAUD     CONFIG_UART1_BAUD
#  define TIVA_CONSOLE_BITS     CONFIG_UART1_BITS
#  define TIVA_CONSOLE_PARITY   CONFIG_UART1_PARITY
#  define TIVA_CONSOLE_2STOP    CONFIG_UART1_2STOP
#elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#  define TIVA_CONSOLE_UART     2
#  define TIVA_CONSOLE_BASE     TIVA_UART2_BASE
#  define TIVA_CONSOLE_BAUD     CONFIG_UART2_BAUD
#  define TIVA_CONSOLE_BITS     CONFIG_UART2_BITS
#  define TIVA_CONSOLE_PARITY   CONFIG_UART2_PARITY
#  define TIVA_CONSOLE_2STOP    CONFIG_UART2_2STOP
#elif defined(CONFIG_UART3_SERIAL_CONSOLE)
#  define TIVA_CONSOLE_UART     3
#  define TIVA_CONSOLE_BASE     TIVA_UART3_BASE
#  define TIVA_CONSOLE_BAUD     CONFIG_UART3_BAUD
#  define TIVA_CONSOLE_BITS     CONFIG_UART3_BITS
#  define TIVA_CONSOLE_PARITY   CONFIG_UART3_PARITY
#  define TIVA_CONSOLE_2STOP    CONFIG_UART3_2STOP
#elif defined(CONFIG_UART4_SERIAL_CONSOLE)
#  define TIVA_CONSOLE_UART     4
#  define TIVA_CONSOLE_BASE     TIVA_UART4_BASE
#  define TIVA_CONSOLE_BAUD     CONFIG_UART4_BAUD
#  define TIVA_CONSOLE_BITS     CONFIG_UART4_BITS
#  define TIVA_CONSOLE_PARITY   CONFIG_UART4_PARITY
#  define TIVA_CONSOLE_2STOP    CONFIG_UART4_2STOP
#elif defined(CONFIG_UART5_SERIAL_CONSOLE)
#  define TIVA_CONSOLE_UART     5
#  define TIVA_CONSOLE_BASE     TIVA_UART5_BASE
#  define TIVA_CONSOLE_BAUD     CONFIG_UART5_BAUD
#  define TIVA_CONSOLE_BITS     CONFIG_UART5_BITS
#  define TIVA_CONSOLE_PARITY   CONFIG_UART5_PARITY
#  define TIVA_CONSOLE_2STOP    CONFIG_UART5_2STOP
#elif defined(CONFIG_UART6_SERIAL_CONSOLE)
#  define TIVA_CONSOLE_UART     6
#  define TIVA_CONSOLE_BASE     TIVA_UART6_BASE
#  define TIVA_CONSOLE_BAUD     CONFIG_UART6_BAUD
#  define TIVA_CONSOLE_BITS     CONFIG_UART6_BITS
#  define TIVA_CONSOLE_PARITY   CONFIG_UART6_PARITY
#  define TIVA_CONSOLE_2STOP    CONFIG_UART6_2STOP
#elif defined(CONFIG_UART7_SERIAL_CONSOLE)
#  define TIVA_CONSOLE_UART     7
#  define TIVA_CONSOLE_BASE     TIVA_UART7_BASE
#  define TIVA_CONSOLE_BAUD     CONFIG_UART7_BAUD
#  define TIVA_CONSOLE_BITS     CONFIG_UART7_BITS
#  define TIVA_CONSOLE_PARITY   CONFIG_UART7_PARITY
#  define TIVA_CONSOLE_2STOP    CONFIG_UART7_2STOP
#elif !defined(CONFIG_NO_SERIAL_CONSOLE)
#  error "No CONFIG_UARTn_SERIAL_CONSOLE Setting"
#endif

/* Get LCRH settings */
#ifndef CONFIG_NO_SERIAL_CONSOLE

#  if TIVA_CONSOLE_BITS == 5
#    define UART_LCRH_NBITS UART_LCRH_WLEN_5BITS
#  elif TIVA_CONSOLE_BITS == 6
#    define UART_LCRH_NBITS UART_LCRH_WLEN_6BITS
#  elif TIVA_CONSOLE_BITS == 7
#    define UART_LCRH_NBITS UART_LCRH_WLEN_7BITS
#  elif TIVA_CONSOLE_BITS == 8
#    define UART_LCRH_NBITS UART_LCRH_WLEN_8BITS
#  else
#    error "Number of bits not supported"
#  endif

#  if TIVA_CONSOLE_PARITY == 0
#    define UART_LCRH_PARITY (0)
#  elif TIVA_CONSOLE_PARITY == 1
#    define UART_LCRH_PARITY UART_LCRH_PEN
#  elif TIVA_CONSOLE_PARITY == 2
#    define UART_LCRH_PARITY (UART_LCRH_PEN|UART_LCRH_EPS)
#  else
#    error "Invalid parity selection"
#  endif

#  if TIVA_CONSOLE_2STOP != 0
#    define UART_LCRH_NSTOP UART_LCRH_STP2
#  else
#    define UART_LCRH_NSTOP (0)
#  endif

#  define UART_LCRH_VALUE (UART_LCRH_NBITS|UART_LCRH_PARITY|UART_LCRH_NSTOP|UART_LCRH_FEN)

#endif /* !CONFIG_NO_SERIAL_CONSOLE */

/* Calculate BAUD rate from the SYS clock:
 *
 * "The baud-rate divisor is a 22-bit number consisting of a 16-bit integer and a 6-bit
 *  fractional part. The number formed by these two values is used by the baud-rate generator
 *  to determine the bit period. Having a fractional baud-rate divider allows the UART to
 *  generate all the standard baud rates.
 *
 * "The 16-bit integer is loaded through the UART Integer Baud-Rate Divisor (UARTIBRD)
 *  register ... and the 6-bit fractional part is loaded with the UART Fractional Baud-Rate
 *  Divisor (UARTFBRD) register... The baud-rate divisor (BRD) has the following relationship
 *  to the system clock (where BRDI is the integer part of the BRD and BRDF is the fractional
 *  part, separated by a decimal place.):
 *
 *    "BRD = BRDI + BRDF = UARTSysClk / (16 * Baud Rate)
 *
 * "where UARTSysClk is the system clock connected to the UART. The 6-bit fractional number
 *  (that is to be loaded into the DIVFRAC bit field in the UARTFBRD register) can be calculated
 *  by taking the fractional part of the baud-rate divisor, multiplying it by 64, and adding 0.5
 *  to account for rounding errors:
 *
 *    "UARTFBRD[DIVFRAC] = integer(BRDF * 64 + 0.5)
 *
 * "The UART generates an internal baud-rate reference clock at 16x the baud-rate (referred
 *  to as Baud16). This reference clock is divided by 16 to generate the transmit clock, and is
 *  used for error detection during receive operations.
 *
 * "Along with the UART Line Control, High Byte (UARTLCRH) register ..., the UARTIBRD and
 *  UARTFBRD registers form an internal 30-bit register. This internal register is only
 *  updated when a write operation to UARTLCRH is performed, so any changes to the baud-rate
 *  divisor must be followed by a write to the UARTLCRH register for the changes to take effect. ..."
 */

#define TIVA_BRDDEN     (16 * TIVA_CONSOLE_BAUD)
#define TIVA_BRDI       (SYSCLK_FREQUENCY / TIVA_BRDDEN)
#define TIVA_REMAINDER  (SYSCLK_FREQUENCY - TIVA_BRDDEN * TIVA_BRDI)
#define TIVA_DIVFRAC    ((TIVA_REMAINDER * 64 + (TIVA_BRDDEN/2)) / TIVA_BRDDEN)

/* For example: TIVA_CONSOLE_BAUD = 115,200, SYSCLK_FREQUENCY = 50,000,000:
 *
 * TIVA_BRDDEN    = (16 * 115,200)                           = 1,843,200
 * TIVA_BRDI      = 50,000,000 / 1,843,200                   = 27
 * TIVA_REMAINDER = 50,000,000 - 1,843,200 * 27              = 233,600
 * TIVA_DIVFRAC   = (233,600 * 64 + 921,600) / 1,843,200     = 8
 *
 * Which should yied BAUD = 50,000,000 / (16 * (27 + 8/64)) = 115207.37
 */

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
#ifdef HAVE_SERIAL_CONSOLE
  /* Wait until the TX FIFO is not full */

  while ((getreg32(TIVA_CONSOLE_BASE + TIVA_UART_FR_OFFSET) & UART_FR_TXFF) != 0);

  /* Then send the character */

  putreg32((uint32_t)ch, TIVA_CONSOLE_BASE + TIVA_UART_DR_OFFSET);
#endif
}

/****************************************************************************
 * Name: up_lowsetup
 *
 * Description:
 *   This performs basic initialization of the UART used for the serial
 *   console.  Its purpose is to get the console output availabe as soon
 *   as possible.
 *
 ****************************************************************************/

void up_lowsetup(void)
{
#if defined(HAVE_SERIAL_CONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)
  uint32_t ctl;
#endif

  /* Enable the selected UARTs and configure GPIO pins needed by the
   * the selected UARTs.  NOTE: The serial driver later depends on
   * this pin configuration -- whether or not a serial console is selected.
   *
   * - Enable Power (TM4C129 family only):  Applies power (only) to the UART
   *   peripheral.  This is not an essential step since enabling clocking
   *   will also apply power.  The only significance is that the UART state
   *   will be retained if the UART clocking is subsequently disabled.
   * - Enable Clocking (All families):  Applies both power and clocking to
   *   the UART peripheral, bringing it a fully functional state.
   */

#ifdef CONFIG_TIVA_UART0
  tiva_uart0_enablepwr();
  tiva_uart0_enableclk();

  tiva_configgpio(GPIO_UART0_RX);
  tiva_configgpio(GPIO_UART0_TX);
#endif

#ifdef CONFIG_TIVA_UART1
  tiva_uart1_enablepwr();
  tiva_uart1_enableclk();

  tiva_configgpio(GPIO_UART1_RX);
  tiva_configgpio(GPIO_UART1_TX);
#endif

#ifdef CONFIG_TIVA_UART2
  tiva_uart2_enablepwr();
  tiva_uart2_enableclk();

  tiva_configgpio(GPIO_UART2_RX);
  tiva_configgpio(GPIO_UART2_TX);
#endif

#ifdef CONFIG_TIVA_UART3
  tiva_uart3_enablepwr();
  tiva_uart3_enableclk();

  tiva_configgpio(GPIO_UART3_RX);
  tiva_configgpio(GPIO_UART3_TX);
#endif

#ifdef CONFIG_TIVA_UART4
  tiva_uart4_enablepwr();
  tiva_uart4_enableclk();

  tiva_configgpio(GPIO_UART4_RX);
  tiva_configgpio(GPIO_UART4_TX);
#endif

#ifdef CONFIG_TIVA_UART5
  tiva_uart5_enablepwr();
  tiva_uart5_enableclk();

  tiva_configgpio(GPIO_UART5_RX);
  tiva_configgpio(GPIO_UART5_TX);
#endif

#ifdef CONFIG_TIVA_UART6
  tiva_uart6_enablepwr();
  tiva_uart6_enableclk();

  tiva_configgpio(GPIO_UART6_RX);
  tiva_configgpio(GPIO_UART6_TX);
#endif

#ifdef CONFIG_TIVA_UART7
  tiva_uart7_enablepwr();
  tiva_uart7_enableclk();

  tiva_configgpio(GPIO_UART7_RX);
  tiva_configgpio(GPIO_UART7_TX);
#endif

  /* Enable the selected console device */

#if defined(HAVE_SERIAL_CONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)
#ifdef TIVA_SYSCON_PRUART
  /* Wait for the console UART to be ready before writing to the UART
   * registers.
   */

  while (!tiva_periphrdy(TIVA_SYSCON_PRUART,
                         SYSCON_PRUART(TIVA_CONSOLE_UART)));
#endif

  /* Disable the UART by clearing the UARTEN bit in the UART CTL register */

  ctl = getreg32(TIVA_CONSOLE_BASE + TIVA_UART_CTL_OFFSET);
  ctl &= ~UART_CTL_UARTEN;
  putreg32(ctl, TIVA_CONSOLE_BASE + TIVA_UART_CTL_OFFSET);

  /* Write the integer portion of the BRD to the UART IBRD register */

  putreg32(TIVA_BRDI, TIVA_CONSOLE_BASE + TIVA_UART_IBRD_OFFSET);

  /* Write the fractional portion of the BRD to the UART FBRD register */

  putreg32(TIVA_DIVFRAC, TIVA_CONSOLE_BASE + TIVA_UART_FBRD_OFFSET);

  /* Write the desired serial parameters to the UART LCRH register */

  putreg32(UART_LCRH_VALUE, TIVA_CONSOLE_BASE + TIVA_UART_LCRH_OFFSET);

  /* Enable the UART by setting the UARTEN bit in the UART CTL register */

  ctl |= (UART_CTL_UARTEN | UART_CTL_TXE | UART_CTL_RXE);
  putreg32(ctl, TIVA_CONSOLE_BASE + TIVA_UART_CTL_OFFSET);
#endif
}
