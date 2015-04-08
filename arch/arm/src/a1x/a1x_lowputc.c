/**************************************************************************
 * arch/arm/src/a1x/a1x_lowputc.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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
 **************************************************************************/

/**************************************************************************
 * Included Files
 **************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <arch/irq.h>
#include <arch/board/board.h>

#include "up_internal.h"
#include "up_arch.h"

#include "a1x_config.h"
#include "chip/a1x_uart.h"
#include "a1x_pio.h"

/**************************************************************************
 * Pre-processor Definitions
 **************************************************************************/

/* Select UART parameters for the selected console */

#if defined(CONFIG_UART0_SERIAL_CONSOLE)
#  define CONSOLE_BASE     A1X_UART0_VADDR
#  define CONSOLE_BAUD     CONFIG_UART0_BAUD
#  define CONSOLE_BITS     CONFIG_UART0_BITS
#  define CONSOLE_PARITY   CONFIG_UART0_PARITY
#  define CONSOLE_2STOP    CONFIG_UART0_2STOP
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#  define CONSOLE_BASE     A1X_UART1_VADDR
#  define CONSOLE_BAUD     CONFIG_UART1_BAUD
#  define CONSOLE_BITS     CONFIG_UART1_BITS
#  define CONSOLE_PARITY   CONFIG_UART1_PARITY
#  define CONSOLE_2STOP    CONFIG_UART1_2STOP
#elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#  define CONSOLE_BASE     A1X_UART2_VADDR
#  define CONSOLE_BAUD     CONFIG_UART2_BAUD
#  define CONSOLE_BITS     CONFIG_UART2_BITS
#  define CONSOLE_PARITY   CONFIG_UART2_PARITY
#  define CONSOLE_2STOP    CONFIG_UART2_2STOP
#elif defined(CONFIG_UART3_SERIAL_CONSOLE)
#  define CONSOLE_BASE     A1X_UART3_VADDR
#  define CONSOLE_BAUD     CONFIG_UART3_BAUD
#  define CONSOLE_BITS     CONFIG_UART3_BITS
#  define CONSOLE_PARITY   CONFIG_UART3_PARITY
#  define CONSOLE_2STOP    CONFIG_UART3_2STOP
#elif defined(CONFIG_UART4_SERIAL_CONSOLE)
#  define CONSOLE_BASE     A1X_UART4_VADDR
#  define CONSOLE_BAUD     CONFIG_UART4_BAUD
#  define CONSOLE_BITS     CONFIG_UART4_BITS
#  define CONSOLE_PARITY   CONFIG_UART4_PARITY
#  define CONSOLE_2STOP    CONFIG_UART4_2STOP
#elif defined(CONFIG_UART5_SERIAL_CONSOLE)
#  define CONSOLE_BASE     A1X_UART5_VADDR
#  define CONSOLE_BAUD     CONFIG_UART5_BAUD
#  define CONSOLE_BITS     CONFIG_UART5_BITS
#  define CONSOLE_PARITY   CONFIG_UART5_PARITY
#  define CONSOLE_2STOP    CONFIG_UART5_2STOP
#elif defined(CONFIG_UART6_SERIAL_CONSOLE)
#  define CONSOLE_BASE     A1X_UART6_VADDR
#  define CONSOLE_BAUD     CONFIG_UART6_BAUD
#  define CONSOLE_BITS     CONFIG_UART6_BITS
#  define CONSOLE_PARITY   CONFIG_UART6_PARITY
#  define CONSOLE_2STOP    CONFIG_UART6_2STOP
#elif defined(CONFIG_UART7_SERIAL_CONSOLE)
#  define CONSOLE_BASE     A1X_UART7_VADDR
#  define CONSOLE_BAUD     CONFIG_UART7_BAUD
#  define CONSOLE_BITS     CONFIG_UART7_BITS
#  define CONSOLE_PARITY   CONFIG_UART7_PARITY
#  define CONSOLE_2STOP    CONFIG_UART7_2STOP
#elif defined(HAVE_SERIAL_CONSOLE)
#  error "No CONFIG_UARTn_SERIAL_CONSOLE Setting"
#endif

/* Get word length setting for the console */

#if CONSOLE_BITS == 5
#  define CONSOLE_LCR_DLS UART_LCR_DLS_5BITS
#elif CONSOLE_BITS == 6
#  define CONSOLE_LCR_DLS UART_LCR_DLS_6BITS
#elif CONSOLE_BITS == 7
#  define CONSOLE_LCR_DLS UART_LCR_DLS_7BITS
#elif CONSOLE_BITS == 8
#  define CONSOLE_LCR_DLS UART_LCR_DLS_8BITS
#elif defined(HAVE_SERIAL_CONSOLE)
#  error "Invalid CONFIG_UARTn_BITS setting for console "
#endif

/* Get parity setting for the console */

#if CONSOLE_PARITY == 0
#  define CONSOLE_LCR_PAR 0
#elif CONSOLE_PARITY == 1
#  define CONSOLE_LCR_PAR UART_LCR_PEN
#elif CONSOLE_PARITY == 2
#  define CONSOLE_LCR_PAR (UART_LCR_PEN | UART_LCR_EPS)
#elif CONSOLE_PARITY == 3
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

#define CONSOLE_LCR_VALUE (CONSOLE_LCR_DLS | CONSOLE_LCR_PAR | CONSOLE_LCR_STOP)
#define CONSOLE_FCR_VALUE (UART_FCR_RT_HALF | UART_FCR_XFIFOR |\
                           UART_FCR_RFIFOR | UART_FCR_FIFOE)

/* SCLK is the UART input clock.
 *
 * Through experimentation, it has been found that the serial clock is
 * OSC24M
 */

#define A1X_SCLK 24000000

/* The output baud rate is equal to the serial clock (SCLK) frequency divided
 * by sixteen times the value of the baud rate divisor, as follows:
 *
 * baud rate = Fsclk / (16 * divisor).
 */

#define CONSOLE_DL (A1X_SCLK / (CONSOLE_BAUD << 4))

/**************************************************************************
 * Private Types
 **************************************************************************/

/**************************************************************************
 * Private Function Prototypes
 **************************************************************************/

/**************************************************************************
 * Global Variables
 **************************************************************************/

/**************************************************************************
 * Private Variables
 **************************************************************************/

/**************************************************************************
 * Private Functions
 **************************************************************************/

/**************************************************************************
 * Public Functions
 **************************************************************************/

/**************************************************************************
 * Name: up_lowputc
 *
 * Description:
 *   Output one byte on the serial console
 *
 **************************************************************************/

void up_lowputc(char ch)
{
#if defined HAVE_UART_DEVICE && defined HAVE_SERIAL_CONSOLE
  /* Wait for the transmitter to be available */

  while ((getreg32(CONSOLE_BASE+A1X_UART_LSR_OFFSET) & UART_LSR_THRE) == 0);

  /* Send the character */

  putreg32((uint32_t)ch, CONSOLE_BASE+A1X_UART_THR_OFFSET);
#endif
}

/**************************************************************************
 * Name: a1x_lowsetup
 *
 * Description:
 *   This performs basic initialization of the UART used for the serial
 *   console.  Its purpose is to get the console output available as soon
 *   as possible.
 *
 **************************************************************************/

void a1x_lowsetup(void)
{
#ifdef HAVE_UART_DEVICE
  /* Enable power and clocking to the UART peripheral */
#warning Missing logic

  /* Configure UART pins for the selected CONSOLE.  If there are multiple
   * pin options for a given UART, the the applicable option must be
   * disambiguated in the board.h header file.
   */

#if defined(CONFIG_UART0_SERIAL_CONSOLE)
  a1x_pio_config(PIO_UART0_TX);
  a1x_pio_config(PIO_UART0_RX);
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
  a1x_pio_config(PIO_UART1_TX);
  a1x_pio_config(PIO_UART1_RX);
#elif defined(CONFIG_UART2_SERIAL_CONSOLE)
  a1x_pio_config(PIO_UART2_TX);
  a1x_pio_config(PIO_UART2_RX);
#elif defined(CONFIG_UART3_SERIAL_CONSOLE)
  a1x_pio_config(PIO_UART3_TX);
  a1x_pio_config(PIO_UART3_RX);
#elif defined(CONFIG_UART4_SERIAL_CONSOLE)
  a1x_pio_config(PIO_UART4_TX);
  a1x_pio_config(PIO_UART4_RX);
#elif defined(CONFIG_UART5_SERIAL_CONSOLE)
  a1x_pio_config(PIO_UART5_TX);
  a1x_pio_config(PIO_UART5_RX);
#elif defined(CONFIG_UART6_SERIAL_CONSOLE)
  a1x_pio_config(PIO_UART6_TX);
  a1x_pio_config(PIO_UART6_RX);
#elif defined(CONFIG_UART7_SERIAL_CONSOLE)
  a1x_pio_config(PIO_UART7_TX);
  a1x_pio_config(PIO_UART7_RX);
#endif

  /* Configure the console (only) */

#if defined(HAVE_SERIAL_CONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)

  /* Clear fifos */

  putreg32(UART_FCR_RFIFOR|UART_FCR_XFIFOR, CONSOLE_BASE+A1X_UART_FCR_OFFSET);

  /* Set trigger */

  putreg32(UART_FCR_FIFOE|UART_FCR_RT_HALF, CONSOLE_BASE+A1X_UART_FCR_OFFSET);

  /* Set up the LCR and set DLAB=1 */

  putreg32(CONSOLE_LCR_VALUE|UART_LCR_DLAB, CONSOLE_BASE+A1X_UART_LCR_OFFSET);

  /* Set the BAUD divisor */

  putreg32(CONSOLE_DL >> 8, CONSOLE_BASE+A1X_UART_DLH_OFFSET);
  putreg32(CONSOLE_DL & 0xff, CONSOLE_BASE+A1X_UART_DLL_OFFSET);

  /* Clear DLAB */

  putreg32(CONSOLE_LCR_VALUE, CONSOLE_BASE+A1X_UART_LCR_OFFSET);

  /* Configure the FIFOs */

  putreg32(UART_FCR_RT_HALF|UART_FCR_XFIFOR|UART_FCR_RFIFOR|UART_FCR_FIFOE,
           CONSOLE_BASE+A1X_UART_FCR_OFFSET);
#endif
#endif /* HAVE_UART_DEVICE */
}
