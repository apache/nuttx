/****************************************************************************
 * arch/arm/src/kinetis/kinetis_lowputc.c
 *
 *   Copyright (C) 2011, 2017-2018 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane<david_s5@nscdg.com>
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
#include <stdbool.h>

#include <arch/irq.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "arm_arch.h"

#include "kinetis_config.h"
#include "kinetis.h"
#include "hardware/kinetis_uart.h"
#include "hardware/kinetis_lpuart.h"
#include "hardware/kinetis_sim.h"
#include "hardware/kinetis_pinmux.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Default hardware flow control */

#if !defined(CONFIG_UART0_IFLOWCONTROL)
#  define CONFIG_UART0_IFLOWCONTROL 0
#endif
#if !defined(CONFIG_UART0_OFLOWCONTROL)
#  define CONFIG_UART0_OFLOWCONTROL 0
#endif
#if !defined(CONFIG_UART1_IFLOWCONTROL)
#  define CONFIG_UART1_IFLOWCONTROL 0
#endif
#if !defined(CONFIG_UART1_OFLOWCONTROL)
#  define CONFIG_UART1_OFLOWCONTROL 0
#endif
#if !defined(CONFIG_UART2_IFLOWCONTROL)
#  define CONFIG_UART2_IFLOWCONTROL 0
#endif
#if !defined(CONFIG_UART2_OFLOWCONTROL)
#  define CONFIG_UART2_OFLOWCONTROL 0
#endif
#if !defined(CONFIG_UART3_IFLOWCONTROL)
#  define CONFIG_UART3_IFLOWCONTROL 0
#endif
#if !defined(CONFIG_UART3_OFLOWCONTROL)
#  define CONFIG_UART3_OFLOWCONTROL 0
#endif
#if !defined(CONFIG_UART4_IFLOWCONTROL)
#  define CONFIG_UART4_IFLOWCONTROL 0
#endif
#if !defined(CONFIG_UART4_OFLOWCONTROL)
#  define CONFIG_UART4_OFLOWCONTROL 0
#endif
#if !defined(CONFIG_UART5_IFLOWCONTROL)
#  define CONFIG_UART5_IFLOWCONTROL 0
#endif
#if !defined(CONFIG_UART5_OFLOWCONTROL)
#  define CONFIG_UART5_OFLOWCONTROL 0
#endif

#if !defined(CONFIG_LPUART0_IFLOWCONTROL)
#  define CONFIG_LPUART0_IFLOWCONTROL 0
#endif
#if !defined(CONFIG_LPUART0_OFLOWCONTROL)
#  define CONFIG_LPUART0_OFLOWCONTROL 0
#endif
#if !defined(CONFIG_LPUART1_IFLOWCONTROL)
#  define CONFIG_LPUART1_IFLOWCONTROL 0
#endif
#if !defined(CONFIG_LPUART1_OFLOWCONTROL)
#  define CONFIG_LPUART1_OFLOWCONTROL 0
#endif
#if !defined(CONFIG_LPUART2_IFLOWCONTROL)
#  define CONFIG_LPUART2_IFLOWCONTROL 0
#endif
#if !defined(CONFIG_LPUART2_OFLOWCONTROL)
#  define CONFIG_LPUART2_OFLOWCONTROL 0
#endif
#if !defined(CONFIG_LPUART3_IFLOWCONTROL)
#  define CONFIG_LPUART3_IFLOWCONTROL 0
#endif
#if !defined(CONFIG_LPUART3_OFLOWCONTROL)
#  define CONFIG_LPUART3_OFLOWCONTROL 0
#endif
#if !defined(CONFIG_LPUART4_IFLOWCONTROL)
#  define CONFIG_LPUART4_IFLOWCONTROL 0
#endif
#if !defined(CONFIG_LPUART4_OFLOWCONTROL)
#  define CONFIG_LPUART4_OFLOWCONTROL 0
#endif

/* Select UART parameters for the selected console */

#if defined(HAVE_UART_CONSOLE)
#  if defined(CONFIG_UART0_SERIAL_CONSOLE)
#    define CONSOLE_BASE     KINETIS_UART0_BASE
#    define CONSOLE_FREQ     BOARD_CORECLK_FREQ
#    define CONSOLE_BAUD     CONFIG_UART0_BAUD
#    define CONSOLE_BITS     CONFIG_UART0_BITS
#    define CONSOLE_2STOP    CONFIG_UART0_2STOP
#    define CONSOLE_PARITY   CONFIG_UART0_PARITY
#    define CONSOLE_IFLOW    CONFIG_UART0_IFLOWCONTROL
#    define CONSOLE_OFLOW    CONFIG_UART0_OFLOWCONTROL
#  elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#    define CONSOLE_BASE     KINETIS_UART1_BASE
#    define CONSOLE_FREQ     BOARD_CORECLK_FREQ
#    define CONSOLE_BAUD     CONFIG_UART1_BAUD
#    define CONSOLE_BITS     CONFIG_UART1_BITS
#    define CONSOLE_2STOP    CONFIG_UART1_2STOP
#    define CONSOLE_PARITY   CONFIG_UART1_PARITY
#    define CONSOLE_IFLOW    CONFIG_UART1_IFLOWCONTROL
#    define CONSOLE_OFLOW    CONFIG_UART1_OFLOWCONTROL
#  elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#    define CONSOLE_BASE     KINETIS_UART2_BASE
#    define CONSOLE_FREQ     BOARD_BUS_FREQ
#    define CONSOLE_BAUD     CONFIG_UART2_BAUD
#    define CONSOLE_BITS     CONFIG_UART2_BITS
#    define CONSOLE_2STOP    CONFIG_UART2_2STOP
#    define CONSOLE_PARITY   CONFIG_UART2_PARITY
#    define CONSOLE_IFLOW    CONFIG_UART2_IFLOWCONTROL
#    define CONSOLE_OFLOW    CONFIG_UART2_OFLOWCONTROL
#  elif defined(CONFIG_UART3_SERIAL_CONSOLE)
#    define CONSOLE_BASE     KINETIS_UART3_BASE
#    define CONSOLE_FREQ     BOARD_BUS_FREQ
#    define CONSOLE_BAUD     CONFIG_UART3_BAUD
#    define CONSOLE_BITS     CONFIG_UART3_BITS
#    define CONSOLE_2STOP    CONFIG_UART3_2STOP
#    define CONSOLE_PARITY   CONFIG_UART3_PARITY
#    define CONSOLE_IFLOW    CONFIG_UART3_IFLOWCONTROL
#    define CONSOLE_OFLOW    CONFIG_UART3_OFLOWCONTROL
#  elif defined(CONFIG_UART4_SERIAL_CONSOLE)
#    define CONSOLE_BASE     KINETIS_UART4_BASE
#    define CONSOLE_FREQ     BOARD_BUS_FREQ
#    define CONSOLE_BAUD     CONFIG_UART4_BAUD
#    define CONSOLE_BITS     CONFIG_UART4_BITS
#    define CONSOLE_2STOP    CONFIG_UART4_2STOP
#    define CONSOLE_PARITY   CONFIG_UART4_PARITY
#    define CONSOLE_IFLOW    CONFIG_UART4_IFLOWCONTROL
#    define CONSOLE_OFLOW    CONFIG_UART4_OFLOWCONTROL
#  elif defined(CONFIG_UART5_SERIAL_CONSOLE)
#    define CONSOLE_BASE     KINETIS_UART5_BASE
#    define CONSOLE_FREQ     BOARD_BUS_FREQ
#    define CONSOLE_BAUD     CONFIG_UART5_BAUD
#    define CONSOLE_BITS     CONFIG_UART5_BITS
#    define CONSOLE_2STOP    CONFIG_UART5_2STOP
#    define CONSOLE_PARITY   CONFIG_UART5_PARITY
#    define CONSOLE_IFLOW    CONFIG_UART5_IFLOWCONTROL
#    define CONSOLE_OFLOW    CONFIG_UART5_OFLOWCONTROL
#  elif defined(HAVE_UART_CONSOLE)
#    error "No CONFIG_UARTn_SERIAL_CONSOLE Setting"
#  endif
#elif defined(HAVE_LPUART_CONSOLE)
#  if defined(CONFIG_LPUART0_SERIAL_CONSOLE)
#    define CONSOLE_BASE   KINETIS_LPUART0_BASE
#    define CONSOLE_FREQ   BOARD_LPUART0_FREQ
#    define CONSOLE_BAUD   CONFIG_LPUART0_BAUD
#    define CONSOLE_PARITY CONFIG_LPUART0_PARITY
#    define CONSOLE_BITS   CONFIG_LPUART0_BITS
#    define CONSOLE_2STOP  CONFIG_LPUART0_2STOP
#    define CONSOLE_IFLOW  CONFIG_LPUART0_IFLOWCONTROL
#    define CONSOLE_OFLOW  CONFIG_LPUART0_OFLOWCONTROL
#  elif defined(CONFIG_LPUART1_SERIAL_CONSOLE)
#    define CONSOLE_BASE   KINETIS_LPUART1_BASE
#    define CONSOLE_FREQ   BOARD_LPUART1_FREQ
#    define CONSOLE_BAUD   CONFIG_LPUART1_BAUD
#    define CONSOLE_PARITY CONFIG_LPUART1_PARITY
#    define CONSOLE_BITS   CONFIG_LPUART1_BITS
#    define CONSOLE_2STOP  CONFIG_LPUART1_2STOP
#    define CONSOLE_IFLOW  CONFIG_LPUART1_IFLOWCONTROL
#    define CONSOLE_OFLOW  CONFIG_LPUART1_OFLOWCONTROL
#  elif defined(CONFIG_LPUART2_SERIAL_CONSOLE)
#    define CONSOLE_BASE   KINETIS_LPUART2_BASE
#    define CONSOLE_FREQ   BOARD_LPUART2_FREQ
#    define CONSOLE_BAUD   CONFIG_LPUART2_BAUD
#    define CONSOLE_PARITY CONFIG_LPUART2_PARITY
#    define CONSOLE_BITS   CONFIG_LPUART2_BITS
#    define CONSOLE_2STOP  CONFIG_LPUART2_2STOP
#    define CONSOLE_IFLOW  CONFIG_LPUART2_IFLOWCONTROL
#    define CONSOLE_OFLOW  CONFIG_LPUART2_OFLOWCONTROL
#  elif defined(CONFIG_LPUART3_SERIAL_CONSOLE)
#    define CONSOLE_BASE   KINETIS_LPUART3_BASE
#    define CONSOLE_FREQ   BOARD_LPUART3_FREQ
#    define CONSOLE_BAUD   CONFIG_LPUART3_BAUD
#    define CONSOLE_PARITY CONFIG_LPUART3_PARITY
#    define CONSOLE_BITS   CONFIG_LPUART3_BITS
#    define CONSOLE_2STOP  CONFIG_LPUART3_2STOP
#    define CONSOLE_IFLOW  CONFIG_LPUART3_IFLOWCONTROL
#    define CONSOLE_OFLOW  CONFIG_LPUART3_OFLOWCONTROL
#  elif defined(CONFIG_LPUART4_SERIAL_CONSOLE)
#    define CONSOLE_BASE   KINETIS_LPUART4_BASE
#    define CONSOLE_FREQ   BOARD_LPUART4_FREQ
#    define CONSOLE_BAUD   CONFIG_LPUART4_BAUD
#    define CONSOLE_PARITY CONFIG_LPUART4_PARITY
#    define CONSOLE_BITS   CONFIG_LPUART4_BITS
#    define CONSOLE_2STOP  CONFIG_LPUART4_2STOP
#    define CONSOLE_IFLOW  CONFIG_LPUART4_IFLOWCONTROL
#    define CONSOLE_OFLOW  CONFIG_LPUART4_OFLOWCONTROL
#  else
#    error "No LPUART console is selected"
#  endif
#endif /* HAVE_UART_CONSOLE */

#if defined(HAVE_LPUART_CONSOLE)
#  if ((CONSOLE_FREQ / (CONSOLE_BAUD * 32)) > (LPUART_BAUD_SBR_MASK >> LPUART_BAUD_SBR_SHIFT))
#    error "LPUART Console: Baud rate not obtainable with this input clock!"
#  endif
#endif
#ifdef HAVE_LPUART_DEVICE
#  define LPUART_BAUD_INIT (LPUART_BAUD_SBR_MASK | LPUART_BAUD_SBNS | \
                            LPUART_BAUD_RXEDGIE | LPUART_BAUD_LBKDIE | \
                            LPUART_BAUD_RESYNCDIS |LPUART_BAUD_BOTHEDGE | \
                            LPUART_BAUD_MATCFG_MASK | LPUART_BAUD_RDMAE | \
                            LPUART_BAUD_TDMAE | LPUART_BAUD_OSR_MASK | \
                            LPUART_BAUD_M10 | LPUART_BAUD_MAEN2 | \
                            LPUART_BAUD_MAEN2)
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This array maps an encoded FIFO depth (index) to the actual size of the
 * FIFO (indexed value).  NOTE:  That there is no 8th value.
 */

#ifdef CONFIG_KINETIS_UARTFIFOS
static uint8_t g_sizemap[8] =
{
  1, 4, 8, 16, 32, 64, 128, 0
};
#endif

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
#if defined(HAVE_UART_CONSOLE)
#  ifdef CONFIG_KINETIS_UARTFIFOS
  /* Wait until there is space in the TX FIFO:  Read the number of bytes
   * currently in the FIFO and compare that to the size of the FIFO.  If
   * there are fewer bytes in the FIFO than the size of the FIFO, then we
   * are able to transmit.
   */

#    error "Missing logic"
#  else
  /* Wait until the transmit data register is "empty" (TDRE).  This state
   * depends on the TX watermark setting and may not mean that the transmit
   * buffer is truly empty.  It just means that we can now add another
   * character to the transmit buffer without exceeding the watermark.
   *
   * NOTE:  UART0 has an 8-byte deep FIFO; the other UARTs have no FIFOs
   * (1-deep).  There appears to be no way to know when the FIFO is not
   * full (other than reading the FIFO length and comparing the FIFO count).
   * Hence, the FIFOs are not used in this implementation and, as a result
   * TDRE indeed mean that the single output buffer is available.
   *
   * Performance on UART0 could be improved by enabling the FIFO and by
   * redesigning all of the FIFO status logic.
   */

  while (
    (getreg8(CONSOLE_BASE + KINETIS_UART_S1_OFFSET) & UART_S1_TDRE) == 0);
#  endif

  /* Then write the character to the UART data register */

  putreg8((uint8_t)ch, CONSOLE_BASE + KINETIS_UART_D_OFFSET);

#elif defined(HAVE_LPUART_CONSOLE)
  while ((getreg32(CONSOLE_BASE + KINETIS_LPUART_STAT_OFFSET) &
         LPUART_STAT_TDRE) == 0);

  /* Then send the character */

  putreg32((uint32_t)ch, CONSOLE_BASE + KINETIS_LPUART_DATA_OFFSET);
#endif
}

/****************************************************************************
 * Name: kinetis_lowsetup
 *
 * Description:
 *   This performs basic initialization of the UART used for the serial
 *   console.  Its purpose is to get the console output available as soon
 *   as possible.
 *
 ****************************************************************************/

void kinetis_lowsetup(void)
{
#if defined(HAVE_UART_DEVICE) || defined(HAVE_LPUART_DEVICE)
  uint32_t regval;
#endif

#ifdef HAVE_UART_DEVICE
  /* Enable peripheral clocking for all enabled UARTs.  Clocking for UARTs
   * 0-3 is enabled in the SCGC4 register.
   */

#  if defined(CONFIG_KINETIS_UART0) || defined(CONFIG_KINETIS_UART1) || \
      defined(CONFIG_KINETIS_UART2) || defined(CONFIG_KINETIS_UART3)

  regval = getreg32(KINETIS_SIM_SCGC4);
#    ifdef CONFIG_KINETIS_UART0
  regval |= SIM_SCGC4_UART0;
#    endif
#    ifdef CONFIG_KINETIS_UART1
  regval |= SIM_SCGC4_UART1;
#    endif
#    ifdef CONFIG_KINETIS_UART2
  regval |= SIM_SCGC4_UART2;
#    endif
#    ifdef CONFIG_KINETIS_UART3
  regval |= SIM_SCGC4_UART3;
#    endif
  putreg32(regval, KINETIS_SIM_SCGC4);

#  endif

  /* Clocking for UARTs 4-5 is enabled in the SCGC1 register. */

#  if defined(CONFIG_KINETIS_UART4) || defined(CONFIG_KINETIS_UART5)

  regval = getreg32(KINETIS_SIM_SCGC1);
#    ifdef CONFIG_KINETIS_UART4
  regval |= SIM_SCGC1_UART4;
#    endif
#    ifdef CONFIG_KINETIS_UART5
  regval |= SIM_SCGC1_UART5;
#    endif
  putreg32(regval, KINETIS_SIM_SCGC1);

#  endif

  /* Configure UART pins for the all enabled UARTs */

#  ifdef CONFIG_KINETIS_UART0
  kinetis_pinconfig(PIN_UART0_TX);
  kinetis_pinconfig(PIN_UART0_RX);
#    if CONFIG_UART0_IFLOWCONTROL == 1
  kinetis_pinconfig(PIN_UART0_RTS);
#    endif
#    if CONFIG_UART0_OFLOWCONTROL == 1
  kinetis_pinconfig(PIN_UART0_CTS);
#    endif
#  endif
#  ifdef CONFIG_KINETIS_UART1
  kinetis_pinconfig(PIN_UART1_TX);
  kinetis_pinconfig(PIN_UART1_RX);
#    if CONFIG_UART1_IFLOWCONTROL == 1
  kinetis_pinconfig(PIN_UART1_RTS);
#    endif
#    if CONFIG_UART1_OFLOWCONTROL == 1
  kinetis_pinconfig(PIN_UART1_CTS);
#    endif
#  endif
#  ifdef CONFIG_KINETIS_UART2
  kinetis_pinconfig(PIN_UART2_TX);
  kinetis_pinconfig(PIN_UART2_RX);
#    if CONFIG_UART2_IFLOWCONTROL == 1
  kinetis_pinconfig(PIN_UART2_RTS);
#    endif
#    if CONFIG_UART2_OFLOWCONTROL == 1
  kinetis_pinconfig(PIN_UART2_CTS);
#    endif
#  endif
#  ifdef CONFIG_KINETIS_UART3
  kinetis_pinconfig(PIN_UART3_TX);
  kinetis_pinconfig(PIN_UART3_RX);
#    if CONFIG_UART3_IFLOWCONTROL == 1
  kinetis_pinconfig(PIN_UART3_RTS);
#    endif
#    if CONFIG_UART3_OFLOWCONTROL == 1
  kinetis_pinconfig(PIN_UART3_CTS);
#    endif
#  endif
#  ifdef CONFIG_KINETIS_UART4
  kinetis_pinconfig(PIN_UART4_TX);
  kinetis_pinconfig(PIN_UART4_RX);
#    if CONFIG_UART4_IFLOWCONTROL == 1
  kinetis_pinconfig(PIN_UART4_RTS);
#    endif
#    if CONFIG_UART4_OFLOWCONTROL == 1
  kinetis_pinconfig(PIN_UART4_CTS);
#    endif
#  endif
#  ifdef CONFIG_KINETIS_UART5
  kinetis_pinconfig(PIN_UART5_TX);
  kinetis_pinconfig(PIN_UART5_RX);
#    if CONFIG_UART5_IFLOWCONTROL == 1
  kinetis_pinconfig(PIN_UART5_RTS);
#    endif
#    if CONFIG_UART5_OFLOWCONTROL == 1
  kinetis_pinconfig(PIN_UART5_CTS);
#    endif
#  endif

  /* Configure the console (only) now.  Other UARTs will be configured
   * when the serial driver is opened.
   */

#  if defined(HAVE_UART_CONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)

  kinetis_uartconfigure(CONSOLE_BASE, CONSOLE_BAUD, CONSOLE_FREQ, \
                        CONSOLE_PARITY, CONSOLE_BITS, CONSOLE_2STOP, \
                        CONSOLE_IFLOW, CONSOLE_OFLOW);
#  endif
#endif /* HAVE_UART_DEVICE */

#ifdef HAVE_LPUART_DEVICE
  /* Clocking Source for LPUART0 selected in SIM_SOPT2 */

#if defined(CONFIG_KINETIS_LPUART0)
  regval  = getreg32(KINETIS_SIM_SOPT2);
  regval &= ~(SIM_SOPT2_LPUARTSRC_MASK);
  regval |= BOARD_LPUART0_CLKSRC;
  putreg32(regval, KINETIS_SIM_SOPT2);

  /* Clocking for LPUARTs 0-1 is enabled in the SCGC2 register. */

  regval  = getreg32(KINETIS_SIM_SCGC2);
  regval |= SIM_SCGC2_LPUART0;
  putreg32(regval, KINETIS_SIM_SCGC2);
#endif

#if defined(CONFIG_KINETIS_LPUART1)
#  warning REVISIT
#endif
#if defined(CONFIG_KINETIS_LPUART2)
#  warning REVISIT
#endif
#if defined(CONFIG_KINETIS_LPUART3)
#  warning REVISIT
#endif
#if defined(CONFIG_KINETIS_LPUART4)
#  warning REVISIT
#endif

  /* Configure UART pins for the all enabled UARTs */

#ifdef CONFIG_KINETIS_LPUART0
  kinetis_pinconfig(PIN_LPUART0_TX);
  kinetis_pinconfig(PIN_LPUART0_RX);
#if CONFIG_LPUART0_IFLOWCONTROL == 1
  kinetis_pinconfig(PIN_LPUART0_RTS);
#endif
#if CONFIG_LPUART0_OFLOWCONTROL == 1
  kinetis_pinconfig(PIN_LOUART0_CTS);
#endif
#endif

#ifdef CONFIG_KINETIS_LPUART1
  kinetis_pinconfig(PIN_LPUART1_TX);
  kinetis_pinconfig(PIN_LPUART1_RX);
#if CONFIG_LPUART1_IFLOWCONTROL == 1
  kinetis_pinconfig(PIN_LPUART1_RTS);
#endif
#if CONFIG_LPUART1_OFLOWCONTROL == 1
  kinetis_pinconfig(PIN_LOUART1_CTS);
#endif
#endif

#ifdef CONFIG_KINETIS_LPUART2
  kinetis_pinconfig(PIN_LPUART2_TX);
  kinetis_pinconfig(PIN_LPUART2_RX);
#if CONFIG_LPUART2_IFLOWCONTROL == 1
  kinetis_pinconfig(PIN_LPUART2_RTS);
#endif
#if CONFIG_LPUART2_OFLOWCONTROL == 1
  kinetis_pinconfig(PIN_LOUART2_CTS);
#endif
#endif

#ifdef CONFIG_KINETIS_LPUART3
  kinetis_pinconfig(PIN_LPUART3_TX);
  kinetis_pinconfig(PIN_LPUART3_RX);
#if CONFIG_LPUART3_IFLOWCONTROL == 1
  kinetis_pinconfig(PIN_LPUART3_RTS);
#endif
#if CONFIG_LPUART3_OFLOWCONTROL == 1
  kinetis_pinconfig(PIN_LOUART3_CTS);
#endif
#endif

#ifdef CONFIG_KINETIS_LPUART4
  kinetis_pinconfig(PIN_LPUART4_TX);
  kinetis_pinconfig(PIN_LPUART4_RX);
#if CONFIG_LPUART4_IFLOWCONTROL == 1
  kinetis_pinconfig(PIN_LPUART4_RTS);
#endif
#if CONFIG_LPUART4_OFLOWCONTROL == 1
  kinetis_pinconfig(PIN_LOUART4_CTS);
#endif
#endif

#if defined(HAVE_LPUART_CONSOLE) && !defined(CONFIG_SUPPRESS_LPUART_CONFIG)

  kinetis_lpuartconfigure(CONSOLE_BASE, CONSOLE_BAUD, CONSOLE_FREQ, \
                          CONSOLE_PARITY, CONSOLE_BITS, CONSOLE_2STOP, \
                          CONSOLE_IFLOW, CONSOLE_OFLOW);
#endif
#endif /* HAVE_LPUART_DEVICE */
}

/****************************************************************************
 * Name: kinetis_uartreset
 *
 * Description:
 *   Reset a UART.
 *
 ****************************************************************************/

#ifdef HAVE_UART_DEVICE
void kinetis_uartreset(uintptr_t uart_base)
{
  uint8_t regval;

  /* Just disable the transmitter and receiver */

  regval = getreg8(uart_base + KINETIS_UART_C2_OFFSET);
  regval &= ~(UART_C2_RE | UART_C2_TE);
  putreg8(regval, uart_base + KINETIS_UART_C2_OFFSET);
}
#endif

/****************************************************************************
 * Name: kinetis_lpuartreset
 *
 * Description:
 *   Reset a UART.
 *
 ****************************************************************************/

#ifdef HAVE_LPUART_DEVICE
void kinetis_lpuartreset(uintptr_t uart_base)
{
  uint32_t regval;

  /* Just disable the transmitter and receiver */

  regval = getreg32(uart_base + KINETIS_LPUART_CTRL_OFFSET);
  regval &= ~(LPUART_CTRL_RE | LPUART_CTRL_TE);
  putreg32(regval, uart_base + KINETIS_LPUART_CTRL_OFFSET);
}
#endif

/****************************************************************************
 * Name: kinetis_uartconfigure
 *
 * Description:
 *   Configure a UART as a RS-232 UART.
 *
 ****************************************************************************/

#ifdef HAVE_UART_DEVICE
void kinetis_uartconfigure(uintptr_t uart_base, uint32_t baud,
                           uint32_t clock, unsigned int parity,
                           unsigned int nbits, unsigned int stop2,
                           bool iflow, bool oflow)
{
  uint32_t     sbr;
  uint32_t     brfa;
  uint32_t     tmp;
  uint8_t      regval;
#ifdef CONFIG_KINETIS_UARTFIFOS
  unsigned int depth;
#endif

  /* Disable the transmitter and receiver throughout the reconfiguration */

  regval = getreg8(uart_base + KINETIS_UART_C2_OFFSET);
  regval &= ~(UART_C2_RE | UART_C2_TE);
  putreg8(regval, uart_base + KINETIS_UART_C2_OFFSET);

  /* Configure number of bits, stop bits and parity */

  regval = 0;

  /* Check for odd parity */

  if (parity == 1)
    {
      regval |= (UART_C1_PE | UART_C1_PT); /* Enable + odd parity type */
    }

  /* Check for even parity */

  else if (parity == 2)
    {
      regval |= UART_C1_PE;              /* Enable (even parity default) */
    }

  /* The only other option is no parity */

  else
    {
      DEBUGASSERT(parity == 0);
    }

  /* Check for 9-bit operation or enter 9 bit mode for 8 bit with parity
   * see K66 Sub-Family Reference Manual, Rev. 2, May 2015
   * 59.5.4 Data format
   */

  if (nbits == 9 || (nbits == 8 && parity != 0))
    {
      regval |= UART_C1_M;
    }

  /* The only other option is 8-bit operation */

  else
    {
      DEBUGASSERT(nbits == 8);
    }

  putreg8(regval, uart_base + KINETIS_UART_C1_OFFSET);

  /* Calculate baud settings (truncating) */

  sbr = clock / (baud << 4);
  DEBUGASSERT(sbr < 0x2000);

  /* Save the new baud divisor and stop bits, retaining other bits in the
   * UARTx_BDH register.
   */

  regval  = getreg8(uart_base + KINETIS_UART_BDH_OFFSET);
  regval  &= ~(UART_BDH_SBR_MASK | UART_BDH_SBNS);
  if (stop2)
    {
      regval |= UART_BDH_SBNS;
    }

  tmp     = sbr >> 8;
  regval |= (((uint8_t)tmp) << UART_BDH_SBR_SHIFT) & UART_BDH_SBR_MASK;
  putreg8(regval, uart_base + KINETIS_UART_BDH_OFFSET);

  regval  = sbr & 0xff;
  putreg8(regval, uart_base + KINETIS_UART_BDL_OFFSET);

  /* Calculate a fractional divider to get closer to the requested baud.
   * The fractional divider, BRFA, is a 5 bit fractional value that is
   * logically added to the SBR:
   *
   *   UART baud rate = clock / (16 * (SBR + BRFD))
   *
   * The BRFA the remainder.  This will be a non-negative value since the SBR
   * was calculated by truncation.
   */

  tmp  = clock - (sbr * (baud << 4));
  brfa = (tmp << 5) / (baud << 4);

  /* Set the BRFA field (retaining other bits in the UARTx_C4 register) */

  regval  = getreg8(uart_base + KINETIS_UART_C4_OFFSET) & ~UART_C4_BRFA_MASK;
  regval |= ((uint8_t)brfa << UART_C4_BRFA_SHIFT) & UART_C4_BRFA_MASK;
  putreg8(regval, uart_base + KINETIS_UART_C4_OFFSET);

  /* Set the FIFO watermarks.
   *
   * NOTE:  UART0 has an 8-byte deep FIFO; the other UARTs have no FIFOs
   * (1-deep).  There appears to be no way to know when the FIFO is not
   * full (other than reading the FIFO length and comparing the FIFO count).
   * Hence, the FIFOs are not used in this implementation and, as a result
   * TDRE indeed mean that the single output buffer is available.
   *
   * Performance on UART0 could be improved by enabling the FIFO and by
   * redesigning all of the FIFO status logic.
   */

#ifdef CONFIG_KINETIS_UARTFIFOS
  depth = g_sizemap[(regval & UART_PFIFO_RXFIFOSIZE_MASK) >>
                    UART_PFIFO_RXFIFOSIZE_SHIFT];
  if (depth > 1)
    {
      depth = (3 * depth) >> 2;
    }

  putreg8(depth , uart_base + KINETIS_UART_RWFIFO_OFFSET);

  depth = g_sizemap[(regval & UART_PFIFO_TXFIFOSIZE_MASK) >>
                    UART_PFIFO_TXFIFOSIZE_SHIFT];
  if (depth > 3)
    {
      depth = (depth >> 2);
    }

  putreg8(depth, uart_base + KINETIS_UART_TWFIFO_OFFSET);

  /* Enable RX and TX FIFOs */

  putreg8(UART_PFIFO_RXFE | UART_PFIFO_TXFE,
          uart_base + KINETIS_UART_PFIFO_OFFSET);
#else
  /* Otherwise, disable the FIFOs.  Then the FIFOs are disable, the effective
   * FIFO depth is 1.  So set the watermarks as follows:
   *
   * TWFIFO[TXWATER] = 0:  TDRE will be set when the number of queued bytes
   *  (1 in this case) is less than or equal to 0.
   * RWFIFO[RXWATER] = 1:  RDRF will be set when the number of queued bytes
   *  (1 in this case) is greater than or equal to 1.
   *
   * Set the watermarks to one/zero and disable the FIFOs
   */

  putreg8(1, uart_base + KINETIS_UART_RWFIFO_OFFSET);
  putreg8(0, uart_base + KINETIS_UART_TWFIFO_OFFSET);
  putreg8(0, uart_base + KINETIS_UART_PFIFO_OFFSET);
#endif

  /* Hardware flow control */

  regval  = getreg8(uart_base + KINETIS_UART_MODEM_OFFSET);
  regval &= ~(UART_MODEM_TXCTSE | UART_MODEM_RXRTSE);

#ifdef CONFIG_SERIAL_IFLOWCONTROL
  if (iflow)
    {
      regval |= UART_MODEM_RXRTSE;
    }
#endif

#ifdef CONFIG_SERIAL_OFLOWCONTROL
  if (oflow)
    {
      regval |= UART_MODEM_TXCTSE;
    }
#endif

  putreg8(regval, uart_base + KINETIS_UART_MODEM_OFFSET);

  /* Now we can (re-)enable the transmitter and receiver */

  regval  = getreg8(uart_base + KINETIS_UART_C2_OFFSET);
  regval |= (UART_C2_RE | UART_C2_TE);
  putreg8(regval, uart_base + KINETIS_UART_C2_OFFSET);
}
#endif

/****************************************************************************
 * Name: kinetis_lpuartconfigure
 *
 * Description:
 *   Configure a LPUART as a RS-232 UART.
 *
 ****************************************************************************/

#ifdef HAVE_LPUART_DEVICE
void kinetis_lpuartconfigure(uintptr_t uart_base, uint32_t baud,
                             uint32_t clock, unsigned int parity,
                             unsigned int nbits, unsigned int stop2,
                             bool iflow, bool oflow)
{
  uint32_t     sbrreg;
  uint32_t     osrreg;
  uint32_t     sbr;
  uint32_t     osr;
  uint32_t     actual_baud;
  uint32_t     current_baud;
  uint32_t     baud_error;
  uint32_t     min_baud_error;
  uint32_t     regval;

  /* General note: LPART block input clock can be sourced by
   * SIM_CLKDIV3[PLLFLLFRAC, PLLFLLDIV] since this can be shared with TPM, we
   * would ideally want to maximize the input frequency. This also helps to
   * maximize the oversampling.
   *
   * We would like to maximize oversample and minimize the baud rate error
   *
   * USART baud is generated according to:
   *
   *   baud = clock / (SBR[0:12] * (OSR +1 ))
   *
   * Or, equivalently:
   *
   *   SBR   = clock / (baud * (OSR + 1))
   *   OSR   = clock / (baud * SBR) -1
   *
   *   SBR must be 1..8191
   *   OSR must be 3..31 (macro value 4..32)
   */

  min_baud_error = baud;
  sbrreg = 0;
  osrreg = 0;

  /* While maximizing OSR look for a SBR that minimizes the difference
   * between actual baud and requested baud rate
   */

  for (osr = 32; osr >= 4; osr--)
    {
      sbr = clock / (baud * osr);

      /* Ensure the minimum SBR */

      if (sbr == 0)
        {
          sbr++;
        }

      /* Calculate the actual baud rate */

      current_baud = clock / (sbr * osr);

      /* look at the deviation of current baud to requested */

      baud_error = current_baud - baud;
      if (baud_error <= min_baud_error)
        {
          min_baud_error = baud_error;
          actual_baud = current_baud;
          sbrreg = sbr;
          osrreg = osr;
        }
    }

  UNUSED(actual_baud);
  DEBUGASSERT(actual_baud - baud < (baud / 100) * 2);
  DEBUGASSERT(sbrreg != 0 && sbrreg < 8192);
  DEBUGASSERT(osrreg != 0);

  /* Disable the transmitter and receiver throughout the reconfiguration */

  regval = getreg32(uart_base + KINETIS_LPUART_CTRL_OFFSET);
  regval &= ~(LPUART_CTRL_RE | LPUART_CTRL_TE);
  putreg32(regval, uart_base + KINETIS_LPUART_CTRL_OFFSET);

  /* Reset the BAUD register */

  regval = getreg32(uart_base + KINETIS_LPUART_BAUD_OFFSET);
  regval &= ~(LPUART_BAUD_INIT);

  /* Set the Baud rate, nbits and stop bits */

  regval |= LPUART_BAUD_OSR(osrreg);
  regval |= LPUART_BAUD_SBR(sbrreg);

  /* Set the 10 bit mode */

  if (nbits == 10)
    {
      regval |= LPUART_BAUD_M10;
    }

  /* Set the 2 stop bit mode */

  if (stop2)
    {
      regval |= LPUART_BAUD_SBNS;
    }

  /* BOTHEDG needs to be turned on for 4X-7X */

  if (osrreg >= 4 && osrreg <= 7)
    {
      regval |= LPUART_BAUD_BOTHEDGE;
    }

  putreg32(regval, uart_base + KINETIS_LPUART_BAUD_OFFSET);

  /* Configure number of bits and parity */

  regval = 0;

  /* Check for odd parity */

  if (parity == 1)
    {
      regval |= (LPUART_CTRL_PE | LPUART_CTRL_PT); /* Enable + odd parity type */
    }

  /* Check for even parity */

  else if (parity == 2)
    {
      regval |= LPUART_CTRL_PE;                   /* Enable (even parity default) */
    }

  /* The only other option is no parity */

  else
    {
      DEBUGASSERT(parity == 0);
    }

  /* Check for 9-bit operation */

  if (nbits == 9 || (nbits == 8 && parity != 0))
    {
      regval |= LPUART_CTRL_M;
    }

  /* The only other option is 8-bit operation */

  else
    {
      DEBUGASSERT(nbits == 8);
    }

  /* Hardware flow control */

  regval  = getreg32(uart_base + KINETIS_LPUART_MODIR_OFFSET);
  regval &= ~(UART_MODEM_TXCTSE | UART_MODEM_RXRTSE);

#ifdef CONFIG_SERIAL_IFLOWCONTROL
  if (iflow)
    {
      regval |= LPUART_MODIR_RXRTSE;
    }

#endif
  #ifdef CONFIG_SERIAL_OFLOWCONTROL
  if (oflow)
    {
      regval |= LPUART_MODIR_TXCTSE;
    }

#endif
  putreg32(regval, uart_base + KINETIS_LPUART_MODIR_OFFSET);

  /* Now we can (re-)enable the transmitter and receiver */

  regval |= (LPUART_CTRL_RE | LPUART_CTRL_TE);
  putreg32(regval, uart_base + KINETIS_LPUART_CTRL_OFFSET);
}

#endif
