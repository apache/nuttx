/**************************************************************************
 * arch/arm/src/sama5/sam_lowputc.c
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

#include "sam_pio.h"
#include "sam_periphclks.h"
#include "sam_dbgu.h"
#include "sam_lowputc.h"

#include "chip/sam_uart.h"
#include "chip/sam_dbgu.h"
#include "chip/sam_pinmap.h"

/**************************************************************************
 * Pre-processor Definitions
 **************************************************************************/

/* Configuration **********************************************************/

/* If the USART is not being used as a UART, then it really isn't enabled
 * for our purposes.
 */

#ifndef CONFIG_USART0_ISUART
#  undef CONFIG_SAMA5_USART0
#endif
#ifndef CONFIG_USART1_ISUART
#  undef CONFIG_SAMA5_USART1
#endif
#ifndef CONFIG_USART2_ISUART
#  undef CONFIG_SAMA5_USART2
#endif
#ifndef CONFIG_USART3_ISUART
#  undef CONFIG_SAMA5_USART3
#endif

/* Is there a serial console?  It could be on UART0-1 or USART0-3 */

#if defined(CONFIG_SAMA5_DBGU_CONSOLE) && defined(CONFIG_SAMA5_DBGU)
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  undef HAVE_UART_CONSOLE
#elif defined(CONFIG_UART0_SERIAL_CONSOLE) && defined(CONFIG_SAMA5_UART0)
#  undef CONFIG_SAMA5_DBGU_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  define HAVE_UART_CONSOLE 1
#elif defined(CONFIG_UART1_SERIAL_CONSOLE) && defined(CONFIG_SAMA5_UART1)
#  undef CONFIG_SAMA5_DBGU_CONSOLE
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  define HAVE_UART_CONSOLE 1
#elif defined(CONFIG_USART0_SERIAL_CONSOLE) && defined(CONFIG_SAMA5_USART0)
#  undef CONFIG_SAMA5_DBGU_CONSOLE
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  define HAVE_UART_CONSOLE 1
#elif defined(CONFIG_USART1_SERIAL_CONSOLE) && defined(CONFIG_SAMA5_USART1)
#  undef CONFIG_SAMA5_DBGU_CONSOLE
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  define HAVE_UART_CONSOLE 1
#elif defined(CONFIG_USART2_SERIAL_CONSOLE) && defined(CONFIG_SAMA5_USART2)
#  undef CONFIG_SAMA5_DBGU_CONSOLE
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  define HAVE_UART_CONSOLE 1
#elif defined(CONFIG_USART3_SERIAL_CONSOLE) && defined(CONFIG_SAMA5_USART3)
#  undef CONFIG_SAMA5_DBGU_CONSOLE
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  define HAVE_UART_CONSOLE 1
#else
#  warning "No valid CONFIG_USARTn_SERIAL_CONSOLE Setting"
#  undef CONFIG_SAMA5_DBGU_CONSOLE
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  undef HAVE_UART_CONSOLE
#endif

/* The UART/USART modules are driven by the peripheral clock (MCK or MCK2). */

#define SAM_USART_CLOCK  BOARD_USART_FREQUENCY /* Frequency of the USART clock */
#define SAM_MR_USCLKS    UART_MR_USCLKS_MCK    /* Source = Main clock */

/* Select USART parameters for the selected console */

#if defined(CONFIG_SAMA5_DBGU_CONSOLE)
#  define SAM_CONSOLE_VBASE    SAM_DBGU_VBASE
#  define SAM_CONSOLE_BAUD     CONFIG_SAMA5_DBGU_BAUD
#  define SAM_CONSOLE_BITS     8
#  define SAM_CONSOLE_PARITY   CONFIG_SAMA5_DBGU_PARITY
#  define SAM_CONSOLE_2STOP    0
#elif defined(CONFIG_UART0_SERIAL_CONSOLE)
#  define SAM_CONSOLE_VBASE    SAM_UART0_VBASE
#  define SAM_CONSOLE_BAUD     CONFIG_UART0_BAUD
#  define SAM_CONSOLE_BITS     CONFIG_UART0_BITS
#  define SAM_CONSOLE_PARITY   CONFIG_UART0_PARITY
#  define SAM_CONSOLE_2STOP    CONFIG_UART0_2STOP
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#  define SAM_CONSOLE_VBASE    SAM_UART1_VBASE
#  define SAM_CONSOLE_BAUD     CONFIG_UART1_BAUD
#  define SAM_CONSOLE_BITS     CONFIG_UART1_BITS
#  define SAM_CONSOLE_PARITY   CONFIG_UART1_PARITY
#  define SAM_CONSOLE_2STOP    CONFIG_UART1_2STOP
#elif defined(CONFIG_USART0_SERIAL_CONSOLE)
#  define SAM_CONSOLE_VBASE    SAM_USART0_VBASE
#  define SAM_CONSOLE_BAUD     CONFIG_USART0_BAUD
#  define SAM_CONSOLE_BITS     CONFIG_USART0_BITS
#  define SAM_CONSOLE_PARITY   CONFIG_USART0_PARITY
#  define SAM_CONSOLE_2STOP    CONFIG_USART0_2STOP
#elif defined(CONFIG_USART1_SERIAL_CONSOLE)
#  define SAM_CONSOLE_VBASE    SAM_USART1_VBASE
#  define SAM_CONSOLE_BAUD     CONFIG_USART1_BAUD
#  define SAM_CONSOLE_BITS     CONFIG_USART1_BITS
#  define SAM_CONSOLE_PARITY   CONFIG_USART1_PARITY
#  define SAM_CONSOLE_2STOP    CONFIG_USART1_2STOP
#elif defined(CONFIG_USART2_SERIAL_CONSOLE)
#  define SAM_CONSOLE_VBASE    SAM_USART2_VBASE
#  define SAM_CONSOLE_BAUD     CONFIG_USART2_BAUD
#  define SAM_CONSOLE_BITS     CONFIG_USART2_BITS
#  define SAM_CONSOLE_PARITY   CONFIG_USART2_PARITY
#  define SAM_CONSOLE_2STOP    CONFIG_USART2_2STOP
#elif defined(CONFIG_USART3_SERIAL_CONSOLE)
#  define SAM_CONSOLE_VBASE    SAM_USART3_VBASE
#  define SAM_CONSOLE_BAUD     CONFIG_USART3_BAUD
#  define SAM_CONSOLE_BITS     CONFIG_USART3_BITS
#  define SAM_CONSOLE_PARITY   CONFIG_USART3_PARITY
#  define SAM_CONSOLE_2STOP    CONFIG_USART3_2STOP
#else
#  error "No CONFIG_U[S]ARTn_SERIAL_CONSOLE Setting"
#endif

/* Select the settings for the mode register */

#if SAM_CONSOLE_BITS == 5
#  define MR_CHRL_VALUE UART_MR_CHRL_5BITS /* 5 bits */
#elif SAM_CONSOLE_BITS == 6
#  define MR_CHRL_VALUE UART_MR_CHRL_6BITS  /* 6 bits */
#elif SAM_CONSOLE_BITS == 7
#  define MR_CHRL_VALUE UART_MR_CHRL_7BITS /* 7 bits */
#elif SAM_CONSOLE_BITS == 8
#  define MR_CHRL_VALUE UART_MR_CHRL_8BITS /* 8 bits */
#elif SAM_CONSOLE_BITS == 9 && !defined(CONFIG_UART0_SERIAL_CONSOLE) && \
     !defined(CONFIG_UART1_SERIAL_CONSOLE)
#  define MR_CHRL_VALUE UART_MR_MODE9
#else
#  error "Invalid number of bits"
#endif

#if SAM_CONSOLE_PARITY == 1
#  define MR_PAR_VALUE UART_MR_PAR_ODD
#elif SAM_CONSOLE_PARITY == 2
#  define MR_PAR_VALUE UART_MR_PAR_EVEN
#else
#  define MR_PAR_VALUE UART_MR_PAR_NONE
#endif

#if SAM_CONSOLE_2STOP != 0
#  define MR_NBSTOP_VALUE UART_MR_NBSTOP_2
#else
#  define MR_NBSTOP_VALUE UART_MR_NBSTOP_1
#endif

#define MR_VALUE (UART_MR_MODE_NORMAL | SAM_MR_USCLKS | \
                  MR_CHRL_VALUE | MR_PAR_VALUE | MR_NBSTOP_VALUE)

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
#if defined(HAVE_UART_CONSOLE)
  irqstate_t flags;

  for (;;)
    {
      /* Wait for the transmitter to be available */

      while ((getreg32(SAM_CONSOLE_VBASE + SAM_UART_SR_OFFSET) &
        UART_INT_TXEMPTY) == 0);

      /* Disable interrupts so that the test and the transmission are
       * atomic.
       */

      flags = irqsave();
      if ((getreg32(SAM_CONSOLE_VBASE + SAM_UART_SR_OFFSET) &
        UART_INT_TXEMPTY) != 0)
        {
          /* Send the character */

          putreg32((uint32_t)ch, SAM_CONSOLE_VBASE + SAM_UART_THR_OFFSET);
          irqrestore(flags);
          return;
        }

      irqrestore(flags);
    }
#elif defined(CONFIG_SAMA5_DBGU_CONSOLE)
  irqstate_t flags;

  for (;;)
    {
      /* Wait for the transmitter to be available */

      while ((getreg32(SAM_DBGU_SR) & DBGU_INT_TXEMPTY) == 0);

      /* Disable interrupts so that the test and the transmission are
       * atomic.
       */

      flags = irqsave();
      if ((getreg32(SAM_DBGU_SR) & DBGU_INT_TXEMPTY) != 0)
        {
          /* Send the character */

          putreg32((uint32_t)ch, SAM_DBGU_THR);
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
#if defined(HAVE_UART_CONSOLE) || defined(CONFIG_SAMA5_DBGU_CONSOLE)
  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      up_lowputc('\r');
    }

  up_lowputc(ch);
  return ch;
}
#endif

/**************************************************************************
 * Name: sam_lowsetup
 *
 * Description:
 *   This performs basic initialization of the USART used for the serial
 *   console.  Its purpose is to get the console output availabe as soon
 *   as possible.
 *
 **************************************************************************/

void sam_lowsetup(void)
{
  /* Enable clocking for all selected UART/USARTs */

#ifdef CONFIG_SAMA5_UART0
  sam_uart0_enableclk();
#endif
#ifdef CONFIG_SAMA5_UART1
  sam_uart1_enableclk();
#endif
#ifdef CONFIG_SAMA5_USART0
  sam_usart0_enableclk();
#endif
#ifdef CONFIG_SAMA5_USART1
  sam_usart1_enableclk();
#endif
#ifdef CONFIG_SAMA5_USART2
  sam_usart2_enableclk();
#endif
#ifdef CONFIG_SAMA5_USART3
  sam_usart3_enableclk();
#endif

  /* Configure UART pins for all selected UART/USARTs */

#ifdef CONFIG_SAMA5_UART0
  (void)sam_configpio(PIO_UART0_RXD);
  (void)sam_configpio(PIO_UART0_TXD);
#endif

#ifdef CONFIG_SAMA5_UART1
  (void)sam_configpio(PIO_UART1_RXD);
  (void)sam_configpio(PIO_UART1_TXD);
#endif

#ifdef CONFIG_SAMA5_USART0
  (void)sam_configpio(PIO_USART0_RXD);
  (void)sam_configpio(PIO_USART0_TXD);
#ifdef CONFIG_USART0_OFLOWCONTROL
  (void)sam_configpio(PIO_USART0_CTS);
#endif
#ifdef CONFIG_USART0_IFLOWCONTROL
  (void)sam_configpio(PIO_USART0_RTS);
#endif
#endif

#ifdef CONFIG_SAMA5_USART1
  (void)sam_configpio(PIO_USART1_RXD);
  (void)sam_configpio(PIO_USART1_TXD);
#ifdef CONFIG_USART1_OFLOWCONTROL
  (void)sam_configpio(PIO_USART1_CTS);
#endif
#ifdef CONFIG_USART1_IFLOWCONTROL
  (void)sam_configpio(PIO_USART1_RTS);
#endif
#endif

#ifdef CONFIG_SAMA5_USART2
  (void)sam_configpio(PIO_USART2_RXD);
  (void)sam_configpio(PIO_USART2_TXD);
#ifdef CONFIG_USART2_OFLOWCONTROL
  (void)sam_configpio(PIO_USART2_CTS);
#endif
#ifdef CONFIG_USART2_IFLOWCONTROL
  (void)sam_configpio(PIO_USART2_RTS);
#endif
#endif

#ifdef CONFIG_SAMA5_USART3
  (void)sam_configpio(PIO_USART3_RXD);
  (void)sam_configpio(PIO_USART3_TXD);
#ifdef CONFIG_USART3_OFLOWCONTROL
  (void)sam_configpio(PIO_USART3_CTS);
#endif
#ifdef CONFIG_USART3_IFLOWCONTROL
  (void)sam_configpio(PIO_USART3_RTS);
#endif
#endif

  /* Configure the console (only) */

#if defined(HAVE_UART_CONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)
  /* Reset and disable receiver and transmitter */

  putreg32((UART_CR_RSTRX | UART_CR_RSTTX | UART_CR_RXDIS | UART_CR_TXDIS),
           SAM_CONSOLE_VBASE + SAM_UART_CR_OFFSET);

  /* Disable all interrupts */

  putreg32(0xffffffff, SAM_CONSOLE_VBASE + SAM_UART_IDR_OFFSET);

  /* Set up the mode register */

  putreg32(MR_VALUE, SAM_CONSOLE_VBASE + SAM_UART_MR_OFFSET);

  /* Configure the console baud.  NOTE: Oversampling by 8 is not supported.
   * This may limit BAUD rates for lower USART clocks.
   */

  putreg32(((SAM_USART_CLOCK + (SAM_CONSOLE_BAUD << 3)) / (SAM_CONSOLE_BAUD << 4)),
           SAM_CONSOLE_VBASE + SAM_UART_BRGR_OFFSET);

  /* Enable receiver & transmitter */

  putreg32((UART_CR_RXEN | UART_CR_TXEN),
           SAM_CONSOLE_VBASE + SAM_UART_CR_OFFSET);
#endif

#ifdef CONFIG_SAMA5_DBGU
  /* Initialize the DBGU (might be the serial console) */

  sam_dbgu_initialize();
#endif
}
