/****************************************************************************
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/irq.h>

#include "arm_internal.h"
#include "arm_arch.h"

#include "sam_pio.h"
#include "sam_periphclks.h"
#include "sam_config.h"
#include "sam_dbgu.h"
#include "sam_lowputc.h"

#include "hardware/sam_uart.h"
#include "hardware/sam_flexcom.h"
#include "hardware/sam_dbgu.h"
#include "hardware/sam_pinmap.h"

#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The UART/USART modules are driven by the peripheral clock (MCK or MCK2). */

#ifdef SAMA5_HAVE_FLEXCOM_CONSOLE
#  define SAM_USART_CLOCK      BOARD_FLEXCOM_FREQUENCY /* Frequency of the FLEXCOM clock */
#  define SAM_MR_USCLKS        FLEXUS_MR_USCLKS_MCK    /* Source = Main clock */
#else
#  define SAM_USART_CLOCK      BOARD_USART_FREQUENCY   /* Frequency of the USART clock */
#  define SAM_MR_USCLKS        UART_MR_USCLKS_MCK      /* Source = Main clock */
#endif

/* Select USART parameters for the selected console */

#if defined(CONFIG_SAMA5_DBGU_CONSOLE)
#  define SAM_CONSOLE_VBASE    SAM_DBGU_VBASE
#  define SAM_CONSOLE_BITS     8
#  define SAM_CONSOLE_2STOP    0
#  ifdef CONFIG_SAMA5_DBGU_NOCONFIG
#    undef  SUPPRESS_CONSOLE_CONFIG
#    define SUPPRESS_CONSOLE_CONFIG 1
#    define SAM_CONSOLE_BAUD   115200
#    define SAM_CONSOLE_PARITY 0
#  else
#    define SAM_CONSOLE_BAUD   CONFIG_SAMA5_DBGU_BAUD
#    define SAM_CONSOLE_PARITY CONFIG_SAMA5_DBGU_PARITY
#  endif
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
#elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#  define SAM_CONSOLE_VBASE    SAM_UART2_VBASE
#  define SAM_CONSOLE_BAUD     CONFIG_UART2_BAUD
#  define SAM_CONSOLE_BITS     CONFIG_UART2_BITS
#  define SAM_CONSOLE_PARITY   CONFIG_UART2_PARITY
#  define SAM_CONSOLE_2STOP    CONFIG_UART2_2STOP
#elif defined(CONFIG_UART3_SERIAL_CONSOLE)
#  define SAM_CONSOLE_VBASE    SAM_UART3_VBASE
#  define SAM_CONSOLE_BAUD     CONFIG_UART3_BAUD
#  define SAM_CONSOLE_BITS     CONFIG_UART3_BITS
#  define SAM_CONSOLE_PARITY   CONFIG_UART3_PARITY
#  define SAM_CONSOLE_2STOP    CONFIG_UART3_2STOP
#elif defined(CONFIG_UART4_SERIAL_CONSOLE)
#  define SAM_CONSOLE_VBASE    SAM_UART4_VBASE
#  define SAM_CONSOLE_BAUD     CONFIG_UART4_BAUD
#  define SAM_CONSOLE_BITS     CONFIG_UART4_BITS
#  define SAM_CONSOLE_PARITY   CONFIG_UART4_PARITY
#  define SAM_CONSOLE_2STOP    CONFIG_UART4_2STOP
#elif defined(CONFIG_USART0_SERIAL_CONSOLE)
#  ifdef CONFIG_SAMA5_FLEXCOM0_USART
#    define SAM_CONSOLE_VBASE  SAM_FLEXCOM0_VBASE
#  else
#    define SAM_CONSOLE_VBASE  SAM_USART0_VBASE
#  endif
#  define SAM_CONSOLE_BAUD     CONFIG_USART0_BAUD
#  define SAM_CONSOLE_BITS     CONFIG_USART0_BITS
#  define SAM_CONSOLE_PARITY   CONFIG_USART0_PARITY
#  define SAM_CONSOLE_2STOP    CONFIG_USART0_2STOP
#elif defined(CONFIG_USART1_SERIAL_CONSOLE)
#  ifdef CONFIG_SAMA5_FLEXCOM1_USART
#    define SAM_CONSOLE_VBASE  SAM_FLEXCOM1_VBASE
#  else
#    define SAM_CONSOLE_VBASE  SAM_USART1_VBASE
#  endif
#  define SAM_CONSOLE_BAUD     CONFIG_USART1_BAUD
#  define SAM_CONSOLE_BITS     CONFIG_USART1_BITS
#  define SAM_CONSOLE_PARITY   CONFIG_USART1_PARITY
#  define SAM_CONSOLE_2STOP    CONFIG_USART1_2STOP
#elif defined(CONFIG_USART2_SERIAL_CONSOLE)
#  ifdef CONFIG_SAMA5_FLEXCOM2_USART
#    define SAM_CONSOLE_VBASE  SAM_FLEXCOM2_VBASE
#  else
#    define SAM_CONSOLE_VBASE  SAM_USART2_VBASE
#  endif
#  define SAM_CONSOLE_BAUD     CONFIG_USART2_BAUD
#  define SAM_CONSOLE_BITS     CONFIG_USART2_BITS
#  define SAM_CONSOLE_PARITY   CONFIG_USART2_PARITY
#  define SAM_CONSOLE_2STOP    CONFIG_USART2_2STOP
#elif defined(CONFIG_USART3_SERIAL_CONSOLE)
#  ifdef CONFIG_SAMA5_FLEXCOM3_USART
#   define SAM_CONSOLE_VBASE   SAM_FLEXCOM3_VBASE
#  else
#   define SAM_CONSOLE_VBASE   SAM_USART3_VBASE
#  endif
#  define SAM_CONSOLE_BAUD     CONFIG_USART3_BAUD
#  define SAM_CONSOLE_BITS     CONFIG_USART3_BITS
#  define SAM_CONSOLE_PARITY   CONFIG_USART3_PARITY
#  define SAM_CONSOLE_2STOP    CONFIG_USART3_2STOP
#elif defined(CONFIG_USART4_SERIAL_CONSOLE)
#  ifdef CONFIG_SAMA5_FLEXCOM4_USART
#    define SAM_CONSOLE_VBASE  SAM_FLEXCOM4_VBASE
#  else
#    define SAM_CONSOLE_VBASE  SAM_USART4_VBASE
#  endif
#  define SAM_CONSOLE_BAUD     CONFIG_USART4_BAUD
#  define SAM_CONSOLE_BITS     CONFIG_USART4_BITS
#  define SAM_CONSOLE_PARITY   CONFIG_USART4_PARITY
#  define SAM_CONSOLE_2STOP    CONFIG_USART4_2STOP
#else
#  error "No CONFIG_U[S]ARTn_SERIAL_CONSOLE Setting"
#endif

/* Select the settings for the mode register */

#if defined(SAMA5_HAVE_UART_CONSOLE)
#  if SAM_CONSOLE_BITS == 8 && SAM_CONSOLE_PARITY == 0
#  elif SAM_CONSOLE_BITS == 7 && SAM_CONSOLE_PARITY != 0
#  else
#    error "Unsupported combination of bits and parity in UART console"
#  endif

#  if SAM_CONSOLE_2STOP != 0
#    error "Unsupported number of stop bits and parity for UART console"
#  endif
#endif

#if SAM_CONSOLE_BITS == 5
#  define MR_CHRL_VALUE UART_MR_CHRL_5BITS /* 5 bits */
#elif SAM_CONSOLE_BITS == 6
#  define MR_CHRL_VALUE UART_MR_CHRL_6BITS /* 6 bits */
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

#if defined(ATSAMA5D2)
#  define MR_VALUE (MR_PAR_VALUE | UART_MR_PERIPHCLK | \
                    UART_MR_CHMODE_NORMAL)
#elif defined(ATSAMA5D3) ||defined(ATSAMA5D4)
#  define MR_VALUE (UART_MR_MODE_NORMAL | SAM_MR_USCLKS | \
                    MR_CHRL_VALUE | MR_PAR_VALUE | MR_NBSTOP_VALUE | \
                    UART_MR_CHMODE_NORMAL)
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
#if defined(SAMA5_HAVE_UART_CONSOLE) || defined(SAMA5_HAVE_USART_CONSOLE)
  irqstate_t flags;

  for (; ; )
    {
      /* Wait for the transmitter to be available */

      while ((getreg32(SAM_CONSOLE_VBASE + SAM_UART_SR_OFFSET) &
        UART_INT_TXEMPTY) == 0);

      /* Disable interrupts so that the test and the transmission are
       * atomic.
       */

      flags = enter_critical_section();
      if ((getreg32(SAM_CONSOLE_VBASE + SAM_UART_SR_OFFSET) &
        UART_INT_TXEMPTY) != 0)
        {
          /* Send the character */

          putreg32((uint32_t)ch, SAM_CONSOLE_VBASE + SAM_UART_THR_OFFSET);
          leave_critical_section(flags);
          return;
        }

      leave_critical_section(flags);
    }

#elif defined(SAMA5_HAVE_FLEXCOM_CONSOLE)
  irqstate_t flags;

  for (; ; )
    {
      /* Wait for the transmitter to be available */

      while ((getreg32(SAM_CONSOLE_VBASE + SAM_FLEXUS_CSR_OFFSET) &
        FLEXUS_INT_TXEMPTY) == 0);

      /* Disable interrupts so that the test and the transmission are
       * atomic.
       */

      flags = enter_critical_section();
      if ((getreg32(SAM_CONSOLE_VBASE + SAM_FLEXUS_CSR_OFFSET) &
        FLEXUS_INT_TXEMPTY) != 0)
        {
          /* Send the character */

          putreg32((uint32_t)ch, SAM_CONSOLE_VBASE + SAM_FLEXUS_THR_OFFSET);
          leave_critical_section(flags);
          return;
        }

      leave_critical_section(flags);
    }

#elif defined(CONFIG_SAMA5_DBGU_CONSOLE)
  irqstate_t flags;

  for (; ; )
    {
      /* Wait for the transmitter to be available */

      while ((getreg32(SAM_DBGU_SR) & DBGU_INT_TXEMPTY) == 0);

      /* Disable interrupts so that the test and the transmission are
       * atomic.
       */

      flags = enter_critical_section();
      if ((getreg32(SAM_DBGU_SR) & DBGU_INT_TXEMPTY) != 0)
        {
          /* Send the character */

          putreg32((uint32_t)ch, SAM_DBGU_THR);
          leave_critical_section(flags);
          return;
        }

      leave_critical_section(flags);
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
#if defined(SAMA5_HAVE_UART_CONSOLE) || defined(SAMA5_HAVE_USART_CONSOLE) || \
    defined(SAMA5_HAVE_FLEXCOM_CONSOLE) || defined(CONFIG_SAMA5_DBGU_CONSOLE)
  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      arm_lowputc('\r');
    }

  arm_lowputc(ch);
#endif
  return ch;
}

/****************************************************************************
 * Name: sam_lowsetup
 *
 * Description:
 *   This performs basic initialization of the USART used for the serial
 *   console.  Its purpose is to get the console output available as soon
 *   as possible.
 *
 ****************************************************************************/

void sam_lowsetup(void)
{
  /* Enable clocking for all selected UART/USARTs (USARTs may not
   * necessarily be configured as UARTs).
   */

#ifdef CONFIG_SAMA5_UART0
  sam_uart0_enableclk();
#endif
#ifdef CONFIG_SAMA5_UART1
  sam_uart1_enableclk();
#endif
#ifdef CONFIG_SAMA5_UART2
  sam_uart2_enableclk();
#endif
#ifdef CONFIG_SAMA5_UART3
  sam_uart3_enableclk();
#endif
#ifdef CONFIG_SAMA5_UART4
  sam_uart4_enableclk();
#endif
#ifdef CONFIG_USART0_SERIALDRIVER
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
#ifdef CONFIG_SAMA5_FLEXCOM0
  sam_flexcom0_enableclk();
#endif
#ifdef CONFIG_SAMA5_FLEXCOM1
  sam_flexcom1_enableclk();
#endif
#ifdef CONFIG_SAMA5_FLEXCOM2
  sam_flexcom2_enableclk();
#endif
#ifdef CONFIG_SAMA5_FLEXCOM3
  sam_flexcom3_enableclk();
#endif
#ifdef CONFIG_SAMA5_FLEXCOM4
  sam_flexcom4_enableclk();
#endif

  /* Configure UART pins for all selected UART/USARTs.  USARTs pins are
   * only configured if the USART is also configured as as a UART.
   */

#ifdef CONFIG_SAMA5_UART0
  sam_configpio(PIO_UART0_RXD);
  sam_configpio(PIO_UART0_TXD);
#endif

#ifdef CONFIG_SAMA5_UART1
  sam_configpio(PIO_UART1_RXD);
  sam_configpio(PIO_UART1_TXD);
#endif

#ifdef CONFIG_SAMA5_UART2
  sam_configpio(PIO_UART2_RXD);
  sam_configpio(PIO_UART2_TXD);
#endif

#ifdef CONFIG_SAMA5_UART3
  sam_configpio(PIO_UART3_RXD);
  sam_configpio(PIO_UART3_TXD);
#endif

#ifdef CONFIG_SAMA5_UART4
  sam_configpio(PIO_UART4_RXD);
  sam_configpio(PIO_UART4_TXD);
#endif

#if defined(CONFIG_USART0_SERIALDRIVER) && defined(CONFIG_SAMA5_USART0)
  sam_configpio(PIO_USART0_RXD);
  sam_configpio(PIO_USART0_TXD);
#ifdef CONFIG_USART0_OFLOWCONTROL
  sam_configpio(PIO_USART0_CTS);
#endif
#ifdef CONFIG_USART0_IFLOWCONTROL
  sam_configpio(PIO_USART0_RTS);
#endif
#endif

#if defined(CONFIG_USART1_SERIALDRIVER) && defined(CONFIG_SAMA5_USART1)
  sam_configpio(PIO_USART1_RXD);
  sam_configpio(PIO_USART1_TXD);
#ifdef CONFIG_USART1_OFLOWCONTROL
  sam_configpio(PIO_USART1_CTS);
#endif
#ifdef CONFIG_USART1_IFLOWCONTROL
  sam_configpio(PIO_USART1_RTS);
#endif
#endif

#if defined(CONFIG_USART2_SERIALDRIVER) && defined(CONFIG_SAMA5_USART2)
  sam_configpio(PIO_USART2_RXD);
  sam_configpio(PIO_USART2_TXD);
#ifdef CONFIG_USART2_OFLOWCONTROL
  sam_configpio(PIO_USART2_CTS);
#endif
#ifdef CONFIG_USART2_IFLOWCONTROL
  sam_configpio(PIO_USART2_RTS);
#endif
#endif

#if defined(CONFIG_USART3_SERIALDRIVER) && defined(CONFIG_SAMA5_USART3)
  sam_configpio(PIO_USART3_RXD);
  sam_configpio(PIO_USART3_TXD);
#ifdef CONFIG_USART3_OFLOWCONTROL
  sam_configpio(PIO_USART3_CTS);
#endif
#ifdef CONFIG_USART3_IFLOWCONTROL
  sam_configpio(PIO_USART3_RTS);
#endif
#endif

#if defined(CONFIG_USART4_SERIALDRIVER) && defined(CONFIG_SAMA5_USART4)
  sam_configpio(PIO_USART4_RXD);
  sam_configpio(PIO_USART4_TXD);
#ifdef CONFIG_USART4_OFLOWCONTROL
  sam_configpio(PIO_USART4_CTS);
#endif
#ifdef CONFIG_USART4_IFLOWCONTROL
  sam_configpio(PIO_USART4_RTS);
#endif
#endif

  /* For Flexcom USARTs:
   *
   *   FLEXCOM_IO0 = TXD
   *   FLEXCOM_IO1 = RXD
   *   FLEXCOM_IO2 = SCK
   *   FLEXCOM_IO3 = CTS
   *   FLEXCOM_IO4 = RTS
   */

#if defined(CONFIG_USART0_SERIALDRIVER) && defined(CONFIG_SAMA5_FLEXCOM0_USART)
  sam_configpio(PIO_FLEXCOM0_IO0);
  sam_configpio(PIO_FLEXCOM0_IO1);
#ifdef CONFIG_USART0_OFLOWCONTROL
  sam_configpio(PIO_FLEXCOM0_IO3);
#endif
#ifdef CONFIG_USART0_IFLOWCONTROL
  sam_configpio(PIO_FLEXCOM0_IO4);
#endif
#endif

#if defined(CONFIG_USART1_SERIALDRIVER) && defined(CONFIG_SAMA5_FLEXCOM1_USART)
  sam_configpio(PIO_FLEXCOM1_IO0);
  sam_configpio(PIO_FLEXCOM1_IO1);
#ifdef CONFIG_USART1_OFLOWCONTROL
  sam_configpio(PIO_FLEXCOM1_IO3);
#endif
#ifdef CONFIG_USART1_IFLOWCONTROL
  sam_configpio(PIO_FLEXCOM1_IO4);
#endif
#endif

#if defined(CONFIG_USART2_SERIALDRIVER) && defined(CONFIG_SAMA5_FLEXCOM2_USART)
  sam_configpio(PIO_FLEXCOM2_IO0);
  sam_configpio(PIO_FLEXCOM2_IO1);
#ifdef CONFIG_USART2_OFLOWCONTROL
  sam_configpio(PIO_FLEXCOM2_IO3);
#endif
#ifdef CONFIG_USART2_IFLOWCONTROL
  sam_configpio(PIO_FLEXCOM2_IO4);
#endif
#endif

#if defined(CONFIG_USART3_SERIALDRIVER) && defined(CONFIG_SAMA5_FLEXCOM3_USART)
  sam_configpio(PIO_FLEXCOM3_IO0);
  sam_configpio(PIO_FLEXCOM3_IO1);
#ifdef CONFIG_USART3_OFLOWCONTROL
  sam_configpio(PIO_FLEXCOM3_IO3);
#endif
#ifdef CONFIG_USART3_IFLOWCONTROL
  sam_configpio(PIO_FLEXCOM3_IO4);
#endif
#endif

#if defined(CONFIG_USART4_SERIALDRIVER) && defined(CONFIG_SAMA5_FLEXCOM4_USART)
  sam_configpio(PIO_FLEXCOM4_IO0);
  sam_configpio(PIO_FLEXCOM4_IO1);
#ifdef CONFIG_USART4_OFLOWCONTROL
  sam_configpio(PIO_FLEXCOM4_IO3);
#endif
#ifdef CONFIG_USART4_IFLOWCONTROL
  sam_configpio(PIO_FLEXCOM4_IO4);
#endif
#endif

  /* Configure the console (only) */

#if (defined(SAMA5_HAVE_UART_CONSOLE) || defined(SAMA5_HAVE_USART_CONSOLE)) && \
    !defined(SUPPRESS_CONSOLE_CONFIG)
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

#elif defined(SAMA5_HAVE_FLEXCOM_CONSOLE) &&  !defined(SUPPRESS_CONSOLE_CONFIG)
  /* Select USART mode for the Flexcom */

  putreg32(FLEX_MR_OPMODE_USART, SAM_CONSOLE_VBASE + SAM_FLEX_MR_OFFSET);

  /* Reset and disable receiver and transmitter */

  putreg32((FLEXUS_CR_RSTRX | FLEXUS_CR_RSTTX | FLEXUS_CR_RXDIS | FLEXUS_CR_TXDIS),
           SAM_CONSOLE_VBASE + SAM_FLEXUS_CR_OFFSET);

  /* Disable all interrupts */

  putreg32(0xffffffff, SAM_CONSOLE_VBASE + SAM_FLEXUS_IDR_OFFSET);

  /* Set up the mode register */

  putreg32(MR_VALUE, SAM_CONSOLE_VBASE + SAM_FLEXUS_MR_OFFSET);

  /* Configure the console baud.  NOTE: Oversampling by 8 is not supported.
   * This may limit BAUD rates for lower USART clocks.
   */

  putreg32(((SAM_USART_CLOCK + (SAM_CONSOLE_BAUD << 3)) / (SAM_CONSOLE_BAUD << 4)),
           SAM_CONSOLE_VBASE + SAM_FLEXUS_BRGR_OFFSET);

  /* Enable receiver & transmitter */

  putreg32((FLEXUS_CR_RXEN | FLEXUS_CR_TXEN),
           SAM_CONSOLE_VBASE + SAM_FLEXUS_CR_OFFSET);

#endif

#ifdef CONFIG_SAMA5_DBGU
  /* Initialize the DBGU (might be the serial console) */

  sam_dbgu_initialize();
#endif
}
