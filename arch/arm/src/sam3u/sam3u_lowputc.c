/**************************************************************************
 * arch/arm/src/sam3u/sam3u_lowputc.c
 *
 *   Copyright (C) 2010, 2013 Gregory Nutt. All rights reserved.
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

#include "sam3u_internal.h"
#include "chip/sam_pmc.h"
#include "chip/sam_uart.h"

/**************************************************************************
 * Private Definitions
 **************************************************************************/

/* Configuration **********************************************************/

/* If the USART is not being used as a UART, then it really isn't enabled
 * for our purposes.
 */

#ifndef CONFIG_USART0_ISUART
#  undef CONFIG_SAM34_USART0
#endif
#ifndef CONFIG_USART1_ISUART
#  undef CONFIG_SAM34_USART1
#endif
#ifndef CONFIG_USART2_ISUART
#  undef CONFIG_SAM34_USART2
#endif
#ifndef CONFIG_USART3_ISUART
#  undef CONFIG_SAM34_USART3
#endif

/* Is there a serial console? It could be on the UART, or USARTn */

#if defined(CONFIG_UART_SERIAL_CONSOLE) && defined(CONFIG_SAM34_UART)
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_USART0_SERIAL_CONSOLE) && defined(CONFIG_SAM34_USART0)
#  undef CONFIG_USART_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_USART1_SERIAL_CONSOLE) && defined(CONFIG_SAM34_USART1)
#  undef CONFIG_USART_SERIAL_CONSOLE
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_USART2_SERIAL_CONSOLE) && defined(CONFIG_SAM34_USART2)
#  undef CONFIG_USART_SERIAL_CONSOLE
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_USART3_SERIAL_CONSOLE) && defined(CONFIG_SAM34_USART3)
#  undef CONFIG_USART_SERIAL_CONSOLE
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  define HAVE_CONSOLE 1
#else
#  undef CONFIG_USART_SERIAL_CONSOLE
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  undef HAVE_CONSOLE
#endif

/* Select USART parameters for the selected console */

#if defined(CONFIG_UART_SERIAL_CONSOLE)
#  define SAM_CONSOLE_BASE     SAM_UART_BASE
#  define SAM_CONSOLE_BAUD     CONFIG_UART_BAUD
#  define SAM_CONSOLE_BITS     CONFIG_UART_BITS
#  define SAM_CONSOLE_PARITY   CONFIG_UART_PARITY
#  define SAM_CONSOLE_2STOP    CONFIG_UART_2STOP
#elif defined(CONFIG_USART0_SERIAL_CONSOLE)
#  define SAM_CONSOLE_BASE     SAM_USART0_BASE
#  define SAM_CONSOLE_BAUD     CONFIG_USART0_BAUD
#  define SAM_CONSOLE_BITS     CONFIG_USART0_BITS
#  define SAM_CONSOLE_PARITY   CONFIG_USART0_PARITY
#  define SAM_CONSOLE_2STOP    CONFIG_USART0_2STOP
#elif defined(CONFIG_USART1_SERIAL_CONSOLE)
#  define SAM_CONSOLE_BASE     SAM_USART1_BASE
#  define SAM_CONSOLE_BAUD     CONFIG_USART1_BAUD
#  define SAM_CONSOLE_BITS     CONFIG_USART1_BITS
#  define SAM_CONSOLE_PARITY   CONFIG_USART1_PARITY
#  define SAM_CONSOLE_2STOP    CONFIG_USART1_2STOP
#elif defined(CONFIG_USART2_SERIAL_CONSOLE)
#  define SAM_CONSOLE_BASE     SAM_USART2_BASE
#  define SAM_CONSOLE_BAUD     CONFIG_USART2_BAUD
#  define SAM_CONSOLE_BITS     CONFIG_USART2_BITS
#  define SAM_CONSOLE_PARITY   CONFIG_USART2_PARITY
#  define SAM_CONSOLE_2STOP    CONFIG_USART2_2STOP
#elif defined(CONFIG_USART3_SERIAL_CONSOLE)
#  define SAM_CONSOLE_BASE     SAM_USART3_BASE
#  define SAM_CONSOLE_BAUD     CONFIG_USART3_BAUD
#  define SAM_CONSOLE_BITS     CONFIG_USART3_BITS
#  define SAM_CONSOLE_PARITY   CONFIG_USART3_PARITY
#  define SAM_CONSOLE_2STOP    CONFIG_USART3_2STOP
#else
#  error "No CONFIG_U[S]ARTn_SERIAL_CONSOLE Setting"
#endif

/* Select the settings for the mode register */

#if SAM_CONSOLE_BITS == 5
#  define MR_CHRL_VALUE USART_MR_CHRL_5BITS /* 5 bits */
#elif SAM_CONSOLE_BITS == 6
#  define MR_CHRL_VALUE USART_MR_CHRL_6BITS  /* 6 bits */
#elif SAM_CONSOLE_BITS == 7
#  define MR_CHRL_VALUE USART_MR_CHRL_7BITS /* 7 bits */
#elif SAM_CONSOLE_BITS == 8
#  define MR_CHRL_VALUE USART_MR_CHRL_8BITS /* 8 bits */
#elif SAM_CONSOLE_BITS == 9 && !defined(CONFIG_UART_SERIAL_CONSOLE)
#  define MR_CHRL_VALUE USART_MR_MODE9
#else
#  error "Invlaid number of bits"
#endif

#if SAM_CONSOLE_PARITY == 1
#  define MR_PAR_VALUE UART_MR_PAR_ODD
#elif SAM_CONSOLE_PARITY == 2
#  define MR_PAR_VALUE UART_MR_PAR_EVEN
#else
#  define MR_PAR_VALUE UART_MR_PAR_NONE
#endif

#if SAM_CONSOLE_2STOP != 0
#  define MR_NBSTOP_VALUE USART_MR_NBSTOP_2
#else
#  define MR_NBSTOP_VALUE USART_MR_NBSTOP_1
#endif

#define MR_VALUE (USART_MR_MODE_NORMAL | USART_MR_USCLKS_MCK | \
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
  /* Wait for the transmitter to be available */

  while ((getreg32(SAM_CONSOLE_BASE + SAM_UART_SR_OFFSET) & UART_INT_TXEMPTY) == 0);

  /* Send the character */

  putreg32((uint32_t)ch, SAM_CONSOLE_BASE + SAM_UART_THR_OFFSET);
}

/**************************************************************************
 * Name: sam3u_lowsetup
 *
 * Description:
 *   This performs basic initialization of the USART used for the serial
 *   console.  Its purpose is to get the console output availabe as soon
 *   as possible.
 *
 **************************************************************************/

void sam3u_lowsetup(void)
{
  uint32_t regval;

  /* Enable clocking for all selected UART/USARTs */

  regval = 0;
#ifdef CONFIG_SAM34_UART
  regval |= (1 << SAM_PID_UART);
#endif
#ifdef CONFIG_SAM34_USART0
  regval |= (1 << SAM_PID_USART0);
#endif
#ifdef CONFIG_SAM34_USART1
  regval |= (1 << SAM_PID_USART1);
#endif
#ifdef CONFIG_SAM34_USART2
  regval |= (1 << SAM_PID_USART2);
#endif
#ifdef CONFIG_SAM34_USART3
  regval |= (1 << SAM_PID_USART3);
#endif
  putreg32(regval, SAM_PMC_PCER);

  /* Configure UART pins for all selected UART/USARTs */

#ifdef CONFIG_SAM34_UART
  (void)sam3u_configgpio(GPIO_UART_RXD);
  (void)sam3u_configgpio(GPIO_UART_TXD);
#endif
#ifdef CONFIG_SAM34_USART0
  (void)sam3u_configgpio(GPIO_USART0_RXD);
  (void)sam3u_configgpio(GPIO_USART0_TXD);
  (void)sam3u_configgpio(GPIO_USART0_CTS);
  (void)sam3u_configgpio(GPIO_USART0_RTS);
#endif
#ifdef CONFIG_SAM34_USART1
  (void)sam3u_configgpio(GPIO_USART1_RXD);
  (void)sam3u_configgpio(GPIO_USART1_TXD);
  (void)sam3u_configgpio(GPIO_USART1_CTS);
  (void)sam3u_configgpio(GPIO_USART1_RTS);
#endif
#ifdef CONFIG_SAM34_USART2
  (void)sam3u_configgpio(GPIO_USART2_RXD);
  (void)sam3u_configgpio(GPIO_USART2_TXD);
  (void)sam3u_configgpio(GPIO_USART2_CTS);
  (void)sam3u_configgpio(GPIO_USART2_RTS);
#endif
#ifdef CONFIG_SAM34_USART3
  (void)sam3u_configgpio(GPIO_USART3_RXD);
  (void)sam3u_configgpio(GPIO_USART3_TXD);
  (void)sam3u_configgpio(GPIO_USART3_CTS);
  (void)sam3u_configgpio(GPIO_USART3_RTS);
#endif

#ifdef GPIO_CONSOLE_RXD
#endif
#ifdef GPIO_CONSOLE_TXD
  (void)sam3u_configgpio(GPIO_CONSOLE_TXD);
#endif
#ifdef GPIO_CONSOLE_CTS
  (void)sam3u_configgpio(GPIO_CONSOLE_CTS);
#endif
#ifdef GPIO_CONSOLE_RTS
  (void)sam3u_configgpio(GPIO_CONSOLE_RTS);
#endif

  /* Configure the console (only) */
#if defined(HAVE_CONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)
  /* Reset and disable receiver and transmitter */

  putreg32((UART_CR_RSTRX | UART_CR_RSTTX | UART_CR_RXDIS | UART_CR_TXDIS),
           SAM_CONSOLE_BASE + SAM_UART_CR_OFFSET);

  /* Disable all interrupts */

  putreg32(0xffffffff, SAM_CONSOLE_BASE + SAM_UART_IDR_OFFSET);

  /* Set up the mode register */

  putreg32(MR_VALUE, SAM_CONSOLE_BASE + SAM_UART_MR_OFFSET);

  /* Configure the console baud */

  putreg32(((SAM_MCK_FREQUENCY + (SAM_CONSOLE_BAUD << 3))/(SAM_CONSOLE_BAUD << 4)),
           SAM_CONSOLE_BASE + SAM_UART_BRGR_OFFSET);

  /* Enable receiver & transmitter */

  putreg32((UART_CR_RXEN | UART_CR_TXEN),
           SAM_CONSOLE_BASE + SAM_UART_CR_OFFSET);
#endif
}


