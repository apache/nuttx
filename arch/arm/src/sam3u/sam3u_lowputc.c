/**************************************************************************
 * arch/arm/src/sam3u/sam3u_lowputc.c
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
#include "sam3u_pmc.h"

/**************************************************************************
 * Private Definitions
 **************************************************************************/

/* Configuration **********************************************************/

#ifndef CONFIG_USART0_ISUART
#  undef CONFIG_SAM3U_USART0
#endif
#ifndef CONFIG_USART1_ISUART
#  undef CONFIG_SAM3U_USART1
#endif
#ifndef CONFIG_USART2_ISUART
#  undef CONFIG_SAM3U_USART2
#endif
#ifndef CONFIG_USART3_ISUART
#  undef CONFIG_SAM3U_USART3
#endif

/* Is there a serial console? It could be on the UART, or USARTn */

#if defined(CONFIG_UART_SERIAL_CONSOLE) && defined(CONFIG_SAM3U_UART)
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_USART0_SERIAL_CONSOLE) && defined(CONFIG_SAM3U_USART0)
#  undef CONFIG_USART_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_USART1_SERIAL_CONSOLE) && defined(CONFIG_SAM3U_USART1)
#  undef CONFIG_USART_SERIAL_CONSOLE
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_USART2_SERIAL_CONSOLE) && defined(CONFIG_SAM3U_USART2)
#  undef CONFIG_USART_SERIAL_CONSOLE
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_USART3_SERIAL_CONSOLE) && defined(CONFIG_SAM3U_USART3)
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
#  define SAM3U_CONSOLE_BASE     SAM3U_UART_BASE
#  define SAM3U_CONSOLE_BAUD     CONFIG_USART_BAUD
#  define SAM3U_CONSOLE_BITS     CONFIG_USART_BITS
#  define SAM3U_CONSOLE_PARITY   CONFIG_USART_PARITY
#  define SAM3U_CONSOLE_2STOP    CONFIG_USART_2STOP
#elif defined(CONFIG_USART0_SERIAL_CONSOLE)
#  define SAM3U_CONSOLE_BASE     SAM3U_USART0_BASE
#  define SAM3U_CONSOLE_BAUD     CONFIG_USART0_BAUD
#  define SAM3U_CONSOLE_BITS     CONFIG_USART0_BITS
#  define SAM3U_CONSOLE_PARITY   CONFIG_USART0_PARITY
#  define SAM3U_CONSOLE_2STOP    CONFIG_USART0_2STOP
#elif defined(CONFIG_USART1_SERIAL_CONSOLE)
#  define SAM3U_CONSOLE_BASE     SAM3U_USART1_BASE
#  define SAM3U_CONSOLE_BAUD     CONFIG_USART1_BAUD
#  define SAM3U_CONSOLE_BITS     CONFIG_USART1_BITS
#  define SAM3U_CONSOLE_PARITY   CONFIG_USART1_PARITY
#  define SAM3U_CONSOLE_2STOP    CONFIG_USART1_2STOP
#elif defined(CONFIG_USART2_SERIAL_CONSOLE)
#  define SAM3U_CONSOLE_BASE     SAM3U_USART2_BASE
#  define SAM3U_CONSOLE_BAUD     CONFIG_USART2_BAUD
#  define SAM3U_CONSOLE_BITS     CONFIG_USART2_BITS
#  define SAM3U_CONSOLE_PARITY   CONFIG_USART2_PARITY
#  define SAM3U_CONSOLE_2STOP    CONFIG_USART2_2STOP
#elif defined(CONFIG_USART3_SERIAL_CONSOLE)
#  define SAM3U_CONSOLE_BASE     SAM3U_USART3_BASE
#  define SAM3U_CONSOLE_BAUD     CONFIG_USART3_BAUD
#  define SAM3U_CONSOLE_BITS     CONFIG_USART3_BITS
#  define SAM3U_CONSOLE_PARITY   CONFIG_USART3_PARITY
#  define SAM3U_CONSOLE_2STOP    CONFIG_USART3_2STOP
#else
#  error "No CONFIG_U[S]ARTn_SERIAL_CONSOLE Setting"
#endif

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
#warning "To be provided"
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

#warning "To be provided"
  /* Enable clocking for the UART */

  regval = getreg32(SAM3U_PMC_PCER);
  regval |= (1 << SAM3U_PID_UART);
  putreg32(regval, SAM3U_PMC_PCER);
}


