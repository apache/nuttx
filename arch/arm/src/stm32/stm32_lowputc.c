/**************************************************************************
 * arch/arm/src/stm32/stm32_lowputc.c
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
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
#include <sys/types.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"
#include "stm32_internal.h"

/**************************************************************************
 * Private Definitions
 **************************************************************************/

/* Configuration **********************************************************/

/* Is there a serial console? */

#if defined(CONFIG_USART1_SERIAL_CONSOLE) && !defined(CONFIG_USART1_DISABLE)
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_USART2_SERIAL_CONSOLE) && !defined(CONFIG_USART2_DISABLE)
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_USART3_SERIAL_CONSOLE) && !defined(CONFIG_USART3_DISABLE)
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  define HAVE_CONSOLE 1
#else
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  undef HAVE_CONSOLE
#endif

/* Select USART parameters for the selected console */

#if defined(CONFIG_USART1_SERIAL_CONSOLE)
#  define STM32_CONSOLE_BASE     STM32_USART1_BASE
#  define STM32_CONSOLE_BAUD     CONFIG_USART1_BAUD
#  define STM32_CONSOLE_BITS     CONFIG_USART1_BITS
#  define STM32_CONSOLE_PARITY   CONFIG_USART1_PARITY
#  define STM32_CONSOLE_2STOP    CONFIG_USART1_2STOP
#elif defined(CONFIG_USART2_SERIAL_CONSOLE)
#  define STM32_CONSOLE_BASE     STM32_USART2_BASE
#  define STM32_CONSOLE_BAUD     CONFIG_USART2_BAUD
#  define STM32_CONSOLE_BITS     CONFIG_USART2_BITS
#  define STM32_CONSOLE_PARITY   CONFIG_USART2_PARITY
#  define STM32_CONSOLE_2STOP    CONFIG_USART2_2STOP
#elif defined(CONFIG_USART3_SERIAL_CONSOLE)
#  define STM32_CONSOLE_BASE     STM32_USART2_BASE
#  define STM32_CONSOLE_BAUD     CONFIG_USART2_BAUD
#  define STM32_CONSOLE_BITS     CONFIG_USART2_BITS
#  define STM32_CONSOLE_PARITY   CONFIG_USART2_PARITY
#  define STM32_CONSOLE_2STOP    CONFIG_USART2_2STOP
#else
#  error "No CONFIG_USARTn_SERIAL_CONSOLE Setting"
#endif

/* Calculate BAUD rate from the SYS clock */

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
#ifdef HAVE_CONSOLE
  /* Wait until the TX FIFO is not full */


  /* Then send the character */
#endif
}

/**************************************************************************
 * Name: up_lowsetup
 *
 * Description:
 *   This performs basic initialization of the USART used for the serial
 *   console.  Its purpose is to get the console output availabe as soon
 *   as possible.
 *
 **************************************************************************/

void up_lowsetup(void)
{
  /* Enable the selected USARTs and configure GPIO pins need byed the
   * the selected USARTs.  NOTE: The serial driver later depends on
   * this pin configuration -- whether or not a serial console is selected.
   */

#ifndef CONFIG_USART1_DISABLE
#endif

#ifndef CONFIG_USART1_DISABLE
#endif

#ifndef CONFIG_USART2_DISABLE
#endif

  /* Enable and configure the selected console device */

#if defined(HAVE_CONSOLE) && !defined(CONFIG_SUPPRESS_USART_CONFIG)
#endif
}


