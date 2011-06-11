/******************************************************************************
 * arch/avr/src/at90usb/at90usb_lowconsole.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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
 ******************************************************************************/

/******************************************************************************
 * Included Files
 ******************************************************************************/

#include <nuttx/config.h>
#include "at90usb_config.h"

#include <assert.h>
#include <debug.h>

#include <arch/irq.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "up_internal.h"

#include "at90usb_internal.h"

/******************************************************************************
 * Private Definitions
 ******************************************************************************/

/* Select USART parameters for the selected console -- there is only USART1 */

#if defined(CONFIG_USART1_SERIAL_CONSOLE)
#  define AVR_CONSOLE_BASE     AVR_USART1_BASE
#  define AVR_CONSOLE_BAUD     CONFIG_USART1_BAUD
#  define AVR_CONSOLE_BITS     CONFIG_USART1_BITS
#  define AVR_CONSOLE_PARITY   CONFIG_USART1_PARITY
#  define AVR_CONSOLE_2STOP    CONFIG_USART1_2STOP
#else
#  error "No CONFIG_USARTn_SERIAL_CONSOLE Setting"
#endif

/* Baud rate settings for normal and double speed settings  */

#define AVR_NORMAL_UBRR1 \
  (((BOARD_CPU_CLOCK / 16) + (CONFIG_USART1_BAUD / 2)) / (CONFIG_USART1_BAUD)) - 1)

#define AVR_DBLSPEED_UBRR1 \
  (((BOARD_CPU_CLOCK / 8) + (CONFIG_USART1_BAUD / 2)) / (CONFIG_USART1_BAUD)) - 1)

/* Select normal or double speed baud settings.  This is a trade-off between the
 * sampling rate and the accuracy of the divisor for high baud rates.
 *
 * As examples, consider:
 *
 *   BOARD_CPU_CLOCK=8MHz and BAUD=115200:
 *     AVR_NORMAL_UBRR1 = 4 (rounded), actual baud = 125,000
 *     AVR_DBLSPEED_UBRR1 = 9 (rounded), actual baud = 111,111
 *
 *   BOARD_CPU_CLOCK=8MHz and BAUD=9600:
 *     AVR_NORMAL_UBRR1 = 52 (rounded), actual baud = 9615
 *     AVR_DBLSPEED_UBRR1 = 104 (rounded), actual baud = 9615
 */
 
#undef HAVE_DOUBLE_SPEED
#if BOARD_CPU_CLOCK <= 4000000
#  if CONFIG_USART1_BAUD <= 9600
#    define AVR_UBRR1 AVR_NORMAL_UBRR1
#  else
#    define AVR_UBRR1 AVR_DBLSPEED_UBRR1
#    define HAVE_DOUBLE_SPEED 1
#  endif
#elif BOARD_CPU_CLOCK <= 8000000
#  if CONFIG_USART1_BAUD <= 19200
#    define AVR_UBRR1 AVR_NORMAL_UBRR1
#  else
#    define AVR_UBRR1 AVR_DBLSPEED_UBRR1
#    define HAVE_DOUBLE_SPEED 1
#  endif
#elif BOARD_CPU_CLOCK <= 12000000
#  if CONFIG_USART1_BAUD <= 28800
#    define AVR_UBRR1 AVR_NORMAL_UBRR1
#  else
#    define AVR_UBRR1 AVR_DBLSPEED_UBRR1
#    define HAVE_DOUBLE_SPEED 1
#  endif
#else
#  if CONFIG_USART1_BAUD <= 38400
#    define AVR_UBRR1 AVR_NORMAL_UBRR1
#  else
#    define AVR_UBRR1 AVR_DBLSPEED_UBRR1
#    define HAVE_DOUBLE_SPEED 1
#  endif
#endif

/******************************************************************************
 * Private Types
 ******************************************************************************/

/******************************************************************************
 * Private Function Prototypes
 ******************************************************************************/

/******************************************************************************
 * Global Variables
 ******************************************************************************/

/******************************************************************************
 * Private Variables
 ******************************************************************************/

/******************************************************************************
 * Private Functions
 ******************************************************************************/

/******************************************************************************
 * Public Functions
 ******************************************************************************/

/******************************************************************************
 * Name: usart1_reset
 *
 * Description:
 *   Reset a USART.
 *
 ******************************************************************************/

#ifdef HAVE_USART_DEVICE
void usart1_reset(void)
{
# warning "Missing Logic"
}
#endif

/******************************************************************************
 * Name: usart1_configure
 *
 * Description:
 *   Configure USART1.
 *
 ******************************************************************************/

#ifdef HAVE_USART_DEVICE
void usart1_configure(void)
{
# warning "Missing Logic"
}
#endif

/******************************************************************************
 * Name: up_consoleinit
 *
 * Description:
 *   Initialize a console for debug output.  This function is called very
 *   early in the intialization sequence to configure the serial console uart
 *   (only).
 *
 ******************************************************************************/

void up_consoleinit(void)
{
#ifdef HAVE_SERIAL_CONSOLE
  usart1_configure();
#endif
}

/******************************************************************************
 * Name: up_lowputc
 *
 * Description:
 *   Output one byte on the serial console
 *
 ******************************************************************************/

void up_lowputc(char ch)
{
#ifdef HAVE_SERIAL_CONSOLE
# warning "Missing Logic"
#endif
}

