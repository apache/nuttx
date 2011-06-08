/******************************************************************************
 * arch/avr/src/at90usb/at90usb_lowconsole.c
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
 * Name: usart_setbaudrate
 *
 * Description:
 *   Configure the USART baud rate.
 *
 ******************************************************************************/

#ifdef HAVE_USART_DEVICE
static void usart_setbaudrate(uintptr_t usart_base, uint32_t baudrate)
{
# warning "Missing logic"
}
#endif

/******************************************************************************
 * Public Functions
 ******************************************************************************/

/******************************************************************************
 * Name: usart_reset
 *
 * Description:
 *   Reset a USART.
 *
 ******************************************************************************/

#ifdef HAVE_USART_DEVICE
void usart_reset(uintptr_t usart_base)
{
# warning "Missing Logic"
}
#endif

/******************************************************************************
 * Name: usart_configure
 *
 * Description:
 *   Configure a USART as a RS-232 USART.
 *
 ******************************************************************************/

#ifdef HAVE_USART_DEVICE
void usart_configure(uintptr_t usart_base, uint32_t baud, unsigned int parity,
                     unsigned int nbits, bool stop2)
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
# warning "Missing Logic"
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

