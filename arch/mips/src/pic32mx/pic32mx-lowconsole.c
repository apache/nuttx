/******************************************************************************
 * arch/mips/src/pic32mx/pic32mx-lowconsole.c
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

#include <assert.h>
#include <debug.h>

#include <arch/irq.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "up_internal.h"

#include "pic32mx-config.h"
#include "pic32mx-internal.h"

/******************************************************************************
 * Private Definitions
 ******************************************************************************/

/* Select UART parameters for the selected console */

#if defined(CONFIG_UART1_SERIAL_CONSOLE)
#  define AVR32_CONSOLE_BASE     PIC32MX_UART1_K1BASE
#  define AVR32_CONSOLE_BAUD     CONFIG_UART1_BAUD
#  define AVR32_CONSOLE_BITS     CONFIG_UART1_BITS
#  define AVR32_CONSOLE_PARITY   CONFIG_UART1_PARITY
#  define AVR32_CONSOLE_2STOP    CONFIG_UART1_2STOP
#elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#  define AVR32_CONSOLE_BASE     PIC32MX_UART2_K1BASE
#  define AVR32_CONSOLE_BAUD     CONFIG_UART2_BAUD
#  define AVR32_CONSOLE_BITS     CONFIG_UART2_BITS
#  define AVR32_CONSOLE_PARITY   CONFIG_UART2_PARITY
#  define AVR32_CONSOLE_2STOP    CONFIG_UART2_2STOP
#else
#  error "No CONFIG_UARTn_SERIAL_CONSOLE Setting"
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
 * Name: pic32mx_uartputreg
 *
 * Description:
 *   Write a value to a UART register
 *
 ******************************************************************************/

#ifdef HAVE_UART_DEVICE
static inline void pic32mx_uartputreg(uintptr_t uart_base, unsigned int offset,
                                      uint32_t value)
{
  putreg32(value, uart_base + offset);
}
#endif

/******************************************************************************
 * Name: pic32mx_uartgetreg
 *
 * Description:
 *   Get a value from a UART register
 *
 ******************************************************************************/

#ifdef HAVE_UART_DEVICE
static inline uint32_t pic32mx_uartgetreg(uintptr_t uart_base,
                                          unsigned int offset)
{
  return getreg32(uart_base + offset);
}
#endif

/******************************************************************************
 * Name: pic32mx_uartsetbaud
 *
 * Description:
 *   Configure the UART baud rate.
 *
 ******************************************************************************/

#ifdef HAVE_UART_DEVICE
static void pic32mx_uartsetbaud(uintptr_t uart_base, uint32_t baudrate)
{
#warning "Missing logic"
}
#endif

/******************************************************************************
 * Public Functions
 ******************************************************************************/

/******************************************************************************
 * Name: pic32mx_uartreset
 *
 * Description:
 *   Reset a UART.
 *
 ******************************************************************************/

#ifdef HAVE_UART_DEVICE
void pic32mx_uartreset(uintptr_t uart_base)
{
#warning "Missing logic"
}
#endif

/******************************************************************************
 * Name: pic32mx_uartconfigure
 *
 * Description:
 *   Configure a UART as a RS-232 UART.
 *
 ******************************************************************************/

#ifdef HAVE_UART_DEVICE
void pic32mx_uartconfigure(uintptr_t uart_base, uint32_t baud,
                           unsigned int parity, unsigned int nbits, bool stop2)
{
}
#endif

/******************************************************************************
 * Name: pic32mx_consoleinit
 *
 * Description:
 *   Initialize a console for debug output.  This function is called very
 *   early in the intialization sequence to configure the serial console uart
 *   (only).
 *
 ******************************************************************************/

void pic32mx_consoleinit(void)
{
#warning "Missing logic"
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
#warning "Missing logic"
}

