/**************************************************************************
 * arch/risc-v/src/nr5m100/nr5_lowputc.c
 *
 *   Copyright (C) 2016 Ken Pettit. All rights reserved.
 *   Author: Ken Pettit <pettitkd@gmail.com>
 *
 *   Modified for RISC-V:
 *
 *   Copyright (C) 2016 Ken Pettit. All rights reserved.
 *   Author: Ken Pettit <pettitkd@gmail.com>
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

#include <arch/board/board.h>

#include "up_internal.h"
#include "up_arch.h"

#include "nr5_config.h"
#include "nr5.h"

/**************************************************************************
 * Pre-processor Definitions
 **************************************************************************/

/* Select UART parameters for the selected console */

#ifdef HAVE_SERIAL_CONSOLE
#  if defined(CONFIG_UART1_SERIAL_CONSOLE)
#    define NR5_CONSOLE_BASE        NR5_UART1_BASE
#    define NR5_CONSOLE_BAUD        CONFIG_UART1_BAUD
#    define NR5_CONSOLE_BITS        CONFIG_UART1_BITS
#    define NR5_CONSOLE_PARITY      CONFIG_UART1_PARITY
#    define NR5_CONSOLE_2STOP       CONFIG_UART1_2STOP
#    define NR5_CONSOLE_TX          GPIO_UART1_TX
#    define NR5_CONSOLE_RX          GPIO_UART1_RX
#    define HAVE_UART
#  elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#    define NR5_CONSOLE_BASE        NR5_UART1_BASE
#    define NR5_CONSOLE_BAUD        CONFIG_UART1_BAUD
#    define NR5_CONSOLE_BITS        CONFIG_UART1_BITS
#    define NR5_CONSOLE_PARITY      CONFIG_UART1_PARITY
#    define NR5_CONSOLE_2STOP       CONFIG_UART1_2STOP
#    define NR5_CONSOLE_TX          GPIO_UART1_TX
#    define NR5_CONSOLE_RX          GPIO_UART1_RX
#    define HAVE_UART
#  endif

  /* Calculate UART BAUD rate divider */

#  if defined(CONFIG_NR5_NR5M1XX)

    /* Baud rate for standard UART:
     *
     * In case of oversampling by 16, the equation is:
     *   UARTDIV = fCK / 32 / baud
     */

#    define NR5_UARTDIV \
      ((NR5_HCLK_FREQUENCY >> 5) / NR5_CONSOLE_BAUD)

#  endif /* CONFIG_NR5_NR5M1XX */
#endif /* HAVE_CONSOLE */

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
#ifdef HAVE_SERIAL_CONSOLE
  /* Wait until the TX data register is empty */

  while ((getreg32(NR5_CONSOLE_BASE + NR5_UART_STATUS_REG_OFFSET) & NR5_UART_STATUS_TX_EMPTY) == 0)
    ;

  /* Then send the character */

  putreg32((uint32_t)ch, NR5_CONSOLE_BASE + NR5_UART_TX_REG_OFFSET);

#endif /* HAVE_CONSOLE */
}

/**************************************************************************
 * Name: nr5_lowsetup
 *
 * Description:
 *   This performs basic initialization of the UART used for the serial
 *   console.  Its purpose is to get the console output availabe as soon
 *   as possible.
 *
 **************************************************************************/

#if defined(CONFIG_NR5_NR5M1XX)

void nr5_lowsetup(void)
{
#if defined(HAVE_UART)

  /* Enable and configure the selected console device */

#if defined(HAVE_SERIAL_CONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)

  /* Configure the UART Baud Rate */

  putreg32(NR5_UARTDIV, NR5_CONSOLE_BASE + NR5_UART_BAUD_RATE_OFFSET);

  /* Configure the RX interrupt */

  putreg32(NR5_UART_CTRL_ENABLE_RX_IRQ, NR5_CONSOLE_BASE + NR5_UART_CTRL_REG_OFFSET);

#endif /* HAVE_SERIAL_CONSOLE && !CONFIG_SUPPRESS_UART_CONFIG */
#endif /* HAVE_UART */
}

#else
#  error "Unsupported NR5 chip"
#endif
