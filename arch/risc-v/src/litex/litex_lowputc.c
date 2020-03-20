/****************************************************************************
 * arch/risc-v/src/litex/litex_lowputc.c
 *
 *   Copyright (C) 2020 Gregory Nutt. All rights reserved.
 *   Author: hctang <aenrbesaen@126.com>
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

#include <arch/board/board.h>

#include "up_internal.h"
#include "up_arch.h"

#include "litex_config.h"
#include "hardware/litex_memorymap.h"
#include "hardware/litex_uart.h"
#include "litex_clockconfig.h"
#include "litex.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Select UART parameters for the selected console */

#ifdef HAVE_SERIAL_CONSOLE
#  if defined(CONFIG_UART0_SERIAL_CONSOLE)
#    define LITEX_CONSOLE_BASE        LITEX_UART0_BASE
#    define LITEX_CONSOLE_BAUD        CONFIG_UART0_BAUD
#    define LITEX_CONSOLE_BITS        CONFIG_UART0_BITS
#    define LITEX_CONSOLE_PARITY      CONFIG_UART0_PARITY
#    define LITEX_CONSOLE_2STOP       CONFIG_UART0_2STOP
#    define LITEX_CONSOLE_TX          GPIO_UART0_TX
#    define LITEX_CONSOLE_RX          GPIO_UART0_RX
#    define HAVE_UART
#  elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#    define LITEX_CONSOLE_BASE        LITEX_UART1_BASE
#    define LITEX_CONSOLE_BAUD        CONFIG_UART1_BAUD
#    define LITEX_CONSOLE_BITS        CONFIG_UART1_BITS
#    define LITEX_CONSOLE_PARITY      CONFIG_UART1_PARITY
#    define LITEX_CONSOLE_2STOP       CONFIG_UART1_2STOP
#    define LITEX_CONSOLE_TX          GPIO_UART1_TX
#    define LITEX_CONSOLE_RX          GPIO_UART1_RX
#    define HAVE_UART
#  endif
#endif /* HAVE_CONSOLE */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_lowputc
 *
 * Description:
 *   Output one byte on the serial console
 *
 ****************************************************************************/

void up_lowputc(char ch)
{
#ifdef HAVE_SERIAL_CONSOLE
  /* Wait until the TX data register is empty */

  while (getreg8(LITEX_CONSOLE_BASE + UART_TXFULL_OFFSET))
    ;

  /* Then send the character */

  putreg8(ch, LITEX_CONSOLE_BASE + UART_RXTX_OFFSET);
  putreg8(UART_EV_TX, LITEX_CONSOLE_BASE + UART_EV_PENDING_OFFSET);

#endif /* HAVE_CONSOLE */
}

/****************************************************************************
 * Name: litex_lowsetup
 *
 * Description:
 *   This performs basic initialization of the UART used for the serial
 *   console.  Its purpose is to get the console output available as soon
 *   as possible.
 *
 ****************************************************************************/

void litex_lowsetup(void)
{
#if defined(HAVE_UART)

  /* Enable and configure the selected console device */

#if defined(HAVE_SERIAL_CONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)

  /* defualt baudrate set by fpga fabric is 1e6 */

  /* Enable TX */

  putreg8(getreg8(LITEX_CONSOLE_BASE + UART_EV_PENDING_OFFSET),         \
                            LITEX_CONSOLE_BASE + UART_EV_PENDING_OFFSET);
  putreg8(UART_EV_TX, LITEX_CONSOLE_BASE + UART_EV_ENABLE_OFFSET);

#endif /* HAVE_SERIAL_CONSOLE && !CONFIG_SUPPRESS_UART_CONFIG */
#endif /* HAVE_UART */
}
