/****************************************************************************
 * arch/risc-v/src/fe310/fe310_lowputc.c
 *
 *   Copyright (C) 2019 Masayuki Ishikawa. All rights reserved.
 *   Author: Masayuki Ishikawa <masayuki.ishikawa@gmail.com>
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

#include "riscv_internal.h"
#include "riscv_arch.h"

#include "fe310_config.h"
#include "hardware/fe310_memorymap.h"
#include "hardware/fe310_uart.h"
#include "fe310_clockconfig.h"
#include "fe310.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Select UART parameters for the selected console */

#ifdef HAVE_SERIAL_CONSOLE
#  if defined(CONFIG_UART0_SERIAL_CONSOLE)
#    define FE310_CONSOLE_BASE        FE310_UART0_BASE
#    define FE310_CONSOLE_BAUD        CONFIG_UART0_BAUD
#    define FE310_CONSOLE_BITS        CONFIG_UART0_BITS
#    define FE310_CONSOLE_PARITY      CONFIG_UART0_PARITY
#    define FE310_CONSOLE_2STOP       CONFIG_UART0_2STOP
#    define FE310_CONSOLE_TX          GPIO_UART0_TX
#    define FE310_CONSOLE_RX          GPIO_UART0_RX
#    define HAVE_UART
#  elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#    define FE310_CONSOLE_BASE        FE310_UART1_BASE
#    define FE310_CONSOLE_BAUD        CONFIG_UART1_BAUD
#    define FE310_CONSOLE_BITS        CONFIG_UART1_BITS
#    define FE310_CONSOLE_PARITY      CONFIG_UART1_PARITY
#    define FE310_CONSOLE_2STOP       CONFIG_UART1_2STOP
#    define FE310_CONSOLE_TX          GPIO_UART1_TX
#    define FE310_CONSOLE_RX          GPIO_UART1_RX
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

  while ((getreg32(FE310_CONSOLE_BASE + UART_TXDATA_OFFSET) & UART_TX_FULL))
    ;

  /* Then send the character */

  putreg32((uint32_t)ch, FE310_CONSOLE_BASE + UART_TXDATA_OFFSET);

#endif /* HAVE_CONSOLE */
}

/****************************************************************************
 * Name: fe310_lowsetup
 *
 * Description:
 *   This performs basic initialization of the UART used for the serial
 *   console.  Its purpose is to get the console output available as soon
 *   as possible.
 *
 ****************************************************************************/

void fe310_lowsetup(void)
{
#if defined(HAVE_UART)

  /* Enable and configure the selected console device */

#if defined(HAVE_SERIAL_CONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)

  /* Configure the UART Baud Rate */

  uint32_t hfclk = fe310_get_hfclk();
  uint32_t div;

  div  = hfclk / 1152; /* NOTE: To avoid a bug with debugger */
  div /= 100;
  div -= 1;

  putreg32(div, FE310_CONSOLE_BASE + UART_DIV_OFFSET);

  /* Enable TX */

  putreg32(1, FE310_CONSOLE_BASE + UART_TXCTL_OFFSET);
#endif /* HAVE_SERIAL_CONSOLE && !CONFIG_SUPPRESS_UART_CONFIG */
#endif /* HAVE_UART */
}
