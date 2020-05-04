/****************************************************************************
 * arch/risc-v/src/litex/litex_lowputc.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
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
