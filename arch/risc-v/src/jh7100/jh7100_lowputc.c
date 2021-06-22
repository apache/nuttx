/****************************************************************************
 * arch/risc-v/src/jh7100/jh7100_lowputc.c
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

#include "jh7100_config.h"
#include "hardware/jh7100_memorymap.h"
#include "hardware/jh7100_uart.h"
#include "jh7100_clockconfig.h"
#include "jh7100.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Select UART parameters for the selected console */

#ifdef HAVE_SERIAL_CONSOLE
#  if defined(CONFIG_UART0_SERIAL_CONSOLE)
#    define JH7100_CONSOLE_BASE        JH7100_UART0_BASE
#    define JH7100_CONSOLE_BAUD        CONFIG_UART0_BAUD
#    define JH7100_CONSOLE_BITS        CONFIG_UART0_BITS
#    define JH7100_CONSOLE_PARITY      CONFIG_UART0_PARITY
#    define JH7100_CONSOLE_2STOP       CONFIG_UART0_2STOP
#    define JH7100_CONSOLE_TX          GPIO_UART0_TX
#    define JH7100_CONSOLE_RX          GPIO_UART0_RX
#    define HAVE_UART
#  elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#    define JH7100_CONSOLE_BASE        JH7100_UART1_BASE
#    define JH7100_CONSOLE_BAUD        CONFIG_UART1_BAUD
#    define JH7100_CONSOLE_BITS        CONFIG_UART1_BITS
#    define JH7100_CONSOLE_PARITY      CONFIG_UART1_PARITY
#    define JH7100_CONSOLE_2STOP       CONFIG_UART1_2STOP
#    define JH7100_CONSOLE_TX          GPIO_UART1_TX
#    define JH7100_CONSOLE_RX          GPIO_UART1_RX
#    define HAVE_UART
#  endif
#endif /* HAVE_CONSOLE */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: riscv_lowputc
 *
 * Description:
 *   Output one byte on the serial console
 *
 ****************************************************************************/

void riscv_lowputc(char ch)
{
#ifdef HAVE_SERIAL_CONSOLE
  /* Wait until the TX data register is empty */

  while ((!(getreg32(JH7100_UART0_LSR) & DW_LSR_TRANS_EMPTY)))
    {
    }

  /* Then send the character */

  putreg32(ch, JH7100_UART0_TXDATA);

#endif /* HAVE_CONSOLE */
}

/****************************************************************************
 * Name: jh7100_lowsetup
 *
 * Description:
 *   This performs basic initialization of the UART used for the serial
 *   console.  Its purpose is to get the console output available as soon
 *   as possible.
 *
 ****************************************************************************/

void jh7100_lowsetup(void)
{
#if defined(HAVE_UART)

  /* Enable and configure the selected console device */

#if defined(HAVE_SERIAL_CONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)

  /* Configure the UART Baud Rate */

  /* Enable TX */

#endif /* HAVE_SERIAL_CONSOLE && !CONFIG_SUPPRESS_UART_CONFIG */
#endif /* HAVE_UART */
}
