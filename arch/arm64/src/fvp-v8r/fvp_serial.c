/***************************************************************************
 * arch/arm64/src/fvp-v8r/fvp_serial.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
 ***************************************************************************/

/***************************************************************************
 * Included Files
 ***************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <nuttx/debug.h>

#include <nuttx/serial/uart_pl011.h>
#include "arm64_internal.h"

/***************************************************************************
 * Pre-processor definitions
 ***************************************************************************/

#if defined(CONFIG_UART0_SERIAL_CONSOLE)
  #define CONSOLE_DEV g_pl011_port0
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
  #define CONSOLE_DEV g_pl011_port1
#elif defined(CONFIG_UART2_SERIAL_CONSOLE)
  #define CONSOLE_DEV g_pl011_port2
#elif defined(CONFIG_UART3_SERIAL_CONSOLE)
  #define CONSOLE_DEV g_pl011_port3
#endif

/* QEMU-specific configuration parameters */

#define UART0_BASEADDR (0x9c090000)
#define UART0_CLK_FREQ (24000000)
#define UART0_IRQ (37)

#define UART1_BASEADDR (0x9c0a0000)
#define UART1_CLK_FREQ (24000000)
#define UART1_IRQ (38)

#define UART2_BASEADDR (0x9c0b0000)
#define UART2_CLK_FREQ (24000000)
#define UART2_IRQ (39)

#define UART3_BASEADDR (0x9c0c0000)
#define UART3_CLK_FREQ (24000000)
#define UART3_IRQ (40)

/***************************************************************************
 * Private data
 ***************************************************************************/

#ifdef CONFIG_UART0_SERIALDRIVER
static char g_uart0_rx_buf[CONFIG_UART0_RXBUFSIZE];
static char g_uart0_tx_buf[CONFIG_UART0_TXBUFSIZE];

static struct pl011_uart_port_s g_pl011_port0 =
{
  .config =
    {
      .baseaddr = (void *)UART0_BASEADDR,
      .baud_rate = CONFIG_UART0_BAUD,
      .irq_num = UART0_IRQ,
      .sbsa = false,
      .sys_clk_freq = UART0_CLK_FREQ,
    },

  .uart =
    {
      .recv =
        {
          .buffer = g_uart0_rx_buf,
          .size = CONFIG_UART0_RXBUFSIZE,
        },
      .xmit =
        {
          .buffer = g_uart0_tx_buf,
          .size = CONFIG_UART0_TXBUFSIZE,
        },
    },
};
#endif

#ifdef CONFIG_UART1_SERIALDRIVER
static char g_uart1_rx_buf[CONFIG_UART1_RXBUFSIZE];
static char g_uart1_tx_buf[CONFIG_UART1_TXBUFSIZE];

static struct pl011_uart_port_s g_pl011_port1 =
{
  .config =
    {
      .baseaddr = (void *)UART1_BASEADDR,
      .baud_rate = CONFIG_UART1_BAUD,
      .irq_num = UART1_IRQ,
      .sbsa = false,
      .sys_clk_freq = UART1_CLK_FREQ,
    },

  .uart =
    {
      .recv =
        {
          .buffer = g_uart1_rx_buf,
          .size = CONFIG_UART1_RXBUFSIZE,
        },
      .xmit =
        {
          .buffer = g_uart1_tx_buf,
          .size = CONFIG_UART1_TXBUFSIZE,
        },
    },
};
#endif

#ifdef CONFIG_UART2_SERIALDRIVER
static char g_uart2_rx_buf[CONFIG_UART2_RXBUFSIZE];
static char g_uart2_tx_buf[CONFIG_UART2_TXBUFSIZE];

static struct pl011_uart_port_s g_pl011_port2 =
{
  .config =
    {
      .baseaddr = (void *)UART2_BASEADDR,
      .baud_rate = CONFIG_UART2_BAUD,
      .irq_num = UART2_IRQ,
      .sbsa = false,
      .sys_clk_freq = UART2_CLK_FREQ,
    },

  .uart =
    {
      .recv =
        {
          .buffer = g_uart2_rx_buf,
          .size = CONFIG_UART2_RXBUFSIZE,
        },
      .xmit =
        {
          .buffer = g_uart2_tx_buf,
          .size = CONFIG_UART2_TXBUFSIZE,
        },
    },
};
#endif

#ifdef CONFIG_UART3_SERIALDRIVER
static char g_uart3_rx_buf[CONFIG_UART3_RXBUFSIZE];
static char g_uart3_tx_buf[CONFIG_UART3_TXBUFSIZE];

static struct pl011_uart_port_s g_pl011_port3 =
{
  .config =
    {
      .baseaddr = (void *)UART3_BASEADDR,
      .baud_rate = CONFIG_UART3_BAUD,
      .irq_num = UART3_IRQ,
      .sbsa = false,
      .sys_clk_freq = UART3_CLK_FREQ,
    },

  .uart =
    {
      .recv =
        {
          .buffer = g_uart3_rx_buf,
          .size = CONFIG_UART3_RXBUFSIZE,
        },
      .xmit =
        {
          .buffer = g_uart3_tx_buf,
          .size = CONFIG_UART3_TXBUFSIZE,
        },
    },
};
#endif

#ifdef USE_SERIALDRIVER

/***************************************************************************
 * Public Functions
 ***************************************************************************/

/***************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes
 *
 ***************************************************************************/

#ifdef CONSOLE_DEV
void up_putc(int ch)
{
  pl011_putc(&CONSOLE_DEV.uart, ch);
}
#endif

/***************************************************************************
 * Name: arm64_earlyserialinit
 *
 * Description:
 *   see arm64_internal.h
 *
 ***************************************************************************/

void arm64_earlyserialinit(void)
{
  /* Enable the console UART.  The other UARTs will be initialized if and
   * when they are first opened.
   */

#ifdef CONSOLE_DEV
  pl011_dev_init(&CONSOLE_DEV);
  CONSOLE_DEV.uart.isconsole = true;
  CONSOLE_DEV.uart.ops->setup(&CONSOLE_DEV.uart); /* Early set up */
#endif
}

/***************************************************************************
 * Name: arm64_serialinit
 *
 * Description:
 *   see arm64_internal.h
 *
 ***************************************************************************/

void arm64_serialinit(void)
{
#ifdef CONSOLE_DEV
  pl011_dev_init(&CONSOLE_DEV);
  CONSOLE_DEV.uart.isconsole = true;
  uart_register("/dev/console", &CONSOLE_DEV.uart);
#endif

#ifdef CONFIG_UART0_SERIALDRIVER
  pl011_dev_init(&g_pl011_port0);
  uart_register("/dev/ttyS0", &g_pl011_port0.uart);
#endif

#ifdef CONFIG_UART1_SERIALDRIVER
  pl011_dev_init(&g_pl011_port1);
  uart_register("/dev/ttyS1", &g_pl011_port1.uart);
#endif

#ifdef CONFIG_UART2_SERIALDRIVER
  pl011_dev_init(&g_pl011_port2);
  uart_register("/dev/ttyS2", &g_pl011_port2.uart);
#endif

#ifdef CONFIG_UART3_SERIALDRIVER
  pl011_dev_init(&g_pl011_port3);
  uart_register("/dev/ttyS3", &g_pl011_port3.uart);
#endif
}

#endif /* USE_SERIALDRIVER */
