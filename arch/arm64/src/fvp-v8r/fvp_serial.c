/***************************************************************************
 * arch/arm64/src/fvp-v8r/fvp_serial.c
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

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <nuttx/config.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>

#include "arm64_internal.h"
#include <nuttx/serial/uart_pl011.h>

#ifdef USE_SERIALDRIVER

/***************************************************************************
 * Pre-processor Definitions
 ***************************************************************************/

/* Serial consoles */

#if defined(CONFIG_UART0_SERIAL_CONSOLE)
#define CONSOLE_DEV g_uart0port
#endif // defined(CONFIG_UART0_SERIAL_CONSOLE)

#if defined(CONFIG_UART1_SERIAL_CONSOLE)
#define CONSOLE_DEV g_uart1port
#endif // defined(CONFIG_UART1_SERIAL_CONSOLE)

#if defined(CONFIG_UART2_SERIAL_CONSOLE)
#define CONSOLE_DEV g_uart2port
#endif // defined(CONFIG_UART2_SERIAL_CONSOLE)

#if defined(CONFIG_UART3_SERIAL_CONSOLE)
#define CONSOLE_DEV g_uart3port
#endif // defined(CONFIG_UART3_SERIAL_CONSOLE)

/* TTYS devices */

#if defined(CONFIG_UART0_PL011)
#define TTYS0_DEV g_uart0port
#endif // defined(CONFIG_UART0_PL011)

#if defined(CONFIG_UART1_PL011)
#define TTYS1_DEV g_uart1port
#endif // defined(CONFIG_UART1_PL011)

#if defined(CONFIG_UART2_PL011)
#define TTYS2_DEV g_uart2port
#endif // defined(CONFIG_UART2_PL011)

#if defined(CONFIG_UART3_PL011)
#define TTYS3_DEV g_uart3port
#endif // defined(CONFIG_UART3_PL011)

/***************************************************************************
 * Private Data
 ***************************************************************************/

/* IO buffers */

#if defined(CONFIG_UART0_PL011)
static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_RXBUFSIZE];
#endif // defined(CONFIG_UART0_PL011)

#if defined(CONFIG_UART1_PL011)
static char g_uart1rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_UART1_RXBUFSIZE];
#endif // defined(CONFIG_UART1_PL011)

#if defined(CONFIG_UART2_PL011)
static char g_uart2rxbuffer[CONFIG_UART2_RXBUFSIZE];
static char g_uart2txbuffer[CONFIG_UART2_RXBUFSIZE];
#endif // defined(CONFIG_UART2_PL011)

#if defined(CONFIG_UART3_PL011)
static char g_uart3rxbuffer[CONFIG_UART3_RXBUFSIZE];
static char g_uart3txbuffer[CONFIG_UART3_RXBUFSIZE];
#endif // defined(CONFIG_UART3_PL011)

/* UART devices */

#if defined(CONFIG_UART0_PL011)

static struct pl011_uart_port_s g_uart0priv =
{
    .data =
        {
            .baud_rate = CONFIG_UART0_BAUD,
            .sbsa = false,
        },

    .config =
        {
            .uart = (void *)CONFIG_UART0_BASE,
            .sys_clk_freq = CONFIG_UART0_CLK_FREQ,
        },

    .irq_num = CONFIG_UART0_IRQ,
};

static struct uart_dev_s g_uart0port =
{
    .recv =
        {
            .size = CONFIG_UART0_RXBUFSIZE,
            .buffer = g_uart0rxbuffer,
        },

    .xmit =
        {
            .size = CONFIG_UART0_TXBUFSIZE,
            .buffer = g_uart0txbuffer,
        },

    .priv = &g_uart0priv,
};

#endif // defined(CONFIG_UART0_PL011)

#if defined(CONFIG_UART1_PL011)

static struct pl011_uart_port_s g_uart1priv =
{
    .data =
        {
            .baud_rate = CONFIG_UART1_BAUD,
            .sbsa = false,
        },

    .config =
        {
            .uart = (void *)CONFIG_UART1_BASE,
            .sys_clk_freq = CONFIG_UART1_CLK_FREQ,
        },

    .irq_num = CONFIG_UART1_IRQ,
};

static struct uart_dev_s g_uart1port =
{
    .recv =
        {
            .size = CONFIG_UART1_RXBUFSIZE,
            .buffer = g_uart1rxbuffer,
        },

    .xmit =
        {
            .size = CONFIG_UART1_TXBUFSIZE,
            .buffer = g_uart1txbuffer,
        },

    .priv = &g_uart1priv,
};

#endif // defined(CONFIG_UART1_PL011)

#if defined(CONFIG_UART2_PL011)

static struct pl011_uart_port_s g_uart2priv =
{
    .data =
        {
            .baud_rate = CONFIG_UART2_BAUD,
            .sbsa = false,
        },

    .config =
        {
            .uart = (void *)CONFIG_UART2_BASE,
            .sys_clk_freq = CONFIG_UART2_CLK_FREQ,
        },

    .irq_num = CONFIG_UART2_IRQ,
};

static struct uart_dev_s g_uart2port =
{
    .recv =
        {
            .size = CONFIG_UART2_RXBUFSIZE,
            .buffer = g_uart2rxbuffer,
        },

    .xmit =
        {
            .size = CONFIG_UART2_TXBUFSIZE,
            .buffer = g_uart2txbuffer,
        },

    .priv = &g_uart2priv,
};

#endif // defined(CONFIG_UART2_PL011)

#if defined(CONFIG_UART3_PL011)

static struct pl011_uart_port_s g_uart3priv =
{
    .data =
        {
            .baud_rate = CONFIG_UART3_BAUD,
            .sbsa = false,
        },

    .config =
        {
            .uart = (void *)CONFIG_UART3_BASE,
            .sys_clk_freq = CONFIG_UART3_CLK_FREQ,
        },

    .irq_num = CONFIG_UART3_IRQ,
};

static struct uart_dev_s g_uart3port =
{
    .recv =
        {
            .size = CONFIG_UART3_RXBUFSIZE,
            .buffer = g_uart3rxbuffer,
        },

    .xmit =
        {
            .size = CONFIG_UART3_TXBUFSIZE,
            .buffer = g_uart3txbuffer,
        },

    .priv = &g_uart3priv,
};

#endif // defined(CONFIG_UART3_PL011)

/***************************************************************************
 * Public Functions
 ***************************************************************************/

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

#if defined(CONSOLE_DEV)
  CONSOLE_DEV.isconsole = true;
  pl011_init_ops(&CONSOLE_DEV);
  CONSOLE_DEV.ops->setup(&CONSOLE_DEV);
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
#if defined(CONSOLE_DEV)
  pl011_uart_register("/dev/console", &CONSOLE_DEV);
#endif
#if defined(TTYS0_DEV)
  pl011_uart_register("/dev/ttyS0", &TTYS0_DEV);
#endif
#if defined(TTYS1_DEV)
  pl011_uart_register("/dev/ttyS1", &TTYS1_DEV);
#endif
#if defined(TTYS2_DEV)
  pl011_uart_register("/dev/ttyS2", &TTYS2_DEV);
#endif
#if defined(TTYS3_DEV)
  pl011_uart_register("/dev/ttyS3", &TTYS3_DEV);
#endif
}

/***************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes
 *
 ***************************************************************************/

#if defined(CONSOLE_DEV)
int up_putc(int ch)
{
  FAR struct uart_dev_s *dev = &CONSOLE_DEV;

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      dev->ops->send(dev, '\r');
    }

  dev->ops->send(dev, ch);

  return ch;
}
#endif // defined(CONSOLE_DEV)

#endif /* USE_SERIALDRIVER */
