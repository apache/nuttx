/***************************************************************************
 * arch/arm/src/cxd32xx/cxd32_serial_pl011.c
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
#include <debug.h>

#include <nuttx/serial/uart_pl011.h>

#include "arm_internal.h"

#ifdef USE_SERIALDRIVER

/***************************************************************************
 * Pre-processor Definitions
 ***************************************************************************/

#if defined(CONFIG_UART0_SERIAL_CONSOLE)
#define CONSOLE_DEV g_uart0port
#endif // defined(CONFIG_UART0_SERIAL_CONSOLE)

#if defined(CONFIG_UART0_PL011)
#define TTYS0_DEV g_uart0port
#endif // defined(CONFIG_UART0_PL011)

/***************************************************************************
 * Private Data
 ***************************************************************************/

/* IO buffers */
#if defined(CONFIG_UART0_PL011)
static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_RXBUFSIZE];
#endif // defined(CONFIG_UART0_PL011)

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

/***************************************************************************
 * Public Functions
 ***************************************************************************/

/***************************************************************************
 * Name: arm_earlyserialinit
 *
 * Description:
 *   see arm_internal.h
 *
 ***************************************************************************/

#ifdef USE_EARLYSERIALINIT
void arm_earlyserialinit(void)
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
#endif

/***************************************************************************
 * Name: arm_serialinit
 *
 * Description:
 *   see arm_internal.h
 *
 ***************************************************************************/

void arm_serialinit(void)
{
#if defined(CONSOLE_DEV)
  pl011_uart_register("/dev/console", &CONSOLE_DEV);
#endif
#if defined(TTYS0_DEV)
  pl011_uart_register("/dev/ttyS0", &TTYS0_DEV);
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

#ifdef CONFIG_UART_PL011_PLATFORMIF
/***************************************************************************
 * Name: pl011_platform interface
 *
 * Description:
 *   see drivers/serial/serial_pl011.c
 *        pl011_setup
 *        pl011_shutdown
 *
 ***************************************************************************/

int pl011_platform_setup(uint32_t base)
{
  /* If needed, implement platform specific process such as enabling pl011
   * to reduce power consumption.
   */

  return 0;
}

int pl011_platform_shutdown(uint32_t base)
{
  /* If needed, implement platform specific process such as disabling pl011
   * to reduce power consumption.
   */

  return 0;
}
#endif /* CONFIG_UART_PL011_PLATFORMIF */
#endif /* USE_SERIALDRIVER */
