/****************************************************************************
 * boards/arm/rtl8721f/rtl8721f_evb/src/rtl8721f_uart.c
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/param.h>
#include <syslog.h>

#include "ameba_gpio.h"
#include "ameba_uart.h"
#include "rtl8721f_rtl8721f_evb.h"

#ifdef CONFIG_AMEBA_UART

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* One entry per general-purpose UART exposed to NuttX.  The LOG-UART owns
 * the console and /dev/ttyS0, so these ports are registered starting at
 * /dev/ttyS1 in the order listed below.  The TX/RX pads are examples used by
 * the `uart` config (examples/serialrx / serialblaster) -- any pad can be
 * routed to a UART through the pin mux, so adjust them to match your board's
 * wiring.
 */

struct rtl8721f_uart_s
{
  const char *path;             /* Device path (/dev/ttyS1, ...) */
  int         uart;             /* Controller index (AMEBA_UART0/AMEBA_UART1) */
  uint8_t     txpin;            /* TX pad (AMEBA_PA()/AMEBA_PB() encoding) */
  uint8_t     rxpin;            /* RX pad (AMEBA_PA()/AMEBA_PB() encoding) */
  uint32_t    baud;             /* Initial baud rate */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct rtl8721f_uart_s g_uart_ports[] =
{
  {
    "/dev/ttyS1", AMEBA_UART0, AMEBA_PA(24), AMEBA_PA(25), 115200
  },
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rtl8721f_uart_initialize
 *
 * Description:
 *   Register the board's general-purpose UART ports with the NuttX serial
 *   upper half.
 *
 ****************************************************************************/

int rtl8721f_uart_initialize(void)
{
  int ret;
  size_t i;

  for (i = 0; i < nitems(g_uart_ports); i++)
    {
      ret = ameba_uart_register(g_uart_ports[i].path, g_uart_ports[i].uart,
                                g_uart_ports[i].txpin, g_uart_ports[i].rxpin,
                                g_uart_ports[i].baud);
      if (ret < 0)
        {
          syslog(LOG_ERR,
                 "ERROR: ameba_uart_register(%s) failed: %d\n",
                 g_uart_ports[i].path, ret);
          return ret;
        }
    }

  return OK;
}

#endif /* CONFIG_AMEBA_UART */
