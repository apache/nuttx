/****************************************************************************
 * arch/arm/src/am67/am67_serial.c
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
#include <nuttx/serial/serial.h>
#include <nuttx/serial/uart_16550.h>

#ifdef CONFIG_SERIAL_TERMIOS
#  include <termios.h>
#endif

#include "arm_internal.h"

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#if defined(USE_SERIALDRIVER) /* && defined(HAVE_UART_DEVICE)*/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: open_uart
 *
 * Description:
 *   Initialize UART by clearing the register at offset 0x20 from base.
 *
 ****************************************************************************/

static void open_uart(void)
{
  putreg32(0, CONFIG_16550_UART0_BASE + 0x20);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_serialinit
 *
 * Description:
 *   Initialize the serial port by performing early initialization, main
 *   initialization, and opening the UART.
 *
 ****************************************************************************/

void arm_serialinit(void)
{
  u16550_earlyserialinit();
  u16550_serialinit();
  open_uart();
}

/****************************************************************************
 * Name: uart_getreg
 *
 * Description:
 *   Read a value from the UART register at the specified offset from the
 *   base address.
 *
 ****************************************************************************/

uart_datawidth_t uart_getreg(FAR struct u16550_s *priv, unsigned int offset)
{
  return getreg32(priv->uartbase + offset);
}

/****************************************************************************
 * Name: uart_putreg
 *
 * Description:
 *   Write a value to the UART register at the specified offset from the
 *   base address.
 *
 ****************************************************************************/

void uart_putreg(FAR struct u16550_s *priv, unsigned int offset,
                 uart_datawidth_t value)
{
  putreg32(value, priv->uartbase + offset);
}

#endif /* USE_SERIALDRIVER */
