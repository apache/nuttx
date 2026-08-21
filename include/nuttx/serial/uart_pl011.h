/****************************************************************************
 * include/nuttx/serial/uart_pl011.h
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

#ifndef __INCLUDE_NUTTX_SERIAL_UART_PL011_H
#define __INCLUDE_NUTTX_SERIAL_UART_PL011_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/serial/serial.h>>

#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct pl011_regs;        /* Forward opaque type declaration */

/* Configuration settings for a PL011 UART port */

struct pl011_config
{
  FAR volatile struct pl011_regs *baseaddr;
  uint32_t sys_clk_freq;
  unsigned int irq_num;
  uint32_t baud_rate;
  bool sbsa;
};

/* PL011 UART device */

struct pl011_uart_port_s
{
  struct uart_dev_s uart;     /* Underlying UART device */
  struct pl011_config config; /* Configuration settings */
  spinlock_t lock;            /* Lock */
  bool inited;                /* Initialization tracker */
};

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

void pl011_dev_init(FAR struct pl011_uart_port_s *priv);

void pl011_putc(struct uart_dev_s *dev, int ch);

#endif /* __INCLUDE_NUTTX_SERIAL_UART_PL011_H */
