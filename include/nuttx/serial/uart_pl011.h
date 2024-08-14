/****************************************************************************
 * include/nuttx/serial/uart_pl011.h
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

#include <nuttx/serial/serial.h>
#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/***************************************************************************
 * Public Types
 ***************************************************************************/

/* Configuration specific to the PL011 UART interface. */

struct pl011_config
{
  void *uart;            /* The base address of the PL011 interface. */
  uint32_t sys_clk_freq; /* System clock frequency used by PL011 interface. */
};

/* Device data structure */

struct pl011_data
{
  uint32_t baud_rate;
  bool sbsa;
};

struct pl011_uart_port_s
{
  struct pl011_data data;
  struct pl011_config config;
  unsigned int irq_num;
};

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/***************************************************************************
 * Name: pl011_init_ops
 *
 * Description:
 *   Initialize the `ops` member of a PL011 UART device. This function is
 *   useful for setting up a PL011 device in earlyserialinit for use in a
 *   function such as `up_putc`.
 *
 ***************************************************************************/

void pl011_init_ops(FAR uart_dev_t *dev);

/***************************************************************************
 * Name: pl011_uart_register
 *
 * Description:
 *   Register a PL011 UART device. This is a wrapper around `uart_register()`
 *   which initializes the `ops` member of the device to the PL011 operations
 *   implementation.
 *
 * Returns:
 *   0 on success, a negated errno on failure.
 *
 ***************************************************************************/

int pl011_uart_register(char *path, FAR uart_dev_t *dev);

#ifdef CONFIG_UART_PL011_PLATFORMIF

/* If needed, implement platform specific process such as enabling pl011
 * to reduce power consumption.
 */

int pl011_platform_setup(uint32_t base);
int pl011_platform_shutdown(uint32_t base);

#endif /* CONFIG_UART_PL011_PLATFORMIF */

#endif /* __INCLUDE_NUTTX_SERIAL_UART_PL011_H */
