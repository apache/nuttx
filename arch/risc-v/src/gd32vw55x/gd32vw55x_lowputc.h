/****************************************************************************
 * arch/risc-v/src/gd32vw55x/gd32vw55x_lowputc.h
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

#ifndef __ARCH_RISCV_SRC_GD32VW55X_GD32VW55X_LOWPUTC_H
#define __ARCH_RISCV_SRC_GD32VW55X_GD32VW55X_LOWPUTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: gd32vw55x_lowsetup
 *
 * Description:
 *   Called early in the boot sequence to configure the serial console
 *   UART (pins, baud rate, format) so that debug output is available.
 *
 ****************************************************************************/

void gd32vw55x_lowsetup(void);

/****************************************************************************
 * Name: gd32vw55x_uart_configure
 *
 * Description:
 *   Configure baud rate and 8N1 format of a USART/UART peripheral and
 *   enable transmitter/receiver.  uartclk is the peripheral kernel clock
 *   (PCLK1 or PCLK2 depending on the instance).
 *
 ****************************************************************************/

void gd32vw55x_uart_configure(uint32_t uart_base, uint32_t uartclk,
                              uint32_t baud);

/****************************************************************************
 * Name: gd32vw55x_gpio_config_af
 *
 * Description:
 *   Minimal helper to place one pin in alternate-function mode.  (The
 *   full GPIO driver arrives with the peripheral phase; the console only
 *   needs AF muxing.)
 *
 ****************************************************************************/

void gd32vw55x_gpio_config_af(uint32_t port_base, int pin, int af,
                              int pull);

#endif /* __ARCH_RISCV_SRC_GD32VW55X_GD32VW55X_LOWPUTC_H */
