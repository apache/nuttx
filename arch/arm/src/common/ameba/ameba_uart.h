/****************************************************************************
 * arch/arm/src/common/ameba/ameba_uart.h
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

#ifndef __ARCH_ARM_SRC_COMMON_AMEBA_AMEBA_UART_H
#define __ARCH_ARM_SRC_COMMON_AMEBA_AMEBA_UART_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The Ameba high-speed UART controllers exposed to NuttX.  UART2 is shared
 * with Bluetooth on this family and is not wired for general serial use, so
 * only UART0 and UART1 are registered by boards.  TX/RX pins are given with
 * the same AMEBA_PA()/AMEBA_PB() PinName encoding used by the GPIO driver
 * (see ameba_gpio.h); any pad can be routed to a UART through the pin mux.
 */

#define AMEBA_UART0           0
#define AMEBA_UART1           1

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: ameba_uart_register
 *
 * Description:
 *   Configure one Ameba high-speed UART and register it with the NuttX
 *   serial upper half at the given device path (e.g. "/dev/ttyS1").  The
 *   LOG-UART console is owned separately (arch/.../ameba_loguart.c) and
 *   already occupies /dev/ttyS0, so boards register the general-purpose
 *   UARTs starting at /dev/ttyS1.
 *
 * Input Parameters:
 *   path  - The serial device path to register (e.g. "/dev/ttyS1").
 *   uart  - The controller index, AMEBA_UART0 or AMEBA_UART1.
 *   txpin - The TX pad, encoded with AMEBA_PA()/AMEBA_PB().
 *   rxpin - The RX pad, encoded with AMEBA_PA()/AMEBA_PB().
 *   baud  - The initial baud rate.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ameba_uart_register(const char *path, int uart, uint8_t txpin,
                        uint8_t rxpin, uint32_t baud);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ARCH_ARM_SRC_COMMON_AMEBA_AMEBA_UART_H */
