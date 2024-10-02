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

#ifdef CONFIG_UART_PL011

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

void pl011_earlyserialinit(void);

void pl011_serialinit(void);

#ifdef CONFIG_UART_PL011_PLATFORMIF
/* If needed, implement platform specific process such as enabling pl011
 * to reduce power consumption.
 */

int pl011_platform_setup(uint32_t base);
int pl011_platform_shutdown(uint32_t base);
#endif  /* CONFIG_UART_PL011_PLATFORMIF */

#endif  /* CONFIG_UART_PL011 */
#endif /* __INCLUDE_NUTTX_SERIAL_UART_PL011_H */
