/****************************************************************************
 * arch/arm/src/nrf54l/nrf54l_config.h
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

#ifndef __ARCH_ARM_SRC_NRF54L_NRF54L_CONFIG_H
#define __ARCH_ARM_SRC_NRF54L_NRF54L_CONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/board/board.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* UART configuration *******************************************************/

#undef HAVE_UART_DEVICE
#undef HAVE_UART_CONSOLE

#ifdef CONFIG_NRF54L_UART
#  define HAVE_UART_DEVICE 1
#endif

#if defined(CONFIG_UART0_SERIAL_CONSOLE) && \
      defined(CONFIG_NRF54L_UART0)
#  define HAVE_UART_CONSOLE 1
#elif defined(CONFIG_UART1_SERIAL_CONSOLE) && \
      defined(CONFIG_NRF54L_UART1)
#  define HAVE_UART_CONSOLE 1
#elif defined(CONFIG_UART2_SERIAL_CONSOLE) && \
      defined(CONFIG_NRF54L_UART2)
#  define HAVE_UART_CONSOLE 1
#elif defined(CONFIG_UART3_SERIAL_CONSOLE) && \
      defined(CONFIG_NRF54L_UART3)
#  define HAVE_UART_CONSOLE 1
#elif defined(CONFIG_UART4_SERIAL_CONSOLE) && \
      defined(CONFIG_NRF54L_UART4)
#  define HAVE_UART_CONSOLE 1
#elif defined(CONFIG_UART5_SERIAL_CONSOLE) && \
      defined(CONFIG_NRF54L_UART5)
#  define HAVE_UART_CONSOLE 1
#elif defined(CONFIG_UART6_SERIAL_CONSOLE) && \
      defined(CONFIG_NRF54L_UART6)
#  define HAVE_UART_CONSOLE 1
#endif

#endif /* __ARCH_ARM_SRC_NRF54L_NRF54L_CONFIG_H */
