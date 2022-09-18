/****************************************************************************
 * arch/arm/src/nrf53/nrf53_config.h
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

#ifndef __ARCH_ARM_SRC_NRF53_NRF53_CONFIG_H
#define __ARCH_ARM_SRC_NRF53_NRF53_CONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/board/board.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Make sure that no unsupported UART, I2C master, or SPI master peripherals
 * are enabled.
 */

/* Map logical UART names (Just for simplicity of naming) */

#undef HAVE_UART0
#undef HAVE_UART1

#ifdef CONFIG_NRF53_UART0
#  define HAVE_UART0 1
#endif

#ifdef CONFIG_NRF53_UART1
#  define HAVE_UART1 1
#endif

/* Check if we have a UART device */

#undef CONFIG_NRF53_HAVE_UART
#undef HAVE_UART_DEVICE

#if defined(HAVE_UART0)
#  define HAVE_UART_DEVICE 1
#endif

#if defined(HAVE_UART1)
#  define HAVE_UART_DEVICE 1
#endif

/* Is there a serial console? There should be at most one defined. */

#undef HAVE_UART_CONSOLE

#if defined(CONFIG_UART0_SERIAL_CONSOLE) && defined(HAVE_UART0)
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  define HAVE_UART_CONSOLE 1
#endif

#if defined(CONFIG_UART1_SERIAL_CONSOLE) && defined(HAVE_UART1)
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  define HAVE_UART_CONSOLE 1
#endif

#endif /* __ARCH_ARM_SRC_NRF53_NRF53_CONFIG_H */
