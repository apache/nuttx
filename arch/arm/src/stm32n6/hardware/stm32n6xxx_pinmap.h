/****************************************************************************
 * arch/arm/src/stm32n6/hardware/stm32n6xxx_pinmap.h
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

#ifndef __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_PINMAP_H
#define __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_PINMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Alternate Pin Functions.
 *
 * Alternative pin selections are provided with a numeric suffix like _1, _2,
 * etc.  Drivers, however, will use the pin selection without the numeric
 * suffix.  Additional definitions are required in the board.h file.  For
 * example, if USART1_TX connects via PE5 on some board, then the following
 * definition should appear in the board.h header file for that board:
 *
 * #define GPIO_USART1_TX GPIO_USART1_TX_1
 *
 * The driver will then automatically configure PE5 as the USART1 TX pin.
 */

/* USART1: PE5=TX (AF7), PE6=RX (AF7) - ST-Link Virtual COM Port */

#define GPIO_USART1_TX_1   (GPIO_ALT | GPIO_AF7 | GPIO_SPEED_50MHZ | GPIO_PUSHPULL | GPIO_PORTE | GPIO_PIN5)
#define GPIO_USART1_RX_1   (GPIO_ALT | GPIO_AF7 | GPIO_SPEED_50MHZ | GPIO_PORTE | GPIO_PIN6)

#endif /* __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_PINMAP_H */
