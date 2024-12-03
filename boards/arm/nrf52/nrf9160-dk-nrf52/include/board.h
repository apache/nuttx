/****************************************************************************
 * boards/arm/nrf52/nrf9160-dk-nrf52/include/board.h
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

#ifndef __BOARDS_ARM_NRF52_NRF9160_DK_NRF52_INCLUDE_BOARD_H
#define __BOARDS_ARM_NRF52_NRF9160_DK_NRF52_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

#define BOARD_SYSTICK_CLOCK         (64000000)

/* LED definitions **********************************************************/

#define BOARD_NLEDS       0

/* Button definitions *******************************************************/

#define NUM_BUTTONS       0

/* UART Pins ****************************************************************/

/* UART0 is connected to the virtual COM1 port:
 *   UART0_RX - P0-03
 *   UART0_TX - P0-05
 */

#define BOARD_UART0_RX_PIN  (GPIO_INPUT  | GPIO_PORT0 | GPIO_PIN(3))
#define BOARD_UART0_TX_PIN  (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT0 | GPIO_PIN(5))

#endif /* __BOARDS_ARM_NRF52_NRF9160_DK_NRF52_INCLUDE_BOARD_H */
