/****************************************************************************
 * boards/arm/stm32n6/nucleo-n657x0-q/src/nucleo-n657x0-q.h
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

#ifndef __BOARDS_ARM_STM32N6_NUCLEO_N657X0_Q_SRC_NUCLEO_N657X0_Q_H
#define __BOARDS_ARM_STM32N6_NUCLEO_N657X0_Q_SRC_NUCLEO_N657X0_Q_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#include "stm32_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LED definitions **********************************************************/

/* The Nucleo-N657X0-Q has three user LEDs (UM3417 silkscreen):
 *
 *   LD5  PG10  Red
 *   LD6  PG0   Green
 *   LD7  PG8   Blue
 *
 * - When the I/O is LOW,  the LED is on.
 * - When the I/O is HIGH, the LED is off.
 */

#define GPIO_LD5       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHZ | \
                        GPIO_OUTPUT_SET | GPIO_PORTG | GPIO_PIN10)
#define GPIO_LD6       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHZ | \
                        GPIO_OUTPUT_SET | GPIO_PORTG | GPIO_PIN0)
#define GPIO_LD7       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHZ | \
                        GPIO_OUTPUT_SET | GPIO_PORTG | GPIO_PIN8)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void);

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_STM32N6_NUCLEO_N657X0_Q_SRC_NUCLEO_N657X0_Q_H */
