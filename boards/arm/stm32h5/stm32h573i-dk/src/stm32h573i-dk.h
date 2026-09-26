/****************************************************************************
 * boards/arm/stm32h5/stm32h573i-dk/src/stm32h573i-dk.h
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

#ifndef __BOARDS_ARM_STM32H5_STM32H573I_DK_SRC_STM32H573I_DK_H
#define __BOARDS_ARM_STM32H5_STM32H573I_DK_SRC_STM32H573I_DK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "stm32_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* User LEDs are active low.  Initially all LEDs are off. */

#define GPIO_LD1        (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHZ | \
                         GPIO_OUTPUT_SET | GPIO_PORTI | GPIO_PIN9)
#define GPIO_LD2        (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHZ | \
                         GPIO_OUTPUT_SET | GPIO_PORTI | GPIO_PIN8)
#define GPIO_LD3        (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHZ | \
                         GPIO_OUTPUT_SET | GPIO_PORTF | GPIO_PIN1)
#define GPIO_LD4        (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHZ | \
                         GPIO_OUTPUT_SET | GPIO_PORTF | GPIO_PIN4)

/* B1 USER is active high. */

#define GPIO_BTN_USER   (GPIO_INPUT | GPIO_PULLDOWN | GPIO_EXTI | \
                         GPIO_PORTC | GPIO_PIN13)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform board initialization from board_late_initialize().
 *
 ****************************************************************************/

int stm32_bringup(void);

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_STM32H5_STM32H573I_DK_SRC_STM32H573I_DK_H */
