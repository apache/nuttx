/****************************************************************************
 * boards/arm/eoss3/quickfeather/include/board.h
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
 ****************************************************************************/

#ifndef __BOARDS_ARM_EOSS3_QUICKFEATHER_INCLUDE_BOARD_H
#define __BOARDS_ARM_EOSS3_QUICKFEATHER_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "eoss3_gpio.h"

#define BOARD_HCLK_FREQUENCY 79790000
#define BOARD_CPU_FREQUENCY  79790000

/* LED definitions **********************************************************/

#define LED_STARTED       0  /* LED off */
#define LED_HEAPALLOCATE  0  /* LED off */
#define LED_IRQSENABLED   0  /* LED off */
#define LED_STACKCREATED  1  /* LED on */
#define LED_INIRQ         2  /* LED no change */
#define LED_SIGNAL        2  /* LED no change */
#define LED_ASSERTION     2  /* LED no change */
#define LED_PANIC         3  /* LED flashing */

#define GPIO_LED_B       ((IO_REG_P18 << GPIO_REG_BIT_SHIFT) | \
                         (GPIO_REG_EN_MASK) | \
                         ((PAD_FUNC_3) << GPIO_CTRL_SHIFT) | \
                         GPIO_PIN18)
#define GPIO_LED_G       ((IO_REG_P21 << GPIO_REG_BIT_SHIFT) | \
                         (GPIO_REG_EN_MASK) | \
                         ((PAD_FUNC_3) << GPIO_CTRL_SHIFT) | \
                         GPIO_PIN21)
#define GPIO_LED_R       ((IO_REG_P22 << GPIO_REG_BIT_SHIFT) | \
                         (GPIO_REG_EN_MASK) | \
                         ((PAD_FUNC_3) << GPIO_CTRL_SHIFT) | \
                         GPIO_PIN22)

/* UART definitions *********************************************************/

#define GPIO_UART_RX    ((UART_RXD_SEL_P45 << GPIO_INPUT_SEL_SHIFT) | \
                        ((PAD_OEN | PAD_REN) << GPIO_CTRL_SHIFT) | \
                        GPIO_PIN45)

#define GPIO_UART_TX    (((PAD_FUNC_3) << GPIO_CTRL_SHIFT) | GPIO_PIN44)

#endif /* __BOARDS_ARM_EOSS3_QUICKFEATHER_INCLUDE_BOARD_H */
