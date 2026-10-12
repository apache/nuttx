/****************************************************************************
 * boards/arm/stm32f1/nvmpg-stm32f1/src/nvmpg-stm32f1.h
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

#ifndef __BOARDS_ARM_STM32F1_NVMPG_STM32F1_SRC_NVMPG_STM32F1_H
#define __BOARDS_ARM_STM32F1_NVMPG_STM32F1_SRC_NVMPG_STM32F1_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LCD: ILI9225 on a bit-banged 8080 16-bit bus *****************************
 *
 *   D0..D15  PB0..PB15  (needs JTAG disabled to free PB3/PB4; SWD is kept)
 *   /WR      PC10
 *   RS       PC11       (0 = index register, 1 = data)
 *   /CS      PC12
 *   /RD      PA15
 *   /RESET   not connected to the MCU
 */

#define GPIO_LCD_DATA   (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTB)
#define GPIO_LCD_WR     (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN10)
#define GPIO_LCD_RS     (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN11)
#define GPIO_LCD_CS     (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN12)
#define GPIO_LCD_RD     (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN15)

/* Keys: active low, external pull-ups (floating inputs).  The order below
 * matches the key index used by the original firmware (code = index + 2).
 */

#define GPIO_KEY(port, pin) (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|\
                             (port)|(pin))

#define GPIO_KEY0       GPIO_KEY(GPIO_PORTA, GPIO_PIN2)   /* Axis Up    0x02 */
#define GPIO_KEY1       GPIO_KEY(GPIO_PORTA, GPIO_PIN1)   /* Axis Down  0x03 */
#define GPIO_KEY2       GPIO_KEY(GPIO_PORTA, GPIO_PIN0)   /* Spindle    0x04 */
#define GPIO_KEY3       GPIO_KEY(GPIO_PORTC, GPIO_PIN3)   /* Step x1..  0x05 */
#define GPIO_KEY4       GPIO_KEY(GPIO_PORTC, GPIO_PIN2)   /* Home       0x06 */
#define GPIO_KEY5       GPIO_KEY(GPIO_PORTC, GPIO_PIN1)   /* GOTOZ      0x07 */
#define GPIO_KEY6       GPIO_KEY(GPIO_PORTC, GPIO_PIN0)   /* Zero       0x08 */
#define GPIO_KEY7       GPIO_KEY(GPIO_PORTC, GPIO_PIN15)  /* 1/2        0x09 */
#define GPIO_KEY8       GPIO_KEY(GPIO_PORTC, GPIO_PIN14)  /* Side right 0x0a */
#define GPIO_KEY9       GPIO_KEY(GPIO_PORTA, GPIO_PIN3)   /* Side left  0x0b */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization.  Called from
 *   board_late_initialize() (CONFIG_BOARD_LATE_INITIALIZE=y) or from
 *   board_app_initialize().
 *
 ****************************************************************************/

int stm32_bringup(void);

/****************************************************************************
 * Name: stm32_gpio_initialize
 *
 * Description:
 *   Register the keys as /dev/gpio0 .. /dev/gpio9.
 *
 ****************************************************************************/

#ifdef CONFIG_DEV_GPIO
int stm32_gpio_initialize(void);
#endif

#endif /* __BOARDS_ARM_STM32F1_NVMPG_STM32F1_SRC_NVMPG_STM32F1_H */
