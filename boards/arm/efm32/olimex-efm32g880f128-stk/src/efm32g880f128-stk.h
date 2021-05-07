/****************************************************************************
 * boards/arm/efm32/olimex-efm32g880f128-stk/src/efm32g880f128-stk.h
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

#ifndef __BOARDS_ARM_EFM32_OLIMEX_EFM32G880F128_STK_SRC_EFM32G880F128_STK_H
#define __BOARDS_ARM_EFM32_OLIMEX_EFM32G880F128_STK_SRC_EFM32G880F128_STK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Buttons:
 *
 * The Olimex board has four buttons, BUT1-4.  Each is grounded and so should
 * have a weak pull-up so that it will be sensed as "1" when open and "0"
 * when closed.
 *
 * --------------------- ---------------------
 * PIN                   CONNECTIONS
 * --------------------- ---------------------
 * PE0/PCNT0_S0IN/U0_TX  BUT1, EXT-18
 * PE1/PCNT0_S1IN/U0_RX  BUT2, EXT-19
 * PE2/ACMP0_O           BUT3, EXT-20
 * PE3/ACMP1_O           BUT4, EXT-21
 * --------------------- ---------------------
 */

#ifdef CONFIG_EFM32_GPIO_IRQ
#  define GPIO_BUTTON_1 (GPIO_INPUT_PULLUP|GPIO_INT_BOTH|GPIO_PORTE|GPIO_PIN0)
#  define GPIO_BUTTON_2 (GPIO_INPUT_PULLUP|GPIO_INT_BOTH|GPIO_PORTE|GPIO_PIN1)
#  define GPIO_BUTTON_3 (GPIO_INPUT_PULLUP|GPIO_INT_BOTH|GPIO_PORTE|GPIO_PIN2)
#  define GPIO_BUTTON_4 (GPIO_INPUT_PULLUP|GPIO_INT_BOTH|GPIO_PORTE|GPIO_PIN3)

#  define GPIO_IRQ_BUTTON_1 EFM32_IRQ_EXTI0
#  define GPIO_IRQ_BUTTON_2 EFM32_IRQ_EXTI1
#  define GPIO_IRQ_BUTTON_3 EFM32_IRQ_EXTI2
#  define GPIO_IRQ_BUTTON_4 EFM32_IRQ_EXTI3
#else
#  define GPIO_BUTTON_1 (GPIO_INPUT_PULLUP|GPIO_PORTE|GPIO_PIN0)
#  define GPIO_BUTTON_2 (GPIO_INPUT_PULLUP|GPIO_PORTE|GPIO_PIN1)
#  define GPIO_BUTTON_3 (GPIO_INPUT_PULLUP|GPIO_PORTE|GPIO_PIN2)
#  define GPIO_BUTTON_4 (GPIO_INPUT_PULLUP|GPIO_PORTE|GPIO_PIN3)
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __BOARDS_ARM_EFM32_OLIMEX_EFM32G880F128_STK_SRC_EFM32G880F128_STK_H */
