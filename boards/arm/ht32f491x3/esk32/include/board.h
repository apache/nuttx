/****************************************************************************
 * boards/arm/ht32f491x3/esk32/include/board.h
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

#ifndef __BOARDS_ARM_HT32F491X3_ESK32_INCLUDE_BOARD_H
#define __BOARDS_ARM_HT32F491X3_ESK32_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Board USART Pin Mapping
 ****************************************************************************/

/* Populate BOARD_USARTx_* for each routed USART on the board. The selected
 * console still comes from CONFIG_USARTx_SERIAL_CONSOLE in the defconfig.
 */

#define BOARD_USART1_GPIO_CLKEN        (1u << 0)
#define BOARD_USART1_TX_GPIO_BASE      0x40020000u
#define BOARD_USART1_RX_GPIO_BASE      0x40020000u
#define BOARD_USART1_TX_PIN            9u
#define BOARD_USART1_RX_PIN            10u
#define BOARD_USART1_TX_AF             7u
#define BOARD_USART1_RX_AF             7u

/****************************************************************************
 * Board PWM Pin Mapping
 ****************************************************************************/

#if defined(CONFIG_HT32F491X3_TMR1_PWM)  || defined(CONFIG_HT32F491X3_TMR2_PWM) || \
    defined(CONFIG_HT32F491X3_TMR4_PWM)  || defined(CONFIG_HT32F491X3_TMR9_PWM) || \
    defined(CONFIG_HT32F491X3_TMR10_PWM) || defined(CONFIG_HT32F491X3_TMR11_PWM) || \
    defined(CONFIG_HT32F491X3_TMR12_PWM) || defined(CONFIG_HT32F491X3_TMR13_PWM) || \
    defined(CONFIG_HT32F491X3_TMR14_PWM)
#  error "esk32 currently exposes PWM only through TMR3"
#endif

#if defined(CONFIG_HT32F491X3_TMR3_PWM)
#  if CONFIG_HT32F491X3_TMR3_CHANNEL == 1
#    define BOARD_TMR3_PWM_GPIO_CLKEN  (1u << 0)
#    define BOARD_TMR3_PWM_GPIO_BASE   0x40020000u
#    define BOARD_TMR3_PWM_GPIO_PIN    6u
#    define BOARD_TMR3_PWM_GPIO_AF     2u
#  elif CONFIG_HT32F491X3_TMR3_CHANNEL == 2
#    define BOARD_TMR3_PWM_GPIO_CLKEN  (1u << 0)
#    define BOARD_TMR3_PWM_GPIO_BASE   0x40020000u
#    define BOARD_TMR3_PWM_GPIO_PIN    7u
#    define BOARD_TMR3_PWM_GPIO_AF     2u
#  elif CONFIG_HT32F491X3_TMR3_CHANNEL == 3
#    define BOARD_TMR3_PWM_GPIO_CLKEN  (1u << 1)
#    define BOARD_TMR3_PWM_GPIO_BASE   0x40020400u
#    define BOARD_TMR3_PWM_GPIO_PIN    0u
#    define BOARD_TMR3_PWM_GPIO_AF     2u
#  elif CONFIG_HT32F491X3_TMR3_CHANNEL == 4
#    define BOARD_TMR3_PWM_GPIO_CLKEN  (1u << 1)
#    define BOARD_TMR3_PWM_GPIO_BASE   0x40020400u
#    define BOARD_TMR3_PWM_GPIO_PIN    1u
#    define BOARD_TMR3_PWM_GPIO_AF     2u
#  else
#    error "Unsupported CONFIG_HT32F491X3_TMR3_CHANNEL value"
#  endif
#endif

/****************************************************************************
 * Board LED Pin Mapping
 ****************************************************************************/

#define BOARD_LED2_GPIO_CLKEN          (1u << 3)
#define BOARD_LED2_GPIO_BASE           0x40020c00u
#define BOARD_LED2_GPIO_PIN            13u

#define BOARD_LED3_GPIO_CLKEN          (1u << 3)
#define BOARD_LED3_GPIO_BASE           0x40020c00u
#define BOARD_LED3_GPIO_PIN            14u

#define BOARD_LED4_GPIO_CLKEN          (1u << 3)
#define BOARD_LED4_GPIO_BASE           0x40020c00u
#define BOARD_LED4_GPIO_PIN            15u

#define BOARD_LED2                     0
#define BOARD_LED3                     1
#define BOARD_LED4                     2
#define BOARD_NLEDS                    3

#define BOARD_LED2_BIT                 (1 << BOARD_LED2)
#define BOARD_LED3_BIT                 (1 << BOARD_LED3)
#define BOARD_LED4_BIT                 (1 << BOARD_LED4)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void ht32f491x3_clockconfig(void);
void ht32f491x3_boardinitialize(void);

#ifdef CONFIG_PWM
int ht32_pwm_setup(void);
#endif

#if defined(CONFIG_USERLED) && !defined(CONFIG_ARCH_LEDS)
uint32_t board_userled_initialize(void);
void board_userled(int led, bool ledon);
void board_userled_all(uint32_t ledset);
#endif

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void);
#endif

#endif /* __BOARDS_ARM_HT32F491X3_ESK32_INCLUDE_BOARD_H */
