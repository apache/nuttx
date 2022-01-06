/****************************************************************************
 * boards/arm/stm32/emw3162/src/emw3162.h
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

#ifndef __BOARDS_ARM_STM32_EMW3162_SRC_EMW3162_H
#define __BOARDS_ARM_STM32_EMW3162_SRC_EMW3162_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <arch/stm32/chip.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LEDs */

#define GPIO_LED1       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN0)

#define GPIO_LED2       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN1)

/* WLAN chip */

#define SDIO_WLAN0_SLOTNO 0 /* EMW3162 board has only one sdio device */
#define SDIO_WLAN0_MINOR  0 /* Register "wlan0" device */

#define GPIO_WLAN0_RESET (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                          GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN5)

#define GPIO_WLAN0_PWRDN (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                          GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN1)

#define GPIO_WLAN0_32K_CLK (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                            GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN8)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Called either by board_initialize() if CONFIG_BOARD_LATE_INITIALIZE or
 *   by board_app_initialize if CONFIG_BOARDCTL is selected.  This
 *   function initializes and configures all on-board features appropriate
 *   for the selected configuration.
 *
 ****************************************************************************/

#if defined(CONFIG_BOARDCTL) || defined(CONFIG_BOARD_LATE_INITIALIZE)
int stm32_bringup(void);
#endif

/****************************************************************************
 * Name: emw3162_wlan_initialize
 *
 * Description:
 *   Initialize wlan hardware and driver for EMW3162 board.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_EMW3162_WLAN
int emw3162_wlan_initialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_STM32_EMW3162_SRC_EMW3162_H */
