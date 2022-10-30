/****************************************************************************
 * boards/arm/stm32wb/flipperzero/src/flipperzero.h
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

#ifndef __BOARDS_ARM_STM32WB_FLIPPERZERO_SRC_FLIPPERZERO_H
#define __BOARDS_ARM_STM32WB_FLIPPERZERO_SRC_FLIPPERZERO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#include <stm32wb_gpio.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#define HAVE_PROC             1
#define HAVE_RTC_DRIVER       1

#if !defined(CONFIG_FS_PROCFS)
#  undef HAVE_PROC
#endif

#if defined(HAVE_PROC) && defined(CONFIG_DISABLE_MOUNTPOINT)
#  warning Mountpoints disabled.  No procfs support
#  undef HAVE_PROC
#endif

/* Check if we can support the RTC driver */

#if !defined(CONFIG_RTC) || !defined(CONFIG_RTC_DRIVER)
#  undef HAVE_RTC_DRIVER
#endif

/* Flipper Zero GPIOs *******************************************************/

/* Button definitions *******************************************************/

#define GPIO_BTN_UP     (GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI | GPIO_PORTB | GPIO_PIN10)
#define GPIO_BTN_DOWN   (GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI | GPIO_PORTC | GPIO_PIN6)
#define GPIO_BTN_LEFT   (GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI | GPIO_PORTB | GPIO_PIN11)
#define GPIO_BTN_RIGH   (GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI | GPIO_PORTB | GPIO_PIN12)
#define GPIO_BTN_BACK   (GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI | GPIO_PORTC | GPIO_PIN13)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32wb_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins.
 *
 ****************************************************************************/

#ifdef CONFIG_SPI
void weak_function stm32wb_spidev_initialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_STM32WB_FLIPPERZERO_SRC_FLIPPERZERO_H */
