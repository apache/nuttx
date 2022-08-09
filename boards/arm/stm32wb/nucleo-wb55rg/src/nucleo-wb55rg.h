/****************************************************************************
 * boards/arm/stm32wb/nucleo-wb55rg/src/nucleo-wb55rg.h
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

#ifndef __BOARDS_ARM_STM32WB_NUCLEO_WB55RG_SRC_NUCLEO_WB55RG_H
#define __BOARDS_ARM_STM32WB_NUCLEO_WB55RG_SRC_NUCLEO_WB55RG_H

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

/* Nucleo-wb55rg GPIOs ******************************************************/

/* Led definitions **********************************************************/

#define GPIO_LED1       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_CLEAR | \
                         GPIO_PORTB | GPIO_PIN5)
#define GPIO_LED2       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_CLEAR | \
                         GPIO_PORTB | GPIO_PIN0)
#define GPIO_LED3       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_CLEAR | \
                         GPIO_PORTB | GPIO_PIN1)

#define GPIO_LED_BLUE   GPIO_LED1
#define GPIO_LED_GREEN  GPIO_LED2
#define GPIO_LED_RED    GPIO_LED3

/* Button definitions *******************************************************/

#define MIN_IRQBUTTON   BUTTON_SW1
#define MAX_IRQBUTTON   BUTTON_SW3
#define NUM_IRQBUTTONS  3

#define GPIO_BTN_SW1    (GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI | GPIO_PORTC | GPIO_PIN4)
#define GPIO_BTN_SW2    (GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI | GPIO_PORTD | GPIO_PIN0)
#define GPIO_BTN_SW3    (GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI | GPIO_PORTD | GPIO_PIN1)

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

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_STM32WB_NUCLEO_WB55RG_SRC_NUCLEO_WB55RG_H */
