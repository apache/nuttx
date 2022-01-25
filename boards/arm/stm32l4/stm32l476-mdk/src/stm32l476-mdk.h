/****************************************************************************
 * boards/arm/stm32l4/stm32l476-mdk/src/stm32l476-mdk.h
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

#ifndef __BOARDS_ARM_STM32L4_STM32L476_MDK_SRC_STM32L476_MDK_H
#define __BOARDS_ARM_STM32L4_STM32L476_MDK_SRC_STM32L476_MDK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>

#include "stm32l4_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#define HAVE_PROC             1
#define HAVE_RTC_DRIVER       1
#define HAVE_USERLED_DRIVER   1

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

/* Check if we have the LED driver */

#if !defined(CONFIG_USERLED) || !defined(CONFIG_USERLED_LOWER)
#  undef HAVE_USERLED_DRIVER
#endif

/* LEDs.
 * The Reference Moto Mod contains three LEDs.  Two LEDs, are by convention,
 * used to indicate the Reference Moto Mod battery state of charge, and the
 * other is available for you to use in your applications.
 *
 *   1. The red LED on PD7.  Part of the (rear-firing) red/green LED.
 *   2. The green LED on PE7.  Part of the (rear-firing) red/green LED.
 *   3. The white (top-firing) LED on PE8
 *
 * When the I/O is HIGH value, the LED is OFF.
 * When the I/O is LOW, the LED is ON.
 */

#define GPIO_LED_RED \
  (GPIO_PORTD | GPIO_PIN7 | GPIO_OUTPUT_SET | GPIO_OUTPUT | GPIO_PUSHPULL | \
   GPIO_PULLUP | GPIO_SPEED_50MHz)

#define GPIO_LED_GREEN \
  (GPIO_PORTE | GPIO_PIN7 | GPIO_OUTPUT_SET | GPIO_OUTPUT | GPIO_PUSHPULL | \
   GPIO_PULLUP | GPIO_SPEED_50MHz)

#define GPIO_LED_WHITE \
  (GPIO_PORTE | GPIO_PIN8 | GPIO_OUTPUT_SET | GPIO_OUTPUT | GPIO_PUSHPULL | \
   GPIO_PULLUP | GPIO_SPEED_50MHz)

/* BUTTONS -- NOTE that all have EXTI interrupts configured */

#define MIN_IRQBUTTON   BUTTON_POWER
#define MAX_IRQBUTTON   BUTTON_POWER
#define NUM_IRQBUTTONS  1

#define GPIO_BTN_POWER  (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTD|GPIO_PIN2)

/* SPI1 off */

#define GPIO_SPI1_MOSI_OFF (GPIO_INPUT | GPIO_PULLDOWN | \
                            GPIO_PORTE | GPIO_PIN15)
#define GPIO_SPI1_MISO_OFF (GPIO_INPUT | GPIO_PULLDOWN | \
                            GPIO_PORTE | GPIO_PIN14)
#define GPIO_SPI1_SCK_OFF  (GPIO_INPUT | GPIO_PULLDOWN | \
                            GPIO_PORTE | GPIO_PIN13)
#define GPIO_SPI1_NSS_OFF  (GPIO_INPUT | GPIO_PULLDOWN | \
                            GPIO_PORTE | GPIO_PIN12)

/* Devices on the onboard I2C bus.
 *
 * Note that these are unshifted addresses.
 */

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Global driver instances */

#ifdef CONFIG_STM32_SPI1
extern struct spi_dev_s *g_spi1;
#endif
#ifdef CONFIG_STM32_SPI2
extern struct spi_dev_s *g_spi2;
#endif

/****************************************************************************
 * Public Functions Definitions
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
 *   CONFIG_BOARD_LATE_INITIALIZE=y && CONFIG_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void);

/****************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins.
 *
 ****************************************************************************/

void stm32_spiinitialize(void);

#endif /* __BOARDS_ARM_STM32L4_STM32L476_MDK_SRC_STM32L476_MDK_H */
