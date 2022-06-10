/****************************************************************************
 * boards/arm/stm32l4/nucleo-l432kc/src/nucleo-l432kc.h
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

#ifndef __BOARDS_ARM_STM32L4_NUCLEO_L432KC_SRC_NUCLEO_L432KC_H
#define __BOARDS_ARM_STM32L4_NUCLEO_L432KC_SRC_NUCLEO_L432KC_H

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
#define HAVE_AT45DB           1

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

/* Check if we can support AT45DB FLASH file system */

#if !defined(CONFIG_STM32L4_SPI1) || !defined(CONFIG_MTD_AT45DB)
#  undef HAVE_AT45DB
#endif

/* LED.  User LD3: the green LED is a user LED connected to Arduino signal
 * D13 corresponding to MCU I/O PB3 (pin 26)
 * target.
 *
 * - When the I/O is HIGH value, the LED is on.
 * - When the I/O is LOW, the LED is off.
 */

#define GPIO_LD3 \
  (GPIO_PORTB | GPIO_PIN3 | GPIO_OUTPUT_CLEAR | GPIO_OUTPUT | GPIO_PULLUP | \
   GPIO_SPEED_50MHz)
#define LED_DRIVER_PATH "/dev/userleds"

/* SPI chip selects */

#ifdef CONFIG_MTD_AT45DB
#  define AT45DB_SPI1_CS \
     (GPIO_PORTA | GPIO_PIN11 | GPIO_OUTPUT_SET | GPIO_OUTPUT | GPIO_PUSHPULL | \
      GPIO_SPEED_50MHz)
#endif

/* GPIO pins used by the GPIO Subsystem */

#define BOARD_NGPIOIN     1 /* Amount of GPIO Input pins */
#define BOARD_NGPIOOUT    1 /* Amount of GPIO Output pins */
#define BOARD_NGPIOINT    1 /* Amount of GPIO Input w/ Interruption pins */

#define GPIO_IN1          (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTA|GPIO_PIN0)
#define GPIO_OUT1         (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                           GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN1)

#define GPIO_INT1         (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTA|GPIO_PIN3)

/* ZERO CROSS pin definition */

#define GPIO_ZEROCROSS    (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTA|GPIO_PIN0)

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Global driver instances */

#ifdef CONFIG_STM32L4_SPI1
extern struct spi_dev_s *g_spi1;
#endif
#ifdef CONFIG_STM32L4_SPI2
extern struct spi_dev_s *g_spi2;
#endif

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32l4_gpio_initialize
 *
 * Description:
 *   Initialize GPIO drivers for use with /apps/examples/gpio
 *
 ****************************************************************************/

#ifdef CONFIG_DEV_GPIO
int stm32l4_gpio_initialize(void);
#endif

/****************************************************************************
 * Name: stm32l4_spiregister
 *
 * Description:
 *   Called to register spi character driver of initialized
 *   spi device for the Nucleo-L432KC board.
 *
 ****************************************************************************/

void stm32l4_spiregister(void);

/****************************************************************************
 * Name: stm32l4_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins.
 *
 ****************************************************************************/

void stm32l4_spiinitialize(void);

/****************************************************************************
 * Name: stm32l4_usbinitialize
 *
 * Description:
 *   Called to setup USB-related GPIO pins.
 *
 ****************************************************************************/

void stm32l4_usbinitialize(void);

/****************************************************************************
 * Name: stm32l4_pwm_setup
 *
 * Description:
 *   Initialize PWM and register the PWM device.
 *
 ****************************************************************************/

#ifdef CONFIG_PWM
int stm32l4_pwm_setup(void);
#endif

/****************************************************************************
 * Name: stm32l4_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC driver.
 *
 ****************************************************************************/

#ifdef CONFIG_ADC
int stm32l4_adc_setup(void);
#endif

/****************************************************************************
 * Name: stm32_dac7571initialize
 *
 * Description:
 *   Initialize and register the DAC7571 driver.
 *
 ****************************************************************************/

#ifdef CONFIG_DAC7571
int stm32_dac7571initialize(const char *devpath);
#endif

/****************************************************************************
 * Name: stm32_at45dbinitialize
 *
 * Description:
 *   Initialize and register the AT45DB driver.
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_AT45DB
int stm32_at45dbinitialize(int minor);
#endif

/****************************************************************************
 * Name: stm32_ina226initialize
 *
 * Description:
 *   Initialize and register the INA226 driver.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_INA226
int stm32_ina226initialize(const char *devpath);
#endif

/****************************************************************************
 * Name: stm32_ina219initialize
 *
 * Description:
 *   Initialize and register the INA219 driver.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_INA219
int stm32_ina219initialize(const char *devpath);
#endif

/****************************************************************************
 * Name: stm32_zerocross_initialize
 *
 * Description:
 *   Initialize and register the zero cross driver
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_ZEROCROSS
int stm32_zerocross_initialize(void);
#endif

/****************************************************************************
 * Name: board_timer_driver_initialize
 *
 * Description:
 *   Initialize and register a timer
 *
 ****************************************************************************/

#ifdef CONFIG_TIMER
int board_timer_driver_initialize(const char *devpath, int timer);
#endif

/****************************************************************************
 * Name: stm32l4_qencoder_initialize
 *
 * Description:
 *   Initialize and register a qencoder
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_QENCODER
int stm32l4_qencoder_initialize(const char *devpath, int timer);
#endif

#endif /* __BOARDS_ARM_STM32L4_NUCLEO_L432KC_SRC_NUCLEO_L432KC_H */
