/****************************************************************************
 * boards/arm/stm32l4/nucleo-l452re/src/nucleo-l452re.h
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

#ifndef __BOARDS_ARM_STM32L4_NUCLEO_L452RE_SRC_NUCLEO_L452RE_H
#define __BOARDS_ARM_STM32L4_NUCLEO_L452RE_SRC_NUCLEO_L452RE_H

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
#define HAVE_MMCSD 1

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

#if !defined(CONFIG_STM32L4_SDIO) || !defined(CONFIG_MMCSD) || \
    !defined(CONFIG_MMCSD_SDIO)
#  undef HAVE_MMCSD
#endif

/* How many SPI modules does this chip support? */

#if STM32L4_NSPI < 1
#  undef CONFIG_STM32L4_SPI1
#  undef CONFIG_STM32L4_SPI2
#  undef CONFIG_STM32L4_SPI3
#elif STM32L4_NSPI < 2
#  undef CONFIG_STM32L4_SPI2
#  undef CONFIG_STM32L4_SPI3
#elif STM32L4_NSPI < 3
#  undef CONFIG_STM32L4_SPI3
#endif

/* Nucleo-L452RE GPIOs ******************************************************/

/* LED.  User LD2: the green LED is a user LED connected to Arduino signal
 * D13 corresponding to MCU I/O PA5 (pin 21) or PB13 (pin 34) depending on
 * the STM32 target.
 *
 * - When the I/O is HIGH value, the LED is on.
 * - When the I/O is LOW, the LED is off.
 */

#define GPIO_LD2        (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | \
                         GPIO_OUTPUT_CLEAR | GPIO_PORTA | GPIO_PIN5)

/* Button definitions *******************************************************/

/* B1 USER:
 * the user button is connected to the I/O PC13 (pin 2) of the STM32
 * microcontroller.
 */

#define MIN_IRQBUTTON   BUTTON_USER
#define MAX_IRQBUTTON   BUTTON_USER
#define NUM_IRQBUTTONS  1

#define GPIO_BTN_USER   (GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI | \
                         GPIO_PORTC | GPIO_PIN13)

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
 * Name: stm32l4_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC driver.
 *
 ****************************************************************************/

int stm32l4_adc_setup(void);

/****************************************************************************
 * Name: stm32l4_adc_measure_voltages
 *
 * Description:
 *   Read internal reference voltage, internal VBAT and one external voltage.
 *
 ****************************************************************************/

int stm32l4_adc_measure_voltages(uint32_t *vrefint,
                                 uint32_t *vbat,
                                 uint32_t *vext);

/****************************************************************************
 * Name: stm32l4_dac_setup
 *
 * Description:
 *   Initialize DAC and register the DAC driver.
 *
 ****************************************************************************/

int stm32l4_dac_setup(void);

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void);

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_STM32L4_NUCLEO_L452RE_SRC_NUCLEO_L452RE_H */
