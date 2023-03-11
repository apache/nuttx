/****************************************************************************
 * boards/arm/nrf53/nrf5340-dk/src/nrf5340-dk.h
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

#ifndef __BOARDS_ARM_NRF53_NRF5340_DK_SRC_NRF53_NRF5340_DK_H
#define __BOARDS_ARM_NRF53_NRF5340_DK_SRC_NRF53_NRF5340_DK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include "nrf53_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* procfs File System */

#ifdef CONFIG_FS_PROCFS
#  ifdef CONFIG_NSH_PROC_MOUNTPOINT
#    define NRF53_PROCFS_MOUNTPOINT CONFIG_NSH_PROC_MOUNTPOINT
#  else
#    define NRF53_PROCFS_MOUNTPOINT "/proc"
#  endif
#endif

/* LED definitions **********************************************************/

/* Definitions to configure LED GPIO as outputs */

#define GPIO_LED1  (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT0 | GPIO_PIN(28))
#define GPIO_LED2  (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT0 | GPIO_PIN(29))
#define GPIO_LED3  (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT0 | GPIO_PIN(30))
#define GPIO_LED4  (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT0 | GPIO_PIN(31))

/* Button definitions *******************************************************/

/* Board supports four buttons. */

#define GPIO_BUTTON1 (GPIO_INPUT | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN(23))
#define GPIO_BUTTON2 (GPIO_INPUT | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN(24))
#define GPIO_BUTTON3 (GPIO_INPUT | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN(8))
#define GPIO_BUTTON4 (GPIO_INPUT | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN(9))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: nrf53_bringup
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

int nrf53_bringup(void);

/****************************************************************************
 * Name: nrf53_timer_driver_setup
 *
 * Description:
 *   Initialize TIMER driver.
 *
 ****************************************************************************/

#ifdef CONFIG_TIMER
int nrf53_timer_driver_setup(const char *devpath, int timer);
#endif

/****************************************************************************
 * Name: nrf53_pwm_setup
 *
 * Description:
 *   Initialize PWM driver.
 *
 ****************************************************************************/

#ifdef CONFIG_PWM
int nrf53_pwm_setup(void);
#endif

/****************************************************************************
 * Name: nrf53_adc_setup
 *
 * Description:
 *   Initialize ADC driver.
 *
 ****************************************************************************/

#ifdef CONFIG_ADC
int nrf53_adc_setup(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_NRF53_NRF5340_DK_SRC_NRF53_NRF5340_DK_H */
