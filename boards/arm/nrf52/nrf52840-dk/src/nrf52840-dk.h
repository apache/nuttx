/****************************************************************************
 * boards/arm/nrf52/nrf52840-dk/src/nrf52840-dk.h
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

#ifndef __BOARDS_ARM_NRF52_NRF52840_DK_SRC_NRF52840_DK_H
#define __BOARDS_ARM_NRF52_NRF52840_DK_SRC_NRF52840_DK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include "nrf52_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LED definitions **********************************************************/

/* Definitions to configure LED GPIO as outputs */

#define GPIO_LED1  (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT0 | GPIO_PIN(13))
#define GPIO_LED2  (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT0 | GPIO_PIN(14))
#define GPIO_LED3  (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT0 | GPIO_PIN(15))
#define GPIO_LED4  (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT0 | GPIO_PIN(16))

/* Button definitions *******************************************************/

/* Board supports four buttons. */

#define GPIO_BUTTON1 (GPIO_INPUT | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN(11))
#define GPIO_BUTTON2 (GPIO_INPUT | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN(12))
#define GPIO_BUTTON3 (GPIO_INPUT | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN(24))
#define GPIO_BUTTON4 (GPIO_INPUT | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN(25))

/* Dragino LORA shield (v1.4) - RF98 module (based on SX127X)
 * RESET - P1.11 (D9)
 * CS    - P1.12 (D10)
 * DIO0  - P1.03 (D2)
 */

#define GPIO_SX127X_RESET (GPIO_PORT1 | GPIO_PIN(11))
#define GPIO_SX127X_CS    (GPIO_OUTPUT | GPIO_PORT1 | GPIO_PIN(12))
#define GPIO_SX127X_DIO0  (GPIO_INPUT  | GPIO_PORT1 | GPIO_PIN(3))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_LIB_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int nrf52_bringup(void);

/****************************************************************************
 * Name: nrf52_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the
 *   nrf52840-dk board.
 *
 ****************************************************************************/

#ifdef CONFIG_NRF52_SPI_MASTER
void nrf52_spidev_initialize(void);
#endif

/****************************************************************************
 * Name: nrf52_lsm6dsl_initialize
 *
 * Description:
 *   Initialize I2C-based LSM6DSL.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_LSM6DSL
int nrf52_lsm6dsl_initialize(char *devpath);
#endif

/****************************************************************************
 * Name: nrf52_lsm303agr_initialize
 *
 * Description:
 *   Initialize I2C-based LSM303AGR.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_LSM303AGR
int nrf52_lsm303agr_initialize(char *devpath);
#endif

/****************************************************************************
 * Name: nrf52_hts221_initialize
 *
 * Description:
 *   Initialize I2C-based HTS221.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_HTS221
int nrf52_hts221_initialize(char *devpath);
#endif

/****************************************************************************
 * Name: nrf52_lpwaninitialize
 *
 * Description:
 *   Initialize SX127X LPWAN interaface.
 *
 ****************************************************************************/

#ifdef CONFIG_LPWAN_SX127X
int nrf52_lpwaninitialize(void);
#endif

/****************************************************************************
 * Name: nrf52_timer_driver_setup
 *
 * Description:
 *   Initialize TIMER driver.
 *
 ****************************************************************************/

#ifdef CONFIG_TIMER
int nrf52_timer_driver_setup(FAR const char *devpath, int timer);
#endif

/****************************************************************************
 * Name: nrf52_pwm_setup
 *
 * Description:
 *   Initialize PWM driver.
 *
 ****************************************************************************/

#ifdef CONFIG_PWM
int nrf52_pwm_setup(void);
#endif

/****************************************************************************
 * Name: nrf52_adc_setup
 *
 * Description:
 *   Initialize ADC driver.
 *
 ****************************************************************************/

#ifdef CONFIG_ADC
int nrf52_adc_setup(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_NRF52_NRF52840_DK_SRC_NRF52840_DK_H */
