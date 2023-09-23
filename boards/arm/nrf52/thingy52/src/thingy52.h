/****************************************************************************
 * boards/arm/nrf52/thingy52/src/thingy52.h
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

#ifndef __BOARDS_ARM_NRF52_THINGY52_SRC_THINGY52_H
#define __BOARDS_ARM_NRF52_THINGY52_SRC_THINGY52_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdbool.h>

#include "nrf52_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* procfs File System */

#ifdef CONFIG_FS_PROCFS
#  ifdef CONFIG_NSH_PROC_MOUNTPOINT
#    define NRF52_PROCFS_MOUNTPOINT CONFIG_NSH_PROC_MOUNTPOINT
#  else
#    define NRF52_PROCFS_MOUNTPOINT "/proc"
#  endif
#endif

/* Button definitions *******************************************************/

#define GPIO_BUTTON1 (GPIO_INPUT | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN(11))

/* Supported devices ********************************************************/

/* LIS2DH12 (I2C address: 0x19)
 *   INT - P0.12
 */

#define LIS2DH12_I2C_ADDR 0x19
#define GPIO_LIS2DH12_INT (GPIO_INPUT | GPIO_PORT0 | GPIO_PIN(12))

/* MPU9250 (I2C address: 0x68)
 *   INT - P0.05
 */

#define MPU9250_I2C_ADDR  0x68
#define GPIO_MPU9250_INT (GPIO_INPUT | GPIO_PORT0 | GPIO_PIN(5))

/* LPS22HB (I2C address: 0x5c)
 *   INT - P0.23
 */

#define LPS22HB_I2C_ADDR  0x5c
#define GPIO_LPS22HB_INT (GPIO_INPUT | GPIO_PORT0 | GPIO_PIN(23))

/* HTS221 (I2C address: 0x5f)
 *   INT - P0.24
 */

#define HTS221_I2C_ADDR   0x5f
#define GPIO_HTS221_INT (GPIO_INPUT | GPIO_PORT0 | GPIO_PIN(24))

/* BH1745NUC (I2C address: 0x38)
 *   INT - P0.31
 */

#define BH1745NUC_I2C_ADDR 0x38
#define GPIO_BH1745NUC_INT (GPIO_INPUT | GPIO_PORT0 | GPIO_PIN(31))

/* CCS811 (I2C address: 0x5a)
 *   INT - P0.22
 */

#define CCS811_I2C_ADDR   0x5a
#define GPIO_CCS811_INT (GPIO_INPUT | GPIO_PORT0 | GPIO_PIN(22))

/* SX1509 (I2C address: 0x3e)
 *   RESET - P0.16
 *   INT   - not connected
 */

#define GPIO_SX1509_RESET (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT0 | GPIO_PIN(16))

/* VDD_PWR_CTRL - P0.30
 *   Enables the board VDD rail that supplies all on-board sensors.
 *   Must be driven high, otherwise the rail floats at a too-low voltage.
 */

#define GPIO_VDD_PWR_CTRL (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT0 | GPIO_PIN(30))

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
 * Name: nrf52_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 ****************************************************************************/

int nrf52_bringup(void);

/****************************************************************************
 * Name: nrf52_sensors_init
 *
 * Description:
 *   Initialzie on-board sensors
 *
 ****************************************************************************/

#ifdef CONFIG_NRF52_I2C0_MASTER
int nrf52_sensors_init(void);
#endif

#ifdef CONFIG_ADC
/****************************************************************************
 * Name: nrf52_adc_setup
 *
 * Description:
 *   Initialize the ADC used to measure the battery voltage.
 *
 ****************************************************************************/

int nrf52_adc_setup(void);
#endif

#ifdef CONFIG_IOEXPANDER_SX1509
/****************************************************************************
 * Name: nrf52_sx1509_initialize
 *
 * Description:
 *   Configure the SX1509 io expander.
 *
 ****************************************************************************/

int nrf52_sx1509_initialize(void);

/****************************************************************************
 * Name: nrf52_ccs811_wake
 *
 * Description:
 *   Drive the CCS811 nWAKE line (SX1509 pin 12, active low).
 *
 ****************************************************************************/

void nrf52_ccs811_wake(bool on);

/****************************************************************************
 * Name: nrf52_sx1509_get
 *
 * Description:
 *   Return the io-expander device, or NULL if not yet initialized.
 *
 ****************************************************************************/

FAR struct ioexpander_dev_s *nrf52_sx1509_get(void);
#endif

#if defined(CONFIG_RGBLED) && defined(CONFIG_IOEXPANDER_SX1509) && \
    defined(CONFIG_IOEXPANDER_PWM)
/****************************************************************************
 * Name: nrf52_rgbled_initialize
 *
 * Description:
 *   Configure the RGB LED.
 *
 ****************************************************************************/

int nrf52_rgbled_initialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_NRF52_THINGY52_SRC_THINGY52_H */
