/****************************************************************************
 * boards/arm/nrf54l/nrf54l15-tag/src/nrf54l15-tag.h
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

#ifndef __BOARDS_ARM_NRF54L_NRF54L15_TAG_SRC_NRF54L15_TAG_H
#define __BOARDS_ARM_NRF54L_NRF54L15_TAG_SRC_NRF54L15_TAG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include "nrf54l_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* procfs File System */

#ifdef CONFIG_FS_PROCFS
#  ifdef CONFIG_NSH_PROC_MOUNTPOINT
#    define NRF54L_PROCFS_MOUNTPOINT CONFIG_NSH_PROC_MOUNTPOINT
#  else
#    define NRF54L_PROCFS_MOUNTPOINT "/proc"
#  endif
#endif

/* LED definitions **********************************************************/

/* Definitions to configure LED GPIO as outputs */

#define GPIO_LED1 (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT2 | GPIO_PIN(8))
#define GPIO_LED2 (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT2 | GPIO_PIN(10))
#define GPIO_LED3 (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT2 | GPIO_PIN(9))

/* Antenna switch ***********************************************************/

#define GPIO_ANT1 (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT1 | GPIO_PIN(9))
#define GPIO_ANT2 (GPIO_OUTPUT | GPIO_PORT1 | GPIO_PIN(10))

/* Button and sensors *******************************************************/

#define GPIO_BUTTON1   (GPIO_INPUT | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN(0))
#define GPIO_BMI270_CS (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT1 | GPIO_PIN(7))

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: nrf54l_bringup
 *
 * Description:
 *   Perform board-specific initialization. Called from
 *   board_late_initialize() when CONFIG_BOARD_LATE_INITIALIZE is selected.
 *
 ****************************************************************************/

int nrf54l_bringup(void);

#ifdef CONFIG_NRF54L_SPI2_MASTER
void nrf54l_spidev_initialize(void);
#endif

#ifdef CONFIG_SYSTEM_I2CTOOL
int nrf54l_i2ctool(void);
#endif

#ifdef CONFIG_SENSORS
int nrf54l_sensors_init(void);
#endif

#ifdef CONFIG_NRF54L_SAADC
int nrf54l_adc_setup(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_NRF54L_NRF54L15_TAG_SRC_NRF54L15_TAG_H */
