/****************************************************************************
 * boards/arm/nrf53/thingy53/src/thingy53.h
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

#ifndef __BOARDS_ARM_NRF53_THINGY53_SRC_THINGY53_H
#define __BOARDS_ARM_NRF53_THINGY53_SRC_THINGY53_H

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

#define GPIO_LED_R  (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT1 | GPIO_PIN(8))
#define GPIO_LED_G  (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT1 | GPIO_PIN(6))
#define GPIO_LED_B  (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT1 | GPIO_PIN(7))

/* Button definitions *******************************************************/

#define GPIO_BUTTON1 (GPIO_INPUT | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN(14))
#define GPIO_BUTTON2 (GPIO_INPUT | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN(13))

/* nRF21540 front end module
 *   RX_EN - P1.11
 *   MODE  - P1.12
 *   SEL   - P1.10
 *   TX_EN - P0.30
 *   CSN   - P0.24
 */

#define GPIO_NRF21540_RXEN (GPIO_OUTPUT | GPIO_PORT1 | GPIO_PIN(11))
#define GPIO_NRF21540_MODE (GPIO_OUTPUT | GPIO_PORT1 | GPIO_PIN(12))
#define GPIO_NRF21540_SEL  (GPIO_OUTPUT | GPIO_PORT1 | GPIO_PIN(10))
#define GPIO_NRF21540_TXEN (GPIO_OUTPUT | GPIO_PORT0 | GPIO_PIN(30))
#define GPIO_NRF21540_CSN  (GPIO_OUTPUT | GPIO_PORT0 | GPIO_PIN(24))

/* Sensor power control
 *   SENS_PWR_CTRL - P0.31
 */

#define GPIO_SENS_PWRCTRL (GPIO_OUTPUT | GPIO_PORT0 | GPIO_PIN(31))

/* ADXL362
 *   INT1 - P0.19
 *   CS   - P0.22
 */

#define GPIO_ADXL362_INT1 (GPIO_INPUT  | GPIO_PORT0 | GPIO_PIN(19))
#define GPIO_ADXL362_CS   (GPIO_OUTPUT | GPIO_PORT0 | GPIO_PIN(22))

/* BMI270
 *   INT1 - P0.23
 *   CS   - P1.04
 */

#define GPIO_BMI270_INT1 (GPIO_INPUT  | GPIO_PORT0 | GPIO_PIN(23))
#define GPIO_BMI270_CS   (GPIO_OUTPUT | GPIO_PORT1 | GPIO_PIN(4))

/* BMM150
 *   INT  - P0.20
 *   DRDY - P0.21
 */

#define GPIO_BMM150_INT  (GPIO_INPUT  | GPIO_PORT0 | GPIO_PIN(20))
#define GPIO_BMM150_DRDY (GPIO_INPUT  | GPIO_PORT0 | GPIO_PIN(21))

/* BH1749
 *   INT  - P1.05
 */

#define GPIO_BH1749_INT (GPIO_INPUT  | GPIO_PORT1 | GPIO_PIN(5))

/* nPM1100 PMIC status and control
 *   ERR  - P1.01
 *   CHG  - P1.00
 *   ISET - P0.07
 */

#define GPIO_NPM1100_ERR  (GPIO_INPUT  | GPIO_PORT1 | GPIO_PIN(1))
#define GPIO_NPM1100_CHG  (GPIO_INPUT  | GPIO_PORT1 | GPIO_PIN(0))
#define GPIO_NPM1100_ISET (GPIO_OUTPUT | GPIO_PORT0 | GPIO_PIN(7))

/* Battery monitoring
 *   EN   - P0.16
 */

#define GPIO_BATT_EN      (GPIO_OUTPUT | GPIO_PORT0 | GPIO_PIN(16))

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

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_NRF53_THINGY53_SRC_THINGY53_H */
