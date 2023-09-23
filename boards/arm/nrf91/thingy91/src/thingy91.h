/****************************************************************************
 * boards/arm/nrf91/thingy91/src/thingy91.h
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

#ifndef __BOARDS_ARM_NRF91_THINGY91_SRC_THINGY91_H
#define __BOARDS_ARM_NRF91_THINGY91_SRC_THINGY91_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include "nrf91_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* procfs File System */

#ifdef CONFIG_FS_PROCFS
#  ifdef CONFIG_NSH_PROC_MOUNTPOINT
#    define NRF91_PROCFS_MOUNTPOINT CONFIG_NSH_PROC_MOUNTPOINT
#  else
#    define NRF91_PROCFS_MOUNTPOINT "/proc"
#  endif
#endif

/* LED definitions **********************************************************/

#define GPIO_LEDS_R  (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT0 | GPIO_PIN(0))
#define GPIO_LEDS_G  (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT0 | GPIO_PIN(1))
#define GPIO_LEDS_B  (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT0 | GPIO_PIN(2))

#define GPIO_LEDW_R  (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT0 | GPIO_PIN(29))
#define GPIO_LEDW_G  (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT0 | GPIO_PIN(30))
#define GPIO_LEDW_B  (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT0 | GPIO_PIN(31))

/* Button definitions *******************************************************/

/* Board supports four buttons. */

#define GPIO_BUTTON1 (GPIO_INPUT | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN(26))

/* Supported devices ********************************************************/

/* ADXL372
 *   INT1 - P0.09
 *   CS   - P0.08
 */

#define GPIO_ADXL372_INT1 (GPIO_INPUT  | GPIO_PORT0 | GPIO_PIN(6))
#define GPIO_ADXL372_CS   (GPIO_OUTPUT | GPIO_PORT0 | GPIO_PIN(7))

/* ADXL362
 *   INT1 - P0.09
 *   CS   - P0.08
 */

#define GPIO_ADXL362_INT1 (GPIO_INPUT  | GPIO_PORT0 | GPIO_PIN(9))
#define GPIO_ADXL362_CS   (GPIO_OUTPUT | GPIO_PORT0 | GPIO_PIN(8))

/* BH1749
 *   INT  - P0.27
 */

#define GPIO_BH1749_INT (GPIO_INPUT  | GPIO_PORT0 | GPIO_PIN(27))

/* ADP5360
 *   INT  - P0.17
 */

#define GPIO_ADP5360_INT (GPIO_INPUT  | GPIO_PORT0 | GPIO_PIN(17))

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
 * Name: nrf91_bringup
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

int nrf91_bringup(void);

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_NRF91_THINGY91_SRC_THINGY91_H */
