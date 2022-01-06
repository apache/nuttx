/****************************************************************************
 * boards/arm/s32k1xx/s32k148evb/src/s32k148evb.h
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

#ifndef __BOARDS_ARM_S32K1XX_S32K148EVB_SRC_S32K148EVB_H
#define __BOARDS_ARM_S32K1XX_S32K148EVB_SRC_S32K148EVB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>

#include "hardware/s32k1xx_pinmux.h"
#include "s32k1xx_periphclocks.h"
#include "s32k1xx_pin.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* S32K148EVB GPIOs *********************************************************/

/* LEDs.  The S32K148EVB has one RGB LED:
 *
 *   RedLED    PTE21  (FTM4 CH1)
 *   GreenLED  PTE22  (FTM4 CH2)
 *   BlueLED   PTE23  (FTM4 CH3)
 *
 * An output of '1' illuminates the LED.
 */

#define GPIO_LED_R  (PIN_PTE21 | GPIO_LOWDRIVE | GPIO_OUTPUT_ZERO)
#define GPIO_LED_G  (PIN_PTE22 | GPIO_LOWDRIVE | GPIO_OUTPUT_ZERO)
#define GPIO_LED_B  (PIN_PTE23 | GPIO_LOWDRIVE | GPIO_OUTPUT_ZERO)

/* Buttons.  The S32K148EVB supports two buttons:
 *
 *   SW3  PTC12
 *   SW4  PTC13
 */

#define GPIO_SW3    (PIN_PTC12 | PIN_INT_BOTH)
#define GPIO_SW4    (PIN_PTC13 | PIN_INT_BOTH)

/* Count of peripheral clock user configurations */

#define NUM_OF_PERIPHERAL_CLOCKS_0 18

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* User peripheral configuration structure 0 */

extern const struct peripheral_clock_config_s g_peripheral_clockconfig0[];

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: s32k1xx_bringup
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

int s32k1xx_bringup(void);

/****************************************************************************
 * Name: s32k1xx_i2cdev_initialize
 *
 * Description:
 *   Initialize I2C driver and register /dev/i2cN devices.
 *
 ****************************************************************************/

int s32k1xx_i2cdev_initialize(void);

/****************************************************************************
 * Name: s32k1xx_spidev_initialize
 *
 * Description:
 *   Configure chip select pins, initialize the SPI driver and register
 *   /dev/spiN devices.
 *
 ****************************************************************************/

int s32k1xx_spidev_initialize(void);

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_S32K1XX_S32K148EVB_SRC_S32K148EVB_H */
