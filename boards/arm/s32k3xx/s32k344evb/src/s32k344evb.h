/****************************************************************************
 * boards/arm/s32k3xx/s32k344evb/src/s32k344evb.h
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

/* Copyright 2022 NXP */

#ifndef __BOARDS_ARM_S32K3XX_S32K344EVB_SRC_S32K344EVB_H
#define __BOARDS_ARM_S32K3XX_S32K344EVB_SRC_S32K344EVB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>

#include "hardware/s32k344_pinmux.h"
#include "s32k3xx_periphclocks.h"
#include "s32k3xx_pin.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* S32K344EVB GPIOs *********************************************************/

/* LEDs.  The S32K344EVB has two RGB LEDs:
 *
 *   RedLED0    PTA29  (EMIOS1 CH12 / EMIOS2 CH12)
 *   GreenLED0  PTA30  (EMIOS1 CH13 / EMIOS2 CH13)
 *   BlueLED0   PTA31  (EMIOS1 CH14 / FXIO D0)
 *
 *   RedLED1    PTB18  (EMIOS1 CH15 / EMIOS2 CH14 / FXIO D1)
 *   GreenLED1  PTB25  (EMIOS1 CH21 / EMIOS2 CH21 / FXIO D6)
 *   BlueLED1   PTE12  (EMIOS1 CH5  / FXIO D8)
 *
 * An output of '1' illuminates the LED.
 */

#define GPIO_LED0_R    (PIN_PTA29 | GPIO_LOWDRIVE | GPIO_OUTPUT_ZERO)
#define GPIO_LED0_G    (PIN_PTA30 | GPIO_LOWDRIVE | GPIO_OUTPUT_ZERO)
#define GPIO_LED0_B    (PIN_PTA31 | GPIO_LOWDRIVE | GPIO_OUTPUT_ZERO)

#define GPIO_LED1_R    (PIN_PTB18 | GPIO_LOWDRIVE | GPIO_OUTPUT_ZERO)
#define GPIO_LED1_G    (PIN_PTB25 | GPIO_LOWDRIVE | GPIO_OUTPUT_ZERO)
#define GPIO_LED1_B    (PIN_PTE12 | GPIO_LOWDRIVE | GPIO_OUTPUT_ZERO)

/* Buttons.  The S32K344EVB supports two buttons:
 *
 *   SW0  PTB26  (EIRQ13 / WKPU41)
 *   SW1  PTB19  (WKPU38)
 */

#define GPIO_SW0       (PIN_WKPU41 | PIN_INT_BOTH)
#define GPIO_SW1       (PIN_WKPU38 | PIN_INT_BOTH)

/* Count of peripheral clock user configurations */

#define NUM_OF_PERIPHERAL_CLOCKS_0 25

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
 * Name: s32k3xx_bringup
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

int s32k3xx_bringup(void);

/****************************************************************************
 * Name: s32k3xx_i2cdev_initialize
 *
 * Description:
 *   Initialize I2C driver and register /dev/i2cN devices.
 *
 ****************************************************************************/

int s32k3xx_i2cdev_initialize(void);

/****************************************************************************
 * Name: s32k3xx_spidev_initialize
 *
 * Description:
 *   Configure chip select pins, initialize the SPI driver and register
 *   /dev/spiN devices.
 *
 ****************************************************************************/

int s32k3xx_spidev_initialize(void);

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_S32K3XX_S32K344EVB_SRC_S32K344EVB_H */
