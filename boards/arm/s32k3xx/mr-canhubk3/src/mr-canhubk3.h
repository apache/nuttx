/****************************************************************************
 * boards/arm/s32k3xx/mr-canhubk3/src/mr-canhubk3.h
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

#ifndef __BOARDS_ARM_S32K3XX_MR_CANHUBK3_SRC_MR_CANHUBK3_H
#define __BOARDS_ARM_S32K3XX_MR_CANHUBK3_SRC_MR_CANHUBK3_H

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

/* MR-CANHUBK3 GPIOs ********************************************************/

/* LEDs.  The MR-CANHUBK3 has one RGB LED:
 *
 *   RedLED    PTE14  (FXIO D7 / EMIOS0 CH19)
 *   GreenLED  PTA27  (FXIO D5 / EMIOS1 CH10 / EMIOS2 CH10)
 *   BlueLED   PTE12  (FXIO D8 / EMIOS1 CH5)
 *
 * An output of '0' illuminates the LED.
 */

#define GPIO_LED_R     (PIN_PTE14 | GPIO_LOWDRIVE | GPIO_OUTPUT_ONE)
#define GPIO_LED_G     (PIN_PTA27 | GPIO_LOWDRIVE | GPIO_OUTPUT_ONE)
#define GPIO_LED_B     (PIN_PTE12 | GPIO_LOWDRIVE | GPIO_OUTPUT_ONE)

/* Buttons.  The MR-CANHUBK3 supports two buttons:
 *
 *   SW1  PTD15  (EIRQ31)
 *   SW2  PTA25  (EIRQ5 / WKPU34)
 */

#define GPIO_SW1       (PIN_EIRQ31_2 | PIN_INT_BOTH)  /* PTD15 */
#define GPIO_SW2       (PIN_EIRQ5_2  | PIN_INT_BOTH)  /* PTA25 */

/* Count of peripheral clock user configurations */

#define NUM_OF_PERIPHERAL_CLOCKS_0 26

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

/****************************************************************************
 * Name: s32k3xx_tja1153_initialize
 *
 * Description:
 *   Initialize a TJA1153 CAN PHY connected to a FlexCAN peripheral (0-5)
 *
 ****************************************************************************/

int s32k3xx_tja1153_initialize(int bus);

/****************************************************************************
 * Name: s32k3xx_selftest
 *
 * Description:
 *   Runs basic routines to verify that all board components are up and
 *   running.  Results are send to the syslog, it is recommended to
 *   enable all output levels (error, warning and info).
 *
 ****************************************************************************/

void s32k3xx_selftest(void);

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_S32K3XX_MR_CANHUBK3_SRC_MR_CANHUBK3_H */
