/****************************************************************************
 * boards/arm/s32k1xx/rddrone-bms772/src/rddrone-bms772.h
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

#ifndef __BOARDS_ARM_S32K1XX_RDDRONE_BMS772_SRC_RDDRONE_BMS772_H
#define __BOARDS_ARM_S32K1XX_RDDRONE_BMS772_SRC_RDDRONE_BMS772_H

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

/* RDDRONE-BMS772 GPIOs *****************************************************/

/* LEDs.  The RDDRONE-BMS772 has one RGB LED:
 *
 *   RedLED    PTD16  (FTM0 CH1)
 *   GreenLED  PTB13  (FTM0 CH1)
 *   BlueLED   PTD15  (FTM0 CH0)
 *
 * An output of '0' illuminates the LED.
 */

#define GPIO_LED_R  (PIN_PTD16 | GPIO_LOWDRIVE | GPIO_OUTPUT_ONE)
#define GPIO_LED_G  (PIN_PTB13 | GPIO_LOWDRIVE | GPIO_OUTPUT_ONE)
#define GPIO_LED_B  (PIN_PTD15 | GPIO_LOWDRIVE | GPIO_OUTPUT_ONE)

/* GPIO pins to be registered to the GPIO driver.  These definitions need to
 * be added to the g_gpiopins array in s32k1xx_gpio.c!
 */

#define GPIO0       (PIN_PTC1  | GPIO_OUTPUT) /* GATE_CTRL_CP */
#define GPIO1       (PIN_PTC2  | GPIO_OUTPUT) /* GATE_CTRL_D */
#define GPIO2       (PIN_PTD5  | GPIO_OUTPUT) /* BCC_RESET */
#define GPIO3       (PIN_PTA12 | GPIO_OUTPUT) /* NFC_HPD */
#define GPIO4       (PIN_PTC15 | GPIO_OUTPUT) /* AUTH_WAKE */

#define GPIO5       (PIN_PTE8  | GPIO_INPUT | PIN_INT_BOTH) /* EXT_OUT1 (to ext. header) */
#define GPIO6       (PIN_PTC3  | GPIO_INPUT | PIN_INT_BOTH) /* OVERCURRENT */
#define GPIO7       (PIN_PTC14 | GPIO_INPUT | PIN_INT_BOTH) /* SBC Wake */
#define GPIO8       (PIN_PTC8  | GPIO_INPUT | PIN_INT_BOTH) /* GATE_RS */
#define GPIO9       (PIN_PTA11 | GPIO_INPUT | PIN_INT_BOTH) /* SBC_LIMP */
#define GPIO10      (PIN_PTC9  | GPIO_INPUT | PIN_INT_BOTH) /* BCC_FAULT */
#define GPIO11      (PIN_PTA13 | GPIO_INPUT | PIN_INT_BOTH) /* NFC_ED */

#define NUM_OF_GPIO 12

/* Count of peripheral clock user configurations */

#define NUM_OF_PERIPHERAL_CLOCKS_0 12

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
 * Name: s32k1xx_gpio_initialize
 *
 * Description:
 *   Initialize GPIO drivers for use with /apps/examples/gpio
 *
 ****************************************************************************/

int s32k1xx_gpio_initialize(void);

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

/****************************************************************************
 * Name: s32k1xx_nrstcheck_procfs_register
 *
 * Description:
 *   Check if the (active low) reset pin is being pulled high externally by
 *   reconfiguring the pin (temporarily) to a GPIO input with weak pull-down.
 *   The pin state is saved and registered as a PROCFS entry.  The pin will
 *   then be reconfigured again as reset pin.
 *
 ****************************************************************************/

int s32k1xx_nrstcheck_procfs_register(void);

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_S32K1XX_RDDRONE_BMS772_SRC_RDDRONE_BMS772_H */
