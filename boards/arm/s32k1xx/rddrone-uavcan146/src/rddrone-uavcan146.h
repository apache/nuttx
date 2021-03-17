/****************************************************************************
 * boards/arm/s32k1xx/rddrone-uavcan146/src/rddrone-uavcan146.h
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

#ifndef __BOARDS_ARM_S32K1XX_RDDRONE_UAVCAN146_SRC_RDDRONE_UAVCAN146_H
#define __BOARDS_ARM_S32K1XX_RDDRONE_UAVCAN146_SRC_RDDRONE_UAVCAN146_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>

#include "hardware/s32k1xx_pinmux.h"
#include "s32k1xx_periphclocks.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* RDDRONE-UAVCAN146 GPIOs **************************************************/

/* LEDs.  The RDDRONE-UAVCAN146 has one RGB LED:
 *
 *   RedLED   PTD15 (FTM0 CH0)
 *   GreenLED PTD16 (FTM0 CH1)
 *   BlueLED  PTD0  (FTM0 CH2)
 *
 * An output of '1' illuminates the LED.
 */

#define GPIO_LED_R     (PIN_PTD15 | GPIO_LOWDRIVE | GPIO_OUTPUT_ZERO)
#define GPIO_LED_G     (PIN_PTD16 | GPIO_LOWDRIVE | GPIO_OUTPUT_ZERO)
#define GPIO_LED_B     (PIN_PTD0  | GPIO_LOWDRIVE | GPIO_OUTPUT_ZERO)

/* Buttons.  The RDDRONE-UAVCAN146 supports one button:
 *
 *   SW3  PTC14
 */

#define GPIO_SW3       (PIN_PTC14 | PIN_INT_BOTH)

/* SPI chip selects */

/* SE050 Enable */

#define GPIO_SE050_EN  (PIN_PTA6 | GPIO_LOWDRIVE)

/* Count of peripheral clock user configurations */

#define NUM_OF_PERIPHERAL_CLOCKS_0 12

/****************************************************************************
 * Public Types
 ****************************************************************************/

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
 *   CONFIG_BOARD_LATE_INITIALIZE=y && CONFIG_LIB_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int s32k1xx_bringup(void);

/****************************************************************************
 * Name: s32k1xx_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the RDDRONE-UAVCAN146
 *   board.
 *
 ****************************************************************************/

#ifdef CONFIG_S32K1XX_LPSPI
void s32k1xx_spidev_initialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_S32K1XX_RDDRONE_UAVCAN146_SRC_RDDRONE_UAVCAN146_H */
