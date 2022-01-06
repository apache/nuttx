/****************************************************************************
 * boards/arm/tiva/launchxl-cc1310/src/launchxl-cc1310.h
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

#ifndef __BOARDS_ARM_TIVA_LAUNCH_CC1310_SRC_LAUNCH_CC1310_H
#define __BOARDS_ARM_TIVA_LAUNCH_CC1310_SRC_LAUNCH_CC1310_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Button GPIO IRQ numbers
 *
 *   DIO13_BTN1  SW1  Low input sensed when depressed
 *   DIO14_BTN2  SW2  Low input sensed when depressed
 */

#define CC1310_SW1_IRQ  TIVA_IRQ_DIO_13
#define CC1310_SW2_IRQ  TIVA_IRQ_DIO_14

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct cc13xx_pinconfig_s; /* Forward reference */

/* The LaunchXL-cc1310 has two LEDs controlled by software: DIO7_GLED (CR1)
 * and DIO6_RLED (CR2).  A high output value illuminates an LED.
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in
 * any way.  The following definitions are used to access individual LEDs.
 */

extern const struct cc13xx_pinconfig_s g_gpio_gled;
extern const struct cc13xx_pinconfig_s g_gpio_rled;

/* The LaunchXL-CC1310 has two push-puttons:
 *
 *   DIO13_BTN1  SW1  Low input sensed when depressed
 *   DIO14_BTN2  SW2  Low input sensed when depressed
 */

extern const struct cc13xx_pinconfig_s g_gpio_sw1;
extern const struct cc13xx_pinconfig_s g_gpio_sw2;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: cc1310_bringup
 *
 * Description:
 *   Bring up board features.
 *
 *   If CONFIG_BOARD_LATE_INITIALIZE=y, then this function will be called
 *   from board_late_initialize().
 *
 *   If CONFIG_BOARD_LATE_INITIALIZE is not selected,
 *   but CONFIG_BOARDCTL=y
 *   then this function will *probably* be called from application logic via
 *   boardctl().
 *
 *   Otherwise, this function will not be called (which is usually a bad
 *   thing)
 *
 ****************************************************************************/

int cc1310_bringup(void);

/****************************************************************************
 * Name: cc1310_ssidev_initialize
 *
 * Description:
 *   Called to configure SSI chip select GPIO pins for the LAUNCHXL-CC1310
 *   board.
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_SSI
void cc1310_ssidev_initialize(void);
#endif

#endif /* __BOARDS_ARM_TIVA_LAUNCH_CC1310_SRC_LAUNCH_CC1310_H */
