/****************************************************************************
 * boards/arm/imx6/sabre-6quad/src/sabre-6quad.h
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

#ifndef __BOARDS_ARM_IMX6_SABRE_6QUAD_SRC_SABRE_6QUAD_H
#define __BOARDS_ARM_IMX6_SABRE_6QUAD_SRC_SABRE_6QUAD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "imx_gpio.h"
#include "imx_iomuxc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* SABRE-6QUAD GPIO Pin Definitions *****************************************/

/* LED
 *
 * A single LED is available driven GPIO1_IO02.
 * On the schematic this is USR_DEF_RED_LED signal to pin T1 (GPIO_2).
 * This signal is shared with KEY_ROW6 (ALT2).
 * A high value illuminates the LED.
 */

#define IOMUX_LED  (IOMUX_PULL_NONE | IOMUX_CMOS_OUTPUT | \
                    IOMUX_DRIVE_40OHM | \
                    IOMUX_SPEED_MEDIUM | IOMUX_SLEW_SLOW)
#define GPIO_LED   (GPIO_OUTPUT | GPIO_OUTPUT_ZERO | \
                    GPIO_PORT1 | GPIO_PIN2 | \
                    IOMUX_LED)

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
 * Name: imx_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

#if defined(CONFIG_BOARDCTL) || defined(CONFIG_BOARD_LATE_INITIALIZE)
int imx_bringup(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_SABRE_6QUAD_SRC_SABRE_6QUAD_H */
