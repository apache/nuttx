/****************************************************************************
 * boards/arm/rm57/rm57l843-launchxl2/src/rm57l843-launchxl2.h
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

#ifndef __BOARDS_ARM_RM57_RM57L843_LAUNCHXL2_SRC_RM57L843_LAUNCHXL2_H
#define __BOARDS_ARM_RM57_RM57L843_LAUNCHXL2_SRC_RM57L843_LAUNCHXL2_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "rm57_gio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LEDs
 *
 * The LAUNCHXL2-RM57L has two user LEDs, labeled B6 and B7 on the board
 * silkscreen after the GIO pins that drive them: GIOB[6] (ball J2) and
 * GIOB[7] (ball F1) on the RM57L843 337ZWT package. Both are the reset
 * default (primary) function on their respective balls per the RM57L843
 * datasheet pin table, so no PINMUX configuration is required to use them
 * as plain GIO.
 *
 * LED illumination polarity is assumed active-high (GIO_OUTPUT_SET = on);
 * this has not been confirmed against the board schematic. If the LEDs
 * appear inverted on real hardware, swap GIO_OUTPUT_SET/GIO_OUTPUT_CLEAR
 * here and invert the true/false sense in rm57_autoleds.c /
 * rm57_userleds.c.
 */

#define GIO_LED_B6    (GIO_OUTPUT | GIO_CFG_DEFAULT | GIO_OUTPUT_SET | \
                       GIO_PORT_GIOB | GIO_PIN6)
#define GIO_LED_B7    (GIO_OUTPUT | GIO_CFG_DEFAULT | GIO_OUTPUT_SET | \
                       GIO_PORT_GIOB | GIO_PIN7)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: rm57_bringup
 *
 * Description:
 *   Bring up board features.
 *
 ****************************************************************************/

int rm57_bringup(void);

#endif /* __BOARDS_ARM_RM57_RM57L843_LAUNCHXL2_SRC_RM57L843_LAUNCHXL2_H */
