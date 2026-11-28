/****************************************************************************
 * boards/tricore/tc3x/a2g-tc375-lite/src/tc375.h
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

#ifndef __BOARDS_TRICORE_TC3X_A2G_TC375_LITE_SRC_TC375_H
#define __BOARDS_TRICORE_TC3X_A2G_TC375_LITE_SRC_TC375_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "aurix/tricore_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* User LEDs LED1 on P00.5 and LED2 on P00.6, on when low */

#define GPIO_LED1        (GPIO_PORT(0) | GPIO_PIN(5) | GPIO_OUTPUT | \
                          GPIO_OUTPUT_SET)
#define GPIO_LED2        (GPIO_PORT(0) | GPIO_PIN(6) | GPIO_OUTPUT | \
                          GPIO_OUTPUT_SET)

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
 * Name: tc375_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

#if defined(CONFIG_BOARDCTL) || defined(CONFIG_BOARD_LATE_INITIALIZE)
int tc375_bringup(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_TRICORE_TC3X_A2G_TC375_LITE_SRC_TC375_H */
