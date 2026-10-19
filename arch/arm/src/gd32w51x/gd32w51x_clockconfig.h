/****************************************************************************
 * arch/arm/src/gd32w51x/gd32w51x_clockconfig.h
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

#ifndef __ARCH_ARM_SRC_GD32W51X_GD32W51X_CLOCKCONFIG_H
#define __ARCH_ARM_SRC_GD32W51X_GD32W51X_CLOCKCONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clock tree after gd32w51x_clockconfig():
 *
 *   HXTAL (BOARD_HXTAL_FREQUENCY) -> /PSC = 1 MHz -> x360 = 360 MHz VCO
 *   -> /2 (PLLP) -> SYSCLK 180 MHz
 *   AHB = 180 MHz, APB2 = 90 MHz, APB1 = 45 MHz
 *
 * The prescaler is set so the VCO input is 1 MHz: the crystal must be a
 * whole number of MHz (the GD-xD-W515-EVAL has 25 MHz, the vendor
 * library assumes 40 MHz).
 */

#define GD32W51X_HXTAL_FREQ     BOARD_HXTAL_FREQUENCY
#define GD32W51X_SYSCLK_FREQ    180000000
#define GD32W51X_AHB_FREQ       GD32W51X_SYSCLK_FREQ
#define GD32W51X_PCLK2_FREQ     (GD32W51X_SYSCLK_FREQ / 2)
#define GD32W51X_PCLK1_FREQ     (GD32W51X_SYSCLK_FREQ / 4)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Name: gd32w51x_clockconfig
 *
 * Description:
 *   Switch the system clock from the IRC16M reset state to the PLL fed by
 *   the HXTAL, at 180 MHz.
 *
 ****************************************************************************/

void gd32w51x_clockconfig(void);

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_GD32W51X_GD32W51X_CLOCKCONFIG_H */
