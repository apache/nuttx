/****************************************************************************
 * boards/risc-v/hpm6000/hpm6360evk/src/hpm6360evk.h
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

#ifndef __BOARDS_RISCV_HPM6000_HPM6360EVK_SRC_HPM6360EVK_H
#define __BOARDS_RISCV_HPM6000_HPM6360EVK_SRC_HPM6360EVK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hpm_gpio.h"
#include "hpm_iomux.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GPIO Pin Definitions *****************************************************/

/* LEDs */

/* There are four LED status indicators located on the EVK Board.
 * The functions of these LEDs include:
 *
 *   - Main Power Supply(D3)
 *     Green: DC 5V main supply is normal.
 *     Red:   J2 input voltage is over 5.6V.
 *     Off:   The board is not powered.
 *   - Reset RED LED(D15)
 *   - OpenSDA LED(D16)
 *   - USER LED(D18)
 *
 * Only a single LED, D18, is under software control.
 */

#define GPIO_LED        (GPIO_OUTPUT | GPIO_OUTPUT_ZERO | GPIO_PORTA | GPIO_PIN7)  /* PA07 */

#define LED_DRIVER_PATH "/dev/userleds"

int hpm6360_bringup(void);

/****************************************************************************
 * Name: imxrt_autoled_initialize
 *
 * Description:
 *   Initialize NuttX-controlled LED logic
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void hpm6360_autoled_initialize(void);
#endif

#endif /* __BOARDS_RISCV_HPM6000_HPM6360EVK_SRC_HPM6360EVK_H */
