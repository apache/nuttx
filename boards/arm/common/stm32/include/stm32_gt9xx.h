/****************************************************************************
 * boards/arm/common/stm32/include/stm32_gt9xx.h
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

#ifndef __BOARDS_ARM_COMMON_STM32_INCLUDE_STM32_GT9XX_H
#define __BOARDS_ARM_COMMON_STM32_INCLUDE_STM32_GT9XX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: board_gt9xx_initialize
 *
 * Description:
 *   Reset a Goodix GT9xx touch controller, select its I2C address 0x5d and
 *   register it as /dev/input<devno>.  The controller reports touches
 *   through a rising edge on the intpin GPIO.
 *
 * Input Parameters:
 *   devno  - Minor number of the /dev/inputN device
 *   busno  - I2C bus the controller is on
 *   intpin - GPIO configuration of the interrupt line
 *   rstpin - GPIO configuration of the active low reset line, initially
 *            asserted
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_gt9xx_initialize(int devno, int busno, uint32_t intpin,
                           uint32_t rstpin);

#ifdef __cplusplus
}
#endif

#endif /* __BOARDS_ARM_COMMON_STM32_INCLUDE_STM32_GT9XX_H */
