/****************************************************************************
 * boards/arm/common/stm32/include/stm32_led1642gw.h
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

#ifndef __BOARDS_ARM_COMMON_STM32_INCLUDE_STM32_LED1642GW_H
#define __BOARDS_ARM_COMMON_STM32_INCLUDE_STM32_LED1642GW_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: board_led1642gw_initialize
 *
 * Description:
 *   Initialize one LED1642GW LED driver and register the RGB LEDs connected
 *   to its outputs.  RGB LED n is connected to outputs 3n (R), 3n + 1 (G)
 *   and 3n + 2 (B).  Can be called for each LED1642GW device connected to
 *   the board.
 *
 * Input Parameters:
 *   devno - The number of the first RGB LED device (/dev/rgbledN)
 *   nleds - The number of RGB LEDs connected to the device
 *   clk   - The GPIO connected to the CLK line
 *   sdi   - The GPIO connected to the SDI line
 *   le    - The GPIO connected to the LE line
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_led1642gw_initialize(int devno, int nleds, uint32_t clk,
                               uint32_t sdi, uint32_t le);

#ifdef __cplusplus
}
#endif

#endif /* __BOARDS_ARM_COMMON_STM32_INCLUDE_STM32_LED1642GW_H */
