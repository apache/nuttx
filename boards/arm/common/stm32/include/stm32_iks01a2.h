/****************************************************************************
 * boards/arm/common/stm32/include/stm32_iks01a2.h
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

#ifndef __BOARDS_ARM_COMMON_STM32_INCLUDE_STM32_IKS01A2_H
#define __BOARDS_ARM_COMMON_STM32_INCLUDE_STM32_IKS01A2_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: board_iks01a2_initialize
 *
 * Description:
 *   Register the sensors of the X-NUCLEO-IKS01A2 shield as uORB topics.
 *   The LSM6DSL, HTS221 and LPS22HB use devno, the LSM303AGR uses
 *   devno + 1, so the two accelerometers get different topics.
 *
 * Input Parameters:
 *   devno - The first device number of the topics
 *   busno - The I2C bus of the Arduino connector
 *
 * Returned Value:
 *   Zero (OK) if all sensors were registered; a negated errno value of the
 *   last failure otherwise.
 *
 ****************************************************************************/

int board_iks01a2_initialize(int devno, int busno);

#ifdef __cplusplus
}
#endif

#endif /* __BOARDS_ARM_COMMON_STM32_INCLUDE_STM32_IKS01A2_H */
