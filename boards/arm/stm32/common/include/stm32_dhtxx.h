/****************************************************************************
 * boards/arm/stm32/common/include/stm32_dhtxx.h
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

#ifndef __BOARDS_ARM_STM32_COMMON_INCLUDE_STM32_DHTXX_H
#define __BOARDS_ARM_STM32_COMMON_INCLUDE_STM32_DHTXX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: board_dhtxx_initialize
 *
 * Description:
 *   This function is called by application-specific, setup logic to
 *   configure the DHTxx sensor.
 *
 * Input Parameters:
 *   devno - The device number, used to build the device path as /dev/humN.
 *
 * Returned Value:
 *   Zero (OK) returned on success;
 *   a negated errno value is returned on failure.
 *
 ****************************************************************************/

int board_dhtxx_initialize(int devno);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __BOARDS_ARM_STM32_COMMON_INCLUDE_STM32_DHTXX_H */
