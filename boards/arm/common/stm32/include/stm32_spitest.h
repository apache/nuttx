/****************************************************************************
 * boards/arm/common/stm32/include/stm32_spitest.h
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

#ifndef __BOARDS_ARM_COMMON_STM32_INCLUDE_STM32_SPITEST_H
#define __BOARDS_ARM_COMMON_STM32_INCLUDE_STM32_SPITEST_H

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
 * Name: stm32_spidev_bus_test
 *
 * Description:
 *   Called to create the defined SPI buses and test them by initializing
 *   them and sending the CONFIG_STM32_SPI_TEST_MESSAGE (no chip select).
 *
 ****************************************************************************/

int stm32_spidev_bus_test(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __BOARDS_ARM_COMMON_STM32_INCLUDE_STM32_SPITEST_H */
