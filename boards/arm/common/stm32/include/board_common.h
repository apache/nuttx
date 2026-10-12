/****************************************************************************
 * boards/arm/common/stm32/include/board_common.h
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

#ifndef __BOARDS_ARM_COMMON_STM32_INCLUDE_BOARD_COMMON_H
#define __BOARDS_ARM_COMMON_STM32_INCLUDE_BOARD_COMMON_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_common_initialize
 *
 * Description:
 *   Initialize the board features that are common to all STM32 boards.
 *
 ****************************************************************************/

int stm32_common_initialize(void);

#endif /* __BOARDS_ARM_COMMON_STM32_INCLUDE_BOARD_COMMON_H */
