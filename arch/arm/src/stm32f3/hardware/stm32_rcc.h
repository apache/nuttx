/****************************************************************************
 * arch/arm/src/stm32f3/hardware/stm32_rcc.h
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

#ifndef __ARCH_ARM_SRC_STM32F3_HARDWARE_STM32_RCC_H
#define __ARCH_ARM_SRC_STM32F3_HARDWARE_STM32_RCC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_STM32_STM32F30XX) || defined(CONFIG_STM32_STM32F302X8) || \
    defined(CONFIG_STM32_STM32F302XC) || defined(CONFIG_STM32_STM32F303XC) || \
    defined(CONFIG_STM32_STM32F303XE)
#  include "hardware/stm32f30xxx_rcc.h"
#elif defined(CONFIG_STM32_STM32F33XX)
#  include "hardware/stm32f33xxx_rcc.h"
#elif defined(CONFIG_STM32_STM32F37XX)
#  include "hardware/stm32f37xxx_rcc.h"
#else
#  error "Unsupported STM32 F3 part"
#endif

#endif /* __ARCH_ARM_SRC_STM32F3_HARDWARE_STM32_RCC_H */
