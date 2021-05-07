/****************************************************************************
 * arch/arm/src/stm32/hardware/stm32_memorymap.h
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

#ifndef __ARCH_ARM_SRC_STM32_HARDWARE_STM32_MEMORYMAP_H
#define __ARCH_ARM_SRC_STM32_HARDWARE_STM32_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

#if defined(CONFIG_STM32_STM32L15XX)
#  include "hardware/stm32l15xxx_memorymap.h"
#elif defined(CONFIG_STM32_STM32F10XX)
#  include "hardware/stm32f10xxx_memorymap.h"
#elif defined(CONFIG_STM32_STM32F20XX)
#  include "hardware/stm32f20xxx_memorymap.h"
#elif defined(CONFIG_STM32_STM32F30XX)
#  include "hardware/stm32f30xxx_memorymap.h"
#elif defined(CONFIG_STM32_STM32F33XX)
#  include "hardware/stm32f33xxx_memorymap.h"
#elif defined(CONFIG_STM32_STM32F37XX)
#  include "hardware/stm32f37xxx_memorymap.h"
#elif defined(CONFIG_STM32_STM32F4XXX)
#  include "hardware/stm32f40xxx_memorymap.h"
#elif defined(CONFIG_STM32_STM32G4XXX)
#  include "hardware/stm32g4xxxx_memorymap.h"
#else
#  error "Unsupported STM32 memory map"
#endif

#endif /* __ARCH_ARM_SRC_STM32_HARDWARE_STM32_MEMORYMAP_H */
