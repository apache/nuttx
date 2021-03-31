/****************************************************************************
 * arch/arm/src/stm32f0l0g0/hardware/stm32_memorymap.h
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

#ifndef __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32_MEMORYMAP_H
#define __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

#if defined(CONFIG_STM32F0L0G0_STM32F03X)
#  include "hardware/stm32f03x_memorymap.h"
#elif defined(CONFIG_STM32F0L0G0_STM32F05X) || \
      defined(CONFIG_STM32F0L0G0_STM32F07X) || \
      defined(CONFIG_STM32F0L0G0_STM32F09X)
#  include "hardware/stm32f05xf07xf09x_memorymap.h"
#elif defined(CONFIG_ARCH_CHIP_STM32L0)
#  include "hardware/stm32l0_memorymap.h"
#elif defined(CONFIG_ARCH_CHIP_STM32G0)
#  include "hardware/stm32g0_memorymap.h"
#else
#  error "Unsupported STM32 M0 memory map"
#endif

#endif /* __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32_MEMORYMAP_H */
