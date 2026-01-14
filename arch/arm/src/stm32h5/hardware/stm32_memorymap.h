/****************************************************************************
 * arch/arm/src/stm32h5/hardware/stm32_memorymap.h
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

#ifndef __ARCH_ARM_SRC_STM32H5_HARDWARE_STM32_MEMORYMAP_H
#define __ARCH_ARM_SRC_STM32H5_HARDWARE_STM32_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

#if defined(CONFIG_STM32H5_STM32H52XXX) || defined(CONFIG_STM32H5_STM32H53XXX) || \
    defined(CONFIG_STM32H5_STM32H56XXX) || defined(CONFIG_STM32H5_STM32H57XXX)
#  include "hardware/stm32h5xxx_memorymap.h"
#else
#  error "Unsupported STM32 H5 memory map"
#endif

#endif /* __ARCH_ARM_SRC_STM32H5_HARDWARE_STM32_MEMORYMAP_H */