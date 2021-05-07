/****************************************************************************
 * arch/arm/src/stm32/hardware/stm32_dma.h
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

#ifndef __ARCH_ARM_SRC_STM32_HARDWARE_STM32_DMA_H
#define __ARCH_ARM_SRC_STM32_HARDWARE_STM32_DMA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/* Include the correct DMA register definitions for
 * selected STM32 DMA IP core:
 *   - STM32 DMA IP version 1 - F0, F1, F3, G4, L0, L1, L4
 *   - STM32 DMA IP version 2 - F2, F4, F7, H7
 */

#if defined(CONFIG_STM32_HAVE_IP_DMA_V1) && defined(CONFIG_STM32_HAVE_IP_DMA_V2)
#  error Only one STM32 DMA IP version must be selected
#endif

#if defined(CONFIG_STM32_HAVE_IP_DMA_V1)
#  include "stm32_dma_v1.h"
#elif defined(CONFIG_STM32_HAVE_IP_DMA_V2)
#  include "stm32_dma_v2.h"
#else
#  error "STM32 DMA IP version not specified"
#endif

#endif /* __ARCH_ARM_SRC_STM32_HARDWARE_STM32_DMA_H */
