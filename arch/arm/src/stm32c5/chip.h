/****************************************************************************
 * arch/arm/src/stm32c5/chip.h
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

#ifndef __ARCH_ARM_SRC_STM32C5_CHIP_H
#define __ARCH_ARM_SRC_STM32C5_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/* Include the memory map and the chip definitions file.  Other chip hardware
 * files should then include this file for the proper setup.
 */

#include <arch/irq.h>
#include <arch/stm32c5/chip.h>
#include "hardware/stm32_pinmap.h"
#include "hardware/stm32_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* If the common ARMv8-M vector handling logic is used, then it expects the
 * following definition in this file that provides the number of supported
 * external interrupts which, for this architecture, is provided in the
 * arch/stm32c5/chip.h header file.
 */

#define ARMV8M_PERIPHERAL_INTERRUPTS STM32_IRQ_NEXTINTS

/* DMA instance parameters.  DMA1 and DMA2 are the LPDMA1 and LPDMA2
 * instances.
 */

#define STM32_DMA1_NCHAN      8
#define STM32_DMA1_IRQ_FIRST  STM32_IRQ_LPDMA1_CH0
#define STM32_DMA2_NCHAN      4
#define STM32_DMA2_IRQ_FIRST  STM32_IRQ_LPDMA2_CH0

#endif /* __ARCH_ARM_SRC_STM32C5_CHIP_H */
