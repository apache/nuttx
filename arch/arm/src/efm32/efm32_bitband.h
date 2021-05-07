/****************************************************************************
 * arch/arm/src/efm32/efm32_bitband.h
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

#ifndef __ARCH_ARM_SRC_EFM32_EFM32_BITBAND_H
#define __ARCH_ARM_SRC_EFM32_EFM32_BITBAND_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/efm32_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#if defined(CONFIG_EFM32_BITBAND)
inline void bitband_set_peripheral(uint32_t addr,
                                   uint32_t bit, uint32_t val);
inline uint32_t bitband_get_peripheral(uint32_t addr, uint32_t bit);
inline void bitband_set_sram(uint32_t addr, uint32_t bit, uint32_t val);
inline uint32_t bitband_get_sram(uint32_t addr, uint32_t bit);

#else

#   define bitband_set_peripheral(addr,bit,val)\
    modifyreg32(addr,~(1<<bit),(1<<bit))

#   define bitband_get_peripheral(addr,bit) (((getreg32(addr)) >> bit) & 1)

#   define bitband_set_sram(add,bit,val)\
    modifyreg32(addr,~(1<<bit),(1<<bit))

#   define bitband_get_sram(addr,bit) (((getreg32(addr)) >> bit) & 1)

#endif

#endif /* __ARCH_ARM_SRC_EFM32_EFM32_BITBAND_H */
