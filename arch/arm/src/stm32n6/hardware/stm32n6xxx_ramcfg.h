/****************************************************************************
 * arch/arm/src/stm32n6/hardware/stm32n6xxx_ramcfg.h
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

#ifndef __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_RAMCFG_H
#define __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_RAMCFG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/stm32n6xxx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

/* AXISRAMn control register, n = 1..6 */

#define STM32_RAMCFG_AXISRAM_CR_OFFSET(n) (((n) - 1) * 0x80)

/* Register Addresses *******************************************************/

#define STM32_RAMCFG_AXISRAM_CR(n)  (STM32_RAMCFG_BASE + \
                                     STM32_RAMCFG_AXISRAM_CR_OFFSET(n))
#define STM32_RAMCFG_AXISRAM3_CR    STM32_RAMCFG_AXISRAM_CR(3)
#define STM32_RAMCFG_AXISRAM4_CR    STM32_RAMCFG_AXISRAM_CR(4)
#define STM32_RAMCFG_AXISRAM5_CR    STM32_RAMCFG_AXISRAM_CR(5)
#define STM32_RAMCFG_AXISRAM6_CR    STM32_RAMCFG_AXISRAM_CR(6)

/* Register Bitfield Definitions ********************************************/

/* AXISRAMn control register */

#define RAMCFG_CR_SRAMSD            (1 << 20) /* Bit 20: SRAM shutdown */

#endif /* __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_RAMCFG_H */
