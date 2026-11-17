/****************************************************************************
 * arch/arm/src/stm32n6/hardware/stm32n6xxx_rifsc.h
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

#ifndef __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_RIFSC_H
#define __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_RIFSC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/stm32_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

/* SECCFGR/PRIVCFGR: n = 0..5, RIMC_ATTR: n = 0..12 */

#define STM32_RIFSC_RISC_CR_OFFSET          0x0000
#define STM32_RIFSC_RISC_SECCFGR_OFFSET(n)  (0x0010 + ((n) << 2))
#define STM32_RIFSC_RISC_PRIVCFGR_OFFSET(n) (0x0030 + ((n) << 2))
#define STM32_RIFSC_RIMC_CR_OFFSET          0x0c00
#define STM32_RIFSC_RIMC_ATTR_OFFSET(n)     (0x0c10 + ((n) << 2))

/* Register Addresses *******************************************************/

#define STM32_RIFSC_RISC_CR          (STM32_RIFSC_BASE + \
                                      STM32_RIFSC_RISC_CR_OFFSET)
#define STM32_RIFSC_RISC_SECCFGR(n)  (STM32_RIFSC_BASE + \
                                      STM32_RIFSC_RISC_SECCFGR_OFFSET(n))
#define STM32_RIFSC_RISC_PRIVCFGR(n) (STM32_RIFSC_BASE + \
                                      STM32_RIFSC_RISC_PRIVCFGR_OFFSET(n))
#define STM32_RIFSC_RIMC_CR          (STM32_RIFSC_BASE + \
                                      STM32_RIFSC_RIMC_CR_OFFSET)
#define STM32_RIFSC_RIMC_ATTR(n)     (STM32_RIFSC_BASE + \
                                      STM32_RIFSC_RIMC_ATTR_OFFSET(n))

/* Register Bitfield Definitions ********************************************/

/* RIMC master attribute register */

#define RIFSC_RIMC_ATTR_MCID_SHIFT  (4)       /* Bits 4-6: Master compartment ID */
#define RIFSC_RIMC_ATTR_MCID_MASK   (0x7 << RIFSC_RIMC_ATTR_MCID_SHIFT)
#  define RIFSC_RIMC_ATTR_MCID(n)   ((uint32_t)(n) << RIFSC_RIMC_ATTR_MCID_SHIFT)
#define RIFSC_RIMC_ATTR_MSEC        (1 << 8)  /* Bit 8: Master secure */
#define RIFSC_RIMC_ATTR_MPRIV       (1 << 9)  /* Bit 9: Master privileged */

/* RIMC bus master indexes (RIMC_ATTRx) */

#define RIFSC_MASTER_ETR            0
#define RIFSC_MASTER_NPU            1
#define RIFSC_MASTER_SDMMC1         2
#define RIFSC_MASTER_SDMMC2         3
#define RIFSC_MASTER_OTG1           4
#define RIFSC_MASTER_OTG2           5
#define RIFSC_MASTER_ETH1           6
#define RIFSC_MASTER_GPU2D          7
#define RIFSC_MASTER_DMA2D          8
#define RIFSC_MASTER_DCMIPP         9
#define RIFSC_MASTER_LTDC1          10
#define RIFSC_MASTER_LTDC2          11
#define RIFSC_MASTER_VENC           12

/* RISC peripheral indexes: register number * 32 + bit */

#define RIFSC_PERIPH_REG(p)         ((p) >> 5)
#define RIFSC_PERIPH_BIT(p)         (1 << ((p) & 31))

#define RIFSC_PERIPH_ETH1           (1 * 32 + 28)

#endif /* __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_RIFSC_H */
