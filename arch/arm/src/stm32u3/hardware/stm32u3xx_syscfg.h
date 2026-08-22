/****************************************************************************
 * arch/arm/src/stm32u3/hardware/stm32u3xx_syscfg.h
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

#ifndef __ARCH_ARM_SRC_STM32U3_HARDWARE_STM32U3XX_SYSCFG_H
#define __ARCH_ARM_SRC_STM32U3_HARDWARE_STM32U3XX_SYSCFG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#if defined(CONFIG_STM32_STM32U3C5XX)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_SYSCFG_SECCFGR_OFFSET 0x0000
#define STM32_SYSCFG_CFGR1_OFFSET   0x0004
#define STM32_SYSCFG_FPUIMR_OFFSET  0x0008
#define STM32_SYSCFG_CNSLCKR_OFFSET 0x000c
#define STM32_SYSCFG_CSLCKR_OFFSET  0x0010
#define STM32_SYSCFG_CFGR2_OFFSET   0x0014
#define STM32_SYSCFG_CCCSR_OFFSET   0x001c
#define STM32_SYSCFG_CCVR_OFFSET    0x0020
#define STM32_SYSCFG_CCCR_OFFSET    0x0024
#define STM32_SYSCFG_RSSCMDR_OFFSET 0x002c

/* Register Addresses *******************************************************/

#define STM32_SYSCFG_SECCFGR (STM32_SYSCFG_BASE + STM32_SYSCFG_SECCFGR_OFFSET)
#define STM32_SYSCFG_CFGR1   (STM32_SYSCFG_BASE + STM32_SYSCFG_CFGR1_OFFSET)
#define STM32_SYSCFG_FPUIMR  (STM32_SYSCFG_BASE + STM32_SYSCFG_FPUIMR_OFFSET)
#define STM32_SYSCFG_CNSLCKR (STM32_SYSCFG_BASE + STM32_SYSCFG_CNSLCKR_OFFSET)
#define STM32_SYSCFG_CSLCKR  (STM32_SYSCFG_BASE + STM32_SYSCFG_CSLCKR_OFFSET)
#define STM32_SYSCFG_CFGR2   (STM32_SYSCFG_BASE + STM32_SYSCFG_CFGR2_OFFSET)
#define STM32_SYSCFG_CCCSR   (STM32_SYSCFG_BASE + STM32_SYSCFG_CCCSR_OFFSET)
#define STM32_SYSCFG_CCVR    (STM32_SYSCFG_BASE + STM32_SYSCFG_CCVR_OFFSET)
#define STM32_SYSCFG_CCCR    (STM32_SYSCFG_BASE + STM32_SYSCFG_CCCR_OFFSET)
#define STM32_SYSCFG_RSSCMDR (STM32_SYSCFG_BASE + STM32_SYSCFG_RSSCMDR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* SYSCFG secure configuration register */

#define SYSCFG_SECCFGR_RESET         0x00000000
#define SYSCFG_SECCFGR_SYSCFGSEC     (1 << 0)
#define SYSCFG_SECCFGR_CLASSBSEC     (1 << 1)
#define SYSCFG_SECCFGR_FPUSEC        (1 << 3)

/* SYSCFG configuration register 1 */

#define SYSCFG_CFGR1_RESET           0x00000000
#define SYSCFG_CFGR1_IR_POL          (1 << 5)
#define SYSCFG_CFGR1_IR_MOD_SHIFT    (6)
#define SYSCFG_CFGR1_IR_MOD_MASK     (0x3 << SYSCFG_CFGR1_IR_MOD_SHIFT)
#define SYSCFG_CFGR1_IR_MOD(n)       ((n) << SYSCFG_CFGR1_IR_MOD_SHIFT)
#define SYSCFG_CFGR1_BOOSTEN         (1 << 8)
#define SYSCFG_CFGR1_ANASWVDD        (1 << 9)
#define SYSCFG_CFGR1_PB6_FMP         (1 << 16)
#define SYSCFG_CFGR1_PB7_FMP         (1 << 17)
#define SYSCFG_CFGR1_PB8_FMP         (1 << 18)
#define SYSCFG_CFGR1_PB9_FMP         (1 << 19)
#define SYSCFG_CFGR1_TSC_G2_IO1_COMP (1 << 24)
#define SYSCFG_CFGR1_TSC_G2_IO3_COMP (1 << 25)

/* SYSCFG FPU interrupt mask register */

#define SYSCFG_FPUIMR_RESET          0x0000001f
#define SYSCFG_FPUIMR_FPU_IOIE       (1 << 0)
#define SYSCFG_FPUIMR_FPU_DZIE       (1 << 1)
#define SYSCFG_FPUIMR_FPU_UFIE       (1 << 2)
#define SYSCFG_FPUIMR_FPU_OFIE       (1 << 3)
#define SYSCFG_FPUIMR_FPU_IDIE       (1 << 4)
#define SYSCFG_FPUIMR_FPU_IXIE       (1 << 5)

/* SYSCFG CPU nonsecure lock register */

#define SYSCFG_CNSLCKR_RESET         0x00000000
#define SYSCFG_CNSLCKR_LOCKNSVTOR    (1 << 0)
#define SYSCFG_CNSLCKR_LOCKNSMPU     (1 << 1)

/* SYSCFG CPU secure lock register */

#define SYSCFG_CSLCKR_RESET          0x00000000
#define SYSCFG_CSLCKR_LOCKSVTAIRCR   (1 << 0)
#define SYSCFG_CSLCKR_LOCKSMPU       (1 << 1)
#define SYSCFG_CSLCKR_LOCKSAU        (1 << 2)

/* SYSCFG configuration register 2 */

#define SYSCFG_CFGR2_RESET           0x00000000
#define SYSCFG_CFGR2_CLL             (1 << 0)
#define SYSCFG_CFGR2_SPL             (1 << 1)
#define SYSCFG_CFGR2_PVDL            (1 << 2)
#define SYSCFG_CFGR2_ECCL            (1 << 3)

/* SYSCFG compensation cell control/status register */

#define SYSCFG_CCCSR_RESET           0x0000000a
#define SYSCFG_CCCSR_EN1             (1 << 0)
#define SYSCFG_CCCSR_CS1             (1 << 1)
#define SYSCFG_CCCSR_EN2             (1 << 2)
#define SYSCFG_CCCSR_CS2             (1 << 3)
#define SYSCFG_CCCSR_RDY1            (1 << 8)
#define SYSCFG_CCCSR_RDY2            (1 << 9)

/* SYSCFG compensation cell value register */

#define SYSCFG_CCVR_RESET            0x00000000
#define SYSCFG_CCVR_NCV1_SHIFT       (0)
#define SYSCFG_CCVR_NCV1_MASK        (0xf << SYSCFG_CCVR_NCV1_SHIFT)
#define SYSCFG_CCVR_NCV1(n)          ((n) << SYSCFG_CCVR_NCV1_SHIFT)
#define SYSCFG_CCVR_PCV1_SHIFT       (4)
#define SYSCFG_CCVR_PCV1_MASK        (0xf << SYSCFG_CCVR_PCV1_SHIFT)
#define SYSCFG_CCVR_PCV1(n)          ((n) << SYSCFG_CCVR_PCV1_SHIFT)
#define SYSCFG_CCVR_NCV2_SHIFT       (8)
#define SYSCFG_CCVR_NCV2_MASK        (0xf << SYSCFG_CCVR_NCV2_SHIFT)
#define SYSCFG_CCVR_NCV2(n)          ((n) << SYSCFG_CCVR_NCV2_SHIFT)
#define SYSCFG_CCVR_PCV2_SHIFT       (12)
#define SYSCFG_CCVR_PCV2_MASK        (0xf << SYSCFG_CCVR_PCV2_SHIFT)
#define SYSCFG_CCVR_PCV2(n)          ((n) << SYSCFG_CCVR_PCV2_SHIFT)

/* SYSCFG compensation cell code register */

#define SYSCFG_CCCR_RESET            0x00007878
#define SYSCFG_CCCR_NCC1_SHIFT       (0)
#define SYSCFG_CCCR_NCC1_MASK        (0xf << SYSCFG_CCCR_NCC1_SHIFT)
#define SYSCFG_CCCR_NCC1(n)          ((n) << SYSCFG_CCCR_NCC1_SHIFT)
#define SYSCFG_CCCR_PCC1_SHIFT       (4)
#define SYSCFG_CCCR_PCC1_MASK        (0xf << SYSCFG_CCCR_PCC1_SHIFT)
#define SYSCFG_CCCR_PCC1(n)          ((n) << SYSCFG_CCCR_PCC1_SHIFT)
#define SYSCFG_CCCR_NCC2_SHIFT       (8)
#define SYSCFG_CCCR_NCC2_MASK        (0xf << SYSCFG_CCCR_NCC2_SHIFT)
#define SYSCFG_CCCR_NCC2(n)          ((n) << SYSCFG_CCCR_NCC2_SHIFT)
#define SYSCFG_CCCR_PCC2_SHIFT       (12)
#define SYSCFG_CCCR_PCC2_MASK        (0xf << SYSCFG_CCCR_PCC2_SHIFT)
#define SYSCFG_CCCR_PCC2(n)          ((n) << SYSCFG_CCCR_PCC2_SHIFT)

/* SYSCFG RSS command register */

#define SYSCFG_RSSCMDR_RESET         0x00000000
#define SYSCFG_RSSCMDR_RSSCMD_SHIFT  (0)
#define SYSCFG_RSSCMDR_RSSCMD_MASK   (0xffff << SYSCFG_RSSCMDR_RSSCMD_SHIFT)
#define SYSCFG_RSSCMDR_RSSCMD(n)     ((n) << SYSCFG_RSSCMDR_RSSCMD_SHIFT)

#else
#  error "Unsupported STM32U3 SYSCFG"
#endif

#endif /* __ARCH_ARM_SRC_STM32U3_HARDWARE_STM32U3XX_SYSCFG_H */
