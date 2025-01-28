/****************************************************************************
 * arch/arm/src/stm32h5/hardware/stm32_icache.h
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

#ifndef __ARCH_ARM_SRC_STM32H5_HARDWARE_STM32_ICACHE_H
#define __ARCH_ARM_SRC_STM32H5_HARDWARE_STM32_ICACHE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_ICACHE_CR_OFFSET        0x00             /* ICACHE Control Register Offset */
#define STM32_ICACHE_SR_OFFSET        0x04             /* ICACHE Status Register Offset */
#define STM32_ICACHE_IER_OFFSET       0x08             /* ICACHE Interrupt Enable Register Offset */
#define STM32_ICACHE_FCR_OFFSET       0x0c             /* ICACHE Flag Clear Register Offset */
#define STM32_ICACHE_HMONR_OFFSET     0x10             /* ICACHE Hit Monitor Register Offset */
#define STM32_ICACHE_MMONR_OFFSET     0x14             /* ICACHE Miss Monitor Register Offset */
#define STM32_ICACHE_CRR_OFFSET(x)   (0x20 + (x << 3)) /* ICACHE Region Configuration Register Offset */

/* Register Addresses *******************************************************/

#define STM32_ICACHE_CR              (STM32_ICACHE_BASE + STM32_ICACHE_CR_OFFSET)     /* ICACHE Control Register */
#define STM32_ICACHE_SR              (STM32_ICACHE_BASE + STM32_ICACHE_SR_OFFSET)     /* ICACHE Status Register */
#define STM32_ICACHE_IER             (STM32_ICACHE_BASE + STM32_ICACHE_IER_OFFSET)    /* ICACHE Interrupt Enable Register */
#define STM32_ICACHE_FCR             (STM32_ICACHE_BASE + STM32_ICACHE_FCR_OFFSET)    /* ICACHE Flag Clear Register */
#define STM32_ICACHE_HMONR           (STM32_ICACHE_BASE + STM32_ICACHE_HMONR_OFFSET)  /* ICACHE Hit Monitor Register */
#define STM32_ICACHE_MMONR           (STM32_ICACHE_BASE + STM32_ICACHE_MMONR_OFFSET)  /* ICACHE Miss Monitor Register */
#define STM32_ICACHE_CRR(x)          (STM32_ICACHE_BASE + STM32_ICACHE_CRR_OFFSET(x)) /* ICACHE Region Configuration Register */

/* Register Bitfield Definitions ********************************************/

/* Control Register */

#define ICACHE_CR_EN                 (1 << 0)  /* Enable */
#define ICACHE_CR_CACHEINV           (1 << 1)  /* Cache Invalidate */
#define ICACHE_CR_WAYSEL             (1 << 2)  /* Associativity Mode Selection */
#define ICACHE_CR_HITMEN             (1 << 16) /* Hit Monitor Enable */
#define ICACHE_CR_MISSMEN            (1 << 17) /* Miss Monitor Enable */
#define ICACHE_CR_HITMRST            (1 << 18) /* Hit Monitor Reset */
#define ICACHE_CR_MISSMRST           (1 << 19) /* Miss Monitor Reset */

/* Status Register */

#define ICACHE_SR_BUSYF              (1 << 0) /* Full Invalidate Busy Flag */
#define ICACHE_SR_BSYENDF            (1 << 1) /* Full Invalidate FInished Flag */
#define ICACHE_SR_ERRF               (1 << 2) /* Cache Error Flag */

/* Interrupt Enable Register */

#define ICACHE_IER_BSYENDIE          (1 << 1) /* Full Invalidate Finished Interrupt Enable */
#define ICACHE_IER_ERRIE             (1 << 2) /* Cache Error Interrupt Enable */

#define ICACHE_IER_ALLINTS           (ICACHE_IER_BSYENDIE | ICACHE_IER_ERRIE) /* All Cache Interrupts Mask */

/* Flag Clear Register */

#define ICACHE_FCR_CBSYENDF          (1 << 1) /* Clear Full Invalidate Finished Flag */
#define ICACHE_FCR_CERRF             (1 << 2) /* Clear Cache Error Flag */

/* Hit Monitor Register */

/* Miss Monitor Register */

#define ICACHE_MMONR_MISSMON_MASK    (0xffff) /* 16-bit Miss Monitor Mask */

/* Region x Configuration Register */

#define ICACHE_CRR_BASEADDR_SHIFT    (0) /* Base Address for Region x */
#define ICACHE_CRR_BASEADDR_MASK     (0xff << ICACHE_CRR_BASEADDR_SHIFT)
#define ICACHE_CRR_RSIZE_SHIFT       (9) /* Size for Region x (n=1..7, 2^(11+n)) */
#define ICACHE_CRR_RSIZE_MASK        (0x7 << ICACHE_CRR_RSIZE_SHIFT)
#define ICACHE_CRR_REN               (1 << 15)
#define ICACHE_CRR_REMAPADDR_SHIFT   (16) /* Remapped Address for Region x */
#define ICACHE_CRR_REMAPADDR_MASK    (0x7ff << ICACHE_CRR_REMAPADDR_SHIFT)
#define ICACHE_CRR_MSTSEL_SHIFT      (28) /* AHB Cache Master Selection for Region x */
#define ICACHE_CRR_MSTSEL            (1 << ICACHE_CRR_MSTSEL_SHIFT)
#define ICACHE_CRR_HBURST_SHIFT      (31) /* Output Burst Type for Region x (0=Wrap, 1=INCR) */
#define ICACHE_CRR_HBURST            (1 << ICACHE_CRR_HBURST_SHIFT)

#endif /* __ARCH_ARM_SRC_STM32H5_HARDWARE_STM32_ICACHE_H */
