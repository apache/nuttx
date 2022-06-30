/****************************************************************************
 * arch/arm/src/stm32wb/hardware/stm32wb_ipcc.h
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

#ifndef __ARCH_ARM_SRC_STM32WB_HARDWARE_STM32WB_IPCC_H
#define __ARCH_ARM_SRC_STM32WB_HARDWARE_STM32WB_IPCC_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32WB_IPCC_C1CR_OFFSET      0x0000  /* CPU1 control register */
#define STM32WB_IPCC_C1MR_OFFSET      0x0004  /* CPU1 mask register */
#define STM32WB_IPCC_C1SCR_OFFSET     0x0008  /* CPU1 status set/clear register */
#define STM32WB_IPCC_C1TOC2SR_OFFSET  0x000c  /* CPU1 to CPU2 status register */
#define STM32WB_IPCC_C2CR_OFFSET      0x0010  /* CPU2 control register */
#define STM32WB_IPCC_C2MR_OFFSET      0x0014  /* CPU2 mask register */
#define STM32WB_IPCC_C2SCR_OFFSET     0x0018  /* CPU2 status set/clear register */
#define STM32WB_IPCC_C2TOC1SR_OFFSET  0x001c  /* CPU2 to CPU2 status register */

/* Register Addresses *******************************************************/

#define STM32WB_IPCC_C1CR             (STM32WB_IPCC_BASE + STM32WB_IPCC_C1CR_OFFSET)
#define STM32WB_IPCC_C1MR             (STM32WB_IPCC_BASE + STM32WB_IPCC_C1MR_OFFSET)
#define STM32WB_IPCC_C1SCR            (STM32WB_IPCC_BASE + STM32WB_IPCC_C1SCR_OFFSET)
#define STM32WB_IPCC_C1TOC2SR         (STM32WB_IPCC_BASE + STM32WB_IPCC_C1TOC2SR_OFFSET)
#define STM32WB_IPCC_C2CR             (STM32WB_IPCC_BASE + STM32WB_IPCC_C2CR_OFFSET)
#define STM32WB_IPCC_C2MR             (STM32WB_IPCC_BASE + STM32WB_IPCC_C2MR_OFFSET)
#define STM32WB_IPCC_C2SCR            (STM32WB_IPCC_BASE + STM32WB_IPCC_C2SCR_OFFSET)
#define STM32WB_IPCC_C2TOC1SR         (STM32WB_IPCC_BASE + STM32WB_IPCC_C2TOC1SR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* CPU1 control register (C1CR) */

#define IPCC_C1CR_RXOIE           (1 << 0)  /* Bit 0: Receive channel occupied interrupt enable */
#define IPCC_C1CR_TXFIE           (1 << 16) /* Bit 16: Transmit channel free interrupt enable */

/* CPU1 mask register (C1MR) */

#define IPCC_C1MR_OM_SHIFT        (0)       /* Bits 0-5: Receive channels occupied interrupt mask */
#define IPCC_C1MR_OM_MASK         (0x3f << IPCC_C1MR_OM_SHIFT)
#define IPCC_C1MR_OM_BIT(n)       (1 << (IPCC_C1MR_OM_SHIFT + (n) - 1)) /* Receive channel n = 1..6 */

#define IPCC_C1MR_FM_SHIFT        (16)      /* Bits 16-21: Transmit channels free interrupt mask */
#define IPCC_C1MR_FM_MASK         (0x3f << IPCC_C1MR_FM_SHIFT)
#define IPCC_C1MR_FM_BIT(n)       (1 << (IPCC_C1MR_FM_SHIFT + (n) - 1)) /* Transmit channel n = 1..6 */

/* CPU1 status set/clear register (C1SCR) */

#define IPCC_C1SCR_CLR_SHIFT      (0)       /* Bits 0-5: Receive channels status clear mask */
#define IPCC_C1SCR_CLR_MASK       (0x3f << IPCC_C1SCR_CLR_SHIFT)
#define IPCC_C1SCR_CLR_BIT(n)     (1 << (IPCC_C1SCR_CLR_SHIFT + (n) - 1)) /* Receive channel n = 1..6 */

#define IPCC_C1SCR_SET_SHIFT      (16)      /* Bits 16-21: Transmit channels status set mask */
#define IPCC_C1SCR_SET_MASK       (0x3f << IPCC_C1SCR_SET_SHIFT)
#define IPCC_C1SCR_SET_BIT(n)     (1 << (IPCC_C1SCR_SET_SHIFT + (n) - 1)) /* Transmit channel n = 1..6 */

/* CPU1 to CPU2 status register (C1TOC2SR) */

#define IPCC_C1TOC2SR_SHIFT       (0)       /* Bits 0-5: CPU1 to CPU2 channels status flag */
#define IPCC_C1TOC2SR_MASK        (0x3f << IPCC_C1TOC2SR_SHIFT)
#define IPCC_C1TOC2SR_BIT(n)      (1 << (IPCC_C1TOC2SR_SHIFT + (n) - 1)) /* Channel n = 1..6 */

/* CPU2 control register (C2CR) */

#define IPCC_C2CR_RXOIE           (1 << 0)  /* Bit 0: Receive channel occupied interrupt enable */
#define IPCC_C2CR_TXFIE           (1 << 16) /* Bit 16: Transmit channel free interrupt enable */

/* CPU2 mask register (C2MR) */

#define IPCC_C2MR_OM_SHIFT        (0)       /* Bits 0-5: Receive channels occupied interrupt mask */
#define IPCC_C2MR_OM_MASK         (0x3f << IPCC_C2MR_OM_SHIFT)
#define IPCC_C2MR_OM_BIT(n)       (1 << (IPCC_C2MR_OM_SHIFT + (n) - 1)) /* Receive channel n = 1..6 */

#define IPCC_C2MR_FM_SHIFT        (16)      /* Bits 16-21: Transmit channels free interrupt mask */
#define IPCC_C2MR_FM_MASK         (0x3f << IPCC_C2MR_FM_SHIFT)
#define IPCC_C2MR_FM_BIT(n)       (1 << (IPCC_C2MR_FM_SHIFT + (n) - 1)) /* Transmit channel n = 1..6 */

/* CPU2 status set/clear register (C2SCR) */

#define IPCC_C2SCR_CLR_SHIFT      (0)       /* Bits 0-5: Receive channels status clear mask */
#define IPCC_C2SCR_CLR_MASK       (0x3f << IPCC_C2SCR_CLR_SHIFT)
#define IPCC_C2SCR_CLR_BIT(n)     (1 << (IPCC_C2SCR_CLR_SHIFT + (n) - 1)) /* Receive channel n = 1..6 */

#define IPCC_C2SCR_SET_SHIFT      (16)      /* Bits 16-21: Transmit channels status set mask */
#define IPCC_C2SCR_SET_MASK       (0x3f << IPCC_C2SCR_SET_SHIFT)
#define IPCC_C2SCR_SET_BIT(n)     (1 << (IPCC_C2SCR_SET_SHIFT + (n) - 1)) /* Transmit channel n = 1..6 */

/* CPU2 to CPU1 status register (C2TOC1SR) */

#define IPCC_C2TOC1SR_SHIFT       (0)       /* Bits 0-5: CPU2 to CPU1 channels status flag */
#define IPCC_C2TOC1SR_MASK        (0x3f << IPCC_C2TOC1SR_SHIFT)
#define IPCC_C2TOC1SR_BIT(n)      (1 << (IPCC_C2TOC1SR_SHIFT + (n) - 1)) /* Channel n = 1..6 */

#endif /* __ARCH_ARM_SRC_STM32WB_HARDWARE_STM32WB_IPCC_H */
