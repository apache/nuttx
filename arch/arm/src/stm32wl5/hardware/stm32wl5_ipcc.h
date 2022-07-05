/****************************************************************************
 * arch/arm/src/stm32wl5/hardware/stm32wl5_ipcc.h
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

#ifndef __ARCH_ARM_SRC_STM32WL5_HARDWARE_STM32WL5_IPCC_H
#define __ARCH_ARM_SRC_STM32WL5_HARDWARE_STM32WL5_IPCC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define STM32WL5_IPCC_CPU1_OFFSET    0x00
#define STM32WL5_IPCC_CPU2_OFFSET    0x10

/* Register Offsets *********************************************************/

#define STM32WL5_IPCC_CR_OFFSET      0x00 /* IPCC control register */
#define STM32WL5_IPCC_MR_OFFSET      0x04 /* IPCC mask register */
#define STM32WL5_IPCC_SCR_OFFSET     0x08 /* IPCC status set clear register */
#define STM32WL5_IPCC_CTOCSR_OFFSET  0x0c /* IPCC processor to processor status register */

/* Register Addresses *******************************************************/

#define STM32WL5_IPCC_C1CR      (STM32WL5_IPCC_BASE+STM32WL5_IPCC_CR_OFFSET+STM32WL5_IPCC_CPU1_OFFSET)
#define STM32WL5_IPCC_C1MR      (STM32WL5_IPCC_BASE+STM32WL5_IPCC_MR_OFFSET+STM32WL5_IPCC_CPU1_OFFSET)
#define STM32WL5_IPCC_C1SCR     (STM32WL5_IPCC_BASE+STM32WL5_IPCC_SCR_OFFSET+STM32WL5_IPCC_CPU1_OFFSET)
#define STM32WL5_IPCC_C1TOC2SR  (STM32WL5_IPCC_BASE+STM32WL5_IPCC_CTOCSR_OFFSET+STM32WL5_IPCC_CPU1_OFFSET)
#define STM32WL5_IPCC_C2CR      (STM32WL5_IPCC_BASE+STM32WL5_IPCC_CR_OFFSET+STM32WL5_IPCC_CPU2_OFFSET)
#define STM32WL5_IPCC_C2MR      (STM32WL5_IPCC_BASE+STM32WL5_IPCC_MR_OFFSET+STM32WL5_IPCC_CPU2_OFFSET)
#define STM32WL5_IPCC_C2SCR     (STM32WL5_IPCC_BASE+STM32WL5_IPCC_SCR_OFFSET+STM32WL5_IPCC_CPU2_OFFSET)
#define STM32WL5_IPCC_C2TOC1SR  (STM32WL5_IPCC_BASE+STM32WL5_IPCC_CTOCSR_OFFSET+STM32WL5_IPCC_CPU2_OFFSET)

/* Register Bitfield Definitions ********************************************/

#define STM32WL5_IPCC_TX_SHIFT    (16)              /* TX shift for all registers */

/* IPCC control register */

#define STM32WL5_IPCC_CR_RXOIE    (1 <<  0)         /* Bit  0: Receive channel occupied interrupt enable */
#define STM32WL5_IPCC_CR_TXFIE    (1 << 16)         /* Bit 16: Transmit channel free interrupt enable */

/* IPCC mask register */

#define STM32WL5_IPCC_MR_CHNOM(n) (1 << (n))        /* Bit   0..5: Receive channel n occupied interrupt enable, Channels 0..5 */
#define STM32WL5_IPCC_MR_CHNFM(n) (1 << (16 + (n))) /* Bit 16..21: Transmit channel n free interrupt enable, Channels 0..5 */

/* IPCC status set clear register */

#define STM32WL5_IPCC_SCR_CHNC(n) (1 << (n))        /* Bit   0..5: Receive channel n status bit clear, Channels 0..5 */
#define STM32WL5_IPCC_SCR_CHNS(n) (1 << (16 + (n))) /* Bit 16..21: Transmit channel n status bit set, Channels 0..5 */

/* IPCC processor to processor status register */

#define STM32WL5_IPCC_CTOCSR_CHNF(n) (1 << (n))     /* Bit   0..5: Channel n occupied, Channels 0..5 */

#endif /* __ARCH_ARM_SRC_STM32WL5_HARDWARE_STM32WL5_IPCC_H */
