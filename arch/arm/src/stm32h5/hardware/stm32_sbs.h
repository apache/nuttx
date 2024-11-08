/****************************************************************************
 * arch/arm/src/stm32h5/hardware/stm32_sbs.h
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

#ifndef __ARCH_ARM_SRC_STM32H5_HARDWARE_STM32_SBS_H
#define __ARCH_ARM_SRC_STM32H5_HARDWARE_STM32_SBS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

#if !defined(CONFIG_STM32H5_STM32H52XXX) && \
    !defined(CONFIG_STM32H5_STM32H53XXX) && \
    !defined(CONFIG_STM32H5_STM32H56XXX) && \
    !defined(CONFIG_STM32H5_STM32H57XXX)
#  warning "SBS not verified on STM32H50x variants."
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_SBS_HDPLCR_OFFSET       0x0010
#define STM32_SBS_HPDLSR_OFFSET       0x0014
#define STM32_SBS_NEXTHDPLCR_OFFSET   0x0018
#define STM32_SBS_DBGCR_OFFSET        0x0020
#define STM32_SBS_DBGLOCKR_OFFSET     0x0024
#define STM32_SBS_RSSCMDR_OFFSET      0x0034
#define STM32_SBS_EPOCHSELCR_OFFSET   0x00A0
#define STM32_SBS_SECCFGR_OFFSET      0x00C0
#define STM32_SBS_PMCR_OFFSET         0x0100
#define STM32_SBS_FPUIMR_OFFSET       0x0104
#define STM32_SBS_MESR_OFFSET         0x0108
#define STM32_SBS_CCCSR_OFFSET        0x0110
#define STM32_SBS_CCVALR_OFFSET       0x0114
#define STM32_SBS_CCSWCR_OFFSET       0x0118
#define STM32_SBS_CFGR2_OFFSET        0x0120
#define STM32_SBS_CNSLCKR_OFFSET      0x0144
#define STM32_SBS_CSLCKR_OFFSET       0x0148
#define STM32_SBS_ECCNMIR_OFFSET      0x014C

/* Register Addresses *******************************************************/

#define STM32_SBS_HDPLCR      (STM32_SBS_BASE + STM32_SBS_HDPLCR_OFFSET)
#define STM32_SBS_HPDLSR      (STM32_SBS_BASE + STM32_SBS_HPDLSR_OFFSET)
#define STM32_SBS_NEXTHDPLCR  (STM32_SBS_BASE + STM32_SBS_NEXTHDPLCR_OFFSET)
#define STM32_SBS_DBGCR       (STM32_SBS_BASE + STM32_SBS_DBGCR_OFFSET)
#define STM32_SBS_DBGLOCKR    (STM32_SBS_BASE + STM32_SBS_DBGLOCKR_OFFSET)
#define STM32_SBS_RSSCMDR     (STM32_SBS_BASE + STM32_SBS_RSSCMDR_OFFSET)
#define STM32_SBS_EPOCHSELCR  (STM32_SBS_BASE + STM32_SBS_EPOCHSELCR_OFFSET)
#define STM32_SBS_SECCFGR     (STM32_SBS_BASE + STM32_SBS_SECCFGR_OFFSET)
#define STM32_SBS_PMCR        (STM32_SBS_BASE + STM32_SBS_PMCR_OFFSET)
#define STM32_SBS_FPUIMR      (STM32_SBS_BASE + STM32_SBS_FPUIMR_OFFSET)
#define STM32_SBS_MESR        (STM32_SBS_BASE + STM32_SBS_MESR_OFFSET)
#define STM32_SBS_CCCSR       (STM32_SBS_BASE + STM32_SBS_CCCSR_OFFSET)
#define STM32_SBS_CCVALR      (STM32_SBS_BASE + STM32_SBS_CCVALR_OFFSET)
#define STM32_SBS_CCSWCR      (STM32_SBS_BASE + STM32_SBS_CCSWCR_OFFSET)
#define STM32_SBS_CFGR2       (STM32_SBS_BASE + STM32_SBS_CFGR2_OFFSET)
#define STM32_SBS_CNSLCKR     (STM32_SBS_BASE + STM32_SBS_CNSLCKR_OFFSET)
#define STM32_SBS_CSLCKR      (STM32_SBS_BASE + STM32_SBS_CSLCKR_OFFSET)
#define STM32_SBS_ECCNMIR     (STM32_SBS_BASE + STM32_SBS_ECCNMIR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Product mode and configuration register (PMCR) */

#define SBS_PMCR_PB6_FMP  (1 << 16) /* Fast-mode Plus on PB6*/
#define SBS_PMCR_PB7_FMP  (1 << 17) /* Fast-mode Plus on PB7*/
#define SBS_PMCR_PB8_FMP  (1 << 18) /* Fast-mode Plus on PB8*/
#define SBS_PMCR_PB9_FMP  (1 << 19) /* Fast-mode Plus on PB9*/

#define SBS_PMCR_ETH_SEL_PHY_SHIFT           (21) /* Bits 23-21 Ethernet PHY interface selection */
#define SBS_PMCR_ETH_SEL_PHY_MASK            (0b111 << SBS_PMCR_ETH_SEL_PHY_SHIFT)
#  define SBS_PMCR_ETH_SEL_PHY_GMII_OR_MII   (0 << SBS_PMCR_ETH_SEL_PHY_SHIFT)
#  define SBS_PMCR_ETH_SEL_PHY_RMII          (4 << SBS_PMCR_ETH_SEL_PHY_SHIFT)

#endif /* __ARCH_ARM_SRC_STM32H5_HARDWARE_STM32_SBS_H */
