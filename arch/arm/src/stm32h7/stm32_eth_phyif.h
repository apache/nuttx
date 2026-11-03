/****************************************************************************
 * arch/arm/src/stm32h7/stm32_eth_phyif.h
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

#ifndef __ARCH_ARM_SRC_STM32H7_STM32_ETH_PHYIF_H
#define __ARCH_ARM_SRC_STM32H7_STM32_ETH_PHYIF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_STM32_HAVE_SBS
#  include "hardware/stm32h7rsxx_sbs.h"
#else
#  include "hardware/stm32_syscfg.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The PHY interface is selected in SBS on the STM32H7R/S and in SYSCFG on
 * the other STM32H7.
 * The common Ethernet MAC driver writes one of the STM32_ETH_PHYIF_<mode>
 * values to the STM32_ETH_PHYIF_MASK field of STM32_ETH_PHYIF_REG before it
 * resets the MAC.
 */

#ifdef CONFIG_STM32_HAVE_SBS
#  define STM32_ETH_PHYIF_REG    STM32_SBS_PMCR
#  define STM32_ETH_PHYIF_MASK   SBS_PMCR_ETH_PHYSEL_MASK
#  define STM32_ETH_PHYIF_MII    SBS_PMCR_ETH_PHYSEL_GMII_MII
#  define STM32_ETH_PHYIF_RMII   SBS_PMCR_ETH_PHYSEL_RMII
#else
#  define STM32_ETH_PHYIF_REG    STM32_SYSCFG_PMC
#  define STM32_ETH_PHYIF_MASK   SYSCFG_PMC_EPIS_MASK
#  define STM32_ETH_PHYIF_MII    SYSCFG_PMC_EPIS_MII
#  define STM32_ETH_PHYIF_RMII   SYSCFG_PMC_EPIS_RMII
#endif

#endif /* __ARCH_ARM_SRC_STM32H7_STM32_ETH_PHYIF_H */
