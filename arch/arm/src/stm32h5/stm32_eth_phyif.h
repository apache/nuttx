/****************************************************************************
 * arch/arm/src/stm32h5/stm32_eth_phyif.h
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

#ifndef __ARCH_ARM_SRC_STM32H5_STM32_ETH_PHYIF_H
#define __ARCH_ARM_SRC_STM32H5_STM32_ETH_PHYIF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/stm32_sbs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The PHY interface is selected in SBS.  The SBS is secure: with
 * TrustZone enabled the selection fails from the non-secure world.
 * The common Ethernet MAC driver writes one of the STM32_ETH_PHYIF_<mode>
 * values to the STM32_ETH_PHYIF_MASK field of STM32_ETH_PHYIF_REG before it
 * resets the MAC.
 */

#define STM32_ETH_PHYIF_REG      STM32_SBS_PMCR
#define STM32_ETH_PHYIF_MASK     SBS_PMCR_ETH_SEL_PHY_MASK
#define STM32_ETH_PHYIF_MII      SBS_PMCR_ETH_SEL_PHY_GMII_OR_MII
#define STM32_ETH_PHYIF_RMII     SBS_PMCR_ETH_SEL_PHY_RMII

#endif /* __ARCH_ARM_SRC_STM32H5_STM32_ETH_PHYIF_H */
