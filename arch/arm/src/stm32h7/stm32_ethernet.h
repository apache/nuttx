/****************************************************************************
 * arch/arm/src/stm32h7/stm32_ethernet.h
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

#ifndef __ARCH_ARM_SRC_STM32H7_STM32_ETHERNET_H
#define __ARCH_ARM_SRC_STM32H7_STM32_ETHERNET_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/stm32_ethernet.h"

#if STM32H7_NETHERNET > 0
#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Function: stm32_ethinitialize
 *
 * Description:
 *   Initialize the Ethernet driver for one interface.  If the STM32 chip
 *   supports multiple Ethernet controllers, then board specific logic must
 *   implement arm_netinitialize() and call this function to initialize the
 *   desired interfaces.
 *
 * Parameters:
 *   intf - In the case where there are multiple EMACs, this value identifies
 *          which EMAC is to be initialized.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

#if STM32H7_NETHERNET > 1 || defined(CONFIG_NETDEV_LATEINIT)
int stm32_ethinitialize(int intf);
#endif

/****************************************************************************
 * Function: stm32_phy_boardinitialize
 *
 * Description:
 *   Some boards require specialized initialization of the PHY before it can
 *   be used.  This may include such things as configuring GPIOs, resetting
 *   the PHY, etc.  If CONFIG_STM32H7_PHYINIT is defined in the configuration
 *   then the board specific logic must provide stm32_phyinitialize();  The
 *   STM32 Ethernet driver will call this function one time before it first
 *   uses the PHY.
 *
 * Parameters:
 *   intf - Always zero for now.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_STM32H7_PHYINIT
int stm32_phy_boardinitialize(int intf);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* STM32H7_NETHERNET > 0 */
#endif /* __ARCH_ARM_SRC_STM32H7_STM32_ETHERNET_H */
