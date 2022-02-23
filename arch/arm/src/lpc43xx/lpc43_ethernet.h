/****************************************************************************
 * arch/arm/src/lpc43xx/lpc43_ethernet.h
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

#ifndef __ARCH_ARM_SRC_LPC43XX_LPC43_ETHERNET_H
#define __ARCH_ARM_SRC_LPC43XX_LPC43_ETHERNET_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#include "hardware/lpc43_ethernet.h"

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Functions Prototypes
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
 * Function: lpc43_phy_boardinitialize
 *
 * Description:
 *   Some boards require specialized initialization of the PHY before it can
 *   be used. This may include such things as configuring GPIOs, resetting
 *   the PHY, etc.  If CONFIG_LPC43_PHYINIT is defined in the configuration
 *   then the board specific logic must provide lpc43_phyinitialize();
 *   The LPC43 Ethernet driver will call this function one time before it
 *   first uses the PHY.
 *
 * Input Parameters:
 *   intf - Always zero for now.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_LPC43_PHYINIT
int lpc43_phy_boardinitialize(int intf);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_LPC43XX_LPC43_ETHERNET_H */
