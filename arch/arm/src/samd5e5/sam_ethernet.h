/****************************************************************************
 * arch/arm/src/samd5e5/sam_ethernet.h
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

#ifndef __ARCH_ARM_SRC_SAMD5E5_SAM_ETHERNET_H
#define __ARCH_ARM_SRC_SAMD5E5_SAM_ETHERNET_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/sam_gmac.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Understood PHY types */

#define SAMD5E5_PHY_DM9161  0
#define SAMD5E5_PHY_LAN8700 1
#define SAMD5E5_PHY_KSZ8051 2
#define SAMD5E5_PHY_KSZ8081 3
#define SAMD5E5_PHY_KSZ90x1 4

/* Definitions for use with sam_phy_boardinitialize */

#  if defined(CONFIG_ETH0_PHY_DM9161)
#    define SAMD5E5_GMAC_PHY_DM9161  1
#    define SAMD5E5_GMAC_PHY_TYPE    SAMD5E5_PHY_DM9161
#  elif defined(CONFIG_ETH0_PHY_LAN8700)
#    define SAMD5E5_GMAC_PHY_LAN8700 1
#    define SAMD5E5_GMAC_PHY_TYPE    SAMD5E5_PHY_LAN8700
#  elif defined(CONFIG_ETH0_PHY_KSZ8051)
#    define SAMD5E5_GMAC_PHY_KSZ8051 1
#    define SAMD5E5_GMAC_PHY_TYPE    SAMD5E5_PHY_KSZ8051
#  elif defined(CONFIG_ETH0_PHY_KSZ8081)
#    define SAMD5E5_GMAC_PHY_KSZ8081 1
#    define SAMD5E5_GMAC_PHY_TYPE    SAMD5E5_PHY_KSZ8081
#  elif defined(CONFIG_ETH0_PHY_KSZ90x1)
#    define SAMD5E5_GMAC_PHY_KSZ90x1 1
#    define SAMD5E5_GMAC_PHY_TYPE    SAMD5E5_PHY_KSZ90x1
#  else
#    error ETH0 PHY unrecognized
#  endif

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Function: sam_gmac_initialize
 *
 * Description:
 *   Initialize the GMAC driver.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *   Called very early in the initialization sequence.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMD5E5_GMAC
int sam_gmac_initialize(void);
#endif

/****************************************************************************
 * Function: sam_phy_boardinitialize
 *
 * Description:
 *   Some boards require specialized initialization of the PHY before it can
 *   be used.  This may include such things as configuring GPIOs, resetting
 *   the PHY, etc.  If CONFIG_SAMD5E5_GMAC_PHYINIT is defined in the
 *   configuration then the board specific logic must provide
 *   sam_phyinitialize();  The SAMD5E5 Ethernet driver will call this
 *   function one time before it first uses the PHY.
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

#ifdef CONFIG_SAMD5E5_GMAC_PHYINIT
int sam_phy_boardinitialize(int intf);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_SAMD5E5_SAM_ETHERNET_H */
