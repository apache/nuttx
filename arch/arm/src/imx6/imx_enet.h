/****************************************************************************
 * arch/arm/src/imx6/imx_enet.h
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

#ifndef __ARCH_ARM_SRC_IMX6_IMX_ENET_H
#define __ARCH_ARM_SRC_IMX6_IMX_ENET_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/imx_enet.h"

#ifdef CONFIG_IMX_ENET

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Definitions for use with imx_phy_boardinitialize */

#define EMAC_INTF 0

/****************************************************************************
 * Public Function Prototypes
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
 * Function: imx_phy_boardinitialize
 *
 * Description:
 *   Some boards require specialized initialization of the PHY before it can
 *   be used.  This may include such things as configuring GPIOs, resetting
 *   the PHY, etc.  If CONFIG_IMX_ENET_PHYINIT is defined in the
 *   configuration then the board specific logic must provide
 *   imx_phyinitialize();  The i.MX RT Ethernet driver will call this
 *   function one time before it first uses the PHY.
 *
 * Input Parameters:
 *   intf - Always zero for now.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_IMX_ENET_PHYINIT
int imx_phy_boardinitialize(int intf);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_IMX_ENET */
#endif /* __ARCH_ARM_SRC_IMX6_IMX_ENET_H */
