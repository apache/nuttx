/****************************************************************************
 * arch/arm/src/tiva/tiva_ethernet.h
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

#ifndef __ARCH_ARM_SRC_TIVA_TIVA_ETHERNET_H
#define __ARCH_ARM_SRC_TIVA_TIVA_ETHERNET_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
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
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Function: tiva_ethinitialize
 *
 * Description:
 *   Initialize the Ethernet driver for one interface.  If the Tiva/Stellaris
 *   chip supports multiple Ethernet controllers, then board specific logic
 *   must implement arm_netinitialize() and call this function to initialize
 *   the desired interfaces.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

#if TIVA_NETHCONTROLLERS > 1 || defined(CONFIG_NETDEV_LATEINIT)
int tiva_ethinitialize(int intf);
#endif

/****************************************************************************
 * Name: tiva_ethernetmac
 *
 * Description:
 *   For the Ethernet Eval Kits, the MAC address will be stored in the non-
 *   volatile USER0 and USER1 registers.  If CONFIG_TIVA_BOARDMAC is defined,
 *   this function will obtain the MAC address from these registers.
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_BOARDMAC
struct ether_addr;
void tiva_ethernetmac(struct ether_addr *ethaddr);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_TIVA_TIVA_ETHERNET_H */
