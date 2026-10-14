/****************************************************************************
 * arch/arm/src/rm57/rm57_ethernet.h
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

#ifndef __ARCH_ARM_SRC_RM57_RM57_ETHERNET_H
#define __ARCH_ARM_SRC_RM57_RM57_ETHERNET_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#ifdef CONFIG_RM57_EMAC

/****************************************************************************
 * Name: rm57_ethinitialize
 *
 * Description:
 *   Initialize the RM57L843 EMAC driver and register it with the network
 *   stack as "eth<intf>".  There is exactly one EMAC on this device, so
 *   intf is always 0; the parameter exists for API symmetry with other
 *   NuttX MAC drivers.
 *
 * Input Parameters:
 *   intf    - Interface number, always 0 on this device.
 *   macaddr - 6-byte MAC address to program, or NULL to use a fixed
 *             locally-administered placeholder address (02:00:00:00:00:01).
 *             Board bring-up code should normally pass a real address
 *             derived from the device's unique ID (RM57_SYS_DIEIDL/H).
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

int rm57_ethinitialize(int intf, FAR const uint8_t *macaddr);

#endif /* CONFIG_RM57_EMAC */

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ARCH_ARM_SRC_RM57_RM57_ETHERNET_H */
