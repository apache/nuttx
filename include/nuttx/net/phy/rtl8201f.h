/****************************************************************************
 * include/nuttx/net/phy/rtl8201f.h
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

#ifndef __INCLUDE_NUTTX_NET_PHY_RTL8201F_H
#define __INCLUDE_NUTTX_NET_PHY_RTL8201F_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/net/mdio.h>
#include <nuttx/net/mii.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RTL8201F_RESET_DELAY_MS       10
#define RTL8201F_RESET_POST_DELAY_MS  150
#define RTL8201F_RESET_TIMEOUT_MS     500

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern const struct phy_ops_s g_rtl8201f_phy_ops;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: rtl8201f_phy_reset
 *
 * Description:
 *   Reset the RTL8201F PHY and wait for reset completion.
 *
 * Input Parameters:
 *   desc - PHY descriptor containing the PHY address.
 *   bus  - Registered MDIO bus. Caller excludes concurrent PHY access.
 *
 * Returned Value:
 *   OK on success; negated errno on failure or reset timeout.
 *
 ****************************************************************************/

int rtl8201f_phy_reset(FAR const struct phy_desc_s *desc,
                      FAR struct mdio_bus_s *bus);

/****************************************************************************
 * Name: rtl8201f_phy_init
 *
 * Description:
 *   Reset the RTL8201F PHY and configure its interface and RMII clock
 *   direction.  Restore register page zero before returning.
 *
 * Input Parameters:
 *   desc - PHY descriptor containing the address and interface settings.
 *   bus  - Registered MDIO bus. Caller excludes concurrent PHY access.
 *
 * Returned Value:
 *   OK on success; negated errno on failure.
 *
 ****************************************************************************/

int rtl8201f_phy_init(FAR const struct phy_desc_s *desc,
                     FAR struct mdio_bus_s *bus);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_NET_PHY_RTL8201F_H */
