/****************************************************************************
 * include/nuttx/net/phy/rtl8211f.h
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

#ifndef __INCLUDE_NUTTX_NET_PHY_RTL8211F_H
#define __INCLUDE_NUTTX_NET_PHY_RTL8211F_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/net/gmii.h>
#include <nuttx/net/mdio.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RTL8211F_RESET_DELAY_MS       10
#define RTL8211F_RESET_POST_DELAY_MS  50
#define RTL8211F_RESET_TIMEOUT_MS     500

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern const struct phy_ops_s g_rtl8211f_phy_ops;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: rtl8211f_phy_reset
 *
 * Description:
 *   Reset the RTL8211F PHY and wait for reset completion.
 *
 * Input Parameters:
 *   desc - PHY descriptor containing the PHY address.
 *   bus  - Registered MDIO bus. Caller excludes concurrent PHY access.
 *
 * Returned Value:
 *   OK on success; negated errno on failure or reset timeout.
 *
 ****************************************************************************/

int rtl8211f_phy_reset(FAR const struct phy_desc_s *desc,
                      FAR struct mdio_bus_s *bus);

/****************************************************************************
 * Name: rtl8211f_phy_init
 *
 * Description:
 *   Reset the RTL8211F PHY and configure RGMII delays for RTL8211F-VD.
 *   RTL8211FDI RXDLY/TXDLY are selected by hardware straps. Restore register
 *   page zero before returning.
 *
 * Input Parameters:
 *   desc - PHY descriptor containing the address and interface settings.
 *   bus  - Registered MDIO bus. Caller excludes concurrent PHY access.
 *
 * Returned Value:
 *   OK on success; negated errno on failure.
 *
 ****************************************************************************/

int rtl8211f_phy_init(FAR const struct phy_desc_s *desc,
                     FAR struct mdio_bus_s *bus);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_NET_PHY_RTL8211F_H */
