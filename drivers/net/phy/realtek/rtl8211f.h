/****************************************************************************
 * drivers/net/phy/realtek/rtl8211f.h
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

#ifndef __DRIVERS_NET_PHY_REALTEK_RTL8211F_H
#define __DRIVERS_NET_PHY_REALTEK_RTL8211F_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PHY IDs (revision ignored when matching) */

#define RTL8211F_VD_PHY_ID      0x001cc878
#define RTL8211F_PHY_ID         0x001cc916
#define RTL8211F_PHY_ID_MASK    0xfffffff0

/* Register pages and page selector */

#define RTL8211F_DEFAULT_PAGE   0
#define RTL8211F_DELAY_PAGE     0x0d08
#define RTL8211F_PAGE_SELECT    0x1f

/* RGMII delay controls (page 0xd08) */

#define RTL8211F_TX_DELAY_CTRL  0x11
#define RTL8211F_RX_DELAY_CTRL  0x15
#define RTL8211F_TX_DELAY       (1 << 8)
#define RTL8211F_RX_DELAY       (1 << 3)

#endif /* __DRIVERS_NET_PHY_REALTEK_RTL8211F_H */
