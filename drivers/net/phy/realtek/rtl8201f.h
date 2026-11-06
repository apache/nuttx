/****************************************************************************
 * drivers/net/phy/realtek/rtl8201f.h
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

#ifndef __DRIVERS_NET_PHY_REALTEK_RTL8201F_H
#define __DRIVERS_NET_PHY_REALTEK_RTL8201F_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PHY ID (revision ignored when matching) */

#define RTL8201F_PHY_ID         0x001cc816
#define RTL8201F_PHY_ID_MASK    0xfffffff0

/* Register pages and page selector */

#define RTL8201F_DEFAULT_PAGE   0
#define RTL8201F_RMII_PAGE      7
#define RTL8201F_PAGE_SELECT    0x1f

/* RMII mode setting register (page 7) */

#define RTL8201F_RMII_MODE      0x10
#define RTL8201F_RMII_SEL       (1 << 3)
#define RTL8201F_REF_CLK_IN     (1 << 12)

#endif /* __DRIVERS_NET_PHY_REALTEK_RTL8201F_H */
