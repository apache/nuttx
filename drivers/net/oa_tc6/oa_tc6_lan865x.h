/****************************************************************************
 * drivers/net/oa_tc6/oa_tc6_lan865x.h
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

#ifndef __DRIVERS_NET_OA_TC6_LAN865X_H
#define __DRIVERS_NET_OA_TC6_LAN865X_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "oa_tc6.h"

/****************************************************************************
 * Preprocessor Macros
 ****************************************************************************/

#define OA_TC6_LAN865X_PHYID 0x0007C1B4U

/* Registers specific to the LAN865x */

#define LAN865X_MAC_NCR_MMS         1
#define LAN865X_MAC_NCR_ADDR        0x0U
#define LAN865X_MAC_NCR_REGID       OA_TC6_MAKE_REGID(LAN865X_MAC_NCR_MMS, LAN865X_MAC_NCR_ADDR)
#define LAN865X_MAC_NCR_TXEN_POS    3
#define LAN865X_MAC_NCR_RXEN_POS    2

#define LAN865X_MAC_NCFGR_MMS       1
#define LAN865X_MAC_NCFGR_ADDR      0x1U
#define LAN865X_MAC_NCFGR_REGID     OA_TC6_MAKE_REGID(LAN865X_MAC_NCFGR_MMS, LAN865X_MAC_NCFGR_ADDR)
#define LAN865X_MAC_NCFGR_EFRHD_POS 25
#define LAN865X_MAC_NCFGR_CAF_POS   4

#define LAN865X_MAC_SAB1_MMS        1
#define LAN865X_MAC_SAB1_ADDR       0x22U
#define LAN865X_MAC_SAB1_REGID      OA_TC6_MAKE_REGID(LAN865X_MAC_SAB1_MMS, LAN865X_MAC_SAB1_ADDR)

#define LAN865X_MAC_SAB_REGID(i)    OA_TC6_MAKE_REGID(LAN865X_MAC_SAB1_MMS, LAN865X_MAC_SAB1_ADDR + 2 * (i - 1))
#define LAN865X_MAC_SAT_REGID(i)    OA_TC6_MAKE_REGID(LAN865X_MAC_SAB1_MMS, LAN865X_MAC_SAB1_ADDR + 2 * (i - 1) + 1)

#endif /* __DRIVERS_NET_OA_TC6_LAN865X_H */
