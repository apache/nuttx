/****************************************************************************
 * drivers/net/phy/realtek/rtl8221b_fw.h
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

#ifndef __DRIVERS_NET_PHY_REALTEK_RTL8221B_FW_H
#define __DRIVERS_NET_PHY_REALTEK_RTL8221B_FW_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stddef.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* A single MMD register write entry used by the RTL8221B firmware patch
 * (ramcode) sequences.  Each entry writes 'value' to register 'addr' via
 * Clause-45 access on the vendor MMD (MII_MMD_VEND2); the MMD is implied
 * rather than stored per-entry to keep these large tables compact.
 */

struct rtl8221b_mmd_reg_s
{
  uint16_t addr;
  uint16_t value;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

extern const struct rtl8221b_mmd_reg_s g_rtl8221b_n0_ramcode[];
extern const size_t g_rtl8221b_n0_ramcode_len;

extern const struct rtl8221b_mmd_reg_s g_rtl8221b_n1_ramcode[];
extern const size_t g_rtl8221b_n1_ramcode_len;

extern const struct rtl8221b_mmd_reg_s g_rtl8221b_n2_ramcode[];
extern const size_t g_rtl8221b_n2_ramcode_len;

extern const struct rtl8221b_mmd_reg_s g_rtl8221b_uc_ramcode[];
extern const size_t g_rtl8221b_uc_ramcode_len;

extern const struct rtl8221b_mmd_reg_s g_rtl8221b_uc2_ramcode[];
extern const size_t g_rtl8221b_uc2_ramcode_len;

extern const struct rtl8221b_mmd_reg_s g_rtl8221b_gphy_tuning[];
extern const size_t g_rtl8221b_gphy_tuning_len;

#ifdef __cplusplus
}
#endif

#endif /* __DRIVERS_NET_PHY_REALTEK_RTL8221B_FW_H */
