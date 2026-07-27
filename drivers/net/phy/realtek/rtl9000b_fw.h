/****************************************************************************
 * drivers/net/phy/realtek/rtl9000b_fw.h
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

#ifndef __DRIVERS_NET_PHY_REALTEK_RTL9000B_FW_H
#define __DRIVERS_NET_PHY_REALTEK_RTL9000B_FW_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stddef.h>

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/* DSP / config verification parameter tables, copied verbatim from the
 * vendor reference (RTL9000Bx_Initial_Configuration_Check).
 */

extern const uint16_t g_rtl9000b_param_check[27];
extern const uint16_t g_rtl9000b_param_check2[5];
extern const uint16_t g_rtl9000b_param_check3[3];

#ifdef __cplusplus
}
#endif

#endif /* __DRIVERS_NET_PHY_REALTEK_RTL9000B_FW_H */
