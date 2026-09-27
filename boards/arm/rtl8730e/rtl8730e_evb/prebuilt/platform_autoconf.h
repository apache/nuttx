/****************************************************************************
 * boards/arm/rtl8730e/rtl8730e_evb/prebuilt/platform_autoconf.h
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

/* Minimal platform_autoconf.h for RTL8730E CA32 NuttX build.
 * Static file -- not auto-generated from SDK menuconfig.
 * Only the defines required by the fwlib sources we compile are listed.
 */

#ifndef __AMEBA_PLATFORM_AUTOCONF_H__
#define __AMEBA_PLATFORM_AUTOCONF_H__

#define CONFIG_AMEBASMART              1
#define CONFIG_ARM_CORE_CA32           1

/* Flash VFS1 partition (SDK defaults for amebasmart NOR layout).
 * XIP base is 0x08000000; offset is absolute XIP address.
 */

#define CONFIG_FLASH_VFS1_OFFSET       0x8600000
#define CONFIG_FLASH_VFS1_SIZE         0x200000

/* Disable LOGUART aggregator (causes SYNC-byte noise on CA32 console). */

#undef  CONFIG_LOGUART_AGG_EN

/* WiFi WHC host (CA32 = WHC HOST, KM4 = WHC DEVICE).
 * Force-included into every WiFi SDK source compiled for CA32.
 */

#define CONFIG_WIFI_POWER_TABLE_USRCFG_3v3_1v25  1
#define CONFIG_WHC_WIFI_API_PATH  1
#define CONFIG_WHC_HOST           1
#define CONFIG_WHC_INTF_IPC       1
#define CONFIG_WLAN               1
#define CONFIG_LWIP_LAYER         1

#endif /* __AMEBA_PLATFORM_AUTOCONF_H__ */
