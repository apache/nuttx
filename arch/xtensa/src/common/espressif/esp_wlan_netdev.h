/****************************************************************************
 * arch/xtensa/src/common/espressif/esp_wlan_netdev.h
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

#ifndef __ARCH_XTENSA_SRC_COMMON_ESPRESSIF_ESP_WLAN_NETDEV_H
#define __ARCH_XTENSA_SRC_COMMON_ESPRESSIF_ESP_WLAN_NETDEV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_ESPRESSIF_WIFI_STATION)
#  define ESP_WLAN_HAS_STA
#  define ESP_WLAN_STA_DEVNO    0
#  define ESP_WLAN_DEVS         1
#elif defined(CONFIG_ESPRESSIF_WIFI_SOFTAP)
#  define ESP_WLAN_HAS_SOFTAP
#  define ESP_WLAN_SOFTAP_DEVNO 0
#  define ESP_WLAN_DEVS         1
#elif defined(CONFIG_ESPRESSIF_WIFI_STATION_SOFTAP)
#  define ESP_WLAN_HAS_STA
#  define ESP_WLAN_HAS_SOFTAP
#  define ESP_WLAN_HAS_APSTA
#  define ESP_WLAN_STA_DEVNO    0
#  define ESP_WLAN_SOFTAP_DEVNO 1
#  define ESP_WLAN_DEVS         2
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp_wlan_sta_connect_success_hook
 *
 * Description:
 *   Notify the networking layer that connection has succeeded.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef ESP_WLAN_HAS_STA
void esp_wlan_sta_connect_success_hook(void);
#endif

/****************************************************************************
 * Name: esp_wlan_sta_disconnect_hook
 *
 * Description:
 *   Notify the networking layer that connection has been disconnected.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef ESP_WLAN_HAS_STA
void esp_wlan_sta_disconnect_hook(void);
#endif

/****************************************************************************
 * Name: esp_wlan_softap_connect_success_hook
 *
 * Description:
 *   Notify the networking layer that connection has succeeded.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef ESP_WLAN_HAS_SOFTAP
void esp_wlan_softap_connect_success_hook(void);
#endif

/****************************************************************************
 * Name: esp_wlan_softap_disconnect_hook
 *
 * Description:
 *   Notify the networking layer that connection has been disconnected.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef ESP_WLAN_HAS_SOFTAP
void esp_wlan_softap_disconnect_hook(void);
#endif

/****************************************************************************
 * Name: esp_wlan_sta_initialize
 *
 * Description:
 *   Initialize the Wi-Fi adapter for station mode.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

#ifdef ESP_WLAN_HAS_STA
int esp_wlan_sta_initialize(void);
#endif

/****************************************************************************
 * Name: esp_wlan_softap_initialize
 *
 * Description:
 *   Initialize the Wi-Fi adapter for SoftAP mode.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

#ifdef ESP_WLAN_HAS_SOFTAP
int esp_wlan_softap_initialize(void);
#endif

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_XTENSA_SRC_COMMON_ESPRESSIF_ESP_WLAN_NETDEV_H */
