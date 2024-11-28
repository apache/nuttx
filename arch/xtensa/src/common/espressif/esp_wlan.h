/****************************************************************************
 * arch/xtensa/src/common/espressif/esp_wlan.h
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

#ifndef __ARCH_XTENSA_SRC_COMMON_ESPRESSIF_ESP_WLAN_H
#define __ARCH_XTENSA_SRC_COMMON_ESPRESSIF_ESP_WLAN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_ARCH_CHIP_ESP32S2
#  include "esp32s2_wifi_adapter.h"
#endif

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#ifdef CONFIG_ESPRESSIF_WIFI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef ESPRESSIF_WLAN_HAS_STA

/****************************************************************************
 * Name: esp_wlan_sta_set_linkstatus
 *
 * Description:
 *   Set Wi-Fi station link status
 *
 * Parameters:
 *   linkstatus - true Notifies the networking layer about an available
 *                carrier, false Notifies the networking layer about an
 *                disappeared carrier.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp_wlan_sta_set_linkstatus(bool linkstatus);

/****************************************************************************
 * Name: esp_wlan_sta_initialize
 *
 * Description:
 *   Initialize the ESP32|S2|S3 WLAN station netcard driver
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp_wlan_sta_initialize(void);
#endif /* ESPRESSIF_WLAN_HAS_STA */

/****************************************************************************
 * Name: esp_wlan_softap_initialize
 *
 * Description:
 *   Initialize the ESP32|S2|S3 WLAN softAP netcard driver
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

#ifdef ESPRESSIF_WLAN_HAS_SOFTAP
int esp_wlan_softap_initialize(void);
#endif /* ESPRESSIF_WLAN_HAS_SOFTAP */

#endif /* CONFIG_ESPRESSIF_WIFI */
#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_XTENSA_SRC_COMMON_ESPRESSIF_ESP_WLAN_H */
