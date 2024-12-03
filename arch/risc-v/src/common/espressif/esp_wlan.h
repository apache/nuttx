/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_wlan.h
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

#ifndef __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_WLAN_H
#define __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_WLAN_H

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
#  define ESP_WLAN_STA_DEVNO    0
#  define ESP_WLAN_SOFTAP_DEVNO 1
#  define ESP_WLAN_DEVS         2
#endif

#define MAC_LEN                 (6)

#ifdef CONFIG_ESPRESSIF_WIFI

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nuttx_err_to_freertos
 *
 * Description:
 *   Transform from Nuttx OS error code to FreeRTOS's pdTRUE or pdFALSE.
 *
 * Input Parameters:
 *   ret - NuttX error code
 *
 * Returned Value:
 *   Wi-Fi adapter error code
 *
 ****************************************************************************/

static inline int32_t nuttx_err_to_freertos(int ret)
{
  return ret >= 0;
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef ESP_WLAN_HAS_STA

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
 *   Initialize the ESP32-S3 WLAN station netcard driver
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp_wlan_sta_initialize(void);
#endif /* ESP_WLAN_HAS_STA */

/****************************************************************************
 * Name: esp_wlan_softap_initialize
 *
 * Description:
 *   Initialize the ESP32-S3 WLAN softAP netcard driver
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

#ifdef ESP_WLAN_HAS_SOFTAP
int esp_wlan_softap_initialize(void);
#endif /* ESP_WLAN_HAS_SOFTAP */

/****************************************************************************
 * Name: esp_wifi_tx_done_cb
 *
 * Description:
 *   Wi-Fi TX done callback function.
 *
 * Input Parameters:
 *   ifidx    - The interface id that the tx callback has been triggered from
 *   data     - Pointer to the data transmitted
 *   data_len - Length of the data transmitted
 *   txstatus - True:if the data was transmitted sucessfully False: if data
 *              transmission failed
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

void esp_wifi_tx_done_cb(uint8_t ifidx,
                         uint8_t *data,
                         uint16_t *len,
                         bool txstatus);

#endif /* CONFIG_ESPRESSIF_WIFI */
#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_WLAN_H */
