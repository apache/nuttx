/****************************************************************************
 * arch/risc-v/src/esp32c3/esp_wifi_adapter.h
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

#ifndef __ARCH_RISCV_SRC_ESP32C3_ESP_WIFI_ADAPTER_H
#define __ARCH_RISCV_SRC_ESP32C3_ESP_WIFI_ADAPTER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/wireless/wireless.h>

#include <sys/types.h>

#include "esp_wlan.h"

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

#define SSID_MAX_LEN                (32)
#define PWD_MAX_LEN                 (64)

#define CONFIG_IDF_TARGET_ESP32C3   1

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp_wifi_adapter_init
 *
 * Description:
 *   Initialize ESP32S3 Wi-Fi adapter
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

int esp_wifi_adapter_init(void);

/****************************************************************************
 * Name: esp_wifi_notify_subscribe
 *
 * Description:
 *   Enable event notification
 *
 * Input Parameters:
 *   pid   - Task PID
 *   event - Signal event data pointer
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

int esp_wifi_notify_subscribe(pid_t pid, struct sigevent *event);

#ifdef ESP_WLAN_HAS_STA

/****************************************************************************
 * Name: esp_wifi_sta_start
 *
 * Description:
 *   Start Wi-Fi station.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_sta_start(void);

/****************************************************************************
 * Name: esp_wifi_sta_stop
 *
 * Description:
 *   Stop Wi-Fi station.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_sta_stop(void);

/****************************************************************************
 * Name: esp_wifi_sta_send_data
 *
 * Description:
 *   Use Wi-Fi station interface to send 802.3 frame
 *
 * Input Parameters:
 *   pbuf - Packet buffer pointer
 *   len  - Packet length
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_sta_send_data(void *pbuf, size_t len);

/****************************************************************************
 * Name: esp_wifi_set_password
 *
 * Description:
 *   Set/Get Wi-Fi station password
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set   - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_sta_password(struct iwreq *iwr, bool set);

/****************************************************************************
 * Name: esp_wifi_sta_essid
 *
 * Description:
 *   Set/Get Wi-Fi station ESSID
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set   - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_sta_essid(struct iwreq *iwr, bool set);

/****************************************************************************
 * Name: esp_wifi_sta_bssid
 *
 * Description:
 *   Set/Get Wi-Fi station BSSID
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set   - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_sta_bssid(struct iwreq *iwr, bool set);

/****************************************************************************
 * Name: esp_wifi_sta_connect
 *
 * Description:
 *   Trigger Wi-Fi station connection action
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_sta_connect(void);

/****************************************************************************
 * Name: esp_wifi_sta_disconnect
 *
 * Description:
 *   Trigger Wi-Fi station disconnection action
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_sta_disconnect(void);

/****************************************************************************
 * Name: esp_wifi_sta_mode
 *
 * Description:
 *   Set/Get Wi-Fi Station mode code.
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_sta_mode(struct iwreq *iwr, bool set);

/****************************************************************************
 * Name: esp_wifi_sta_auth
 *
 * Description:
 *   Set/Get station authentication mode params.
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_sta_auth(struct iwreq *iwr, bool set);

/****************************************************************************
 * Name: esp_wifi_sta_freq
 *
 * Description:
 *   Get station frequency.
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_sta_freq(struct iwreq *iwr, bool set);

/****************************************************************************
 * Name: esp_wifi_sta_bitrate
 *
 * Description:
 *   Get station default bit rate (Mbps).
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_sta_bitrate(struct iwreq *iwr, bool set);

/****************************************************************************
 * Name: esp_wifi_sta_get_txpower
 *
 * Description:
 *   Get station transmit power (dBm).
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_sta_txpower(struct iwreq *iwr, bool set);

/****************************************************************************
 * Name: esp_wifi_sta_get_channel_range
 *
 * Description:
 *   Get station range of channel parameters.
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_sta_channel(struct iwreq *iwr, bool set);

/****************************************************************************
 * Name: esp_wifi_sta_country
 *
 * Description:
 *   Configure country info.
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_sta_country(struct iwreq *iwr, bool set);

/****************************************************************************
 * Name: esp_wifi_sta_rssi
 *
 * Description:
 *   Get Wi-Fi sensitivity (dBm).
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_sta_rssi(struct iwreq *iwr, bool set);
#endif /* ESP_WLAN_HAS_STA */

#ifdef ESP_WLAN_HAS_SOFTAP

/****************************************************************************
 * Name: esp_wifi_softap_start
 *
 * Description:
 *   Start Wi-Fi softAP.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_softap_start(void);

/****************************************************************************
 * Name: esp_wifi_softap_stop
 *
 * Description:
 *   Stop Wi-Fi softAP.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_softap_stop(void);

/****************************************************************************
 * Name: esp_wifi_softap_send_data
 *
 * Description:
 *   Use Wi-Fi softAP interface to send 802.3 frame
 *
 * Input Parameters:
 *   pbuf - Packet buffer pointer
 *   len  - Packet length
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_softap_send_data(void *pbuf, size_t len);

/****************************************************************************
 * Name: esp_wifi_softap_password
 *
 * Description:
 *   Set/Get Wi-Fi SoftAP password
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set   - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_softap_password(struct iwreq *iwr, bool set);

/****************************************************************************
 * Name: esp_wifi_softap_essid
 *
 * Description:
 *   Set/Get Wi-Fi SoftAP ESSID
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_softap_essid(struct iwreq *iwr, bool set);

/****************************************************************************
 * Name: esp_wifi_softap_bssid
 *
 * Description:
 *   Set/Get Wi-Fi softAP BSSID
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_softap_bssid(struct iwreq *iwr, bool set);

/****************************************************************************
 * Name: esp_wifi_softap_connect
 *
 * Description:
 *   Trigger Wi-Fi softAP accept connection action
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_softap_connect(void);

/****************************************************************************
 * Name: esp_wifi_softap_disconnect
 *
 * Description:
 *   Trigger Wi-Fi softAP drop connection action
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_softap_disconnect(void);

/****************************************************************************
 * Name: esp_wifi_softap_mode
 *
 * Description:
 *   Set/Get Wi-Fi SoftAP mode code.
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_softap_mode(struct iwreq *iwr, bool set);

/****************************************************************************
 * Name: esp_wifi_softap_auth
 *
 * Description:
 *   Set/Get authentication mode params.
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_softap_auth(struct iwreq *iwr, bool set);

/****************************************************************************
 * Name: esp_wifi_softap_freq
 *
 * Description:
 *   Set/Get SoftAP frequency.
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_softap_freq(struct iwreq *iwr, bool set);

/****************************************************************************
 * Name: esp_wifi_softap_get_bitrate
 *
 * Description:
 *   Get SoftAP default bit rate (Mbps).
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_softap_bitrate(struct iwreq *iwr, bool set);

/****************************************************************************
 * Name: esp_wifi_softap_txpower
 *
 * Description:
 *   Get SoftAP transmit power (dBm).
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_softap_txpower(struct iwreq *iwr, bool set);

/****************************************************************************
 * Name: esp_wifi_softap_channel
 *
 * Description:
 *   Get SoftAP range of channel parameters.
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_softap_channel(struct iwreq *iwr, bool set);

/****************************************************************************
 * Name: esp_wifi_softap_country
 *
 * Description:
 *   Configure country info.
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_softap_country(struct iwreq *iwr, bool set);

/****************************************************************************
 * Name: esp_wifi_softap_rssi
 *
 * Description:
 *   Get Wi-Fi sensitivity (dBm).
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_softap_rssi(struct iwreq *iwr, bool set);
#endif /* ESP_WLAN_HAS_SOFTAP */

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_ESP32C3_ESP_WIFI_ADAPTER_H */
