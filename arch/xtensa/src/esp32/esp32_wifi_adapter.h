/****************************************************************************
 * arch/xtensa/src/esp32/esp32_wifi_adapter.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32_ESP32_WIFI_ADAPTER_H
#define __ARCH_XTENSA_SRC_ESP32_ESP32_WIFI_ADAPTER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/wireless/wireless.h>

#include <sys/types.h>

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

#if defined(CONFIG_ESP32_WIFI_STATION)
#  define ESP32_WLAN_HAS_STA
#  define ESP32_WLAN_STA_DEVNO    0
#  define ESP32_WLAN_DEVS         1
#elif defined(CONFIG_ESP32_WIFI_SOFTAP)
#  define ESP32_WLAN_HAS_SOFTAP
#  define ESP32_WLAN_SOFTAP_DEVNO 0
#  define ESP32_WLAN_DEVS         1
#elif defined(CONFIG_ESP32_WIFI_STATION_SOFTAP)
#  define ESP32_WLAN_HAS_STA
#  define ESP32_WLAN_HAS_SOFTAP
#  define ESP32_WLAN_STA_DEVNO    0
#  define ESP32_WLAN_SOFTAP_DEVNO 1
#  define ESP32_WLAN_DEVS         2
#endif

/* Needed to fix coex_adapter_funcs_t definition */

#define CONFIG_IDF_TARGET_ESP32   1

/* Define esp_err_t */

typedef int esp_err_t;

/* Wi-Fi event ID */

enum wifi_adpt_evt_e
{
  WIFI_ADPT_EVT_SCAN_DONE = 0,
  WIFI_ADPT_EVT_STA_START,
  WIFI_ADPT_EVT_STA_CONNECT,
  WIFI_ADPT_EVT_STA_DISCONNECT,
  WIFI_ADPT_EVT_STA_AUTHMODE_CHANGE,
  WIFI_ADPT_EVT_STA_STOP,
  WIFI_ADPT_EVT_AP_START,
  WIFI_ADPT_EVT_AP_STOP,
  WIFI_ADPT_EVT_AP_STACONNECTED,
  WIFI_ADPT_EVT_AP_STADISCONNECTED,
  WIFI_ADPT_EVT_MAX,
};

enum coex_log_level_e
{
    COEX_LOG_NONE = 0,
    COEX_LOG_ERROR,
    COEX_LOG_WARN,
    COEX_LOG_INFO,
    COEX_LOG_DEBUG,
    COEX_LOG_VERBOSE
};

/* Wi-Fi event callback function */

typedef void (*wifi_evt_cb_t)(void *p);

/* Wi-Fi TX done callback function */

typedef void (*wifi_txdone_cb_t)(uint8_t *data, uint16_t *len, bool status);

#define COEX_ADAPTER_FUNCS_TIME_BLOCKING      0xffffffff

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp_wifi_adapter_init
 *
 * Description:
 *   Initialize ESP32 Wi-Fi adapter
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
 * Name: esp_wifi_free_eb
 *
 * Description:
 *   Free Wi-Fi receive callback input eb pointer
 *
 * Input Parameters:
 *   eb - Wi-Fi receive callback input eb pointer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp_wifi_free_eb(void *eb);

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

#ifdef ESP32_WLAN_HAS_STA

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

int esp_wifi_sta_send_data(void *pbuf, uint32_t len);

/****************************************************************************
 * Name: esp_wifi_sta_register_recv_cb
 *
 * Description:
 *   Regitser Wi-Fi station receive packet callback function
 *
 * Input Parameters:
 *   recv_cb - Receive callback function
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_sta_register_recv_cb(int (*recv_cb)(void *buffer,
                                                 uint16_t len,
                                                 void *eb));

/****************************************************************************
 * Name: esp_wifi_sta_register_txdone_cb
 *
 * Description:
 *   Register the station TX done callback function.
 *
 * Input Parameters:
 *   cb - The callback function
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp_wifi_sta_register_txdone_cb(wifi_txdone_cb_t cb);

/****************************************************************************
 * Name: esp_wifi_sta_read_mac
 *
 * Description:
 *   Read station interface MAC address from efuse
 *
 * Input Parameters:
 *   mac  - MAC address buffer pointer
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

int esp_wifi_sta_read_mac(uint8_t *mac);

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
#endif

#ifdef ESP32_WLAN_HAS_SOFTAP

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

int esp_wifi_softap_send_data(void *pbuf, uint32_t len);

/****************************************************************************
 * Name: esp_wifi_softap_register_recv_cb
 *
 * Description:
 *   Regitser Wi-Fi softAP receive packet callback function
 *
 * Input Parameters:
 *   recv_cb - Receive callback function
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_softap_register_recv_cb(int (*recv_cb)(void *buffer,
                                                    uint16_t len,
                                                    void *eb));

/****************************************************************************
 * Name: esp_wifi_softap_register_txdone_cb
 *
 * Description:
 *   Register the softAP TX done callback function.
 *
 * Input Parameters:
 *   cb - The callback function
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp_wifi_softap_register_txdone_cb(wifi_txdone_cb_t cb);

/****************************************************************************
 * Name: esp_wifi_softap_read_mac
 *
 * Description:
 *   Read softAP interface MAC address from efuse
 *
 * Input Parameters:
 *   mac  - MAC address buffer pointer
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

int esp_wifi_softap_read_mac(uint8_t *mac);

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
 *   set   - true: set data; false: get data
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
#endif /* ESP32_WLAN_HAS_SOFTAP */

/****************************************************************************
 * Name: esp32_wifi_bt_coexist_init
 *
 * Description:
 *   Initialize ESP32 Wi-Fi and BT coexistence module.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32_WIFI_BT_COEXIST
int esp32_wifi_bt_coexist_init(void);
void coex_dbg_set_log_level(int level);
#endif

/****************************************************************************
 * Name: esp_wifi_stop_callback
 *
 * Description:
 *   Callback to stop Wi-Fi
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp_wifi_stop_callback(void);

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_XTENSA_SRC_ESP32_ESP32_WIFI_ADAPTER_H */
