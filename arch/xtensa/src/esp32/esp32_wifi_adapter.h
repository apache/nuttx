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
#elif defined(CONFIG_ESP32_WIFI_STATION_SOFTAP_COEXISTENCE)
#  define ESP32_WLAN_HAS_STA
#  define ESP32_WLAN_HAS_SOFTAP
#  define ESP32_WLAN_STA_DEVNO    0
#  define ESP32_WLAN_SOFTAP_DEVNO 1
#  define ESP32_WLAN_DEVS         2
#endif

/* WiFi event ID */

enum wifi_adpt_evt_e
{
  WIFI_ADPT_EVT_STA_START = 0,
  WIFI_ADPT_EVT_STA_CONNECT,
  WIFI_ADPT_EVT_STA_DISCONNECT,
  WIFI_ADPT_EVT_STA_AUTHMODE_CHANGE,
  WIFI_ADPT_EVT_STA_STOP,
  WIFI_ADPT_EVT_MAX,
};

/* WiFi event callback function */

typedef void (*wifi_evt_cb_t)(void *p);

/* WiFi TX done callback function */

typedef void (*wifi_txdone_cb_t)(uint8_t *data, uint16_t *len, bool status);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp_wifi_adapter_init
 *
 * Description:
 *   Initialize ESP32 WiFi adapter
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
 *   Free WiFi receive callback input eb pointer
 *
 * Input Parameters:
 *   eb - WiFi receive callback input eb pointer
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

int esp_wifi_notify_subscribe(pid_t pid, FAR struct sigevent *event);

#ifdef ESP32_WLAN_HAS_STA

/****************************************************************************
 * Name: esp_wifi_sta_start
 *
 * Description:
 *   Start WiFi station.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

int esp_wifi_sta_start(void);

/****************************************************************************
 * Name: esp_wifi_sta_stop
 *
 * Description:
 *   Stop WiFi station.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

int esp_wifi_sta_stop(void);

/****************************************************************************
 * Name: esp_wifi_sta_send_data
 *
 * Description:
 *   Use WiFi station interface to send 802.3 frame
 *
 * Input Parameters:
 *   pbuf - Packet buffer pointer
 *   len  - Packet length
 *
 * Returned Value:
 *   0 if success or others if fail
 *
 ****************************************************************************/

int esp_wifi_sta_send_data(void *pbuf, uint32_t len);

/****************************************************************************
 * Name: esp_wifi_sta_register_recv_cb
 *
 * Description:
 *   Regitser WiFi station receive packet callback function
 *
 * Input Parameters:
 *   recv_cb - Receive callback function
 *
 * Returned Value:
 *   0 if success or others if fail
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
 *   Set WiFi station password
 *
 * Input Parameters:
 *   pdata - Password buffer pointer
 *   len   - Password length
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

int esp_wifi_sta_set_password(const uint8_t *pdata, uint8_t len);

/****************************************************************************
 * Name: esp_wifi_set_ssid
 *
 * Description:
 *   Set WiFi station SSID
 *
 * Input Parameters:
 *   pdata - SSID buffer pointer
 *   len   - SSID length
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

int esp_wifi_sta_set_ssid(const uint8_t *pdata, uint8_t len);

/****************************************************************************
 * Name: esp_wifi_sta_connect
 *
 * Description:
 *   Trigger WiFi station connection action
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

int esp_wifi_sta_connect(void);

/****************************************************************************
 * Name: esp_wifi_sta_disconnect
 *
 * Description:
 *   Trigger WiFi station disconnection action
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

int esp_wifi_sta_disconnect(void);
#endif

#ifdef ESP32_WLAN_HAS_SOFTAP

/****************************************************************************
 * Name: esp_wifi_softap_start
 *
 * Description:
 *   Start WiFi softAP.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

int esp_wifi_softap_start(void);

/****************************************************************************
 * Name: esp_wifi_softap_stop
 *
 * Description:
 *   Stop WiFi softAP.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

int esp_wifi_softap_stop(void);

/****************************************************************************
 * Name: esp_wifi_softap_send_data
 *
 * Description:
 *   Use WiFi softAP interface to send 802.3 frame
 *
 * Input Parameters:
 *   pbuf - Packet buffer pointer
 *   len  - Packet length
 *
 * Returned Value:
 *   0 if success or others if fail
 *
 ****************************************************************************/

int esp_wifi_softap_send_data(void *pbuf, uint32_t len);

/****************************************************************************
 * Name: esp_wifi_softap_register_recv_cb
 *
 * Description:
 *   Regitser WiFi softAP receive packet callback function
 *
 * Input Parameters:
 *   recv_cb - Receive callback function
 *
 * Returned Value:
 *   0 if success or others if fail
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
 * Name: esp_wifi_softap_set_password
 *
 * Description:
 *   Set WiFi softAP password
 *
 * Input Parameters:
 *   pdata - Password buffer pointer
 *   len   - Password length
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

int esp_wifi_softap_set_password(const uint8_t *pdata, uint8_t len);

/****************************************************************************
 * Name: esp_wifi_softap_set_ssid
 *
 * Description:
 *   Set WiFi softAP SSID
 *
 * Input Parameters:
 *   pdata - SSID buffer pointer
 *   len   - SSID length
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

int esp_wifi_softap_set_ssid(const uint8_t *pdata, uint8_t len);

/****************************************************************************
 * Name: esp_wifi_softap_connect
 *
 * Description:
 *   Trigger WiFi softAP accept connection action
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

int esp_wifi_softap_connect(void);

/****************************************************************************
 * Name: esp_wifi_softap_disconnect
 *
 * Description:
 *   Trigger WiFi softAP drop connection action
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

int esp_wifi_softap_disconnect(void);
#endif

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_XTENSA_SRC_ESP32_ESP32_WIFI_ADAPTER_H */
