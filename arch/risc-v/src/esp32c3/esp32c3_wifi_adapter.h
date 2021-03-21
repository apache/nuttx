/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_wifi_adapter.h
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

#ifndef __ARCH_RISCV_SRC_ESP32C3_ESP32C3_WIFI_ADAPTER_H
#define __ARCH_RISCV_SRC_ESP32C3_ESP32C3_WIFI_ADAPTER_H

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

/* Wi-Fi event ID */

enum wifi_adpt_evt_e
{
  WIFI_ADPT_EVT_STA_START = 0,
  WIFI_ADPT_EVT_STA_CONNECT,
  WIFI_ADPT_EVT_STA_DISCONNECT,
  WIFI_ADPT_EVT_STA_AUTHMODE_CHANGE,
  WIFI_ADPT_EVT_STA_STOP,
  WIFI_ADPT_EVT_MAX,
};

/* Wi-Fi event callback function */

typedef void (*wifi_evt_cb_t)(void *p);

typedef void (* wifi_tx_done_cb_t)(uint8_t ifidx, uint8_t *data,
                                   uint16_t *len, bool txstatus);

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
 *   0 if success or others if fail
 *
 ****************************************************************************/

int esp_wifi_sta_send_data(void *pbuf, uint32_t len);

/****************************************************************************
 * Name: esp_wifi_sta_register_recv_cb
 *
 * Description:
 *   Register Wi-Fi receive packet callback function
 *
 * Input Parameters:
 *   input_cb - Receive callback function
 *
 * Returned Value:
 *   0 if success or others if fail
 *
 ****************************************************************************/

int esp_wifi_sta_register_recv_cb(int (*recv_cb)(void *buffer,
                                                 uint16_t len,
                                                 void *eb));

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
 * Name: esp_wifi_set_password
 *
 * Description:
 *   Set Wi-Fi password
 *
 * Input Parameters:
 *   pdata - Password buffer pointer
 *   len   - Password length
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

int esp_wifi_set_password(const uint8_t *pdata, uint8_t len);

/****************************************************************************
 * Name: esp_wifi_set_ssid
 *
 * Description:
 *   Set Wi-Fi SSID
 *
 * Input Parameters:
 *   pdata - SSID buffer pointer
 *   len   - SSID length
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

int esp_wifi_set_ssid(const uint8_t *pdata, uint8_t len);

/****************************************************************************
 * Name: esp_wifi_connect_internal
 *
 * Description:
 *   Trigger Wi-Fi connection action
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

int esp_wifi_connect_internal(void);

/****************************************************************************
 * Name: esp_wifi_sta_register_txdone_cb
 *
 * Description:
 *   Register the txDone callback function of type wifi_tx_done_cb_t
 *
 * Input Parameters:
 *   callback - The callback function
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

int esp_wifi_sta_register_txdone_cb(void *callback);

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_ESP32C3_ESP32C3_WIFI_ADAPTER_H */
