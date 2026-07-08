/****************************************************************************
 * arch/arm/src/common/ameba/ameba_wlan.h
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

#ifndef __ARCH_ARM_SRC_COMMON_AMEBA_AMEBA_WLAN_H
#define __ARCH_ARM_SRC_COMMON_AMEBA_AMEBA_WLAN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* One scan result, in plain types so it can cross the SDK<->NuttX boundary
 * (the SDK's struct rtw_scan_result lives behind colliding headers).
 */

#define AMEBA_WLAN_SSID_MAXLEN 32
#define AMEBA_WLAN_MAX_SCAN_AP 48

struct ameba_scan_ap
{
  uint8_t  ssid[AMEBA_WLAN_SSID_MAXLEN + 1]; /* NUL-terminated SSID        */
  uint8_t  ssid_len;                         /* SSID length                */
  uint8_t  bssid[6];                         /* AP MAC                     */
  int16_t  rssi;                             /* signal strength (dBm)      */
  uint8_t  channel;                          /* radio channel              */
  uint8_t  security;                         /* 0 open, else needs PSK     */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ameba_wlan_initialize
 *
 * Description:
 *   Register the WLAN (STA) network device (wlan0).  Call after the WHC
 *   host stack is up (wifi_on).  Returns 0 on success, a negated errno on
 *   failure.
 *
 ****************************************************************************/

int ameba_wlan_initialize(void);

/****************************************************************************
 * Name: ameba_wlan_rxframe
 *
 * Description:
 *   RX entry called by the WHC host RX path (netif_adapter_wifi_recv_whc,
 *   SDK side) with one received Ethernet frame.  Injects it into the NuttX
 *   network stack.  Runs in the WHC host RX task context.
 *
 ****************************************************************************/

void ameba_wlan_rxframe(int idx, const unsigned char *buf, unsigned int len);

#ifdef __cplusplus
}
#endif

#endif /* __ARCH_ARM_SRC_COMMON_AMEBA_AMEBA_WLAN_H */
