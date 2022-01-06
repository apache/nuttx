/****************************************************************************
 * arch/arm/src/rtl8720c/amebaz_wlan.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <string.h>
#include <syslog.h>
#include "amebaz_netdev.h"

enum _WIFI_EVENT_INDICATE
{
  WIFI_EVENT_CONNECT                = 0,
  WIFI_EVENT_DISCONNECT             = 1,
  WIFI_EVENT_FOURWAY_HANDSHAKE_DONE = 2,
  WIFI_EVENT_SCAN_RESULT_REPORT     = 3,
  WIFI_EVENT_SCAN_DONE              = 4,
  WIFI_EVENT_RECONNECTION_FAIL      = 5,
  WIFI_EVENT_SEND_ACTION_DONE       = 6,
  WIFI_EVENT_RX_MGNT                = 7,
  WIFI_EVENT_STA_ASSOC              = 8,
  WIFI_EVENT_STA_DISASSOC           = 9,
  WIFI_EVENT_STA_WPS_START          = 10,
  WIFI_EVENT_WPS_FINISH             = 11,
  WIFI_EVENT_EAPOL_START            = 12,
  WIFI_EVENT_EAPOL_RECVD            = 13,
  WIFI_EVENT_NO_NETWORK             = 14,
  WIFI_EVENT_BEACON_AFTER_DHCP      = 15,
  WIFI_EVENT_IP_CHANGED             = 16,
  WIFI_EVENT_ICV_ERROR              = 17,
  WIFI_EVENT_CHALLENGE_FAIL         = 18,
  WIFI_EVENT_STA_START              = 19,
  WIFI_EVENT_STA_STOP               = 20,
  WIFI_EVENT_AP_START               = 21,
  WIFI_EVENT_AP_STOP                = 22,
  WIFI_EVENT_STA_GOT_IP             = 23,
  WIFI_EVENT_STA_LOST_IP            = 24,
  WIFI_EVENT_MAX,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void wifi_set_country_code(void)
{
  /* wifi_set_country(RTW_COUNTRY_US); */
}

void wifi_indication(unsigned long event, char *buf, int buf_len, int flags)
{
  if (event != WIFI_EVENT_BEACON_AFTER_DHCP)
    {
      syslog(1, "%s: %d, event: %x\n", __func__, __LINE__, event);
    }
}

void wext_wlan_indicate(unsigned int cmd, union iwreq_data *wrqu,
                        char *extra)
{
  int index = 0;
  if (cmd == IWEVCUSTOM)
    {
      amebaz_wl_connection_handler(index, wrqu, extra);
    }

  else if (cmd == SIOCGIWAP)
    {
      amebaz_wl_connection_handler(index, wrqu, extra);
    }

  else if (cmd == SIOCGIWSCAN)
    {
      amebaz_wl_scan_handler(index, wrqu, extra);
    }

  else
    {
      syslog(1, "%s: %d, event: %x\n", __func__, __LINE__, cmd);
    }
}

void netif_post_sleep_processing(void)
{
}

int netif_is_valid_ip(int index, unsigned char *ip_dest)
{
  return true;
}

unsigned char *rltk_wlan_get_ip(int index)
{
  return NULL;
}

void netif_rx(int index, unsigned int len)
{
  amebaz_wl_notify_rx_handler(index, len);
}

void rltk_wlan_set_netif_info(int index, void *dev, unsigned char *addr)
{
  amebaz_wl_netif_info_handler(index, dev, addr);
}

