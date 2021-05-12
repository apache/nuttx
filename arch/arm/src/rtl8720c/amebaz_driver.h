/****************************************************************************
 * drivers/wireless/ieee80211/amebaz/amebaz_depend.h
 *
 *   Copyright (C) 2019 Xiaomi Inc. All rights reserved.
 *   Author: Chao An <anchao@xiaomi.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __DRIVERS_WIRELESS_IEEE80211_AMEBAZ_AMEBAZ_DRIVER_H
#define __DRIVERS_WIRELESS_IEEE80211_AMEBAZ_AMEBAZ_DRIVER_H

#include <nuttx/config.h>

#include <semaphore.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>
#include <nuttx/net/netdev.h>

#include "amebaz_wlan.h"

#define AMEBAZ_SCAN_AP_COUNT        (10)

enum
{
  AMEBAZ_STATUS_DONE = 0,
  AMEBAZ_STATUS_DISABLED,
  AMEBAZ_STATUS_RUN,
  AMEBAZ_STATUS_TIMEOUT,
};

enum
{
  AMEBAZ_ASSOCIATE_SSID    = 0x1,
  AMEBAZ_ASSOCIATE_BSSID   = 0x2,
  AMEBAZ_ASSOCIATE_ALG     = 0x4,
  AMEBAZ_ASSOCIATE_CHANNEL = 0x8,
  AMEBAZ_ASSOCIATE_MASK    = 0xf,
};

struct amebaz_state_s
{
  sem_t                   mutex;
  struct wdog_s           timeout;
  int                     status;
};

struct amebaz_associate_s {
  rtw_ssid_t              ssid;
  rtw_mac_t               mac;
  uint8_t                 alg;
  unsigned int            channel;
  uint8_t                 mask;
};

struct amebaz_dev_s
{
  struct net_driver_s       dev;
  int                       devnum;

  struct amebaz_state_s     scan;
  struct amebaz_state_s     conn;

  struct work_s             pollwork;
  struct wdog_s             txpoll;

  struct sk_buff            *curr;

  int                       mode;

  struct amebaz_associate_s assoc;

  rtw_scan_result_t         scan_data[AMEBAZ_SCAN_AP_COUNT];
  unsigned int              scan_count;

  unsigned char             country[2];
};

int amebaz_wl_start_scan          (FAR struct amebaz_dev_s *priv, struct iwreq *iwr);
int amebaz_wl_get_scan_results    (FAR struct amebaz_dev_s *priv, struct iwreq *iwr);
int amebaz_wl_set_encode_ext      (FAR struct amebaz_dev_s *priv, struct iwreq *iwr);
int amebaz_wl_get_encode_ext      (FAR struct amebaz_dev_s *priv, struct iwreq *iwr);
int amebaz_wl_set_ssid            (FAR struct amebaz_dev_s *priv, struct iwreq *iwr);
int amebaz_wl_set_bssid           (FAR struct amebaz_dev_s *priv, struct iwreq *iwr);
int amebaz_wl_set_mode            (FAR struct amebaz_dev_s *priv, struct iwreq *iwr);
int amebaz_wl_set_country         (FAR struct amebaz_dev_s *priv, struct iwreq *iwr);
int amebaz_wl_get_freq            (FAR struct amebaz_dev_s *priv, struct iwreq *iwr);
int amebaz_wl_set_freq            (FAR struct amebaz_dev_s *priv, struct iwreq *iwr);

int amebaz_wl_process_command     (FAR struct amebaz_dev_s *priv, int cmd, void *req);

void amebaz_wl_connection_handler (int index, union iwreq_data *wrqu, char *extra);
void amebaz_wl_scan_handler       (int index, union iwreq_data *wrqu, char *extra);
void amebaz_wl_netif_info_handler (int index, void *dev, unsigned char *addr);
void amebaz_wl_notify_rx_handler  (int index, unsigned int len);

#endif /* __DRIVERS_WIRELESS_IEEE80211_AMEBAZ_AMEBAZ_DRIVER_H */
