/****************************************************************************
 * arch/arm/src/rtl8720c/amebaz_driver.h
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

#ifndef __ARCH_ARM_SRC_RTL8720C_AMEBAZ_DRIVER_H
#define __ARCH_ARM_SRC_RTL8720C_AMEBAZ_DRIVER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/semaphore.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>
#include <nuttx/net/netdev.h>
#include "amebaz_wlan.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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
  sem_t                   sem;
  struct wdog_s           timeout;
  int status;
};

struct amebaz_associate_s
{
  rtw_ssid_t              ssid;
  rtw_mac_t               mac;
  uint8_t alg;
  unsigned int channel;
  uint8_t mask;
};

struct amebaz_dev_s
{
  struct net_driver_s       dev;
  int devnum;
  struct amebaz_state_s     scan;
  struct amebaz_state_s     conn;
  struct work_s             pollwork;
  struct wdog_s             txpoll;
  struct sk_buff            *curr;
  int mode;
  struct amebaz_associate_s assoc;
  rtw_scan_result_t         scan_data[AMEBAZ_SCAN_AP_COUNT];
  unsigned int scan_count;
  unsigned char             country[2];
};

int amebaz_wl_start_scan(struct amebaz_dev_s *priv,
                         struct iwreq *iwr);
int amebaz_wl_get_scan_results(struct amebaz_dev_s *priv,
                               struct iwreq *iwr);
int amebaz_wl_set_encode_ext(struct amebaz_dev_s *priv,
                             struct iwreq *iwr);
int amebaz_wl_get_encode_ext(struct amebaz_dev_s *priv,
                             struct iwreq *iwr);
int amebaz_wl_set_ssid(struct amebaz_dev_s *priv,
                       struct iwreq *iwr);
int amebaz_wl_set_bssid(struct amebaz_dev_s *priv,
                        struct iwreq *iwr);
int amebaz_wl_set_mode(struct amebaz_dev_s *priv,
                       struct iwreq *iwr);
int amebaz_wl_set_country(struct amebaz_dev_s *priv,
                          struct iwreq *iwr);
int amebaz_wl_get_freq(struct amebaz_dev_s *priv,
                       struct iwreq *iwr);
int amebaz_wl_set_freq(struct amebaz_dev_s *priv,
                       struct iwreq *iwr);
int amebaz_wl_process_command(struct amebaz_dev_s *priv,
                              int cmd, void *req);
void amebaz_wl_connection_handler(int index,
                                  union iwreq_data *wrqu, char *extra);
void amebaz_wl_scan_handler(int index,
                            union iwreq_data *wrqu, char *extra);
void amebaz_wl_netif_info_handler(int index, void *dev,
                                  unsigned char *addr);
void amebaz_wl_notify_rx_handler(int index, unsigned int len);

#endif /* __ARCH_ARM_SRC_RTL8720C_AMEBAZ_DRIVER_H */
