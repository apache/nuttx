/****************************************************************************
 * drivers/wireless/ieee80211/bcm43xxx/bcmf_driver.h
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

#ifndef __DRIVERS_WIRELESS_IEEE80211_BCM43XXX_BCMF_DRIVER_H
#define __DRIVERS_WIRELESS_IEEE80211_BCM43XXX_BCMF_DRIVER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>
#include <stdint.h>

#include <nuttx/net/netdev.h>
#include <nuttx/semaphore.h>
#include <net/if.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>

#include "bcmf_ioctl.h"

struct bcmf_dev_s;
struct bcmf_frame_s;

#include "bcmf_bdc.h"

struct bcmf_bus_dev_s;

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Chip interfaces */

#define CHIP_STA_INTERFACE   0
#define CHIP_AP_INTERFACE    1
#define CHIP_P2P_INTERFACE   2

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure contains the unique state of the Broadcom FullMAC driver */

struct bcmf_dev_s
{
  FAR struct bcmf_bus_dev_s *bus; /* Bus interface structure */

  bool bc_bifup;             /* true:ifup false:ifdown */
  struct work_s bc_rxwork;   /* For deferring rx work to the work queue */
  struct work_s bc_pollwork; /* For deferring poll work to the work queue */

  /* This holds the information visible to the NuttX network */

  struct net_driver_s bc_dev;        /* Network interface structure */
  struct bcmf_frame_s *cur_tx_frame; /* Frame used to interface network layer */

  /* Event registration array */

  event_handler_t event_handlers[BCMF_EVENT_COUNT];

  sem_t control_mutex;         /* Cannot handle multiple control requests */
  sem_t control_timeout;       /* Semaphore to wait for control frame rsp */
  uint16_t control_reqid;      /* Current control request id */
  uint16_t control_rxdata_len; /* Received control frame out buffer length */
  uint8_t *control_rxdata;     /* Received control frame out buffer */
  uint32_t control_status;     /* Last received frame status */

  /* AP Scan state machine.
   * During scan, control_mutex is locked to prevent control requests
   */

  int scan_status;                     /* Current scan status */
  struct wdog_s scan_timeout;          /* Scan timeout timer */
  FAR wl_bss_info_t *scan_result;      /* Temp buffer that holds results */
  unsigned int scan_result_entries;    /* Current entries of temp buffer */

  sem_t *auth_signal;   /* Authentication notification signal */
  uint32_t auth_status; /* Authentication status */
  wsec_pmk_t auth_pmk;  /* Authentication pmk */
  bool auth_pending;    /* Authentication pending */

#ifdef CONFIG_IEEE80211_BROADCOM_LOWPOWER
  struct work_s lp_work_ifdown; /* Ifdown work to work queue */
  struct work_s lp_work_dtim;   /* Low power work to work queue */
  int           lp_dtim;        /* Listen interval Delivery Traffic Indication Message */
  sclock_t      lp_ticks;       /* Ticks of last tx time */
#endif
#ifdef CONFIG_IEEE80211_BROADCOM_PTA_PRIORITY
  int pta_priority; /* Current priority of Packet Traffic Arbitration */
#endif
};

/* Default bus interface structure */

struct bcmf_bus_dev_s
{
  void (*stop)(FAR struct bcmf_dev_s *priv);
  int (*txframe)(FAR struct bcmf_dev_s *priv, struct bcmf_frame_s *frame,
                 bool control);
  struct bcmf_frame_s *(*rxframe)(FAR struct bcmf_dev_s *priv);

  /* Frame buffer allocation primitives
   * len     - requested payload length
   * control - true if control frame else false
   * block   - true to block until free frame is available
   */

  struct bcmf_frame_s *(*allocate_frame)(FAR struct bcmf_dev_s *priv,
                                         unsigned int len, bool block,
                                         bool control);

  void (*free_frame)(FAR struct bcmf_dev_s *priv,
                     FAR struct bcmf_frame_s *frame);
};

/* bcmf frame definition */

struct bcmf_frame_s
{
  uint8_t *base; /* Frame base buffer used by low level layer (SDIO) */
  uint8_t *data; /* Payload data (Control, data and event messages) */
  uint16_t len;  /* Frame buffer size */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* IOCTLs network interface implementation */

int bcmf_wl_set_mac_address(FAR struct bcmf_dev_s *priv, struct ifreq *req);

int bcmf_wl_enable(FAR struct bcmf_dev_s *priv, bool enable);

int bcmf_wl_active(FAR struct bcmf_dev_s *priv, bool active);

#ifdef CONFIG_IEEE80211_BROADCOM_LOWPOWER
int bcmf_wl_set_dtim(FAR struct bcmf_dev_s *priv, uint32_t interval_ms);
#endif

int bcmf_wl_set_country_code(FAR struct bcmf_dev_s *priv,
                             int interface, FAR void *code);

/* IOCTLs AP scan interface implementation */

int bcmf_wl_start_scan(FAR struct bcmf_dev_s *priv, struct iwreq *iwr);

int bcmf_wl_get_scan_results(FAR struct bcmf_dev_s *priv, struct iwreq *iwr);

/* IOCTLs authentication interface implementation */

int bcmf_wl_set_auth_param(FAR struct bcmf_dev_s *priv, struct iwreq *iwr);

int bcmf_wl_set_encode_ext(FAR struct bcmf_dev_s *priv, struct iwreq *iwr);

int bcmf_wl_set_mode(FAR struct bcmf_dev_s *priv, struct iwreq *iwr);
int bcmf_wl_get_mode(FAR struct bcmf_dev_s *priv, struct iwreq *iwr);

int bcmf_wl_set_ssid(FAR struct bcmf_dev_s *priv, struct iwreq *iwr);
int bcmf_wl_get_ssid(FAR struct bcmf_dev_s *priv, struct iwreq *iwr);

int bcmf_wl_set_bssid(FAR struct bcmf_dev_s *priv, struct iwreq *iwr);
int bcmf_wl_get_bssid(FAR struct bcmf_dev_s *priv, struct iwreq *iwr);

int bcmf_wl_get_frequency(FAR struct bcmf_dev_s *priv, struct iwreq *iwr);

int bcmf_wl_get_rate(FAR struct bcmf_dev_s *priv, struct iwreq *iwr);

int bcmf_wl_get_txpower(FAR struct bcmf_dev_s *priv, struct iwreq *iwr);

int bcmf_wl_get_rssi(FAR struct bcmf_dev_s *priv, struct iwreq *iwr);

int bcmf_wl_get_iwrange(FAR struct bcmf_dev_s *priv, struct iwreq *iwr);

int bcmf_wl_set_country(FAR struct bcmf_dev_s *priv, struct iwreq *iwr);
int bcmf_wl_get_country(FAR struct bcmf_dev_s *priv, struct iwreq *iwr);

int bcmf_wl_get_channel(FAR struct bcmf_dev_s *priv, int interface);

#ifdef CONFIG_IEEE80211_BROADCOM_PTA_PRIORITY
int bcmf_wl_get_pta(FAR struct bcmf_dev_s *priv, struct iwreq *iwr);
int bcmf_wl_set_pta(FAR struct bcmf_dev_s *priv, struct iwreq *iwr);

int bcmf_wl_set_pta_priority(FAR struct bcmf_dev_s *priv, uint32_t prio);
#else
# define bcmf_wl_get_pta(...)
# define bcmf_wl_set_pta(...)
# define bcmf_wl_set_pta_priority(...)
#endif

#endif /* __DRIVERS_WIRELESS_IEEE80211_BCM43XXX_BCMF_DRIVER_H */
