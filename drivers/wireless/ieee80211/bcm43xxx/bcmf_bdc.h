/****************************************************************************
 * drivers/wireless/ieee80211/bcm43xxx/bcmf_bdc.h
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

#ifndef __DRIVERS_WIRELESS_IEEE80211_BCM43XXX_BCMF_BDC_H
#define __DRIVERS_WIRELESS_IEEE80211_BCM43XXX_BCMF_BDC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "bcmf_driver.h"

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Event frame content */

begin_packed_struct struct bcmf_event_s
{
  uint16_t version;       /* Vendor specific type */
  uint16_t flags;
  uint32_t type;          /* Id of received event */
  uint32_t status;        /* Event status code */
  uint32_t reason;        /* Reason code */
  uint32_t auth_type;
  uint32_t len;           /* Data size following this header */
  struct ether_addr addr; /* AP MAC address */
  char     src_name[16];  /* Event source interface name */
  uint8_t  dst_id;        /* Event destination interface id */
  uint8_t  bss_cfg_id;
} end_packed_struct;

/* Event callback handler */

typedef void (*event_handler_t)(FAR struct bcmf_dev_s *priv,
                                struct bcmf_event_s *event,
                                unsigned int len);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Function called from lower layer */

int bcmf_bdc_process_event_frame(FAR struct bcmf_dev_s *priv,
                                 struct bcmf_frame_s *frame);

/* Function called from upper layer */

struct bcmf_frame_s *bcmf_bdc_allocate_frame(FAR struct bcmf_dev_s *priv,
                                             uint32_t len, bool block);

int bcmf_bdc_transmit_frame(FAR struct bcmf_dev_s *priv,
                            struct bcmf_frame_s *frame);

struct bcmf_frame_s *bcmf_bdc_rx_frame(FAR struct bcmf_dev_s *priv);

/* Event frames API */

int bcmf_event_register(FAR struct bcmf_dev_s *priv, event_handler_t handler,
                        unsigned int event_id);

int bcmf_event_unregister(FAR struct bcmf_dev_s *priv,
                          unsigned int event_id);

int bcmf_event_push_config(FAR struct bcmf_dev_s *priv);

#endif /* __DRIVERS_WIRELESS_IEEE80211_BCM43XXX_BCMF_BDC_H */
