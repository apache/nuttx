/****************************************************************************
 * drivers/wireless/ieee80211/bcmf_bdc.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Simon Piriou <spiriou31@gmail.com>
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

#ifndef __DRIVERS_WIRELESS_IEEE80211_BCMF_BDC_H
#define __DRIVERS_WIRELESS_IEEE80211_BCMF_BDC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "bcmf_driver.h"

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Event frame content */

struct __attribute__((packed)) bcmf_event_s
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
};

/* Event callback handler */

typedef void (*event_handler_t)(FAR struct bcmf_dev_s *priv,
                                struct bcmf_event_s *event, unsigned int len);

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

#endif /* __DRIVERS_WIRELESS_IEEE80211_BCMF_BDC_H */