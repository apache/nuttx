/****************************************************************************
 * drivers/wireless/ieee80211/bcm43xxx/bcmf_cdc.h
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

#ifndef __DRIVERS_WIRELESS_IEEE80211_BCM43XXX_BCMF_CDC_H
#define __DRIVERS_WIRELESS_IEEE80211_BCM43XXX_BCMF_CDC_H

#include "bcmf_driver.h"
#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Send safe cdc request */

int bcmf_cdc_iovar_request(FAR struct bcmf_dev_s *priv,
                           uint32_t ifidx, bool set, char *name,
                           uint8_t *data, uint32_t *len);

int bcmf_cdc_ioctl(FAR struct bcmf_dev_s *priv, uint32_t ifidx, bool set,
                   uint32_t cmd, uint8_t *data, uint32_t *len);

/* Send cdc request without locking control_mutex */

int bcmf_cdc_iovar_request_unsafe(FAR struct bcmf_dev_s *priv,
                                  uint32_t ifidx, bool set, char *name,
                                  uint8_t *data, uint32_t *len);

/* Callback used by bus layer to notify cdc response frame is available */

int bcmf_cdc_process_control_frame(FAR struct bcmf_dev_s *priv,
                                   struct bcmf_frame_s *frame);

#endif /* __DRIVERS_WIRELESS_IEEE80211_BCM43XXX_BCMF_CDC_H */
