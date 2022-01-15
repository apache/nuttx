/****************************************************************************
 * drivers/wireless/ieee80211/bcm43xxx/bcmf_sdpcm.h
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

#ifndef __DRIVERS_WIRELESS_IEEE80211_BCM43XXX_BCMF_SDPCM_H
#define __DRIVERS_WIRELESS_IEEE80211_BCM43XXX_BCMF_SDPCM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "bcmf_driver.h"

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int bcmf_sdpcm_readframe(FAR struct bcmf_dev_s *priv);

int bcmf_sdpcm_sendframe(FAR struct bcmf_dev_s *priv);

int bcmf_sdpcm_queue_frame(FAR struct bcmf_dev_s *priv,
                           struct bcmf_frame_s *frame, bool control);

void bcmf_sdpcm_free_frame(FAR struct bcmf_dev_s *priv,
                           struct bcmf_frame_s *frame);

struct bcmf_frame_s *bcmf_sdpcm_alloc_frame(FAR struct bcmf_dev_s *priv,
                                            unsigned int len, bool block,
                                            bool control);

struct bcmf_frame_s *bcmf_sdpcm_get_rx_frame(FAR struct bcmf_dev_s *priv);

#endif /* __DRIVERS_WIRELESS_IEEE80211_BCM43XXX_BCMF_SDPCM_H */
