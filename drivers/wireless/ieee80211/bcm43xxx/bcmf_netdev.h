/****************************************************************************
 * drivers/wireless/ieee80211/bcm43xxx/bcmf_netdev.h
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

#ifndef __DRIVERS_WIRELESS_IEEE80211_BCM43XXX_BCMF_NETDEV_H
#define __DRIVERS_WIRELESS_IEEE80211_BCM43XXX_BCMF_NETDEV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "bcmf_driver.h"

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

int bcmf_netdev_register(FAR struct bcmf_dev_s *priv);

void bcmf_netdev_notify_rx(FAR struct bcmf_dev_s *priv);

void bcmf_netdev_notify_tx(FAR struct bcmf_dev_s *priv);

#endif /* __DRIVERS_WIRELESS_IEEE80211_BCM43XXX_BCMF_NETDEV_H */
