/****************************************************************************
 * drivers/wireless/ieee802154/mrf24j40/mrf24j40_getset.h
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

#ifndef __DRIVERS_WIRELESS_IEEE802154_MRF24J40_MRF24J40_GETSET_H
#define __DRIVERS_WIRELESS_IEEE802154_MRF24J40_MRF24J40_GETSET_H

int mrf24j40_setrxmode(FAR struct mrf24j40_radio_s *dev, int mode);

int mrf24j40_setchannel(FAR struct mrf24j40_radio_s *dev, uint8_t chan);

int mrf24j40_setpanid(FAR struct mrf24j40_radio_s *dev,
                      FAR const uint8_t *panid);

int mrf24j40_setsaddr(FAR struct mrf24j40_radio_s *dev,
                      FAR const uint8_t *saddr);

int mrf24j40_seteaddr(FAR struct mrf24j40_radio_s *dev,
                      FAR const uint8_t *eaddr);

int mrf24j40_setcoordsaddr(FAR struct mrf24j40_radio_s *dev,
                           FAR const uint8_t *saddr);

int mrf24j40_setcoordeaddr(FAR struct mrf24j40_radio_s *dev,
                           FAR const uint8_t *eaddr);

int mrf24j40_setdevmode(FAR struct mrf24j40_radio_s *dev, uint8_t mode);

int mrf24j40_settxpower(FAR struct mrf24j40_radio_s *dev, int32_t txpwr);

int mrf24j40_setcca(FAR struct mrf24j40_radio_s *dev,
                    FAR struct ieee802154_cca_s *cca);

int mrf24j40_setpamode(FAR struct mrf24j40_radio_s *dev, int mode);

#endif /* __DRIVERS_WIRELESS_IEEE802154_MRF24J40_MRF24J40_GETSET_H */
