/****************************************************************************
 * drivers/wireless/ieee802154/mrf24j40/mrf24j40_getset.h
 *
 *   Copyright (C) 2015-2016 Sebastien Lorquet. All rights reserved.
 *   Copyright (C) 2017 Verge Inc. All rights reserved.
 *   Author: Sebastien Lorquet <sebastien@lorquet.fr>
 *   Author: Anthony Merlino <anthony@vergeaero.com>
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

#ifndef __DRIVERS_WIRELESS_IEEE802154_MRF24J40_GETSET_H
#define __DRIVERS_WIRELESS_IEEE802154_MRF24J40_GETSET_H

int mrf24j40_setrxmode(FAR struct mrf24j40_radio_s *dev, int mode);

int mrf24j40_setchannel(FAR struct mrf24j40_radio_s *dev, uint8_t chan);

int mrf24j40_setpanid(FAR struct mrf24j40_radio_s *dev, FAR const uint8_t *panid);

int mrf24j40_setsaddr(FAR struct mrf24j40_radio_s *dev, FAR const uint8_t *saddr);

int mrf24j40_seteaddr(FAR struct mrf24j40_radio_s *dev, FAR const uint8_t *eaddr);

int mrf24j40_setcoordsaddr(FAR struct mrf24j40_radio_s *dev,
                           FAR const uint8_t *saddr);

int mrf24j40_setcoordeaddr(FAR struct mrf24j40_radio_s *dev,
                           FAR const uint8_t *eaddr);

int mrf24j40_setdevmode(FAR struct mrf24j40_radio_s *dev, uint8_t mode);

int mrf24j40_settxpower(FAR struct mrf24j40_radio_s *dev, int32_t txpwr);

int mrf24j40_setcca(FAR struct mrf24j40_radio_s *dev,
                    FAR struct ieee802154_cca_s *cca);

int mrf24j40_setpamode(FAR struct mrf24j40_radio_s *dev, int mode);

#endif /* __DRIVERS_WIRELESS_IEEE802154_MRF24J40_GETSET_H */
