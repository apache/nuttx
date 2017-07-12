/****************************************************************************
 * drivers/wireless/ieee802154/mrf24j40/mrf24j40_radif.h
 *
 *   Copyright (C) 2017 Verge Inc. All rights reserved.
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

#ifndef __DRIVERS_WIRELESS_IEEE802154_MRF24J40_RADIF_H
#define __DRIVERS_WIRELESS_IEEE802154_MRF24J40_RADIF_H

int mrf24j40_bind(FAR struct ieee802154_radio_s *radio,
                  FAR struct ieee802154_radiocb_s *radiocb);

int mrf24j40_reset(FAR struct ieee802154_radio_s *radio);

int mrf24j40_getattr(FAR struct ieee802154_radio_s *radio,
                     enum ieee802154_attr_e attr,
                     FAR union ieee802154_attr_u *attrval);

int mrf24j40_setattr(FAR struct ieee802154_radio_s *radio,
                     enum ieee802154_attr_e attr,
                     FAR const union ieee802154_attr_u *attrval);

int mrf24j40_txnotify(FAR struct ieee802154_radio_s *radio, bool gts);

int mrf24j40_txdelayed(FAR struct ieee802154_radio_s *radio,
                      FAR struct ieee802154_txdesc_s *txdesc,
                      uint32_t symboldelay);

int mrf24j40_rxenable(FAR struct ieee802154_radio_s *radio, bool enable);

int mrf24j40_beaconstart(FAR struct ieee802154_radio_s *radio,
                         FAR const struct ieee802154_superframespec_s *sfspec,
                         FAR struct ieee802154_beaconframe_s *beacon);

int mrf24j40_beaconupdate(FAR struct ieee802154_radio_s *radio,
                          FAR struct ieee802154_beaconframe_s *beacon);

int mrf24j40_beaconstop(FAR struct ieee802154_radio_s *radio);

int mrf24j40_sfupdate(FAR struct ieee802154_radio_s *radio,
                      FAR const struct ieee802154_superframespec_s *sfspec);

#endif /* __DRIVERS_WIRELESS_IEEE802154_MRF24J40_RADIF_H */