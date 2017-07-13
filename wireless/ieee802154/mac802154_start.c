/****************************************************************************
 * wireless/ieee802154/mac802154_start.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdlib.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <string.h>

#include "mac802154.h"
#include "mac802154_internal.h"

#include <nuttx/wireless/ieee802154/ieee802154_mac.h>

/****************************************************************************
 * Public MAC Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mac802154_req_start
 *
 * Description:
 *   The MLME-START.request primitive makes a request for the device to start
 *   using a new superframe configuration. Confirmation is returned
 *   via the struct mac802154_maccb_s->conf_start callback.
 *
 ****************************************************************************/

int mac802154_req_start(MACHANDLE mac, FAR struct ieee802154_start_req_s *req)
{
  FAR struct ieee802154_privmac_s *priv =
    (FAR struct ieee802154_privmac_s *)mac;
  int ret;

  /* Get exclusive access to the MAC */

  ret = mac802154_lock(priv, true);
  if (ret < 0)
    {
      return ret;
    }

  /* When the CoordRealignment parameter is set to TRUE, the coordinator
   * attempts to transmit a coordinator realignment command frame as described
   * in 5.1.2.3.2. If the transmission of the coordinator realignment command
   * fails due to a channel access failure, the MLME will not make any changes
   * to the superframe configuration. (i.e., no PIB attributes will be changed).
   * If the coordinator realignment command is successfully transmitted, the
   * MLME updates the PIB attributes BeaconOrder, SuperframeOrder, PANId,
   * ChannelPage, and ChannelNumber parameters. [1] pg. 106
   */

  if (req->coordrealign)
    {
      /* TODO: Finish the realignment functionality */

      return -ENOTTY;
    }

  /* Set the PANID attribute */

  mac802154_setpanid(priv, req->panid);

  /* Tell the radio layer to set the channel number and channel page */

  priv->radio->setattr(priv->radio, IEEE802154_ATTR_PHY_CHAN,
                        (FAR const union ieee802154_attr_u *)&req->chan);
  priv->radio->setattr(priv->radio, IEEE802154_ATTR_PHY_CURRENT_PAGE,
                        (FAR const union ieee802154_attr_u *)&req->chpage);

  /* The address used in the Source Address field of the beacon frame shall
   * contain the value of macExtendedAddress if macShortAddress is equal to
   * 0xfffe or macShortAddress otherwise. [1] pg. 32
   */

  if (IEEE802154_SADDRCMP(priv->addr.saddr, &IEEE802154_SADDR_BCAST))
    {
      priv->addr.mode = IEEE802154_ADDRMODE_EXTENDED;
    }
  else
    {
      priv->addr.mode = IEEE802154_ADDRMODE_SHORT;
    }

  /* Set the beacon order */

  if(req->beaconorder > 15)
    {
      ret = -EINVAL;
      goto errout;
    }

  priv->sfspec.beaconorder = req->beaconorder;

  /* The value of macSuperframeOrder shall be ignored if macBeaconOrder = 15. pg. 19 */

  if (priv->sfspec.beaconorder < 15)
    {
      /* Set the superframe order */

      if(req->superframeorder > 15)
        {
          ret = -EINVAL;
          goto errout;
        }

      priv->sfspec.sforder = req->superframeorder;
    }

  if (req->pancoord)
    {
      mac802154_setdevmode(priv, IEEE802154_DEVMODE_PANCOORD);
    }
  else
    {
      mac802154_setdevmode(priv, IEEE802154_DEVMODE_COORD);
    }

  priv->sfspec.pancoord = req->pancoord;

  if (priv->sfspec.beaconorder < 15)
    {
      /* If the BeaconOrder parameter is less than 15, the MLME sets macBattLifeExt to
       * the value of the BatteryLifeExtension parameter. If the BeaconOrder parameter
       * equals 15, the value of the BatteryLifeExtension parameter is ignored.
       * [1] pg. 106
       */

      priv->sfspec.ble = req->battlifeext;

      /* For now we just set the CAP Slot to 15 */

      priv->sfspec.final_capslot = 15;

      /* If the PAN coordinator parameter is set to TRUE, the MLME ignores the
       * StartTime parameter and begins beacon transmissions immediately.
       */

      if (req->pancoord)
        {
          /* Update the beacon frame to start sending */

          mac802154_updatebeacon(priv);

          /* Tell the radio to start transmitting beacon frames */

          priv->radio->beaconstart(priv->radio, &priv->sfspec,
                                   &priv->beaconframe[priv->bf_ind]);
        }
      else
        {
          /* TODO: Finish non-PAN coordinator delayed start */

          ret = -ENOTTY;
          goto errout;
        }
    }

  mac802154_unlock(priv)

  return OK;

errout:
  mac802154_unlock(priv)
  return ret;
}
