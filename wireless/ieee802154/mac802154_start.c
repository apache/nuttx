/****************************************************************************
 * wireless/ieee802154/mac802154_start.c
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
 * Public Functions
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

int mac802154_req_start(MACHANDLE mac,
                        FAR struct ieee802154_start_req_s *req)
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
   * attempts to transmit a coordinator realignment command frame as
   * described in 5.1.2.3.2. If the transmission of the coordinator
   * realignment command fails due to a channel access failure, the MLME
   * will not make any changes to the superframe configuration. (i.e., no
   * PIB attributes will be changed).  If the coordinator realignment
   * command is successfully transmitted, the MLME updates the PIB
   * attributes BeaconOrder, SuperframeOrder, PANId, ChannelPage, and
   * ChannelNumber parameters. [1] pg. 106
   */

  if (req->coordrealign)
    {
      /* TODO: Finish the realignment functionality */

      return -ENOTTY;
    }

  /* Set the PANID attribute */

  mac802154_setpanid(priv, req->panid);

  /* Tell the radio layer to set the channel number and channel page */

  priv->radio->setattr(priv->radio,
                       IEEE802154_ATTR_PHY_CHAN,
                       (FAR const union ieee802154_attr_u *)&req->chan);
  priv->radio->setattr(priv->radio,
                       IEEE802154_ATTR_PHY_CURRENT_PAGE,
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

  if (req->beaconorder > 15)
    {
      ret = -EINVAL;
      goto errout;
    }

  priv->sfspec.beaconorder = req->beaconorder;

  /* The value of macSuperframeOrder shall be ignored if
   * macBeaconOrder = 15. pg. 19
   */

  if (priv->sfspec.beaconorder < 15)
    {
      /* Set the superframe order */

      if (req->superframeorder > 15)
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
      /* If the BeaconOrder parameter is less than 15, the MLME sets
       * macBattLifeExt to the value of the BatteryLifeExtension parameter.
       * If the BeaconOrder parameter equals 15, the value of the
       * BatteryLifeExtension parameter is ignored.  [1] pg. 106
       */

      priv->sfspec.ble = req->battlifeext;

      /* For now we just set the CAP Slot to 15 */

      priv->sfspec.final_capslot = 15;

      /* If the PAN coordinator parameter is set to TRUE, the MLME ignores
       * the StartTime parameter and begins beacon transmissions immediately.
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
