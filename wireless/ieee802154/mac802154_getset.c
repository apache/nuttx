/****************************************************************************
 * wireless/ieee802154/mac802154_getset.c
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
 * Name: mac802154_req_get
 *
 * Description:
 *   The MLME-GET.request primitive requests information about a given PIB
 *   attribute.
 *
 *   NOTE: The standard specifies that the attribute value should be returned
 *   via the asynchronous MLME-GET.confirm primitive.
 *   However, in our implementation, we synchronously return the value
 *   immediately. Therefore, we merge the functionality of the
 *   MLME-GET.request and MLME-GET.confirm primitives together.
 *
 ****************************************************************************/

int mac802154_req_get(MACHANDLE mac, enum ieee802154_attr_e attr,
                      FAR union ieee802154_attr_u *attrval)
{
  FAR struct ieee802154_privmac_s *priv =
    (FAR struct ieee802154_privmac_s *)mac;
  int ret = IEEE802154_STATUS_SUCCESS;

  switch (attr)
    {
      case IEEE802154_ATTR_MAC_PANID:
        {
          IEEE802154_PANIDCOPY(attrval->mac.panid, priv->addr.panid);
        }
        break;

      case IEEE802154_ATTR_MAC_SADDR:
        {
          IEEE802154_SADDRCOPY(attrval->mac.saddr, priv->addr.saddr);
        }
        break;

      case IEEE802154_ATTR_MAC_EADDR:
        {
          IEEE802154_EADDRCOPY(attrval->mac.eaddr, priv->addr.eaddr);
        }
        break;

      case IEEE802154_ATTR_MAC_COORD_SADDR:
        {
          IEEE802154_SADDRCOPY(attrval->mac.coordsaddr,
                               priv->pandesc.coordaddr.saddr);
        }
        break;

      case IEEE802154_ATTR_MAC_COORD_EADDR:
        {
          IEEE802154_EADDRCOPY(attrval->mac.coordeaddr,
                               priv->pandesc.coordaddr.eaddr);
        }
        break;

      case IEEE802154_ATTR_MAC_DEVMODE:
        {
          attrval->mac.devmode = priv->devmode;
        }
        break;

      case IEEE802154_ATTR_MAC_RESPONSE_WAIT_TIME:
        {
          attrval->mac.resp_waittime = priv->resp_waittime;
        }
        break;

      case IEEE802154_ATTR_MAC_PROMISCUOUS_MODE:
        {
          attrval->mac.promisc_mode = priv->promisc;
        }
        break;

      case IEEE802154_ATTR_MAC_MAX_FRAME_RETRIES:
        {
          attrval->mac.max_retries = priv->maxretries;
        }
        break;

      case IEEE802154_ATTR_MAC_RX_ON_WHEN_IDLE:
        {
          attrval->mac.rxonidle = priv->rxonidle;
        }
        break;

      default:
        {
          /* The attribute may be handled solely in the radio driver, so pass
           * it along.
           */

          ret = priv->radio->getattr(priv->radio, attr, attrval);
        }
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: mac802154_req_set
 *
 * Description:
 *   The MLME-SET.request primitive attempts to write the given value to the
 *   indicated MAC PIB attribute.
 *
 *   NOTE: The standard specifies that confirmation should be indicated via
 *   the asynchronous MLME-SET.confirm primitive.
 *   However, in our implementation we synchronously return the status from
 *   the request. Therefore, we do merge the functionality of the
 *   MLME-SET.request and MLME-SET.confirm primitives together.
 *
 ****************************************************************************/

int mac802154_req_set(MACHANDLE mac, enum ieee802154_attr_e attr,
                      FAR const union ieee802154_attr_u *attrval)
{
  FAR struct ieee802154_privmac_s *priv =
    (FAR struct ieee802154_privmac_s *)mac;
  int ret = IEEE802154_STATUS_SUCCESS;

  switch (attr)
    {
      case IEEE802154_ATTR_MAC_PANID:
        {
          mac802154_setpanid(priv, attrval->mac.panid);
        }
        break;
      case IEEE802154_ATTR_MAC_SADDR:
        {
          mac802154_setsaddr(priv, attrval->mac.saddr);
        }
        break;
      case IEEE802154_ATTR_MAC_EADDR:
        {
          /* macExtendedAddress is a read-only attribute */

          ret = IEEE802154_STATUS_DENIED;
        }
        break;
      case IEEE802154_ATTR_MAC_COORD_SADDR:
        {
          mac802154_setcoordsaddr(priv, attrval->mac.coordsaddr);
        }
        break;
      case IEEE802154_ATTR_MAC_COORD_EADDR:
        {
          mac802154_setcoordeaddr(priv, attrval->mac.coordeaddr);
        }
        break;
      case IEEE802154_ATTR_MAC_ASSOCIATION_PERMIT:
        {
          priv->sfspec.assocpermit = attrval->mac.assocpermit;
          priv->beaconupdate = true;
        }
        break;
      case IEEE802154_ATTR_MAC_RESPONSE_WAIT_TIME:
        {
          priv->resp_waittime = attrval->mac.resp_waittime;
        }
        break;
      case IEEE802154_ATTR_MAC_RX_ON_WHEN_IDLE:
        {
          mac802154_setrxonidle(priv, attrval->mac.rxonidle);
        }
        break;
      case IEEE802154_ATTR_MAC_PROMISCUOUS_MODE:
        {
          ret = priv->radio->setattr(priv->radio, attr, attrval);
          if (ret == 0)
            {
              priv->promisc = attrval->mac.promisc_mode;
            }
        }
        break;
      case IEEE802154_ATTR_MAC_MAX_FRAME_RETRIES:
        {
          priv->maxretries = attrval->mac.max_retries;
        }
        break;
      default:
        {
          /* The attribute may be handled solely in the radio driver, so pass
           * it along.
           */

          ret = priv->radio->setattr(priv->radio, attr, attrval);
        }
        break;
    }

  return ret;
}
