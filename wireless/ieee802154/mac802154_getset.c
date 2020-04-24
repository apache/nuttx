/****************************************************************************
 * wireless/ieee802154/mac802154_getset.c
 *
 *   Copyright (C) 2016 Sebastien Lorquet. All rights reserved.
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2017 Verge Inc. All rights reserved.
 *
 *   Author: Sebastien Lorquet <sebastien@lorquet.fr>
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
