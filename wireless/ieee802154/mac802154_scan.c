/****************************************************************************
 * wireless/ieee802154/mac80215_scan.c
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
#include "mac802154_scan.h"

#include <nuttx/wireless/ieee802154/ieee802154_mac.h>

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void mac802154_scantimeout(FAR void *arg);

/****************************************************************************
 * Public MAC Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mac802154_req_scan
 *
 * Description:
 *   The MLME-SCAN.request primitive is used to initiate a channel scan over a
 *   given list of channels. A device can use a channel scan to measure the
 *   energy on the channel, search for the coordinator with which it associated,
 *   or search for all coordinators transmitting beacon frames within the POS of
 *   the scanning device. Scan results are returned
 *   via MULTIPLE calls to the struct mac802154_maccb_s->conf_scan callback.
 *   This is a difference with the official 802.15.4 specification, implemented
 *   here to save memory.
 *
 ****************************************************************************/

int mac802154_req_scan(MACHANDLE mac, FAR struct ieee802154_scan_req_s *req)
{
  FAR struct ieee802154_privmac_s *priv =
    (FAR struct ieee802154_privmac_s *)mac;
  int ret;

  if (req->duration > 15 || req->numchan < 0 || req->numchan > 15)
    {
      ret = -EINVAL;
      goto errout;
    }

  wlinfo("MLME: SCAN.request received\n");

  /* Need to get access to the ops semaphore since operations are serial. This
   * must be done before locking the MAC so that we don't hold the MAC
   */

  ret = mac802154_takesem(&priv->opsem, true);
  if (ret < 0)
    {
      ret = -EINTR;
      goto errout;
    }

  priv->curr_op = MAC802154_OP_SCAN;

  /* Get exclusive access to the MAC */

  ret = mac802154_lock(priv, true);
  if (ret < 0)
    {
      mac802154_givesem(&priv->opsem);
      ret = -EINTR;
      goto errout;
    }

  /* Copy the request so we have a reference */

  memcpy(&priv->currscan, req, sizeof(struct ieee802154_scan_req_s));
  priv->scanindex = 0;
  priv->npandesc = 0;

  switch (req->type)
    {
      case IEEE802154_SCANTYPE_PASSIVE:
        {
          wlinfo("MLME: Starting Passive scan\n");

          /* Set the channel to the first channel in the list */

          mac802154_setchannel(priv, req->channels[priv->scanindex]);
          mac802154_setchpage(priv, req->chpage);

          /* Before commencing an active or passive scan, the MAC sublayer shall
           * store the value of macPANId and then set it to 0xffff for the
           * duration of the scan. This enables the receive filter to accept all
           * beacons rather than just the beacons from its current PAN, as
           * described in 5.1.6.2. On completion of the scan, the MAC sublayer
           * shall restore the value of macPANId to the value stored before the
           * scan began. [1] pg. 24
           */

          IEEE802154_PANIDCOPY(priv->panidbeforescan, priv->addr.panid);
          mac802154_setpanid(priv, (const uint8_t *)&IEEE802154_PANID_UNSPEC);

          /* ...after switching to the channel for a passive scan, the device
           * shall enable its receiver for at most
           * [aBaseSuperframeDuration × (2 * n + 1)],
           * where n is the value of the ScanDuration parameter. [1] pg. 25
           */

          mac802154_rxenable(priv);

          priv->scansymdur = IEEE802154_BASE_SUPERFRAME_DURATION *
                                     ((1 << req->duration) + 1);
          mac802154_timerstart(priv, priv->scansymdur, mac802154_scantimeout);
        }
        break;
      case IEEE802154_SCANTYPE_ACTIVE:
        {
          ret = -ENOTTY;
          goto errout_with_sem;
        }
        break;
      case IEEE802154_SCANTYPE_ED:
        {
          ret = -ENOTTY;
          goto errout_with_sem;
        }
        break;
      case IEEE802154_SCANTYPE_ORPHAN:
        {
          ret = -ENOTTY;
          goto errout_with_sem;
        }
        break;
      default:
        {
          ret = -EINVAL;
          goto errout_with_sem;
        }
        break;
    }

  mac802154_unlock(priv)
return OK;

errout_with_sem:
  mac802154_unlock(priv)
  mac802154_givesem(&priv->opsem);
errout:
  return ret;
}

/****************************************************************************
 * Internal MAC Functions
 ****************************************************************************/

void mac802154_scanfinish(FAR struct ieee802154_privmac_s *priv,
                          enum ieee802154_status_e status)
{
  FAR struct ieee802154_primitive_s * primitive;
  FAR struct ieee802154_scan_conf_s *scanconf;

  primitive = ieee802154_primitive_allocate();
  scanconf = &primitive->u.scanconf;

  primitive->type = IEEE802154_PRIMITIVE_CONF_SCAN;
  scanconf->type = priv->currscan.type;
  scanconf->chpage = priv->currscan.chpage;

  /* Copy in the channels that did not get scanned */

  if (priv->scanindex != priv->currscan.numchan)
    {
      scanconf->numunscanned = priv->currscan.numchan - priv->scanindex;
      memcpy(scanconf->unscanned, &priv->currscan.channels[priv->scanindex],
             scanconf->numunscanned);
    }

  /* Copy the PAN descriptors into the primitive */

  memcpy(scanconf->pandescs, priv->pandescs,
         sizeof(struct ieee802154_pandesc_s) * priv->npandesc);

  scanconf->numdesc = priv->npandesc;
  scanconf->status = status;

  /* Reset the PAN ID to the setting before the scan started */

  mac802154_setpanid(priv, priv->panidbeforescan);

  priv->curr_op = MAC802154_OP_NONE;
  mac802154_givesem(&priv->opsem);

  mac802154_notify(priv, primitive);
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mac802154_scantimeout
 *
 * Description:
 *   Function registered with MAC timer that gets called via the work queue to
 *   handle a timeout for performing a scan operation.
 *
 ****************************************************************************/

static void mac802154_scantimeout(FAR void *arg)
{
  FAR struct ieee802154_privmac_s *priv = (FAR struct ieee802154_privmac_s *)arg;
  DEBUGASSERT(priv->curr_op == MAC802154_OP_SCAN);

  mac802154_lock(priv, false);

  /* If we got here it means we are done scanning that channel */

  mac802154_rxdisable(priv);
  priv->scanindex++;

  /* Check to see if this was the last channel to scan */

  if (priv->scanindex == priv->currscan.numchan)
    {
      if (priv->npandesc > 0)
        {
          mac802154_scanfinish(priv, IEEE802154_STATUS_SUCCESS);
        }
      else
        {
          mac802154_scanfinish(priv, IEEE802154_STATUS_NO_BEACON);
        }
      return;
    }

  mac802154_setchannel(priv, priv->currscan.channels[priv->scanindex]);

  /* ...after switching to the channel for a passive scan, the device
   * shall enable its receiver for at most
   * [aBaseSuperframeDuration × (2 * n + 1)],
   * where n is the value of the ScanDuration parameter. [1] pg. 25
   */

  mac802154_rxenable(priv);
  mac802154_timerstart(priv, priv->scansymdur, mac802154_scantimeout);
  mac802154_unlock(priv);
}
