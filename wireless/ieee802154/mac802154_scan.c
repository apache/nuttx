/****************************************************************************
 * wireless/ieee802154/mac802154_scan.c
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
 *   The MLME-SCAN.request primitive is used to initiate a channel scan over
 *   a given list of channels. A device can use a channel scan to measure the
 *   energy on the channel, search for the coordinator with which it
 *   associated, or search for all coordinators transmitting beacon frames
 *   within the POS of the scanning device. Scan results are returned
 *   via MULTIPLE calls to the struct mac802154_maccb_s->conf_scan callback.
 *   This is a difference with the official 802.15.4 specification,
 *   implemented here to save memory.
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

  /* Need to get access to the ops semaphore since operations are serial.
   * This must be done before locking the MAC so that we don't hold the MAC
   */

  ret = nxsem_wait_uninterruptible(&priv->opsem);
  if (ret < 0)
    {
      goto errout;
    }

  priv->curr_op = MAC802154_OP_SCAN;

  /* Get exclusive access to the MAC */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      nxsem_post(&priv->opsem);
      goto errout;
    }

  /* Copy the request so we have a reference */

  memcpy(&priv->currscan, req, sizeof(struct ieee802154_scan_req_s));
  priv->scanindex = 0;
  priv->npandesc = 0;

  priv->scansymdur = IEEE802154_BASE_SUPERFRAME_DURATION *
                     ((1 << req->duration) + 1);

  switch (req->type)
    {
      case IEEE802154_SCANTYPE_PASSIVE:
        {
          wlinfo("MLME: Starting Passive Scan\n");

          /* Set the channel to the first channel in the list */

          mac802154_setchannel(priv, req->channels[priv->scanindex]);
          mac802154_setchpage(priv, req->chpage);

          /* Before commencing an active or passive scan, the MAC sublayer
           * shall store the value of macPANId and then set it to 0xffff for
           * the duration of the scan. This enables the receive filter to
           * accept all beacons rather than just the beacons from its current
           * PAN, as described in 5.1.6.2. On completion of the scan, the MAC
           * sublayer shall restore the value of macPANId to the value stored
           * before the scan began. [1] pg. 24
           */

          IEEE802154_PANIDCOPY(priv->panidbeforescan, priv->addr.panid);
          mac802154_setpanid(priv,
                             (const uint8_t *) & IEEE802154_PANID_UNSPEC);

          /* ...after switching to the channel for a passive scan, the device
           * shall enable its receiver for at most
           * [aBaseSuperframeDuration × (2 * n + 1)],
           * where n is the value of the ScanDuration parameter. [1] pg. 25
           */

          mac802154_rxenable(priv);
          mac802154_timerstart(priv,
                               priv->scansymdur,
                               mac802154_scantimeout);
        }
        break;
      case IEEE802154_SCANTYPE_ACTIVE:
        {
          ret = -ENOTTY;
          goto errout_with_lock;
        }
        break;
      case IEEE802154_SCANTYPE_ED:
        {
          wlinfo("MLME: Starting Energy Scan\n");

          /* Set the channel to the first channel in the list, and trigger an
           * energy detect operation with the radio layer.
           */

          mac802154_setchpage(priv, req->chpage);
          mac802154_setchannel(priv, req->channels[priv->scanindex]);
          priv->radio->energydetect(priv->radio, priv->scansymdur);
        }
        break;
      case IEEE802154_SCANTYPE_ORPHAN:
        {
          ret = -ENOTTY;
          goto errout_with_lock;
        }
        break;
      default:
        {
          ret = -EINVAL;
          goto errout_with_lock;
        }
        break;
    }

  nxmutex_unlock(&priv->lock);
  return OK;

errout_with_lock:
  nxmutex_unlock(&priv->lock);
  nxsem_post(&priv->opsem);
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

  if (priv->currscan.type == IEEE802154_SCANTYPE_ED)
    {
      /* "The list of energy measurements, one for each channel searched
       *  during an ED scan. This parameter is null for active, passive,
       *  and orphan scans." [1]
       */

      memcpy(scanconf->edlist,
             priv->edlist,
             sizeof(scanconf->edlist));
      memcpy(scanconf->chlist,
             priv->currscan.channels,
             sizeof(scanconf->chlist));
      scanconf->numresults = priv->currscan.numchan;
    }

  else
    {
      /* "A list of the channels given in the request which were not
       *  scanned. This parameter is not valid for ED scans." [1]
       */

      scanconf->numunscanned = priv->currscan.numchan - priv->scanindex;
      if (scanconf->numunscanned)
        {
          memcpy(scanconf->chlist, &priv->currscan.channels[priv->scanindex],
                 scanconf->numunscanned);
        }

      /* "The list of PAN descriptors, one for each beacon found during an
       *  active or passive scan if macAutoRequest is set to TRUE. This
       *  parameter is null for ED and orphan scans or when macAutoRequest
       *  is set to FALSE during an active or passive scan." [1]
       */

      if (priv->currscan.type != IEEE802154_SCANTYPE_ORPHAN && priv->autoreq)
        {
          memcpy(scanconf->pandescs, priv->pandescs,
                 sizeof(struct ieee802154_pandesc_s) * priv->npandesc);
          scanconf->numresults = priv->npandesc;
        }

      if (priv->currscan.type == IEEE802154_SCANTYPE_PASSIVE)
        {
          /* Reset the PAN ID to the setting before the scan started */

          mac802154_setpanid(priv, priv->panidbeforescan);
        }
    }

  scanconf->status = status;

  priv->curr_op = MAC802154_OP_NONE;
  nxsem_post(&priv->opsem);

  mac802154_notify(priv, primitive);
}

/****************************************************************************
 * Name: mac802154_edscan_onresult
 *
 * Description:
 *   Function indirectly called from the radio layer via the radiocb
 *   edresult() call.
 *
 * Assumptions:
 *   Called with the priv mac struct locked
 *
 ****************************************************************************/

void mac802154_edscan_onresult(FAR struct ieee802154_privmac_s *priv,
                               uint8_t edval)
{
  DEBUGASSERT(priv->curr_op == MAC802154_OP_SCAN &&
              priv->currscan.type == IEEE802154_SCANTYPE_ED);

  /* Copy the energy value into our local list */

  priv->edlist[priv->scanindex] = edval;

  /* If we got here it means we are done scanning that channel */

  priv->scanindex++;

  /* Check to see if this was the last channel to scan */

  if (priv->scanindex == priv->currscan.numchan)
    {
      mac802154_scanfinish(priv, IEEE802154_STATUS_SUCCESS);
      return;
    }

  /* Continue on with the next channel in the list */

  mac802154_setchannel(priv, priv->currscan.channels[priv->scanindex]);

  /* ...after switching to the channel for a passive scan, the device
   * shall enable its receiver for at most
   * [aBaseSuperframeDuration × (2 * n + 1)],
   * where n is the value of the ScanDuration parameter. [1] pg. 25
   */

  priv->radio->energydetect(priv->radio, priv->scansymdur);
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mac802154_scantimeout
 *
 * Description:
 *   Function registered with MAC timer that gets called via the work queue
 *   to handle a timeout for performing a scan operation.
 *
 ****************************************************************************/

static void mac802154_scantimeout(FAR void *arg)
{
  FAR struct ieee802154_privmac_s *priv =
             (FAR struct ieee802154_privmac_s *)arg;
  DEBUGASSERT(priv->curr_op == MAC802154_OP_SCAN);

  nxmutex_lock(&priv->lock);

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
  nxmutex_unlock(&priv->lock);
}
