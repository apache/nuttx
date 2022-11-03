/****************************************************************************
 * drivers/wireless/ieee802154/xbee/xbee_mac.c
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

#include <stdint.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/mm/iob.h>

#include "xbee.h"
#include "xbee_mac.h"

#include <nuttx/wireless/ieee802154/ieee802154_mac.h>
#include <nuttx/wireless/ieee802154/xbee.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define XBEE_ASSOC_POLLDELAY 100

#define XBEE_RESPONSE_TIMEOUT MSEC2TICK(200)

#ifdef CONFIG_XBEE_LOCKUP_WORKAROUND
#define XBEE_LOCKUP_SENDATTEMPTS 20
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void xbee_assoctimer(wdparm_t arg);
static void xbee_assocworker(FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xbee_assoctimer
 *
 * Description:
 *   This function is used to schedule *   an associatioin indication poll.
 *   When association first gets triggered, a watchdog timer is started. This
 *   function is called when it expires. The watchdog timer is scheduled
 *   again until the association is either successful or fails.
 *
 * Input Parameters:
 *   arg  - The argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void xbee_assoctimer(wdparm_t arg)
{
  FAR struct xbee_priv_s *priv = (FAR struct xbee_priv_s *)arg;
  int ret;

  /* In complex environments, we cannot do SPI transfers from the timeout
   * handler because semaphores are probably used to lock the SPI bus.  In
   * this case, we will defer processing to the worker thread.  This is also
   * much kinder in the use of system resources and is, therefore, probably
   * a good thing to do in any event.
   */

  DEBUGASSERT(priv && work_available(&priv->assocwork));

  /* Notice that poll watchdog is not active so further poll timeouts can
   * occur until we restart the poll timeout watchdog.
   */

  ret = work_queue(HPWORK, &priv->assocwork,
                   xbee_assocworker, (FAR void *)priv, 0);
  UNUSED(ret);
  DEBUGASSERT(ret == OK);
}

/****************************************************************************
 * Name: xbee_assocworker
 *
 * Description:
 *   Poll the device for the assosciation status. This function is indirectly
 *   scheduled rom xbee_req_associate in order to poll the device for
 *   association progress.
 *
 * Input Parameters:
 *   arg     - The reference to the driver structure (cast to void*)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void xbee_assocworker(FAR void *arg)
{
  FAR struct xbee_priv_s *priv = (FAR struct xbee_priv_s *)arg;

  if (priv->associating)
    {
      xbee_send_atquery(priv, "AI");

      wd_start(&priv->assocwd, XBEE_ASSOC_POLLDELAY,
               xbee_assoctimer, (wdparm_t)arg);
    }
}

/****************************************************************************
 * Name: xbee_reqdata_timeout
 *
 * Description:
 *   This function runs when a send request has timed out waiting for a
 *   response from the XBee module. This really should never happen, but if
 *   it does, handle it gracefully by retrying the query. Although I still
 *   think this should not happen, it does seem to happen. The XBee seemingly
 *   randomly drops the request and never sends a response.
 *
 * Parameters:
 *   arg  - The argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void xbee_reqdata_timeout(wdparm_t arg)
{
  FAR struct xbee_priv_s *priv = (FAR struct xbee_priv_s *)arg;

  DEBUGASSERT(priv != NULL);

  wlwarn("Send timeout\n");

  /* Wake the pending reqdata thread so it can retry */

  nxsem_post(&priv->txdone_sem);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xbee_bind
 *
 * Description:
 *   Bind the MAC callback table to the XBee driver.
 *
 * Input Parameters:
 *   xbee - Reference to the XBee driver structure
 *   cb   - MAC callback operations
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int xbee_bind(XBEEHANDLE xbee, FAR struct xbee_maccb_s *cb)
{
  FAR struct xbee_priv_s *priv = (FAR struct xbee_priv_s *)xbee;
  FAR struct xbee_maccb_s *next;
  FAR struct xbee_maccb_s *prev;

  /* Add the MAC client callback structure to the list of MAC callbacks in
   * priority order.
   *
   * Search the list to find the location to insert the new instance.
   * The list is maintained in descending priority order.
   */

  for (prev = NULL, next = priv->cb;
      (next != NULL && cb->prio <= next->prio);
       prev = next, next = next->flink);

  /* Add the instance to the spot found in the list.  Check if the instance
   * goes at the head of the list.
   */

  if (prev == NULL)
    {
      cb->flink = priv->cb; /* May be NULL */
      priv->cb  = cb;
    }

  /* No.. the instance goes between prev and next */

  else
    {
      cb->flink   = next; /* May be NULL */
      prev->flink = cb;
    }

  /* Keep track of the number of clients requesting notification */

  if (cb->notify != NULL)
    {
      priv->nclients++;
    }

  return OK;
}

/****************************************************************************
 * Name: xbee_get_mhrlen
 *
 * Description:
 *   Calculate the MAC header length given the frame meta-data. For the XBee,
 *   we use the header to store the entire API frame for the TX request. The
 *   size we need is fixed based on the address mode we are using as it
 *   changes which API frame we need to issue.
 *
 ****************************************************************************/

int xbee_get_mhrlen(XBEEHANDLE xbee,
                    FAR const struct ieee802154_frame_meta_s *meta)
{
  int ret = 9; /* Smallest possible header size */

  /* We assume that the XBee is configured with application header on but
   * encryption not on.
   */

  ret += 2;

  if (meta->srcmode == IEEE802154_ADDRMODE_EXTENDED)
    {
      ret += 6;
    }

  if (meta->destaddr.mode == IEEE802154_ADDRMODE_EXTENDED)
    {
      ret += 6;
    }

  return ret;
}

/****************************************************************************
 * Name: xbee_req_data
 *
 * Description:
 *   The MCPS-DATA.request primitive requests the transfer of a data SPDU
 *   (i.e., MSDU) from a local SSCS entity to a single peer SSCS entity.
 *
 ****************************************************************************/

int xbee_req_data(XBEEHANDLE xbee,
                  FAR const struct ieee802154_frame_meta_s *meta,
                  FAR struct iob_s *frame)
{
  FAR struct xbee_priv_s *priv = (FAR struct xbee_priv_s *)xbee;
  int index;
  uint16_t apiframelen;
  uint8_t frametype;
  int prevoffs = frame->io_offset;
#ifdef CONFIG_XBEE_LOCKUP_WORKAROUND
  int retries = XBEE_LOCKUP_SENDATTEMPTS;
#endif

  /* Support one pending transmit at a time */

  while (nxmutex_lock(&priv->tx_lock) < 0);

  /* Figure out how much room we need to place the API frame header */

  if (meta->destaddr.mode == IEEE802154_ADDRMODE_EXTENDED)
    {
      DEBUGASSERT(frame->io_offset >= 14);
      frame->io_offset -= 14;
      frametype = XBEE_APIFRAME_TXREQ_EADDR;
    }
  else if (meta->destaddr.mode == IEEE802154_ADDRMODE_SHORT)
    {
      DEBUGASSERT(frame->io_offset >= 8);
      frame->io_offset -= 8;
      frametype = XBEE_APIFRAME_TXREQ_SADDR;
    }
  else
    {
      nxmutex_unlock(&priv->tx_lock);
      return -EINVAL;
    }

  index = frame->io_offset;
  apiframelen = (frame->io_len - frame->io_offset - 3);

  frame->io_data[index++] = XBEE_STARTBYTE;
  frame->io_data[index++] = ((apiframelen >> 8) & 0xff);
  frame->io_data[index++] = (apiframelen & 0xff);
  frame->io_data[index++] = frametype;
  frame->io_data[index++] = xbee_next_frameid(priv);

  if (meta->destaddr.mode == IEEE802154_ADDRMODE_EXTENDED)
    {
      frame->io_data[index++] = meta->destaddr.eaddr[7];
      frame->io_data[index++] = meta->destaddr.eaddr[6];
      frame->io_data[index++] = meta->destaddr.eaddr[5];
      frame->io_data[index++] = meta->destaddr.eaddr[4];
      frame->io_data[index++] = meta->destaddr.eaddr[3];
      frame->io_data[index++] = meta->destaddr.eaddr[2];
      frame->io_data[index++] = meta->destaddr.eaddr[1];
      frame->io_data[index++] = meta->destaddr.eaddr[0];
    }
  else
    {
      frame->io_data[index++] = meta->destaddr.saddr[1];
      frame->io_data[index++] = meta->destaddr.saddr[0];
    }

  frame->io_data[index++] = 0; /* Options byte. Currently we do not support anything here */

  DEBUGASSERT(index == prevoffs);

  /* Increment io_len by 1 to account for checksum */

  frame->io_len++;
  xbee_insert_checksum(&frame->io_data[frame->io_offset],
                       (frame->io_len - frame->io_offset));

  priv->txdone = false;

  do
    {
      /* Setup a timeout in case the XBee never responds with a tx status */

      wd_start(&priv->reqdata_wd, XBEE_RESPONSE_TIMEOUT,
               xbee_reqdata_timeout, (wdparm_t)priv);

      /* Send the frame */

      xbee_send_apiframe(priv, &frame->io_data[frame->io_offset],
                         (frame->io_len - frame->io_offset));

      /* Wait for a transmit status to be received. Does not necessarily mean
       * success
       */

      while (nxsem_wait(&priv->txdone_sem) < 0);

      /* If the transmit timeout has occurred, and there are no IOBs
       * available, we may be blocking the context needed to free the IOBs.
       * We cannot receive the Tx status because it requires an IOB.
       * Therefore, if we have hit the timeout, and there are no IOBs, let's
       * move on assuming the transmit was a success
       */

      if (!priv->txdone && iob_navail(false) <= 0)
        {
          wlwarn("Couldn't confirm TX. No IOBs\n");
          break;
        }

#ifdef CONFIG_XBEE_LOCKUP_WORKAROUND
      if (--retries == 0 && !priv->txdone)
        {
          wlerr("XBee not responding. Resetting.\n");
          priv->lower->reset(priv->lower);
          retries = XBEE_LOCKUP_SENDATTEMPTS;
        }
#endif
    }
  while (!priv->txdone);

  nxmutex_unlock(&priv->tx_lock);
  iob_free(frame);
  return OK;
}

/****************************************************************************
 * Name: xbee_req_get
 *
 * Description:
 *   The MLME-GET.request primitive requests information about a given PIB
 *   attribute.
 *
 *   NOTE: The standard specifies that the attribute value should be returned
 *   via the asynchronous MLME-GET.confirm primitive.  However, in our
 *   implementation, we synchronously return the value immediately.Therefore,
 *   we merge the functionality of the MLME-GET.request and MLME-GET.confirm
 *   primitives together.
 *
 ****************************************************************************/

int xbee_req_get(XBEEHANDLE xbee, enum ieee802154_attr_e attr,
                 FAR union ieee802154_attr_u *attrval)
{
  FAR struct xbee_priv_s *priv = (FAR struct xbee_priv_s *)xbee;
  int ret = IEEE802154_STATUS_SUCCESS;

  switch (attr)
    {
      case IEEE802154_ATTR_MAC_PANID:
        {
          xbee_query_panid(priv);
          IEEE802154_PANIDCOPY(attrval->mac.panid, priv->addr.panid);
        }
        break;

      case IEEE802154_ATTR_MAC_SADDR:
        {
          xbee_query_saddr(priv);
          IEEE802154_SADDRCOPY(attrval->mac.saddr, priv->addr.saddr);
        }
        break;

      case IEEE802154_ATTR_MAC_EADDR:
        {
          xbee_query_eaddr(priv);
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

      case IEEE802154_ATTR_PHY_CHAN:
        {
          xbee_query_chan(priv);
          attrval->phy.chan = priv->chan;
        }
        break;

      case IEEE802154_ATTR_PHY_TX_POWER:
        {
          /* TODO: Convert pwrlvl and boost mode settings to int32_t dbm.
           * This depends on whether device is XBee or XBee Pro to do this
           * look-up.
           */

          xbee_query_powerlevel(priv);
          attrval->phy.txpwr = priv->pwrlvl;
        }
        break;

      case IEEE802154_ATTR_PHY_FCS_LEN:
        {
          attrval->phy.fcslen = 2;
          ret = IEEE802154_STATUS_SUCCESS;
        }
        break;

      case IEEE802154_ATTR_PHY_REGDUMP:
        {
          xbee_regdump(priv);
        }
        break;

      default:
        {
          wlwarn("Unsupported attribute\n");
          ret = IEEE802154_STATUS_UNSUPPORTED_ATTRIBUTE;
        }
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: xbee_req_set
 *
 * Description:
 *   The MLME-SET.request primitive attempts to write the given value to the
 *   indicated MAC PIB attribute.
 *
 *   NOTE: The standard specifies that confirmation should be indicated via
 *   the asynchronous MLME-SET.confirm primitive.  However, in our
 *   implementation we synchronously return the status from the request.
 *   Therefore, we do merge the functionality of the MLME-SET.request and
 *   MLME-SET.confirm primitives together.
 *
 ****************************************************************************/

int xbee_req_set(XBEEHANDLE xbee, enum ieee802154_attr_e attr,
                 FAR const union ieee802154_attr_u *attrval)
{
  FAR struct xbee_priv_s *priv = (FAR struct xbee_priv_s *)xbee;
  int ret = IEEE802154_STATUS_SUCCESS;

  switch (attr)
    {
      case IEEE802154_ATTR_MAC_PANID:
        {
          xbee_set_panid(priv, attrval->mac.panid);
        }
        break;

      case IEEE802154_ATTR_MAC_EADDR:
        {
          ret = IEEE802154_STATUS_DENIED;
        }
        break;

      case IEEE802154_ATTR_MAC_SADDR:
        {
          xbee_set_saddr(priv, attrval->mac.saddr);
        }
        break;

      case IEEE802154_ATTR_PHY_CHAN:
        {
          xbee_set_chan(priv, attrval->phy.chan);
        }
        break;

      case IEEE802154_ATTR_MAC_ASSOCIATION_PERMIT:
        {
          if (attrval->mac.assocpermit)
            {
              xbee_set_coordassocflags(priv,
                XBEE_COORDASSOCFLAGS_ALLOWASSOC);
            }
          else
            {
              xbee_set_coordassocflags(priv, 0);
            }
        }
        break;

      case IEEE802154_ATTR_PHY_TX_POWER:
        {
          /* TODO: Convert int32_t dbm input to closest PM/PL settings. Need
           * to know whether device is XBee or XBee Pro to do this look-up.
           */

          xbee_set_powerlevel(priv, attrval->phy.txpwr);
        }
        break;
#if 0
      case IEEE802154_ATTR_MAC_COORD_SADDR:
        {
          xbee_set_coordsaddr(priv, attrval->mac.coordsaddr);
        }
        break;

      case IEEE802154_ATTR_MAC_COORD_EADDR:
        {
          xbee_set_coordeaddr(priv, attrval->mac.coordeaddr);
        }
        break;

      case IEEE802154_ATTR_MAC_RESPONSE_WAIT_TIME:
        {
          priv->resp_waittime = attrval->mac.resp_waittime;
        }
        break;

      case IEEE802154_ATTR_MAC_RX_ON_WHEN_IDLE:
        {
          xbee_setrxonidle(priv, attrval->mac.rxonidle);
        }
        break;
#endif

      default:
        {
          wlwarn("Unsupported attribute\n");
          ret = IEEE802154_STATUS_UNSUPPORTED_ATTRIBUTE;
        }
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: xbee_req_start
 *
 * Description:
 *   The MLME-START.request primitive makes a request for the device to start
 *   acting as a coordinator.  The XBee modules do not support beacon-enabled
 *   networking!
 *
 ****************************************************************************/

int xbee_req_start(XBEEHANDLE xbee, FAR struct ieee802154_start_req_s *req)
{
  FAR struct xbee_priv_s *priv = (FAR struct xbee_priv_s *)xbee;

  if (req->beaconorder != 15)
    {
      wlwarn("xbee: beacon-enabled networks not supported\n");
      return -EINVAL;
    }

  xbee_set_panid(priv, req->panid);
  xbee_set_chan(priv, req->chan);

  xbee_enable_coord(priv, true);
  xbee_set_sleepperiod(priv, 0);

  return OK;
}

/****************************************************************************
 * Name: xbee_req_associate
 *
 * Description:
 *   The MLME-ASSOCIATE.request primitive allows a device to request an
 *   association with a coordinator.
 *
 *   On receipt of the MLME-ASSOCIATE.request primitive, the MLME of an
 *   unassociated device first updates the appropriate PHY and MAC PIB
 *   attributes, as described in 5.1.3.1, and then generates an association
 *   request command, as defined in 5.3.1 [1] pg.80
 *
 ****************************************************************************/

int xbee_req_associate(XBEEHANDLE xbee,
                       FAR struct ieee802154_assoc_req_s *req)
{
  FAR struct xbee_priv_s *priv = (FAR struct xbee_priv_s *)xbee;

  if (req->coordaddr.mode == IEEE802154_ADDRMODE_NONE)
    {
      return -EINVAL;
    }

  xbee_enable_coord(priv, false);

  xbee_set_panid(priv, req->coordaddr.panid);
  xbee_set_chan(priv, req->chan);

  xbee_set_epassocflags(priv, XBEE_EPASSOCFLAGS_AUTOASSOC);

  priv->associating = true;

  /* In order to track the association status, we must poll the device for
   * an update.
   */

  return wd_start(&priv->assocwd, XBEE_ASSOC_POLLDELAY,
                  xbee_assoctimer, (wdparm_t)priv);
}

/****************************************************************************
 * Name: xbee_req_reset
 *
 * Description:
 *   The MLME-RESET.request primitive allows the next higher layer to request
 *   that the MLME performs a reset operation.
 *
 * Input Parameters:
 *   xbee      - Handle to the XBee instance
 *   resetattr - Whether or not to reset the MAC PIB attributes to defaults
 *
 ****************************************************************************/

int xbee_req_reset(XBEEHANDLE xbee, bool resetattr)
{
  FAR struct xbee_priv_s *priv = (FAR struct xbee_priv_s *)xbee;

  /* Reset the XBee radio */

  priv->lower->reset(priv->lower);

  if (resetattr)
    {
      xbee_set_panid(priv, IEEE802154_PANID_UNSPEC);
      xbee_set_saddr(priv, IEEE802154_SADDR_UNSPEC);
      xbee_enable_coord(priv, false);
      xbee_set_epassocflags(priv, 0);
      xbee_set_coordassocflags(priv, 0);
    }

  return OK;
}
