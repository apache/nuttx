/****************************************************************************
 * wireless/ieee802154/mac802154_assoc.c
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

#include <nuttx/wqueue.h>
#include <nuttx/mm/iob.h>

#include "mac802154.h"
#include "mac802154_internal.h"
#include "mac802154_assoc.h"

#include <nuttx/wireless/ieee802154/ieee802154_mac.h>

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void mac802154_assoctimeout(FAR void *arg);
static void mac802154_extract_assocresp(FAR void *arg);

/****************************************************************************
 * Public MAC Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mac802154_req_associate
 *
 * Description:
 *   The MLME-ASSOCIATE.request primitive allows a device to request an
 *   association with a coordinator. Confirmation is returned via the
 *   struct mac802154_maccb_s->conf_associate callback.
 *
 *   On receipt of the MLME-ASSOCIATE.request primitive, the MLME of an
 *   unassociated device first updates the appropriate PHY and MAC PIB
 *   attributes, as described in 5.1.3.1, and then generates an association
 *   request command, as defined in 5.3.1 [1] pg.80
 *
 ****************************************************************************/

int mac802154_req_associate(MACHANDLE mac,
                            FAR struct ieee802154_assoc_req_s *req)
{
  FAR struct ieee802154_privmac_s *priv =
    (FAR struct ieee802154_privmac_s *)mac;
  FAR struct ieee802154_txdesc_s *txdesc;
  FAR struct iob_s *iob;
  int ret;
  int i;

  if (req->coordaddr.mode == IEEE802154_ADDRMODE_NONE)
    {
      return -EINVAL;
    }

  /* Get exclusive access to the operation semaphore. This must happen before
   * getting exclusive access to the MAC struct or else there could be a
   * lockup condition. This would occur if another thread is using the
   * cmdtrans but needs access to the MAC in order to unlock it.
   */

  ret = mac802154_takesem(&priv->opsem, true);
  if (ret < 0)
    {
      return ret;
    }

  priv->curr_op = MAC802154_OP_ASSOC;
  priv->curr_cmd = IEEE802154_CMD_ASSOC_REQ;

  /* Get exclusive access to the MAC */

  ret = mac802154_lock(priv, true);
  if (ret < 0)
    {
      mac802154_givesem(&priv->opsem);
      return ret;
    }

  /* Set the channel and channel page of the PHY layer */

  mac802154_setchannel(priv, req->chan);
  mac802154_setchpage(priv, req->chpage);

  /* Set the coordinator address attributes */

  mac802154_setcoordaddr(priv, &req->coordaddr);

  /* Copy the coordinator PAN ID to our PAN ID */

  mac802154_setpanid(priv, req->coordaddr.panid);

  /* Copy in the capabilities information bitfield */

  if (req->capabilities.devtype)
    {
      mac802154_setdevmode(priv, IEEE802154_DEVMODE_COORD);
    }
  else
    {
      mac802154_setdevmode(priv, IEEE802154_DEVMODE_ENDPOINT);
    }

  mac802154_setrxonidle(priv, req->capabilities.rxonidle);

  /* Allocate an IOB to put the frame in */

  iob = iob_alloc(false);
  DEBUGASSERT(iob != NULL);

  iob->io_flink  = NULL;
  iob->io_len    = 0;
  iob->io_offset = 0;
  iob->io_pktlen = 0;

  /* Allocate the txdesc, waiting if necessary */

  ret = mac802154_txdesc_alloc(priv, &txdesc, true);
  if (ret < 0)
    {
      iob_free(iob);
      mac802154_unlock(priv)
      mac802154_givesem(&priv->opsem);
      return ret;
    }

  /* Get a uin16_t reference to the first two bytes. ie frame control field */

  iob->io_data[0] = 0;
  iob->io_data[1] = 0;

  IEEE802154_SETACKREQ(iob->io_data, 0);
  IEEE802154_SETFTYPE(iob->io_data, 0, IEEE802154_FRAME_COMMAND);
  IEEE802154_SETDADDRMODE(iob->io_data, 0, priv->pandesc.coordaddr.mode);
  IEEE802154_SETSADDRMODE(iob->io_data, 0, IEEE802154_ADDRMODE_EXTENDED);

  iob->io_len = 2;

  /* Each time a data or a MAC command frame is generated, the MAC sublayer
   * shall copy the value of macDSN into the Sequence Number field of the MHR
   * of the outgoing frame and then increment it by one. [1] pg. 40.
   */

  iob->io_data[iob->io_len++] = priv->dsn++;

  /* The Destination PAN Identifier field shall contain the identifier of the
   * PAN to which to associate. [1] pg. 68
   */

  mac802154_putpanid(iob, priv->pandesc.coordaddr.panid);

  /* The Destination Address field shall contain the address from the beacon
   * frame that was transmitted by the coordinator to which the association
   * request command is being sent. [1] pg. 68
   */

  if (priv->pandesc.coordaddr.mode == IEEE802154_ADDRMODE_SHORT)
    {
      mac802154_putsaddr(iob, priv->pandesc.coordaddr.saddr);
    }
  else if (priv->pandesc.coordaddr.mode == IEEE802154_ADDRMODE_EXTENDED)
    {
      mac802154_puteaddr(iob, priv->pandesc.coordaddr.eaddr);
    }

  /* The Source PAN Identifier field shall contain the broadcast PAN
   * identifier.
   */

  mac802154_putsaddr(iob, &IEEE802154_SADDR_BCAST);

  /* The Source Address field shall contain the value of
   * macExtendedAddress.
   */

  mac802154_puteaddr(iob, priv->addr.eaddr);

  /* Copy in the Command Frame Identifier */

  iob->io_data[iob->io_len++] = IEEE802154_CMD_ASSOC_REQ;

  /* Copy in the capability information bits */

  iob->io_data[iob->io_len] = 0;
  iob->io_data[iob->io_len] |= (req->capabilities.devtype <<
                                IEEE802154_CAPABILITY_SHIFT_DEVTYPE);
  iob->io_data[iob->io_len] |= (req->capabilities.powersource <<
                                IEEE802154_CAPABILITY_SHIFT_PWRSRC);
  iob->io_data[iob->io_len] |= (req->capabilities.rxonidle <<
                                IEEE802154_CAPABILITY_SHIFT_RXONIDLE);
  iob->io_data[iob->io_len] |= (req->capabilities.security <<
                                IEEE802154_CAPABILITY_SHIFT_SECURITY);
  iob->io_data[iob->io_len] |= (req->capabilities.allocaddr <<
                                IEEE802154_CAPABILITY_SHIFT_ALLOCADDR);

  iob->io_len++;

  txdesc->frame = iob;
  txdesc->frametype = IEEE802154_FRAME_COMMAND;
  txdesc->ackreq = true;

  /* Save a copy of the destination addressing information into the tx
   * descriptor.  We only do this for commands to help with handling their
   * progession.
   */

  memcpy(&txdesc->destaddr, &req->coordaddr,
         sizeof(struct ieee802154_addr_s));

  /* Save a reference of the tx descriptor */

  priv->cmd_desc = txdesc;

  /* Search the list of PAN descriptors, that would have been populated by
   * the latest scan procedure. If we have seen a beacon from the
   * coordinator that we are about to associate with, we can check the
   * beacon order to determine whether we can send the command during the
   * CAP.  If we haven't received a beacon frame from the desired
   * coordinator address, we have to just send the frame out immediately.
   */

  for (i = 0; i < priv->npandesc; i++)
    {
      /* Check to make sure the beacon is from the same channel as the
       * request
       */

      if (req->chan != priv->pandescs[i].chan)
        {
          continue;
        }

      if (memcmp(&req->coordaddr, &priv->pandescs[i].coordaddr,
          sizeof(struct ieee802154_addr_s)) == 0)
        {
          wlinfo("Found matching beacon to use for settings\n");

          /* We have a beacon frame from this coordinator, we can set the
           * sfspec and send accordingly.
           */

          /* Copy in the new superframe spec */

          memcpy(&priv->sfspec, &priv->pandescs[i].sfspec,
                 sizeof(struct ieee802154_superframespec_s));

          /* Tell the radio layer about the superframe spec update */

          priv->radio->sfupdate(priv->radio, &priv->pandescs[i].sfspec);
        }
    }

  if (priv->sfspec.beaconorder == 15)
    {
      wlinfo("Transmitting assoc request\n");

      /* Association Request command gets sent out immediately */

      priv->radio->txdelayed(priv->radio, txdesc, 0);
    }
  else
    {
      wlinfo("Queuing assoc request for CAP\n");

      /* Link the transaction into the CSMA transaction list */

      sq_addlast((FAR sq_entry_t *)txdesc, &priv->csma_queue);

      /* Notify the radio driver that there is data available */

      priv->radio->txnotify(priv->radio, false);
    }

  /* We no longer need to have the MAC layer locked. */

  mac802154_unlock(priv)

  return OK;
}

/****************************************************************************
 * Name: mac802154_resp_associate
 *
 * Description:
 *   The MLME-ASSOCIATE.response primitive is used to initiate a response to
 *   an MLME-ASSOCIATE.indication primitive.
 *
 ****************************************************************************/

int mac802154_resp_associate(MACHANDLE mac,
                             FAR struct ieee802154_assoc_resp_s *resp)
{
  FAR struct ieee802154_privmac_s *priv =
    (FAR struct ieee802154_privmac_s *)mac;
  FAR struct ieee802154_txdesc_s *txdesc;
  FAR struct iob_s *iob;
  int ret;

  /* Allocate an IOB to put the frame in */

  iob = iob_alloc(false);
  DEBUGASSERT(iob != NULL);

  iob->io_flink  = NULL;
  iob->io_len    = 0;
  iob->io_offset = 0;
  iob->io_pktlen = 0;

  /* The Destination Addressing Mode and Source Addressing Mode fields shall
   * each be set to indicate extended addressing.
   *
   * The Frame Pending field shall be set to zero and ignored upon reception,
   * and the AR field shall be set to one.
   *
   * The PAN ID Compression field shall be set to one. [1] pg.  69
   */

  iob->io_data[0] = 0;
  iob->io_data[1] = 0;
  IEEE802154_SETACKREQ(iob->io_data, 0);
  IEEE802154_SETPANIDCOMP(iob->io_data, 0);
  IEEE802154_SETFTYPE(iob->io_data, 0, IEEE802154_FRAME_COMMAND);
  IEEE802154_SETDADDRMODE(iob->io_data, 0, IEEE802154_ADDRMODE_EXTENDED);
  IEEE802154_SETSADDRMODE(iob->io_data, 0, IEEE802154_ADDRMODE_EXTENDED);
  iob->io_len = 2;

  /* Each time a data or a MAC command frame is generated, the MAC sublayer
   * shall copy the value of macDSN into the Sequence Number field of the MHR
   * of the outgoing frame and then increment it by one. [1] pg. 40.
   */

  iob->io_data[iob->io_len++] = priv->dsn++;

  /* In accordance with this value of the PAN ID Compression field, the
   * Destination PAN Identifier field shall contain the value of macPANId,
   * while the Source PAN Identifier field shall be omitted. [1] pg. 69
   */

  mac802154_putpanid(iob, priv->addr.panid);

  /* The Destination Address field shall contain the extended address of the
   * device requesting association. [1] pg. 69
   */

  mac802154_puteaddr(iob, resp->devaddr);

  /* The Source Address field shall contain the value of
   * macExtendedAddress.
   */

  mac802154_puteaddr(iob, priv->addr.eaddr);

  /* Copy in the Command Frame Identifier */

  iob->io_data[iob->io_len++] = IEEE802154_CMD_ASSOC_RESP;

  /* Copy in the assigned short address */

  if (resp->status == IEEE802154_STATUS_SUCCESS)
    {
      mac802154_putsaddr(iob, resp->assocsaddr);
    }
  else
    {
      mac802154_putsaddr(iob, &IEEE802154_SADDR_UNSPEC);
    }

  /* Copy in the association status */

  iob->io_data[iob->io_len++] = resp->status;

  /* Get exclusive access to the MAC */

  ret = mac802154_lock(priv, true);
  if (ret < 0)
    {
      iob_free(iob);
      return ret;
    }

  /* Allocate the txdesc, waiting if necessary */

  ret = mac802154_txdesc_alloc(priv, &txdesc, true);
  if (ret < 0)
    {
      iob_free(iob);
      mac802154_unlock(priv)
      return ret;
    }

  txdesc->frame = iob;
  txdesc->frametype = IEEE802154_FRAME_COMMAND;
  txdesc->ackreq = true;

  txdesc->destaddr.mode = IEEE802154_ADDRMODE_EXTENDED;
  IEEE802154_PANIDCOPY(txdesc->destaddr.panid, priv->addr.panid);
  IEEE802154_EADDRCOPY(txdesc->destaddr.eaddr, resp->devaddr);

  mac802154_setupindirect(priv, txdesc);

  mac802154_unlock(priv)

  return OK;
}

/****************************************************************************
 * Internal MAC Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mac802154_txdone_assocreq
 *
 * Description:
 *   Handle the completion (success/failure) of transmitting an association
 *   request command.
 *
 * Assumptions:
 *   Called with the MAC locked.
 *
 ****************************************************************************/

void mac802154_txdone_assocreq(FAR struct ieee802154_privmac_s *priv,
                               FAR struct ieee802154_txdesc_s *txdesc)
{
  enum ieee802154_status_e status;
  FAR struct ieee802154_primitive_s *primitive =
    (FAR struct ieee802154_primitive_s *)txdesc->conf;

  if (txdesc->conf->status != IEEE802154_STATUS_SUCCESS)
    {
      /* if the association request command cannot be sent due to a
       * channel access failure, the MAC sublayer shall notify the next
       * higher layer. [1] pg. 33
       */

      /* We can actually high-jack the data conf notification since it
       * is allocated as an ieee80215_primitive_s anyway. Before we overwrite
       * any data though, we need to get the status from the data
       * confirmation as that is the method we use to get the reason
       * why the tx failed from the radio layer.
       */

      status = txdesc->conf->status;
      primitive->type = IEEE802154_PRIMITIVE_CONF_ASSOC;

      primitive->u.assocconf.status = status;

      /* The short device address allocated by the coordinator on
       * successful association. This parameter will be equal to 0xffff
       * if the association attempt was unsuccessful. [1] pg. 81
       */

      IEEE802154_SADDRCOPY(primitive->u.assocconf.saddr,
                           &IEEE802154_SADDR_UNSPEC);

      /* We are now done the operation, unlock the semaphore */

      priv->curr_op = MAC802154_OP_NONE;
      priv->cmd_desc = NULL;
      mac802154_givesem(&priv->opsem);

      /* Release the MAC, call the callback, get exclusive access again */

      mac802154_notify(priv, primitive);
    }
  else
    {
      /* On receipt of the acknowledgment to the association request
       * command, the device shall wait for at most macResponseWaitTime
       * for the coordinator to make its association decision; the PIB
       * attribute macResponseWaitTime is a network-topology-dependent
       * parameter and may be set to match the specific requirements of
       * the network that a device is trying to join. If the device is
       * tracking the beacon, it shall attempt to extract the association
       * response command from the coordinator whenever it is indicated in
       * the beacon frame. If the device is not tracking the beacon, it
       * shall attempt to extract the association response command from
       * the coordinator after macResponseWaitTime. [1] pg. 34
       */

      if (priv->sfspec.beaconorder < 15)
        {
          /* We are tracking the beacon, so we should see our address in the
           * beacon frame within macResponseWaitTime if the coordinator is
           * going to respond. Setup a timeout for macResponseWaitTime so
           * that we can inform the next highest layer if the association
           * attempt fails due to NO_DATA.
           *
           * TODO: The standard defines macResponseWaitTime as:
           * The maximum time, in multiples of aBaseSuperframeDuration, a
           * device shall wait for a response command frame to be available
           * following a request command frame.
           *
           * However, on beacon-enabled networks, it seems the maximum value
           * isn't really that large of a value, AKA:  assoc always fails
           * from timeout even though everything is working as expected.
           * The definition does say after we've sent a data request, which
           * we, haven't sent yet, but we do need a timeout for association
           * in general.  Not sure what the correct answer is. For now, I am
           * going to change the way macResponseWaitTime is used with beacon-
           * enabled networks and make the timeout (BI * macResponseWaitTime)
           * where BI is Beacon Interval = aBaseSuperframeDuration *
           * 2^macBeaconOrder
           */

          wlinfo("Starting timeout timer\n");
          mac802154_timerstart(priv, (priv->resp_waittime *
            (IEEE802154_BASE_SUPERFRAME_DURATION *
             (1 << priv->sfspec.beaconorder))),
            mac802154_assoctimeout);
        }
      else
        {
          /* Make sure the coordinator address mode is not set to none. This
           * shouldn't happen since the association request should have set
           * the mode to short or extended
           */

          DEBUGASSERT(priv->pandesc.coordaddr.mode !=
                      IEEE802154_ADDRMODE_NONE);

          /* Off-load extracting the Association Response to the work queue
           * to avoid locking up the calling thread.
           */

          DEBUGASSERT(work_available(&priv->macop_work));
          work_queue(LPWORK, &priv->macop_work, mac802154_extract_assocresp,
                     priv, 0);
        }

      /* Deallocate the data conf notification as it is no longer needed. */

      ieee802154_primitive_free(primitive);
    }
}

/****************************************************************************
 * Name: mac802154_txdone_datareq_assoc
 *
 * Description:
 *   Handle the completion (success/failure) of transmitting a data request
 *   command in an effort to extract the association response from the
 *   coordinator.
 *
 * Assumptions:
 *   Called with the MAC locked.
 *
 ****************************************************************************/

void mac802154_txdone_datareq_assoc(FAR struct ieee802154_privmac_s *priv,
                                    FAR struct ieee802154_txdesc_s *txdesc)
{
  enum ieee802154_status_e status;
  FAR struct ieee802154_primitive_s *primitive =
    (FAR struct ieee802154_primitive_s *)txdesc->conf;

  /* If the data request failed to be sent, notify the next layer
   * that the association has failed.
   *            OR
   * On receipt of the Ack frame with the Frame Pending field set
   * to zero, the device shall conclude that there are no data
   * pending at the coordinator. [1] pg. 43
   */

  if (primitive->u.dataconf.status != IEEE802154_STATUS_SUCCESS ||
      txdesc->framepending == 0)
    {
      if (primitive->u.dataconf.status != IEEE802154_STATUS_SUCCESS)
        {
          status = primitive->u.dataconf.status;
        }
      else
        {
          /* If the device does not extract an association response
           * command frame from the coordinator within macResponseWaitTime,
           * the MLME shall issue the MLME-ASSOCIATE.confirm primitive,
           * as described in 6.2.2.4, with a status of NO_DATA, and the
           * association attempt shall be deemed a failure. [1] pg. 34
           */

          status = IEEE802154_STATUS_NO_DATA;
        }

      primitive->type = IEEE802154_PRIMITIVE_CONF_ASSOC;
      primitive->u.assocconf.status = status;

      /* The short device address allocated by the coordinator on
       * successful association. This parameter will be equal to 0xffff
       * if the association attempt was unsuccessful. [1] pg. 81
       */

      IEEE802154_SADDRCOPY(primitive->u.assocconf.saddr,
                           &IEEE802154_SADDR_UNSPEC);

      /* We are now done the operation, and can release the command */

      priv->curr_op = MAC802154_OP_NONE;
      priv->cmd_desc = NULL;
      mac802154_givesem(&priv->opsem);

      mac802154_notify(priv, primitive);
    }
  else
    {
      /* On receipt of the acknowledgment frame with the Frame
       * Pending field set to one, a device shall enable its
       * receiver for at most macMaxFrameTotalWaitTime to receive
       * the corresponding data frame from the coordinator. [1] pg.43
       */

      mac802154_rxenable(priv);

      /* If we are on a beacon-enabled network, we already have the
       * association timeout timer scheduled. So we only need to start the
       * timeout timer if we are operating on a non-beacon enabled network.
       *
       * NOTE: This may create a bad side-effect where the receiver is on
       * for longer than it needs to be during association. Revisit if power
       * is ever an issue.
       */

      if (priv->sfspec.beaconorder == 15)
        {
          /* Start a timer, if we receive the data frame, we will cancel
           * the timer, otherwise it will expire and we will notify the
           * next highest layer of the failure.
           */

          wlinfo("Starting timeout timer\n");
          mac802154_timerstart(priv, priv->max_frame_waittime,
                               mac802154_assoctimeout);
        }

      /* Deallocate the data conf notification as it is no longer needed. */

      ieee802154_primitive_free(primitive);
    }
}

/****************************************************************************
 * Name: mac802154_rx_assocreq
 *
 * Description:
 *   Function called from the generic RX Frame worker to parse and handle the
 *   reception of an Association Request MAC command frame.
 *
 ****************************************************************************/

void mac802154_rx_assocreq(FAR struct ieee802154_privmac_s *priv,
                           FAR struct ieee802154_data_ind_s *ind)
{
  FAR struct iob_s *frame = ind->frame;
  FAR struct ieee802154_primitive_s *primitive;
  uint8_t cap;

  /* Allocate a notification to pass to the next highest layer */

  primitive = ieee802154_primitive_allocate();
  primitive->type = IEEE802154_PRIMITIVE_IND_ASSOC;

  /* Association Requests should always be sent from a device with source
   * addressing mode set to extended mode. Throw out any request received
   * without addressing set to extended
   */

  if (ind->src.mode != IEEE802154_ADDRMODE_EXTENDED)
    {
      return;
    }

  /* Copy the extended address of the requesting device */

  IEEE802154_EADDRCOPY(primitive->u.assocind.devaddr, ind->src.eaddr);

  /* Copy in the capability information from the frame to the notification */

  cap = frame->io_data[frame->io_offset++];
  primitive->u.assocind.capabilities.devtype =
    (cap >> IEEE802154_CAPABILITY_SHIFT_DEVTYPE) & 0x01;
  primitive->u.assocind.capabilities.powersource =
    (cap >> IEEE802154_CAPABILITY_SHIFT_PWRSRC) & 0x01;
  primitive->u.assocind.capabilities.rxonidle =
    (cap >> IEEE802154_CAPABILITY_SHIFT_RXONIDLE) & 0x01;
  primitive->u.assocind.capabilities.security =
    (cap >> IEEE802154_CAPABILITY_SHIFT_SECURITY) & 0x01;
  primitive->u.assocind.capabilities.allocaddr =
    (cap >> IEEE802154_CAPABILITY_SHIFT_ALLOCADDR) & 0x01;

#ifdef CONFIG_IEEE802154_SECURITY
#  error Missing security logic
#endif

  /* Get exclusive access to the MAC */

  mac802154_lock(priv, false);

  /* Notify the next highest layer of the association status */

  mac802154_notify(priv, primitive);
  mac802154_unlock(priv)
}

/****************************************************************************
 * Name: mac802154_rx_assocresp
 *
 * Description:
 *   Function called from the generic RX Frame worker to parse and handle the
 *   reception of an Association Response MAC command frame.
 *
 ****************************************************************************/

void mac802154_rx_assocresp(FAR struct ieee802154_privmac_s *priv,
                            FAR struct ieee802154_data_ind_s *ind)
{
  FAR struct iob_s *iob = ind->frame;
  FAR struct ieee802154_primitive_s *primitive;

  /* Check if we are performing an Association operation, if not, we will
   * just ignore the frame.
   */

  if (priv->curr_op != MAC802154_OP_ASSOC)
    {
      /* This situation can occur in a beacon-enabled network if the
       * association request has timed out, but the Coordinator has already
       * queued the response. Which means the beacon would contain our
       * address, causing us to extract the response.
       *
       * TODO: What is supposed to happen in this situation. Are we supposed
       * to accept the request? Are we supposed to Disassociate with the
       * network as a convenience to the PAN Coordinator. So that it does
       * not need to waste space holding our information?
       */

      wlinfo("Ignoring association response frame\n");

      return;
    }

  /* Cancel the timeout used if we didn't get a response */

  mac802154_timercancel(priv);

  /* Allocate a notification to pass to the next highest layer */

  primitive = ieee802154_primitive_allocate();
  primitive->type = IEEE802154_PRIMITIVE_CONF_ASSOC;

  /* Get exclusive access to the MAC */

  mac802154_lock(priv, false);

  /* Parse the short address from the response */

  mac802154_takesaddr(iob, priv->addr.saddr);

  /* Inform the radio of the address change */

  priv->radio->setattr(priv->radio, IEEE802154_ATTR_MAC_SADDR,
                        (FAR union ieee802154_attr_u *)priv->addr.saddr);

  /* A Short Address field value equal to 0xfffe shall indicate that the
   * device has been successfully associated with a PAN but has not been
   * allocated a short address. In this case, the device shall communicate
   * on the PAN using only its extended address. [1] pg. 70
   */

  if (IEEE802154_SADDRCMP(priv->addr.saddr, &IEEE802154_SADDR_BCAST))
    {
      /* TODO: Figure out if this is sufficient */

      priv->addr.mode = IEEE802154_ADDRMODE_SHORT;
    }

  /* Parse the status from the response */

  primitive->u.assocconf.status = iob->io_data[iob->io_offset++];

  if (primitive->u.assocconf.status == IEEE802154_STATUS_SUCCESS)
    {
      priv->isassoc = true;
    }
  else
    {
      priv->isassoc = false;
    }

  IEEE802154_SADDRCOPY(primitive->u.assocconf.saddr, priv->addr.saddr);

  /* We are no longer performing the association operation */

  priv->curr_op = MAC802154_OP_NONE;
  priv->cmd_desc = NULL;
  mac802154_givesem(&priv->opsem);
  mac802154_rxdisable(priv);

  /* Notify the next highest layer of the association status */

  mac802154_notify(priv, primitive);
  mac802154_unlock(priv)
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mac802154_assoctimeout
 *
 * Description:
 *   Function registered with MAC timer that gets called via the work queue
 *   to handle a timeout for extracting the Association Response from the
 *   Coordinator.
 *
 ****************************************************************************/

static void mac802154_assoctimeout(FAR void *arg)
{
  FAR struct ieee802154_privmac_s *priv =
    (FAR struct ieee802154_privmac_s *)arg;
  FAR struct ieee802154_primitive_s *primitive;

  /* If there is work scheduled for the rxframe_worker, we want to reschedule
   * this work, so that we make sure if the frame we were waiting for was
   * just received, we don't timeout
   */

  if (!work_available(&priv->rx_work))
    {
      work_queue(HPWORK, &priv->timer_work, mac802154_assoctimeout, priv, 0);
      return;
    }

  DEBUGASSERT(priv->curr_op == MAC802154_OP_ASSOC);

  /* If the device does not extract an association response command
   * frame from the coordinator within macResponseWaitTime, the MLME
   * shall issue the MLME-ASSOCIATE.confirm primitive, as described
   * in 6.2.2.4, with a status of NO_DATA, and the association attempt
   * shall be deemed a failure. [1] pg. 33
   */

  /* Allocate a notification struct to pass to the next highest layer.
   * Don't allow EINTR to interrupt.
   */

  primitive = ieee802154_primitive_allocate();
  primitive->type = IEEE802154_PRIMITIVE_CONF_ASSOC;

  primitive->u.assocconf.status = IEEE802154_STATUS_NO_DATA;
  IEEE802154_SADDRCOPY(primitive->u.assocconf.saddr,
                       &IEEE802154_SADDR_UNSPEC);

  /* We are no longer performing the association operation */

  priv->curr_op = MAC802154_OP_NONE;
  priv->cmd_desc = NULL;
  mac802154_givesem(&priv->opsem);
  mac802154_rxdisable(priv);

  mac802154_lock(priv, false);
  mac802154_notify(priv, primitive);
  mac802154_unlock(priv)
}

/****************************************************************************
 * Name: mac802154_extract_assocrespj
 *
 * Description:
 *   Create and send a Data request command to extract the Association
 *   response from the Coordinator.
 *
 * Assumptions:
 *   Called with the MAC unlocked.
 *
 ****************************************************************************/

static void mac802154_extract_assocresp(FAR void *arg)
{
  FAR struct ieee802154_privmac_s *priv =
    (FAR struct ieee802154_privmac_s *)arg;
  FAR struct ieee802154_txdesc_s *respdesc;

  mac802154_lock(priv, false);

  mac802154_txdesc_alloc(priv, &respdesc, false);

  mac802154_createdatareq(priv, &priv->pandesc.coordaddr,
                          IEEE802154_ADDRMODE_EXTENDED, respdesc);

  mac802154_unlock(priv)

  priv->curr_cmd = IEEE802154_CMD_DATA_REQ;

  priv->radio->txdelayed(priv->radio, respdesc,
    (priv->resp_waittime*IEEE802154_BASE_SUPERFRAME_DURATION));
}
