/****************************************************************************
 * wireless/ieee802154/mac802154_assoc.c
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

#include <nuttx/wqueue.h>
#include <nuttx/mm/iob.h>

#include "mac802154.h"
#include "mac802154_internal.h"
#include "mac802154_assoc.h"

#include <nuttx/wireless/ieee802154/ieee802154_mac.h>

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void mac802154_timeout_assoc(FAR struct ieee802154_privmac_s *priv);
static FAR struct ieee802154_txdesc_s *
  mac802154_assoc_getresp(FAR struct ieee802154_privmac_s *priv);

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
  FAR uint16_t *u16;
  bool rxonidle;
  int ret;

  /* Get exlusive access to the operation sempaphore. This must happen before
   * getting exclusive access to the MAC struct or else there could be a lockup
   * condition. This would occur if another thread is using the cmdtrans but
   * needs access to the MAC in order to unlock it.
   */

  ret = mac802154_takesem(&priv->op_sem, true);
  if (ret < 0)
    {
      return ret;
    }

  priv->curr_op = MAC802154_OP_ASSOC;
  priv->curr_cmd = IEEE802154_CMD_ASSOC_REQ;

  /* Get exclusive access to the MAC */
   
   ret = mac802154_takesem(&priv->exclsem, true);
   if (ret < 0)
     {
       mac802154_givesem(&priv->op_sem);
       return ret;
     }

  /* Set the channel and channel page of the PHY layer */

  priv->radio->set_attr(priv->radio, IEEE802154_ATTR_PHY_CURRENT_CHANNEL,
                        (FAR const union ieee802154_attr_u *)&req->chnum);

  priv->radio->set_attr(priv->radio, IEEE802154_ATTR_PHY_CURRENT_PAGE,
                        (FAR const union ieee802154_attr_u *)&req->chpage);

  /* Set the PANID attribute */

  priv->addr.panid = req->coordaddr.panid;
  priv->coordaddr.panid = req->coordaddr.panid;
  priv->radio->set_attr(priv->radio, IEEE802154_ATTR_MAC_PANID,
               (FAR const union ieee802154_attr_u *)&req->coordaddr.panid);

  /* Set the coordinator address attributes */

  priv->coordaddr.mode = req->coordaddr.mode;

  if (priv->coordaddr.mode == IEEE802154_ADDRMODE_SHORT)
    {
      priv->coordaddr.saddr = req->coordaddr.saddr;
      memcpy(&priv->coordaddr.eaddr[0], IEEE802154_EADDR_UNSPEC,
             IEEE802154_EADDR_LEN);
    }
  else if (priv->coordaddr.mode == IEEE802154_ADDRMODE_EXTENDED)
    {
      priv->coordaddr.saddr = IEEE802154_SADDR_UNSPEC;
      memcpy(&priv->coordaddr.eaddr[0], &req->coordaddr.eaddr[0],
             IEEE802154_EADDR_LEN);
    }
  else
  {
    ret = -EINVAL;
    goto errout;
  }

  /* Copy in the capabilities information bitfield */

  priv->devmode = (req->capabilities.devtype) ? 
                  IEEE802154_DEVMODE_COORD : IEEE802154_DEVMODE_ENDPOINT;

  /* Unlike other attributes, we can't simply cast this one since it is a bit
   * in a bitfield.  Casting it will give us unpredicatble results.  Instead
   * of creating a ieee802154_attr_u, we use a local bool.  Allocating the
   * ieee802154_attr_u value would take up more room on the stack since it is
   * as large as the largest attribute type.
   */

  rxonidle = req->capabilities.rxonidle;
  priv->radio->set_attr(priv->radio, IEEE802154_ATTR_MAC_RX_ON_WHEN_IDLE,
                        (FAR const union ieee802154_attr_u *)&rxonidle); 
                          
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
      mac802154_givesem(&priv->exclsem);
      mac802154_givesem(&priv->op_sem);
      return ret;
    }

  /* Get a uin16_t reference to the first two bytes. ie frame control field */

  u16 = (FAR uint16_t *)&iob->io_data[0];

  *u16 = (IEEE802154_FRAME_COMMAND << IEEE802154_FRAMECTRL_SHIFT_FTYPE);
  *u16 |= IEEE802154_FRAMECTRL_ACKREQ;
  *u16 |= (priv->coordaddr.mode << IEEE802154_FRAMECTRL_SHIFT_DADDR);
  *u16 |= (IEEE802154_ADDRMODE_EXTENDED << IEEE802154_FRAMECTRL_SHIFT_SADDR);

  iob->io_len = 2;

  /* Each time a data or a MAC command frame is generated, the MAC sublayer
   * shall copy the value of macDSN into the Sequence Number field of the MHR
   * of the outgoing frame and then increment it by one. [1] pg. 40.
   */

  iob->io_data[iob->io_len++] = priv->dsn++;

  /* The Destination PAN Identifier field shall contain the identifier of the
   * PAN to which to associate. [1] pg. 68
   */

  memcpy(&iob->io_data[iob->io_len], &priv->coordaddr.panid, 2);
  iob->io_len += 2;

  /* The Destination Address field shall contain the address from the beacon
   * frame that was transmitted by the coordinator to which the association
   * request command is being sent. [1] pg. 68
   */

  if (priv->coordaddr.mode == IEEE802154_ADDRMODE_SHORT)
    {
      memcpy(&iob->io_data[iob->io_len], &priv->coordaddr.saddr, 2);
      iob->io_len += 2;
    }
  else if (priv->coordaddr.mode == IEEE802154_ADDRMODE_EXTENDED)
    {
      memcpy(&iob->io_data[iob->io_len], &priv->coordaddr.eaddr[0],
             IEEE802154_EADDR_LEN);
      iob->io_len += IEEE802154_EADDR_LEN;
    }
  
  /* The Source PAN Identifier field shall contain the broadcast PAN identifier.*/ 

  u16 = (uint16_t *)&iob->io_data[iob->io_len];
  *u16 = IEEE802154_SADDR_BCAST;
  iob->io_len += 2;

  /* The Source Address field shall contain the value of macExtendedAddress. */

  memcpy(&iob->io_data[iob->io_len], &priv->addr.eaddr[0],
        IEEE802154_EADDR_LEN);
  iob->io_len += IEEE802154_EADDR_LEN;

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

  /* Save a copy of the destination addressing infromation into the tx descriptor.
   * We only do this for commands to help with handling their progession.
   */

  memcpy(&txdesc->destaddr, &req->coordaddr, sizeof(struct ieee802154_addr_s));
  
  /* Save a reference of the tx descriptor */

  priv->cmd_desc = txdesc;

  /* We no longer need to have the MAC layer locked. */

  mac802154_givesem(&priv->exclsem);

  /* Association Request commands get sent out immediately */

  priv->radio->txdelayed(priv->radio, txdesc, 0);

  return OK;

errout:
  mac802154_givesem(&priv->exclsem);
  return ret;
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
  FAR uint16_t *u16;
  int ret;
  
  /* Allocate an IOB to put the frame in */

  iob = iob_alloc(false);
  DEBUGASSERT(iob != NULL);

  iob->io_flink  = NULL;
  iob->io_len    = 0;
  iob->io_offset = 0;
  iob->io_pktlen = 0;

  /* Get a uin16_t reference to the first two bytes. ie frame control field */

  u16 = (FAR uint16_t *)&iob->io_data[0];

  /* The Destination Addressing Mode and Source Addressing Mode fields shall
   * each be set to indicate extended addressing.
   *
   * The Frame Pending field shall be set to zero and ignored upon reception,
   * and the AR field shall be set to one.
   *
   * The PAN ID Compression field shall be set to one. [1] pg.  69
   */

  *u16 = (IEEE802154_FRAME_COMMAND << IEEE802154_FRAMECTRL_SHIFT_FTYPE);
  *u16 |= IEEE802154_FRAMECTRL_ACKREQ;
  *u16 |= IEEE802154_FRAMECTRL_PANIDCOMP;
  *u16 |= (IEEE802154_ADDRMODE_EXTENDED << IEEE802154_FRAMECTRL_SHIFT_DADDR);
  *u16 |= (IEEE802154_ADDRMODE_EXTENDED << IEEE802154_FRAMECTRL_SHIFT_SADDR);

  iob->io_len = 2;

  /* Each time a data or a MAC command frame is generated, the MAC sublayer
   * shall copy the value of macDSN into the Sequence Number field of the MHR
   * of the outgoing frame and then increment it by one. [1] pg. 40.
   */

  iob->io_data[iob->io_len++] = priv->dsn++;

  /* In accordance with this value of the PAN ID Compression field, the
   * Destination PAN Identifier field shall contain the value of macPANId, while
   * the Source PAN Identifier field shall be omitted. [1] pg. 69
   */ 

  memcpy(&iob->io_data[iob->io_len], &priv->addr.panid, 2);
  iob->io_len += 2;

  /* The Destination Address field shall contain the extended address of the
   * device requesting association. [1] pg. 69 */
  
  memcpy(&iob->io_data[iob->io_len], &resp->devaddr[0], IEEE802154_EADDR_LEN);
  iob->io_len += IEEE802154_EADDR_LEN;

  /* The Source Address field shall contain the value of macExtendedAddress. */

  memcpy(&iob->io_data[iob->io_len], &priv->addr.eaddr[0], IEEE802154_EADDR_LEN);
  iob->io_len += IEEE802154_EADDR_LEN;

   /* Copy in the Command Frame Identifier */

  iob->io_data[iob->io_len++] = IEEE802154_CMD_ASSOC_RESP;

  /* Copy in the assigned short address */

  memcpy(&iob->io_data[iob->io_len], &resp->assocsaddr, 2);
  iob->io_len += 2;

  /* Copy in the association status */

  iob->io_data[iob->io_len++] = resp->status;

  /* Get exclusive access to the MAC */
   
   ret = mac802154_takesem(&priv->exclsem, true);
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
      mac802154_givesem(&priv->exclsem);
      return ret;
    }

  txdesc->frame = iob;
  txdesc->frametype = IEEE802154_FRAME_COMMAND;

  txdesc->destaddr.panid = priv->addr.panid;
  txdesc->destaddr.mode = IEEE802154_ADDRMODE_EXTENDED;
  memcpy(&txdesc->destaddr.eaddr[0], &resp->devaddr[0], IEEE802154_EADDR_LEN);

  mac802154_setupindirect(priv, txdesc);

  mac802154_givesem(&priv->exclsem);

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
  FAR struct mac802154_notif_s *privnotif =
    (FAR struct mac802154_notif_s *)txdesc->conf;
  FAR struct ieee802154_notif_s *notif = &privnotif->pub; 
  FAR struct ieee802154_txdesc_s *respdesc;

  if(txdesc->conf->status != IEEE802154_STATUS_SUCCESS)
    {
      /* if the association request command cannot be sent due to a
       * channel access failure, the MAC sublayer shall notify the next
       * higher layer. [1] pg. 33
       */
      
      /* We can actually high-jack the data conf notification since it
       * is allocated as an ieee80215_notif_s anyway. Before we overwrite
       * any data though, we need to get the status from the data
       * confirmation as that is the method we use to get the reason
       * why the tx failed from the radio layer.
       */

      status = txdesc->conf->status;
      notif->notiftype = IEEE802154_NOTIFY_CONF_ASSOC;

      notif->u.assocconf.status = status;

      /* The short device address allocated by the coordinator on
       * successful association. This parameter will be equal to 0xffff
       * if the association attempt was unsuccessful. [1] pg. 81
       */

      notif->u.assocconf.saddr = IEEE802154_SADDR_UNSPEC;

      /* We are now done the operation, unlock the semaphore */

      priv->curr_op = MAC802154_OP_NONE;
      priv->cmd_desc = NULL;
      mac802154_givesem(&priv->op_sem);

      /* Release the MAC, call the callback, get exclusive access again */

      mac802154_givesem(&priv->exclsem);
      priv->cb->notify(priv->cb, notif);
      mac802154_takesem(&priv->exclsem, false);
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

      if (priv->trackingbeacon)
        {
          /* We are tracking the beacon, so we should see our address in the
           * beacon frame within macResponseWaitTime if the coordinator is going
           * to respond. Setup a timeout for macResponseWaitTime so that we
           * can inform the next highest layer if the association attempt fails
           * due to NO_DATA.
           */
          
          mac802154_timerstart(priv,
            priv->resp_waittime*IEEE802154_BASE_SUPERFRAME_DURATION,
            mac802154_timeout_assoc);
        }
      else
        {
          /* Send the Data Request MAC command after macResponseWaitTime to
           * extract the data from the coordinator.
           */

          respdesc = mac802154_assoc_getresp(priv);
          priv->radio->txdelayed(priv->radio, respdesc,
            (priv->resp_waittime*IEEE802154_BASE_SUPERFRAME_DURATION));
        }

      /* We can deallocate the data conf notification as it is no longer
       * needed. We can't use the public function here since we already
       * have the MAC locked. 
       */ 
      
      privnotif->flink = priv->notif_free;
      priv->notif_free = privnotif;
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
  FAR struct mac802154_notif_s *privnotif =
    (FAR struct mac802154_notif_s *)txdesc->conf;
  FAR struct ieee802154_notif_s *notif = &privnotif->pub; 

  /* If the data request failed to be sent, notify the next layer
   * that the association has failed.
   *            OR
   * On receipt of the Ack frame with the Frame Pending field set
   * to zero, the device shall conclude that there are no data
   * pending at the coordinator. [1] pg. 43
   */

  if (notif->u.dataconf.status != IEEE802154_STATUS_SUCCESS ||
      txdesc->framepending == 0)
    {
      if (notif->u.dataconf.status != IEEE802154_STATUS_SUCCESS)
        {
          status = notif->u.dataconf.status;
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

      notif->notiftype = IEEE802154_NOTIFY_CONF_ASSOC;
      notif->u.assocconf.status = status;

      /* The short device address allocated by the coordinator on
       * successful association. This parameter will be equal to 0xffff
       * if the association attempt was unsuccessful. [1] pg. 81
       */

      notif->u.assocconf.saddr = IEEE802154_SADDR_UNSPEC;

      /* We are now done the operation, and can release the command */

      priv->curr_op = MAC802154_OP_NONE;
      priv->cmd_desc = NULL;
      mac802154_givesem(&priv->op_sem);

      /* Release the MAC, call the callback, get exclusive access again */

      mac802154_givesem(&priv->exclsem);
      priv->cb->notify(priv->cb, notif);
      mac802154_takesem(&priv->exclsem, false);
    }
  else
    {
      /* On receipt of the acknowledgment frame with the Frame
       * Pending field set to one, a device shall enable its
       * receiver for at most macMaxFrameTotalWaitTime to receive
       * the corresponding data frame from the coordinator. [1] pg.43
       */

      priv->radio->rxenable(priv->radio, true);

      /* Start a timer, if we receive the data frame, we will cancel
       * the timer, otherwise it will expire and we will notify the
       * next highest layer of the failure.
       */
       
      mac802154_timerstart(priv, priv->max_frame_waittime,
                           mac802154_timeout_assoc);

      /* We can deallocate the data conf notification as it is no longer
       * needed. We can't use the public function here since we already
       * have the MAC locked. 
       */ 

      privnotif->flink = priv->notif_free;
      priv->notif_free = privnotif;
      mac802154_givesem(&priv->notif_sem);
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
  FAR struct ieee802154_notif_s *notif;
  uint8_t cap;

  /* Get exclusive access to the MAC */

  mac802154_takesem(&priv->exclsem, false);

  /* Allocate a notification to pass to the next highest layer */

  mac802154_notif_alloc(priv, &notif, false);
  notif->notiftype = IEEE802154_NOTIFY_IND_ASSOC;

  /* Association Requests should always be sent from a device with source
   * addressing mode set to extended mode. Throw out any request received
   * without addressing set to extended
   */
  
  if (ind->src.mode != IEEE802154_ADDRMODE_EXTENDED)
    {
      goto errout_with_sem;
    }

  /* Copy the extended address of the requesting device */

  memcpy(&notif->u.assocind.devaddr[0], &ind->src.eaddr[0],
         sizeof(struct ieee802154_addr_s));
        
  /* Copy in the capability information from the frame to the notification */

  cap = frame->io_data[frame->io_offset++];
  notif->u.assocind.capabilities.devtype =
    (cap >> IEEE802154_CAPABILITY_SHIFT_DEVTYPE) & 0x01;
  notif->u.assocind.capabilities.powersource =
    (cap >> IEEE802154_CAPABILITY_SHIFT_PWRSRC) & 0x01;
  notif->u.assocind.capabilities.rxonidle =
    (cap >> IEEE802154_CAPABILITY_SHIFT_RXONIDLE) & 0x01;
  notif->u.assocind.capabilities.security =
    (cap >> IEEE802154_CAPABILITY_SHIFT_SECURITY) & 0x01;
  notif->u.assocind.capabilities.allocaddr =
    (cap >> IEEE802154_CAPABILITY_SHIFT_ALLOCADDR) & 0x01;

#ifdef CONFIG_IEEE802154_SECURITY
#error Missing security logic
#endif

  /* Unlock the MAC */

  mac802154_givesem(&priv->exclsem);

  /* Notify the next highest layer of the association status */

  priv->cb->notify(priv->cb, notif);

  return;

errout_with_sem:
  mac802154_givesem(&priv->exclsem);
  return;
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
  FAR struct iob_s *frame = ind->frame;
  FAR struct ieee802154_notif_s *notif;

  /* Check if we are performing an Association operation, if not, we will just
   * ignore the frame.
   */

  if (priv->curr_op != MAC802154_OP_ASSOC)
    {
      return;
    }
  
  /* Cancel the timeout used if we didn't get a response */

  mac802154_timercancel(priv);

  /* Get exclusive access to the MAC */

  mac802154_takesem(&priv->exclsem, false);

  /* Allocate a notification to pass to the next highest layer */

  mac802154_notif_alloc(priv, &notif, false);
  notif->notiftype = IEEE802154_NOTIFY_CONF_ASSOC;

  /* Parse the short address from the response */

  priv->addr.saddr = (uint16_t)(frame->io_data[frame->io_offset]);
  frame->io_offset += 2;

  /* Inform the radio of the address change */

  priv->radio->set_attr(priv->radio, IEEE802154_ATTR_MAC_SHORT_ADDRESS,
                        (FAR union ieee802154_attr_u *)&priv->addr.saddr);
  
  /* A Short Address field value equal to 0xfffe shall indicate that the device
   * has been successfully associated with a PAN but has not been allocated a
   * short address. In this case, the device shall communicate on the PAN using
   * only its extended address. [1] pg. 70
   */

  if (priv->addr.saddr == IEEE802154_SADDR_BCAST)
    {
      /* TODO: Figure out if this is sufficient */

      priv->addr.mode = IEEE802154_ADDRMODE_SHORT;
    }
  
  /* Parse the status from the response */

  notif->u.assocconf.status = frame->io_data[frame->io_offset++];

  if (notif->u.assocconf.status == IEEE802154_STATUS_SUCCESS)
    {
      priv->isassoc = true;
    }
  else
    {
      priv->isassoc = false;
    }

  notif->u.assocconf.saddr = priv->addr.saddr;

  /* Unlock the MAC */

  mac802154_givesem(&priv->exclsem);

  /* We are no longer performing the association operation */

  priv->curr_op = MAC802154_OP_NONE;
  priv->cmd_desc = NULL;
  mac802154_givesem(&priv->op_sem);

  /* Notify the next highest layer of the association status */

  priv->cb->notify(priv->cb, notif);
}

/****************************************************************************
 * Private Function
 ****************************************************************************/

/****************************************************************************
 * Name: mac802154_timeout_assoc
 *
 * Description:
 *   Function registered with MAC timer that gets called via the work queue to
 *   handle a timeout for extracting the Association Response from the Coordinator.
 *
 ****************************************************************************/

static void mac802154_timeout_assoc(FAR struct ieee802154_privmac_s *priv)
{
  FAR struct ieee802154_notif_s *notif;

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

  mac802154_takesem(&priv->exclsem, false);
  mac802154_notif_alloc(priv, &notif, false);

  /* We are no longer performing the association operation */
  priv->curr_op = MAC802154_OP_NONE;
      priv->cmd_desc = NULL;
  mac802154_givesem(&priv->op_sem);

  /* Release the MAC */

  mac802154_givesem(&priv->exclsem);

  notif->notiftype = IEEE802154_NOTIFY_CONF_ASSOC;
  notif->u.assocconf.status = IEEE802154_STATUS_NO_DATA;
  notif->u.assocconf.saddr = IEEE802154_SADDR_UNSPEC;
          
  priv->cb->notify(priv->cb, notif);
}

/****************************************************************************
 * Name: mac802154_assoc_getresp
 *
 * Description:
 *   Send a data request to the coordinator to extract the association response.
 *
 * Assumptions:
 *   MAC is locked when called. 
 *
 * TODO: Can this be used for general data extraction?
 *
 ****************************************************************************/

static FAR struct ieee802154_txdesc_s *
  mac802154_assoc_getresp(FAR struct ieee802154_privmac_s *priv)
{
  FAR struct iob_s *iob;
  FAR struct ieee802154_txdesc_s *txdesc;
  FAR uint16_t *u16;

  /* Allocate an IOB to put the frame in */
  
  iob = iob_alloc(false);
  DEBUGASSERT(iob != NULL);
  
  iob->io_flink  = NULL;
  iob->io_len    = 0;
  iob->io_offset = 0;
  iob->io_pktlen = 0;

  /* Allocate a tx descriptor */  

  mac802154_txdesc_alloc(priv, &txdesc, false);

  priv->curr_cmd = IEEE802154_CMD_DATA_REQ;

  /* Get a uin16_t reference to the first two bytes. ie frame control field */
  
  u16 = (FAR uint16_t *)&iob->io_data[0];
  
  *u16 = (IEEE802154_FRAME_COMMAND << IEEE802154_FRAMECTRL_SHIFT_FTYPE);
  *u16 |= IEEE802154_FRAMECTRL_ACKREQ;
  *u16 |= (priv->coordaddr.mode << IEEE802154_FRAMECTRL_SHIFT_DADDR);
  *u16 |= (IEEE802154_ADDRMODE_EXTENDED << IEEE802154_FRAMECTRL_SHIFT_SADDR);
  
  /* If the Destination Addressing Mode field is set to indicate that
   * destination addressing information is not present, the PAN ID Compression
   * field shall be set to zero and the source PAN identifier shall contain the
   * value of macPANId. Otherwise, the PAN ID Compression field shall be set to
   * one. In this case and in accordance with the PAN ID Compression field, the
   * Destination PAN Identifier field shall contain the value of macPANId, while
   * the Source PAN Identifier field shall be omitted. [1] pg. 72
   *
   * The destination address for a data request to extract an assoication request
   * should never be set to none.  So we always set the PAN ID compression field
   */
  
  DEBUGASSERT(priv->coordaddr.mode != IEEE802154_ADDRMODE_NONE);
  
  *u16 |= IEEE802154_FRAMECTRL_PANIDCOMP;
  
  iob->io_len = 2;
  
  /* Each time a data or a MAC command frame is generated, the MAC sublayer
   * shall copy the value of macDSN into the Sequence Number field of the
   * MHR of the outgoing frame and then increment it by one. [1] pg. 40.
   */
  
  iob->io_data[iob->io_len++] = priv->dsn++;
  
  /* The Destination PAN Identifier field shall contain the identifier of
   * the PAN to which to associate. [1] pg. 68
   */
  
  memcpy(&iob->io_data[iob->io_len], &priv->coordaddr.panid, 2);
  iob->io_len += 2;
  
  /* The Destination Address field shall contain the address from the
   * beacon frame that was transmitted by the coordinator to which the
   * association request command is being sent. [1] pg. 68
   */
  
  if (priv->coordaddr.mode == IEEE802154_ADDRMODE_SHORT)
    {
      memcpy(&iob->io_data[iob->io_len], &priv->coordaddr.saddr, 2);
      iob->io_len += 2;
    }
  else if (priv->coordaddr.mode == IEEE802154_ADDRMODE_EXTENDED)
    {
      memcpy(&iob->io_data[iob->io_len], &priv->coordaddr.eaddr[0],
             IEEE802154_EADDR_LEN);
      iob->io_len += IEEE802154_EADDR_LEN;
    }
  
  /* The Source Address field shall contain the value of macExtendedAddress. */
  
  memcpy(&iob->io_data[iob->io_len], &priv->addr.eaddr[0],
        IEEE802154_EADDR_LEN);
  iob->io_len += IEEE802154_EADDR_LEN;

  /* Copy in the Command Frame Identifier */

  iob->io_data[iob->io_len++] = IEEE802154_CMD_DATA_REQ;

  /* Copy the IOB reference to the descriptor */

  txdesc->frame = iob;

  return txdesc;
}
