/****************************************************************************
 * wireless/ieee802154/mac802154_poll.c
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
#include <nuttx/wireless/ieee802154/ieee802154_radio.h>

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void mac802154_polltimeout(FAR void *arg);

/****************************************************************************
 * Public MAC Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mac802154_req_poll
 *
 * Description:
 *   The MLME-POLL.request primitive prompts the device to request data from
 *   the coordinator. Confirmation is returned via the
 *   struct mac802154_maccb_s->conf_poll callback, followed by a
 *   struct mac802154_maccb_s->ind_data callback.
 *
 ****************************************************************************/

int mac802154_req_poll(MACHANDLE mac, FAR struct ieee802154_poll_req_s *req)
{
  FAR struct ieee802154_privmac_s *priv =
    (FAR struct ieee802154_privmac_s *)mac;
  FAR struct ieee802154_txdesc_s *txdesc;
  int ret;

  /* On receipt of the MLME-POLL.request primitive, the MLME requests data from
   * the coordinator, as described in 5.1.6.3. If the poll is directed to the
   * PAN coordinator, the data request command may be generated without any
   * destination address information present. Otherwise, the data request
   * command is always generated with the destination address information in the
   * CoordPANId and CoordAddress parameters.
   */

  /* Get exlusive access to the operation sempaphore. This must happen before
   * getting exclusive access to the MAC struct or else there could be a lockup
   * condition. This would occur if another thread is using the cmdtrans but
   * needs access to the MAC in order to unlock it.
   */

  ret = mac802154_takesem(&priv->opsem, true);
  if (ret < 0)
    {
      return ret;
    }

  /* Get exclusive access to the MAC */

   ret = mac802154_lock(priv, true);
   if (ret < 0)
     {
       mac802154_givesem(&priv->opsem);
       return ret;
     }

  priv->curr_op = MAC802154_OP_POLL;
  priv->curr_cmd = IEEE802154_CMD_DATA_REQ;

  /* Allocate the txdesc, waiting if necessary */

  ret = mac802154_txdesc_alloc(priv, &txdesc, true);
  if (ret < 0)
    {
      mac802154_unlock(priv)
      mac802154_givesem(&priv->opsem);
      return ret;
    }

  /* The Source Addressing Mode field shall be set according to the value of
   * macShortAddress. If macShortAddress is less than 0xfffe, short addressing
   * shall be used. Extended addressing shall be used otherwise.
   */

  if (priv->addr.saddr[0] >= 0xfe && priv->addr.saddr[1] == 0xff)
    {
      mac802154_createdatareq(priv, &req->coordaddr, IEEE802154_ADDRMODE_EXTENDED,
                              txdesc);
    }
  else
    {
      mac802154_createdatareq(priv, &req->coordaddr, IEEE802154_ADDRMODE_SHORT,
                              txdesc);
    }

  /* Save a copy of the destination addressing infromation into the tx descriptor.
   * We only do this for commands to help with handling their progession.
   */

  memcpy(&txdesc->destaddr, &req->coordaddr, sizeof(struct ieee802154_addr_s));

  /* Save a reference of the tx descriptor */

  priv->cmd_desc = txdesc;

  wlinfo("Queuing POLL.request in CSMA queue\n");

  /* Link the transaction into the CSMA transaction list */

  sq_addlast((FAR sq_entry_t *)txdesc, &priv->csma_queue);

  /* We no longer need to have the MAC layer locked. */

  mac802154_unlock(priv)

  /* Notify the radio driver that there is data available */

  priv->radio->txnotify(priv->radio, false);

  return OK;
}

/****************************************************************************
 * Internal MAC Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mac802154_txdone_datareq_poll
 *
 * Description:
 *   Handle the completion (success/failure) of transmitting a data request
 *   command in an effort to extract data from the coordinator triggered by
 *   a POLL.request from the next highest layer
 *
 * Assumptions:
 *   Called with the MAC locked.
 *
 ****************************************************************************/

void mac802154_txdone_datareq_poll(FAR struct ieee802154_privmac_s *priv,
                                   FAR struct ieee802154_txdesc_s *txdesc)
{
  FAR struct ieee802154_primitive_s * primitive =
    (FAR struct ieee802154_primitive_s *)txdesc->conf;
  enum ieee802154_status_e status;

  /* If the data request failed to be sent, notify the next layer
   * that the poll has failed.
   *            OR
   * On receipt of the Ack frame with the Frame Pending field set
   * to zero, the device shall conclude that there are no data
   * pending at the coordinator. [1] pg. 43
   */

  if (txdesc->conf->status != IEEE802154_STATUS_SUCCESS ||
      txdesc->framepending == 0)
    {
      if (txdesc->conf->status != IEEE802154_STATUS_SUCCESS)
        {
          status = txdesc->conf->status;
        }
      else
        {
          status = IEEE802154_STATUS_NO_DATA;
        }

      primitive->type = IEEE802154_PRIMITIVE_CONF_POLL;
      txdesc->conf->status = status;

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

      /* Start a timer, if we receive the data frame, we will cancel
       * the timer, otherwise it will expire and we will notify the
       * next highest layer of the failure.
       */

      mac802154_timerstart(priv, priv->max_frame_waittime,
                           mac802154_polltimeout);

      /* Deallocate the data conf primitive as it is no longer needed. */

      ieee802154_primitive_free(primitive);
    }
}

/****************************************************************************
 * Name: mac802154_polltimeout
 *
 * Description:
 *   Function registered with MAC timer that gets called via the work queue to
 *   handle a timeout for extracting a response from the Coordinator.
 *
 ****************************************************************************/

void mac802154_polltimeout(FAR void *arg)
{
  FAR struct ieee802154_privmac_s *priv = (FAR struct ieee802154_privmac_s *)arg;
  FAR struct ieee802154_primitive_s *primitive;

  /* If there is work scheduled for the rxframe_worker, we want to reschedule
   * this work, so that we make sure if the frame we were waiting for was just
   * received, we don't timeout
   */

  if (!work_available(&priv->rx_work))
    {
      work_queue(HPWORK, &priv->timer_work, mac802154_polltimeout, priv, 0);
      return;
    }

  DEBUGASSERT(priv->curr_op == MAC802154_OP_POLL);

  primitive = ieee802154_primitive_allocate();
  primitive->type = IEEE802154_PRIMITIVE_CONF_POLL;
  primitive->u.pollconf.status = IEEE802154_STATUS_NO_DATA;

  mac802154_lock(priv, false);

  /* We are no longer performing the association operation */
  priv->curr_op = MAC802154_OP_NONE;
  priv->cmd_desc = NULL;
  mac802154_givesem(&priv->opsem);

  mac802154_notify(priv, primitive);
  mac802154_unlock(priv);
}