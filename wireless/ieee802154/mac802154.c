/****************************************************************************
 * wireless/ieee802154/mac802154.c
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

#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>

#include <nuttx/mm/iob.h>

#include "mac802154.h"
#include "mac802154_notif.h"
#include "mac802154_internal.h"
#include "mac802154_assoc.h"
#include "mac802154_data.h"
#include "mac802154_poll.h"

#include <nuttx/wireless/ieee802154/ieee802154_mac.h>
#include <nuttx/wireless/ieee802154/ieee802154_radio.h>

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Data structure pools and allocation helpers */

static void mac802154_resetqueues(FAR struct ieee802154_privmac_s *priv);

/* IEEE 802.15.4 PHY Interface OPs */

static int mac802154_radiopoll(FAR const struct ieee802154_radiocb_s *radiocb,
                               bool gts, FAR struct ieee802154_txdesc_s **tx_desc);

static void mac802154_txdone(FAR const struct ieee802154_radiocb_s *radiocb,
                             FAR struct ieee802154_txdesc_s *tx_desc);
static void mac802154_txdone_worker(FAR void *arg);

static void mac802154_rxframe(FAR const struct ieee802154_radiocb_s *radiocb,
                              FAR struct ieee802154_data_ind_s *ind);
static void mac802154_rxframe_worker(FAR void *arg);

static void mac802154_rx_datareq(FAR struct ieee802154_privmac_s *priv,
                                 FAR struct ieee802154_data_ind_s *ind);
static void mac802154_rx_dataframe(FAR struct ieee802154_privmac_s *priv,
                                   FAR struct ieee802154_data_ind_s *ind);

static void mac802154_purge_worker(FAR void *arg);

/* Watchdog Timeout Functions */

static void mac802154_timeout_expiry(int argc, wdparm_t arg, ...);

static uint32_t mac802154_symtoticks(FAR struct ieee802154_privmac_s *priv,
                              uint32_t symbols);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mac802154_resetqueues
 *
 * Description:
 *   Initializes the various queues used in the MAC layer. Called on creation
 *   of MAC.
 *
 ****************************************************************************/

static void mac802154_resetqueues(FAR struct ieee802154_privmac_s *priv)
{
  int i;

  sq_init(&priv->txdone_queue);
  sq_init(&priv->csma_queue);
  sq_init(&priv->gts_queue);
  sq_init(&priv->indirect_queue);
  sq_init(&priv->dataind_queue);

  /* Initialize the tx descriptor allocation pool */

  sq_init(&priv->txdesc_queue);
  for (i = 0; i < CONFIG_MAC802154_NTXDESC; i++)
    {
      sq_addlast((FAR sq_entry_t *)&priv->txdesc_pool[i], &priv->txdesc_queue);
    }
  sem_init(&priv->txdesc_sem, 0, CONFIG_MAC802154_NTXDESC);

  /* Initialize the notifcation allocation pool */

  mac802154_notifpool_init(priv);
}

/****************************************************************************
 * Name: mac802154_txdesc_pool
 *
 * Description:
 *   This function allocates a tx descriptor and the dependent notification (data
 *   confirmation) from the free list. The notification and tx descriptor will
 *   be freed seperately, both by the MAC layer either directly, or through
 *   mac802154_notif_free in the case of the notification.
 *
 * Assumptions:
 *   priv MAC struct is locked when calling.
 *
 * Notes:
 *   If any of the semaphore waits inside this function get interrupted, the
 *   function will release the MAC layer.  If this function returns -EINTR, the
 *   calling code should NOT release the MAC semaphore.
 *
 ****************************************************************************/

int mac802154_txdesc_alloc(FAR struct ieee802154_privmac_s *priv,
                           FAR struct ieee802154_txdesc_s **txdesc,
                           bool allow_interrupt)
{
  int ret;
  FAR struct ieee802154_notif_s *notif;

  /* Try and take a count from the semaphore.  If this succeeds, we have
   * "reserved" the structure, but still need to unlink it from the free list.
   * The MAC is already locked, so there shouldn't be any other conflicting calls
   */

  ret = sem_trywait(&priv->txdesc_sem);

  if (ret == OK)
    {
      *txdesc = (FAR struct ieee802154_txdesc_s *)sq_remfirst(&priv->txdesc_queue);
    }
  else
    {
      /* Unlock MAC so that other work can be done to free a notification */

      mac802154_givesem(&priv->exclsem);

      /* Take a count from the tx desc semaphore, waiting if necessary. We
       * only return from here with an error if we are allowing interruptions
       * and we received a signal */

      ret = mac802154_takesem(&priv->txdesc_sem, allow_interrupt);
      if (ret < 0)
        {
          /* MAC is already released */

          return -EINTR;
        }

      /* If we've taken a count from the semaphore, we have "reserved" the struct
       * but now we need to pop it off of the free list. We need to re-lock the
       * MAC in order to ensure this happens correctly.
       */

      ret = mac802154_takesem(&priv->exclsem, allow_interrupt);
      if (ret < 0)
        {
          mac802154_givesem(&priv->txdesc_sem);
          return -EINTR;
        }

      /* We can now safely unlink the next free structure from the free list */

      *txdesc = (FAR struct ieee802154_txdesc_s *)sq_remfirst(&priv->txdesc_queue);
    }

  /* We have now successfully allocated the tx descriptor.  Now we need to allocate
   * the notification for the data confirmation that gets passed along with the
   * tx descriptor. These are allocated together, but not freed together.
   */

  ret = mac802154_notif_alloc(priv, &notif, allow_interrupt);
  if (ret < 0)
    {
      /* The mac802154_notif_alloc function follows the same rules as this
       * function.  If it returns -EINTR, the MAC layer is already released
       */

      /* We need to free the txdesc */

      mac802154_txdesc_free(priv, *txdesc);
      return -EINTR;
    }

  (*txdesc)->conf = &notif->u.dataconf;

  return OK;
}

/****************************************************************************
 * Name: mac802154_create_datareq
 *
 * Description:
 *    Internal function used by various parts of the MAC layer. This function
 *    allocates an IOB, populates the frame according to input args, and links
 *    the IOB into the provided tx descriptor.
 *
 * Assumptions:
 *    Called with the MAC locked
 *
 ****************************************************************************/

void mac802154_create_datareq(FAR struct ieee802154_privmac_s *priv,
                              FAR struct ieee802154_addr_s *coordaddr,
                              enum ieee802154_addrmode_e srcmode,
                              FAR struct ieee802154_txdesc_s *txdesc)
{
  FAR struct iob_s *iob;
  FAR uint16_t *u16;

  /* The only node allowed to use a source address of none is the PAN Coordinator.
   * PAN coordinators should not be sending data request commans.
   */

  DEBUGASSERT(srcmode != IEEE802154_ADDRMODE_NONE);

  /* Allocate an IOB to put the frame in */

  iob = iob_alloc(false);
  DEBUGASSERT(iob != NULL);

  iob->io_flink  = NULL;
  iob->io_len    = 0;
  iob->io_offset = 0;
  iob->io_pktlen = 0;

  /* Get a uin16_t reference to the first two bytes. ie frame control field */

  u16 = (FAR uint16_t *)&iob->io_data[iob->io_len];
  iob->io_len = 2;

  *u16 = (IEEE802154_FRAME_COMMAND << IEEE802154_FRAMECTRL_SHIFT_FTYPE);
  *u16 |= IEEE802154_FRAMECTRL_ACKREQ;
  *u16 |= (coordaddr->mode << IEEE802154_FRAMECTRL_SHIFT_DADDR);
  *u16 |= (srcmode << IEEE802154_FRAMECTRL_SHIFT_SADDR);

  /* Each time a data or a MAC command frame is generated, the MAC sublayer
   * shall copy the value of macDSN into the Sequence Number field of the
   * MHR of the outgoing frame and then increment it by one. [1] pg. 40.
   */

  iob->io_data[iob->io_len++] = priv->dsn++;

  /* If the destination address is present, copy the PAN ID and one of the
   * addresses, depending on mode, into the MHR.
   */

  if (coordaddr->mode != IEEE802154_ADDRMODE_NONE)
    {
      memcpy(&iob->io_data[iob->io_len], &coordaddr->panid, 2);
      iob->io_len += 2;

      if (coordaddr->mode == IEEE802154_ADDRMODE_SHORT)
        {
          memcpy(&iob->io_data[iob->io_len], &coordaddr->saddr, 2);
          iob->io_len += 2;
        }
      else if (coordaddr->mode == IEEE802154_ADDRMODE_EXTENDED)
        {
          memcpy(&iob->io_data[iob->io_len], &coordaddr->eaddr,
                 IEEE802154_EADDR_LEN);
          iob->io_len += IEEE802154_EADDR_LEN;
        }
    }

  *u16 |= (coordaddr->mode << IEEE802154_FRAMECTRL_SHIFT_DADDR);

  /* If the Destination Addressing Mode field is set to indicate that
   * destination addressing information is not present, the PAN ID Compression
   * field shall be set to zero and the source PAN identifier shall contain the
   * value of macPANId. Otherwise, the PAN ID Compression field shall be set to
   * one. In this case and in accordance with the PAN ID Compression field, the
   * Destination PAN Identifier field shall contain the value of macPANId, while
   * the Source PAN Identifier field shall be omitted. [1] pg. 72
   */

  if (coordaddr->mode  != IEEE802154_ADDRMODE_NONE &&
      coordaddr->panid == priv->addr.panid)
    {
      *u16 |= IEEE802154_FRAMECTRL_PANIDCOMP;
    }
  else
    {
      memcpy(&iob->io_data[iob->io_len], &priv->addr.panid, 2);
      iob->io_len += 2;
    }

  if (srcmode == IEEE802154_ADDRMODE_SHORT)
    {
      memcpy(&iob->io_data[iob->io_len], &priv->addr.saddr, 2);
      iob->io_len += 2;
    }
  else if (srcmode == IEEE802154_ADDRMODE_EXTENDED)
    {
      memcpy(&iob->io_data[iob->io_len], &priv->addr.eaddr, IEEE802154_EADDR_LEN);
      iob->io_len += IEEE802154_EADDR_LEN;
    }

  /* Copy in the Command Frame Identifier */

  iob->io_data[iob->io_len++] = IEEE802154_CMD_DATA_REQ;

  /* Copy the IOB reference to the descriptor */

  txdesc->frame = iob;
  txdesc->frametype = IEEE802154_FRAME_COMMAND;

  /* Save a copy of the destination addressing infromation into the tx descriptor.
   * We only do this for commands to help with handling their progession.
   */

  memcpy(&txdesc->destaddr, &coordaddr, sizeof(struct ieee802154_addr_s));

  /* Copy the IOB reference to the descriptor */

  txdesc->frame = iob;
}

/****************************************************************************
 * Name: mac802154_setupindirect
 *
 * Description:
 *    Internal function used by various parts of the MAC layer. This function
 *    places the provided tx descriptor in the indirect list and manages the
 *    scheduling for purging the transaction if it does not get extracted in
 *    time.
 *
 * Assumptions:
 *    Called with the MAC locked
 *
 ****************************************************************************/

void mac802154_setupindirect(FAR struct ieee802154_privmac_s *priv,
                             FAR struct ieee802154_txdesc_s *txdesc)
{
  uint32_t ticks;
  uint32_t symbols;

  /* Link the tx descriptor into the list */

  sq_addlast((FAR sq_entry_t *)txdesc, &priv->indirect_queue);

  /* Update the timestamp for purging the transaction */

  /* The maximum time (in unit periods) that a transaction is stored by a
   * coordinator and indicated in its beacon. The unit period is governed by
   * macBeaconOrder, BO, as follows: For 0 ≤ BO ≤ 14, the unit period will be
   * aBaseSuperframeDuration × 2 BO . For BO = 15, the unit period will be
   * aBaseSuperframeDuration. [1] pg. 129
   */

  if (priv->beaconorder < 15)
    {
      symbols = priv->trans_persisttime *
        (IEEE802154_BASE_SUPERFRAME_DURATION * (1 << priv->beaconorder));
    }
  else
    {
      symbols = priv->trans_persisttime * IEEE802154_BASE_SUPERFRAME_DURATION;
    }

  ticks = mac802154_symtoticks(priv, symbols);

  txdesc->purge_time = clock_systimer() + ticks;

  /* Check to see if the purge indirect timer is scheduled. If it is, when the
   * timer fires, it will schedule the next purge timer event. Inherently, the
   * queue will be in order of which transaction needs to be purged next.
   *
   * If the purge indirect timer has not been scheduled, schedule it for when
   * this transaction should expire.
   */

  if (work_available(&priv->purge_work))
    {
      //work_queue(MAC802154_WORK, &priv->purge_work, mac802154_purge_worker,
       //          (FAR void *)priv, ticks);
    }
}

/****************************************************************************
 * Name: mac802154_purge_worker
 *
 * Description:
 *   Worker function scheduled in order to purge expired indirect transactions.
 *   The first element in the list should always be removed. The list is searched
 *   and transactions are removed until a transaction has not yet expired.  Then
 *   if there are any remaining transactions, the work function is rescheduled
 *   for the next expiring transaction.
 *
 ****************************************************************************/

static void mac802154_purge_worker(FAR void *arg)
{
  FAR struct ieee802154_privmac_s *priv =
    (FAR struct ieee802154_privmac_s *)arg;
  FAR struct ieee802154_txdesc_s *txdesc;

  /* Get exclusive access to the driver structure.  We don't care about any
   * signals so don't allow interruptions
   */

  mac802154_takesem(&priv->exclsem, false);

  while (1)
  {
    /* Pop transactions off indirect queue until the transaction timeout has not
     * passed.
     */

    txdesc = (FAR struct ieee802154_txdesc_s *)sq_peek(&priv->indirect_queue);

    if (txdesc == NULL)
      {
        break;
      }

  /* Should probably check a little ahead and remove the transaction if it is within
   * a certain number of clock ticks away.  There is no since in scheduling the
   * timer to expire in only a few ticks.
   */

    if (clock_systimer() >= txdesc->purge_time)
      {
        /* Unlink the transaction */

        sq_remfirst(&priv->indirect_queue);

        /* Free the IOB, the notification, and the tx descriptor */

        iob_free(txdesc->frame);
        ((FAR struct mac802154_notif_s *)txdesc->conf)->flink = priv->notif_free;
        priv->notif_free = ((FAR struct mac802154_notif_s *)txdesc->conf);
        mac802154_txdesc_free(priv, txdesc);

        wlinfo("Indirect TX purged");
      }
    else
      {
        /* Reschedule the transaction for the next timeout */

        work_queue(MAC802154_WORK, &priv->purge_work, mac802154_purge_worker,
                   (FAR void *)priv, txdesc->purge_time - clock_systimer());
        break;
      }
  }
}

/****************************************************************************
 * Name: mac802154_radiopoll
 *
 * Description:
 *   Called from the radio driver through the callback struct.  This function is
 *   called when the radio has room for another transaction.  If the MAC
 *   layer has a transaction, it copies it into the supplied buffer and
 *   returns the length.  A descriptor is also populated with the transaction.
 *
 ****************************************************************************/

static int mac802154_radiopoll(FAR const struct ieee802154_radiocb_s *radiocb,
                               bool gts, FAR struct ieee802154_txdesc_s **txdesc)
{
  FAR struct mac802154_radiocb_s *cb =
    (FAR struct mac802154_radiocb_s *)radiocb;
  FAR struct ieee802154_privmac_s *priv;

  DEBUGASSERT(cb != NULL && cb->priv != NULL);
  priv = cb->priv;

  /* Get exclusive access to the driver structure.  Ignore any EINTR signals */

  mac802154_takesem(&priv->exclsem, false);

  if (gts)
    {
      /* Check to see if there are any GTS transactions waiting */

      *txdesc = (FAR struct ieee802154_txdesc_s *)sq_remfirst(&priv->gts_queue);
    }
  else
    {
      /* Check to see if there are any CSMA transactions waiting */

      *txdesc = (FAR struct ieee802154_txdesc_s *)sq_remfirst(&priv->csma_queue);
    }

  mac802154_givesem(&priv->exclsem);

  if (*txdesc != NULL)
    {
      return (*txdesc)->frame->io_len;
    }

  return 0;
}

/****************************************************************************
 * Name: mac802154_txdone
 *
 * Description:
 *   Called from the radio driver through the callback struct.  This function is
 *   called when the radio has completed a transaction.  The txdesc passed gives
 *   provides information about the completed transaction including the original
 *   handle provided when the transaction was created and the status of the
 *   transaction.  This function copies the descriptor and schedules work to
 *   handle the transaction without blocking the radio.
 *
 ****************************************************************************/

static void mac802154_txdone(FAR const struct ieee802154_radiocb_s *radiocb,
                             FAR struct ieee802154_txdesc_s *txdesc)
{
  FAR struct mac802154_radiocb_s *cb =
    (FAR struct mac802154_radiocb_s *)radiocb;
  FAR struct ieee802154_privmac_s *priv;

  DEBUGASSERT(cb != NULL && cb->priv != NULL);
  priv = cb->priv;

  /* Get exclusive access to the driver structure.  We don't care about any
   * signals so don't allow interruptions
   */

  mac802154_takesem(&priv->exclsem, false);

  sq_addlast((FAR sq_entry_t *)txdesc, &priv->txdone_queue);

  mac802154_givesem(&priv->exclsem);

  /* Schedule work with the work queue to process the completion further */

  if (work_available(&priv->tx_work))
    {
      work_queue(MAC802154_WORK, &priv->tx_work, mac802154_txdone_worker,
                 (FAR void *)priv, 0);
    }
}

/****************************************************************************
 * Name: mac802154_txdone_worker
 *
 * Description:
 *   Worker function scheduled from mac802154_txdone.  This function pops any
 *   TX descriptors off of the list and calls the next highest layers callback
 *   to inform the layer of the completed transaction and the status of it.
 *
 ****************************************************************************/

static void mac802154_txdone_worker(FAR void *arg)
{
  FAR struct ieee802154_privmac_s *priv =
    (FAR struct ieee802154_privmac_s *)arg;
  FAR struct ieee802154_txdesc_s *txdesc;
  FAR struct ieee802154_notif_s *notif;
  FAR struct mac802154_notif_s *privnotif;

  /* Get exclusive access to the driver structure.  We don't care about any
   * signals so don't allow interruptions
   */

  mac802154_takesem(&priv->exclsem, false);

  while (1)
    {
      txdesc = (FAR struct ieee802154_txdesc_s *)sq_remfirst(&priv->txdone_queue);

      if (txdesc == NULL)
        {
          break;
        }

      /* Cast the data_conf to a notification. We get both the private and public
       * notification structure to make it easier to use.
       */

      privnotif = (FAR struct mac802154_notif_s *)txdesc->conf;
      notif = &privnotif->pub;

      switch(txdesc->frametype)
        {
          case IEEE802154_FRAME_DATA:
            {
              notif->notiftype = IEEE802154_NOTIFY_CONF_DATA;

              /* Release the MAC, call the callback, get exclusive access again */

              mac802154_givesem(&priv->exclsem);
              mac802154_notify(priv, notif);
              mac802154_takesem(&priv->exclsem, false);
            }
            break;

          case IEEE802154_FRAME_COMMAND:
            {
              switch (priv->curr_cmd)
                {
                  case IEEE802154_CMD_ASSOC_REQ:
                    mac802154_txdone_assocreq(priv, txdesc);
                    break;

                  case IEEE802154_CMD_ASSOC_RESP:
                    break;

                  case IEEE802154_CMD_DISASSOC_NOT:
                    break;

                  case IEEE802154_CMD_DATA_REQ:
                    /* Data requests can be sent for 3 different reasons.
                     *
                     *   1. On a beacon-enabled PAN, this command shall be sent
                     *      by a device when macAutoRequest is equal to TRUE and
                     *      a beacon frame indicating that data are pending for
                     *      that device is received from its coordinator.
                     *   2. when instructed to do so by the next higher layer on
                     *      reception of the MLME-POLL.request primitive.
                     *   3. a device may send this command to the coordinator
                     *      macResponseWaitTime after the acknowledgment to an
                     *      association request command.
                     */

                    switch (priv->curr_op)
                      {
                        case MAC802154_OP_ASSOC:
                          mac802154_txdone_datareq_assoc(priv, txdesc);
                          break;

                        case MAC802154_OP_POLL:
                          mac802154_txdone_datareq_poll(priv, txdesc);
                          break;

                        default:
                          break;
                      }
                    break;

                  case IEEE802154_CMD_PANID_CONF_NOT:
                    break;

                  case IEEE802154_CMD_ORPHAN_NOT:
                    break;

                  case IEEE802154_CMD_BEACON_REQ:
                    break;

                  case IEEE802154_CMD_COORD_REALIGN:
                    break;

                  case IEEE802154_CMD_GTS_REQ:
                    break;

                  default:
                    /* We can deallocate the data conf notification as it is no
                     * longer needed. We can't use the public function here
                     * since we already have the MAC locked.
                     */

                    privnotif->flink = priv->notif_free;
                    priv->notif_free = privnotif;
                    priv->nnotif     = 0;
                    break;
                }
            }
            break;

          default:
            {
              /* We can deallocate the data conf notification as it is no longer
               * needed. We can't use the public function here since we already
               * have the MAC locked.
               */

              privnotif->flink = priv->notif_free;
              priv->notif_free = privnotif;
            }
            break;
        }

      /* Free the IOB and the tx descriptor */

      iob_free(txdesc->frame);
      mac802154_txdesc_free(priv, txdesc);
    }

  mac802154_givesem(&priv->exclsem);
}

/****************************************************************************
 * Name: mac802154_rxframe
 *
 * Description:
 *   Called from the radio driver through the callback struct.  This function is
 *   called when the radio has received a frame. The frame is passed in an iob,
 *   so that we can free it when we are done processing.  A pointer to the RX
 *   descriptor is passed along with the iob, but it must be copied here as it
 *   is allocated directly on the caller's stack.  We simply link the frame,
 *   copy the RX descriptor, and schedule a worker to process the frame later so
 *   that we do not hold up the radio.
 *
 ****************************************************************************/

static void mac802154_rxframe(FAR const struct ieee802154_radiocb_s *radiocb,
                              FAR struct ieee802154_data_ind_s *ind)
{
  FAR struct mac802154_radiocb_s *cb =
    (FAR struct mac802154_radiocb_s *)radiocb;
  FAR struct ieee802154_privmac_s *priv;

  DEBUGASSERT(cb != NULL && cb->priv != NULL);
  priv = cb->priv;

  /* Get exclusive access to the driver structure.  We don't care about any
   * signals so if we see one, just go back to trying to get access again.
   */

  mac802154_takesem(&priv->exclsem, false);

  /* Push the iob onto the tail of the frame list for processing */

  sq_addlast((FAR sq_entry_t *)ind, &priv->dataind_queue);

  wlinfo("frame received\n");

  mac802154_givesem(&priv->exclsem);

  /* Schedule work with the work queue to process the completion further */

  if (work_available(&priv->rx_work))
    {
      work_queue(MAC802154_WORK, &priv->rx_work, mac802154_rxframe_worker,
                 (FAR void *)priv, 0);
    }
}

/****************************************************************************
 * Name: mac802154_rxframe_worker
 *
 * Description:
 *   Worker function scheduled from mac802154_rxframe.  This function processes
 *   any frames in the list.  Frames intended to be consumed by the MAC layer
 *   will not produce any callbacks to the next highest layer.  Frames intended
 *   for the application layer will be forwarded to them.
 *
 ****************************************************************************/

static void mac802154_rxframe_worker(FAR void *arg)
{
  FAR struct ieee802154_privmac_s *priv =
    (FAR struct ieee802154_privmac_s *)arg;
  FAR struct ieee802154_data_ind_s *ind;
  FAR struct iob_s *frame;
  uint16_t *frame_ctrl;
  bool panid_comp;
  uint8_t ftype;

  while(1)
    {
      /* Get exclusive access to the driver structure.  We don't care about any
       * signals so if we see one, just go back to trying to get access again.
       */

      mac802154_takesem(&priv->exclsem, false);

      /* Pop the iob from the head of the frame list for processing */

      ind = (FAR struct ieee802154_data_ind_s *)sq_remfirst(&priv->dataind_queue);

      /* Once we pop off the indication, we don't need to keep the mac locked */

      mac802154_givesem(&priv->exclsem);

      if (ind == NULL)
        {
          return;
        }

      /* Get a local copy of the frame to make it easier to access */

      frame = ind->frame;

      /* Set a local pointer to the frame control then move the offset past
       * the frame control field
       */

      frame_ctrl = (uint16_t *)&frame->io_data[frame->io_offset];
      frame->io_offset += 2;

      /* We use the data_ind_s as a container for the frame information even if
       * this isn't a data frame
       */

      ind->src.mode = (*frame_ctrl & IEEE802154_FRAMECTRL_SADDR) >>
                      IEEE802154_FRAMECTRL_SHIFT_SADDR;

      ind->dest.mode = (*frame_ctrl & IEEE802154_FRAMECTRL_DADDR) >>
                       IEEE802154_FRAMECTRL_SHIFT_DADDR;

      panid_comp = (*frame_ctrl & IEEE802154_FRAMECTRL_PANIDCOMP) >>
                   IEEE802154_FRAMECTRL_SHIFT_PANIDCOMP;

      ind->dsn = frame->io_data[frame->io_offset++];

      /* If the destination address is included */

      if (ind->dest.mode != IEEE802154_ADDRMODE_NONE)
        {
          /* Get the destination PAN ID */

          memcpy(&ind->dest.panid, &frame->io_data[frame->io_offset], 2);
          frame->io_offset += 2;

          if (ind->dest.mode == IEEE802154_ADDRMODE_SHORT)
            {
              memcpy(&ind->dest.saddr, &frame->io_data[frame->io_offset], 2);
              frame->io_offset += 2;
            }
          else if (ind->dest.mode == IEEE802154_ADDRMODE_EXTENDED)
            {
              memcpy(&ind->dest.eaddr[0], &frame->io_data[frame->io_offset],
                     IEEE802154_EADDR_LEN);
              frame->io_offset += IEEE802154_EADDR_LEN;
            }
        }

      if (ind->src.mode != IEEE802154_ADDRMODE_NONE)
        {
          /* If the source address is included, and the PAN ID compression field
           * is set, get the PAN ID from the header.
           */

          if (panid_comp)
            {
              /* The source PAN ID is equal to the destination PAN ID */

              ind->src.panid = ind->dest.panid;
            }
          else
            {
              memcpy(&ind->src.panid, &frame->io_data[frame->io_offset], 2);
              frame->io_offset += 2;
            }

          if (ind->src.mode == IEEE802154_ADDRMODE_SHORT)
            {
              memcpy(&ind->src.saddr, &frame->io_data[frame->io_offset], 2);
              frame->io_offset += 2;
            }
          else if (ind->src.mode == IEEE802154_ADDRMODE_EXTENDED)
            {
              memcpy(&ind->src.eaddr[0], &frame->io_data[frame->io_offset],
                     IEEE802154_EADDR_LEN);
              frame->io_offset += 8;
            }
        }

      ftype = (*frame_ctrl & IEEE802154_FRAMECTRL_FTYPE) >>
              IEEE802154_FRAMECTRL_SHIFT_FTYPE;

      switch (ftype)
        {
          case IEEE802154_FRAME_DATA:
            {
              mac802154_rx_dataframe(priv, ind);
            }
            break;

          case IEEE802154_FRAME_COMMAND:
            {
              /* Get the command type.  The command type is always the first
               * field after the MHR. Consu;me the byte by increasing offset so that
               * subsequent functions can start from the byte after the command ID.
               */

              uint8_t cmdtype = frame->io_data[frame->io_offset++];

              switch (cmdtype)
                {
                  case IEEE802154_CMD_ASSOC_REQ:
                    mac802154_rx_assocreq(priv, ind);
                    break;

                  case IEEE802154_CMD_ASSOC_RESP:
                    mac802154_rx_assocresp(priv, ind);
                    break;

                  case IEEE802154_CMD_DISASSOC_NOT:
                    break;

                  case IEEE802154_CMD_DATA_REQ:
                    mac802154_rx_datareq(priv, ind);
                    break;

                  case IEEE802154_CMD_PANID_CONF_NOT:
                    break;

                  case IEEE802154_CMD_ORPHAN_NOT:
                    break;

                  case IEEE802154_CMD_BEACON_REQ:
                    break;

                  case IEEE802154_CMD_COORD_REALIGN:
                    break;

                  case IEEE802154_CMD_GTS_REQ:
                    break;
                }

              /* Free the data indication struct from the pool */

              ieee802154_ind_free(ind);
            }
            break;

          case IEEE802154_FRAME_BEACON:
            {
              /* TODO: Add logic here to handle extracting association response from
               * coordinator if beacon tracking was enabled during the Association
               * operation.
               *
               *  mac802154_txdesc_alloc(priv, &respdec, false);
               *  mac802154_create_datareq(priv, &req->coordaddr,
               *                           IEEE802154_ADDRMODE_EXTENDED, respdesc);
               *  sq_addlast((FAR sq_entry_t *)respdesc, &priv->csma_queue);
               */
            }
            break;

          case IEEE802154_FRAME_ACK:
            {
              /* The radio layer is responsible for handling all ACKs and retries.
               * If for some reason an ACK gets here, just throw it out.
               */

              wlinfo("ACK received\n");
              ieee802154_ind_free(ind);
            }
            break;
        }
    }
}

/****************************************************************************
 * Name: mac802154_rx_dataframe
 *
 * Description:
 *   Function called from the generic RX Frame worker to parse and handle the
 *   reception of a data frame.
 *
 ****************************************************************************/

static void mac802154_rx_dataframe(FAR struct ieee802154_privmac_s *priv,
                                   FAR struct ieee802154_data_ind_s *ind)
{
  FAR struct ieee802154_notif_s *notif;

  /* Get exclusive access to the MAC */

  mac802154_takesem(&priv->exclsem, false);

  /* If we are currently performing a POLL operation and we've
    * received a data response, use the addressing information
    * to determine if it is extracted data. If the addressing info
    * matches, notify the next highest layer using POLL.confirm
    * primitive.  If the addressing information does not match,
    * handle the transaction like any other data transaction.
    *
    * Note: We can't receive frames without addressing information
    * unless we are the PAN coordinator. And in that situation, we
    * wouldn't be performing a POLL operation. Meaning:
    *
    * If the current operation is POLL, we aren't the PAN coordinator
    * so the incoming frame CAN'T
    *
    * FIXME: Fix documentation
    */

  if (priv->curr_op == MAC802154_OP_POLL || priv->curr_op == MAC802154_OP_ASSOC)
    {
      /* If we are in promiscuous mode, we need to check if the
       * frame is even for us first. If the address is not ours,
       * then handle the frame like a normal transaction.
       */

      if (priv->promisc)
        {
          if (ind->dest.panid != priv->addr.panid)
            {
              goto notify_with_lock;
            }

          if (ind->dest.mode == IEEE802154_ADDRMODE_SHORT &&
              ind->dest.saddr != priv->addr.saddr)
            {
              goto notify_with_lock;
            }
          else if (ind->dest.mode == IEEE802154_ADDRMODE_EXTENDED &&
                   (memcmp(&ind->dest.eaddr[0], &priv->addr.eaddr[0],
                          IEEE802154_EADDR_LEN) != 0))
            {
              goto notify_with_lock;
            }
          else
            {
              goto notify_with_lock;
            }
        }

      /* If this was our extracted data, the source addressing field can only
        * be NONE if we are trying to extract data from the PAN coordinator.
        * A PAN coordinator shouldn't be sending us a frame if it wasn't
        * our extracted data. Therefore just assume if the address mode is set
        * to NONE, we process it as our extracted frame
        */

      if (ind->src.mode != priv->cmd_desc->destaddr.mode)
        {
          goto notify_with_lock;
        }

      if (ind->src.mode == IEEE802154_ADDRMODE_SHORT &&
          ind->src.saddr != priv->cmd_desc->destaddr.saddr)
        {
          goto notify_with_lock;
        }
      else if (ind->src.mode == IEEE802154_ADDRMODE_EXTENDED &&
                (memcmp(&ind->src.eaddr[0], &priv->cmd_desc->destaddr.eaddr[0],
                        IEEE802154_EADDR_LEN) != 0))
        {
          goto notify_with_lock;
        }

      /* If we've gotten this far, the frame is our extracted data. Cancel the
       * timeout */

      mac802154_timercancel(priv);

      /* If a frame is received from the coordinator with a zero length payload
       * or if the frame is a MAC command frame, the MLME will issue the
       * MLME-POLL.confirm primitive with a status of NO_DATA. [1] pg. 111
       */

      mac802154_notif_alloc(priv, &notif, false);

      if (priv->curr_op == MAC802154_OP_POLL)
        {
          notif->notiftype = IEEE802154_NOTIFY_CONF_POLL;

          if (ind->frame->io_offset == ind->frame->io_len)
            {
              ieee802154_ind_free(ind);
              notif->u.pollconf.status = IEEE802154_STATUS_NO_DATA;
            }
          else
            {
              notif->u.pollconf.status = IEEE802154_STATUS_SUCCESS;
            }
        }
      else if (priv->curr_op == MAC802154_OP_ASSOC)
        {
          /* If we ever receive a data frame back as a response to the
           * association request, we assume it means there wasn't any data.
           */

          notif->notiftype = IEEE802154_NOTIFY_CONF_ASSOC;
          notif->u.assocconf.status = IEEE802154_STATUS_NO_DATA;
        }

      /* We are no longer performing the association operation */

      priv->curr_op = MAC802154_OP_NONE;
      priv->cmd_desc = NULL;
      mac802154_givesem(&priv->op_sem);

      /* Release the MAC */

      mac802154_givesem(&priv->exclsem);
      mac802154_notify(priv, notif);

      /* If there was data, pass it along */

      if (ind->frame->io_len > ind->frame->io_offset)
        {
          goto notify_without_lock;
        }
    }
  else
    {
      FAR struct mac802154_maccb_s *cb;

notify_with_lock:

      mac802154_givesem(&priv->exclsem);

notify_without_lock:

      /* If there are registered MCPS callback receivers registered,
       * then forward the frame in priority order.  If there are no
       * registered receivers or if none of the receivers accept the
       * data frame then drop the frame.
        */

      for (cb = priv->cb; cb != NULL; cb = cb->flink)
        {
          int ret;

          /* Does this MAC client want frames? */

          if (cb->rxframe != NULL)
            {
              /* Yes.. Offer this frame to the receiver */

              ret = cb->rxframe(cb, ind);
              if (ret >= 0)
                {
                  /* The receiver accepted and disposed of the frame and
                   * its metadata.  We are done.
                   */

                  return;
                }
            }
        }

      /* We get here if the there are no registered receivers or if
       * all of the registered receivers declined the frame.
       * Free the data indication struct from the pool
       */

      ieee802154_ind_free(ind);
    }
}

/****************************************************************************
 * Name: mac802154_rx_datareq
 *
 * Description:
 *   Function called from the generic RX Frame worker to parse and handle the
 *   reception of an Data Request MAC command frame.
 *
 ****************************************************************************/

static void mac802154_rx_datareq(FAR struct ieee802154_privmac_s *priv,
                                 FAR struct ieee802154_data_ind_s *ind)
{
  FAR struct ieee802154_txdesc_s *txdesc;
  FAR struct iob_s *iob;
  uint16_t *frame_ctrl;

  /* Get exclusive access to the MAC */

  mac802154_takesem(&priv->exclsem, false);

  /* Search the list of indirect transactions to see if there are any waiting
   * for the requesting device.
   */

  /* TODO: I believe there is an issue here. If there is for some reason a
   * outgoing data frame to a device who is currently requesting association,
   * we will send the data frame as a response to an association request. We
   * need to check for this condition.
   */

  txdesc = (FAR struct ieee802154_txdesc_s *)sq_peek(&priv->indirect_queue);

  if (txdesc == NULL)
    {
      goto no_data;
    }

  do
    {
      if (txdesc->destaddr.mode == ind->src.mode)
        {
          if (txdesc->destaddr.mode == IEEE802154_ADDRMODE_SHORT)
            {
              if (txdesc->destaddr.saddr == ind->src.saddr)
                {
                  /* Remove the transaction from the queue */

                  sq_rem((FAR sq_entry_t *)txdesc, &priv->indirect_queue);

                  /* The addresses match, send the transaction immediately */

                  priv->radio->txdelayed(priv->radio, txdesc, 0);
                  break;
                }
            }
          else if (txdesc->destaddr.mode == IEEE802154_ADDRMODE_EXTENDED)
            {
              if (memcmp(&txdesc->destaddr.eaddr[0], &ind->src.eaddr[0],
                         sizeof(IEEE802154_EADDR_LEN)) == 0)
                {
                  /* Remove the transaction from the queue */

                  sq_rem((FAR sq_entry_t *)txdesc, &priv->indirect_queue);

                  /* The addresses match, send the transaction immediately */

                  priv->radio->txdelayed(priv->radio, txdesc, 0);
                  break;
                }
            }
          else
            {
              DEBUGASSERT(false);
            }
        }

      txdesc = (FAR struct ieee802154_txdesc_s *)sq_next((FAR sq_entry_t *)txdesc);

      if (txdesc == NULL)
        {
          goto no_data;
        }
    }
  while (1);

  mac802154_givesem(&priv->exclsem);
  return;

no_data:

  /* If there is no data frame pending for the requesting device, the coordinator
   * shall send a data frame without requesting acknowledgment to the device
   * containing a zero length payload, indicating that no data are present, using
   * one of the mechanisms described in this subclause. [1] pg. 43
   */

  /* Allocate an IOB to put the frame in */

  iob = iob_alloc(false);
  DEBUGASSERT(iob != NULL);

  iob->io_flink  = NULL;
  iob->io_len    = 0;
  iob->io_offset = 0;
  iob->io_pktlen = 0;

  iob->io_len += 2;

  /* Cast the first two bytes of the IOB to a uint16_t frame control field */

  frame_ctrl = (FAR uint16_t *)&iob->io_data[0];

  /* Ensure we start with a clear frame control field */

  *frame_ctrl = 0;

  /* Set the frame type to Data */

  *frame_ctrl |= IEEE802154_FRAME_DATA << IEEE802154_FRAMECTRL_SHIFT_FTYPE;

  /* Each time a data or a MAC command frame is generated, the MAC sublayer
   * shall copy the value of macDSN into the Sequence Number field of the MHR
   * of the outgoing frame and then increment it by one. [1] pg. 40.
   */

  iob->io_data[iob->io_len++] = priv->dsn++;

  /* Use the source address information from the received data request to
   * respond.
   */

  *((uint16_t *)&iob->io_data[iob->io_len]) = ind->src.panid;
  iob->io_len += 2;

  if (ind->src.mode == IEEE802154_ADDRMODE_SHORT)
    {
       memcpy(&iob->io_data[iob->io_len], &ind->src.saddr, 2);
       iob->io_len += 2;
    }
  else if (ind->src.mode == IEEE802154_ADDRMODE_EXTENDED)
    {
      memcpy(&iob->io_data[iob->io_len], &ind->src.eaddr, IEEE802154_EADDR_LEN);
      iob->io_len += IEEE802154_EADDR_LEN;
    }
  else
    {
      DEBUGASSERT(false);
    }

  /* Set the destination addr mode inside the frame control field */

  *frame_ctrl |= (ind->src.mode << IEEE802154_FRAMECTRL_SHIFT_DADDR);

  /* Check if the source PAN ID of the incoming request is the same as ours. */

  if (ind->src.panid == priv->addr.panid)
    {
      *frame_ctrl |= IEEE802154_FRAMECTRL_PANIDCOMP;
    }
  else
    {
      /* Copy in our PAN ID */

      memcpy(&iob->io_data[iob->io_len], &priv->addr.panid, 2);
      iob->io_len += 2;
    }

  /* Copy in our address using the mode that the device used to address us */

  if (ind->dest.mode == IEEE802154_ADDRMODE_SHORT)
    {
      memcpy(&iob->io_data[iob->io_len], &priv->addr.saddr, 2);
      iob->io_len += 2;

      *frame_ctrl |= (IEEE802154_ADDRMODE_SHORT << IEEE802154_FRAMECTRL_SHIFT_SADDR);
    }
  else
    {
      memcpy(&iob->io_data[iob->io_len], &ind->dest.eaddr, IEEE802154_EADDR_LEN);
      iob->io_len += IEEE802154_EADDR_LEN;
      *frame_ctrl |= (IEEE802154_ADDRMODE_EXTENDED << IEEE802154_FRAMECTRL_SHIFT_SADDR);
    }

  /* Allocate the txdesc, waiting if necessary, allow interruptions */

  mac802154_txdesc_alloc(priv, &txdesc, false);

  txdesc->frame = iob;
  txdesc->frametype = IEEE802154_FRAME_DATA;

  mac802154_givesem(&priv->exclsem);

  priv->radio->txdelayed(priv->radio, txdesc, 0);
}

/****************************************************************************
 * Name: mac802154_symtoticks
 *
 * Description:
 *   Helper function for converting symbols to system clock ticks
 *
 * Assumptions:
 *   priv MAC struct is locked when calling.
 *
 ****************************************************************************/

static uint32_t mac802154_symtoticks(FAR struct ieee802154_privmac_s *priv,
                                     uint32_t symbols)
{
  union ieee802154_attr_u attrval;
  uint32_t ret;

  /* First, get the symbol duration from the radio layer.  Symbol duration is
   * returned in picoseconds to ensure precision is kept when multiplying to
   * get overall times.
   */

  priv->radio->get_attr(priv->radio, IEEE802154_ATTR_PHY_SYMBOL_DURATION,
                        &attrval);

  /* After this step, ret represents microseconds */

  ret = ((uint64_t)attrval.phy.symdur_picosec * symbols) / (1000 * 1000);

  /* This method should only be used for things that can be late. For instance,
   * it's always okay to wait a little longer before disabling your receiver.
   * Therefore, we force the tick count to round up.
   */

  if (ret % USEC_PER_TICK == 0)
    {
      ret = ret/USEC_PER_TICK;
    }
  else
    {
      ret = ret/USEC_PER_TICK;
      ret++;
    }

  return ret;
}

/****************************************************************************
 * Name: mac802154_timerstart
 *
 * Description:
 *   Helper function wrapping the watchdog timer interface. Helps isolate
 *   different operations from having to worry about work queues and watchdog
 *   timers.
 *
 * Assumptions:
 *   priv MAC struct is locked when calling.
 *
 ****************************************************************************/

int mac802154_timerstart(FAR struct ieee802154_privmac_s *priv,
                         uint32_t numsymbols, mac802154_worker_t worker)
{
  /* TODO: Add check to make sure timer is not already being used.  I'd like to
   * design this so that it absolutely never happens */

  /* Convert the number of symbols to the number of CPU ticks */

  uint32_t ticks = mac802154_symtoticks(priv, numsymbols);

  /* Save the function pointer to call if the timeout expires */

  priv->timeout_worker = worker;

  /* Start the watchdog */

  wd_start(priv->timeout, (int32_t)ticks, mac802154_timeout_expiry,
           1, (wdparm_t)priv);

  return OK;
}

/****************************************************************************
 * Function: mac802154_timeout_expiry
 *
 * Description:
 *   The watchdog timed out.  Called from the timer interrupt handler.
 *
 * Parameters:
 *   argc - The number of available arguments
 *   arg  - The first argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by the watchdog logic.
 *
 ****************************************************************************/

static void mac802154_timeout_expiry(int argc, wdparm_t arg, ...)
{
  FAR struct ieee802154_privmac_s *priv = (FAR struct ieee802154_privmac_s *)arg;

  /* There should never be a case where the timeout is used twice at the same
   * time. */

  DEBUGASSERT(work_available(&priv->timeout_work));

  /* Check to make sure the function pointer is still valid */

  DEBUGASSERT(priv->timeout_worker != NULL);

  work_queue(MAC802154_WORK, &priv->timeout_work, (worker_t)priv->timeout_worker,
             priv, 0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************

 * Name: mac802154_create
 *
 * Description:
 *   Create a 802.15.4 MAC device from a 802.15.4 compatible radio device.
 *
 *   The returned MAC structure should be passed to either the next highest
 *   layer in the network stack, or registered with a mac802154dev character
 *   or network drivers.  In any of these scenarios, the next highest layer
 *   should  register a set of callbacks with the MAC layer by setting the
 *   mac->cbs member.
 *
 *   NOTE: This API does not create any device accessible to userspace. If
 *   you want to call these APIs from userspace, you have to wrap your mac
 *   in a character device via mac802154_device.c.
 *
 * Input Parameters:
 *   radiodev - an instance of an IEEE 802.15.4 radio
 *
 * Returned Value:
 *   An opaque reference to the MAC state data.
 *
 ****************************************************************************/

MACHANDLE mac802154_create(FAR struct ieee802154_radio_s *radiodev)
{
  FAR struct ieee802154_privmac_s *mac;
  FAR struct ieee802154_radiocb_s *radiocb;
  uint8_t eaddr[IEEE802154_EADDR_LEN];
  int i;

  /* Allocate object */

  mac = (FAR struct ieee802154_privmac_s *)
    kmm_zalloc(sizeof(struct ieee802154_privmac_s));

  if (mac == NULL)
    {
      return NULL;
    }

  /* Allow exclusive access to the privmac struct */

  sem_init(&mac->exclsem, 0, 1);

  /* Allow exclusive access to the dedicated command transaction */

  sem_init(&mac->op_sem, 0, 1);

  /* Setup watchdog for extraction timeout */

  mac->timeout = wd_create();

  /* Initialize fields */

  mac->radio = radiodev;

  mac802154_req_reset((MACHANDLE)mac, true);

  /* Initialize the Radio callbacks */

  mac->radiocb.priv = mac;

  radiocb            = &mac->radiocb.cb;
  radiocb->poll      = mac802154_radiopoll;
  radiocb->txdone    = mac802154_txdone;
  radiocb->rxframe   = mac802154_rxframe;

  /* Bind our callback structure */

  radiodev->bind(radiodev, &mac->radiocb.cb);

  /* Initialize our various data pools */

  ieee802154_indpool_initialize();
  mac802154_resetqueues(mac);

  /* Set the default extended address */

  for (i = 0; i < IEEE802154_EADDR_LEN; i++)
    {
      eaddr[i] = (CONFIG_IEEE802154_DEFAULT_EADDR >> (8 * i)) & 0xFF;
    }

  memcpy(&mac->addr.eaddr, &eaddr[0], IEEE802154_EADDR_LEN);
  mac->radio->set_attr(mac->radio, IEEE802154_ATTR_MAC_EXTENDED_ADDR,
                      (union ieee802154_attr_u *)&eaddr[0]);

  return (MACHANDLE)mac;
}
