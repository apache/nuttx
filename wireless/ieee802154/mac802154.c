/****************************************************************************
 * wireless/ieee802154/mac802154.c
 *
 *   Copyright (C) 2016 Sebastien Lorquet. All rights reserved.
 *   Author: Sebastien Lorquet <sebastien@lorquet.fr>
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

#include <nuttx/kmalloc.h>
#include <nuttx/wireless/ieee802154/ieee802154_radio.h>
#include <nuttx/wireless/ieee802154/ieee802154_mac.h>

#include "mac802154.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The privmac structure holds the internal state of the MAC and is the
 * underlying represention of the opaque MACHANDLE.  It contains storage for
 * the IEEE802.15.4 MIB attributes.
 */

struct ieee802154_privmac_s
{
  FAR struct ieee802154_radio_s *radio;     /* Contained IEEE802.15.4 radio dev */
  FAR const struct ieee802154_maccb_s *cb;  /* Contained MAC callbacks */
  FAR struct ieee802154_phyif_s phyif;      /* Interface to bind to radio */

  sem_t excl_sem; /* Support exclusive access */

  /* Support a singly linked list of transactions that will be sent using the
   * CSMA algorithm.  On a non-beacon enabled PAN, these transactions will be
   * sent whenever. On a beacon-enabled PAN, these transactions will be sent
   * during the CAP of the Coordinator's superframe. */

  FAR struct mac802154_trans_s *csma_head;
  FAR struct mac802154_trans_s *csma_tail;

  struct mac802154_trans_s csma_buf[5];

  /* Support a singly linked list of transactions that will be sent indirectly.
   * This list should only be used by a MAC acting as a coordinator.  These
   * transactions will stay here until the data is extracted by the destination
   * device sending a Data Request MAC command or if too much time passes. This
   * list should also be used to populate the address list of the outgoing
   * beacon frame */
   
  FAR struct mac802154_trans_s *indirect_head;
  FAR struct mac802154_trans_s *indirect_tail;

  FAR struct mac802154_trans_s *active_trans;

  /* MAC PIB attributes, grouped to save memory */

  /* Holds all address information (Extended, Short, and PAN ID) for the MAC */

  struct ieee802154_addr_s addr;

  /* Holds all address information (Extended, Short) for Coordinator */

  struct ieee802154_addr_s coord_addr;

  /* The maximum number of symbols to wait for an acknowledgement frame to
   * arrive following a transmitted data frame. [1] pg. 126
   *
   * NOTE: This may be able to be a 16-bit or even an 8-bit number.  I wasn't
   * sure at the time what the range of reasonable values was */
   
  uint32_t ack_wait_dur;

  /* The maximum time to wait either for a frame intended as a response to a
   * data request frame or for a broadcast frame following a beacon with the
   * Frame Pending field set to one. [1] pg. 127
   *
   * NOTE: This may be able to be a 16-bit or even an 8-bit number.  I wasn't
   * sure at the time what the range of reasonable values was */

  uint32_t max_frame_wait_time;

  /* The maximum time (in unit periods) that a transaction is stored by a
   * coordinator and indicated in its beacon. */

  uint16_t trans_persist_time;

  /* Contents of beacon payload */ 

  uint8_t  beacon_payload[IEEE802154_MAX_BEACON_PAYLOAD_LENGTH];
  uint8_t  beacon_payload_len;    /* Length of beacon payload */ 

  uint8_t batt_life_ext_periods;  /* # of backoff periods during which rx is
                                   * enabled after the IFS following beacon */

  uint8_t bsn;          /* Seq. num added to tx beacon frame */
  uint8_t dsn;          /* Seq. num added to tx data or MAC frame */
  uint8_t max_retries;  /* Max # of retries alloed after tx failure */

  /* The maximum time, in multiples of aBaseSuperframeDuration, a device shall
   * wait for a response command frame to be available following a request 
   * command frame. [1] 128 */

  uint8_t resp_wait_time;

  /* The total transmit duration (including PHY header and FCS) specified in
   * symbols. [1] pg. 129 */

  uint32_t tx_total_dur;

  struct
  { 
    uint32_t is_assoc           : 1;  /* Are we associated to the PAN */
    uint32_t assoc_permit       : 1;  /* Are we allowing assoc. as a coord. */
    uint32_t auto_req           : 1;  /* Automatically send data req. if addr 
                                       * addr is in the beacon frame */

    uint32_t batt_life_ext      : 1;  /* Is BLE enabled */
    uint32_t gts_permit         : 1;  /* Is PAN Coord. accepting GTS reqs. */
    uint32_t promiscuous_mode   : 1;  /* Is promiscuous mode on? */
    uint32_t ranging_supported  : 1;  /* Does MAC sublayer support ranging */
    uint32_t rx_when_idle       : 1;  /* Recvr. on during idle periods */
    uint32_t sec_enabled        : 1;  /* Does MAC sublayer have security en. */

    uint32_t max_csma_backoffs  : 3;  /* Max num backoffs for CSMA algorithm
                                       * before declaring ch access failure */

    uint32_t beacon_order       : 4;  /* Freq. that beacon is transmitted */

    uint32_t superframe_order   : 4;  /* Length of active portion of outgoing
                                       * superframe, including the beacon */

    /* The offset, measured is symbols, between the symbol boundary at which the 
     * MLME captures the timestamp of each transmitted and received frame, and
     * the onset of the first symbol past the SFD, namely the first symbol of
     * the frames [1] pg. 129 */
     
    uint32_t sync_symb_offset   : 12;
  }

  struct
  {
    uint32_t beacon_tx_time     : 24; /* Time of last beacon transmit */
    uint32_t min_be             : 4;  /* Min value of backoff exponent (BE) */
    uint32_t max_be             : 4;  /* Max value of backoff exponent (BE) */
  }

  struct
  {
    uint32_t tx_ctrl_active_dur : 17; /* Duration for which tx is permitted to 
                                       * be active */
                                      
    uint32_t tx_ctrl_pause_dur  : 1;  /* Duration after tx before another tx is
                                       * permitted. 0=2000, 1= 10000 */

    uint32_t timestamp_support  : 1;  /* Does MAC layer supports timestamping */
  }

  /* TODO: Add Security-related MAC PIB attributes */

};

struct mac802154_trans_s
{
  /* Supports a singly linked list */

  FAR struct mac802154_trans_s *flink;

  uint8_t msdu_handle;

  uint8_t *mhr_buf;
  uint8_t mhr_len;

  uint8_t *d_buf;
  uint8_t d_len;

  sem_t sem;
};

struct mac802154_unsec_mhr_s
{
  uint8_t length;
  union {
    uint16_t frame_control;
    uint8_t data[IEEE802154_MAX_UNSEC_MHR_OVERHEAD];
  };
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct ieee802154_phyifops_s mac802154_phyifops =
{
  mac802154_poll_csma,
  mac802154_poll_gts
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mac802154_semtake
 *
 * Description:
 *   Acquire the semaphore used for access serialization.
 *
 ****************************************************************************/

static inline int mac802154_takesem(sem_t *sem)
{
  /* Take a count from the semaphore, possibly waiting */

  if (sem_wait(sem) < 0)
    {
      /* EINTR is the only error that we expect */

      int errcode = get_errno();
      DEBUGASSERT(errcode == EINTR);
      return -errcode;
    }

  return OK;
}

/****************************************************************************
 * Name: mac802154_defaultmib
 *
 * Description:
 *   Set the MIB to its default values.
 *
 ****************************************************************************/

static int mac802154_defaultmib(FAR struct ieee802154_privmac_s *priv)
{
  /* TODO: Set all MAC fields to default values */
  return OK;
}

/****************************************************************************
 * Name: mac802154_applymib
 *
 * Description:
 *   Some parts of the MIB must be sent to the radio device. This routine
 *   calls the radio device routines to store the related parameters in the
 *   radio driver. It must be called each time a MIB parameter is changed.
 *
 ****************************************************************************/

static int mac802154_applymib(FAR struct ieee802154_privmac_s *priv)
{
  return OK;
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
 *   NOTE: This API does not create any device accessible to userspace. If you
 *   want to call these APIs from userspace, you have to wrap your mac in a
 *   character device via mac802154_device.c.
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

  /* Allocate object */

  mac = (FAR struct ieee802154_privmac_s *)
    kmm_zalloc(sizeof(struct ieee802154_privmac_s));

  if (mac == NULL)
    {
      return NULL;
    }

  /* Initialize fields */

  mac->radio = radiodev;

  mac802154_defaultmib(mac);
  mac802154_applymib(mac);
 
  mac->phyif.ops = &mac802154_phyifops;
  mac->phyif.priv = mac;

  /* Bind our PHY interface to the radio */

  radiodev->ops->bind(radiodev, mac->phyif);

  return (MACHANDLE)mac;
}

/****************************************************************************
 * Name: mac802154_bind
 *
 * Description:
 *   Bind the MAC callback table to the MAC state.
 *
 * Parameters:
 *   mac - Reference to the MAC driver state structure
 *   cb  - MAC callback operations
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int mac802154_bind(MACHANDLE mac, FAR const struct ieee802154_maccb_s *cb)
{
  FAR struct ieee802154_privmac_s *priv =
    (FAR struct ieee802154_privmac_s *)mac;

  priv->cb = cb;
  return OK;
}

/****************************************************************************
 * Name: mac802154_ioctl
 *
 * Description:
 *   Handle MAC and radio IOCTL commands directed to the MAC.
 *
 * Parameters:
 *   mac - Reference to the MAC driver state structure
 *   cmd - The IOCTL command
 *   arg - The argument for the IOCTL command
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int mac802154_ioctl(MACHANDLE mac, int cmd, unsigned long arg)
{
  FAR struct ieee802154_privmac_s *priv =
    (FAR struct ieee802154_privmac_s *)mac;
  int ret = -EINVAL;

  DEBUGASSERT(priv != NULL);

  /* Check for IOCTLs aimed at the IEEE802.15.4 MAC layer */

  if (_MAC802154IOCVALID(cmd))
    {
      /* Handle the MAC IOCTL command */
#warning Missing logic
    }

  /* No, other IOCTLs must be aimed at the IEEE802.15.4 radio layer */

  else
   {
     DEBUGASSERT(priv->radio != NULL &&
                 priv->radio->ops != NULL &&
                 priv->radio->ops->ioctl != NULL);

     ret = priv->radio->ops->ioctl(priv->radio, cmd, arg);
   }

 return ret;
}

/****************************************************************************
 * MAC Interface Operations
 ****************************************************************************/

/****************************************************************************
 * Name: mac802154_req_data
 *
 * Description:
 *   The MCPS-DATA.request primitive requests the transfer of a data SPDU
 *   (i.e., MSDU) from a local SSCS entity to a single peer SSCS entity. 
 *   Confirmation is returned via the
 *   struct ieee802154_maccb_s->conf_data callback.
 *
 ****************************************************************************/

int mac802154_req_data(MACHANDLE mac, FAR struct ieee802154_data_req_s *req)
{
  FAR struct ieee802154_privmac_s *priv = (FAR struct ieee802154_privmac_s *)mac;
  FAR struct mac802154_trans_s *trans;
  struct mac802154_unsec_mhr_s mhr;
  int ret;

  /* Start off assuming there is only the frame_control field in the MHR */

  mhr.length = 2;

  /* Do a preliminary check to make sure the MSDU isn't too long for even the
   * best case */
  
  if (req->msdu_length > IEEE802154_MAX_MAC_PAYLOAD_SIZE)
    {
      return -EINVAL;
    }

  /* Ensure we start with a clear frame control field */

  mhr.frame_control = 0;

  /* Set the frame type to Data */

  mhr.frame_control |= IEEE802154_FRAME_DATA << IEEE802154_FRAMECTRL_SHIFT_FTYPE;

  /* If the msduLength is greater than aMaxMACSafePayloadSize, the MAC sublayer
   * will set the Frame Version to one. [1] pg. 118 */

  if (req->msdu_length > IEEE802154_MAX_SAFE_MAC_PAYLOAD_SIZE)
    {
      mhr.frame_ctrl |= IEEE802154_FRAMECTRL_VERSION;
    }

  /* If the TXOptions parameter specifies that an acknowledged transmission is
   * required, the AR field will be set appropriately, as described in
   * 5.1.6.4 [1] pg. 118 */

  mhr.frame_ctrl |= (req->ack_tx << IEEE802154_FRAMECTRL_SHIFT_ACKREQ);

  /* If the destination address is present, copy the PAN ID and one of the 
   * addresses, depending on mode, into the MHR */ 
  
  if (req->dest_addr.mode != IEEE802154_ADDRMODE_NONE)
    {
      memcpy(&mhr.data[mhr.length], req->dest_addr.panid, 2);
      mhr.length += 2;

      if (req->dest_addr.mode == IEEE802154_ADDRMODE_SHORT)
        {
          memcpy(&mhr.data[mhr.length], req->dest_addr.saddr, 2);
          mhr.length += 2;
        }
      else if (req->dest_addr.mode == IEEE802154_ADDRMODE_EXTENDED)
        {
          memcpy(&mhr.data[mhr.length], req->dest_addr.eaddr, 8);
          mhr.length += 8;
        }
    }
  
  /* Set the destination addr mode inside the frame contorl field */

  mhr.frame_ctrl |= (req->dest_addr.mode << IEEE802154_FRAMECTRL_SHIFT_DADDR);

  /* From this point on, we need exclusive access to the privmac struct */

  ret = mac802154dev_takesem(&dev->md_exclsem);
  if (ret < 0)
    {
      wlerr("ERROR: mac802154_takesem failed: %d\n", ret);
      return ret;
    }

  /* If both destination and source addressing information is present, the MAC
   * sublayer shall compare the destination and source PAN identifiers. 
   * [1] pg. 41 */

  if (req->src_addr_mode  != IEEE802154_ADDRMODE_NONE &&
      req->dest_addr.mode != IEEE802154_ADDRMODE_NONE)
    {
      /* If the PAN identifiers are identical, the PAN ID Compression field
       * shall be set to one, and the source PAN identifier shall be omitted
       * from the transmitted frame. [1] pg. 41 */

      if(req->dest_addr.panid == priv->addr.panid)
        {
          mhr.frame_control |= IEEE802154_FRAMECTRL_PANIDCOMP;
        }
    }

  if (req->src_addr_mode != IEEE802154_ADDRMODE_NONE)
    {
      /* If the destination address is not included, or if PAN ID Compression
       * is off, we need to include the Source PAN ID */

      if (req->dest_addr.mode == IEEE802154_ADDRMODE_NONE ||
          (mhr.frame_control & IEEE802154_FRAMECTRL_PANIDCOMP)
        {
          memcpy(&mhr.data[mhr.length], priv->addr.panid, 2);
          mhr.length += 2;
        }

      if (req->src_addr_mode == IEEE802154_ADDRMODE_SHORT)
        {
          memcpy(&mhr.data[mhr.length], priv->addr.saddr, 2);
          mhr.length += 2;
        }
      else if (req->src_addr_mode == IEEE802154_ADDRMODE_EXTENDED)
        {
          memcpy(&mhr.data[mhr.length], priv->addr.eaddr, 8);
          mhr.length += 8;
        }
    }

  /* Set the source addr mode inside the frame contorl field */

  mhr.frame_ctrl |= (req->src_addr_mode << IEEE802154_FRAMECTRL_SHIFT_SADDR);

  /* Each time a data or a MAC command frame is generated, the MAC sublayer
   * shall copy the value of macDSN into the Sequence Number field of the MHR
   * of the outgoing frame and then increment it by one. [1] pg. 40 */

  mhr.data[mhr.length++] = priv.dsn++;

  /* Now that we know which fields are included in the header, we can make
   * sure we actually have enough room in the PSDU */
   
  if (mhr.length + req->msdu_length + IEEE802154_MFR_LENGTH >
      IEEE802154_MAX_PHY_PACKET_SIZE)
  {
    return -E2BIG;
  }

  trans->mhr_buf = &mhr.data[0];
  trans->mhr_len = mhr.length;

  trans->d_buf = &req->msdu[0];
  trans->d_len = req->msdu_length;

  trans->msdu_handle = req->msdu_handle;

  /* If the TxOptions parameter specifies that a GTS transmission is required, 
   * the MAC sublayer will determine whether it has a valid GTS as described 
   * 5.1.7.3. If a valid GTS could not be found, the MAC sublayer will discard
   * the MSDU. If a valid GTS was found, the MAC sublayer will defer, if 
   * necessary, until the GTS. If the TxOptions parameter specifies that a GTS
   * transmission is not required, the MAC sublayer will transmit the MSDU using
   * either slotted CSMA-CA in the CAP for a beacon-enabled PAN or unslotted
   * CSMA-CA for a nonbeacon-enabled PAN. Specifying a GTS transmission in the
   * TxOptions parameter overrides an indirect transmission request.
   * [1] pg. 118 */

  if (req->gts_tx)
    {
      /* TODO: Support GTS transmission. This should just change where we link
       * the transaction.  Instead of going in the CSMA transaction list, it
       * should be linked to the GTS' transaction list. We'll need to check if
       * the GTS is valid, and then find the GTS, before linking. Note, we also
       * don't have to try and kick-off any transmission here. */

      return -ENOTSUP;
    }
  else
    {
      /* If the TxOptions parameter specifies that an indirect transmission is
       * required and this primitive is received by the MAC sublayer of a
       * coordinator, the data frame is sent using indirect transmission, as
       * described in 5.1.5 and 5.1.6.3. [1] */

      if (req->indirect_tx)
        {
          /* If the TxOptions parameter specifies that an indirect transmission
           * is required and if the device receiving this primitive is not a
           * coordinator, the destination address is not present, or the 
           * TxOptions parameter also specifies a GTS transmission, the indirect
           * transmission option will be ignored. [1] */

          if (priv->is_coord && req->dest_addr.mode != IEEE802154_ADDRMODE_NONE) 
            {
              /* Link the transaction into the indirect_trans list */

              priv->indirect_tail->flink = trans;
              priv->indirect_tail = trans;
            }
          else
            {
              /* Override the setting since it wasn't valid */

              req->indirect_tx = 0;
            }
        }

      /* If this is a direct transmission not during a GTS */

      if (!req->indirect_tx)
        {
          /* Link the transaction into the CSMA transaction list */

          priv->csma_tail->flink = trans;
          priv->csma_tail = trans;

          /* Notify the radio driver that there is data available */

          priv->radio->tx_notify(priv->radio);

          sem_wait(&trans->sem);
        }
    }

  return OK;
}


/* Called from interrupt level or worker thread with interrupts disabled */

static uint16_t mac802154_poll_csma(FAR struct ieee802154_phyif_s *phyif, 
                                    FAR struct ieee802154_txdesc_s *tx_desc,
                                    uint8_t *buf)
{
  FAR struct ieee802154_privmac_s *priv = 
      (FAR struct ieee802154_privmac_s *)&phyif->priv;

  FAR struct mac802154_trans_s *trans;

  /* Check to see if there are any CSMA transactions waiting */

  if (mac->csma_head)
    {
      /* Pop a CSMA transaction off the list */

      trans = mac->csma_head;
      mac->csma_head = mac->csma_head.flink;

      /* Setup the transmit descriptor */

      tx_desc->psdu_handle = trans->msdu_handle;
      tx_desc->psdu_length = trans->mhr_len + trans->d_len;

      /* Copy the frame into the buffer */

      memcpy(&buf[0], trans->mhr_buf, trans->mhr_len);
      memcpy(&buf[trans->mhr_len], trans->d_buf, trans->d_len);

      /* Now that we've passed off the data, notify the waiting thread.
       * NOTE: The transaction was allocated on the waiting thread's stack so
       * it will be automatically deallocated when that thread awakens and
       * returns */

      sem_post(trans->sem);

      return txdesc->psdu_length;
    }

  return 0;
}

/****************************************************************************
 * Name: mac802154_req_purge
 *
 * Description:
 *   The MCPS-PURGE.request primitive allows the next higher layer to purge an
 *   MSDU from the transaction queue. Confirmation is returned via
 *   the struct ieee802154_maccb_s->conf_purge callback.
 *
 ****************************************************************************/

int mac802154_req_purge(MACHANDLE mac, uint8_t handle)
{
  FAR struct ieee802154_privmac_s *priv = (FAR struct ieee802154_privmac_s *)mac;
  return -ENOTTY;
}

/****************************************************************************
 * Name: mac802154_req_associate
 *
 * Description:
 *   The MLME-ASSOCIATE.request primitive allows a device to request an 
 *   association with a coordinator. Confirmation is returned via the
 *   struct ieee802154_maccb_s->conf_associate callback.
 *
 ****************************************************************************/

int mac802154_req_associate(MACHANDLE mac,
                            FAR struct ieee802154_assoc_req_s *req)
{
  FAR struct ieee802154_privmac_s *priv = (FAR struct ieee802154_privmac_s *)mac;

  /* Set the channel of the PHY layer */

  /* Set the channel page of the PHY layer */

  /* Set the macPANId */

  /* Set either the macCoordExtendedAddress and macCoordShortAddress
   * depending on the CoordAddrMode in the primitive */

  if (req->coord_addr.mode == IEEE802154_ADDRMODE_EXTENDED)
    {

    } 
  else if (req->coord_addr.mode == IEEE802154_ADDRMODE_EXTENDED)
    {

    }
  else 
    {
      return -EINVAL;
    }

  return -ENOTTY;
}

/****************************************************************************
 * Name: mac802154_req_disassociate
 *
 * Description:
 *   The MLME-DISASSOCIATE.request primitive is used by an associated device to
 *   notify the coordinator of its intent to leave the PAN. It is also used by
 *   the coordinator to instruct an associated device to leave the PAN.
 *   Confirmation is returned via the
 *   struct ieee802154_maccb_s->conf_disassociate callback.
 *
 ****************************************************************************/

int mac802154_req_disassociate(MACHANDLE mac,
                               FAR struct ieee802154_disassoc_req_s *req)
{
  FAR struct ieee802154_privmac_s *priv = (FAR struct ieee802154_privmac_s *)mac;
  return -ENOTTY;
}

/****************************************************************************
 * Name: mac802154_req_get
 *
 * Description:
 *   The MLME-GET.request primitive requests information about a given PIB
 *   attribute. Actual data is returned via the
 *   struct ieee802154_maccb_s->conf_get callback.
 *
 ****************************************************************************/

int mac802154_req_get(MACHANDLE mac, enum ieee802154_pib_attr_e attr)
{
  FAR struct ieee802154_privmac_s *priv = (FAR struct ieee802154_privmac_s *)mac;
  return -ENOTTY;
}

/****************************************************************************
 * Name: mac802154_req_gts
 *
 * Description:
 *   The MLME-GTS.request primitive allows a device to send a request to the PAN
 *   coordinator to allocate a new GTS or to deallocate an existing GTS.
 *   Confirmation is returned via the
 *   struct ieee802154_maccb_s->conf_gts callback.
 *
 ****************************************************************************/

int mac802154_req_gts(MACHANDLE mac, FAR uint8_t *characteristics)
{
  FAR struct ieee802154_privmac_s *priv = (FAR struct ieee802154_privmac_s *)mac;
  return -ENOTTY;
}

/****************************************************************************
 * Name: mac802154_req_reset
 *
 * Description:
 *   The MLME-RESET.request primitive allows the next higher layer to request
 *   that the MLME performs a reset operation. Confirmation is returned via
 *   the struct ieee802154_maccb_s->conf_reset callback.
 *
 ****************************************************************************/

int mac802154_req_reset(MACHANDLE mac, bool setdefaults)
{
  FAR struct ieee802154_privmac_s * priv = (FAR struct ieee802154_privmac_s *) mac;
  return -ENOTTY;
}

/****************************************************************************
 * Name: mac802154_req_rxenable
 *
 * Description:
 *   The MLME-RX-ENABLE.request primitive allows the next higher layer to
 *   request that the receiver is enable for a finite period of time.
 *   Confirmation is returned via the
 *   struct ieee802154_maccb_s->conf_rxenable callback.
 *
 ****************************************************************************/

int mac802154_req_rxenable(MACHANDLE mac, bool deferrable, int ontime,
                           int duration)
{
  FAR struct ieee802154_privmac_s * priv = (FAR struct ieee802154_privmac_s *)mac;
  return -ENOTTY;
}

/****************************************************************************
 * Name: mac802154_req_scan
 *
 * Description:
 *   The MLME-SCAN.request primitive is used to initiate a channel scan over a
 *   given list of channels. A device can use a channel scan to measure the
 *   energy on the channel, search for the coordinator with which it associated,
 *   or search for all coordinators transmitting beacon frames within the POS of
 *   the scanning device. Scan results are returned
 *   via MULTIPLE calls to the struct ieee802154_maccb_s->conf_scan callback.
 *   This is a difference with the official 802.15.4 specification, implemented
 *   here to save memory.
 *
 ****************************************************************************/

int mac802154_req_scan(MACHANDLE mac, uint8_t type, uint32_t channels,
                       int duration)
{
  FAR struct ieee802154_privmac_s *priv = (FAR struct ieee802154_privmac_s *)mac;
  return -ENOTTY;
}

/****************************************************************************
 * Name: mac802154_req_set
 *
 * Description:
 *   The MLME-SET.request primitive attempts to write the given value to the
 *   indicated MAC PIB attribute. Confirmation is returned via the
 *   struct ieee802154_maccb_s->conf_set callback.
 *
 ****************************************************************************/

int mac802154_req_set(MACHANDLE mac, int attribute, FAR uint8_t *value,
                      int valuelen)
{
  FAR struct ieee802154_privmac_s *priv = (FAR struct ieee802154_privmac_s *)mac;
  return -ENOTTY;
}

/****************************************************************************
 * Name: mac802154_req_start
 *
 * Description:
 *   The MLME-START.request primitive makes a request for the device to start
 *   using a new superframe configuration. Confirmation is returned
 *   via the struct ieee802154_maccb_s->conf_start callback.
 *
 ****************************************************************************/

int mac802154_req_start(MACHANDLE mac, uint16_t panid, int channel,
                        uint8_t bo, uint8_t fo, bool coord, bool batext,
                        bool realign)
{
  FAR struct ieee802154_privmac_s *priv = (FAR struct ieee802154_privmac_s *)mac;
  return -ENOTTY;
}

/****************************************************************************
 * Name: mac802154_req_sync
 *
 * Description:
 *   The MLME-SYNC.request primitive requests to synchronize with the
 *   coordinator by acquiring and, if specified, tracking its beacons.
 *   Confirmation is returned via the
 *   struct ieee802154_maccb_s->int_commstatus callback. TOCHECK.
 *
 ****************************************************************************/

int mac802154_req_sync(MACHANDLE mac, int channel, bool track)
{
  FAR struct ieee802154_privmac_s *priv = (FAR struct ieee802154_privmac_s *)mac;
  return -ENOTTY;
}

/****************************************************************************
 * Name: mac802154_req_poll
 *
 * Description:
 *   The MLME-POLL.request primitive prompts the device to request data from the
 *   coordinator. Confirmation is returned via the 
 *   struct ieee802154_maccb_s->conf_poll callback, followed by a
 *   struct ieee802154_maccb_s->ind_data callback.
 *
 ****************************************************************************/

int mac802154_req_poll(MACHANDLE mac, FAR uint8_t *coordaddr)
{
  FAR struct ieee802154_privmac_s *priv = (FAR struct ieee802154_privmac_s *)mac;
  return -ENOTTY;
}

/****************************************************************************
 * Name: mac802154_rsp_associate
 *
 * Description:
 *   The MLME-ASSOCIATE.response primitive is used to initiate a response to an 
 *   MLME-ASSOCIATE.indication primitive.
 *
 ****************************************************************************/

int mac802154_rsp_associate(MACHANDLE mac, uint8_t eadr, uint16_t saddr,
                            int status)
{
  FAR struct ieee802154_privmac_s *priv = (FAR struct ieee802154_privmac_s *)mac;
  return -ENOTTY;
}

/****************************************************************************
 * Name: mac802154_rsp_orphan
 *
 * Description:
 *   The MLME-ORPHAN.response primitive allows the next higher layer of a
 *   coordinator to respond to the MLME-ORPHAN.indication primitive.
 *
 ****************************************************************************/

int mac802154_rsp_orphan(MACHANDLE mac, FAR uint8_t *orphanaddr,
                         uint16_t saddr, bool associated)
{
  FAR struct ieee802154_privmac_s *priv =
    (FAR struct ieee802154_privmac_s *)mac;
  return -ENOTTY;
}
