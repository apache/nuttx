/****************************************************************************
 * wireless/ieee802154/mac802154.c
 *
 *   Copyright (C) 2016 Sebastien Lorquet. All rights reserved.
 *   Copyright (C) 2017 Verge Inc. All rights reserved.
 *   Author: Sebastien Lorquet <sebastien@lorquet.fr>
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

#include <nuttx/wireless/ieee802154/ieee802154_mac.h>
#include <nuttx/wireless/ieee802154/ieee802154_radio.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* If processing is not done at the interrupt level, then work queue support
 * is required.
 */

#if !defined(CONFIG_SCHED_WORKQUEUE)
#  error Work queue support is required in this configuration (CONFIG_SCHED_WORKQUEUE)
#else

  /* Use the low priority work queue if possible */

#  if defined(CONFIG_MAC802154_HPWORK)
#    define MAC802154_WORK HPWORK
#  elif defined(CONFIG_MAC802154_LPWORK)
#    define MAC802154_WORK LPWORK
#  else
#    error Neither CONFIG_MAC802154_HPWORK nor CONFIG_MAC802154_LPWORK defined
#  endif
#endif

#if !defined(CONFIG_MAC802154_NNOTIF) || CONFIG_MAC802154_NNOTIF <= 0
#  undef CONFIG_MAC802154_NNOTIF
#  define CONFIG_MAC802154_NNOTIF 6 
#endif

#if !defined(CONFIG_MAC802154_NTXDESC) || CONFIG_MAC802154_NTXDESC <= 0
#  undef CONFIG_MAC802154_NTXDESC
#  define CONFIG_MAC802154_NTXDESC 3 
#endif

#if CONFIG_MAC802154_NTXDESC > CONFIG_MAC802154_NNOTIF
#error CONFIG_MAC802154_NNOTIF must be greater than CONFIG_MAC802154_NTXDESC
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct mac802154_txtrans_s
{
  /* Supports a singly linked list */

  FAR struct mac802154_txtrans_s *flink;
  FAR struct iob_s *frame;
  uint8_t handle;
  enum ieee802154_frametype_e frametype;
  sem_t sem;
};

struct mac802154_unsec_mhr_s
{
  uint8_t length;
  union
  {
    uint16_t frame_control;
    uint8_t data[IEEE802154_MAX_UNSEC_MHR_OVERHEAD];
  } u;
};

struct mac802154_radiocb_s
{
  struct ieee802154_radiocb_s cb;
  FAR struct ieee802154_privmac_s *priv;
};

/* The privmac structure holds the internal state of the MAC and is the
 * underlying represention of the opaque MACHANDLE.  It contains storage for
 * the IEEE802.15.4 MIB attributes.
 */

struct ieee802154_privmac_s
{
  FAR struct ieee802154_radio_s *radio;     /* Contained IEEE802.15.4 radio dev */
  FAR const struct mac802154_maccb_s *cb;   /* Contained MAC callbacks */
  FAR struct mac802154_radiocb_s radiocb;   /* Interface to bind to radio */

  sem_t exclsem; /* Support exclusive access */

  /* Support a single transaction dedicated to commands. As of now I see no
   * condition where you need to have more than one command frame simultaneously
   */ 
  
  struct
    {
      sem_t sem;                            /* Exclusive use of the cmdtrans */
      enum ieee802154_cmdid_e type;         /* Type of cmd in the cmdtrans */
      struct mac802154_txtrans_s trans;     /* Dedicated txframe for cmds */

      /* Has the command been successfully sent. This is to help protect
       * against an odd edge case that may or may not ever happen. The condition
       * occurs when you receive a seemingly appropriate response to the command
       * yet the command was never actually sent.
       */

      bool txdone;                              
    } cmd;

  /* Pre-allocated notifications to be passed to the registered callback.  These
   * need to be freed by the application using mac802154_xxxxnotif_free when
   * the callee layer is finished with it's use.
   */

  FAR struct ieee802154_notif_s *notif_free;
  struct ieee802154_notif_s notif_alloc[CONFIG_MAC802154_NNOTIF];
  sq_queue_t notif_queue;

  FAR struct ieee802154_txdesc_s *txdesc_free;
  struct ieee802154_txdesc_s txdesc_alloc[CONFIG_IEEE802154_NTXDESC];
  sq_queue_t txdesc_queue;
  sq_queue_t txdone_queue;

  /* Work structures for offloading aynchronous work */

  struct work_s tx_work;
  struct work_s rx_work;

  /* Support a singly linked list of transactions that will be sent using the
   * CSMA algorithm.  On a non-beacon enabled PAN, these transactions will be
   * sent whenever. On a beacon-enabled PAN, these transactions will be sent
   * during the CAP of the Coordinator's superframe.
   */

  sq_queue_t csma_queue;

  /* Support a singly linked list of transactions that will be sent indirectly.
   * This list should only be used by a MAC acting as a coordinator.  These
   * transactions will stay here until the data is extracted by the destination
   * device sending a Data Request MAC command or if too much time passes. This
   * list should also be used to populate the address list of the outgoing
   * beacon frame.
   */
  
  sq_queue_t indirect_queue;

  /* Support a singly linked list of frames received */

  sq_queue_t dataind_queue;

  /* MAC PIB attributes, grouped to save memory */

  /* Holds all address information (Extended, Short, and PAN ID) for the MAC. */

  struct ieee802154_addr_s addr;

  /* Holds all address information (Extended, Short) for Coordinator */

  struct ieee802154_addr_s coordaddr;

  /* The maximum number of symbols to wait for an acknowledgement frame to
   * arrive following a transmitted data frame. [1] pg. 126
   *
   * NOTE: This may be able to be a 16-bit or even an 8-bit number.  I wasn't
   * sure at the time what the range of reasonable values was.
   */

  uint32_t ack_waitdur;

  /* The maximum time to wait either for a frame intended as a response to a
   * data request frame or for a broadcast frame following a beacon with the
   * Frame Pending field set to one. [1] pg. 127
   *
   * NOTE: This may be able to be a 16-bit or even an 8-bit number.  I wasn't
   * sure at the time what the range of reasonable values was.
   */

  uint32_t max_frame_waittime;

  /* The maximum time (in unit periods) that a transaction is stored by a
   * coordinator and indicated in its beacon.
   */

  uint16_t trans_persisttime;

  /* Contents of beacon payload */

  uint8_t beacon_payload[IEEE802154_MAX_BEACON_PAYLOAD_LEN];
  uint8_t beacon_payload_len;       /* Length of beacon payload */

  uint8_t battlifeext_periods;    /* # of backoff periods during which rx is
                                     * enabled after the IFS following beacon */

  uint8_t bsn;                      /* Seq. num added to tx beacon frame */
  uint8_t dsn;                      /* Seq. num added to tx data or MAC frame */
  uint8_t maxretries;               /* Max # of retries alloed after tx failure */

  /* The maximum time, in multiples of aBaseSuperframeDuration, a device shall
   * wait for a response command frame to be available following a request
   * command frame. [1] 128.
   */

  uint8_t resp_waittime;

  /* The total transmit duration (including PHY header and FCS) specified in
   * symbols. [1] pg. 129.
   */

  uint32_t tx_totaldur;

  /* Start of 32-bit bitfield */

  uint32_t isassoc            : 1;  /* Are we associated to the PAN */
  uint32_t assocpermit        : 1;  /* Are we allowing assoc. as a coord. */
  uint32_t autoreq            : 1;  /* Automatically send data req. if addr
                                     * addr is in the beacon frame */

  uint32_t battlifeext        : 1;  /* Is BLE enabled */
  uint32_t gtspermit          : 1;  /* Is PAN Coord. accepting GTS reqs. */
  uint32_t promisc            : 1;  /* Is promiscuous mode on? */
  uint32_t rngsupport         : 1;  /* Does MAC sublayer support ranging */
  uint32_t sec_enabled        : 1;  /* Does MAC sublayer have security en. */
  uint32_t timestamp_support  : 1;  /* Does MAC layer supports timestamping */

  uint32_t max_csmabackoffs   : 3;  /* Max num backoffs for CSMA algorithm
                                     * before declaring ch access failure */

  uint32_t beaconorder        : 4;  /* Freq. that beacon is transmitted */

  uint32_t superframeorder    : 4;  /* Length of active portion of outgoing
                                     * superframe, including the beacon */

  /* The offset, measured is symbols, between the symbol boundary at which the
   * MLME captures the timestamp of each transmitted and received frame, and
   * the onset of the first symbol past the SFD, namely the first symbol of
   * the frames [1] pg. 129.
   */

  uint32_t sync_symboffset    : 12;

  /* End of 32-bit bitfield */

  /* Start of 32-bit bitfield */

  uint32_t beacon_txtime      : 24; /* Time of last beacon transmit */
  uint32_t minbe              : 4;  /* Min value of backoff exponent (BE) */
  uint32_t maxbe              : 4;  /* Max value of backoff exponent (BE) */

  /* End of 32-bit bitfield */

  /* Start of 32-bit bitfield */

  uint32_t txctrl_activedur   : 17; /* Duration for which tx is permitted to
                                       * be active */
  uint32_t txctrl_pausedur    : 1;  /* Duration after tx before another tx is
                                     * permitted. 0=2000, 1= 10000 */

  /* What type of device is this node acting as */

  enum ieee802154_devmode_e devmode : 2;

  bool csma_tryagain          : 1;
  bool gts_tryagain           : 1;

                                    /* 10-bits remaining */

  /* End of 32-bit bitfield. */

  /* TODO: Add Security-related MAC PIB attributes */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Internal Functions */

static inline int mac802154_takesem(sem_t *sem);
#define mac802154_givesem(s) sem_post(s);

static void mac802154_resetqueues(FAR struct ieee802154_privmac_s *priv);
static void mac802154_notifpool_init(FAR struct ieee802154_privmac_s *priv);
static FAR struct ieee802154_notif_s *
  mac802154_notif_alloc(FAR struct ieee802154_privmac_s *priv);

static int mac802154_defaultmib(FAR struct ieee802154_privmac_s *priv);
static int mac802154_applymib(FAR struct ieee802154_privmac_s *priv);

static void mac802154_txdone_worker(FAR void *arg);
static void mac802154_rxframe_worker(FAR void *arg);

static void mac802154_cmd_txdone(FAR struct ieee802154_privmac_s *priv,
                                 FAR struct ieee802154_txdesc_s *txdesc);

/* IEEE 802.15.4 PHY Interface OPs */

static int mac802154_poll_csma(FAR const struct ieee802154_radiocb_s *radiocb,
                               FAR struct ieee802154_txdesc_s **tx_desc,
                               FAR struct iob_s **frame);

static int mac802154_poll_gts(FAR const struct ieee802154_radiocb_s *radiocb,
                              FAR struct ieee802154_txdesc_s **tx_desc,
                              FAR struct iob_s **frame);

static void mac802154_txdone(FAR const struct ieee802154_radiocb_s *radiocb,
                             FAR const struct ieee802154_txdesc_s *tx_desc);

static void mac802154_rxframe(FAR const struct ieee802154_radiocb_s *radiocb,
                              FAR struct ieee802154_data_ind_s *ind);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Map between ieee802154_addrmode_e enum and actual address length */

static const uint8_t mac802154_addr_length[4] = {0, 0, 2, 8};

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
  sq_init(&priv->indirect_queue);
  sq_init(&priv->dataind_queue);

  sq_init(&priv->notif_queue);
  sq_init(&priv->txdesc_queue);

  for (i = 0; i < CONFIG_MAC802154_NNOTIF; i++)
    {
      sq_addlast((FAR sq_entry_t *)&priv->notif_alloc[i], &priv->notif_queue);
    }

  for (i = 0; i < CONFIG_MAC802154_NTXDESC; i++)
    {
      sq_addlast((FAR sq_entry_t *)&priv->txdesc_alloc[i], &priv->txdesc_queue);
    }
  
  mac802154_notifpool_init(priv);
}

/****************************************************************************
 * Name: mac802154_notifpool_init
 *
 * Description:
 *   This function initializes the notification structure pool. It allows the
 *   MAC to pass notifications and for the callee to free them when they are
 *   done using them, saving copying the data when passing.
 *
 ****************************************************************************/

static void mac802154_notifpool_init(FAR struct ieee802154_privmac_s *priv)
{
  FAR struct ieee802154_notif_s *pool = priv->notif_alloc;
  int remaining = CONFIG_MAC802154_NNOTIF;

  priv->notif_free = NULL;
  while (remaining > 0)
    {
      FAR struct ieee802154_notif_s *notif = pool;

      /* Add the next meta data structure from the pool to the list of
       * general structures.
       */

      notif->flink = priv->notif_free;
      priv->notif_free  = notif;

      /* Set up for the next structure from the pool */

      pool++;
      remaining--;
    }
}

/****************************************************************************
 * Name: mac802154_notif_alloc
 *
 * Description:
 *   This function allocates a free notification structure from the free list
 *   to be used for passing to the registered notify callback. The callee software
 *   is responsible for freeing the notification structure after it is done using
 *   it via mac802154_notif_free.
 *
 * Assumptions:
 *   priv MAC struct is locked when calling.
 *
 ****************************************************************************/
static FAR struct ieee802154_notif_s *
  mac802154_notif_alloc(FAR struct ieee802154_privmac_s *priv)
{
  FAR struct ieee802154_notif_s *notif;

  if (priv->notif_free == NULL)
    {
      return NULL;
    }

  notif             = priv->notif_free;
  priv->notif_free  = notif->flink;

  return notif;
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
  priv->isassoc = false;        /* Not associated with a PAN */
  priv->assocpermit = false;    /* Device (if coord) not accepting association */
  priv->autoreq = true;         /* Auto send data req if addr. in beacon */
  priv->battlifeext = false;    /* BLE disabled */
  priv->beacon_payload_len = 0; /* Beacon payload NULL */
  priv->beaconorder = 15;       /* Non-beacon enabled network */
  priv->superframeorder = 15;   /* Length of active portion of outgoing SF */
  priv->beacon_txtime = 0;      /* Device never sent a beacon */
#warning Set BSN and DSN to random values!
  priv->bsn = 0;
  priv->dsn = 0;
  priv->gtspermit = true;       /* PAN Coord accepting GTS requests */
  priv->minbe = 3;              /* Min value of backoff exponent (BE) */
  priv->maxbe = 5;              /* Max value of backoff exponent (BE) */
  priv->max_csmabackoffs = 4;   /* Max # of backoffs before failure */
  priv->maxretries = 3;         /* Max # of retries allowed after failure */
  priv->promisc = false;        /* Device not in promiscuous mode */
  priv->rngsupport = false;     /* Ranging not yet supported */
  priv->resp_waittime = 32;     /* 32 SF durations */
  priv->sec_enabled = false;    /* Security disabled by default */
  priv->tx_totaldur = 0;        /* 0 transmit duration */

  priv->trans_persisttime = 0x01F4;

  /* Reset the Coordinator address */

  priv->coordaddr.mode = IEEE802154_ADDRMODE_NONE;
  priv->coordaddr.saddr = IEEE802154_SADDR_UNSPEC;
  memcpy(&priv->coordaddr.eaddr[0], IEEE802154_EADDR_UNSPEC,
         IEEE802154_EADDR_LEN);

  /* Reset the device's address */

  priv->addr.mode = IEEE802154_ADDRMODE_NONE;
  priv->addr.panid = IEEE802154_PAN_UNSPEC;
  priv->addr.saddr = IEEE802154_SADDR_UNSPEC;
  memcpy(&priv->addr.eaddr[0], IEEE802154_EADDR_UNSPEC, IEEE802154_EADDR_LEN);


  /* These attributes are effected and determined based on the PHY.  Need to
   * figure out how to "share" attributes between the radio driver and this
   * MAC layer
   *
   *    macAckWaitDuration
   *    macBattLifeExtPeriods
   *    macMaxFrameTotalWaitTime
   *    macLIFSPeriod
   *    macSIFSPeriod
   *    macSyncSymbolOffset
   *    macTimestampSupported
   *    macTxControlActiveDuration
   *    macTxControlPauseDuration
   *    macRxOnWhenIdle
   */

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
 * Name: mac802154_poll_csma
 *
 * Description:
 *   Called from the radio driver through the callback struct.  This function is
 *   called when the radio has room for another CSMA transaction.  If the MAC
 *   layer has a CSMA transaction, it copies it into the supplied buffer and
 *   returns the length.  A descriptor is also populated with the transaction.
 *
 ****************************************************************************/

static int mac802154_poll_csma(FAR const struct ieee802154_radiocb_s *radiocb,
                               FAR struct ieee802154_txdesc_s **txdesc,
                               FAR struct iob_s **frame)
{
  FAR struct mac802154_radiocb_s *cb =
    (FAR struct mac802154_radiocb_s *)radiocb;
  FAR struct ieee802154_privmac_s *priv;
  FAR struct mac802154_txtrans_s *trans;
  FAR struct ieee802154_txdesc_s *desc;
  FAR struct ieee802154_notif_s *notif;

  DEBUGASSERT(cb != NULL && cb->priv != NULL);
  priv = cb->priv;

  /* Get exclusive access to the driver structure.  We don't care about any
   * signals so if we see one, just go back to trying to get access again.
   */

  while (mac802154_takesem(&priv->exclsem) != 0);

  /* Check to see if there are any CSMA transactions waiting */

  trans = (FAR struct mac802154_txtrans_s *)sq_remfirst(&priv->csma_queue);
  mac802154_givesem(&priv->exclsem);

  if (trans != NULL)
    {
      /* Allocate a Tx descriptor to pass */

      desc = (FAR struct ieee802154_txdesc_s *)sq_remfirst(&priv->txdesc_queue);
      if (desc == NULL)
        {
          wlerr("ERROR: Failed to allocate ieee802154_txdesc_s");
          goto errout;
        }

      /* Allocate a notif struct (ie data confirmation struct) to pass with
       * the tx descriptor.
       */
      
      notif = mac802154_notif_alloc(priv);
      if (notif == NULL)
        {
          wlerr("ERROR: Failed to allocate ieee802154_notif_s");

          /* Free the tx descriptor */

          sq_addlast((FAR sq_entry_t *)desc, &priv->txdesc_queue);
          goto errout;
        }

      desc->conf = (FAR struct ieee802154_data_conf_s *)notif;
      desc->conf->handle = trans->handle;
      desc->frametype = trans->frametype;

      *frame = trans->frame;
      *txdesc = desc;

      /* Now that we've passed off the data, notify the waiting thread.
       * NOTE: The transaction was allocated on the waiting thread's stack so
       * it will be automatically deallocated when that thread awakens and
       * returns. */

      sem_post(&trans->sem);
      return (trans->frame->io_len);
    }

errout:
  /* Need to set flag to tell MAC to retry notifying radio layer about transmit
   * since we couldn't allocate the required data structures at this time.
   */

  priv->csma_tryagain = true;
  mac802154_givesem(&priv->exclsem);
  return 0;
}

/****************************************************************************
 * Name: mac802154_poll_gts
 *
 * Description:
 *   Called from the radio driver through the callback struct.  This function is
 *   called when the radio has room for another GTS transaction.  If the MAC
 *   layer has a GTS transaction, it copies it into the supplied buffer and
 *   returns the length.  A descriptor is also populated with the transaction.
 *
 ****************************************************************************/

static int mac802154_poll_gts(FAR const struct ieee802154_radiocb_s *radiocb,
                              FAR struct ieee802154_txdesc_s **tx_desc,
                              FAR struct iob_s **frame)
{
  FAR struct mac802154_radiocb_s *cb =
    (FAR struct mac802154_radiocb_s *)radiocb;
  FAR struct ieee802154_privmac_s *priv;
  FAR struct mac802154_txtrans_s *trans;
  FAR struct ieee802154_txdesc_s *desc;
  int ret = 0;

  DEBUGASSERT(cb != NULL && cb->priv != NULL);
  priv = cb->priv;

  /* Get exclusive access to the driver structure.  We don't care about any
   * signals so if we see one, just go back to trying to get access again.
   */

  while (mac802154_takesem(&priv->exclsem) != 0);

#warning Missing logic.

  mac802154_givesem(&priv->exclsem);

  return 0;

errout:
  /* Need to set flag to tell MAC to retry notifying radio layer about transmit
   * since we couldn't allocate the required data structures at this time.
   */

  priv->gts_tryagain = true;
  mac802154_givesem(&priv->exclsem);
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
                            FAR const struct ieee802154_txdesc_s *txdesc)
{
  FAR struct mac802154_radiocb_s *cb =
    (FAR struct mac802154_radiocb_s *)radiocb;
  FAR struct ieee802154_privmac_s *priv;

  DEBUGASSERT(cb != NULL && cb->priv != NULL);
  priv = cb->priv;

  /* Get exclusive access to the driver structure.  We don't care about any
   * signals so if we see one, just go back to trying to get access again.
   */

  while (mac802154_takesem(&priv->exclsem) != 0);

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
  FAR struct ieee802154_data_conf_s *conf;
  FAR struct ieee802154_notif_s *notif;
  enum ieee802154_frametype_e frametype;
  int count;

  /* Get exclusive access to the driver structure.  We don't care about any
   * signals so if we see one, just go back to trying to get access again.
   */

  while (mac802154_takesem(&priv->exclsem) != 0);

  while (1)
    {
      txdesc = (FAR struct ieee802154_txdesc_s *)sq_remfirst(&priv->txdone_queue);

      if (txdesc == NULL)
        {
          break;
        }

      count++;
      
      /* Once we get the frametype and data confirmation struct, we can free
       * the tx descriptor.
       */

      conf      = txdesc->conf;
      frametype = txdesc->frametype;
      sq_addlast((FAR sq_entry_t *)txdesc, &priv->txdesc_queue);

      /* Cast the data_conf to a notification */

      notif = (FAR struct ieee802154_notif_s *)conf;
      
      switch(frametype)
        {
          case IEEE802154_FRAME_DATA:
            {
              notif->notiftype = IEEE802154_NOTIFY_CONF_DATA;

              /* Release the MAC then call the callback */

              mac802154_givesem(&priv->exclsem);
              priv->cb->notify(priv->cb, notif);
            }
            break;

          case IEEE802154_FRAME_COMMAND:
            {
              mac802154_cmd_txdone(priv, txdesc);

              /* We can deallocate the data conf notification as it is no longer
               * needed. We don't use the public function here since we already
               * have the MAC locked. Additionally, we are already handling the
               * tx_tryagain here, so we wouldn't want to handle it twice.
               */ 

               notif->flink = priv->notif_free;
               priv->notif_free = notif;
               mac802154_givesem(&priv->exclsem);
            }
            break;

          default:
            {
               mac802154_givesem(&priv->exclsem);
            }
            break;

        }
    }

  /* If we've freed a tx descriptor or notification structure and a previous
   * attempt at passing data to the radio layer failed due to insufficient
   * available structures, try again now that we've freed some resources */

  if (count > 0 && priv->csma_tryagain)
    {
      priv->csma_tryagain = false;
      priv->radio->ops->txnotify_csma(priv->radio);
    }
  
  if (count > 0 && priv->gts_tryagain)
    {
      priv->gts_tryagain = false;
      priv->radio->ops->txnotify_gts(priv->radio);
    }
}

/****************************************************************************
 * Name: mac802154_cmd_txdone
 *
 * Description:
 *   Called from mac802154_txdone_worker, this is a helper function for
 *   handling command frames that have either successfully sent or failed.
 *
 ****************************************************************************/

static void mac802154_cmd_txdone(FAR struct ieee802154_privmac_s *priv,
                                 FAR struct ieee802154_txdesc_s *txdesc)
{

  /* Check to see what type of command it was. All information about the command
   * will still be valid because it is protected by a semaphore.
   */

  switch (priv->cmd.type)
    {
      case IEEE802154_CMD_ASSOC_REQ:
        if(txdesc->conf->status != IEEE802154_STATUS_SUCCESS)
          {
            /* if the association request command cannot be sent due to a
             * channel access failure, the MAC sublayer shall notify the next
             * higher layer. [1] pg. 33
             */


          }
        else
          {
            priv->cmd.txdone = true;
          }
        break;
      default:
        break;
    }
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

  while (mac802154_takesem(&priv->exclsem) != 0);

  /* Push the iob onto the tail of the frame list for processing */

  sq_addlast((FAR sq_entry_t *)ind, &priv->dataind_queue);

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

      while (mac802154_takesem(&priv->exclsem) != 0);

      /* Push the iob onto the tail of the frame list for processing */

      ind = (FAR struct ieee802154_data_ind_s *)sq_remfirst(&priv->dataind_queue);

      if (ind == NULL)
        {
          mac802154_givesem(&priv->exclsem);
          break;
        }

      /* Once we pop off the indication, we don't need to keep the mac locked */

      mac802154_givesem(&priv->exclsem);

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

          ind->dest.panid = frame->io_data[frame->io_offset];
          frame->io_offset += 2;

          if (ind->dest.mode == IEEE802154_ADDRMODE_SHORT)
            {
              ind->dest.saddr = frame->io_data[frame->io_offset];
              frame->io_offset += 2;
            }
          else if (ind->dest.mode == IEEE802154_ADDRMODE_EXTENDED)
            {
              memcpy(&ind->dest.eaddr[0], &frame->io_data[frame->io_offset], 
                     IEEE802154_EADDR_LEN);
              frame->io_offset += 8;
            }
        }

      if (ind->src.mode != IEEE802154_ADDRMODE_NONE)
        {
          /* If the source address is included, and the PAN ID compression field
           * is set, get the PAN ID from the header.
           */

          if (!panid_comp)
            {
              ind->src.panid = frame->io_data[frame->io_offset];
              frame->io_offset += 2;
            }
          
          /* If the source address is included, and the PAN ID compression field
           * is set, the source PAN ID is the same as the destination PAN ID
           */

          else
            {
              ind->src.panid = ind->dest.panid;
            }

          if (ind->src.mode == IEEE802154_ADDRMODE_SHORT)
            {
              ind->src.saddr = frame->io_data[frame->io_offset];
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
      
      if (ftype == IEEE802154_FRAME_DATA)
        {
          /* If there is a registered MCPS callback receiver registered, send
           * the frame, otherwise, throw it out.
           */

          if (priv->cb->rxframe != NULL)
            {
              priv->cb->rxframe(priv->cb, ind);
            }
          else
            {
              /* Free the data indication struct from the pool */

              ieee802154_ind_free(ind);
            }
        }
      else if (ftype == IEEE802154_FRAME_COMMAND)
        {

        }
      else if (ftype == IEEE802154_FRAME_BEACON)
        {

        }
      else
        {
          /* The radio layer is responsible for handling all ACKs and retries. If for
           * some reason an ACK gets here, just throw it out.
           */
        }
    }
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

  /* Allocate object */

  mac = (FAR struct ieee802154_privmac_s *)
    kmm_zalloc(sizeof(struct ieee802154_privmac_s));

  if (mac == NULL)
    {
      return NULL;
    }

  /* Allow exclusive access to the privmac struct */

  sem_init(&mac->exclsem, 0, 1);

  /* Allow exlusive access to the dedicated command transaction */

  sem_init(&mac->cmd.sem, 0, 1);

  /* Setup the signaling semaphore for the dedicated command transaction */

  sem_init(&mac->cmd.trans.sem, 0, 0);
  sem_setprotocol(&mac->cmd.trans.sem, SEM_PRIO_NONE);

  /* Initialize fields */

  mac->radio = radiodev;

  mac802154_defaultmib(mac);
  mac802154_applymib(mac);

  /* Initialize the Radio callbacks */

  mac->radiocb.priv = mac;

  radiocb              = &mac->radiocb.cb;
  radiocb->poll_csma   = mac802154_poll_csma;
  radiocb->poll_gts    = mac802154_poll_gts;
  radiocb->txdone      = mac802154_txdone;
  radiocb->rxframe     = mac802154_rxframe;

  /* Bind our callback structure */

  radiodev->ops->bind(radiodev, &mac->radiocb.cb);

  /* Initialize our various data pools */

  ieee802154_indpool_initialize();
  mac802154_resetqueues(mac);
  
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

int mac802154_bind(MACHANDLE mac, FAR const struct mac802154_maccb_s *cb)
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

  FAR union ieee802154_macarg_u *macarg =
    (FAR union ieee802154_macarg_u *)((uintptr_t)arg);

  DEBUGASSERT(priv != NULL);

  /* Check for IOCTLs aimed at the IEEE802.15.4 MAC layer */

  if (_MAC802154IOCVALID(cmd))
    {
      /* Handle the MAC IOCTL command */

      switch (cmd)
        {
          case MAC802154IOC_MLME_ASSOC_REQUEST:
            {
              ret = mac802154_req_associate(mac, &macarg->assocreq);
            }
            break;
          case MAC802154IOC_MLME_ASSOC_RESPONSE:
            {
              ret = mac802154_resp_associate(mac, &macarg->assocresp);
            }
            break;
          case MAC802154IOC_MLME_DISASSOC_REQUEST:
            {
              ret = mac802154_req_disassociate(mac, &macarg->disassocreq);
            }
            break;
          case MAC802154IOC_MLME_GET_REQUEST:
            {
              ret = mac802154_req_get(mac, macarg->getreq.pib_attr,
                                      &macarg->getreq.attrval);
            }
            break;
          case MAC802154IOC_MLME_GTS_REQUEST:
            {
              ret = mac802154_req_gts(mac, &macarg->gtsreq);
            }
            break;
          case MAC802154IOC_MLME_ORPHAN_RESPONSE:
            {
              ret = mac802154_resp_orphan(mac, &macarg->orphanresp);
            }
            break;
          case MAC802154IOC_MLME_RESET_REQUEST:
            {
              ret = mac802154_req_reset(mac, macarg->resetreq.rst_pibattr);
            }
            break;
          case MAC802154IOC_MLME_RXENABLE_REQUEST:
            {
              ret = mac802154_req_rxenable(mac, &macarg->rxenabreq);
            }
            break;
          case MAC802154IOC_MLME_SCAN_REQUEST:
            {
              ret = mac802154_req_scan(mac, &macarg->scanreq);
            }
            break;
          case MAC802154IOC_MLME_SET_REQUEST:
            {
              ret = mac802154_req_set(mac, macarg->setreq.pib_attr,
                                      &macarg->setreq.attrval);
            }
            break;
          case MAC802154IOC_MLME_START_REQUEST:
            {
              ret = mac802154_req_start(mac, &macarg->startreq);
            }
            break;
          case MAC802154IOC_MLME_SYNC_REQUEST:
            {
              ret = mac802154_req_sync(mac, &macarg->syncreq);
            }
            break;
          case MAC802154IOC_MLME_POLL_REQUEST:
            {
              ret = mac802154_req_poll(mac, &macarg->pollreq);
            }
            break;
          default:
              wlerr("ERROR: Unrecognized cmd: %d\n", cmd);
              ret = -ENOTTY;
              break;
        }
    }
  return ret;
}

/****************************************************************************
 * MAC Interface Operations
 ****************************************************************************/

/****************************************************************************
 * Name: mac802154_get_mhrlen
 *
 * Description:
 *   Calculate the MAC header length given the frame meta-data.
 *
 ****************************************************************************/

int mac802154_get_mhrlen(MACHANDLE mac,
                         FAR const struct ieee802154_frame_meta_s *meta)
{
  FAR struct ieee802154_privmac_s *priv =
    (FAR struct ieee802154_privmac_s *)mac;
  int ret = 3; /* Always frame control (2 bytes) and seq. num (1 byte) */

  /* Check to make sure both the dest address and the source address are not set
   * to NONE */

  if (meta->dest_addr.mode == IEEE802154_ADDRMODE_NONE &&
      meta->src_addrmode == IEEE802154_ADDRMODE_NONE)
    {
      return -EINVAL;
    }

  /* The source address can only be set to NONE if the device is the PAN coord */

  if (meta->src_addrmode == IEEE802154_ADDRMODE_NONE && 
      priv->devmode != IEEE802154_DEVMODE_PANCOORD)
    {
      return -EINVAL;
    }

  /* Add the destination address length */

  ret += mac802154_addr_length[meta->dest_addr.mode];

  /* Add the source address length */

  ret += mac802154_addr_length[ meta->src_addrmode];

  /* If both destination and source addressing information is present, the MAC
   * sublayer shall compare the destination and source PAN identifiers.
   * [1] pg. 41.
   */

  if (meta->src_addrmode  != IEEE802154_ADDRMODE_NONE &&
      meta->dest_addr.mode != IEEE802154_ADDRMODE_NONE)
    {
      /* If the PAN identifiers are identical, the PAN ID Compression field
       * shall be set to one, and the source PAN identifier shall be omitted
       * from the transmitted frame. [1] pg. 41.
       */

      if (meta->dest_addr.panid == priv->addr.panid)
        {
          ret += 2; /* 2 bytes for destination PAN ID */
          return ret;
        }
    }

  /* If we are here, PAN ID compression is off, so include the dest and source
   * PAN ID if the respective address is included
   */

  if (meta->src_addrmode != IEEE802154_ADDRMODE_NONE)
    {
      ret += 2; /* 2 bytes for source PAN ID */
    }

  if (meta->dest_addr.mode != IEEE802154_ADDRMODE_NONE)
    {
      ret += 2; /* 2 bytes for destination PAN ID */
    }

  return ret;
}

/****************************************************************************
 * Name: mac802154_req_data
 *
 * Description:
 *   The MCPS-DATA.request primitive requests the transfer of a data SPDU
 *   (i.e., MSDU) from a local SSCS entity to a single peer SSCS entity.
 *   Confirmation is returned via the
 *   struct mac802154_maccb_s->conf_data callback.
 *
 ****************************************************************************/

int mac802154_req_data(MACHANDLE mac, 
                       FAR const struct ieee802154_frame_meta_s *meta,
                       FAR struct iob_s *frame)
{
  FAR struct ieee802154_privmac_s *priv =
    (FAR struct ieee802154_privmac_s *)mac;
  FAR struct mac802154_txtrans_s trans;
  uint16_t *frame_ctrl;
  uint8_t mhr_len = 3; /* Start assuming frame control and seq. num */
  int ret;

  /* Check the required frame size */

  if (frame->io_len > IEEE802154_MAX_PHY_PACKET_SIZE)
  {
    return -E2BIG;
  }

  /* Cast the first two bytes of the IOB to a uint16_t frame control field */

  frame_ctrl = (FAR uint16_t *)&frame->io_data[0];

  /* Ensure we start with a clear frame control field */

  *frame_ctrl = 0;

  /* Set the frame type to Data */

  *frame_ctrl |= IEEE802154_FRAME_DATA << IEEE802154_FRAMECTRL_SHIFT_FTYPE;

  /* If the msduLength is greater than aMaxMACSafePayloadSize, the MAC
   * sublayer will set the Frame Version to one. [1] pg. 118.
   */

  if ((frame->io_len - frame->io_offset) > IEEE802154_MAX_SAFE_MAC_PAYLOAD_SIZE)
    {
      *frame_ctrl |= IEEE802154_FRAMECTRL_VERSION;
    }

  /* If the TXOptions parameter specifies that an acknowledged transmission
   * is required, the AR field will be set appropriately, as described in
   * 5.1.6.4 [1] pg. 118.
   */

  *frame_ctrl |= (meta->msdu_flags.ack_tx << IEEE802154_FRAMECTRL_SHIFT_ACKREQ);

  /* If the destination address is present, copy the PAN ID and one of the
   * addresses, depending on mode, into the MHR.
   */

  if (meta->dest_addr.mode != IEEE802154_ADDRMODE_NONE)
    {
      memcpy(&frame->io_data[mhr_len], &meta->dest_addr.panid, 2);
      mhr_len += 2;

      if (meta->dest_addr.mode == IEEE802154_ADDRMODE_SHORT)
        {
          memcpy(&frame->io_data[mhr_len], &meta->dest_addr.saddr, 2);
          mhr_len += 2;
        }
      else if (meta->dest_addr.mode == IEEE802154_ADDRMODE_EXTENDED)
        {
          memcpy(&frame->io_data[mhr_len], &meta->dest_addr.eaddr,
                 IEEE802154_EADDR_LEN);

          mhr_len += IEEE802154_EADDR_LEN;
        }
    }

  /* Set the destination addr mode inside the frame control field */

  *frame_ctrl |= (meta->dest_addr.mode << IEEE802154_FRAMECTRL_SHIFT_DADDR);

  /* From this point on, we need exclusive access to the privmac struct */

  ret = mac802154_takesem(&priv->exclsem);
  if (ret < 0)
    {
      wlerr("ERROR: mac802154_takesem failed: %d\n", ret);
      return ret;
    }

  /* If both destination and source addressing information is present, the MAC
   * sublayer shall compare the destination and source PAN identifiers.
   * [1] pg. 41.
   */

  if (meta->src_addrmode  != IEEE802154_ADDRMODE_NONE &&
      meta->dest_addr.mode != IEEE802154_ADDRMODE_NONE)
    {
      /* If the PAN identifiers are identical, the PAN ID Compression field
       * shall be set to one, and the source PAN identifier shall be omitted
       * from the transmitted frame. [1] pg. 41.
       */

      if (meta->dest_addr.panid == priv->addr.panid)
        {
          *frame_ctrl |= IEEE802154_FRAMECTRL_PANIDCOMP;
        }
    }

  if (meta->src_addrmode != IEEE802154_ADDRMODE_NONE)
    {
      /* If the destination address is not included, or if PAN ID Compression
       * is off, we need to include the Source PAN ID.
       */

      if ((meta->dest_addr.mode == IEEE802154_ADDRMODE_NONE) ||
          (!(*frame_ctrl & IEEE802154_FRAMECTRL_PANIDCOMP)))
        {
          memcpy(&frame->io_data[mhr_len], &priv->addr.panid, 2);
          mhr_len += 2;
        }

      if (meta->src_addrmode == IEEE802154_ADDRMODE_SHORT)
        {
          memcpy(&frame->io_data[mhr_len], &priv->addr.saddr, 2);
          mhr_len += 2;
        }
      else if (meta->src_addrmode == IEEE802154_ADDRMODE_EXTENDED)
        {
          memcpy(&frame->io_data[mhr_len], &priv->addr.eaddr,
                 IEEE802154_EADDR_LEN);

          mhr_len += IEEE802154_EADDR_LEN;
        }
    }

  /* Set the source addr mode inside the frame control field */

  *frame_ctrl |= (meta->src_addrmode << IEEE802154_FRAMECTRL_SHIFT_SADDR);

  /* Each time a data or a MAC command frame is generated, the MAC sublayer
   * shall copy the value of macDSN into the Sequence Number field of the MHR
   * of the outgoing frame and then increment it by one. [1] pg. 40.
   */

  frame->io_data[2] = priv->dsn++;

  /* The MAC header we just created must never have exceeded where the app
   * data starts.  This should never happen since the offset should have
   * been set via the same logic to calculate the header length as the logic
   * here that created the header
   */

  DEBUGASSERT(mhr_len == frame->io_offset);

  frame->io_offset = 0; /* Set the offset to 0 to include the header */

  /* Setup our transaction */

  trans.handle = meta->msdu_handle;
  trans.frametype = IEEE802154_FRAME_DATA;
  trans.frame = frame;
  sem_init(&trans.sem, 0, 0);
  sem_setprotocol(&trans.sem, SEM_PRIO_NONE);

  /* If the TxOptions parameter specifies that a GTS transmission is required,
   * the MAC sublayer will determine whether it has a valid GTS as described
   * 5.1.7.3. If a valid GTS could not be found, the MAC sublayer will discard
   * the MSDU. If a valid GTS was found, the MAC sublayer will defer, if
   * necessary, until the GTS. If the TxOptions parameter specifies that a GTS
   * transmission is not required, the MAC sublayer will transmit the MSDU using
   * either slotted CSMA-CA in the CAP for a beacon-enabled PAN or unslotted
   * CSMA-CA for a nonbeacon-enabled PAN. Specifying a GTS transmission in the
   * TxOptions parameter overrides an indirect transmission request.
   * [1] pg. 118.
   */

  if (meta->msdu_flags.gts_tx)
    {
      /* TODO: Support GTS transmission. This should just change where we link
       * the transaction.  Instead of going in the CSMA transaction list, it
       * should be linked to the GTS' transaction list. We'll need to check if
       * the GTS is valid, and then find the GTS, before linking. Note, we also
       * don't have to try and kick-off any transmission here.
       */

      return -ENOTSUP;
    }
  else
    {
      /* If the TxOptions parameter specifies that an indirect transmission is
       * required and this primitive is received by the MAC sublayer of a
       * coordinator, the data frame is sent using indirect transmission, as
       * described in 5.1.5 and 5.1.6.3. [1]
       */

      if (meta->msdu_flags.indirect_tx)
        {
          /* If the TxOptions parameter specifies that an indirect transmission
           * is required and if the device receiving this primitive is not a
           * coordinator, the destination address is not present, or the
           * TxOptions parameter also specifies a GTS transmission, the indirect
           * transmission option will be ignored. [1]
           *
           * NOTE: We don't just ignore the parameter.  Instead, we throw an
           * error, since this really shouldn't be happening.
           */

          if (priv->devmode == IEEE802154_DEVMODE_PANCOORD &&
              meta->dest_addr.mode != IEEE802154_ADDRMODE_NONE)
            {
              /* Link the transaction into the indirect_trans list */

            }
          else
            {
              return -EINVAL;
            }
        }
      else
        {
          /* Link the transaction into the CSMA transaction list */

          sq_addlast((FAR sq_entry_t *)&trans, &priv->csma_queue);

          /* We no longer need to have the MAC layer locked. */

          mac802154_givesem(&priv->exclsem);

          /* Notify the radio driver that there is data available */

          priv->radio->ops->txnotify_csma(priv->radio);

          ret = sem_wait(&trans.sem);
          if (ret < 0)
            {
              return -EINTR;
            }
        }
    }

  sem_destroy(&trans.sem);
  return OK;
}

/****************************************************************************
 * Name: mac802154_req_purge
 *
 * Description:
 *   The MCPS-PURGE.request primitive allows the next higher layer to purge
 *   an MSDU from the transaction queue. Confirmation is returned via
 *   the struct mac802154_maccb_s->conf_purge callback.
 *
 *   NOTE: The standard specifies that confirmation should be indicated via 
 *   the asynchronous MLME-PURGE.confirm primitve.  However, in our
 *   implementation we synchronously return the status from the request.
 *   Therefore, we merge the functionality of the MLME-PURGE.request and 
 *   MLME-PURGE.confirm primitives together.
 *
 ****************************************************************************/

int mac802154_req_purge(MACHANDLE mac, uint8_t msdu_handle)
{
  FAR struct ieee802154_privmac_s *priv =
    (FAR struct ieee802154_privmac_s *)mac;
  return -ENOTTY;
}

/****************************************************************************
 * Name: mac802154_req_associate
 *
 * Description:
 *   The MLME-ASSOCIATE.request primitive allows a device to request an
 *   association with a coordinator. Confirmation is returned via the
 *   struct mac802154_maccb_s->conf_associate callback.
 *
 ****************************************************************************/

int mac802154_req_associate(MACHANDLE mac,
                            FAR struct ieee802154_assoc_req_s *req)
{
  FAR struct ieee802154_privmac_s *priv =
    (FAR struct ieee802154_privmac_s *)mac;
  FAR struct mac802154_txtrans_s trans;
  FAR struct iob_s *iob;
  FAR uint16_t *u16;
  bool rxonidle;
  int ret;

  /* On receipt of the MLME-ASSOCIATE.request primitive, the MLME of an
   * unassociated device first updates the appropriate PHY and MAC PIB
   * attributes, as described in 5.1.3.1, and then generates an association
   * request command, as defined in 5.3.1 [1] pg.80
   */

  /* Get exlusive access to the shared command transaction. This must happen
   * before getting exclusive access to the MAC struct or else there could be
   * a lockup condition. This would occur if another thread is using the cmdtrans
   * but needs access to the MAC in order to unlock it.
   */

  if (sem_wait(&priv->cmd.sem) < 0)
    {
      /* EINTR is the only error that we expect */

      int errcode = get_errno();
      DEBUGASSERT(errcode == EINTR);
      return -errcode;
    }

  /* Get exclusive access to the MAC */
   
   ret = mac802154_takesem(&priv->exclsem);
   if (ret < 0)
     {
       wlerr("ERROR: mac802154_takesem failed: %d\n", ret);
       return ret;
     }

  /* Set the channel and channel page of the PHY layer */

  priv->radio->ops->set_attr(priv->radio, IEEE802154_PIB_PHY_CURRENT_CHANNEL,
                             (FAR const union ieee802154_attr_u *)&req->chnum);

  priv->radio->ops->set_attr(priv->radio, IEEE802154_PIB_PHY_CURRENT_PAGE,
                             (FAR const union ieee802154_attr_u *)&req->chpage);

  /* Set the PANID attribute */

  priv->addr.panid = req->coordaddr.panid;
  priv->coordaddr.panid = req->coordaddr.panid;
  priv->radio->ops->set_attr(priv->radio, IEEE802154_PIB_MAC_PANID,
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
  priv->radio->ops->set_attr(priv->radio, IEEE802154_PIB_MAC_RX_ON_WHEN_IDLE,
                            (FAR const union ieee802154_attr_u *)&rxonidle); 
                          
  /* Allocate an IOB to put the frame in */

  iob = iob_alloc(false);
  DEBUGASSERT(iob != NULL);

  iob->io_flink  = NULL;
  iob->io_len    = 0;
  iob->io_offset = 0;
  iob->io_pktlen = 0;

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

  /* Copy reference to the frame into the shared command transaction */

  priv->cmd.trans.frame = iob;
  priv->cmd.trans.frametype = IEEE802154_FRAME_COMMAND;
  priv->cmd.type = IEEE802154_CMD_ASSOC_REQ;
  
  /* Link the transaction into the CSMA transaction list */

  sq_addlast((FAR sq_entry_t *)&trans, &priv->csma_queue);

  /* We no longer need to have the MAC layer locked. */

  mac802154_givesem(&priv->exclsem);

  /* TODO: Need to setup a timeout here so that we can return an error to the
   * user if the device never receives a response.
   */

  /* Notify the radio driver that there is data available */

  priv->radio->ops->txnotify_csma(priv->radio);

  /* Wait for the transaction to be passed to the radio layer */

  ret = sem_wait(&priv->cmd.trans.sem);
  if (ret < 0)
    {
      return -EINTR;
    }

  return OK;

errout:
  mac802154_givesem(&priv->exclsem);
  return ret;
}

/****************************************************************************
 * Name: mac802154_req_disassociate
 *
 * Description:
 *   The MLME-DISASSOCIATE.request primitive is used by an associated device to
 *   notify the coordinator of its intent to leave the PAN. It is also used by
 *   the coordinator to instruct an associated device to leave the PAN.
 *   Confirmation is returned via the
 *   struct mac802154_maccb_s->conf_disassociate callback.
 *
 ****************************************************************************/

int mac802154_req_disassociate(MACHANDLE mac,
                               FAR struct ieee802154_disassoc_req_s *req)
{
  FAR struct ieee802154_privmac_s *priv =
    (FAR struct ieee802154_privmac_s *)mac;
  return -ENOTTY;
}

/****************************************************************************
 * Name: mac802154_req_gts
 *
 * Description:
 *   The MLME-GTS.request primitive allows a device to send a request to the PAN
 *   coordinator to allocate a new GTS or to deallocate an existing GTS.
 *   Confirmation is returned via the
 *   struct mac802154_maccb_s->conf_gts callback.
 *
 ****************************************************************************/

int mac802154_req_gts(MACHANDLE mac, FAR struct ieee802154_gts_req_s *req)
{
  FAR struct ieee802154_privmac_s *priv =
    (FAR struct ieee802154_privmac_s *)mac;
  return -ENOTTY;
}

/****************************************************************************
 * Name: mac802154_req_reset
 *
 * Description:
 *   The MLME-RESET.request primitive allows the next higher layer to request
 *   that the MLME performs a reset operation.
 *
 *   NOTE: The standard specifies that confirmation should be provided via
 *   via the asynchronous MLME-RESET.confirm primitve.  However, in our
 *   implementation we synchronously return the value immediately. Therefore,
 *   we merge the functionality of the MLME-RESET.request and MLME-RESET.confirm
 *   primitives together.
 *
 * Input Parameters:
 *   mac          - Handle to the MAC layer instance
 *   rst_pibattr  - Whether or not to reset the MAC PIB attributes to defaults
 *
 ****************************************************************************/

int mac802154_req_reset(MACHANDLE mac, bool rst_pibattr)
{
  FAR struct ieee802154_privmac_s * priv =
    (FAR struct ieee802154_privmac_s *) mac;
  return -ENOTTY;
}

/****************************************************************************
 * Name: mac802154_req_rxenable
 *
 * Description:
 *   The MLME-RX-ENABLE.request primitive allows the next higher layer to
 *   request that the receiver is enable for a finite period of time.
 *   Confirmation is returned via the
 *   struct mac802154_maccb_s->conf_rxenable callback.
 *
 ****************************************************************************/

int mac802154_req_rxenable(MACHANDLE mac,
                           FAR struct ieee802154_rxenable_req_s *req)
{
  FAR struct ieee802154_privmac_s * priv =
    (FAR struct ieee802154_privmac_s *)mac;
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
 *   via MULTIPLE calls to the struct mac802154_maccb_s->conf_scan callback.
 *   This is a difference with the official 802.15.4 specification, implemented
 *   here to save memory.
 *
 ****************************************************************************/

int mac802154_req_scan(MACHANDLE mac, FAR struct ieee802154_scan_req_s *req)
{
  FAR struct ieee802154_privmac_s *priv =
    (FAR struct ieee802154_privmac_s *)mac;
  return -ENOTTY;
}

/****************************************************************************
 * Name: mac802154_req_get
 *
 * Description:
 *   The MLME-GET.request primitive requests information about a given PIB
 *   attribute.
 *
 *   NOTE: The standard specifies that the attribute value should be returned
 *   via the asynchronous MLME-GET.confirm primitve.  However, in our
 *   implementation, we synchronously return the value immediately.Therefore, we
 *   merge the functionality of the MLME-GET.request and MLME-GET.confirm
 *   primitives together.
 *
 ****************************************************************************/

int mac802154_req_get(MACHANDLE mac, enum ieee802154_pib_attr_e pib_attr,
                      FAR union ieee802154_attr_u *attrval)
{
  FAR struct ieee802154_privmac_s *priv =
    (FAR struct ieee802154_privmac_s *)mac;
  return -ENOTTY;
}

/****************************************************************************
 * Name: mac802154_req_set
 *
 * Description:
 *   The MLME-SET.request primitive attempts to write the given value to the
 *   indicated MAC PIB attribute. 
 *
 *   NOTE: The standard specifies that confirmation should be indicated via 
 *   the asynchronous MLME-SET.confirm primitve.  However, in our implementation
 *   we synchronously return the status from the request. Therefore, we do merge
 *   the functionality of the MLME-SET.request and MLME-SET.confirm primitives
 *   together.
 *
 ****************************************************************************/

int mac802154_req_set(MACHANDLE mac, enum ieee802154_pib_attr_e pib_attr,
                      FAR const union ieee802154_attr_u *attrval)
{
  FAR struct ieee802154_privmac_s *priv =
    (FAR struct ieee802154_privmac_s *)mac;
  int ret;

  switch (pib_attr)
    {
      case IEEE802154_PIB_MAC_EXTENDED_ADDR:
        {
          /* Set the MAC copy of the address in the table */

          memcpy(&priv->addr.eaddr[0], &attrval->mac.eaddr[0],
                 IEEE802154_EADDR_LEN);

          /* Tell the radio about the attribute */

          priv->radio->ops->set_attr(priv->radio, pib_attr, attrval);
                         
          ret = IEEE802154_STATUS_SUCCESS;
        }
        break;
      default:
        {
          /* The attribute may be handled soley in the radio driver, so pass
           * it along.
           */

          ret = priv->radio->ops->set_attr(priv->radio, pib_attr, attrval);
        }
        break;
    }
  return ret;
}

/****************************************************************************
 * Name: mac802154_req_start
 *
 * Description:
 *   The MLME-START.request primitive makes a request for the device to start
 *   using a new superframe configuration. Confirmation is returned
 *   via the struct mac802154_maccb_s->conf_start callback.
 *
 ****************************************************************************/

int mac802154_req_start(MACHANDLE mac, FAR struct ieee802154_start_req_s *req)
{
  FAR struct ieee802154_privmac_s *priv =
    (FAR struct ieee802154_privmac_s *)mac;
  int ret;

  /* Get exclusive access to the MAC */
  
  ret = mac802154_takesem(&priv->exclsem);
  if (ret < 0)
    {
      wlerr("ERROR: mac802154_takesem failed: %d\n", ret);
      return ret;
    }

  /* When the CoordRealignment parameter is set to TRUE, the coordinator
   * attempts to transmit a coordinator realignment command frame as described
   * in 5.1.2.3.2. If the transmission of the coordinator realignment command
   * fails due to a channel access failure, the MLME will not make any changes
   * to the superframe configuration. (i.e., no PIB attributes will be changed).
   * If the coordinator realignment command is successfully transmitted, the
   * MLME updates the PIB attributes BeaconOrder, SuperframeOrder, PANId,
   * ChannelPage, and ChannelNumber parameters. [1] pg. 106
   */

  if (req->coordrealign)
    {
      /* TODO: Finish the realignment functionality */

      return -ENOTTY;
    }
  
  /* Set the PANID attribute */

  priv->addr.panid = req->panid;
  priv->radio->ops->set_attr(priv->radio, IEEE802154_PIB_MAC_PANID,
                             (FAR const union ieee802154_attr_u *)&req->panid);

  /* Set the radio attributes */
  priv->radio->ops->set_attr(priv->radio, IEEE802154_PIB_PHY_CURRENT_CHANNEL,
                             (FAR const union ieee802154_attr_u *)&req->chnum);

  priv->radio->ops->set_attr(priv->radio, IEEE802154_PIB_PHY_CURRENT_PAGE,
                             (FAR const union ieee802154_attr_u *)&req->chpage);
  
  /* Set the superframe order */

  if(req->superframeorder > 15)
    {
      ret = -EINVAL;
      goto errout;
    }

  priv->superframeorder = req->superframeorder;

  /* Set the beacon order */

  if(req->beaconorder > 15)
    {
      ret = -EINVAL;
      goto errout;
    }

  priv->beaconorder = req->beaconorder;

  if (req->pancoord)
    {
      priv->devmode = IEEE802154_DEVMODE_PANCOORD;
    }
  else
    {
      priv->devmode = IEEE802154_DEVMODE_COORD;
    }

 /* If the BeaconOrder parameter is less than 15, the MLME sets macBattLifeExt to
  * the value of the BatteryLifeExtension parameter. If the BeaconOrder parameter
  * equals 15, the value of the BatteryLifeExtension parameter is ignored. 
  * [1] pg. 106
  */

  if (priv->beaconorder < 15)
    {
      priv->battlifeext = req->battlifeext;

      /* TODO: Finish starting beacon enabled network */
      return -ENOTTY;  
    }

  mac802154_givesem(&priv->exclsem);

  return OK;

errout:
  mac802154_givesem(&priv->exclsem);
  return ret;
}

/****************************************************************************
 * Name: mac802154_req_sync
 *
 * Description:
 *   The MLME-SYNC.request primitive requests to synchronize with the
 *   coordinator by acquiring and, if specified, tracking its beacons.
 *   Confirmation is returned via the
 *   struct mac802154_maccb_s->int_commstatus callback. TOCHECK.
 *
 ****************************************************************************/

int mac802154_req_sync(MACHANDLE mac, FAR struct ieee802154_sync_req_s *req)
{
  FAR struct ieee802154_privmac_s *priv =
    (FAR struct ieee802154_privmac_s *)mac;
  return -ENOTTY;
}

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
  return -ENOTTY;
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
  return -ENOTTY;
}

/****************************************************************************
 * Name: mac802154_resp_orphan
 *
 * Description:
 *   The MLME-ORPHAN.response primitive allows the next higher layer of a
 *   coordinator to respond to the MLME-ORPHAN.indication primitive.
 *
 ****************************************************************************/

int mac802154_resp_orphan(MACHANDLE mac,
                          FAR struct ieee802154_orphan_resp_s *resp)
{
  FAR struct ieee802154_privmac_s *priv =
    (FAR struct ieee802154_privmac_s *)mac;
  return -ENOTTY;
}

/****************************************************************************
 * Name: mac802154_notif_free
 *
 * Description:
 *   When the MAC calls the registered callback, it passes a reference
 *   to a mac802154_notify_s structure.  This structure needs to be freed
 *   after the callback handler is done using it.
 *
 ****************************************************************************/

int mac802154_notif_free(MACHANDLE mac,
                         FAR struct ieee802154_notif_s *notif)
{
  FAR struct ieee802154_privmac_s *priv =
    (FAR struct ieee802154_privmac_s *)mac;

  /* Get exclusive access to the MAC */
  
  while(mac802154_takesem(&priv->exclsem) < 0);
  
  notif->flink = priv->notif_free;
  priv->notif_free = notif;

  mac802154_givesem(&priv->exclsem);

  if (priv->csma_tryagain)
    {
      priv->csma_tryagain = false;
      priv->radio->ops->txnotify_csma(priv->radio);
    }

  if (priv->gts_tryagain)
    {
      priv->gts_tryagain = false;
      priv->radio->ops->txnotify_gts(priv->radio);
    }

  return -ENOTTY;
}
