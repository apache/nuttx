/****************************************************************************
 * wireless/ieee802154/mac802154_internal.h
 *
 *   Copyright (C) 2016 Sebastien Lorquet. All rights reserved.
 *   Copyright (C) 2017 Verge Inc. All rights reserved.
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *
 *   Author: Sebastien Lorquet <sebastien@lorquet.fr>
 *   Author: Anthony Merlino <anthony@vergeaero.com>
 *
 *   The naming and comments for various fields are taken directly
 *   from the IEEE 802.15.4 2011 standard.
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

#ifndef __WIRELESS_IEEE802154__MAC802154_INTERNAL_H
#define __WIRELESS_IEEE802154__MAC802154_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdlib.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <string.h>

#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>

#include "mac802154_notif.h"

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

#if !defined(CONFIG_IEEE802154_DEFAULT_EADDR)
#define CONFIG_IEEE802154_DEFAULT_EADDR 0xFFFFFFFFFFFFFFFF
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Map between ieee802154_addrmode_e enum and actual address length */

static const uint8_t mac802154_addr_length[4] = {0, 0, 2, 8};

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct mac802154_radiocb_s
{
  struct ieee802154_radiocb_s cb;
  FAR struct ieee802154_privmac_s *priv;
};

/* Enumeration for representing what operation the MAC layer is currently doing.
 * There can only be one command being handled at any given time, but certain
 * operations such as association requires more than one command to be sent.
 * Therefore, the need to track not only what command is currently active, but
 * also, what overall operation the command is apart of is necessary.
 */

enum mac802154_operation_e
{
  MAC802154_OP_NONE,
  MAC802154_OP_ASSOC,
  MAC802154_OP_POLL
};

struct ieee802154_privmac_s; /* Forward Reference */
typedef void (*mac802154_worker_t)(FAR struct ieee802154_privmac_s *priv);

/* The privmac structure holds the internal state of the MAC and is the
 * underlying represention of the opaque MACHANDLE.  It contains storage for
 * the IEEE802.15.4 MIB attributes.
 */

struct ieee802154_privmac_s
{
  FAR struct ieee802154_radio_s *radio;     /* Contained IEEE802.15.4 radio dev */
  FAR struct mac802154_maccb_s *cb;         /* Head of a list of MAC callbacks */
  FAR struct mac802154_radiocb_s radiocb;   /* Interface to bind to radio */

  sem_t exclsem;                            /* Support exclusive access */
  uint8_t nclients;                         /* Number of notification clients */
  uint8_t nnotif;                           /* Number of remaining notifications */

  /* Only support a single command at any given time. As of now I see no
   * condition where you need to have more than one command frame simultaneously
   */

  sem_t                       op_sem;       /* Exclusive operations */
  enum mac802154_operation_e  curr_op;      /* The current overall operation */
  enum ieee802154_cmdid_e     curr_cmd;     /* Type of the current cmd */
  FAR struct ieee802154_txdesc_s *cmd_desc; /* TX descriptor for current cmd */

  /* Pre-allocated notifications to be passed to the registered callback.  These
   * need to be freed by the application using mac802154_xxxxnotif_free when
   * the callee layer is finished with it's use.
   */

  FAR struct mac802154_notif_s *notif_free;
  struct mac802154_notif_s notif_pool[CONFIG_MAC802154_NNOTIF];
  sem_t notif_sem;

  struct ieee802154_txdesc_s txdesc_pool[CONFIG_IEEE802154_NTXDESC];
  sem_t txdesc_sem;
  sq_queue_t txdesc_queue;
  sq_queue_t txdone_queue;


  /* Support a singly linked list of transactions that will be sent using the
   * CSMA algorithm.  On a non-beacon enabled PAN, these transactions will be
   * sent whenever. On a beacon-enabled PAN, these transactions will be sent
   * during the CAP of the Coordinator's superframe.
   */

  sq_queue_t csma_queue;
  sq_queue_t gts_queue;

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

  /* Work structures for offloading aynchronous work */

  struct work_s tx_work;
  struct work_s rx_work;

  struct work_s timeout_work;
  WDOG_ID       timeout;  /* Timeout watchdog */
  mac802154_worker_t timeout_worker;

  struct work_s purge_work;

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

  uint8_t battlifeext_periods;      /* # of backoff periods during which rx is
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

  uint32_t trackingbeacon     : 1;  /* Are we tracking the beacon */
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

                                    /* 2 available bits */

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

  uint32_t max_csmabackoffs   : 3;  /* Max num backoffs for CSMA algorithm
                                     * before declaring ch access failure */

                                    /* 9-bits remaining */

  /* End of 32-bit bitfield. */

  /* TODO: Add Security-related MAC PIB attributes */
};

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#define mac802154_givesem(s) sem_post(s);

static inline int mac802154_takesem(sem_t *sem, bool allowinterrupt)
{
  int ret;
  do
    {
      /* Take a count from the semaphore, possibly waiting */

      ret = sem_wait(sem);
      if (ret < 0)
        {
          /* EINTR is the only error that we expect */

          DEBUGASSERT(get_errno() == EINTR);

          if (allowinterrupt)
            {
              return -EINTR;
            }
        }
    }
  while (ret != OK);

  return OK;
}

static inline void mac802154_txdesc_free(FAR struct ieee802154_privmac_s *priv,
                                         FAR struct ieee802154_txdesc_s *txdesc)
{
  sq_addlast((FAR sq_entry_t *)txdesc, &priv->txdesc_queue);
  mac802154_givesem(&priv->txdesc_sem);
}

/****************************************************************************
 * Name: mac802154_timercancel
 *
 * Description:
 *   Cancel timer and remove reference to callback function
 *
 * Assumptions:
 *   priv MAC struct is locked when calling.
 *
 ****************************************************************************/

static inline int mac802154_timercancel(FAR struct ieee802154_privmac_s *priv)
{
  wd_cancel(priv->timeout);
  priv->timeout_worker = NULL;
  return OK;
}

/****************************************************************************
 * Function Prototypes
 ****************************************************************************/


int mac802154_txdesc_alloc(FAR struct ieee802154_privmac_s *priv,
                                  FAR struct ieee802154_txdesc_s **txdesc,
                                  bool allow_interrupt);

int mac802154_timerstart(FAR struct ieee802154_privmac_s *priv,
                         uint32_t numsymbols, mac802154_worker_t);

void mac802154_setupindirect(FAR struct ieee802154_privmac_s *priv,
                             FAR struct ieee802154_txdesc_s *txdesc);

void mac802154_create_datareq(FAR struct ieee802154_privmac_s *priv,
                              FAR struct ieee802154_addr_s *coordaddr,
                              enum ieee802154_addrmode_e srcmode,
                              FAR struct ieee802154_txdesc_s *txdesc);

#endif /* __WIRELESS_IEEE802154__MAC802154_INTERNAL_H */
