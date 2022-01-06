/****************************************************************************
 * wireless/ieee802154/mac802154_internal.h
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
#include <nuttx/semaphore.h>

#include <nuttx/wireless/ieee802154/ieee802154_mac.h>
#include <nuttx/wireless/ieee802154/ieee802154_radio.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* If processing is not done at the interrupt level, then work queue support
 * is required.
 */

#if !defined(CONFIG_SCHED_HPWORK) || !defined(CONFIG_SCHED_LPWORK)
#  error Both Low and High priority work queues are required for this driver
#endif

#if !defined(CONFIG_MAC802154_NTXDESC) || CONFIG_MAC802154_NTXDESC <= 0
#  undef CONFIG_MAC802154_NTXDESC
#  define CONFIG_MAC802154_NTXDESC 5
#endif

#if !defined(CONFIG_IEEE802154_DEFAULT_EADDR)
#  define CONFIG_IEEE802154_DEFAULT_EADDR 0xFFFFFFFFFFFFFFFF
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct mac802154_radiocb_s
{
  struct ieee802154_radiocb_s cb;
  FAR struct ieee802154_privmac_s *priv;
};

/* Enumeration for representing what operation the MAC layer is currently
 * doing. There can only be one command being handled at any given time, but
 * certain operations such as association requires more than one command to
 * be sent. Therefore, the need to track not only what command is currently
 * active, but also, what overall operation the command is apart of is
 * necessary.
 */

enum mac802154_operation_e
{
  MAC802154_OP_NONE,
  MAC802154_OP_ASSOC,
  MAC802154_OP_POLL,
  MAC802154_OP_SCAN,
  MAC802154_OP_AUTOEXTRACT,
  MAC802154_OP_RXENABLE,
};

/* The privmac structure holds the internal state of the MAC and is the
 * underlying represention of the opaque MACHANDLE.  It contains storage for
 * the IEEE802.15.4 MIB attributes.
 */

struct ieee802154_privmac_s
{
  /*************************** General Fields *******************************/

  FAR struct ieee802154_radio_s  *radio;    /* Contained IEEE802.15.4 radio dev */
  FAR struct mac802154_maccb_s   *cb;       /* Head of a list of MAC callbacks */
  FAR struct mac802154_radiocb_s radiocb;   /* Interface to bind to radio */

  sem_t exclsem;                            /* Support exclusive access */
  uint8_t nclients;                         /* Number of notification clients */

  /* Only support a single command at any given time. As of now I see no
   * condition where you need to have more than one command frame
   * simultaneously
   */

  sem_t                          opsem;     /* Exclusive operations */

  /******************* Fields related to MAC operations *********************/

  enum mac802154_operation_e     curr_op;   /* The current overall operation */
  enum ieee802154_cmdid_e        curr_cmd;  /* Type of the current cmd */
  FAR struct ieee802154_txdesc_s *cmd_desc; /* TX descriptor for current cmd */
  uint8_t                        nrxusers;
  struct work_s                  macop_work;

  /******************* Fields related to SCAN operation *********************/

  uint8_t scanindex;
  uint8_t edlist[15];
  uint8_t npandesc;
  struct ieee802154_pandesc_s pandescs[MAC802154_NPANDESC];
  uint8_t panidbeforescan[IEEE802154_PANIDSIZE];
  struct ieee802154_scan_req_s currscan;
  uint32_t scansymdur;

  /******************* Fields related to notifications **********************/

  sq_queue_t primitive_queue;     /* Queue of primitives to pass via notify()
                                   * callback to registered receivers
                                   */
  struct work_s notifwork;        /* For deferring notifications to LPWORK queue */

  /******************* Tx descriptor queues and pools ***********************/

  struct ieee802154_txdesc_s txdesc_pool[CONFIG_MAC802154_NTXDESC];
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

  /* Support a singly linked list of transactions that will be sent
   * indirectly. This list should only be used by a MAC acting as a
   * coordinator.  These transactions will stay here until the data
   * is extracted by the destination device sending a Data Request
   * MAC command or if too much time passes. This list should also
   * be used to populate the address list of the outgoing beacon
   * frame.
   */

  sq_queue_t indirect_queue;

  /* Support a singly linked list of frames received */

  sq_queue_t dataind_queue;

  /************* Fields related to addressing and coordinator ***************/

  /* Holds all address information (Extended, Short, and PAN ID) for MAC */

  struct ieee802154_addr_s addr;

  struct ieee802154_pandesc_s pandesc;

  /*************** Fields related to beacon-enabled networks ****************/

  /* Holds attributes pertaining to the superframe specification */

  struct ieee802154_superframespec_s sfspec;

  /* We use 2 beacon frame structures so that we can ping-pong between them
   * while updating the beacon
   */

  struct ieee802154_beaconframe_s beaconframe[2];

  /* Contents of beacon payload */

  uint8_t beaconpayload[IEEE802154_MAX_BEACON_PAYLOAD_LEN];
  uint8_t beaconpayloadlength;

  /****************** Fields related to offloading work *********************/

  /* Work structures for offloading aynchronous work */

  struct work_s txdone_work;
  struct work_s rx_work;
  struct work_s purge_work;
  struct work_s timer_work;

  /****************** Uncategorized MAC PIB attributes **********************/

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

  uint8_t dsn;                      /* Seq. num added to tx data or MAC frame */

  /* The maximum time, in multiples of aBaseSuperframeDuration, a device
   * shall wait for a response command frame to be available following a
   * request command frame. [1] 128.
   */

  uint8_t resp_waittime;

  /* The total transmit duration (including PHY header and FCS) specified in
   * symbols. [1] pg. 129.
   */

  uint32_t tx_totaldur;

  /* Start of 8-bit bitfield */

  uint32_t trackingbeacon     : 1;  /* Are we tracking the beacon */
  uint32_t isassoc            : 1;  /* Are we associated to the PAN */
  uint32_t autoreq            : 1;  /* Automatically send data req. if addr
                                     * addr is in the beacon frame */

  uint32_t gtspermit          : 1;  /* Is PAN Coord. accepting GTS reqs. */
  uint32_t promisc            : 1;  /* Is promiscuous mode on? */
  uint32_t rngsupport         : 1;  /* Does MAC sublayer support ranging */
  uint32_t sec_enabled        : 1;  /* Does MAC sublayer have security en. */
  uint32_t timestamp_support  : 1;  /* Does MAC layer supports timestamping */

  /* End of 8-bit bitfield */

  /* Start of 32-bit bitfield */

  /* The offset, measured is symbols, between the symbol boundary at which
   * MLME captures the timestamp of each transmitted and received frame, and
   * the onset of the first symbol past the SFD, namely the first symbol of
   * the frames [1] pg. 129.
   */

  uint32_t sync_symboffset    : 12;

  uint32_t txctrl_activedur   : 17; /* Duration for which tx is permitted to
                                       * be active */
  uint32_t txctrl_pausedur    : 1;  /* Duration after tx before another tx is
                                     * permitted. 0=2000, 1= 10000 */

  /* What type of device is this node acting as */

  enum ieee802154_devmode_e devmode : 2;

  /* End of 32-bit bitfield */

  /* Start of 32-bit bitfield */

  uint32_t beacon_txtime      : 24; /* Time of last beacon transmit */
  uint32_t minbe              : 4;  /* Min value of backoff exponent (BE) */
  uint32_t maxbe              : 4;  /* Max value of backoff exponent (BE) */

  /* End of 32-bit bitfield */

  /* Start of 8-bit bitfield */

  uint8_t bf_ind              : 1;  /* Ping-pong index for beacon frame */
  uint8_t beaconupdate        : 1;  /* Does the beacon frame need to be updated */
  uint8_t max_csmabackoffs    : 3;  /* Max num backoffs for CSMA algorithm
                                     * before declaring ch access failure */
  uint8_t maxretries          : 3;  /* Max # of retries allowed after tx fail */

  /* End of 8-bit bitfield. */

  /* Start of 8-bit bitfield */

  uint8_t rxonidle            : 1;  /* Receiver on when idle? */

  /* End of 8-bit bitfield. */

  /* TODO: Add Security-related MAC PIB attributes */
};

/****************************************************************************
 * Function Prototypes
 ****************************************************************************/

int  mac802154_txdesc_alloc(FAR struct ieee802154_privmac_s *priv,
      FAR struct ieee802154_txdesc_s **txdesc, bool allow_interrupt);

void mac802154_setupindirect(FAR struct ieee802154_privmac_s *priv,
      FAR struct ieee802154_txdesc_s *txdesc);

void mac802154_createdatareq(FAR struct ieee802154_privmac_s *priv,
      FAR struct ieee802154_addr_s *coordaddr,
      enum ieee802154_addrmode_e srcmode,
      FAR struct ieee802154_txdesc_s *txdesc);

void mac802154_updatebeacon(FAR struct ieee802154_privmac_s *priv);

void mac802154_notify(FAR struct ieee802154_privmac_s *priv,
      FAR struct ieee802154_primitive_s *primitive);

/****************************************************************************
 * Helper Macros/Inline Functions
 ****************************************************************************/

#define mac802154_putpanid(iob, panid) \
  do \
    { \
      IEEE802154_PANIDCOPY(&iob->io_data[iob->io_len], panid); \
      iob->io_len += IEEE802154_PANIDSIZE; \
    } \
  while(0)

#define mac802154_putsaddr(iob, saddr) \
  do \
    { \
      IEEE802154_SADDRCOPY(&iob->io_data[iob->io_len], saddr); \
      iob->io_len += IEEE802154_SADDRSIZE; \
    } \
  while(0)

/* The IEEE 802.15.4 Standard is confusing with regards to byte-order for
 * extended address. More research discovers that the extended address should
 * be sent in reverse-canonical form.
 */

#define mac802154_puteaddr(iob, eaddr) \
  do \
    { \
      for (int index = IEEE802154_EADDRSIZE - 1; index >= 0; index--) \
        { \
          iob->io_data[iob->io_len++] = eaddr[index]; \
        } \
    } \
  while(0)

#define mac802154_takepanid(iob, panid) \
  do \
    { \
      IEEE802154_PANIDCOPY(panid, &iob->io_data[iob->io_offset]); \
      iob->io_offset += IEEE802154_PANIDSIZE; \
    } \
  while(0)

#define mac802154_takesaddr(iob, saddr) \
  do \
    { \
      IEEE802154_SADDRCOPY(saddr, &iob->io_data[iob->io_offset]); \
      iob->io_offset += IEEE802154_SADDRSIZE; \
    } \
  while(0)

/* The IEEE 802.15.4 Standard is confusing with regards to byte-order for
 * extended address. More research discovers that the extended address should
 * be sent in reverse-canonical form.
 */

#define mac802154_takeeaddr(iob, eaddr) \
  do \
    { \
      for (int index = IEEE802154_EADDRSIZE - 1; index >= 0; index--) \
        { \
          eaddr[index] = iob->io_data[iob->io_offset++]; \
        } \
    } \
  while(0)

/* General helper macros ****************************************************/

/* GET 16-bit data:  source in network order, result in host order */

#define GETHOST16(ptr,index) \
  ((((uint16_t)((ptr)[(index) + 1])) << 8) | ((uint16_t)(((ptr)[index]))))

/* GET 16-bit data:  source in network order, result in network order */

#define GETNET16(ptr,index) \
  ((((uint16_t)((ptr)[index])) << 8) | ((uint16_t)(((ptr)[(index) + 1]))))

/* PUT 16-bit data:  source in host order, result in network order */

#define PUTHOST16(ptr,index,value) \
  do \
    { \
      (ptr)[index]      = (uint16_t)(value) & 0xff; \
      (ptr)[index + 1]  = ((uint16_t)(value) >> 8) & 0xff; \
    } \
  while(0)

/* Set bit in 16-bit value:  source in host order, result in network order. */

#define IEEE802154_SETBITS_U16(ptr,index,value) \
  do \
    { \
      (ptr)[index]     |= (uint16_t)(value) & 0xff; \
      (ptr)[index + 1] |= ((uint16_t)(value) >> 8) & 0xff; \
    } \
  while(0)

/* Helper macros for setting/receiving bits for frame control field */

#define IEEE802154_SETFTYPE(ptr, index, ftype) \
  IEEE802154_SETBITS_U16(ptr, index, (ftype << IEEE802154_FRAMECTRL_SHIFT_FTYPE))

#define IEEE802154_SETACKREQ(ptr, index) \
  IEEE802154_SETBITS_U16(ptr, index, IEEE802154_FRAMECTRL_ACKREQ)

#define IEEE802154_SETDADDRMODE(ptr, index, mode) \
  IEEE802154_SETBITS_U16(ptr, index, (mode << IEEE802154_FRAMECTRL_SHIFT_DADDR))

#define IEEE802154_SETSADDRMODE(ptr, index, mode) \
  IEEE802154_SETBITS_U16(ptr, index, (mode << IEEE802154_FRAMECTRL_SHIFT_SADDR))

#define IEEE802154_SETPANIDCOMP(ptr, index) \
  IEEE802154_SETBITS_U16(ptr, index, IEEE802154_FRAMECTRL_PANIDCOMP)

#define IEEE802154_SETVERSION(ptr, index, version) \
  IEEE802154_SETBITS_U16(ptr, index, (version << IEEE802154_FRAMECTRL_SHIFT_VERSION))

/* Helper macros for setting/receiving bits for superframe specification */

#define IEEE802154_SETBEACONORDER(ptr, index, val) \
  IEEE802154_SETBITS_U16(ptr, index, (val << IEEE802154_SFSPEC_SHIFT_BEACONORDER))

#define IEEE802154_SETSFORDER(ptr, index, val) \
  IEEE802154_SETBITS_U16(ptr, index, (val << IEEE802154_SFSPEC_SHIFT_SFORDER))

#define IEEE802154_SETFINCAPSLOT(ptr, index, val) \
  IEEE802154_SETBITS_U16(ptr, index, (val << IEEE802154_SFSPEC_SHIFT_FINCAPSLOT))

#define IEEE802154_SETBLE(ptr, index) \
  IEEE802154_SETBITS_U16(ptr, index, IEEE802154_SFSPEC_BLE)

#define IEEE802154_SETPANCOORD(ptr, index) \
  IEEE802154_SETBITS_U16(ptr, index, IEEE802154_SFSPEC_PANCOORD)

#define IEEE802154_SETASSOCPERMIT(ptr, index) \
  IEEE802154_SETBITS_U16(ptr, index, IEEE802154_SFSPEC_ASSOCPERMIT)

#define IEEE802154_GETBEACONORDER(ptr, index) \
  ((GETHOST16(ptr, index) & IEEE802154_SFSPEC_BEACONORDER) >> \
  IEEE802154_SFSPEC_SHIFT_BEACONORDER)

#define IEEE802154_GETSFORDER(ptr, index) \
  ((GETHOST16(ptr, index) & IEEE802154_SFSPEC_SFORDER) >> \
  IEEE802154_SFSPEC_SHIFT_SFORDER)

#define IEEE802154_GETFINCAPSLOT(ptr, index) \
  ((GETHOST16(ptr, index) & IEEE802154_SFSPEC_FINCAPSLOT) >> \
  IEEE802154_SFSPEC_SHIFT_FINCAPSLOT)

#define IEEE802154_GETBLE(ptr, index) \
  ((GETHOST16(ptr, index) & IEEE802154_SFSPEC_BLE) >> \
  IEEE802154_SFSPEC_SHIFT_BLE)

#define IEEE802154_GETPANCOORD(ptr, index) \
  ((GETHOST16(ptr, index) & IEEE802154_SFSPEC_PANCOORD) >> \
  IEEE802154_SFSPEC_SHIFT_PANCOORD)

#define IEEE802154_GETASSOCPERMIT(ptr, index) \
  ((GETHOST16(ptr, index) & IEEE802154_SFSPEC_ASSOCPERMIT) >> \
  IEEE802154_SFSPEC_SHIFT_ASSOCPERMIT)

/* Helper macros for setting/receiving bits for GTS specification */

#define IEEE802154_GETGTSDESCCOUNT(ptr, index) \
  ((GETHOST16(ptr, index) & IEEE802154_GTSSPEC_DESCCOUNT) >> \
  IEEE802154_GTSSPEC_SHIFT_DESCCOUNT)

#define IEEE802154_GETGTSPERMIT(ptr, index) \
  ((GETHOST16(ptr, index) & IEEE802154_GTSSPEC_PERMIT) >> \
  IEEE802154_GTSSPEC_SHIFT_PERMIT)

/* Helper macros for setting/receiving bits for GTS Directions */

#define IEEE802154_GETGTSDIRMASK(ptr, index) \
  ((GETHOST16(ptr, index) & IEEE802154_GTSDIR_MASK) >> \
  IEEE802154_GTSDIR_SHIFT_MASK)

/* Helper macros for setting/receiving bits for Pending Address */

#define IEEE802154_GETNPENDSADDR(ptr, index) \
  ((GETHOST16(ptr, index) & IEEE802154_PENDADDR_NSADDR) >> \
  IEEE802154_PENDADDR_SHIFT_NSADDR)

#define IEEE802154_GETNPENDEADDR(ptr, index) \
  ((GETHOST16(ptr, index) & IEEE802154_PENDADDR_NEADDR) >> \
  IEEE802154_PENDADDR_SHIFT_NEADDR)

/* General helpers **********************************************************/

#define mac802154_givesem(s) nxsem_post(s)

static inline int mac802154_takesem(sem_t *sem, bool allowinterrupt)
{
  if (allowinterrupt)
    {
      return nxsem_wait(sem);
    }
  else
    {
      return nxsem_wait_uninterruptible(sem);
    }
}

#ifdef CONFIG_MAC802154_LOCK_VERBOSE
#define mac802154_unlock(dev) \
  mac802154_givesem(&dev->exclsem); \
  wlinfo("MAC unlocked\n");
#else
#define mac802154_unlock(dev) \
  mac802154_givesem(&dev->exclsem);
#endif

#define mac802154_lock(dev, allowinterrupt) \
  mac802154_lockpriv(dev, allowinterrupt, __FUNCTION__)

static inline int
mac802154_lockpriv(FAR struct ieee802154_privmac_s *dev,
                   bool allowinterrupt, FAR const char *funcname)
{
  int ret;

#ifdef CONFIG_MAC802154_LOCK_VERBOSE
  wlinfo("Locking MAC: %s\n", funcname);
#endif
  ret = mac802154_takesem(&dev->exclsem, allowinterrupt);
  if (ret < 0)
    {
      wlwarn("Failed to lock MAC\n");
    }
  else
    {
#ifdef CONFIG_MAC802154_LOCK_VERBOSE
      wlinfo("MAC locked\n");
#endif
    }

  return ret;
}

static inline void
mac802154_txdesc_free(FAR struct ieee802154_privmac_s *priv,
                      FAR struct ieee802154_txdesc_s *txdesc)
{
  sq_addlast((FAR sq_entry_t *)txdesc, &priv->txdesc_queue);
  mac802154_givesem(&priv->txdesc_sem);
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

static inline uint32_t
mac802154_symtoticks(FAR struct ieee802154_privmac_s *priv, uint32_t symbols)
{
  union ieee802154_attr_u attrval;
  uint32_t ret;

  /* First, get the symbol duration from the radio layer.  Symbol duration is
   * returned in picoseconds to ensure precision is kept when multiplying to
   * get overall times.
   */

  priv->radio->getattr(priv->radio, IEEE802154_ATTR_PHY_SYMBOL_DURATION,
                        &attrval);

  /* After this step, ret represents microseconds */

  ret = ((uint64_t)attrval.phy.symdur_picosec * symbols) / (1000 * 1000);

  /* This method should only be used for things that can be late. For
   * instance, it's always okay to wait a little longer before disabling
   * your receiver. Therefore, we force the tick count to round up.
   */

  if (ret % USEC_PER_TICK == 0)
    {
      ret = ret / USEC_PER_TICK;
    }
  else
    {
      ret = ret / USEC_PER_TICK;
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

static inline void
mac802154_timerstart(FAR struct ieee802154_privmac_s *priv,
                     uint32_t numsymbols, worker_t worker)
{
  DEBUGASSERT(work_available(&priv->timer_work));

  /* Schedule the work
   * converting the number of symbols to the number of CPU ticks
   */

  work_queue(HPWORK, &priv->timer_work, worker, priv,
             mac802154_symtoticks(priv, numsymbols));
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

static inline int
mac802154_timercancel(FAR struct ieee802154_privmac_s *priv)
{
  work_cancel(HPWORK, &priv->timer_work);
  wlinfo("Timer cancelled\n");
  return OK;
}

static inline void
mac802154_rxenable(FAR struct ieee802154_privmac_s *priv)
{
  priv->nrxusers++;

  /* If this is the first user, actually enable the receiver */

  if (priv->nrxusers == 1)
    {
      wlinfo("Receiver enabled\n");
      priv->radio->rxenable(priv->radio, true);
    }
}

static inline void
mac802154_rxdisable(FAR struct ieee802154_privmac_s *priv)
{
  priv->nrxusers--;

  /* If this is the first user, actually enable the receiver */

  if (priv->nrxusers == 0)
    {
      wlinfo("Receiver disabled\n");
      priv->radio->rxenable(priv->radio, false);
    }
}

static inline void
mac802154_setchannel(FAR struct ieee802154_privmac_s *priv,
                     uint8_t channel)
{
  priv->radio->setattr(priv->radio, IEEE802154_ATTR_PHY_CHAN,
                        (FAR const union ieee802154_attr_u *)&channel);
}

static inline void
mac802154_setchpage(FAR struct ieee802154_privmac_s *priv,
                    uint8_t chpage)
{
  priv->radio->setattr(priv->radio, IEEE802154_ATTR_PHY_CURRENT_PAGE,
                        (FAR const union ieee802154_attr_u *)&chpage);
}

static inline void
mac802154_setpanid(FAR struct ieee802154_privmac_s *priv,
                   FAR const uint8_t *panid)
{
  IEEE802154_PANIDCOPY(priv->addr.panid, panid);
  priv->radio->setattr(priv->radio, IEEE802154_ATTR_MAC_PANID,
                        (FAR const union ieee802154_attr_u *)panid);
}

static inline void
mac802154_setsaddr(FAR struct ieee802154_privmac_s *priv,
                   FAR const uint8_t *saddr)
{
  IEEE802154_SADDRCOPY(priv->addr.saddr, saddr);
  priv->radio->setattr(priv->radio, IEEE802154_ATTR_MAC_SADDR,
                        (FAR const union ieee802154_attr_u *)saddr);
}

static inline void
mac802154_setcoordsaddr(FAR struct ieee802154_privmac_s *priv,
                        FAR const uint8_t *saddr)
{
  IEEE802154_SADDRCOPY(priv->pandesc.coordaddr.saddr, saddr);
  priv->radio->setattr(priv->radio, IEEE802154_ATTR_MAC_COORD_SADDR,
                        (FAR const union ieee802154_attr_u *)saddr);
}

static inline void
mac802154_setcoordeaddr(FAR struct ieee802154_privmac_s *priv,
                        FAR const uint8_t *eaddr)
{
  IEEE802154_EADDRCOPY(priv->pandesc.coordaddr.eaddr, eaddr);
  priv->radio->setattr(priv->radio, IEEE802154_ATTR_MAC_COORD_EADDR,
                        (FAR const union ieee802154_attr_u *)eaddr);
}

static inline void
mac802154_setcoordaddr(FAR struct ieee802154_privmac_s *priv,
                       FAR const struct ieee802154_addr_s *addr)
{
  memcpy(&priv->pandesc.coordaddr, addr, sizeof(struct ieee802154_addr_s));
  priv->radio->setattr(priv->radio, IEEE802154_ATTR_MAC_COORD_EADDR,
                        (FAR const union ieee802154_attr_u *)addr->eaddr);
  priv->radio->setattr(priv->radio, IEEE802154_ATTR_MAC_COORD_SADDR,
                        (FAR const union ieee802154_attr_u *)addr->saddr);
}

static inline void
mac802154_setrxonidle(FAR struct ieee802154_privmac_s *priv, bool rxonidle)
{
  priv->rxonidle = rxonidle;
  if (priv->rxonidle)
    {
      mac802154_rxenable(priv);
    }
  else
    {
      mac802154_rxdisable(priv);
    }

  priv->radio->setattr(priv->radio, IEEE802154_ATTR_MAC_RX_ON_WHEN_IDLE,
                        (FAR const union ieee802154_attr_u *)&rxonidle);
}

static inline void
mac802154_setdevmode(FAR struct ieee802154_privmac_s *priv, uint8_t mode)
{
  priv->devmode = mode;
  priv->radio->setattr(priv->radio, IEEE802154_ATTR_MAC_DEVMODE,
                        (FAR const union ieee802154_attr_u *)&mode);
}

#endif /* __WIRELESS_IEEE802154__MAC802154_INTERNAL_H */
