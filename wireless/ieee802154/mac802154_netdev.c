/****************************************************************************
 * wireless/ieee802154/mac802154_netdev.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <arpa/inet.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>
#include <nuttx/net/iob.h>
#include <nuttx/net/arp.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/sixlowpan.h>
#include <nuttx/wireless/ieee80154_device.h>
#include <nuttx/wireless/ieee80154_mac.h>

#ifdef CONFIG_NET_6LOWPAN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* If processing is not done at the interrupt level, then work queue support
 * is required.
 */

#if !defined(CONFIG_SCHED_WORKQUEUE)
#  error Work queue support is required in this configuration (CONFIG_SCHED_WORKQUEUE)
#else

   /* Use the selected work queue */

#  if defined(CONFIG_IEEE802154_NETDEV_HPWORK)
#    define ETHWORK HPWORK
#  elif defined(CONFIG_IEEE802154_NETDEV_LPWORK)
#    define ETHWORK LPWORK
#  else
#    error Neither CONFIG_IEEE802154_NETDEV_HPWORK nor CONFIG_IEEE802154_NETDEV_LPWORK defined
#  endif
#endif

/* CONFIG_IEEE802154_NETDEV_NINTERFACES determines the number of physical interfaces
 * that will be supported.
 */

#ifndef CONFIG_IEEE802154_NETDEV_NINTERFACES
# define CONFIG_IEEE802154_NETDEV_NINTERFACES 1
#endif

/* TX poll delay = 1 seconds. CLK_TCK is the number of clock ticks per second */

#define skeleton_WDDELAY   (1*CLK_TCK)

/* TX timeout = 1 minute */

#define skeleton_TXTIMEOUT (60*CLK_TCK)

/* This is a helper pointer for accessing the contents of the Ethernet header */

#define BUF ((struct eth_hdr_s *)priv->m8_dev.d_buf)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The mac802154_driver_s encapsulates all state information for a single
 * IEEE802.15.4 MAC interface.
 */

struct mac802154_driver_s
{
  /* This holds the information visible to the NuttX network */

  struct ieee802154_driver_s m8_dev;   /* Interface understood by the network */
  FAR struct ieee802154_mac_s *m8_mac; /* Contained MAC interface */

  /* For internal use by this driver */

  bool m8_bifup;               /* true:ifup false:ifdown */
  WDOG_ID m8_txpoll;           /* TX poll timer */
  WDOG_ID m8_txtimeout;        /* TX timeout timer */
  struct work_s m8_irqwork;    /* For deferring interupt work to the work queue */
  struct work_s m8_pollwork;   /* For deferring poll work to the work queue */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* A single packet buffer per device is used here.  There might be multiple
 * packet buffers in a more complex, pipelined design.
 */

static uint8_t g_pktbuf[MAX_NET_DEV_MTU + CONFIG_NET_GUARDSIZE];

/* Driver state structure */

static struct mac802154_driver_s g_skel[CONFIG_IEEE802154_NETDEV_NINTERFACES];

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* IEE802.15.4 MAC callback functions ***************************************/
/* Asynchronous confirmations to requests */

static void mac802154_conf_data(FAR struct ieee802154_mac_s *mac,
             FAR struct ieee802154_data_conf_s *conf);
static void mac802154_conf_purge(FAR struct ieee802154_mac_s *mac,
             uint8_t handle, int status);
static void mac802154_conf_associate(FAR struct ieee802154_mac_s *mac,
             uint16_t saddr, int status);
static void mac802154_conf_disassociate(FAR struct ieee802154_mac_s *mac,
static void mac802154_conf_get(FAR struct ieee802154_mac_s *mac, int status,
             int attribute, FAR uint8_t *value, int valuelen);
static void mac802154_conf_gts(FAR struct ieee802154_mac_s *mac,
             FAR uint8_t *characteristics, int status);
static void mac802154_conf_reset(FAR struct ieee802154_mac_s *mac,
             int status);
static void mac802154_conf_rxenable(FAR struct ieee802154_mac_s *mac,
             int status);
static void mac802154_conf_scan(FAR struct ieee802154_mac_s *mac,
             int status, uint8_t type, uint32_t unscanned, int rsltsize,
             FAR uint8_t *edlist, FAR uint8_t *pandescs);
static void mac802154_conf_set(FAR struct ieee802154_mac_s *mac, int status,
             int attribute);
static void mac802154_conf_start(FAR struct ieee802154_mac_s *mac,
             int status);
static void mac802154_conf_poll(FAR struct ieee802154_mac_s *mac,
             int status);

  /* Asynchronous event indications, replied to synchronously with responses */

static void mac802154_ind_data(FAR struct ieee802154_mac_s *mac,
              FAR uint8_t *buf, int len);
static void mac802154_ind_associate(FAR struct ieee802154_mac_s *mac,
              uint16_t clipanid, FAR uint8_t *clieaddr);
static void mac802154_ind_disassociate(FAR struct ieee802154_mac_s *mac,
              FAR uint8_t *eadr, uint8_t reason);
static void mac802154_ind_beaconnotify(FAR struct ieee802154_mac_s *mac,
              FAR uint8_t *bsn, FAR struct ieee802154_pan_desc_s *pandesc,
              FAR uint8_t *sdu, int sdulen);
static void mac802154_ind_gts(FAR struct ieee802154_mac_s *mac,
              FAR uint8_t *devaddr, FAR uint8_t *characteristics);
static void mac802154_ind_orphan(FAR struct ieee802154_mac_s *mac,
              FAR uint8_t *orphanaddr);
static void mac802154_ind_commstatus(FAR struct ieee802154_mac_s *mac,
              uint16_t panid, FAR uint8_t *src, FAR uint8_t *dst,
              int status);
static void mac802154_ind_syncloss(FAR struct ieee802154_mac_s *mac,
              int reason);

/* Network interface support ************************************************/
/* Common TX logic */

static int  mac802154_transmit(FAR struct mac802154_driver_s *priv);
static int  mac802154_txpoll(FAR struct net_driver_s *dev);

/* Frame transfer */

static void mac802154_receive(FAR struct mac802154_driver_s *priv);
static void mac802154_txdone(FAR struct mac802154_driver_s *priv);

static void mac802154_transfer_work(FAR void *arg);
static int  mac802154_transfer(int irq, FAR void *context, FAR void *arg);

/* Watchdog timer expirations */

static void mac802154_txtimeout_work(FAR void *arg);
static void mac802154_txtimeout_expiry(int argc, wdparm_t arg, ...);

static void mac802154_poll_work(FAR void *arg);
static void mac802154_poll_expiry(int argc, wdparm_t arg, ...);

/* NuttX callback functions */

static int  mac802154_ifup(FAR struct net_driver_s *dev);
static int  mac802154_ifdown(FAR struct net_driver_s *dev);

static void mac802154_txavail_work(FAR void *arg);
static int  mac802154_txavail(FAR struct net_driver_s *dev);

#if defined(CONFIG_NET_IGMP) || defined(CONFIG_NET_ICMPv6)
static int  mac802154_addmac(FAR struct net_driver_s *dev, FAR const uint8_t *mac);
#ifdef CONFIG_NET_IGMP
static int  mac802154_rmmac(FAR struct net_driver_s *dev, FAR const uint8_t *mac);
#endif
#ifdef CONFIG_NET_ICMPv6
static void mac802154_ipv6multicast(FAR struct mac802154_driver_s *priv);
#endif
#endif
#ifdef CONFIG_NETDEV_IOCTL
static int mac802154_ioctl(FAR struct net_driver_s *dev, int cmd, long arg);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mac802154_conf_data
 *
 * Description:
 *   Data frame was received by remote device
 *
 ****************************************************************************/

static void mac802154_conf_data(FAR struct ieee802154_mac_s *mac,
                                FAR struct ieee802154_data_conf_s *conf)
{
  FAR struct mac802154_driver_s *priv;

  DEBUGASSERT(mac != NULL && mac->cb_context);
  priv = (FAR struct mac802154_driver_s *)mac->cb_context;
}

/****************************************************************************
 * Name: mac802154_conf_purge
 *
 * Description:
 *   Data frame was purged
 *
 ****************************************************************************/

static void mac802154_conf_purge(FAR struct ieee802154_mac_s *mac,
                                 uint8_t handle, int status)
{
  FAR struct mac802154_driver_s *priv;

  DEBUGASSERT(mac != NULL && mac->cb_context);
  priv = (FAR struct mac802154_driver_s *)mac->cb_context;
}

/****************************************************************************
 * Name: mac802154_conf_associate
 *
 * Description:
 *   Association request completed
 *
 ****************************************************************************/

static void mac802154_conf_associate(FAR struct ieee802154_mac_s *mac,
                                     uint16_t saddr, int status)
{
  FAR struct mac802154_driver_s *priv;

  DEBUGASSERT(mac != NULL && mac->cb_context);
  priv = (FAR struct mac802154_driver_s *)mac->cb_context;
}

/****************************************************************************
 * Name: mac802154_conf_disassociate
 *
 * Description:
 *   Disassociation request completed
 *
 ****************************************************************************/

static void mac802154_conf_disassociate(FAR struct ieee802154_mac_s *mac,
{
  FAR struct mac802154_driver_s *priv;

  DEBUGASSERT(mac != NULL && mac->cb_context);
  priv = (FAR struct mac802154_driver_s *)mac->cb_context;
}

/****************************************************************************
 * Name: mac802154_conf_get
 *
 * Description:
 *   PIvoata returned
 *
 ****************************************************************************/

static void mac802154_conf_get(FAR struct ieee802154_mac_s *mac, int status,
                               int attribute, FAR uint8_t *value,
                               int valuelen)
{
  FAR struct mac802154_driver_s *priv;

  DEBUGASSERT(mac != NULL && mac->cb_context);
  priv = (FAR struct mac802154_driver_s *)mac->cb_context;
}

/****************************************************************************
 * Name: mac802154_conf_gts
 *
 * Description:
 *   GTvoanagement completed
 *
 ****************************************************************************/

static void mac802154_conf_gts(FAR struct ieee802154_mac_s *mac,
                               FAR uint8_t *characteristics, int status)
{
  FAR struct mac802154_driver_s *priv;

  DEBUGASSERT(mac != NULL && mac->cb_context);
  priv = (FAR struct mac802154_driver_s *)mac->cb_context;
}

/****************************************************************************
 * Name: mac802154_conf_reset
 *
 * Description:
 *   MAveset completed
 *
 ****************************************************************************/

static void mac802154_conf_reset(FAR struct ieee802154_mac_s *mac,
                                 int status)
{
  FAR struct mac802154_driver_s *priv;

  DEBUGASSERT(mac != NULL && mac->cb_context);
  priv = (FAR struct mac802154_driver_s *)mac->cb_context;
}

/****************************************************************************
 * Name: mac802154_conf_rxenable
 *
 * Description:
 *
 ****************************************************************************/

static void mac802154_conf_rxenable(FAR struct ieee802154_mac_s *mac,
                                    int status)
{
  FAR struct mac802154_driver_s *priv;

  DEBUGASSERT(mac != NULL && mac->cb_context);
  priv = (FAR struct mac802154_driver_s *)mac->cb_context;
}

/****************************************************************************
 * Name: mac802154_conf_scan
 *
 * Description:
 *
 ****************************************************************************/

static void mac802154_conf_scan(FAR struct ieee802154_mac_s *mac,
                                int status, uint8_t type,
                                uint32_t unscanned, int rsltsize,
                                FAR uint8_t *edlist, FAR uint8_t *pandescs)
{
  FAR struct mac802154_driver_s *priv;

  DEBUGASSERT(mac != NULL && mac->cb_context);
  priv = (FAR struct mac802154_driver_s *)mac->cb_context;
}

/****************************************************************************
 * Name: mac802154_conf_set
 *
 * Description:
 *
 ****************************************************************************/

static void mac802154_conf_set(FAR struct ieee802154_mac_s *mac, int status,
                               int attribute)
{
  FAR struct mac802154_driver_s *priv;

  DEBUGASSERT(mac != NULL && mac->cb_context);
  priv = (FAR struct mac802154_driver_s *)mac->cb_context;
}

/****************************************************************************
 * Name: mac802154_conf_start
 *
 * Description:
 *
 ****************************************************************************/

static void mac802154_conf_start(FAR struct ieee802154_mac_s *mac,
                                int status)
{
  FAR struct mac802154_driver_s *priv;

  DEBUGASSERT(mac != NULL && mac->cb_context);
  priv = (FAR struct mac802154_driver_s *)mac->cb_context;
}

/****************************************************************************
 * Name: mac802154_conf_poll
 *
 * Description:
 *
 ****************************************************************************/

static void mac802154_conf_poll(FAR struct ieee802154_mac_s *mac,
                                int status)
{
  FAR struct mac802154_driver_s *priv;

  DEBUGASSERT(mac != NULL && mac->cb_context);
  priv = (FAR struct mac802154_driver_s *)mac->cb_context;
}

/****************************************************************************
 * Name: mac802154_ind_data
 *
 * Description:
 *    Data frame received
 *
 ****************************************************************************/

static void mac802154_ind_data(FAR struct ieee802154_mac_s *mac,
                               FAR uint8_t *buf, int len)
{
  FAR struct mac802154_driver_s *priv;

  DEBUGASSERT(mac != NULL && mac->cb_context);
  priv = (FAR struct mac802154_driver_s *)mac->cb_context;
}

/****************************************************************************
 * Name: mac802154_ind_associate
 *
 * Description:
 *   Association request received
 *
 ****************************************************************************/

static void mac802154_ind_associate(FAR struct ieee802154_mac_s *mac,
                                    uint16_t clipanid,
                                    FAR uint8_t *clieaddr)
{
  FAR struct mac802154_driver_s *priv;

  DEBUGASSERT(mac != NULL && mac->cb_context);
  priv = (FAR struct mac802154_driver_s *)mac->cb_context;
}

/****************************************************************************
 * Name: mac802154_ind_disassociate
 *
 * Description:
 *   Disassociation request received
 *
 ****************************************************************************/

static void mac802154_ind_disassociate(FAR struct ieee802154_mac_s *mac,
                                       FAR uint8_t *eadr, uint8_t reason)
{
  FAR struct mac802154_driver_s *priv;

  DEBUGASSERT(mac != NULL && mac->cb_context);
  priv = (FAR struct mac802154_driver_s *)mac->cb_context;
}

/****************************************************************************
 * Name: mac802154_ind_beaconnotify
 *
 * Description:
 *    Beacon notification
 *
 ****************************************************************************/

static void mac802154_ind_beaconnotify(FAR struct ieee802154_mac_s *mac,
                                       FAR uint8_t *bsn,
                                       FAR struct ieee802154_pan_desc_s *pandesc,
                                       FAR uint8_t *sdu, int sdulen)
{
  FAR struct mac802154_driver_s *priv;

  DEBUGASSERT(mac != NULL && mac->cb_context);
  priv = (FAR struct mac802154_driver_s *)mac->cb_context;
}

/****************************************************************************
 * Name: mac802154_ind_gts
 *
 * Description:
 *   GTS management request received
 *
 ****************************************************************************/

static void mac802154_ind_gts(FAR struct ieee802154_mac_s *mac,
                              FAR uint8_t *devaddr,
                              FAR uint8_t *characteristics)
{
  FAR struct mac802154_driver_s *priv;

  DEBUGASSERT(mac != NULL && mac->cb_context);
  priv = (FAR struct mac802154_driver_s *)mac->cb_context;
}

/****************************************************************************
 * Name: mac802154_ind_orphan
 *
 * Description:
 *   Orphan device detected
 *
 ****************************************************************************/

static void mac802154_ind_orphan(FAR struct ieee802154_mac_s *mac,
                                 FAR uint8_t *orphanaddr)
{
  FAR struct mac802154_driver_s *priv;

  DEBUGASSERT(mac != NULL && mac->cb_context);
  priv = (FAR struct mac802154_driver_s *)mac->cb_context;
}

/****************************************************************************
 * Name: mac802154_ind_commstatus
 *
 * Description:
 *
 ****************************************************************************/

static void mac802154_ind_commstatus(FAR struct ieee802154_mac_s *mac,
                                     uint16_t panid, FAR uint8_t *src,
                                     FAR uint8_t *dst, int status)
{
  FAR struct mac802154_driver_s *priv;

  DEBUGASSERT(mac != NULL && mac->cb_context);
  priv = (FAR struct mac802154_driver_s *)mac->cb_context;
}

/****************************************************************************
 * Name: mac802154_ind_syncloss
 *
 * Description:
 *
 ****************************************************************************/

static void mac802154_ind_syncloss(FAR struct ieee802154_mac_s *mac,
                                   int reason)
{
  FAR struct mac802154_driver_s *priv;

  DEBUGASSERT(mac != NULL && mac->cb_context);
  priv = (FAR struct mac802154_driver_s *)mac->cb_context;
}

/****************************************************************************
 * Name: mac802154_transmit
 *
 * Description:
 *   Start hardware transmission.  Called either from the txdone interrupt
 *   handling or from watchdog based polling.
 *
 * Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   the network is locked.
 *
 ****************************************************************************/

static int mac802154_transmit(FAR struct mac802154_driver_s *priv)
{
  /* Verify that the hardware is ready to send another packet.  If we get
   * here, then we are committed to sending a packet; Higher level logic
   * must have assured that there is no transmission in progress.
   */

  /* Increment statistics */

  NETDEV_TXPACKETS(priv->m8_dev);

  /* Send the packet: address=priv->m8_dev.d_buf, length=priv->m8_dev.d_len */

  /* Enable Tx interrupts */

  /* Setup the TX timeout watchdog (perhaps restarting the timer) */

  (void)wd_start(priv->m8_txtimeout, skeleton_TXTIMEOUT,
                 mac802154_txtimeout_expiry, 1, (wdparm_t)priv);
  return OK;
}

/****************************************************************************
 * Name: mac802154_txpoll
 *
 * Description:
 *   The transmitter is available, check if the network has any outgoing
 *   packets ready to send.  This is a callback from devif_poll().
 *   devif_poll() may be called:
 *
 *   1. When the preceding TX packet send is complete,
 *   2. When the preceding TX packet send timesout and the interface is reset
 *   3. During normal TX polling
 *
 * Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   the network is locked.
 *
 ****************************************************************************/

static int mac802154_txpoll(FAR struct net_driver_s *dev)
{
  FAR struct mac802154_driver_s *priv = (FAR struct mac802154_driver_s *)dev->d_private;

  /* If the polling resulted in data that should be sent out, the field
   * i_framelist will set to a new, outgoing list of frames.
   */

  if (priv->m8_dev.i_framelist != NULL)
    {
       /* And send the packet */

       mac802154_transmit(priv);
    }

  /* If zero is returned, the polling will continue until all connections have
   * been examined.
   */

  return 0;
}

/****************************************************************************
 * Name: mac802154_receive
 *
 * Description:
 *   An interrupt was received indicating the availability of a new RX packet
 *
 * Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static void mac802154_receive(FAR struct mac802154_driver_s *priv)
{
  FAR struct iob_s *iob;
  FAR struct iob_s *fptr;

  /* Allocate an IOB to hold the frame */

  net_lock();
  iob = iob_alloc(false);
  if (iob == NULL)
    {
      nerr("ERROR: Failed to allocate an IOB\n")
      return;
    }

  /* Copy the frame data into the IOB.
   * REVISIT:  Can a copy be avoided?
   */

  fptr = iob->io_data;
#warning Missing logic

  /* Transfer the frame to the network logic */

  priv->m8_dev.framelist = iob;
  sixlowpan_input(&priv->m8_dev);

  /* If the above function invocation resulted in data that should be sent
   * out, the field i_framelist will set to a new, outgoing list of frames.
   */

  if (priv->m8_dev.i_framelist != NULL)
    {
       /* And send the packet */

       mac802154_transmit(priv);
    }
}

/****************************************************************************
 * Name: mac802154_txdone
 *
 * Description:
 *   An interrupt was received indicating that the last TX packet(s) is done
 *
 * Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static void mac802154_txdone(FAR struct mac802154_driver_s *priv)
{
  int delay;

  /* Check for errors and update statistics */

  NETDEV_TXDONE(priv->m8_dev);

  /* Check if there are pending transmissions */

  /* If no further transmissions are pending, then cancel the TX timeout and
   * disable further Tx interrupts.
   */

  wd_cancel(priv->m8_txtimeout);

  /* And disable further TX interrupts. */

  /* In any event, poll the network for new TX data */

  (void)devif_poll(&priv->m8_dev, mac802154_txpoll);
}

/****************************************************************************
 * Name: mac802154_transfer_work
 *
 * Description:
 *   Perform interrupt related work from the worker thread
 *
 * Parameters:
 *   arg - The argument passed when work_queue() was called.
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static void mac802154_transfer_work(FAR void *arg)
{
  FAR struct mac802154_driver_s *priv = (FAR struct mac802154_driver_s *)arg;

  /* Lock the network and serialize driver operations if necessary.
   * NOTE: Serialization is only required in the case where the driver work
   * is performed on an LP worker thread and where more than one LP worker
   * thread has been configured.
   */

  net_lock();

  /* Process pending Ethernet interrupts */

  /* Get and clear interrupt status bits */

  /* Handle interrupts according to status bit settings */

  /* Check if we received an incoming packet, if so, call mac802154_receive() */

  mac802154_receive(priv);

  /* Check if a packet transmission just completed.  If so, call mac802154_txdone.
   * This may disable further Tx interrupts if there are no pending
   * transmissions.
   */

  mac802154_txdone(priv);
  net_unlock();

  /* Re-enable Ethernet interrupts */

  up_enable_irq(CONFIG_IEEE802154_NETDEV_IRQ);
}

/****************************************************************************
 * Name: mac802154_transfer
 *
 * Description:
 *   Hardware interrupt handler
 *
 * Parameters:
 *   irq     - Number of the IRQ that generated the interrupt
 *   context - Interrupt register state save info (architecture-specific)
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *
 ****************************************************************************/

static int mac802154_transfer(int irq, FAR void *context, FAR void *arg)
{
  FAR struct mac802154_driver_s *priv = &g_skel[0];

  /* Disable further Ethernet interrupts.  Because Ethernet interrupts are
   * also disabled if the TX timeout event occurs, there can be no race
   * condition here.
   */

  up_disable_irq(CONFIG_IEEE802154_NETDEV_IRQ);

  /* TODO: Determine if a TX transfer just completed */

    {
      /* If a TX transfer just completed, then cancel the TX timeout so
       * there will be no race condition between any subsequent timeout
       * expiration and the deferred interrupt processing.
       */

       wd_cancel(priv->m8_txtimeout);
    }

  /* Schedule to perform the interrupt processing on the worker thread. */

  work_queue(ETHWORK, &priv->m8_irqwork, mac802154_transfer_work, priv, 0);
  return OK;
}

/****************************************************************************
 * Name: mac802154_txtimeout_work
 *
 * Description:
 *   Perform TX timeout related work from the worker thread
 *
 * Parameters:
 *   arg - The argument passed when work_queue() as called.
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static void mac802154_txtimeout_work(FAR void *arg)
{
  FAR struct mac802154_driver_s *priv = (FAR struct mac802154_driver_s *)arg;

  /* Lock the network and serialize driver operations if necessary.
   * NOTE: Serialization is only required in the case where the driver work
   * is performed on an LP worker thread and where more than one LP worker
   * thread has been configured.
   */

  net_lock();

  /* Increment statistics and dump debug info */

  NETDEV_TXTIMEOUTS(priv->m8_dev);

  /* Then reset the hardware */

  /* Then poll the network for new XMIT data */

  (void)devif_poll(&priv->m8_dev, mac802154_txpoll);
  net_unlock();
}

/****************************************************************************
 * Name: mac802154_txtimeout_expiry
 *
 * Description:
 *   Our TX watchdog timed out.  Called from the timer interrupt handler.
 *   The last TX never completed.  Reset the hardware and start again.
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

static void mac802154_txtimeout_expiry(int argc, wdparm_t arg, ...)
{
  FAR struct mac802154_driver_s *priv = (FAR struct mac802154_driver_s *)arg;

  /* Disable further Ethernet interrupts.  This will prevent some race
   * conditions with interrupt work.  There is still a potential race
   * condition with interrupt work that is already queued and in progress.
   */

  up_disable_irq(CONFIG_IEEE802154_NETDEV_IRQ);

  /* Schedule to perform the TX timeout processing on the worker thread. */

  work_queue(ETHWORK, &priv->m8_irqwork, mac802154_txtimeout_work, priv, 0);
}

/****************************************************************************
 * Name: mac802154_poll_process
 *
 * Description:
 *   Perform the periodic poll.  This may be called either from watchdog
 *   timer logic or from the worker thread, depending upon the configuration.
 *
 * Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static inline void mac802154_poll_process(FAR struct mac802154_driver_s *priv)
{
}

/****************************************************************************
 * Name: mac802154_poll_work
 *
 * Description:
 *   Perform periodic polling from the worker thread
 *
 * Parameters:
 *   arg - The argument passed when work_queue() as called.
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static void mac802154_poll_work(FAR void *arg)
{
  FAR struct mac802154_driver_s *priv = (FAR struct mac802154_driver_s *)arg;

  /* Lock the network and serialize driver operations if necessary.
   * NOTE: Serialization is only required in the case where the driver work
   * is performed on an LP worker thread and where more than one LP worker
   * thread has been configured.
   */

  net_lock();

  /* Perform the poll */

  /* Check if there is room in the send another TX packet.  We cannot perform
   * the TX poll if he are unable to accept another packet for transmission.
   */

  /* If so, update TCP timing states and poll the network for new XMIT data.
   * Hmmm.. might be bug here.  Does this mean if there is a transmit in
   * progress, we will missing TCP time state updates?
   */

  (void)devif_timer(&priv->m8_dev, mac802154_txpoll);

  /* Setup the watchdog poll timer again */

  (void)wd_start(priv->m8_txpoll, skeleton_WDDELAY, mac802154_poll_expiry, 1,
                 (wdparm_t)priv);
  net_unlock();
}

/****************************************************************************
 * Name: mac802154_poll_expiry
 *
 * Description:
 *   Periodic timer handler.  Called from the timer interrupt handler.
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

static void mac802154_poll_expiry(int argc, wdparm_t arg, ...)
{
  FAR struct mac802154_driver_s *priv = (FAR struct mac802154_driver_s *)arg;

  /* Schedule to perform the interrupt processing on the worker thread. */

  work_queue(ETHWORK, &priv->m8_pollwork, mac802154_poll_work, priv, 0);
}

/****************************************************************************
 * Name: mac802154_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the Ethernet interface when an IP address is
 *   provided
 *
 * Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int mac802154_ifup(FAR struct net_driver_s *dev)
{
  FAR struct mac802154_driver_s *priv = (FAR struct mac802154_driver_s *)dev->d_private;

#ifdef CONFIG_NET_IPv4
  ninfo("Bringing up: %d.%d.%d.%d\n",
        dev->d_ipaddr & 0xff, (dev->d_ipaddr >> 8) & 0xff,
        (dev->d_ipaddr >> 16) & 0xff, dev->d_ipaddr >> 24);
#endif
#ifdef CONFIG_NET_IPv6
  ninfo("Bringing up: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        dev->d_ipv6addr[0], dev->d_ipv6addr[1], dev->d_ipv6addr[2],
        dev->d_ipv6addr[3], dev->d_ipv6addr[4], dev->d_ipv6addr[5],
        dev->d_ipv6addr[6], dev->d_ipv6addr[7]);
#endif

  /* Initialize PHYs, the Ethernet interface, and setup up Ethernet interrupts */

  /* Instantiate the MAC address from priv->m8_dev.d_mac.ether_addr_octet */

#ifdef CONFIG_NET_ICMPv6
  /* Set up IPv6 multicast address filtering */

  mac802154_ipv6multicast(priv);
#endif

  /* Set and activate a timer process */

  (void)wd_start(priv->m8_txpoll, skeleton_WDDELAY, mac802154_poll_expiry, 1,
                 (wdparm_t)priv);

  /* Enable the Ethernet interrupt */

  priv->m8_bifup = true;
  up_enable_irq(CONFIG_IEEE802154_NETDEV_IRQ);
  return OK;
}

/****************************************************************************
 * Name: mac802154_ifdown
 *
 * Description:
 *   NuttX Callback: Stop the interface.
 *
 * Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int mac802154_ifdown(FAR struct net_driver_s *dev)
{
  FAR struct mac802154_driver_s *priv = (FAR struct mac802154_driver_s *)dev->d_private;
  irqstate_t flags;

  /* Disable the Ethernet interrupt */

  flags = enter_critical_section();
  up_disable_irq(CONFIG_IEEE802154_NETDEV_IRQ);

  /* Cancel the TX poll timer and TX timeout timers */

  wd_cancel(priv->m8_txpoll);
  wd_cancel(priv->m8_txtimeout);

  /* Put the EMAC in its reset, non-operational state.  This should be
   * a known configuration that will guarantee the mac802154_ifup() always
   * successfully brings the interface back up.
   */

  /* Mark the device "down" */

  priv->m8_bifup = false;
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: mac802154_txavail_work
 *
 * Description:
 *   Perform an out-of-cycle poll on the worker thread.
 *
 * Parameters:
 *   arg - Reference to the NuttX driver state structure (cast to void*)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called on the higher priority worker thread.
 *
 ****************************************************************************/

static void mac802154_txavail_work(FAR void *arg)
{
  FAR struct mac802154_driver_s *priv = (FAR struct mac802154_driver_s *)arg;

  /* Lock the network and serialize driver operations if necessary.
   * NOTE: Serialization is only required in the case where the driver work
   * is performed on an LP worker thread and where more than one LP worker
   * thread has been configured.
   */

  net_lock();

  /* Ignore the notification if the interface is not yet up */

  if (priv->m8_bifup)
    {
      /* Check if there is room in the hardware to hold another outgoing packet. */

      /* If so, then poll the network for new XMIT data */

      (void)devif_poll(&priv->m8_dev, mac802154_txpoll);
    }

  net_unlock();
}

/****************************************************************************
 * Name: mac802154_txavail
 *
 * Description:
 *   Driver callback invoked when new TX data is available.  This is a
 *   stimulus perform an out-of-cycle poll and, thereby, reduce the TX
 *   latency.
 *
 * Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called in normal user mode
 *
 ****************************************************************************/

static int mac802154_txavail(FAR struct net_driver_s *dev)
{
  FAR struct mac802154_driver_s *priv = (FAR struct mac802154_driver_s *)dev->d_private;

  /* Is our single work structure available?  It may not be if there are
   * pending interrupt actions and we will have to ignore the Tx
   * availability action.
   */

  if (work_available(&priv->m8_pollwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      work_queue(ETHWORK, &priv->m8_pollwork, mac802154_txavail_work, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: mac802154_addmac
 *
 * Description:
 *   NuttX Callback: Add the specified MAC address to the hardware multicast
 *   address filtering
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be added
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#if defined(CONFIG_NET_IGMP) || defined(CONFIG_NET_ICMPv6)
static int mac802154_addmac(FAR struct net_driver_s *dev, FAR const uint8_t *mac)
{
  FAR struct mac802154_driver_s *priv = (FAR struct mac802154_driver_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

  return OK;
}
#endif

/****************************************************************************
 * Name: mac802154_rmmac
 *
 * Description:
 *   NuttX Callback: Remove the specified MAC address from the hardware multicast
 *   address filtering
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be removed
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IGMP
static int mac802154_rmmac(FAR struct net_driver_s *dev, FAR const uint8_t *mac)
{
  FAR struct mac802154_driver_s *priv = (FAR struct mac802154_driver_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

  return OK;
}
#endif

/****************************************************************************
 * Name: mac802154_ipv6multicast
 *
 * Description:
 *   Configure the IPv6 multicast MAC address.
 *
 * Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6
static void mac802154_ipv6multicast(FAR struct mac802154_driver_s *priv)
{
  FAR struct net_driver_s *dev;
  uint16_t tmp16;
  uint8_t mac[6];

  /* For ICMPv6, we need to add the IPv6 multicast address
   *
   * For IPv6 multicast addresses, the Ethernet MAC is derived by
   * the four low-order octets OR'ed with the MAC 33:33:00:00:00:00,
   * so for example the IPv6 address FF02:DEAD:BEEF::1:3 would map
   * to the Ethernet MAC address 33:33:00:01:00:03.
   *
   * NOTES:  This appears correct for the ICMPv6 Router Solicitation
   * Message, but the ICMPv6 Neighbor Solicitation message seems to
   * use 33:33:ff:01:00:03.
   */

  mac[0] = 0x33;
  mac[1] = 0x33;

  dev    = &priv->dev;
  tmp16  = dev->d_ipv6addr[6];
  mac[2] = 0xff;
  mac[3] = tmp16 >> 8;

  tmp16  = dev->d_ipv6addr[7];
  mac[4] = tmp16 & 0xff;
  mac[5] = tmp16 >> 8;

  ninfo("IPv6 Multicast: %02x:%02x:%02x:%02x:%02x:%02x\n",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  (void)mac802154_addmac(dev, mac);

#ifdef CONFIG_NET_ICMPv6_AUTOCONF
  /* Add the IPv6 all link-local nodes Ethernet address.  This is the
   * address that we expect to receive ICMPv6 Router Advertisement
   * packets.
   */

  (void)mac802154_addmac(dev, g_ipv6_ethallnodes.ether_addr_octet);

#endif /* CONFIG_NET_ICMPv6_AUTOCONF */

#ifdef CONFIG_NET_ICMPv6_ROUTER
  /* Add the IPv6 all link-local routers Ethernet address.  This is the
   * address that we expect to receive ICMPv6 Router Solicitation
   * packets.
   */

  (void)mac802154_addmac(dev, g_ipv6_ethallrouters.ether_addr_octet);

#endif /* CONFIG_NET_ICMPv6_ROUTER */
}
#endif /* CONFIG_NET_ICMPv6 */

/****************************************************************************
 * Name: mac802154_ioctl
 *
 * Description:
 *   Handle network IOCTL commands directed to this device.
 *
 * Parameters:
 *   dev - Reference to the NuttX driver state structure
 *   cmd - The IOCTL command
 *   arg - The argument for the IOCTL command
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NETDEV_IOCTL
static int mac802154_ioctl(FAR struct net_driver_s *dev, int cmd, long arg)
{
  FAR struct mac802154_driver_s *priv = (FAR struct mac802154_driver_s *)dev->d_private;
  int ret = -EINVAL;

  /* Check for IOCTLs aimed at the IEEE802.15.4 MAC layer */

  if (_MAC802154IOCVALID(cmd))
    {
      FAR struct ieee802154_netmac_s *netmac =
        (FAR struct ieee802154_netmac_s *)arg;

      if (netmac != NULL)
        {
          unsigned long macarg = (unsigned int)((uintptr_t)&netmac->u);
          ret = priv->m8_mac.macops.ioctl(priv->m8_mac, cmd, macarg);
        }
    }

  /* No, check for IOCTLs aimed at the IEEE802.15.4 radio layer */

  else if (_PHY802154IOCVALID(cmd))
    {
      FAR struct ieee802154_netradio_s *netradio =
        (FAR struct ieee802154_netradio_s *)arg;

      if (netradio != NULL)
        {
          unsigned long radioarg = (unsigned int)((uintptr_t)&netradio->u);
          ret = priv->m8_mac.macops.ioctl(priv->m8_mac, cmd, radioarg);
        }
    }

  /* Okay, we have no idea what this command is.. just give to the
   * IEEE802.15.4 MAC layer without modification.
   */

  else
   {
     ret = priv->m8_mac.macops.ioctl(priv->m8_mac, cmd, arg);
   }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mac802154netdev_register
 *
 * Description:
 *   Register a network driver to access the IEEE 802.15.4 MAC layer from
 *   a socket using 6loWPAN
 *
 * Input Parameters:
 *   mac - Pointer to the mac layer struct to be registered.
 *
 * Returned Values:
 *   Zero (OK) is returned on success.  Otherwise a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int mac802154netdev_register(FAR struct ieee802154_mac_s *mac);
{
  FAR struct mac802154_driver_s *priv;
  FAR truct ieee802154_maccb_s *macb;
  FAR struct net_driver_s *dev;
  FAR uint8_t *pktbuf;

  DEBUGASSERT(radio != NULL);

  /* Get the interface structure associated with this interface number. */

  priv = (FAR struct mac802154_driver_s *)
    kmm_zalloc(sizeof(struct mac802154_driver_s));

  if (priv == NULL)
    {
      nerr("ERROR: Failed to allocate the device structure\n");
      return -ENOMEM;
    }

  /* Allocate a packet buffer (not used by this driver, but need by the
   * upper networking layer)
   */

  pktbuf = (FAR uint8_t *)kmm_malloc(CONFIG_NET_6LOWPAN_MTU + CONFIG_NET_GUARDSIZE);
  if (pktbuf == NULL
    {
      nerr("ERROR: Failed to allocate the packet buffer\n");
      kmm_free(priv);
      return -ENOMEM;
    }

  /* Initialize the driver structure */

  dev                = &ieee->m8_dev.i_dev;
  dev->d_buf         = pktbuf;             /* Single packet buffer */
  dev->d_ifup        = mac802154_ifup;     /* I/F up (new IP address) callback */
  dev->d_ifdown      = mac802154_ifdown;   /* I/F down callback */
  dev->d_txavail     = mac802154_txavail;  /* New TX data callback */
#ifdef CONFIG_NET_IGMP
  dev->d_addmac      = mac802154_addmac;   /* Add multicast MAC address */
  dev->d_rmmac       = mac802154_rmmac;    /* Remove multicast MAC address */
#endif
 #ifdef CONFIG_NETDEV_IOCTL
  dev->d_ioctl       = mac802154_ioctl;    /* Handle network IOCTL commands */
#endif
  dev->d_private     = (FAR void *)priv;   /* Used to recover private state from dev */

  /* Create a watchdog for timing polling for and timing of transmisstions */

  priv->m8_mac       = mac;                /* Save the MAC interface instance */
  priv->m8_txpoll    = wd_create();        /* Create periodic poll timer */
  priv->m8_txtimeout = wd_create();        /* Create TX timeout timer */

  DEBUGASSERT(priv->m8_txpoll != NULL && priv->m8_txtimeout != NULL);

  /* Initialize the MAC callbacks */

  maccb                    = &mac->cbs;
  maccb->cb_context        = priv;
  maccb->conf_data         = mac802154_conf_data;
  maccb->conf_purge        = mac802154_conf_purge;
  maccb->conf_associate    = mac802154_conf_associate;
  maccb->conf_disassociate = mac802154_conf_disassociate;
  maccb->conf_get          = mac802154_conf_get;
  maccb->conf_gts          = mac802154_conf_gts;
  maccb->conf_reset        = mac802154_conf_reset;
  maccb->conf_rxenable     = mac802154_conf_rxenable;
  maccb->conf_scan         = mac802154_conf_scan;
  maccb->conf_set          = mac802154_conf_set;
  maccb->conf_start        = mac802154_conf_start;
  maccb->conf_poll         = mac802154_conf_poll;
  maccb->ind_data          = mac802154_ind_data;
  maccb->ind_associate     = mac802154_ind_associate;
  maccb->ind_disassociate  = mac802154_ind_disassociate;
  maccb->ind_beaconnotify  = mac802154_ind_beaconnotify;
  maccb->ind_gts           = mac802154_ind_gts;
  maccb->ind_orphan        = mac802154_ind_orphan;
  maccb->ind_commstatus    = mac802154_ind_commstatus;
  maccb->ind_syncloss      = mac802154_ind_syncloss;

  /* Put the interface in the down state. */

  mac802154_ifdown(dev);

  /* Register the device with the OS so that socket IOCTLs can be performed */

  (void)netdev_register(&priv->m8_dev, NET_LL_IEEE802154);
  return OK;
}

#endif /* CONFIG_NET && CONFIG_NET_skeleton */
