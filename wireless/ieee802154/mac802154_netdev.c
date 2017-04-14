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

#define BUF ((struct eth_hdr_s *)priv->md_dev.d_buf)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This is our private version of the MAC callback stucture */

struct macnet_callback_s
{
  /* This holds the information visible to the MAC layer */

  struct ieee802154_maccb_s mc_cb;        /* Interface understood by the MAC layer */
  FAR struct macnet_driver_s *mc_priv; /* Our priv data */
};

/* The macnet_driver_s encapsulates all state information for a single
 * IEEE802.15.4 MAC interface.
 */

struct macnet_driver_s
{
  /* This holds the information visible to the NuttX network */

  struct ieee802154_driver_s md_dev;  /* Interface understood by the network */
  struct macnet_callback_s md_cb;     /* Callback information */
  MACHANDLE md_mac;                   /* Contained MAC interface */

  /* For internal use by this driver */

  bool md_bifup;               /* true:ifup false:ifdown */
  WDOG_ID md_txpoll;           /* TX poll timer */
  WDOG_ID md_txtimeout;        /* TX timeout timer */
  struct work_s md_irqwork;    /* For deferring interupt work to the work queue */
  struct work_s md_pollwork;   /* For deferring poll work to the work queue */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* IEE802.15.4 MAC callback functions ***************************************/
/* Asynchronous confirmations to requests */

static void macnet_conf_data(MACHANDLE mac,
             FAR struct ieee802154_data_conf_s *conf);
static void macnet_conf_purge(MACHANDLE mac, uint8_t handle, int status);
static void macnet_conf_associate(MACHANDLE mac, uint16_t saddr, int status);
static void macnet_conf_disassociate(MACHANDLE mac, int status);
static void macnet_conf_get(MACHANDLE mac, int status, int attribute,
             FAR uint8_t *value, int valuelen);
static void macnet_conf_gts(MACHANDLE mac, FAR uint8_t *characteristics,
             int status);
static void macnet_conf_reset(MACHANDLE mac, int status);
static void macnet_conf_rxenable(MACHANDLE mac, int status);
static void macnet_conf_scan(MACHANDLE mac, int status, uint8_t type,
             uint32_t unscanned, int rsltsize, FAR uint8_t *edlist,
             FAR uint8_t *pandescs);
static void macnet_conf_set(MACHANDLE mac, int status, int attribute);
static void macnet_conf_start(MACHANDLE mac, int status);
static void macnet_conf_poll(MACHANDLE mac, int status);

  /* Asynchronous event indications, replied to synchronously with responses */

static void macnet_ind_data(MACHANDLE mac, FAR uint8_t *buf, int len);
static void macnet_ind_associate(MACHANDLE mac, uint16_t clipanid,
              FAR uint8_t *clieaddr);
static void macnet_ind_disassociate(MACHANDLE mac, FAR uint8_t *eadr,
              uint8_t reason);
static void macnet_ind_beaconnotify(MACHANDLE mac, FAR uint8_t *bsn,
              FAR struct ieee802154_pan_desc_s *pandesc, FAR uint8_t *sdu,
              int sdulen);
static void macnet_ind_gts(MACHANDLE mac, FAR uint8_t *devaddr,
              FAR uint8_t *characteristics);
static void macnet_ind_orphan(MACHANDLE mac, FAR uint8_t *orphanaddr);
static void macnet_ind_commstatus(MACHANDLE mac, uint16_t panid,
              FAR uint8_t *src, FAR uint8_t *dst, int status);
static void macnet_ind_syncloss(MACHANDLE mac, int reason);

/* Network interface support ************************************************/
/* Common TX logic */

static int  macnet_transmit(FAR struct macnet_driver_s *priv);
static int  macnet_txpoll(FAR struct net_driver_s *dev);

/* Frame transfer */

static void macnet_receive(FAR struct macnet_driver_s *priv);
static void macnet_txdone(FAR struct macnet_driver_s *priv);

static void macnet_transfer_work(FAR void *arg);
static int  macnet_transfer(int irq, FAR void *context, FAR void *arg);

/* Watchdog timer expirations */

static void macnet_txtimeout_work(FAR void *arg);
static void macnet_txtimeout_expiry(int argc, wdparm_t arg, ...);

static void macnet_poll_work(FAR void *arg);
static void macnet_poll_expiry(int argc, wdparm_t arg, ...);

/* NuttX callback functions */

static int  macnet_ifup(FAR struct net_driver_s *dev);
static int  macnet_ifdown(FAR struct net_driver_s *dev);

static void macnet_txavail_work(FAR void *arg);
static int  macnet_txavail(FAR struct net_driver_s *dev);

#if defined(CONFIG_NET_IGMP) || defined(CONFIG_NET_ICMPv6)
static int  macnet_addmac(FAR struct net_driver_s *dev,
                          FAR const uint8_t *mac);
#ifdef CONFIG_NET_IGMP
static int  macnet_rmmac(FAR struct net_driver_s *dev,
                         FAR const uint8_t *mac);
#endif
#ifdef CONFIG_NET_ICMPv6
static void macnet_ipv6multicast(FAR struct macnet_driver_s *priv);
#endif
#endif
#ifdef CONFIG_NETDEV_IOCTL
static int macnet_ioctl(FAR struct net_driver_s *dev, int cmd, long arg);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: macnet_conf_data
 *
 * Description:
 *   Data frame was received by remote device
 *
 ****************************************************************************/

static void macnet_conf_data(MACHANDLE mac,
                             FAR struct ieee802154_data_conf_s *conf)
{
  FAR struct macnet_callback_s *cb = (FAR struct macnet_callback_s *)mac;
  FAR struct macnet_driver_s *priv;

  DEBUGASSERT(cb != NULL && cb->mc_priv != NULL);
  priv = cb->mc_priv;
}

/****************************************************************************
 * Name: macnet_conf_purge
 *
 * Description:
 *   Data frame was purged
 *
 ****************************************************************************/

static void macnet_conf_purge(MACHANDLE mac, uint8_t handle, int status)
{
  FAR struct macnet_callback_s *cb = (FAR struct macnet_callback_s *)mac;
  FAR struct macnet_driver_s *priv;

  DEBUGASSERT(cb != NULL && cb->mc_priv != NULL);
  priv = cb->mc_priv;
}

/****************************************************************************
 * Name: macnet_conf_associate
 *
 * Description:
 *   Association request completed
 *
 ****************************************************************************/

static void macnet_conf_associate(MACHANDLE mac, uint16_t saddr, int status)
{
  FAR struct macnet_callback_s *cb = (FAR struct macnet_callback_s *)mac;
  FAR struct macnet_driver_s *priv;

  DEBUGASSERT(cb != NULL && cb->mc_priv != NULL);
  priv = cb->mc_priv;
}

/****************************************************************************
 * Name: macnet_conf_disassociate
 *
 * Description:
 *   Disassociation request completed
 *
 ****************************************************************************/

static void macnet_conf_disassociate(MACHANDLE mac, int status)
{
  FAR struct macnet_callback_s *cb = (FAR struct macnet_callback_s *)mac;
  FAR struct macnet_driver_s *priv;

  DEBUGASSERT(cb != NULL && cb->mc_priv != NULL);
  priv = cb->mc_priv;
}

/****************************************************************************
 * Name: macnet_conf_get
 *
 * Description:
 *   PIvoata returned
 *
 ****************************************************************************/

static void macnet_conf_get(MACHANDLE mac, int status, int attribute,
                            FAR uint8_t *value, int valuelen)
{
  FAR struct macnet_callback_s *cb = (FAR struct macnet_callback_s *)mac;
  FAR struct macnet_driver_s *priv;

  DEBUGASSERT(cb != NULL && cb->mc_priv != NULL);
  priv = cb->mc_priv;
}

/****************************************************************************
 * Name: macnet_conf_gts
 *
 * Description:
 *   GTvoanagement completed
 *
 ****************************************************************************/

static void macnet_conf_gts(MACHANDLE mac, FAR uint8_t *characteristics,
                            int status)
{
  FAR struct macnet_callback_s *cb = (FAR struct macnet_callback_s *)mac;
  FAR struct macnet_driver_s *priv;

  DEBUGASSERT(cb != NULL && cb->mc_priv != NULL);
  priv = cb->mc_priv;
}

/****************************************************************************
 * Name: macnet_conf_reset
 *
 * Description:
 *   MAveset completed
 *
 ****************************************************************************/

static void macnet_conf_reset(MACHANDLE mac, int status)
{
  FAR struct macnet_callback_s *cb = (FAR struct macnet_callback_s *)mac;
  FAR struct macnet_driver_s *priv;

  DEBUGASSERT(cb != NULL && cb->mc_priv != NULL);
  priv = cb->mc_priv;
}

/****************************************************************************
 * Name: macnet_conf_rxenable
 *
 * Description:
 *
 ****************************************************************************/

static void macnet_conf_rxenable(MACHANDLE mac, int status)
{
  FAR struct macnet_callback_s *cb = (FAR struct macnet_callback_s *)mac;
  FAR struct macnet_driver_s *priv;

  DEBUGASSERT(cb != NULL && cb->mc_priv != NULL);
  priv = cb->mc_priv;
}

/****************************************************************************
 * Name: macnet_conf_scan
 *
 * Description:
 *
 ****************************************************************************/

static void macnet_conf_scan(MACHANDLE mac, int status, uint8_t type,
                             uint32_t unscanned, int rsltsize,
                             FAR uint8_t *edlist, FAR uint8_t *pandescs)
{
  FAR struct macnet_callback_s *cb = (FAR struct macnet_callback_s *)mac;
  FAR struct macnet_driver_s *priv;

  DEBUGASSERT(cb != NULL && cb->mc_priv != NULL);
  priv = cb->mc_priv;
}

/****************************************************************************
 * Name: macnet_conf_set
 *
 * Description:
 *
 ****************************************************************************/

static void macnet_conf_set(MACHANDLE mac, int status, int attribute)
{
  FAR struct macnet_callback_s *cb = (FAR struct macnet_callback_s *)mac;
  FAR struct macnet_driver_s *priv;

  DEBUGASSERT(cb != NULL && cb->mc_priv != NULL);
  priv = cb->mc_priv;
}

/****************************************************************************
 * Name: macnet_conf_start
 *
 * Description:
 *
 ****************************************************************************/

static void macnet_conf_start(MACHANDLE mac, int status)
{
  FAR struct macnet_callback_s *cb = (FAR struct macnet_callback_s *)mac;
  FAR struct macnet_driver_s *priv;

  DEBUGASSERT(cb != NULL && cb->mc_priv != NULL);
  priv = cb->mc_priv;
}

/****************************************************************************
 * Name: macnet_conf_poll
 *
 * Description:
 *
 ****************************************************************************/

static void macnet_conf_poll(MACHANDLE mac, int status)
{
  FAR struct macnet_callback_s *cb = (FAR struct macnet_callback_s *)mac;
  FAR struct macnet_driver_s *priv;

  DEBUGASSERT(cb != NULL && cb->mc_priv != NULL);
  priv = cb->mc_priv;
}

/****************************************************************************
 * Name: macnet_ind_data
 *
 * Description:
 *    Data frame received
 *
 ****************************************************************************/

static void macnet_ind_data(MACHANDLE mac, FAR uint8_t *buf, int len)
{
  FAR struct macnet_callback_s *cb = (FAR struct macnet_callback_s *)mac;
  FAR struct macnet_driver_s *priv;

  DEBUGASSERT(cb != NULL && cb->mc_priv != NULL);
  priv = cb->mc_priv;
}

/****************************************************************************
 * Name: macnet_ind_associate
 *
 * Description:
 *   Association request received
 *
 ****************************************************************************/

static void macnet_ind_associate(MACHANDLE mac, uint16_t clipanid,
                                 FAR uint8_t *clieaddr)
{
  FAR struct macnet_callback_s *cb = (FAR struct macnet_callback_s *)mac;
  FAR struct macnet_driver_s *priv;

  DEBUGASSERT(cb != NULL && cb->mc_priv != NULL);
  priv = cb->mc_priv;
}

/****************************************************************************
 * Name: macnet_ind_disassociate
 *
 * Description:
 *   Disassociation request received
 *
 ****************************************************************************/

static void macnet_ind_disassociate(MACHANDLE mac, FAR uint8_t *eadr,
                                    uint8_t reason)
{
  FAR struct macnet_callback_s *cb = (FAR struct macnet_callback_s *)mac;
  FAR struct macnet_driver_s *priv;

  DEBUGASSERT(cb != NULL && cb->mc_priv != NULL);
  priv = cb->mc_priv;
}

/****************************************************************************
 * Name: macnet_ind_beaconnotify
 *
 * Description:
 *    Beacon notification
 *
 ****************************************************************************/

static void macnet_ind_beaconnotify(MACHANDLE mac, FAR uint8_t *bsn,
                                    FAR struct ieee802154_pan_desc_s *pandesc,
                                    FAR uint8_t *sdu, int sdulen)
{
  FAR struct macnet_callback_s *cb = (FAR struct macnet_callback_s *)mac;
  FAR struct macnet_driver_s *priv;

  DEBUGASSERT(cb != NULL && cb->mc_priv != NULL);
  priv = cb->mc_priv;
}

/****************************************************************************
 * Name: macnet_ind_gts
 *
 * Description:
 *   GTS management request received
 *
 ****************************************************************************/

static void macnet_ind_gts(MACHANDLE mac, FAR uint8_t *devaddr,
                           FAR uint8_t *characteristics)
{
  FAR struct macnet_callback_s *cb = (FAR struct macnet_callback_s *)mac;
  FAR struct macnet_driver_s *priv;

  DEBUGASSERT(cb != NULL && cb->mc_priv != NULL);
  priv = cb->mc_priv;
}

/****************************************************************************
 * Name: macnet_ind_orphan
 *
 * Description:
 *   Orphan device detected
 *
 ****************************************************************************/

static void macnet_ind_orphan(MACHANDLE mac, FAR uint8_t *orphanaddr)
{
  FAR struct macnet_callback_s *cb = (FAR struct macnet_callback_s *)mac;
  FAR struct macnet_driver_s *priv;

  DEBUGASSERT(cb != NULL && cb->mc_priv != NULL);
  priv = cb->mc_priv;
}

/****************************************************************************
 * Name: macnet_ind_commstatus
 *
 * Description:
 *
 ****************************************************************************/

static void macnet_ind_commstatus(MACHANDLE mac, uint16_t panid,
                                  FAR uint8_t *src, FAR uint8_t *dst,
                                  int status)
{
  FAR struct macnet_callback_s *cb = (FAR struct macnet_callback_s *)mac;
  FAR struct macnet_driver_s *priv;

  DEBUGASSERT(cb != NULL && cb->mc_priv != NULL);
  priv = cb->mc_priv;
}

/****************************************************************************
 * Name: macnet_ind_syncloss
 *
 * Description:
 *
 ****************************************************************************/

static void macnet_ind_syncloss(MACHANDLE mac, int reason)
{
  FAR struct macnet_callback_s *cb = (FAR struct macnet_callback_s *)mac;
  FAR struct macnet_driver_s *priv;

  DEBUGASSERT(cb != NULL && cb->mc_priv != NULL);
  priv = cb->mc_priv;
}

/****************************************************************************
 * Name: macnet_transmit
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

static int macnet_transmit(FAR struct macnet_driver_s *priv)
{
  /* Verify that the hardware is ready to send another packet.  If we get
   * here, then we are committed to sending a packet; Higher level logic
   * must have assured that there is no transmission in progress.
   */

  /* Increment statistics */

  NETDEV_TXPACKETS(priv->md_dev);

  /* Send the packet: address=priv->md_dev.d_buf, length=priv->md_dev.d_len */

  /* Enable Tx interrupts */

  /* Setup the TX timeout watchdog (perhaps restarting the timer) */

  (void)wd_start(priv->md_txtimeout, skeleton_TXTIMEOUT,
                 macnet_txtimeout_expiry, 1, (wdparm_t)priv);
  return OK;
}

/****************************************************************************
 * Name: macnet_txpoll
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

static int macnet_txpoll(FAR struct net_driver_s *dev)
{
  FAR struct macnet_driver_s *priv = (FAR struct macnet_driver_s *)dev->d_private;

  /* If the polling resulted in data that should be sent out, the field
   * i_framelist will set to a new, outgoing list of frames.
   */

  if (priv->md_dev.i_framelist != NULL)
    {
       /* And send the packet */

       macnet_transmit(priv);
    }

  /* If zero is returned, the polling will continue until all connections have
   * been examined.
   */

  return 0;
}

/****************************************************************************
 * Name: macnet_receive
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

static void macnet_receive(FAR struct macnet_driver_s *priv)
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

  priv->md_dev.framelist = iob;
  sixlowpan_input(&priv->md_dev);

  /* If the above function invocation resulted in data that should be sent
   * out, the field i_framelist will set to a new, outgoing list of frames.
   */

  if (priv->md_dev.i_framelist != NULL)
    {
       /* And send the packet */

       macnet_transmit(priv);
    }
}

/****************************************************************************
 * Name: macnet_txdone
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

static void macnet_txdone(FAR struct macnet_driver_s *priv)
{
  int delay;

  /* Check for errors and update statistics */

  NETDEV_TXDONE(priv->md_dev);

  /* Check if there are pending transmissions */

  /* If no further transmissions are pending, then cancel the TX timeout and
   * disable further Tx interrupts.
   */

  wd_cancel(priv->md_txtimeout);

  /* And disable further TX interrupts. */

  /* In any event, poll the network for new TX data */

  (void)devif_poll(&priv->md_dev, macnet_txpoll);
}

/****************************************************************************
 * Name: macnet_transfer_work
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

static void macnet_transfer_work(FAR void *arg)
{
  FAR struct macnet_driver_s *priv = (FAR struct macnet_driver_s *)arg;

  /* Lock the network and serialize driver operations if necessary.
   * NOTE: Serialization is only required in the case where the driver work
   * is performed on an LP worker thread and where more than one LP worker
   * thread has been configured.
   */

  net_lock();

  /* Process pending Ethernet interrupts */

  /* Get and clear interrupt status bits */

  /* Handle interrupts according to status bit settings */

  /* Check if we received an incoming packet, if so, call macnet_receive() */

  macnet_receive(priv);

  /* Check if a packet transmission just completed.  If so, call macnet_txdone.
   * This may disable further Tx interrupts if there are no pending
   * transmissions.
   */

  macnet_txdone(priv);
  net_unlock();

  /* Re-enable Ethernet interrupts */

  up_enable_irq(CONFIG_IEEE802154_NETDEV_IRQ);
}

/****************************************************************************
 * Name: macnet_transfer
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

static int macnet_transfer(int irq, FAR void *context, FAR void *arg)
{
  FAR struct macnet_driver_s *priv = (FAR struct macnet_driver_s *)arg;

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

       wd_cancel(priv->md_txtimeout);
    }

  /* Schedule to perform the interrupt processing on the worker thread. */

  work_queue(ETHWORK, &priv->md_irqwork, macnet_transfer_work, priv, 0);
  return OK;
}

/****************************************************************************
 * Name: macnet_txtimeout_work
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

static void macnet_txtimeout_work(FAR void *arg)
{
  FAR struct macnet_driver_s *priv = (FAR struct macnet_driver_s *)arg;

  /* Lock the network and serialize driver operations if necessary.
   * NOTE: Serialization is only required in the case where the driver work
   * is performed on an LP worker thread and where more than one LP worker
   * thread has been configured.
   */

  net_lock();

  /* Increment statistics and dump debug info */

  NETDEV_TXTIMEOUTS(priv->md_dev);

  /* Then reset the hardware */

  /* Then poll the network for new XMIT data */

  (void)devif_poll(&priv->md_dev, macnet_txpoll);
  net_unlock();
}

/****************************************************************************
 * Name: macnet_txtimeout_expiry
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

static void macnet_txtimeout_expiry(int argc, wdparm_t arg, ...)
{
  FAR struct macnet_driver_s *priv = (FAR struct macnet_driver_s *)arg;

  /* Disable further Ethernet interrupts.  This will prevent some race
   * conditions with interrupt work.  There is still a potential race
   * condition with interrupt work that is already queued and in progress.
   */

  up_disable_irq(CONFIG_IEEE802154_NETDEV_IRQ);

  /* Schedule to perform the TX timeout processing on the worker thread. */

  work_queue(ETHWORK, &priv->md_irqwork, macnet_txtimeout_work, priv, 0);
}

/****************************************************************************
 * Name: macnet_poll_process
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

static inline void macnet_poll_process(FAR struct macnet_driver_s *priv)
{
}

/****************************************************************************
 * Name: macnet_poll_work
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

static void macnet_poll_work(FAR void *arg)
{
  FAR struct macnet_driver_s *priv = (FAR struct macnet_driver_s *)arg;

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

  (void)devif_timer(&priv->md_dev, macnet_txpoll);

  /* Setup the watchdog poll timer again */

  (void)wd_start(priv->md_txpoll, skeleton_WDDELAY, macnet_poll_expiry, 1,
                 (wdparm_t)priv);
  net_unlock();
}

/****************************************************************************
 * Name: macnet_poll_expiry
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

static void macnet_poll_expiry(int argc, wdparm_t arg, ...)
{
  FAR struct macnet_driver_s *priv = (FAR struct macnet_driver_s *)arg;

  /* Schedule to perform the interrupt processing on the worker thread. */

  work_queue(ETHWORK, &priv->md_pollwork, macnet_poll_work, priv, 0);
}

/****************************************************************************
 * Name: macnet_ifup
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

static int macnet_ifup(FAR struct net_driver_s *dev)
{
  FAR struct macnet_driver_s *priv = (FAR struct macnet_driver_s *)dev->d_private;

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

  /* Instantiate the MAC address from priv->md_dev.d_mac.ether_addr_octet */

#ifdef CONFIG_NET_ICMPv6
  /* Set up IPv6 multicast address filtering */

  macnet_ipv6multicast(priv);
#endif

  /* Set and activate a timer process */

  (void)wd_start(priv->md_txpoll, skeleton_WDDELAY, macnet_poll_expiry, 1,
                 (wdparm_t)priv);

  /* Enable the Ethernet interrupt */

  priv->md_bifup = true;
  up_enable_irq(CONFIG_IEEE802154_NETDEV_IRQ);
  return OK;
}

/****************************************************************************
 * Name: macnet_ifdown
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

static int macnet_ifdown(FAR struct net_driver_s *dev)
{
  FAR struct macnet_driver_s *priv = (FAR struct macnet_driver_s *)dev->d_private;
  irqstate_t flags;

  /* Disable the Ethernet interrupt */

  flags = enter_critical_section();
  up_disable_irq(CONFIG_IEEE802154_NETDEV_IRQ);

  /* Cancel the TX poll timer and TX timeout timers */

  wd_cancel(priv->md_txpoll);
  wd_cancel(priv->md_txtimeout);

  /* Put the EMAC in its reset, non-operational state.  This should be
   * a known configuration that will guarantee the macnet_ifup() always
   * successfully brings the interface back up.
   */

  /* Mark the device "down" */

  priv->md_bifup = false;
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: macnet_txavail_work
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

static void macnet_txavail_work(FAR void *arg)
{
  FAR struct macnet_driver_s *priv = (FAR struct macnet_driver_s *)arg;

  /* Lock the network and serialize driver operations if necessary.
   * NOTE: Serialization is only required in the case where the driver work
   * is performed on an LP worker thread and where more than one LP worker
   * thread has been configured.
   */

  net_lock();

  /* Ignore the notification if the interface is not yet up */

  if (priv->md_bifup)
    {
      /* Check if there is room in the hardware to hold another outgoing packet. */

      /* If so, then poll the network for new XMIT data */

      (void)devif_poll(&priv->md_dev, macnet_txpoll);
    }

  net_unlock();
}

/****************************************************************************
 * Name: macnet_txavail
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

static int macnet_txavail(FAR struct net_driver_s *dev)
{
  FAR struct macnet_driver_s *priv = (FAR struct macnet_driver_s *)dev->d_private;

  /* Is our single work structure available?  It may not be if there are
   * pending interrupt actions and we will have to ignore the Tx
   * availability action.
   */

  if (work_available(&priv->md_pollwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      work_queue(ETHWORK, &priv->md_pollwork, macnet_txavail_work, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: macnet_addmac
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
static int macnet_addmac(FAR struct net_driver_s *dev, FAR const uint8_t *mac)
{
  FAR struct macnet_driver_s *priv = (FAR struct macnet_driver_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

  return OK;
}
#endif

/****************************************************************************
 * Name: macnet_rmmac
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
static int macnet_rmmac(FAR struct net_driver_s *dev, FAR const uint8_t *mac)
{
  FAR struct macnet_driver_s *priv = (FAR struct macnet_driver_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

  return OK;
}
#endif

/****************************************************************************
 * Name: macnet_ipv6multicast
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
static void macnet_ipv6multicast(FAR struct macnet_driver_s *priv)
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

  (void)macnet_addmac(dev, mac);

#ifdef CONFIG_NET_ICMPv6_AUTOCONF
  /* Add the IPv6 all link-local nodes Ethernet address.  This is the
   * address that we expect to receive ICMPv6 Router Advertisement
   * packets.
   */

  (void)macnet_addmac(dev, g_ipv6_ethallnodes.ether_addr_octet);

#endif /* CONFIG_NET_ICMPv6_AUTOCONF */

#ifdef CONFIG_NET_ICMPv6_ROUTER
  /* Add the IPv6 all link-local routers Ethernet address.  This is the
   * address that we expect to receive ICMPv6 Router Solicitation
   * packets.
   */

  (void)macnet_addmac(dev, g_ipv6_ethallrouters.ether_addr_octet);

#endif /* CONFIG_NET_ICMPv6_ROUTER */
}
#endif /* CONFIG_NET_ICMPv6 */

/****************************************************************************
 * Name: macnet_ioctl
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
static int macnet_ioctl(FAR struct net_driver_s *dev, int cmd, long arg)
{
  FAR struct macnet_driver_s *priv = (FAR struct macnet_driver_s *)dev->d_private;
  int ret = -EINVAL;

  /* Check for IOCTLs aimed at the IEEE802.15.4 MAC layer */

  if (_MAC802154IOCVALID(cmd))
    {
      FAR struct ieee802154_netmac_s *netmac =
        (FAR struct ieee802154_netmac_s *)arg;

      if (netmac != NULL)
        {
          unsigned long macarg = (unsigned int)((uintptr_t)&netmac->u);
          ret = priv->md_mac.macops.ioctl(priv->md_mac, cmd, macarg);
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
          ret = priv->md_mac.macops.ioctl(priv->md_mac, cmd, radioarg);
        }
    }

  /* Okay, we have no idea what this command is.. just give to the
   * IEEE802.15.4 MAC layer without modification.
   */

  else
   {
     ret = priv->md_mac.macops.ioctl(priv->md_mac, cmd, arg);
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

int mac802154netdev_register(MACHANDLE mac)
{
  FAR struct macnet_driver_s *priv;
  FAR struct net_driver_s *dev;
  FAR uint8_t *pktbuf;
  int ret;

  DEBUGASSERT(radio != NULL);

  /* Get the interface structure associated with this interface number. */

  priv = (FAR struct macnet_driver_s *)
    kmm_zalloc(sizeof(struct macnet_driver_s));

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

  dev                = &ieee->md_dev.i_dev;
  dev->d_buf         = pktbuf;             /* Single packet buffer */
  dev->d_ifup        = macnet_ifup;     /* I/F up (new IP address) callback */
  dev->d_ifdown      = macnet_ifdown;   /* I/F down callback */
  dev->d_txavail     = macnet_txavail;  /* New TX data callback */
#ifdef CONFIG_NET_IGMP
  dev->d_addmac      = macnet_addmac;   /* Add multicast MAC address */
  dev->d_rmmac       = macnet_rmmac;    /* Remove multicast MAC address */
#endif
 #ifdef CONFIG_NETDEV_IOCTL
  dev->d_ioctl       = macnet_ioctl;    /* Handle network IOCTL commands */
#endif
  dev->d_private     = (FAR void *)priv;   /* Used to recover private state from dev */

  /* Create a watchdog for timing polling for and timing of transmisstions */

  priv->md_mac       = mac;                /* Save the MAC interface instance */
  priv->md_txpoll    = wd_create();        /* Create periodic poll timer */
  priv->md_txtimeout = wd_create();        /* Create TX timeout timer */

  DEBUGASSERT(priv->md_txpoll != NULL && priv->md_txtimeout != NULL);

  /* Initialize the MAC callbacks */

  priv->md_cb.mc_priv      = priv;

  maccb                    = &priv->md_cb.mc_cb;
  maccb->conf_data         = macnet_conf_data;
  maccb->conf_purge        = macnet_conf_purge;
  maccb->conf_associate    = macnet_conf_associate;
  maccb->conf_disassociate = macnet_conf_disassociate;
  maccb->conf_get          = macnet_conf_get;
  maccb->conf_gts          = macnet_conf_gts;
  maccb->conf_reset        = macnet_conf_reset;
  maccb->conf_rxenable     = macnet_conf_rxenable;
  maccb->conf_scan         = macnet_conf_scan;
  maccb->conf_set          = macnet_conf_set;
  maccb->conf_start        = macnet_conf_start;
  maccb->conf_poll         = macnet_conf_poll;
  maccb->ind_data          = macnet_ind_data;
  maccb->ind_associate     = macnet_ind_associate;
  maccb->ind_disassociate  = macnet_ind_disassociate;
  maccb->ind_beaconnotify  = macnet_ind_beaconnotify;
  maccb->ind_gts           = macnet_ind_gts;
  maccb->ind_orphan        = macnet_ind_orphan;
  maccb->ind_commstatus    = macnet_ind_commstatus;
  maccb->ind_syncloss      = macnet_ind_syncloss;

  /* Bind the callback structure */

  ret = mac->ops->bind(mac, *priv->md_cb.mc_cb);
  if (ret < 0)
    {
      nerr("ERROR: Failed to bind the MAC callbacks: %d\n", ret);

      /* Release wdog timers */

      wd_delete(priv->md_txpoll);
      wd_delete(priv->md_txtimeout);

      /* Free memory and return the error */
      kmm_free(pktbuf);
      kmm_free(priv);
      return ret
    }

  /* Put the interface in the down state. */

  macnet_ifdown(dev);

  /* Register the device with the OS so that socket IOCTLs can be performed */

  (void)netdev_register(&priv->md_dev, NET_LL_IEEE802154);
  return OK;
}

#endif /* CONFIG_NET && CONFIG_NET_skeleton */
