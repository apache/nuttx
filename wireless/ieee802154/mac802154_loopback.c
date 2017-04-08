/****************************************************************************
 * wireless/iee802154/mac802154_loopback.c
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
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <arpa/inet.h>
#include <net/if.h>

#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>
#include <nuttx/net/net.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/sixlowpan.h>
#include <nuttx/wireless/ieee802154/ieee802154_loopback.h>

#ifdef CONFIG_IEEE802154_LOOPBACK

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* We need to have the work queue to handle SPI interrupts */

#if !defined(CONFIG_SCHED_WORKQUEUE)
#  error Worker thread support is required (CONFIG_SCHED_WORKQUEUE)
#else
#  if defined(CONFIG_IEEE802154_LOOPBACK_HPWORK)
#    define LPBKWORK HPWORK
#  elif defined(CONFIG_IEEE802154_LOOPBACK_LPWORK)
#    define LPBKWORK LPWORK
#  else
#    error Neither CONFIG_IEEE802154_LOOPBACK_HPWORK nor CONFIG_IEEE802154_LOOPBACK_LPWORK defined
#  endif
#endif

/* TX poll delay = 1 seconds. CLK_TCK is the number of clock ticks per second */

#define LO_WDDELAY   (1*CLK_TCK)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The lo_driver_s encapsulates all state information for a single hardware
 * interface
 */

struct lo_driver_s
{
  bool lo_bifup;               /* true:ifup false:ifdown */
  bool lo_txdone;              /* One RX packet was looped back */
  WDOG_ID lo_polldog;          /* TX poll timer */
  struct work_s lo_work;       /* For deferring poll work to the work queue */

  /* This holds the information visible to the NuttX network */

  struct ieee802154_driver_s lo_ieee;  /* Interface understood by the network */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct lo_driver_s g_loopback;
static uint8_t g_iobuffer[CONFIG_NET_6LOWPAN_MTU + CONFIG_NET_GUARDSIZE];

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Polling logic */

static int  lo_txpoll(FAR struct net_driver_s *dev);
static void lo_poll_work(FAR void *arg);
static void lo_poll_expiry(int argc, wdparm_t arg, ...);

/* NuttX callback functions */

static int lo_ifup(FAR struct net_driver_s *dev);
static int lo_ifdown(FAR struct net_driver_s *dev);
static void lo_txavail_work(FAR void *arg);
static int lo_txavail(FAR struct net_driver_s *dev);
#if defined(CONFIG_NET_IGMP) || defined(CONFIG_NET_ICMPv6)
static int lo_addmac(FAR struct net_driver_s *dev, FAR const uint8_t *mac);
#ifdef CONFIG_NET_IGMP
static int lo_rmmac(FAR struct net_driver_s *dev, FAR const uint8_t *mac);
#endif
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: lo_txpoll
 *
 * Description:
 *   Check if the network has any outgoing packets ready to send.  This is
 *   a callback from devif_poll() or devif_timer().  devif_poll() will be
 *   called only during normal TX polling.
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

static int lo_txpoll(FAR struct net_driver_s *dev)
{
  FAR struct lo_driver_s *priv = (FAR struct lo_driver_s *)dev->d_private;
  FAR struct iob_s *head;
  FAR struct iob_s *tail;
  FAR struct iob_s *iob;
  int ret;

  if (dev->d_len > 0 || priv->lo_ieee.i_framelist != NULL)
    {
      ninfo("d_len: %u i_framelist: %p\n",
            dev->d_len, priv->lo_ieee.i_framelist);

      /* The only two valid settings are:
       *
       * 1. Nothing to send:
       *    dev->d_len == 0 && priv->lo_ieee.i_framelist == NULL
       * 2. Outgoing packet has been converted to IEEE802.15.4 frames:
       *    dev->d_len == 0 && priv->lo_ieee.i_framelist != NULL
       */

      DEBUGASSERT(dev->d_len == 0 && priv->lo_ieee.i_framelist != NULL);
    }

  /* Remove the queued IOBs from driver structure */

  head = priv->lo_ieee.i_framelist;

  /* Find the tail of the IOB queue */

  for (tail = NULL, iob = head;
       iob != NULL;
       tail = iob, iob = iob->io_flink);

  /* Loop while there frames to be sent, i.e., while the IOB list is not
   * emtpy. Sending, of course, just means relaying back through the network
   * for this driver.
   */

  while (head != NULL)
    {
      /* Increment statistics */

      NETDEV_RXPACKETS(&priv->lo_ieee.i_dev);

      /* Remove the IOB from the queue */

      iob           = head;
      head          = iob->io_flink;
      iob->io_flink = NULL;

      /* Is the queue now empty? */

      if (head == NULL)
        {
          tail = NULL;
        }

      /* Return the next frame to the network */

      iob->io_flink      = NULL;
      priv->lo_ieee.i_framelist = iob;

      ninfo("Send frame %p to the network. Length=%u\n", iob, iob->io_len);
      ret = sixlowpan_input(&priv->lo_ieee);

      /* Increment statistics */

      NETDEV_TXPACKETS(&priv->lo_ieee.i_dev);

      if (ret < 0)
        {
          nerr("ERROR: sixlowpan_input returned %d\n", ret);
          NETDEV_TXERRORS(&priv->lo_ieee.i_dev);
          NETDEV_ERRORS(&priv->lo_ieee.i_dev);
        }

      /* What if the network responds with more frames to send? */

      if (priv->lo_ieee.i_framelist != NULL)
        {
          /* Append the new list to the tail of the queue */

          iob                       = priv->lo_ieee.i_framelist;
          priv->lo_ieee.i_framelist = NULL;

          if (tail == NULL)
            {
              head = iob;
            }
          else
            {
              tail->io_flink = iob;
            }

          /* Find the new tail of the IOB queue */

          for (tail = iob, iob = iob->io_flink;
               iob != NULL;
               tail = iob, iob = iob->io_flink);
        }

      priv->lo_txdone = true;
    }

  return 0;
}

/****************************************************************************
 * Function: lo_poll_work
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
 *   The network is locked
 *
 ****************************************************************************/

static void lo_poll_work(FAR void *arg)
{
  FAR struct lo_driver_s *priv = (FAR struct lo_driver_s *)arg;

  /* Perform the poll */

  net_lock();
  priv->lo_txdone = false;
  (void)devif_timer(&priv->lo_ieee.i_dev, lo_txpoll);

  /* Was something received and looped back? */

  while (priv->lo_txdone)
    {
      /* Yes, poll again for more TX data */

      priv->lo_txdone = false;
      (void)devif_poll(&priv->lo_ieee.i_dev, lo_txpoll);
    }

  /* Setup the watchdog poll timer again */

  (void)wd_start(priv->lo_polldog, LO_WDDELAY, lo_poll_expiry, 1, priv);
  net_unlock();
}

/****************************************************************************
 * Function: lo_poll_expiry
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
 *   The network is locked.
 *
 ****************************************************************************/

static void lo_poll_expiry(int argc, wdparm_t arg, ...)
{
  FAR struct lo_driver_s *priv = (FAR struct lo_driver_s *)arg;

  if (!work_available(&priv->lo_work))
    {
      nwarn("WARNING: lo_work NOT available\n");
    }

  /* Schedule to perform the interrupt processing on the worker thread. */

  work_queue(LPBKWORK, &priv->lo_work, lo_poll_work, priv, 0);
}

/****************************************************************************
 * Function: lo_ifup
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

static int lo_ifup(FAR struct net_driver_s *dev)
{
  FAR struct lo_driver_s *priv = (FAR struct lo_driver_s *)dev->d_private;

  ninfo("Bringing up: IPv6 %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        dev->d_ipv6addr[0], dev->d_ipv6addr[1], dev->d_ipv6addr[2],
        dev->d_ipv6addr[3], dev->d_ipv6addr[4], dev->d_ipv6addr[5],
        dev->d_ipv6addr[6], dev->d_ipv6addr[7]);

#ifdef CONFIG_NET_6LOWPAN_RIMEADDR_EXTENDED
  ninfo("             Node: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x PANID=%04x\n",
         priv->lo_ieee.i_nodeaddr.u8[0], priv->lo_ieee.i_nodeaddr.u8[1],
         priv->lo_ieee.i_nodeaddr.u8[2], priv->lo_ieee.i_nodeaddr.u8[3],
         priv->lo_ieee.i_nodeaddr.u8[4], priv->lo_ieee.i_nodeaddr.u8[5],
         priv->lo_ieee.i_nodeaddr.u8[6], priv->lo_ieee.i_nodeaddr.u8[7],
         priv->lo_ieee.i_panid);
#else
  ninfo("             Node: %02x:%02x PANID=%04x\n",
         priv->lo_ieee.i_nodeaddr.u8[0], priv->lo_ieee.i_nodeaddr.u8[1],
         priv->lo_ieee.i_panid);
#endif

  /* Set and activate a timer process */

  (void)wd_start(priv->lo_polldog, LO_WDDELAY, lo_poll_expiry,
                 1, (wdparm_t)priv);

  priv->lo_bifup = true;
  return OK;
}

/****************************************************************************
 * Function: lo_ifdown
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

static int lo_ifdown(FAR struct net_driver_s *dev)
{
  FAR struct lo_driver_s *priv = (FAR struct lo_driver_s *)dev->d_private;

  ninfo("IP up: %u\n", priv->lo_bifup);

  /* Cancel the TX poll timer and TX timeout timers */

  wd_cancel(priv->lo_polldog);

  /* Mark the device "down" */

  priv->lo_bifup = false;
  return OK;
}

/****************************************************************************
 * Function: lo_txavail_work
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

static void lo_txavail_work(FAR void *arg)
{
  FAR struct lo_driver_s *priv = (FAR struct lo_driver_s *)arg;

  ninfo("IP up: %u\n", priv->lo_bifup);

  /* Ignore the notification if the interface is not yet up */

  net_lock();
  if (priv->lo_bifup)
    {
      do
        {
          /* If so, then poll the network for new XMIT data */

          priv->lo_txdone = false;
          (void)devif_poll(&priv->lo_ieee.i_dev, lo_txpoll);
        }
      while (priv->lo_txdone);
    }

  net_unlock();
}

/****************************************************************************
 * Function: lo_txavail
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

static int lo_txavail(FAR struct net_driver_s *dev)
{
  FAR struct lo_driver_s *priv = (FAR struct lo_driver_s *)dev->d_private;

  ninfo("Available: %u\n", work_available(&priv->lo_work));

  /* Is our single work structure available?  It may not be if there are
   * pending interrupt actions and we will have to ignore the Tx
   * availability action.
   */

  if (work_available(&priv->lo_work))
    {
      /* Schedule to serialize the poll on the worker thread. */

      work_queue(LPBKWORK, &priv->lo_work, lo_txavail_work, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Function: lo_addmac
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
static int lo_addmac(FAR struct net_driver_s *dev, FAR const uint8_t *mac)
{
#ifdef CONFIG_NET_6LOWPAN_RIMEADDR_EXTENDED
  ninfo("MAC: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n",
         mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], mac[6], mac[7]);
#else
  ninfo("MAC: %02x:%02x\n",
         mac[0], mac[1]);
#endif

  /* There is no multicast support in the loopback driver */

  return OK;
}
#endif

/****************************************************************************
 * Function: lo_rmmac
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
static int lo_rmmac(FAR struct net_driver_s *dev, FAR const uint8_t *mac)
{
#ifdef CONFIG_NET_6LOWPAN_RIMEADDR_EXTENDED
  ninfo("MAC: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n",
         mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], mac[6], mac[7]);
#else
  ninfo("MAC: %02x:%02x\n",
         mac[0], mac[1]);
#endif

  /* There is no multicast support in the loopback driver */

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: ieee8021514_loopback
 *
 * Description:
 *   Initialize and register the Ieee802.15.4 MAC loopback network driver.
 *
 * Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int ieee8021514_loopback(void)
{
  FAR struct lo_driver_s  *priv;
  FAR struct net_driver_s *dev;

  ninfo("Initializing\n");

  /* Get the interface structure associated with this interface number. */

  priv = &g_loopback;

  /* Initialize the driver structure */

  memset(priv, 0, sizeof(struct lo_driver_s));

  dev = &priv->lo_ieee.i_dev;
  dev->d_ifup    = lo_ifup;       /* I/F up (new IP address) callback */
  dev->d_ifdown  = lo_ifdown;     /* I/F down callback */
  dev->d_txavail = lo_txavail;    /* New TX data callback */
#ifdef CONFIG_NET_IGMP
  dev->d_addmac  = lo_addmac;     /* Add multicast MAC address */
  dev->d_rmmac   = lo_rmmac;      /* Remove multicast MAC address */
#endif
  dev->d_buf     = g_iobuffer;    /* Attach the IO buffer */
  dev->d_private = (FAR void *)priv; /* Used to recover private state from dev */

  /* Create a watchdog for timing polling for and timing of transmissions */

  priv->lo_polldog = wd_create(); /* Create periodic poll timer */

  /* Register the loopabck device with the OS so that socket IOCTLs can b
   * performed.
   */

  (void)netdev_register(&priv->lo_ieee.i_dev, NET_LL_IEEE802154);

  /* Put the network in the UP state */

  dev->d_flags = IFF_UP;
  return lo_ifup(&priv->lo_ieee.i_dev);
}

#endif /* CONFIG_IEEE802154_LOOPBACK */
