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
#include <nuttx/kmalloc.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>
#include <nuttx/mm/iob.h>
#include <nuttx/net/arp.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/sixlowpan.h>
#include <nuttx/wireless/ieee802154/ieee802154_mac.h>

#include "mac802154.h"

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
#     define WPANWORK HPWORK
#  elif defined(CONFIG_IEEE802154_NETDEV_LPWORK)
#     define WPANWORK LPWORK
#  else
#     error Neither CONFIG_IEEE802154_NETDEV_HPWORK nor CONFIG_IEEE802154_NETDEV_LPWORK defined
#  endif
#endif

/* CONFIG_IEEE802154_NETDEV_NINTERFACES determines the number of physical interfaces
 * that will be supported.
 */

#ifndef CONFIG_IEEE802154_NETDEV_NINTERFACES
# define CONFIG_IEEE802154_NETDEV_NINTERFACES 1
#endif

/* TX poll delay = 1 seconds. CLK_TCK is the number of clock ticks per second */

#define TXPOLL_WDDELAY   (1*CLK_TCK)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This is our private version of the MAC callback stucture */

struct macnet_callback_s
{
  /* This holds the information visible to the MAC layer */

  struct mac802154_maccb_s mc_cb;        /* Interface understood by the MAC layer */
  FAR struct macnet_driver_s *mc_priv; /* Our priv data */
};

/* The macnet_driver_s encapsulates all state information for a single
 * IEEE802.15.4 MAC interface.
 */

struct macnet_driver_s
{
  /* This holds the information visible to the NuttX network */

  struct ieee802154_driver_s md_dev;  /* Interface understood by the network */

  /* For internal use by this driver */

  struct macnet_callback_s md_cb; /* Callback information */
  MACHANDLE md_mac;               /* Contained MAC interface */
  bool md_bifup;                  /* true:ifup false:ifdown */
  WDOG_ID md_txpoll;              /* TX poll timer */
  struct work_s md_pollwork;      /* Defer poll work to the work queue */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* IEE802.15.4 MAC callback functions ***************************************/

static void macnet_notify(FAR const struct mac802154_maccb_s *maccb,
                          FAR struct ieee802154_notif_s *notif);
static void macnet_rxframe(FAR const struct mac802154_maccb_s *maccb,
                           FAR struct ieee802154_data_ind_s *ind);

/* Asynchronous confirmations to requests */

static void macnet_conf_data(FAR struct macnet_driver_s *priv,
             FAR const struct ieee802154_data_conf_s *conf);
static void macnet_conf_associate(FAR struct macnet_driver_s *priv,
             FAR struct ieee802154_assoc_conf_s *conf);
static void macnet_conf_disassociate(FAR struct macnet_driver_s *priv,
             FAR struct ieee802154_disassoc_conf_s *conf);
static void macnet_conf_gts(FAR struct macnet_driver_s *priv,
             FAR struct ieee802154_gts_conf_s *conf);
static void macnet_conf_rxenable(FAR struct macnet_driver_s *priv,
             FAR struct ieee802154_rxenable_conf_s *conf);
static void macnet_conf_scan(FAR struct macnet_driver_s *priv,
             FAR struct ieee802154_scan_conf_s *conf);
static void macnet_conf_start(FAR struct macnet_driver_s *priv,
             FAR struct ieee802154_start_conf_s *conf);
static void macnet_conf_poll(FAR struct macnet_driver_s *priv,
             FAR struct ieee802154_poll_conf_s *conf);

  /* Asynchronous event indications, replied to synchronously with responses */

static void macnet_ind_associate(FAR struct macnet_driver_s *priv,
             FAR struct ieee802154_assoc_ind_s *conf);
static void macnet_ind_disassociate(FAR struct macnet_driver_s *priv,
             FAR struct ieee802154_disassoc_ind_s *conf);
static void macnet_ind_beaconnotify(FAR struct macnet_driver_s *priv,
             FAR struct ieee802154_beaconnotify_ind_s *conf);
static void macnet_ind_gts(FAR struct macnet_driver_s *priv,
             FAR struct ieee802154_gts_ind_s *conf);
static void macnet_ind_orphan(FAR struct macnet_driver_s *priv,
             FAR struct ieee802154_orphan_ind_s *conf);
static void macnet_ind_commstatus(FAR struct macnet_driver_s *priv,
             FAR struct ieee802154_commstatus_ind_s *conf);
static void macnet_ind_syncloss(FAR struct macnet_driver_s *priv,
             FAR struct ieee802154_syncloss_ind_s *conf);

/* Network interface support ************************************************/
/* Common TX logic */

static int  macnet_txpoll_callback(FAR struct net_driver_s *dev);
static void macnet_txpoll_work(FAR void *arg);
static void macnet_txpoll_expiry(int argc, wdparm_t arg, ...);

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
#endif
#ifdef CONFIG_NETDEV_IOCTL
static int  macnet_ioctl(FAR struct net_driver_s *dev, int cmd,
              unsigned long arg);
#endif
static int macnet_get_mhrlen(FAR struct ieee802154_driver_s *netdev,
              FAR const struct ieee802154_frame_meta_s *meta);
static int macnet_req_data(FAR struct ieee802154_driver_s *netdev,
              FAR const struct ieee802154_frame_meta_s *meta,
              FAR struct iob_s *framelist);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: macnet_notify
 *
 * Description:
 *
 ****************************************************************************/

static void macnet_notify(FAR const struct mac802154_maccb_s *maccb,
                          FAR struct ieee802154_notif_s *notif)
{
  FAR struct macnet_callback_s *cb =
    (FAR struct macnet_callback_s *)maccb;
  FAR struct macnet_driver_s *priv;

  DEBUGASSERT(cb != NULL && cb->mc_priv != NULL);
  priv = cb->mc_priv;

  switch (notif->notiftype)
    {
      case IEEE802154_NOTIFY_CONF_DATA:
        {
          macnet_conf_data(priv, &notif->u.dataconf);
        }
        break;

      default:
        break;
    }
}

/****************************************************************************
 * Name: macnet_rxframe
 *
 * Description:
 *
 ****************************************************************************/

static void macnet_rxframe(FAR const struct mac802154_maccb_s *maccb,
                           FAR struct ieee802154_data_ind_s *ind)
{
  FAR struct macnet_callback_s *cb =
    (FAR struct macnet_callback_s *)maccb;
  FAR struct macnet_driver_s *priv;
  FAR struct iob_s *iob;

  DEBUGASSERT(cb != NULL && cb->mc_priv != NULL);
  priv = cb->mc_priv;

  /* Extract the IOB containing the frame from the struct ieee802154_data_ind_s */

  DEBUGASSERT(priv != NULL && ind != NULL && ind->frame != NULL);
  iob        = ind->frame;
  ind->frame = NULL;

  /* Transfer the frame to the network logic */

  sixlowpan_input(&priv->md_dev, iob, ind);

  /* sixlowpan_input() will free the IOB, but we must free the struct
   * ieee802154_data_ind_s container here.
   */

  ieee802154_ind_free(ind);
}

/****************************************************************************
 * Name: macnet_conf_data
 *
 * Description:
 *   Data frame was received by remote device
 *
 ****************************************************************************/

static void macnet_conf_data(FAR struct macnet_driver_s *priv,
                             FAR const struct ieee802154_data_conf_s *conf)
{

}

/****************************************************************************
 * Name: macnet_conf_associate
 *
 * Description:
 *   Association request completed
 *
 ****************************************************************************/

static void macnet_conf_associate(FAR struct macnet_driver_s *priv,
                                  FAR struct ieee802154_assoc_conf_s *conf)
{

}

/****************************************************************************
 * Name: macnet_conf_disassociate
 *
 * Description:
 *   Disassociation request completed
 *
 ****************************************************************************/

static void macnet_conf_disassociate(FAR struct macnet_driver_s *priv,
                                     FAR struct ieee802154_disassoc_conf_s *conf)
{

}

/****************************************************************************
 * Name: macnet_conf_gts
 *
 * Description:
 *   GTS management completed
 *
 ****************************************************************************/

static void macnet_conf_gts(FAR struct macnet_driver_s *priv,
                            FAR struct ieee802154_gts_conf_s *conf)
{

}

/****************************************************************************
 * Name: macnet_conf_rxenable
 *
 * Description:
 *
 ****************************************************************************/

static void macnet_conf_rxenable(FAR struct macnet_driver_s *priv,
                                 FAR struct ieee802154_rxenable_conf_s *conf)
{

}

/****************************************************************************
 * Name: macnet_conf_scan
 *
 * Description:
 *
 ****************************************************************************/

static void macnet_conf_scan(FAR struct macnet_driver_s *priv,
                             FAR struct ieee802154_scan_conf_s *conf)
{

}

/****************************************************************************
 * Name: macnet_conf_start
 *
 * Description:
 *
 ****************************************************************************/

static void macnet_conf_start(FAR struct macnet_driver_s *priv,
                              FAR struct ieee802154_start_conf_s *conf)
{

}

/****************************************************************************
 * Name: macnet_conf_poll
 *
 * Description:
 *
 ****************************************************************************/

static void macnet_conf_poll(FAR struct macnet_driver_s *priv,
                             FAR struct ieee802154_poll_conf_s *conf)
{

}

/****************************************************************************
 * Name: macnet_ind_associate
 *
 * Description:
 *   Association request received
 *
 ****************************************************************************/

static void macnet_ind_associate(FAR struct macnet_driver_s *priv,
                                 FAR struct ieee802154_assoc_ind_s *ind)
{

}

/****************************************************************************
 * Name: macnet_ind_disassociate
 *
 * Description:
 *   Disassociation request received
 *
 ****************************************************************************/

static void macnet_ind_disassociate(FAR struct macnet_driver_s *priv,
                                    FAR struct ieee802154_disassoc_ind_s *ind)
{

}

/****************************************************************************
 * Name: macnet_ind_beaconnotify
 *
 * Description:
 *    Beacon notification
 *
 ****************************************************************************/

static void macnet_ind_beaconnotify(FAR struct macnet_driver_s *priv,
                                    FAR struct ieee802154_beaconnotify_ind_s *ind)
{

}

/****************************************************************************
 * Name: macnet_ind_gts
 *
 * Description:
 *   GTS management request received
 *
 ****************************************************************************/

static void macnet_ind_gts(FAR struct macnet_driver_s *priv,
                           FAR struct ieee802154_gts_ind_s *ind)
{

}

/****************************************************************************
 * Name: macnet_ind_orphan
 *
 * Description:
 *   Orphan device detected
 *
 ****************************************************************************/

static void macnet_ind_orphan(FAR struct macnet_driver_s *priv,
                              FAR struct ieee802154_orphan_ind_s *ind)
{

}

/****************************************************************************
 * Name: macnet_ind_commstatus
 *
 * Description:
 *
 ****************************************************************************/

static void macnet_ind_commstatus(FAR struct macnet_driver_s *priv,
                                  FAR struct ieee802154_commstatus_ind_s *ind)
{

}

/****************************************************************************
 * Name: macnet_ind_syncloss
 *
 * Description:
 *
 ****************************************************************************/

static void macnet_ind_syncloss(FAR struct macnet_driver_s *priv,
                                FAR struct ieee802154_syncloss_ind_s *ind)
{

}

/****************************************************************************
 * Name: macnet_txpoll_callback
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
 *   The network is locked.
 *
 ****************************************************************************/

static int macnet_txpoll_callback(FAR struct net_driver_s *dev)
{
  /* If zero is returned, the polling will continue until all connections have
   * been examined.
   */

  return 0;
}

/****************************************************************************
 * Name: macnet_txpoll_process
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

static inline void macnet_txpoll_process(FAR struct macnet_driver_s *priv)
{
}

/****************************************************************************
 * Name: macnet_txpoll_work
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

static void macnet_txpoll_work(FAR void *arg)
{
  FAR struct macnet_driver_s *priv = (FAR struct macnet_driver_s *)arg;

  /* Lock the network and serialize driver operations if necessary.
   * NOTE: Serialization is only required in the case where the driver work
   * is performed on an LP worker thread and where more than one LP worker
   * thread has been configured.
   */

  net_lock();

  /* Perform the poll */

  (void)devif_timer(&priv->md_dev.i_dev, macnet_txpoll_callback);

  /* Setup the watchdog poll timer again */

  (void)wd_start(priv->md_txpoll, TXPOLL_WDDELAY, macnet_txpoll_expiry, 1,
                 (wdparm_t)priv);
  net_unlock();
}

/****************************************************************************
 * Name: macnet_txpoll_expiry
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

static void macnet_txpoll_expiry(int argc, wdparm_t arg, ...)
{
  FAR struct macnet_driver_s *priv = (FAR struct macnet_driver_s *)arg;

  /* Schedule to perform the interrupt processing on the worker thread. */

  work_queue(WPANWORK, &priv->md_pollwork, macnet_txpoll_work, priv, 0);
}

/****************************************************************************
 * Name: macnet_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the IEEE 802.15.4 interface when an IP address
 *   is provided
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

  /* Initialize PHYs, the IEEE 802.15.4 interface, and setup up IEEE 802.15.4 interrupts */
#warning Missing logic

  /* Setup up address filtering */

  /* Set and activate a timer process */

  (void)wd_start(priv->md_txpoll, TXPOLL_WDDELAY, macnet_txpoll_expiry, 1,
                 (wdparm_t)priv);

  /* Enable the IEEE 802.15.4 radio */

  priv->md_bifup = true;
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

  /* Disable interruption */

  flags = enter_critical_section();

  /* Cancel the TX poll timer and TX timeout timers */

  wd_cancel(priv->md_txpoll);

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

      (void)devif_poll(&priv->md_dev.i_dev, macnet_txpoll_callback);
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

      work_queue(WPANWORK, &priv->md_pollwork, macnet_txavail_work, priv, 0);
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

  /* Add the MAC address to the hardware multicast routing table.  Not used
   * with IEEE 802.15.4 radios.
   */

  return -ENOSYS;
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

  /* Remove the MAC address from the hardware multicast routing table  Not used
   * with IEEE 802.15.4 radios.
   */

  return -ENOSYS;
}
#endif

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
static int macnet_ioctl(FAR struct net_driver_s *dev, int cmd,
                        unsigned long arg)
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
          ret = mac802154_ioctl(priv->md_mac, cmd, macarg);

        }
    }

  /* Okay, we have no idea what this command is.. just give to the
   * IEEE802.15.4 MAC layer without modification.
   */

  else
   {
     ret = mac802154_ioctl(priv->md_mac, cmd, arg);
   }

 return ret;
}
#endif

/****************************************************************************
 * Name: macnet_get_mhrlen
 *
 * Description:
 *   Calculate the MAC header length given the frame meta-data.
 *
 * Input parameters:
 *   netdev    - The networkd device that will mediate the MAC interface
 *   meta      - Meta data needed to recreate the MAC header
 *
 * Returned Value:
 *   A non-negative MAC headeer length is returned on success; a negated
 *   errno value is returned on any failure.
 *
 ****************************************************************************/

static int macnet_get_mhrlen(FAR struct ieee802154_driver_s *netdev,
                             FAR const struct ieee802154_frame_meta_s *meta)
{
  FAR struct macnet_driver_s *priv;

  DEBUGASSERT(netdev != NULL && netdev->i_dev.d_private != NULL && meta != NULL);
  priv = (FAR struct macnet_driver_s *)netdev->i_dev.d_private;

  return mac802154_get_mhrlen(priv->md_mac, meta);
}

/****************************************************************************
 * Name: macnet_req_data
 *
 * Description:
 *   Requests the transfer of a list of frames to the MAC.
 *
 * Input parameters:
 *   netdev    - The networkd device that will mediate the MAC interface
 *   meta      - Meta data needed to recreate the MAC header
 *   framelist - Head of a list of frames to be transferred.
 *
 * Returned Value:
 *   Zero (OK) returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static int macnet_req_data(FAR struct ieee802154_driver_s *netdev,
                           FAR const struct ieee802154_frame_meta_s *meta,
                           FAR struct iob_s *framelist)
{
  FAR struct macnet_driver_s *priv;
  FAR struct iob_s *iob;
  int ret;

  DEBUGASSERT(netdev != NULL && netdev->i_dev.d_private != NULL);
  priv = (FAR struct macnet_driver_s *)netdev->i_dev.d_private;

  DEBUGASSERT(meta != NULL && framelist != NULL);

  /* Add the incoming list of frames to the MAC's outgoing queue */

  for (iob = framelist; iob != NULL; iob = framelist)
    {
      /* Increment statistics */

      NETDEV_TXPACKETS(&priv->md_dev.i_dev);

      /* Remove the IOB from the queue */

      framelist     = iob->io_flink;
      iob->io_flink = NULL;

      /* Transfer the frame to the MAC */

      ret = mac802154_req_data(priv->md_mac, meta, iob);
      if (ret < 0)
        {
          wlerr("ERROR: mac802154_req_data failed: %d\n", ret);

          iob_free(iob);
          for (iob = framelist; iob != NULL; iob = framelist)
            {
              /* Remove the IOB from the queue and free */

              framelist = iob->io_flink;
              iob_free(iob);
            }

          return ret;
        }
    }

  return OK;
}

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
  FAR struct ieee802154_driver_s *ieee;
  FAR struct net_driver_s  *dev;
  FAR struct mac802154_maccb_s *maccb;
  FAR uint8_t *pktbuf;
  int ret;

  DEBUGASSERT(mac != NULL);

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
  if (pktbuf == NULL)
    {
      nerr("ERROR: Failed to allocate the packet buffer\n");
      kmm_free(priv);
      return -ENOMEM;
    }

  /* Initialize the driver structure */

  ieee                = &priv->md_dev;
  dev                 = &ieee->i_dev;
  dev->d_buf          = pktbuf;            /* Single packet buffer */
  dev->d_ifup         = macnet_ifup;       /* I/F up (new IP address) callback */
  dev->d_ifdown       = macnet_ifdown;     /* I/F down callback */
  dev->d_txavail      = macnet_txavail;    /* New TX data callback */
#ifdef CONFIG_NET_IGMP
  dev->d_addmac       = macnet_addmac;     /* Add multicast MAC address */
  dev->d_rmmac        = macnet_rmmac;      /* Remove multicast MAC address */
#endif
 #ifdef CONFIG_NETDEV_IOCTL
  dev->d_ioctl        = macnet_ioctl;      /* Handle network IOCTL commands */
#endif
  dev->d_private      = (FAR void *)priv;  /* Used to recover private state from dev */

  /* Create a watchdog for timing polling for and timing of transmisstions */

  priv->md_mac        = mac;               /* Save the MAC interface instance */
  priv->md_txpoll     = wd_create();       /* Create periodic poll timer */

  DEBUGASSERT(priv->md_txpoll != NULL);

  /* Initialize the Network frame-related callbacks */

  ieee->i_get_mhrlen  = macnet_get_mhrlen; /* Get MAC header length */
  ieee->i_req_data    = macnet_req_data;   /* Enqueue frame for transmission */

  /* Initialize the MAC callbacks */

  priv->md_cb.mc_priv = priv;

  maccb           = &priv->md_cb.mc_cb;
  maccb->notify   = macnet_notify;
  maccb->rxframe  = macnet_rxframe;

  /* Bind the callback structure */

  ret = mac802154_bind(mac, maccb);
  if (ret < 0)
    {
      nerr("ERROR: Failed to bind the MAC callbacks: %d\n", ret);

      /* Release wdog timers */

      wd_delete(priv->md_txpoll);

      /* Free memory and return the error */

      kmm_free(pktbuf);
      kmm_free(priv);
      return ret;
    }

  /* Put the interface in the down state. */

  macnet_ifdown(dev);

  /* Register the device with the OS so that socket IOCTLs can be performed */

  (void)netdev_register(&priv->md_dev.i_dev, NET_LL_IEEE802154);
  return OK;
}

#endif /* CONFIG_NET && CONFIG_NET_skeleton */
