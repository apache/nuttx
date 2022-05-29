/****************************************************************************
 * drivers/wireless/ieee802154/xbee/xbee_netdev.c
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
#include <nuttx/wqueue.h>
#include <nuttx/signal.h>
#include <nuttx/mm/iob.h>
#include <nuttx/net/arp.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/radiodev.h>
#include <nuttx/net/ieee802154.h>
#include <nuttx/net/sixlowpan.h>
#include <nuttx/wireless/ieee802154/ieee802154_mac.h>
#include <nuttx/wireless/ieee802154/xbee.h>

#include "xbee_mac.h"

#if defined(CONFIG_NET_6LOWPAN) || defined(CONFIG_NET_IEEE802154)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* If processing is not done at the interrupt level, then work queue support
 * is required.
 */

#if !defined(CONFIG_SCHED_WORKQUEUE)
#  error Work queue support is required in this configuration (CONFIG_SCHED_WORKQUEUE)
#endif

/* The low priority work queue is preferred.  If it is not enabled, LPWORK
 * will be the same as HPWORK.
 *
 * NOTE:  However, the network should NEVER run on the high priority work
 * queue!  That queue is intended only to service short back end interrupt
 * processing that never suspends.  Suspending the high priority work queue
 * may bring the system to its knees!
 */

#define XBEENET_WORK LPWORK

/* Preferred address size */

#ifdef CONFIG_NET_6LOWPAN_EXTENDEDADDR
#  define XBEENET_ADDRSIZE IEEE802154_EADDRSIZE
#else
#  define XBEENET_ADDRSIZE IEEE802154_SADDRSIZE
#endif

/* Frame size */

#if defined(CONFIG_NET_IEEE802154_FRAMELEN)
#  define XBEENET_FRAMELEN CONFIG_NET_IEEE802154_FRAMELEN
#else
#  define XBEENET_FRAMELEN IEEE802154_MAX_PHY_PACKET_SIZE
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This is our private version of the MAC callback structure */

struct xbeenet_callback_s
{
  /* This holds the information visible to the MAC layer */

  struct xbee_maccb_s mc_cb;            /* Interface understood by the MAC
                                         * layer */
  FAR struct xbeenet_driver_s *mc_priv; /* Our priv data */
};

/* The xbeenet_driver_s encapsulates all state information for a single
 * IEEE802.15.4 MAC interface.
 */

struct xbeenet_driver_s
{
  /* This holds the information visible to the NuttX network */

  struct radio_driver_s xd_dev;     /* Interface understood by the network
                                     * Cast compatible with struct xbeenet_driver_s */

  /* For internal use by this driver */

  sem_t xd_exclsem;                 /* Exclusive access to struct */
  struct xbeenet_callback_s xd_cb;  /* Callback information */
  XBEEHANDLE xd_mac;                /* Contained XBee MAC interface */
  bool xd_bifup;                    /* true:ifup false:ifdown */
  struct work_s xd_pollwork;        /* Defer poll work to the work queue */

  /* Hold a list of events */

  bool xd_enableevents : 1;         /* Are events enabled? */
  bool xd_eventpending : 1;         /* Is there a get event using the semaphore? */
  sem_t xd_eventsem;                /* Signaling semaphore for waiting get event */
  sq_queue_t primitive_queue;       /* For holding primitives to pass along */

  /* MAC Service notification information */

  bool    xd_notify_registered;
  pid_t   xd_notify_pid;
  struct sigevent xd_notify_event;
  struct sigwork_s xd_notify_work;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Utility functions ********************************************************/

static int xbeenet_set_ipaddress(FAR struct net_driver_s *dev);
static inline void xbeenet_netmask(FAR struct net_driver_s *dev);

/* IEE802.15.4 MAC callback functions ***************************************/

static int  xbeenet_notify(FAR struct xbee_maccb_s *maccb,
                           FAR struct ieee802154_primitive_s *primitive);
static int  xbeenet_rxframe(FAR struct xbeenet_driver_s *maccb,
                            FAR struct ieee802154_data_ind_s *ind);

/* Network interface support ************************************************/

/* Common TX logic */

static int  xbeenet_txpoll_callback(FAR struct net_driver_s *dev);

/* IOCTL support */

#ifdef CONFIG_NET_STARPOINT
#ifdef CONFIG_NET_6LOWPAN_EXTENDEDADDR
static int xbeenet_coord_eaddr(FAR struct radio_driver_s *radio,
                              FAR uint8_t *eaddr);
#else
static int xbeenet_coord_saddr(FAR struct radio_driver_s *radio,
                              FAR uint8_t *saddr);
#endif
#endif

/* NuttX callback functions */

static int  xbeenet_ifup(FAR struct net_driver_s *dev);
static int  xbeenet_ifdown(FAR struct net_driver_s *dev);

static void xbeenet_txavail_work(FAR void *arg);
static int  xbeenet_txavail(FAR struct net_driver_s *dev);

#ifdef CONFIG_NET_MCASTGROUP
static int  xbeenet_addmac(FAR struct net_driver_s *dev,
              FAR const uint8_t *mac);
static int  xbeenet_rmmac(FAR struct net_driver_s *dev,
              FAR const uint8_t *mac);
#endif
#ifdef CONFIG_NETDEV_IOCTL
static int  xbeenet_ioctl(FAR struct net_driver_s *dev, int cmd,
              unsigned long arg);
#endif
static int xbeenet_get_mhrlen(FAR struct radio_driver_s *netdev,
              FAR const void *meta);
static int xbeenet_req_data(FAR struct radio_driver_s *netdev,
              FAR const void *meta, FAR struct iob_s *framelist);
static int xbeenet_properties(FAR struct radio_driver_s *netdev,
              FAR struct radiodev_properties_s *properties);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_NET_6LOWPAN
static struct sixlowpan_reassbuf_s g_iobuffer;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xbeenet_set_ipaddress
 *
 * Description:
 *   Advertise the MAC and IPv6 address for this node.
 *
 *   Creates a MAC-based IP address from the IEEE 802.15.14 short or extended
 *   address assigned to the node.
 *
 *    128  112  96   80    64   48   32   16
 *    ---- ---- ---- ----  ---- ---- ---- ----
 *    fe80 0000 0000 0000  0000 00ff fe00 xxxx 2-byte short address IEEE
 *                                             48-bit MAC
 *    fe80 0000 0000 0000  xxxx xxxx xxxx xxxx 8-byte extended address IEEE
 *                                             EUI-64
 *
 ****************************************************************************/

static int xbeenet_set_ipaddress(FAR struct net_driver_s *dev)
{
  FAR struct xbeenet_driver_s *priv;
  union ieee802154_macarg_u arg;
  int ret;

  dev->d_ipv6addr[0]  = HTONS(CONFIG_XBEE_DEFAULT_PREFIX_0);
  dev->d_ipv6addr[1]  = HTONS(CONFIG_XBEE_DEFAULT_PREFIX_1);
  dev->d_ipv6addr[2]  = HTONS(CONFIG_XBEE_DEFAULT_PREFIX_2);
  dev->d_ipv6addr[3]  = HTONS(CONFIG_XBEE_DEFAULT_PREFIX_3);

#ifdef CONFIG_NET_6LOWPAN_EXTENDEDADDR
  uint8_t *eaddr;

  DEBUGASSERT(dev != NULL && dev->d_private != NULL);
  priv = (FAR struct xbeenet_driver_s *)dev->d_private;

  /* Get the eaddr from the MAC */

  arg.getreq.attr = IEEE802154_ATTR_MAC_EADDR;
  ret = xbee_ioctl(priv->xd_mac, MAC802154IOC_MLME_GET_REQUEST,
                  (unsigned long)((uintptr_t)&arg));
  if (ret < 0)
    {
      wlerr("ERROR: MAC802154IOC_MLME_GET_REQUEST failed: %d\n", ret);
      return ret;
    }
  else
    {
      IEEE802154_EADDRCOPY(dev->d_mac.radio.nv_addr,
                           arg.getreq.attrval.mac.eaddr);
      dev->d_mac.radio.nv_addrlen = IEEE802154_EADDRSIZE;

      /* Set the IP address based on the eaddr */

      dev->d_ipv6addr[4]  = HTONS((uint16_t)eaddr[0] << 8 |
                                  (uint16_t)eaddr[1]);
      dev->d_ipv6addr[5]  = HTONS((uint16_t)eaddr[2] << 8 |
                                  (uint16_t)eaddr[3]);
      dev->d_ipv6addr[6]  = HTONS((uint16_t)eaddr[4] << 8 |
                                  (uint16_t)eaddr[5]);
      dev->d_ipv6addr[7]  = HTONS((uint16_t)eaddr[6] << 8 |
                                  (uint16_t)eaddr[7]);

      /* Invert the U/L bit */

      dev->d_ipv6addr[4] ^= HTONS(0x0200);

      dev->d_mac.radio.nv_addrlen = IEEE802154_EADDRSIZE;
      return OK;
    }

#else
  uint8_t *saddr;

  DEBUGASSERT(dev != NULL && dev->d_private != NULL);
  priv = (FAR struct xbeenet_driver_s *)dev->d_private;

  /* Get the saddr from the MAC */

  arg.getreq.attr = IEEE802154_ATTR_MAC_SADDR;
  ret = xbee_ioctl(priv->xd_mac, MAC802154IOC_MLME_GET_REQUEST,
                  (unsigned long)((uintptr_t)&arg));
  if (ret < 0)
    {
      wlerr("ERROR: MAC802154IOC_MLME_GET_REQUEST failed: %d\n", ret);
      return ret;
    }
  else
    {
      /* Set the MAC address as the saddr */

      saddr = arg.getreq.attrval.mac.saddr;

      /* Network layers expect address in Network Order (Big Endian) */

      dev->d_mac.radio.nv_addr[0] = saddr[1];
      dev->d_mac.radio.nv_addr[1] = saddr[0];

      dev->d_mac.radio.nv_addrlen = IEEE802154_SADDRSIZE;

      /* Set the IP address based on the saddr */

      dev->d_ipv6addr[4]  = 0;
      dev->d_ipv6addr[5]  = HTONS(0x00ff);
      dev->d_ipv6addr[6]  = HTONS(0xfe00);
      dev->d_ipv6addr[7]  = HTONS((uint16_t)saddr[0] << 8 |
                                  (uint16_t)saddr[1]);
      return OK;
    }
#endif
}

/****************************************************************************
 * Name: xbeenet_netmask
 *
 * Description:
 *   Create a netmask of a MAC-based IP address which may be based on either
 *   the IEEE 802.15.14 short or extended address of the MAC.
 *
 *    128  112  96   80    64   48   32   16
 *    ---- ---- ---- ----  ---- ---- ---- ----
 *    fe80 0000 0000 0000  0000 00ff fe00 xxxx 2-byte short address IEEE
 *                                             48-bit MAC
 *    fe80 0000 0000 0000  xxxx xxxx xxxx xxxx 8-byte extended address IEEE
 *                                             EUI-64
 *
 ****************************************************************************/

static inline void xbeenet_netmask(FAR struct net_driver_s *dev)
{
  dev->d_ipv6netmask[0]  = 0xffff;
  dev->d_ipv6netmask[1]  = 0xffff;
  dev->d_ipv6netmask[2]  = 0xffff;
  dev->d_ipv6netmask[3]  = 0xffff;
#ifdef CONFIG_NET_6LOWPAN_EXTENDEDADDR
  dev->d_ipv6netmask[4]  = 0;
  dev->d_ipv6netmask[5]  = 0;
  dev->d_ipv6netmask[6]  = 0;
  dev->d_ipv6netmask[7]  = 0;
#else
  dev->d_ipv6netmask[4]  = 0xffff;
  dev->d_ipv6netmask[5]  = 0xffff;
  dev->d_ipv6netmask[6]  = 0xffff;
  dev->d_ipv6netmask[7]  = 0;
#endif
}

/****************************************************************************
 * Name: xbeenet_notify
 *
 * Description:
 *
 ****************************************************************************/

static int xbeenet_notify(FAR struct xbee_maccb_s *maccb,
                          FAR struct ieee802154_primitive_s *primitive)
{
  FAR struct xbeenet_callback_s *cb =
    (FAR struct xbeenet_callback_s *)maccb;
  FAR struct xbeenet_driver_s *priv;

  DEBUGASSERT(cb != NULL && cb->mc_priv != NULL);
  priv = cb->mc_priv;

  /* Handle the special case for data indications or "incoming frames" */

  if (primitive->type == IEEE802154_PRIMITIVE_IND_DATA)
    {
      return xbeenet_rxframe(priv, &primitive->u.dataind);
    }

  /* If there is a registered primitive receiver, queue the event and signal
   * the receiver. Events should be popped from the queue from the
   * application at a reasonable rate in order for the MAC layer to be able
   * to allocate new primitives.
   */

  if (priv->xd_enableevents)
    {
      /* Get exclusive access to the driver structure.  We don't care about
       * any signals so if we see one, just go back to trying to get access
       * again.
       */

      while (nxsem_wait(&priv->xd_exclsem) < 0);

      sq_addlast((FAR sq_entry_t *)primitive, &priv->primitive_queue);

      /* Check if there is a read waiting for data */

      if (priv->xd_eventpending)
        {
          /* Wake the thread waiting for the data transmission */

          priv->xd_eventpending = false;
          nxsem_post(&priv->xd_eventsem);
        }

      if (priv->xd_notify_registered)
        {
          priv->xd_notify_event.sigev_value.sival_int = primitive->type;
          nxsig_notification(priv->xd_notify_pid, &priv->xd_notify_event,
                             SI_QUEUE, &priv->xd_notify_work);
        }

      nxsem_post(&priv->xd_exclsem);
      return OK;
    }

  /* By returning a negative value, we let the MAC know that we don't want
   * the primitive and it will free it for us
   */

  return -1;
}

/****************************************************************************
 * Name: xbeenet_rxframe
 *
 * Description:
 *   Handle received frames forward by the XBee MAC.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.  On success, the ind and its contained iob will be freed.
 *   The ind will be intact if this function returns a failure.
 *
 ****************************************************************************/

static int xbeenet_rxframe(FAR struct xbeenet_driver_s *priv,
                           FAR struct ieee802154_data_ind_s *ind)
{
  FAR struct iob_s *iob;
  int ret;

  /* Ignore the frame if the network is not up */

  if (!priv->xd_bifup)
    {
      wlwarn("WARNING: Dropped... Network is down\n");
      return -ENETDOWN;
    }

  /* Peek the IOB contained the frame in the struct ieee802154_data_ind_s */

  DEBUGASSERT(priv != NULL && ind != NULL && ind->frame != NULL);
  iob = ind->frame;

  /* Remove the IOB containing the frame. */

  ind->frame = NULL;

  net_lock();

  /* Transfer the frame to the network logic */

#ifdef CONFIG_NET_IEEE802154
  /* Invoke the PF_IEEE802154 tap first.  If the frame matches
   * with a connected PF_IEEE802145 socket, it will take the
   * frame and return success.
   */

  ret = ieee802154_input(&priv->xd_dev, iob, (FAR void *)ind);
  if (ret < 0)
#endif
#ifdef CONFIG_NET_6LOWPAN
    {
      /* If the frame is not a 6LoWPAN frame, then return an error.  The
       * first byte following the MAC head at the io_offset should be a
       * valid IPHC header.
       */

      if ((iob->io_data[iob->io_offset] & SIXLOWPAN_DISPATCH_NALP_MASK) ==
          SIXLOWPAN_DISPATCH_NALP)
        {
          wlwarn("WARNING: Dropped... Not a 6LoWPAN frame: %02x\n",
                 iob->io_data[iob->io_offset]);
          ret = -EINVAL;
        }
      else
        {
          /* Make sure the our single packet buffer is attached */

          priv->xd_dev.r_dev.d_buf = g_iobuffer.rb_buf;

          /* And give the packet to 6LoWPAN */

          ret = sixlowpan_input(&priv->xd_dev, iob, (FAR void *)ind);
        }
    }

  if (ret < 0)
#endif
    {
      net_unlock();
      ind->frame = iob;
      return ret;
    }

  /* Increment statistics */

  NETDEV_RXPACKETS(&priv->xd_dev.r_dev);
  NETDEV_RXIPV6(&priv->xd_dev.r_dev);

  net_unlock();

  /* sixlowpan_input() will free the IOB, but we must free the struct
   * ieee802154_primitive_s container here.
   */

  ieee802154_primitive_free((FAR struct ieee802154_primitive_s *)ind);
  return OK;
}

/****************************************************************************
 * Name: xbeenet_txpoll_callback
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
 * Input Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static int xbeenet_txpoll_callback(FAR struct net_driver_s *dev)
{
  /* If zero is returned, the polling will continue until all connections
   * have been examined.
   */

  return 0;
}

/****************************************************************************
 * Name: xbeenet_coord_eaddr
 *
 * Description:
 *   Get the extended address of the PAN coordinator.
 *
 * Input Parameters:
 *   radio - Reference to a radio network driver state instance.
 *   eaddr - The location in which to return the extended address.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_STARPOINT) && defined(CONFIG_NET_6LOWPAN_EXTENDEDADDR)
static int xbeenet_coord_eaddr(FAR struct radio_driver_s *radio,
                               FAR uint8_t *eaddr)
{
  FAR struct xbeenet_driver_s *priv = (FAR struct xbeenet_driver_s *)radio;
  union ieee802154_macarg_u arg;
  int ret;

  arg.getreq.attr = IEEE802154_ATTR_MAC_COORD_EADDR ;
  ret = xbee_ioctl(priv->xd_mac, MAC802154IOC_MLME_GET_REQUEST,
                   (unsigned long)((uintptr_t)&arg));
  if (ret < 0)
    {
      nerr("ERROR: MAC802154IOC_MLME_GET_REQUEST failed: %d\n", ret);
      return ret;
    }

  IEEE802154_EADDRCOPY(eaddr, arg.getreq.attrval.mac.eaddr);
  return OK;
}
#endif

/****************************************************************************
 * Name: xbeenet_coord_saddr
 *
 * Description:
 *   Get the short address of the PAN coordinator.
 *
 * Input Parameters:
 *   radio - Reference to a radio network driver state instance.
 *   saddr - The location in which to return the short address.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_STARPOINT) && !defined(CONFIG_NET_6LOWPAN_EXTENDEDADDR)
static int xbeenet_coord_saddr(FAR struct radio_driver_s *radio,
                               FAR uint8_t *saddr)
{
  FAR struct xbeenet_driver_s *priv = (FAR struct xbeenet_driver_s *)radio;
  union ieee802154_macarg_u arg;
  int ret;

  arg.getreq.attr = IEEE802154_ATTR_MAC_COORD_SADDR ;
  ret = xbee_ioctl(priv->xd_mac, MAC802154IOC_MLME_GET_REQUEST,
                   (unsigned long)((uintptr_t)&arg));
  if (ret < 0)
    {
      nerr("ERROR: MAC802154IOC_MLME_GET_REQUEST failed: %d\n", ret);
      return ret;
    }

  IEEE802154_SADDRCOPY(saddr, arg.getreq.attrval.mac.saddr);
  return OK;
}
#endif

/****************************************************************************
 * Name: xbeenet_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the IEEE 802.15.4 interface when an IP address
 *   is provided
 *
 * Input Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int xbeenet_ifup(FAR struct net_driver_s *dev)
{
  FAR struct xbeenet_driver_s *priv =
    (FAR struct xbeenet_driver_s *)dev->d_private;
  int ret;

  /* Set the IP address based on the addressing assigned to the node */

  ret = xbeenet_set_ipaddress(dev);
  if (ret >= 0)
    {
      wlinfo("Bringing up: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
             dev->d_ipv6addr[0], dev->d_ipv6addr[1], dev->d_ipv6addr[2],
             dev->d_ipv6addr[3], dev->d_ipv6addr[4], dev->d_ipv6addr[5],
             dev->d_ipv6addr[6], dev->d_ipv6addr[7]);

#ifdef CONFIG_NET_6LOWPAN_EXTENDEDADDR
      wlinfo("             Node: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n",
             dev->d_mac.radio.nv_addr[0], dev->d_mac.radio.nv_addr[1],
             dev->d_mac.radio.nv_addr[2], dev->d_mac.radio.nv_addr[3],
             dev->d_mac.radio.nv_addr[4], dev->d_mac.radio.nv_addr[5],
             dev->d_mac.radio.nv_addr[6], dev->d_mac.radio.nv_addr[7]);
#else
      wlinfo("             Node: %02x:%02x\n",
             dev->d_mac.radio.nv_addr[0], dev->d_mac.radio.nv_addr[1]);
#endif

      /* The interface is now up */

      priv->xd_bifup = true;
      ret = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: xbeenet_ifdown
 *
 * Description:
 *   NuttX Callback: Stop the interface.
 *
 * Input Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int xbeenet_ifdown(FAR struct net_driver_s *dev)
{
  FAR struct xbeenet_driver_s *priv =
    (FAR struct xbeenet_driver_s *)dev->d_private;
  irqstate_t flags;

  /* Disable interruption */

  flags = enter_critical_section();

  /* TODO: Put the xbee driver in its reset, non-operational state.  This
   * should be a known configuration that will guarantee the xbeenet_ifup()
   * always successfully brings the interface back up.
   */

  /* Mark the device "down" */

  priv->xd_bifup = false;
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: xbeenet_txavail_work
 *
 * Description:
 *   Perform an out-of-cycle poll on the worker thread.
 *
 * Input Parameters:
 *   arg - Reference to the NuttX driver state structure (cast to void*)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called on the higher priority worker thread.
 *
 ****************************************************************************/

static void xbeenet_txavail_work(FAR void *arg)
{
  FAR struct xbeenet_driver_s *priv = (FAR struct xbeenet_driver_s *)arg;

  wlinfo("ifup=%u\n", priv->xd_bifup);

  /* Lock the network and serialize driver operations if necessary.
   * NOTE: Serialization is only required in the case where the driver work
   * is performed on an LP worker thread and where more than one LP worker
   * thread has been configured.
   */

  net_lock();

  /* Ignore the notification if the interface is not yet up */

  if (priv->xd_bifup)
    {
#ifdef CONFIG_NET_6LOWPAN
      /* Make sure the our single packet buffer is attached */

      priv->xd_dev.r_dev.d_buf = g_iobuffer.rb_buf;
#endif

      /* Then poll the network for new XMIT data */

      devif_poll(&priv->xd_dev.r_dev, xbeenet_txpoll_callback);
    }

  net_unlock();
}

/****************************************************************************
 * Name: xbeenet_txavail
 *
 * Description:
 *   Driver callback invoked when new TX data is available.  This is a
 *   stimulus perform an out-of-cycle poll and, thereby, reduce the TX
 *   latency.
 *
 * Input Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called in normal user mode
 *
 ****************************************************************************/

static int xbeenet_txavail(FAR struct net_driver_s *dev)
{
  FAR struct xbeenet_driver_s *priv =
    (FAR struct xbeenet_driver_s *)dev->d_private;

  wlinfo("Available=%u\n", work_available(&priv->xd_pollwork));

  /* Is our single work structure available?  It may not be if there are
   * pending interrupt actions and we will have to ignore the Tx
   * availability action.
   */

  if (work_available(&priv->xd_pollwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      work_queue(XBEENET_WORK, &priv->xd_pollwork, xbeenet_txavail_work,
                 priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: xbeenet_addmac
 *
 * Description:
 *   NuttX Callback: Add the specified MAC address to the hardware multicast
 *   address filtering
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be added
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_MCASTGROUP
static int xbeenet_addmac(FAR struct net_driver_s *dev,
                          FAR const uint8_t *mac)
{
  FAR struct xbeenet_driver_s *priv =
    (FAR struct xbeenet_driver_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table.  Not used
   * with IEEE 802.15.4 radios.
   */

  return -ENOSYS;
}
#endif

/****************************************************************************
 * Name: xbeenet_rmmac
 *
 * Description:
 *   NuttX Callback: Remove the specified MAC address from the hardware
 *   multicast address filtering
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be removed
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_MCASTGROUP
static int xbeenet_rmmac(FAR struct net_driver_s *dev,
                         FAR const uint8_t *mac)
{
  FAR struct xbeenet_driver_s *priv =
    (FAR struct xbeenet_driver_s *)dev->d_private;

  /* Remove the MAC address from the hardware multicast routing table.  Not
   * used with IEEE 802.15.4 radios.
   */

  return -ENOSYS;
}
#endif

/****************************************************************************
 * Name: xbeenet_ioctl
 *
 * Description:
 *   Handle network IOCTL commands directed to this device.
 *
 * Input Parameters:
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
static int xbeenet_ioctl(FAR struct net_driver_s *dev, int cmd,
                        unsigned long arg)
{
  FAR struct xbeenet_driver_s *priv =
    (FAR struct xbeenet_driver_s *)dev->d_private;
  int ret = -EINVAL;

  ret = nxsem_wait(&priv->xd_exclsem);
  if (ret < 0)
    {
      wlerr("ERROR: nxsem_wait failed: %d\n", ret);
      return ret;
    }

  /* Check for IOCTLs aimed at the IEEE802.15.4 MAC layer */

  if (_MAC802154IOCVALID(cmd))
    {
      FAR struct ieee802154_netmac_s *netmac =
        (FAR struct ieee802154_netmac_s *)arg;

      if (netmac != NULL)
        {
          unsigned long macarg = (unsigned int)((uintptr_t)&netmac->u);

          switch (cmd)
            {
              /* Command:     MAC802154IOC_NOTIFY_REGISTER
               * Description: Register to receive a signal whenever there is
               *              an event primitive sent from the MAC layer.
               * Argument:    A read-only pointer to an instance of struct
               *              xbeenet_notify_s
               * Return:      Zero (OK) on success.  Minus one will be
               *              returned on failure with the errno value set
               *              appropriately.
               */

              case MAC802154IOC_NOTIFY_REGISTER:
                {
                  /* Save the notification events */

                  priv->xd_notify_event       = netmac->u.event;
                  priv->xd_notify_pid         = getpid();
                  priv->xd_notify_registered  = true;
                  ret = OK;
                }
                break;

              case MAC802154IOC_GET_EVENT:
                {
                  FAR struct ieee802154_primitive_s *primitive;

                  while (1)
                    {
                      /* Try popping an event off the queue */

                      primitive = (FAR struct ieee802154_primitive_s *)
                                      sq_remfirst(&priv->primitive_queue);

                      /* If there was an event to pop off, copy it into the
                       * user data and free it from the MAC layer's memory.
                       */

                      if (primitive != NULL)
                        {
                          memcpy(&netmac->u, primitive,
                                 sizeof(struct ieee802154_primitive_s));

                          /* Free the primitive */

                          ieee802154_primitive_free(primitive);
                          ret = OK;
                          break;
                        }

                      /* There can only be one getevent pending at a time */

                      if (priv->xd_eventpending)
                        {
                          ret = -EAGAIN;
                          break;
                        }

                      priv->xd_eventpending = true;
                      nxsem_post(&priv->xd_exclsem);

                      /* Wait to be signaled when an event is queued */

                      ret = nxsem_wait(&priv->xd_eventsem);
                      if (ret < 0)
                        {
                          priv->xd_eventpending = false;
                          return ret;
                        }

                      /* Get exclusive access again, then loop back around
                       * and try and pop an event off the queue
                       */

                      ret = nxsem_wait(&priv->xd_exclsem);
                      if (ret < 0)
                        {
                          wlerr("ERROR: nxsem_wait failed: %d\n", ret);
                          return ret;
                        }
                    }
                }
                break;
              case MAC802154IOC_ENABLE_EVENTS:
                {
                  priv->xd_enableevents = netmac->u.enable;
                  ret = OK;
                }
                break;
              default:
                {
                  ret = xbee_ioctl(priv->xd_mac, cmd, macarg);
                }
                break;
            }
        }
    }

  /* Okay, we have no idea what this command is.. just give to the
   * XBee MAC layer without modification.
   */

  else
   {
     ret = xbee_ioctl(priv->xd_mac, cmd, arg);
   }

  nxsem_post(&priv->xd_exclsem);
  return ret;
}
#endif

/****************************************************************************
 * Name: xbeemac_get_mhrlen
 *
 * Description:
 *   Calculate the MAC header length given the frame meta-data.
 *
 * Input Parameters:
 *   netdev    - The network device that will mediate the MAC interface
 *   meta      - Obfuscated metadata structure needed to create the radio
 *               MAC header
 *
 * Returned Value:
 *   A non-negative MAC headeer length is returned on success; a negated
 *   errno value is returned on any failure.
 *
 * Assumptions:
 *   Called from network logic running on the low-priority worker thread.
 *
 ****************************************************************************/

static int xbeenet_get_mhrlen(FAR struct radio_driver_s *netdev,
                              FAR const void *meta)
{
  FAR struct xbeenet_driver_s *priv =
    (FAR struct xbeenet_driver_s *)netdev;
  FAR const struct ieee802154_frame_meta_s *pktmeta =
    (FAR const struct ieee802154_frame_meta_s *)meta;

  DEBUGASSERT(priv != NULL && priv->xd_mac != NULL && pktmeta != NULL);
  return xbee_get_mhrlen(priv->xd_mac, pktmeta);
}

/****************************************************************************
 * Name: xbeenet_req_data
 *
 * Description:
 *   Requests the transfer of a list of frames to the XBee MAC.
 *
 * Input Parameters:
 *   netdev    - The networkd device that will mediate the MAC interface
 *   meta      - Obfuscated metadata structure needed to create the radio
 *               MAC header
 *   framelist - Head of a list of frames to be transferred.
 *
 * Returned Value:
 *   Zero (OK) returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static int xbeenet_req_data(FAR struct radio_driver_s *netdev,
                           FAR const void *meta, FAR struct iob_s *framelist)
{
  FAR struct xbeenet_driver_s *priv =
    (FAR struct xbeenet_driver_s *)netdev;
  FAR const struct ieee802154_frame_meta_s *framemeta =
    (FAR const struct ieee802154_frame_meta_s *)meta;
  FAR struct iob_s *iob;
  int ret;

  wlinfo("Received framelist\n");

  DEBUGASSERT(priv != NULL && framemeta != NULL && framelist != NULL);

  /* Add the incoming list of frames to the MAC's outgoing queue */

  for (iob = framelist; iob != NULL; iob = framelist)
    {
      /* Increment statistics */

      NETDEV_TXPACKETS(&priv->xd_dev.r_dev);

      /* Remove the IOB from the queue */

      framelist     = iob->io_flink;
      iob->io_flink = NULL;

      ret = xbee_req_data(priv->xd_mac, framemeta, iob);
      if (ret < 0)
        {
          wlerr("ERROR: xbeemac_req_data failed: %d\n", ret);

          iob_free(iob, IOBUSER_WIRELESS_RAD802154);
          for (iob = framelist; iob != NULL; iob = framelist)
            {
              /* Remove the IOB from the queue and free */

              framelist = iob->io_flink;
              iob_free(iob, IOBUSER_WIRELESS_RAD802154);
            }

          NETDEV_TXERRORS(&priv->xd_dev.r_dev);
          return ret;
        }

      NETDEV_TXDONE(&priv->xd_dev.r_dev);
    }

  return OK;
}

/****************************************************************************
 * Name: xbeenet_properties
 *
 * Description:
 *   Different packet radios may have different properties.  If there are
 *   multiple packet radios, then those properties have to be queried at
 *   run time.  This information is provided to the 6LoWPAN network via the
 *   following structure.
 *
 * Input Parameters:
 *   netdev     - The network device to be queried
 *   properties - Location where radio properties will be returned.
 *
 * Returned Value:
 *   Zero (OK) returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static int xbeenet_properties(FAR struct radio_driver_s *netdev,
                              FAR struct radiodev_properties_s *properties)
{
  DEBUGASSERT(netdev != NULL && properties != NULL);
  memset(properties, 0, sizeof(struct radiodev_properties_s));

  /* General */

  properties->sp_addrlen   = XBEENET_ADDRSIZE;    /* Length of an address */
  properties->sp_framelen  = XBEENET_FRAMELEN;    /* Fixed frame length */

  /* Multicast address (uses broadcast address)
   *
   * For meshes (only) a multicast address candetermined by the first 3 bits
   * of a short address (RFC 4944):
   *
   *   0xxxxxxx xxxxxxxx: Unicast address
   *   100xxxxx xxxxxxxx: Multicast address
   *   101xxxxx xxxxxxxx: Reserved
   *   110xxxxx xxxxxxxx: Reserved
   *   111xxxxx xxxxxxxx: Reserved
   *
   * Otherwise, Multicast is implemented with the broadcast address
   * (qualified by the destination PANID).
   */

  properties->sp_mcast.nv_addrlen = NET_6LOWPAN_SADDRSIZE;
  memset(properties->sp_mcast.nv_addr, 0xff, RADIO_MAX_ADDRLEN);

  /* Broadcast address */

  properties->sp_bcast.nv_addrlen = NET_6LOWPAN_SADDRSIZE;
  memset(properties->sp_mcast.nv_addr, 0xff, RADIO_MAX_ADDRLEN);

#ifdef CONFIG_NET_STARPOINT
  /* Star hub node address.
   *
   * If this node is a "point" in a star topology, then the hub node
   * MAC address is the address of the hub/PAN coordinator.
   */

#ifdef CONFIG_NET_6LOWPAN_EXTENDEDADDR
  xbeenet_coord_eaddr(netdev, properties->sp_hubnode.nv_addr);
  properties->sp_hubnode.nv_addrlen = NET_6LOWPAN_EADDRSIZE;
#else
  xbeenet_coord_saddr(netdev, properties->sp_hubnode.nv_addr);
  properties->sp_hubnode.nv_addrlen = NET_6LOWPAN_SADDRSIZE;
#endif
#endif

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xbee_netdev_register
 *
 * Description:
 *   Register a network driver to access the IEEE 802.15.4 MAC layer from
 *   a socket using 6LoWPAN
 *
 * Input Parameters:
 *   mac - Pointer to the mac layer struct to be registered.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int xbee_netdev_register(XBEEHANDLE xbee)
{
  FAR struct xbeenet_driver_s *priv;
  FAR struct radio_driver_s *radio;
  FAR struct net_driver_s  *dev;
  FAR struct xbee_maccb_s *maccb;
  int ret;

  DEBUGASSERT(xbee != NULL);

  /* Get the interface structure associated with this interface number. */

  priv = (FAR struct xbeenet_driver_s *)
    kmm_zalloc(sizeof(struct xbeenet_driver_s));

  if (priv == NULL)
    {
      nerr("ERROR: Failed to allocate the device structure\n");
      return -ENOMEM;
    }

  /* Initialize the driver structure */

  radio               = &priv->xd_dev;
  dev                 = &radio->r_dev;
  dev->d_ifup         = xbeenet_ifup;       /* I/F up (new IP address) callback */
  dev->d_ifdown       = xbeenet_ifdown;     /* I/F down callback */
  dev->d_txavail      = xbeenet_txavail;    /* New TX data callback */
#ifdef CONFIG_NET_MCASTGROUP
  dev->d_addmac       = xbeenet_addmac;     /* Add multicast MAC address */
  dev->d_rmmac        = xbeenet_rmmac;      /* Remove multicast MAC address */
#endif
 #ifdef CONFIG_NETDEV_IOCTL
  dev->d_ioctl        = xbeenet_ioctl;      /* Handle network IOCTL commands */
#endif
  dev->d_private      = priv;               /* Used to recover private state from dev */
  priv->xd_mac        = xbee;               /* Save the MAC interface instance */

  /* Setup a locking semaphore for exclusive device driver access */

  nxsem_init(&priv->xd_exclsem, 0, 1);

  /* Set the network mask. */

  xbeenet_netmask(dev);

  /* Initialize the Network frame-related callbacks */

  radio->r_get_mhrlen = xbeenet_get_mhrlen;  /* Get MAC header length */
  radio->r_req_data   = xbeenet_req_data;    /* Enqueue frame for transmission */
  radio->r_properties = xbeenet_properties;  /* Return radio properties */

  /* Initialize fields related to MAC event handling */

  priv->xd_eventpending = false;
  nxsem_init(&priv->xd_eventsem, 0, 0);
  nxsem_set_protocol(&priv->xd_eventsem, SEM_PRIO_NONE);

  sq_init(&priv->primitive_queue);

  priv->xd_enableevents = false;
  priv->xd_notify_registered = false;

  /* Initialize the XBee MAC callbacks */

  priv->xd_cb.mc_priv = priv;

  maccb           = &priv->xd_cb.mc_cb;
  maccb->flink    = NULL;
  maccb->prio     = CONFIG_XBEE_NETDEV_RECVRPRIO;
  maccb->notify   = xbeenet_notify;

  /* Bind the callback structure */

  ret = xbee_bind(xbee, maccb);
  if (ret < 0)
    {
      nerr("ERROR: Failed to bind the XBee MAC callbacks: %d\n", ret);

      /* Free memory and return the error */

      kmm_free(priv);
      return ret;
    }

  /* Put the interface in the down state. */

  xbeenet_ifdown(dev);

#ifdef CONFIG_NET_6LOWPAN
  /* Make sure the our single packet buffer is attached. We must do this
   * before registering the device since, once the device is registered, a
   * packet may be attempted to be forwarded and require the buffer.
   */

  priv->xd_dev.r_dev.d_buf = g_iobuffer.rb_buf;
#endif

  /* Register the device with the OS so that socket IOCTLs can be performed */

  netdev_register(&priv->xd_dev.r_dev, NET_LL_IEEE802154);
  return OK;
}

#endif /* CONFIG_NET_6LOWPAN */
