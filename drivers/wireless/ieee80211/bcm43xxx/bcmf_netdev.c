/****************************************************************************
 * drivers/wireless/ieee80211/bcm43xxx/bcmf_netdev.c
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
#if defined(CONFIG_NET) && defined(CONFIG_IEEE80211_BROADCOM_FULLMAC)

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
#include <nuttx/wqueue.h>
#include <nuttx/net/arp.h>
#include <nuttx/net/netdev.h>
#include <nuttx/wireless/wireless.h>
#include <nuttx/wireless/ieee80211/bcmf_board.h>

#ifdef CONFIG_NET_PKT
#  include <nuttx/net/pkt.h>
#endif

#include "bcmf_driver.h"
#include "bcmf_cdc.h"
#include "bcmf_bdc.h"
#include "bcmf_ioctl.h"
#include "bcmf_netdev.h"

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

#define BCMFWORK LPWORK

/* CONFIG_IEEE80211_BROADCOM_NINTERFACES determines the number of physical
 * interfaces that will be supported.
 */

#ifndef CONFIG_IEEE80211_BROADCOM_NINTERFACES
# define CONFIG_IEEE80211_BROADCOM_NINTERFACES 1
#endif

/* This is a helper pointer for accessing the contents of Ethernet header */

#define BUF ((FAR struct eth_hdr_s *)priv->bc_dev.d_buf)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Common TX logic */

static int  bcmf_transmit(FAR struct bcmf_dev_s *priv,
                          FAR struct bcmf_frame_s *frame);
static void bcmf_receive(FAR struct bcmf_dev_s *priv);
static int  bcmf_txpoll(FAR struct net_driver_s *dev);
static void bcmf_rxpoll_work(FAR void *arg);

/* NuttX callback functions */

static int  bcmf_ifup(FAR struct net_driver_s *dev);
static int  bcmf_ifdown(FAR struct net_driver_s *dev);

static int  bcmf_txavail(FAR struct net_driver_s *dev);

#if defined(CONFIG_NET_MCASTGROUP) || defined(CONFIG_NET_ICMPv6)
static int  bcmf_addmac(FAR struct net_driver_s *dev,
                        FAR const uint8_t *mac);
#ifdef CONFIG_NET_MCASTGROUP
static int  bcmf_rmmac(FAR struct net_driver_s *dev,
                       FAR const uint8_t *mac);
#endif
#ifdef CONFIG_NET_ICMPv6
static void bcmf_ipv6multicast(FAR struct bcmf_dev_s *priv);
#endif
#endif
#ifdef CONFIG_NETDEV_IOCTL
static int  bcmf_ioctl(FAR struct net_driver_s *dev, int cmd,
                       unsigned long arg);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

int bcmf_netdev_alloc_tx_frame(FAR struct bcmf_dev_s *priv)
{
  if (priv->cur_tx_frame != NULL)
    {
      /* Frame available */

      return OK;
    }

  /* Allocate frame for TX */

  priv->cur_tx_frame = bcmf_bdc_allocate_frame(priv,
                                               MAX_NETDEV_PKTSIZE, true);
  if (!priv->cur_tx_frame)
    {
      wlerr("ERROR: Cannot allocate TX frame\n");
      return -ENOMEM;
    }

  priv->bc_dev.d_buf = priv->cur_tx_frame->data;
  priv->bc_dev.d_len = 0;

  return OK;
}

/****************************************************************************
 * Name: bcmf_transmit
 *
 * Description:
 *   Start hardware transmission.  Called either from the txdone interrupt
 *   handling or from watchdog based polling.
 *
 * Input Parameters:
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

static int bcmf_transmit(FAR struct bcmf_dev_s *priv,
                         struct bcmf_frame_s *frame)
{
  int ret;

  frame->len = priv->bc_dev.d_len +
      (unsigned int)(frame->data - frame->base);

  ret = bcmf_bdc_transmit_frame(priv, frame);

  if (ret)
    {
      wlerr("ERROR: Failed to transmit frame\n");
      return -EIO;
    }

  NETDEV_TXPACKETS(&priv->bc_dev);

  return OK;
}

/****************************************************************************
 * Name: bcmf_receive
 *
 * Description:
 *   An interrupt was received indicating the availability of a new RX packet
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static void bcmf_receive(FAR struct bcmf_dev_s *priv)
{
  struct bcmf_frame_s *frame;

  do
    {
      /* Request frame buffer from bus interface */

      frame = bcmf_bdc_rx_frame(priv);

      if (frame == NULL)
        {
          /* No more frame to process */

          bcmf_netdev_notify_tx(priv);
          break;
        }

      if (!priv->bc_bifup)
        {
          /* Interface down, drop frame */

          priv->bus->free_frame(priv, frame);
          continue;
        }

      priv->bc_dev.d_buf = frame->data;
      priv->bc_dev.d_len = frame->len - (frame->data - frame->base);

      wlinfo("Got frame %p %d\n", frame, priv->bc_dev.d_len);

#ifdef CONFIG_NET_PKT
      /* When packet sockets are enabled, feed the frame into the tap */

       pkt_input(&priv->bc_dev);
#endif

      /* Check if this is an 802.1Q VLAN tagged packet */

      if (BUF->type == HTONS(TPID_8021QVLAN))
        {
          /* Need to remove the 4 octet VLAN Tag, by moving src and dest
           * addresses 4 octets to the right, and then read the actual
           * ethertype. The VLAN ID and priority fields are currently
           * ignored.
           */

          uint8_t temp_buffer[12];
          memcpy(temp_buffer, frame->data, 12);
          memcpy(frame->data + 4, temp_buffer, 12);

          priv->bc_dev.d_buf = frame->data = frame->data + 4;
          priv->bc_dev.d_len -= 4;
        }

      /* We only accept IP packets of the configured type and ARP packets */

#ifdef CONFIG_NET_IPv4
      if (BUF->type == HTONS(ETHTYPE_IP))
        {
          ninfo("IPv4 frame\n");
          NETDEV_RXIPV4(&priv->bc_dev);

          /* Handle ARP on input then give the IPv4 packet to the network
           * layer
           */

          arp_ipin(&priv->bc_dev);
          ipv4_input(&priv->bc_dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, d_len field will set to a value > 0.
           */

          if (priv->bc_dev.d_len > 0)
            {
              /* Update the Ethernet header with the correct MAC address */

#ifdef CONFIG_NET_IPv6
              if (IFF_IS_IPv4(priv->bc_dev.d_flags))
#endif
                {
                  arp_out(&priv->bc_dev);
                }
#ifdef CONFIG_NET_IPv6
              else
                {
                  neighbor_out(&kel->bc_dev);
                }
#endif

              /* And send the packet */

              bcmf_transmit(priv, frame);
            }
          else
            {
              /* Release RX frame buffer */

              priv->bus->free_frame(priv, frame);
            }
        }
      else
#endif
#ifdef CONFIG_NET_IPv6
      if (BUF->type == HTONS(ETHTYPE_IP6))
        {
          ninfo("IPv6 frame\n");
          NETDEV_RXIPV6(&priv->bc_dev);

          /* Give the IPv6 packet to the network layer */

          ipv6_input(&priv->bc_dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, d_len field will set to a value > 0.
           */

          if (priv->bc_dev.d_len > 0)
            {
              /* Update the Ethernet header with the correct MAC address */

#ifdef CONFIG_NET_IPv4
              if (IFF_IS_IPv4(priv->bc_dev.d_flags))
                {
                  arp_out(&priv->bc_dev);
                }
              else
#endif
#ifdef CONFIG_NET_IPv6
                {
                  neighbor_out(&priv->bc_dev);
                }
#endif

              /* And send the packet */

              bcmf_transmit(priv, frame);
            }
          else
            {
              /* Release RX frame buffer */

              priv->bus->free_frame(priv, frame);
            }
        }
      else
#endif
#ifdef CONFIG_NET_ARP
      if (BUF->type == HTONS(ETHTYPE_ARP))
        {
          arp_arpin(&priv->bc_dev);
          NETDEV_RXARP(&priv->bc_dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, d_len field will set to a value > 0.
           */

          if (priv->bc_dev.d_len > 0)
            {
              bcmf_transmit(priv, frame);
            }
          else
            {
              /* Release RX frame buffer */

              priv->bus->free_frame(priv, frame);
            }
        }
      else
#endif
        {
          /* On some routers, it may constantly receive mysterious packet...
           * https://www.wireshark.org/docs/wsar_html/epan/etypes_8h.html
           * for more etypes definitions.
           */

          NETDEV_RXDROPPED(&priv->bc_dev);
          priv->bus->free_frame(priv, frame);
        }
    }
  while (1); /* While there are more packets to be processed */
}

/****************************************************************************
 * Name: bcmf_txpoll
 *
 * Description:
 *   The transmitter is available, check if the network has any outgoing
 *   packets ready to send.  This is a callback from devif_poll().
 *   devif_poll() may be called:
 *
 *   1. When the preceding TX packet send is complete,
 *   2. When the preceding TX packet send times out and the interface is
 *      reset
 *   3. During normal TX polling
 *
 * Input Parameters:
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

static int bcmf_txpoll(FAR struct net_driver_s *dev)
{
  FAR struct bcmf_dev_s *priv = (FAR struct bcmf_dev_s *)dev->d_private;

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  if (priv->bc_dev.d_len > 0)
    {
      /* Look up the destination MAC address and add it to the Ethernet
       * header.
       */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
      if (IFF_IS_IPv4(priv->bc_dev.d_flags))
#endif
        {
          arp_out(&priv->bc_dev);
        }
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
      else
#endif
        {
          neighbor_out(&priv->bc_dev);
        }
#endif /* CONFIG_NET_IPv6 */

      if (!devif_loopback(&priv->bc_dev))
        {
          /* Send the packet */

          bcmf_transmit(priv, priv->cur_tx_frame);

          /* TODO: Check if there is room in the device to hold another
           * packet. If not, return a non-zero value to terminate the poll.
           */

          priv->cur_tx_frame = NULL;
          return 1;
        }
    }

  /* If zero is returned, the polling will continue until all connections
   * have been examined.
   */

  return 0;
}

/****************************************************************************
 * Function: bcmf_tx_poll_work
 *
 * Description:
 *   The function is called in order to perform an out-of-sequence TX poll.
 *   This is done:
 *
 *   1. After completion of a transmission (bcmf_netdev_notify_tx), and
 *   2. When new TX data is available (bcmf_txavail).
 *
 * Input Parameters:
 *   arg - context of device to use
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void bcmf_tx_poll_work(FAR void *arg)
{
  FAR struct bcmf_dev_s *priv = (FAR struct bcmf_dev_s *)arg;

  net_lock();

  /* Ignore the notification if the interface is not yet up */

  if (priv->bc_bifup)
    {
      /* Check if there is room in the hardware to hold another packet. */

      while (bcmf_netdev_alloc_tx_frame(priv) == OK)
        {
          /* If so, then poll the network for new XMIT data */

          devif_poll(&priv->bc_dev, bcmf_txpoll);

          /* Break out the continuous send if :
           * 1. IP stack has no data to send.
           * 2. RX worker ready.
           */

          if (priv->cur_tx_frame != NULL ||
              !work_available(&priv->bc_rxwork))
            {
              break;
            }
        }
    }

  net_unlock();
}

/****************************************************************************
 * Name: bcmf_rxpoll_work
 *
 * Description:
 *   Process RX frames
 *
 * Input Parameters:
 *   arg - context of device to use
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *
 ****************************************************************************/

static void bcmf_rxpoll_work(FAR void *arg)
{
  FAR struct bcmf_dev_s *priv = (FAR struct bcmf_dev_s *)arg;
  FAR void *oldbuf;

  /* Lock the network and serialize driver operations if necessary.
   * NOTE: Serialization is only required in the case where the driver work
   * is performed on an LP worker thread and where more than one LP worker
   * thread has been configured.
   */

  net_lock();

  /* Tx work will hold the d_buf until there is data to send,
   * replace and cache the d_buf temporarily
   */

  oldbuf = priv->bc_dev.d_buf;

  bcmf_receive(priv);

  priv->bc_dev.d_buf = oldbuf;

  /* Check if a packet transmission just completed.  If so, call bcmf_txdone.
   * This may disable further Tx interrupts if there are no pending
   * transmissions.
   */

#if 0
  bcmf_txdone(priv);
#endif
  net_unlock();
}

/****************************************************************************
 * Name: bcmf_netdev_notify_tx
 *
 * Description:
 *   Notify callback called when TX frame is avail or sent.
 *
 * Assumptions:
 *
 ****************************************************************************/

void bcmf_netdev_notify_tx(FAR struct bcmf_dev_s *priv)
{
  /* Schedule to perform a poll for new Tx data the worker thread. */

  if (work_available(&priv->bc_pollwork))
    {
      work_queue(BCMFWORK, &priv->bc_pollwork,
                 bcmf_tx_poll_work, priv, 0);
    }
}

/****************************************************************************
 * Name: bcmf_netdev_notify_rx
 *
 * Description:
 *   Notify callback called when RX frame is available
 *
 * Assumptions:
 *
 ****************************************************************************/

void bcmf_netdev_notify_rx(FAR struct bcmf_dev_s *priv)
{
  /* Queue a job to process RX frames */

  if (work_available(&priv->bc_rxwork))
    {
      work_queue(BCMFWORK, &priv->bc_rxwork, bcmf_rxpoll_work, priv, 0);
    }
}

/****************************************************************************
 * Name: bcmf_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the Ethernet interface when an IP address is
 *   provided
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

static int bcmf_ifup(FAR struct net_driver_s *dev)
{
  FAR struct bcmf_dev_s *priv = (FAR struct bcmf_dev_s *)dev->d_private;

#ifdef CONFIG_NET_IPv4
  ninfo("Bringing up: %d.%d.%d.%d\n",
        (int)(dev->d_ipaddr & 0xff),
        (int)((dev->d_ipaddr >> 8) & 0xff),
        (int)((dev->d_ipaddr >> 16) & 0xff),
        (int)(dev->d_ipaddr >> 24));
#endif
#ifdef CONFIG_NET_IPv6
  ninfo("Bringing up: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        dev->d_ipv6addr[0], dev->d_ipv6addr[1], dev->d_ipv6addr[2],
        dev->d_ipv6addr[3], dev->d_ipv6addr[4], dev->d_ipv6addr[5],
        dev->d_ipv6addr[6], dev->d_ipv6addr[7]);
#endif

  /* Instantiate MAC address from priv->bc_dev.d_mac.ether.ether_addr_octet */

#ifdef CONFIG_NET_ICMPv6
  /* Set up IPv6 multicast address filtering */

  bcmf_ipv6multicast(priv);
#endif

  /* Enable the hardware interrupt */

  priv->bc_bifup = true;
  return OK;
}

/****************************************************************************
 * Name: bcmf_ifdown
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

static int bcmf_ifdown(FAR struct net_driver_s *dev)
{
  FAR struct bcmf_dev_s *priv = (FAR struct bcmf_dev_s *)dev->d_private;
  irqstate_t flags;

  /* Disable the hardware interrupt */

  flags = enter_critical_section();
#warning Missing logic

  /* Put the EMAC in its reset, non-operational state.  This should be
   * a known configuration that will guarantee the bcmf_ifup() always
   * successfully brings the interface back up.
   */

  /* Mark the device "down" */

  priv->bc_bifup = false;
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: bcmf_txavail
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

static int bcmf_txavail(FAR struct net_driver_s *dev)
{
  FAR struct bcmf_dev_s *priv = (FAR struct bcmf_dev_s *)dev->d_private;

  bcmf_netdev_notify_tx(priv);
  return OK;
}

/****************************************************************************
 * Name: bcmf_addmac
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

#if defined(CONFIG_NET_MCASTGROUP) || defined(CONFIG_NET_ICMPv6)
static int bcmf_addmac(FAR struct net_driver_s *dev, FAR const uint8_t *mac)
{
  FAR struct bcmf_dev_s *priv = (FAR struct bcmf_dev_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

  return OK;
}
#endif

/****************************************************************************
 * Name: bcmf_rmmac
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
static int bcmf_rmmac(FAR struct net_driver_s *dev, FAR const uint8_t *mac)
{
  FAR struct bcmf_dev_s *priv = (FAR struct bcmf_dev_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

  return OK;
}
#endif

/****************************************************************************
 * Name: bcmf_ipv6multicast
 *
 * Description:
 *   Configure the IPv6 multicast MAC address.
 *
 * Input Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6
static void bcmf_ipv6multicast(FAR struct bcmf_dev_s *priv)
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

  dev    = &priv->bc_dev;
  tmp16  = dev->d_ipv6addr[6];
  mac[2] = 0xff;
  mac[3] = tmp16 >> 8;

  tmp16  = dev->d_ipv6addr[7];
  mac[4] = tmp16 & 0xff;
  mac[5] = tmp16 >> 8;

  ninfo("IPv6 Multicast: %02x:%02x:%02x:%02x:%02x:%02x\n",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  bcmf_addmac(dev, mac);

#ifdef CONFIG_NET_ICMPv6_AUTOCONF
  /* Add the IPv6 all link-local nodes Ethernet address.  This is the
   * address that we expect to receive ICMPv6 Router Advertisement
   * packets.
   */

  bcmf_addmac(dev, g_ipv6_ethallnodes.ether_addr_octet);
#endif /* CONFIG_NET_ICMPv6_AUTOCONF */

#ifdef CONFIG_NET_ICMPv6_ROUTER
  /* Add the IPv6 all link-local routers Ethernet address.  This is the
   * address that we expect to receive ICMPv6 Router Solicitation
   * packets.
   */

  bcmf_addmac(dev, g_ipv6_ethallrouters.ether_addr_octet);
#endif /* CONFIG_NET_ICMPv6_ROUTER */
}
#endif /* CONFIG_NET_ICMPv6 */

/****************************************************************************
 * Name: bcmf_ioctl
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
static int bcmf_ioctl(FAR struct net_driver_s *dev, int cmd,
                      unsigned long arg)
{
  int ret;
  FAR struct bcmf_dev_s *priv = (FAR struct bcmf_dev_s *)dev->d_private;

  /* Decode and dispatch the driver-specific IOCTL command */

  switch (cmd)
    {
      case SIOCSIWSCAN:
        ret = bcmf_wl_start_scan(priv, (struct iwreq *)arg);
        break;

      case SIOCGIWSCAN:
        ret = bcmf_wl_get_scan_results(priv, (struct iwreq *)arg);
        break;

      case SIOCSIFHWADDR:    /* Set device MAC address */
        ret = bcmf_wl_set_mac_address(priv, (struct ifreq *)arg);
        break;

      case SIOCSIWAUTH:
        ret = bcmf_wl_set_auth_param(priv, (struct iwreq *)arg);
        break;

      case SIOCSIWENCODEEXT:
        ret = bcmf_wl_set_encode_ext(priv, (struct iwreq *)arg);
        break;

      case SIOCSIWFREQ:     /* Set channel/frequency (Hz) */
        wlwarn("WARNING: SIOCSIWFREQ not implemented\n");
        ret = -ENOSYS;
        break;

      case SIOCGIWFREQ:     /* Get channel/frequency (Hz) */
        ret = bcmf_wl_get_channel(priv, (struct iwreq *)arg);
        break;

      case SIOCSIWMODE:     /* Set operation mode */
        ret = bcmf_wl_set_mode(priv, (struct iwreq *)arg);
        break;

      case SIOCGIWMODE:     /* Get operation mode */
        ret = bcmf_wl_get_mode(priv, (struct iwreq *)arg);
        break;

      case SIOCSIWAP:       /* Set access point MAC addresses */
        ret = bcmf_wl_set_bssid(priv, (struct iwreq *)arg);
        break;

      case SIOCGIWAP:       /* Get access point MAC addresses */
        ret = bcmf_wl_get_bssid(priv, (struct iwreq *)arg);
        break;

      case SIOCSIWESSID:    /* Set ESSID (network name) */
        ret = bcmf_wl_set_ssid(priv, (struct iwreq *)arg);
        break;

      case SIOCGIWESSID:    /* Get ESSID */
        ret = bcmf_wl_get_ssid(priv, (struct iwreq *)arg);
        break;

      case SIOCSIWRATE:     /* Set default bit rate (bps) */
        wlwarn("WARNING: SIOCSIWRATE not implemented\n");
        ret = -ENOSYS;
        break;

      case SIOCGIWRATE:     /* Get default bit rate (bps) */
        ret = bcmf_wl_get_rate(priv, (struct iwreq *)arg);
        break;

      case SIOCSIWTXPOW:    /* Set transmit power (dBm) */
        wlwarn("WARNING: SIOCSIWTXPOW not implemented\n");
        ret = -ENOSYS;
        break;

      case SIOCGIWTXPOW:    /* Get transmit power (dBm) */
        ret = bcmf_wl_get_txpower(priv, (struct iwreq *)arg);
        break;

      case SIOCGIWSENS:     /* Get transmit power (dBm) */
        ret = bcmf_wl_get_rssi(priv, (struct iwreq *)arg);
        break;

      case SIOCGIWRANGE:    /* Get range of parameters */
        ret = bcmf_wl_get_iwrange(priv, (struct iwreq *)arg);
        break;

      default:
        nerr("ERROR: Unrecognized IOCTL command: %d\n", cmd);
        ret = -ENOTTY;  /* Special return value for this case */
        break;
    }

  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bcmf_netdev_register
 *
 * Description:
 *   Register a network driver and set Broadcom chip in a proper state
 *
 * Input Parameters:
 *   priv - Broadcom driver device
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int bcmf_netdev_register(FAR struct bcmf_dev_s *priv)
{
  uint32_t out_len;

  /* Initialize network driver structure */

  memset(&priv->bc_dev, 0, sizeof(priv->bc_dev));
  priv->bc_dev.d_ifup    = bcmf_ifup;     /* I/F up (new IP address) callback */
  priv->bc_dev.d_ifdown  = bcmf_ifdown;   /* I/F down callback */
  priv->bc_dev.d_txavail = bcmf_txavail;  /* New TX data callback */
#ifdef CONFIG_NET_MCASTGROUP
  priv->bc_dev.d_addmac  = bcmf_addmac;   /* Add multicast MAC address */
  priv->bc_dev.d_rmmac   = bcmf_rmmac;    /* Remove multicast MAC address */
#endif
#ifdef CONFIG_NETDEV_IOCTL
  priv->bc_dev.d_ioctl   = bcmf_ioctl;    /* Handle network IOCTL commands */
#endif
  priv->bc_dev.d_private = priv;          /* Used to recover private state from dev */

  /* Initialize network stack interface buffer */

  priv->cur_tx_frame     = NULL;
  priv->bc_dev.d_buf     = NULL;

  /* Put the interface in the down state.  This usually amounts to resetting
   * the device and/or calling bcmf_ifdown().
   */

  /* Enable chip */

  if (bcmf_wl_enable(priv, true) != OK)
    {
      return -EIO;
    }

  /* Set customized MAC address */

  if (bcmf_board_etheraddr(&priv->bc_dev.d_mac.ether))
    {
      out_len = ETHER_ADDR_LEN;
      bcmf_cdc_iovar_request(priv, CHIP_STA_INTERFACE, true,
                             IOVAR_STR_CUR_ETHERADDR,
                             priv->bc_dev.d_mac.ether.ether_addr_octet,
                             &out_len);
    }

  /* Query MAC address */

  out_len = ETHER_ADDR_LEN;
  if (bcmf_cdc_iovar_request(priv, CHIP_STA_INTERFACE, false,
                             IOVAR_STR_CUR_ETHERADDR,
                             priv->bc_dev.d_mac.ether.ether_addr_octet,
                             &out_len) != OK)
    {
      return -EIO;
    }

  /* Register the device with the OS so that socket IOCTLs can be performed */

  netdev_register(&priv->bc_dev, NET_LL_IEEE80211);
  return OK;
}

#endif /* CONFIG_NET && CONFIG_IEEE80211_BROADCOM_FULLMAC */
