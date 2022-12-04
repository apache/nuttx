/****************************************************************************
 * drivers/net/skeleton.c
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
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>
#include <nuttx/net/arp.h>
#include <nuttx/net/netdev.h>

#ifdef CONFIG_NET_PKT
#  include <nuttx/net/pkt.h>
#endif

#ifdef CONFIG_NET_SKELETON

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Work queue support is required. */

#if !defined(CONFIG_SCHED_WORKQUEUE)
#  error Work queue support is required in this configuration (CONFIG_SCHED_WORKQUEUE)
#else

/* The low priority work queue is preferred.  If it is not enabled, LPWORK
 * will be the same as HPWORK.
 *
 * NOTE:  However, the network should NEVER run on the high priority work
 * queue!  That queue is intended only to service short back end interrupt
 * processing that never suspends.  Suspending the high priority work queue
 * may bring the system to its knees!
 */

#define ETHWORK LPWORK

/* CONFIG_NET_SKELETON_NINTERFACES determines the number of
 * physical interfaces that will be supported.
 */

#ifndef CONFIG_NET_SKELETON_NINTERFACES
# define CONFIG_NET_SKELETON_NINTERFACES 1
#endif

/* TX timeout = 1 minute */

#define SKELETON_TXTIMEOUT (60*CLK_TCK)

/* Packet buffer size */

#define PKTBUF_SIZE (MAX_NETDEV_PKTSIZE + CONFIG_NET_GUARDSIZE)

/* This is a helper pointer for accessing the contents of Ethernet header */

#define BUF ((FAR struct eth_hdr_s *)priv->sk_dev.d_buf)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The skel_driver_s encapsulates all state information for a single hardware
 * interface
 */

struct skel_driver_s
{
  bool sk_bifup;               /* true:ifup false:ifdown */
  struct wdog_s sk_txtimeout;  /* TX timeout timer */
  struct work_s sk_irqwork;    /* For deferring interrupt work to the work queue */
  struct work_s sk_pollwork;   /* For deferring poll work to the work queue */

  /* This holds the information visible to the NuttX network */

  struct net_driver_s sk_dev;  /* Interface understood by the network */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* These statically allocated structures would mean that only a single
 * instance of the device could be supported.  In order to support multiple
 * devices instances, this data would have to be allocated dynamically.
 */

/* A single packet buffer per device is used in this example.  There might
 * be multiple packet buffers in a more complex, pipelined design.  Many
 * contemporary Ethernet interfaces, for example,  use multiple, linked DMA
 * descriptors in rings to implement such a pipeline.  This example assumes
 * much simpler hardware that simply handles one packet at a time.
 *
 * NOTE that if CONFIG_NET_SKELETON_NINTERFACES were greater than 1,
 * you would need a minimum on one packet buffer per instance.
 * Much better to be allocated dynamically in cases where more than
 * one are needed.
 */

static uint16_t
  g_pktbuf[CONFIG_NET_SKELETON_NINTERFACES][(PKTBUF_SIZE + 1) / 2];

/* Driver state structure */

static struct skel_driver_s g_skel[CONFIG_NET_SKELETON_NINTERFACES];

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Common TX logic */

static int  skel_transmit(FAR struct skel_driver_s *priv);
static int  skel_txpoll(FAR struct net_driver_s *dev);

/* Interrupt handling */

static void skel_reply(struct skel_driver_s *priv);
static void skel_receive(FAR struct skel_driver_s *priv);
static void skel_txdone(FAR struct skel_driver_s *priv);

static void skel_interrupt_work(FAR void *arg);
static int  skel_interrupt(int irq, FAR void *context, FAR void *arg);

/* Watchdog timer expirations */

static void skel_txtimeout_work(FAR void *arg);
static void skel_txtimeout_expiry(wdparm_t arg);

/* NuttX callback functions */

static int  skel_ifup(FAR struct net_driver_s *dev);
static int  skel_ifdown(FAR struct net_driver_s *dev);

static void skel_txavail_work(FAR void *arg);
static int  skel_txavail(FAR struct net_driver_s *dev);

#if defined(CONFIG_NET_MCASTGROUP) || defined(CONFIG_NET_ICMPv6)
static int  skel_addmac(FAR struct net_driver_s *dev,
              FAR const uint8_t *mac);
#ifdef CONFIG_NET_MCASTGROUP
static int  skel_rmmac(FAR struct net_driver_s *dev,
              FAR const uint8_t *mac);
#endif
#ifdef CONFIG_NET_ICMPv6
static void skel_ipv6multicast(FAR struct skel_driver_s *priv);
#endif
#endif
#ifdef CONFIG_NETDEV_IOCTL
static int  skel_ioctl(FAR struct net_driver_s *dev, int cmd,
              unsigned long arg);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: skel_transmit
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
 *   The network is locked.
 *
 ****************************************************************************/

static int skel_transmit(FAR struct skel_driver_s *priv)
{
  /* Verify that the hardware is ready to send another packet.  If we get
   * here, then we are committed to sending a packet; Higher level logic
   * must have assured that there is no transmission in progress.
   */

  /* Increment statistics */

  NETDEV_TXPACKETS(priv->sk_dev);

  /* Send the packet: address=priv->sk_dev.d_buf, length=priv->sk_dev.d_len */

  /* Enable Tx interrupts */

  /* Setup the TX timeout watchdog (perhaps restarting the timer) */

  wd_start(&priv->sk_txtimeout, SKELETON_TXTIMEOUT,
           skel_txtimeout_expiry, (wdparm_t)priv);
  return OK;
}

/****************************************************************************
 * Name: skel_txpoll
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

static int skel_txpoll(FAR struct net_driver_s *dev)
{
  FAR struct skel_driver_s *priv =
    (FAR struct skel_driver_s *)dev->d_private;

  /* Send the packet */

  skel_transmit(priv);

  /* If zero is returned, the polling will continue until all connections
   * have been examined.
   */

  return 0;
}

/****************************************************************************
 * Name: skel_reply
 *
 * Description:
 *   After a packet has been received and dispatched to the network, it
 *   may return return with an outgoing packet.  This function checks for
 *   that case and performs the transmission if necessary.
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

static void skel_reply(struct skel_driver_s *priv)
{
  /* If the packet dispatch resulted in data that should be sent out on the
   * network, the field d_len will set to a value > 0.
   */

  if (priv->sk_dev.d_len > 0)
    {
      /* And send the packet */

      skel_transmit(priv);
    }
}

/****************************************************************************
 * Name: skel_receive
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

static void skel_receive(FAR struct skel_driver_s *priv)
{
  do
    {
      /* Check for errors and update statistics */

      /* Check if the packet is a valid size for the network buffer
       * configuration.
       */

      /* Copy the data data from the hardware to priv->sk_dev.d_buf.  Set
       * amount of data in priv->sk_dev.d_len
       */

#ifdef CONFIG_NET_PKT
      /* When packet sockets are enabled, feed the frame into the tap */

       pkt_input(&priv->sk_dev);
#endif

#ifdef CONFIG_NET_IPv4
      /* Check for an IPv4 packet */

      if (BUF->type == HTONS(ETHTYPE_IP))
        {
          ninfo("IPv4 frame\n");
          NETDEV_RXIPV4(&priv->sk_dev);

          /* Receive an IPv4 packet from the network device */

          ipv4_input(&priv->sk_dev);

          /* Check for a reply to the IPv4 packet */

          skel_reply(priv);
        }
      else
#endif
#ifdef CONFIG_NET_IPv6
      /* Check for an IPv6 packet */

      if (BUF->type == HTONS(ETHTYPE_IP6))
        {
          ninfo("IPv6 frame\n");
          NETDEV_RXIPV6(&priv->sk_dev);

          /* Dispatch IPv6 packet to the network layer */

          ipv6_input(&priv->sk_dev);

          /* Check for a reply to the IPv6 packet */

          skel_reply(priv);
        }
      else
#endif
#ifdef CONFIG_NET_ARP
      /* Check for an ARP packet */

      if (BUF->type == HTONS(ETHTYPE_ARP))
        {
          /* Dispatch ARP packet to the network layer */

          arp_input(&priv->sk_dev);
          NETDEV_RXARP(&priv->sk_dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, the field  d_len will set to a value
           * > 0.
           */

          if (priv->sk_dev.d_len > 0)
            {
              skel_transmit(priv);
            }
        }
      else
#endif
        {
          NETDEV_RXDROPPED(&priv->sk_dev);
        }
    }
  while (true); /* While there are more packets to be processed */
}

/****************************************************************************
 * Name: skel_txdone
 *
 * Description:
 *   An interrupt was received indicating that the last TX packet(s) is done
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

static void skel_txdone(FAR struct skel_driver_s *priv)
{
  /* Check for errors and update statistics */

  NETDEV_TXDONE(priv->sk_dev);

  /* Check if there are pending transmissions */

  /* If no further transmissions are pending, then cancel the TX timeout and
   * disable further Tx interrupts.
   */

  wd_cancel(&priv->sk_txtimeout);

  /* And disable further TX interrupts. */

  /* In any event, poll the network for new TX data */

  devif_poll(&priv->sk_dev, skel_txpoll);
}

/****************************************************************************
 * Name: skel_interrupt_work
 *
 * Description:
 *   Perform interrupt related work from the worker thread
 *
 * Input Parameters:
 *   arg - The argument passed when work_queue() was called.
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *   Runs on a worker thread.
 *
 ****************************************************************************/

static void skel_interrupt_work(FAR void *arg)
{
  FAR struct skel_driver_s *priv = (FAR struct skel_driver_s *)arg;

  /* Lock the network and serialize driver operations if necessary.
   * NOTE: Serialization is only required in the case where the driver work
   * is performed on an LP worker thread and where more than one LP worker
   * thread has been configured.
   */

  net_lock();

  /* Process pending Ethernet interrupts */

  /* Get and clear interrupt status bits */

  /* Handle interrupts according to status bit settings */

  /* Check if we received an incoming packet, if so, call skel_receive() */

  skel_receive(priv);

  /* Check if a packet transmission just completed.  If so, call skel_txdone.
   * This may disable further Tx interrupts if there are no pending
   * transmissions.
   */

  skel_txdone(priv);
  net_unlock();

  /* Re-enable Ethernet interrupts */

  up_enable_irq(CONFIG_NET_SKELETON_IRQ);
}

/****************************************************************************
 * Name: skel_interrupt
 *
 * Description:
 *   Hardware interrupt handler
 *
 * Input Parameters:
 *   irq     - Number of the IRQ that generated the interrupt
 *   context - Interrupt register state save info (architecture-specific)
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *   Runs in the context of a the Ethernet interrupt handler.  Local
 *   interrupts are disabled by the interrupt logic.
 *
 ****************************************************************************/

static int skel_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct skel_driver_s *priv = (FAR struct skel_driver_s *)arg;

  DEBUGASSERT(priv != NULL);

  /* Disable further Ethernet interrupts.  Because Ethernet interrupts are
   * also disabled if the TX timeout event occurs, there can be no race
   * condition here.
   */

  up_disable_irq(CONFIG_NET_SKELETON_IRQ);

  /* TODO: Determine if a TX transfer just completed */

    {
      /* If a TX transfer just completed, then cancel the TX timeout so
       * there will be no race condition between any subsequent timeout
       * expiration and the deferred interrupt processing.
       */

       wd_cancel(&priv->sk_txtimeout);
    }

  /* Schedule to perform the interrupt processing on the worker thread. */

  work_queue(ETHWORK, &priv->sk_irqwork, skel_interrupt_work, priv, 0);
  return OK;
}

/****************************************************************************
 * Name: skel_txtimeout_work
 *
 * Description:
 *   Perform TX timeout related work from the worker thread
 *
 * Input Parameters:
 *   arg - The argument passed when work_queue() as called.
 *
 * Returned Value:
 *   OK on success
 *
 ****************************************************************************/

static void skel_txtimeout_work(FAR void *arg)
{
  FAR struct skel_driver_s *priv = (FAR struct skel_driver_s *)arg;

  /* Lock the network and serialize driver operations if necessary.
   * NOTE: Serialization is only required in the case where the driver work
   * is performed on an LP worker thread and where more than one LP worker
   * thread has been configured.
   */

  net_lock();

  /* Increment statistics and dump debug info */

  NETDEV_TXTIMEOUTS(priv->sk_dev);

  /* Then reset the hardware */

  /* Then poll the network for new XMIT data */

  devif_poll(&priv->sk_dev, skel_txpoll);
  net_unlock();
}

/****************************************************************************
 * Name: skel_txtimeout_expiry
 *
 * Description:
 *   Our TX watchdog timed out.  Called from the timer interrupt handler.
 *   The last TX never completed.  Reset the hardware and start again.
 *
 * Input Parameters:
 *   arg  - The argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Runs in the context of a the timer interrupt handler.  Local
 *   interrupts are disabled by the interrupt logic.
 *
 ****************************************************************************/

static void skel_txtimeout_expiry(wdparm_t arg)
{
  FAR struct skel_driver_s *priv = (FAR struct skel_driver_s *)arg;

  /* Disable further Ethernet interrupts.  This will prevent some race
   * conditions with interrupt work.  There is still a potential race
   * condition with interrupt work that is already queued and in progress.
   */

  up_disable_irq(CONFIG_NET_SKELETON_IRQ);

  /* Schedule to perform the TX timeout processing on the worker thread. */

  work_queue(ETHWORK, &priv->sk_irqwork, skel_txtimeout_work, priv, 0);
}

/****************************************************************************
 * Name: skel_ifup
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
 *   The network is locked.
 *
 ****************************************************************************/

static int skel_ifup(FAR struct net_driver_s *dev)
{
  FAR struct skel_driver_s *priv =
    (FAR struct skel_driver_s *)dev->d_private;

#ifdef CONFIG_NET_IPv4
  ninfo("Bringing up: %d.%d.%d.%d\n",
        (int)dev->d_ipaddr & 0xff,
        (int)(dev->d_ipaddr >> 8) & 0xff,
        (int)(dev->d_ipaddr >> 16) & 0xff,
        (int)dev->d_ipaddr >> 24);
#endif
#ifdef CONFIG_NET_IPv6
  ninfo("Bringing up: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        dev->d_ipv6addr[0], dev->d_ipv6addr[1], dev->d_ipv6addr[2],
        dev->d_ipv6addr[3], dev->d_ipv6addr[4], dev->d_ipv6addr[5],
        dev->d_ipv6addr[6], dev->d_ipv6addr[7]);
#endif

  /* Initialize PHYs, Ethernet interface, and setup up Ethernet interrupts */

  /* Instantiate MAC address from priv->sk_dev.d_mac.ether.ether_addr_octet */

#ifdef CONFIG_NET_ICMPv6
  /* Set up IPv6 multicast address filtering */

  skel_ipv6multicast(priv);
#endif

  /* Enable the Ethernet interrupt */

  priv->sk_bifup = true;
  up_enable_irq(CONFIG_NET_SKELETON_IRQ);
  return OK;
}

/****************************************************************************
 * Name: skel_ifdown
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
 *   The network is locked.
 *
 ****************************************************************************/

static int skel_ifdown(FAR struct net_driver_s *dev)
{
  FAR struct skel_driver_s *priv =
    (FAR struct skel_driver_s *)dev->d_private;
  irqstate_t flags;

  /* Disable the Ethernet interrupt */

  flags = enter_critical_section();
  up_disable_irq(CONFIG_NET_SKELETON_IRQ);

  /* Cancel the TX timeout timers */

  wd_cancel(&priv->sk_txtimeout);

  /* Put the EMAC in its reset, non-operational state.  This should be
   * a known configuration that will guarantee the skel_ifup() always
   * successfully brings the interface back up.
   */

  /* Mark the device "down" */

  priv->sk_bifup = false;
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: skel_txavail_work
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
 *   Runs on a work queue thread.
 *
 ****************************************************************************/

static void skel_txavail_work(FAR void *arg)
{
  FAR struct skel_driver_s *priv = (FAR struct skel_driver_s *)arg;

  /* Lock the network and serialize driver operations if necessary.
   * NOTE: Serialization is only required in the case where the driver work
   * is performed on an LP worker thread and where more than one LP worker
   * thread has been configured.
   */

  net_lock();

  /* Ignore the notification if the interface is not yet up */

  if (priv->sk_bifup)
    {
      /* Check if there is room in the hardware to hold another packet. */

      /* If so, then poll the network for new XMIT data */

      devif_poll(&priv->sk_dev, skel_txpoll);
    }

  net_unlock();
}

/****************************************************************************
 * Name: skel_txavail
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
 *   The network is locked.
 *
 ****************************************************************************/

static int skel_txavail(FAR struct net_driver_s *dev)
{
  FAR struct skel_driver_s *priv =
    (FAR struct skel_driver_s *)dev->d_private;

  /* Is our single work structure available?  It may not be if there are
   * pending interrupt actions and we will have to ignore the Tx
   * availability action.
   */

  if (work_available(&priv->sk_pollwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      work_queue(ETHWORK, &priv->sk_pollwork, skel_txavail_work, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: skel_addmac
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
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_MCASTGROUP) || defined(CONFIG_NET_ICMPv6)
static int skel_addmac(FAR struct net_driver_s *dev, FAR const uint8_t *mac)
{
  FAR struct skel_driver_s *priv =
    (FAR struct skel_driver_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

  UNUSED(priv);
  return OK;
}
#endif

/****************************************************************************
 * Name: skel_rmmac
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
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_MCASTGROUP
static int skel_rmmac(FAR struct net_driver_s *dev, FAR const uint8_t *mac)
{
  FAR struct skel_driver_s *priv =
    (FAR struct skel_driver_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

  UNUSED(priv);
  return OK;
}
#endif

/****************************************************************************
 * Name: skel_ipv6multicast
 *
 * Description:
 *   Configure the IPv6 multicast MAC address.
 *
 * Input Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6
static void skel_ipv6multicast(FAR struct skel_driver_s *priv)
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

  dev    = &priv->sk_dev;
  tmp16  = dev->d_ipv6addr[6];
  mac[2] = 0xff;
  mac[3] = tmp16 >> 8;

  tmp16  = dev->d_ipv6addr[7];
  mac[4] = tmp16 & 0xff;
  mac[5] = tmp16 >> 8;

  ninfo("IPv6 Multicast: %02x:%02x:%02x:%02x:%02x:%02x\n",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  skel_addmac(dev, mac);

#ifdef CONFIG_NET_ICMPv6_AUTOCONF
  /* Add the IPv6 all link-local nodes Ethernet address.  This is the
   * address that we expect to receive ICMPv6 Router Advertisement
   * packets.
   */

  skel_addmac(dev, g_ipv6_ethallnodes.ether_addr_octet);

#endif /* CONFIG_NET_ICMPv6_AUTOCONF */

#ifdef CONFIG_NET_ICMPv6_ROUTER
  /* Add the IPv6 all link-local routers Ethernet address.  This is the
   * address that we expect to receive ICMPv6 Router Solicitation
   * packets.
   */

  skel_addmac(dev, g_ipv6_ethallrouters.ether_addr_octet);

#endif /* CONFIG_NET_ICMPv6_ROUTER */
}
#endif /* CONFIG_NET_ICMPv6 */

/****************************************************************************
 * Name: skel_ioctl
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
 *   The network is locked.
 *
 ****************************************************************************/

#ifdef CONFIG_NETDEV_IOCTL
static int skel_ioctl(FAR struct net_driver_s *dev, int cmd,
                      unsigned long arg)
{
  FAR struct skel_driver_s *priv =
    (FAR struct skel_driver_s *)dev->d_private;
  int ret;

  /* Decode and dispatch the driver-specific IOCTL command */

  switch (cmd)
    {
      /* Add cases here to support the IOCTL commands */

      default:
        nerr("ERROR: Unrecognized IOCTL command: %d\n", command);
        return -ENOTTY;  /* Special return value for this case */
    }

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: skel_initialize
 *
 * Description:
 *   Initialize the Ethernet controller and driver
 *
 * Input Parameters:
 *   intf - In the case where there are multiple EMACs, this value
 *          identifies which EMAC is to be initialized.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *   Called early in initialization before multi-tasking is initiated.
 *
 ****************************************************************************/

int skel_initialize(int intf)
{
  FAR struct skel_driver_s *priv;

  /* Get the interface structure associated with this interface number. */

  DEBUGASSERT(intf < CONFIG_NET_SKELETON_NINTERFACES);
  priv = &g_skel[intf];

  /* Check if a Ethernet chip is recognized at its I/O base */

  /* Attach the IRQ to the driver */

  if (irq_attach(CONFIG_NET_SKELETON_IRQ, skel_interrupt, priv))
    {
      /* We could not attach the ISR to the interrupt */

      return -EAGAIN;
    }

  /* Initialize the driver structure */

  memset(priv, 0, sizeof(struct skel_driver_s));
  priv->sk_dev.d_buf     = (FAR uint8_t *)g_pktbuf[intf]; /* Single packet buffer */
  priv->sk_dev.d_ifup    = skel_ifup;                     /* I/F up (new IP address) callback */
  priv->sk_dev.d_ifdown  = skel_ifdown;                   /* I/F down callback */
  priv->sk_dev.d_txavail = skel_txavail;                  /* New TX data callback */
#ifdef CONFIG_NET_MCASTGROUP
  priv->sk_dev.d_addmac  = skel_addmac;                   /* Add multicast MAC address */
  priv->sk_dev.d_rmmac   = skel_rmmac;                    /* Remove multicast MAC address */
#endif
#ifdef CONFIG_NETDEV_IOCTL
  priv->sk_dev.d_ioctl   = skel_ioctl;                    /* Handle network IOCTL commands */
#endif
  priv->sk_dev.d_private = g_skel;                        /* Used to recover private state from dev */

  /* Put the interface in the down state.  This usually amounts to resetting
   * the device and/or calling skel_ifdown().
   */

  /* Read the MAC address from the hardware into
   * priv->sk_dev.d_mac.ether.ether_addr_octet
   * Applies only if the Ethernet MAC has its own internal address.
   */

  /* Register the device with the OS so that socket IOCTLs can be performed */

  netdev_register(&priv->sk_dev, NET_LL_ETHERNET);
  return OK;
}

#endif /* !defined(CONFIG_SCHED_WORKQUEUE) */

#endif /* CONFIG_NET_SKELETON */
