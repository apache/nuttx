/****************************************************************************
 * arch/hc/src/m9s12/m9s12_ethernet.c
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
#if defined(CONFIG_NET) && defined(CONFIG_HCS12_EMAC)

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <arpa/inet.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/wdog.h>
#include <nuttx/net/arp.h>
#include <nuttx/net/netdev.h>

#ifdef CONFIG_NET_PKT
#  include <nuttx/net/pkt.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CONFIG_HCS12_NINTERFACES determines the number of physical interfaces
 * that will be supported.
 */

#ifndef CONFIG_HCS12_NINTERFACES
# define CONFIG_HCS12_NINTERFACES 1
#endif

/* TX timeout = 1 minute */

#define HCS12_TXTIMEOUT (60*CLK_TCK)

/* This is a helper pointer for accessing the contents of Ethernet header */

#define BUF ((FAR struct eth_hdr_s *)priv->d_dev.d_buf)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The emac_driver_s encapsulates all state information for a single hardware
 * interface
 */

struct emac_driver_s
{
  bool    d_bifup;            /* true:ifup false:ifdown */
  struct wdog_s d_txtimeout;  /* TX timeout timer */

  /* This holds the information visible to the NuttX network */

  struct net_driver_s d_dev;  /* Interface understood by the network */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* A single packet buffer is used */

static uint8_t g_pktbuf[CONFIG_HCS12_NINTERFACES]
                       [MAX_NETDEV_PKTSIZE + CONFIG_NET_GUARDSIZE];

/* Driver state structure */

static struct emac_driver_s g_emac[CONFIG_HCS12_NINTERFACES];

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Common TX logic */

static int  emac_transmit(FAR struct emac_driver_s *priv);
static int  emac_txpoll(struct net_driver_s *dev);

/* Interrupt handling */

static void emac_receive(FAR struct emac_driver_s *priv);
static void emac_txdone(FAR struct emac_driver_s *priv);
static int  emac_interrupt(int irq, FAR void *context, FAR void *arg);

/* Watchdog timer expirations */

static void emac_txtimeout(wdparm_t arg);

/* NuttX callback functions */

static int emac_ifup(struct net_driver_s *dev);
static int emac_ifdown(struct net_driver_s *dev);
static int emac_txavail(struct net_driver_s *dev);
#ifdef CONFIG_NET_MCASTGROUP
static int emac_addmac(struct net_driver_s *dev, FAR const uint8_t *mac);
static int emac_rmmac(struct net_driver_s *dev, FAR const uint8_t *mac);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: emac_transmit
 *
 * Description:
 *   Start hardware transmission.  Called either from the txdone interrupt
 *   handling or from watchdog based polling.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/

static int emac_transmit(FAR struct emac_driver_s *priv)
{
  /* Verify that the hardware is ready to send another packet.  If we get
   * here, then we are committed to sending a packet; Higher level logic
   * must have assured that there is not transmission in progress.
   */

  /* Increment statistics */

  /* Send the packet: address=priv->d_dev.d_buf, length=priv->d_dev.d_len */

  /* Enable Tx interrupts */

  /* Setup the TX timeout watchdog (perhaps restarting the timer) */

  wd_start(&priv->d_txtimeout, HCS12_TXTIMEOUT,
           emac_txtimeout, (wdparm_t)priv);
  return OK;
}

/****************************************************************************
 * Function: emac_txpoll
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
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/

static int emac_txpoll(struct net_driver_s *dev)
{
  FAR struct emac_driver_s *priv =
    (FAR struct emac_driver_s *)dev->d_private;

  /* Send the packet */

  emac_transmit(priv);

  /* If zero is returned, the polling will continue until all connections
   * have been examined.
   */

  return 0;
}

/****************************************************************************
 * Function: emac_receive
 *
 * Description:
 *   An interrupt was received indicating the availability of a new RX packet
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static void emac_receive(FAR struct emac_driver_s *priv)
{
  do
    {
      /* Check for errors and update statistics */

      /* Check if the packet is a valid size for the network buffer
       * configuration
       */

      /* Copy the data data from the hardware to priv->d_dev.d_buf.  Set
       * amount of data in priv->d_dev.d_len
       */

#ifdef CONFIG_NET_PKT
      /* When packet sockets are enabled, feed the frame into the tap */

      pkt_input(&priv->d_dev);
#endif

      /* We only accept IP packets of the configured type and ARP packets */

#ifdef CONFIG_NET_IPv4
      if (BUF->type == HTONS(ETHTYPE_IP))
        {
          ninfo("IPv4 frame\n");

          /* Receive an IPv4 packet from the network device */

          ipv4_input(&priv->d_dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, d_len field will set to a value > 0.
           */

          if (priv->d_dev.d_len > 0)
            {
              /* And send the packet */

              emac_transmit(priv);
            }
        }
      else
#endif
#ifdef CONFIG_NET_IPv6
      if (BUF->type == HTONS(ETHTYPE_IP6))
        {
          ninfo("IPv6 frame\n");

          /* Give the IPv6 packet to the network layer */

          ipv6_input(&priv->d_dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, d_len field will set to a value > 0.
           */

          if (priv->d_dev.d_len > 0)
            {
              /* And send the packet */

              emac_transmit(priv);
            }
        }
      else
#endif
#ifdef CONFIG_NET_ARP
      if (BUF->type == HTONS(ETHTYPE_ARP))
        {
          arp_input(&priv->d_dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, d_len field will set to a value > 0.
           */

          if (priv->d_dev.d_len > 0)
            {
              emac_transmit(priv);
            }
        }
#endif
    }
  while (true); /* While there are more packets to be processed */
}

/****************************************************************************
 * Function: emac_txdone
 *
 * Description:
 *   An interrupt was received indicating that the last TX packet(s) is done
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by the watchdog logic.
 *
 ****************************************************************************/

static void emac_txdone(FAR struct emac_driver_s *priv)
{
  /* Check for errors and update statistics */

  /* If no further xmits are pending, then cancel the TX timeout and
   * disable further Tx interrupts.
   */

  wd_cancel(&priv->d_txtimeout);

  /* Then poll the network for new XMIT data */

  devif_poll(&priv->d_dev, emac_txpoll);
}

/****************************************************************************
 * Function: emac_interrupt
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
 *
 ****************************************************************************/

static int emac_interrupt(int irq, FAR void *context, FAR void *arg)
{
  register FAR struct emac_driver_s *priv = &g_emac[0];

  /* Get and clear interrupt status bits */

  /* Handle interrupts according to status bit settings */

  /* Check if we received an incoming packet, if so, call emac_receive() */

  emac_receive(priv);

  /* Check is a packet transmission just completed.  If so, call emac_txdone.
   * This may disable further Tx interrupts if there are no pending
   * tansmissions.
   */

  emac_txdone(priv);

  return OK;
}

/****************************************************************************
 * Function: emac_txtimeout
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
 *   Global interrupts are disabled by the watchdog logic.
 *
 ****************************************************************************/

static void emac_txtimeout(wdparm_t arg)
{
  FAR struct emac_driver_s *priv = (FAR struct emac_driver_s *)arg;

  /* Increment statistics and dump debug info */

  /* Then reset the hardware */

  /* Then poll the network for new XMIT data */

  devif_poll(&priv->d_dev, emac_txpoll);
}

/****************************************************************************
 * Function: emac_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the Ethernet interface when an IP address is
 *   provided
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int emac_ifup(struct net_driver_s *dev)
{
  FAR struct emac_driver_s *priv =
    (FAR struct emac_driver_s *)dev->d_private;

  ninfo("Bringing up: %d.%d.%d.%d\n",
        dev->d_ipaddr & 0xff, (dev->d_ipaddr >> 8) & 0xff,
        (dev->d_ipaddr >> 16) & 0xff, dev->d_ipaddr >> 24);

  /* Initialize PHYs, Ethernet interface, and setup up Ethernet interrupts */

  /* Enable the Ethernet interrupt */

  priv->d_bifup = true;
  up_enable_irq(CONFIG_HCS12_IRQ);
  return OK;
}

/****************************************************************************
 * Function: emac_ifdown
 *
 * Description:
 *   NuttX Callback: Stop the interface.
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int emac_ifdown(struct net_driver_s *dev)
{
  FAR struct emac_driver_s *priv =
    (FAR struct emac_driver_s *)dev->d_private;
  irqstate_t flags;

  /* Disable the Ethernet interrupt */

  flags = enter_critical_section();
  up_disable_irq(CONFIG_HCS12_IRQ);

  /* Cancel the TX timeout timers */

  wd_cancel(&priv->d_txtimeout);

  /* Put the EMAC is its reset, non-operational state.  This should be
   * a known configuration that will guarantee the emac_ifup() always
   * successfully brings the interface back up.
   */

  /* Mark the device "down" */

  priv->d_bifup = false;
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Function: emac_txavail
 *
 * Description:
 *   Driver callback invoked when new TX data is available.  This is a
 *   stimulus perform an out-of-cycle poll and, thereby, reduce the TX
 *   latency.
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called in normal user mode
 *
 ****************************************************************************/

static int emac_txavail(struct net_driver_s *dev)
{
  FAR struct emac_driver_s *priv =
    (FAR struct emac_driver_s *)dev->d_private;
  irqstate_t flags;

  /* Disable interrupts because this function may be called from interrupt
   * level processing.
   */

  flags = enter_critical_section();

  /* Ignore the notification if the interface is not yet up */

  if (priv->d_bifup)
    {
      /* Check if there is room in the hardware to hold another packet. */

      /* If so, then poll the network for new XMIT data */

      devif_poll(&priv->d_dev, emac_txpoll);
    }

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Function: emac_addmac
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
static int emac_addmac(struct net_driver_s *dev, FAR const uint8_t *mac)
{
  FAR struct emac_driver_s *priv =
    (FAR struct emac_driver_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

  return OK;
}
#endif

/****************************************************************************
 * Function: emac_rmmac
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
static int emac_rmmac(struct net_driver_s *dev, FAR const uint8_t *mac)
{
  FAR struct emac_driver_s *priv =
    (FAR struct emac_driver_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: emac_initialize
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
 *
 ****************************************************************************/

int emac_initialize(int intf)
{
  struct emac_driver_s *priv;

  /* Get the interface structure associated with this interface number. */

  DEBUGASSERT(inf < CONFIG_HCS12_NINTERFACES);
  priv = &g_emac[intf];

  /* Check if a Ethernet chip is recognized at its I/O base */

  /* Attach the IRQ to the driver */

  if (irq_attach(CONFIG_HCS12_IRQ, emac_interrupt, NULL))
    {
      /* We could not attach the ISR to the interrupt */

      return -EAGAIN;
    }

  /* Initialize the driver structure */

  memset(priv, 0, sizeof(struct emac_driver_s));
  priv->d_dev.d_buf     = g_pktbuf[inf]; /* Single packet buffer */
  priv->d_dev.d_ifup    = emac_ifup;     /* I/F down callback */
  priv->d_dev.d_ifdown  = emac_ifdown;   /* I/F up (new IP address) callback */
  priv->d_dev.d_txavail = emac_txavail;  /* New TX data callback */
#ifdef CONFIG_NET_MCASTGROUP
  priv->d_dev.d_addmac  = emac_addmac;   /* Add multicast MAC address */
  priv->d_dev.d_rmmac   = emac_rmmac;    /* Remove multicast MAC address */
#endif
  priv->d_dev.d_private = priv;          /* Used to recover private state from dev */

  /* Put the interface in the down state.  This usually amounts to resetting
   * the device and/or calling emac_ifdown().
   */

  /* Read the MAC address from the hardware into
   * priv->d_dev.d_mac.ether.ether_addr_octet
   */

  /* Register the device with the OS so that socket IOCTLs can be performed */

  netdev_register(&priv->d_dev, NET_LL_ETHERNET);
  return OK;
}

#endif /* CONFIG_NET && CONFIG_HCS12_EMAC */
