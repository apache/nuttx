/****************************************************************************
 * arch/arm/src/lpc54xx/lpx54_ethernet.c
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

/* TODO:
 *
 * Timestamps not supported
 * Multi-queuing not supported.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/ioctl.h>
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
#include <nuttx/net/mii.h>
#include <nuttx/net/arp.h>
#include <nuttx/net/netdev.h>

#ifdef CONFIG_NET_PKT
#  include <nuttx/net/pkt.h>
#endif

#include "up_arch.h"
#include "chip/lpc54_syscon.h"
#include "chip/lpc54_ethernet.h"
#include "lpc54_enableclk.h"
#include "lpc54_reset.h"

#ifdef CONFIG_LPC54_ETHERNET

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Work queue support is required. */

#if !defined(CONFIG_SCHED_WORKQUEUE)
#  error Work queue support is required in this configuration (CONFIG_SCHED_WORKQUEUE)
#else

/* The low priority work queue is preferred.  If it is not enabled, LPWORK
 * will be the same as HPWORK.
 */

#define ETHWORK LPWORK

/* TX poll delay = 1 seconds. CLK_TCK is the number of clock ticks per second */

#define LPC54_WDDELAY   (1*CLK_TCK)

/* TX timeout = 1 minute */

#define LPC54_TXTIMEOUT (60*CLK_TCK)

/* PHY-related definitions */

#define LPC54_PHY_TIMEOUT 0x00ffffff    /* Timeout for PHY register accesses */

#ifdef CONFIG_ETH0_PHY_LAN8720
#  define LPC54_PHYID1_VAL MII_PHYID1_LAN8720
#else
#  error Unrecognized PHY selection
#endif

/* This is a helper pointer for accessing the contents of the Ethernet header */

#define BUF ((struct eth_hdr_s *)priv->eth_dev.d_buf)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The lpc54_ethdriver_s encapsulates all state information for a single
 * Ethernet interface
 */

struct lpc54_ethdriver_s
{
  bool eth_bifup;               /* true:ifup false:ifdown */
  bool eth_fullduplex;          /* true:Full duplex false:Half duplex mode */
  bool eth_100mbps;             /* true:100mbps false:10mbps */
  WDOG_ID eth_txpoll;           /* TX poll timer */
  WDOG_ID eth_txtimeout;        /* TX timeout timer */
  struct work_s eth_irqwork;    /* For deferring interupt work to the work queue */
  struct work_s eth_pollwork;   /* For deferring poll work to the work queue */

  /* This holds the information visible to the NuttX network */

  struct net_driver_s eth_dev;  /* Interface understood by the network */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* These statically allocated structures are possible because only a single
 * instance of the Ethernet device could be supported.  In order to support
 * multiple devices instances, this data would have to be allocated
 * dynamically.
 */

/* A single packet buffer per device is used here.  There might be multiple
 * packet buffers in a more complex, pipelined design.
 */

static uint8_t g_pktbuf[MAX_NET_DEV_MTU + CONFIG_NET_GUARDSIZE];

/* Driver state structure */

static struct lpc54_ethdriver_s g_ethdriver;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Common TX logic */

static int  lpc54_eth_transmit(FAR struct lpc54_ethdriver_s *priv);
static int  lpc54_eth_txpoll(FAR struct net_driver_s *dev);

/* Interrupt handling */

static void lpc54_eth_receive(FAR struct lpc54_ethdriver_s *priv);
static void lpc54_eth_txdone(FAR struct lpc54_ethdriver_s *priv);

static void lpc54_eth_interrupt_work(FAR void *arg);
static int  lpc54_eth_interrupt(int irq, FAR void *context, FAR void *arg);

/* Watchdog timer expirations */

static void lpc54_eth_txtimeout_work(FAR void *arg);
static void lpc54_eth_txtimeout_expiry(int argc, wdparm_t arg, ...);

static void lpc54_eth_poll_work(FAR void *arg);
static void lpc54_eth_poll_expiry(int argc, wdparm_t arg, ...);

/* NuttX callback functions */

static int  lpc54_eth_ifup(FAR struct net_driver_s *dev);
static int  lpc54_eth_ifdown(FAR struct net_driver_s *dev);

static void lpc54_eth_txavail_work(FAR void *arg);
static int  lpc54_eth_txavail(FAR struct net_driver_s *dev);

#if defined(CONFIG_NET_IGMP) || defined(CONFIG_NET_ICMPv6)
static int  lpc54_eth_addmac(FAR struct net_driver_s *dev,
              FAR const uint8_t *mac);
#ifdef CONFIG_NET_IGMP
static int  lpc54_eth_rmmac(FAR struct net_driver_s *dev,
              FAR const uint8_t *mac);
#endif
#ifdef CONFIG_NET_ICMPv6
static void lpc54_eth_ipv6multicast(FAR struct lpc54_ethdriver_s *priv);
#endif
#endif
#ifdef CONFIG_NETDEV_IOCTL
static int  lpc54_eth_ioctl(FAR struct net_driver_s *dev, int cmd,
              unsigned long arg);
#endif

/* Initialization/PHY control */

static void lpc54_set_csrdiv(void);
static uint16_t lpc54_phy_read(FAR struct lpc54_ethdriver_s *priv,
              uint8_t phyreg);
static void lpc54_phy_write(FAR struct lpc54_ethdriver_s *priv,
              uint8_t phyreg, uint16_t phyval);
static inline bool lpc54_phy_linkstatus(ENET_Type *base);
static int  lpc54_phy_autonegotiate(FAR struct lpc54_ethdriver_s *priv);
static int  lpc54_phy_reset(FAR struct lpc54_ethdriver_s *priv);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc54_eth_transmit
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

static int lpc54_eth_transmit(FAR struct lpc54_ethdriver_s *priv)
{
  /* Verify that the hardware is ready to send another packet.  If we get
   * here, then we are committed to sending a packet; Higher level logic
   * must have assured that there is no transmission in progress.
   */
#warning Missing logic

  /* Increment statistics */

  NETDEV_TXPACKETS(priv->eth_dev);

  /* Send the packet: address=priv->eth_dev.d_buf, length=priv->eth_dev.d_len */
#warning Missing logic

  /* Enable Tx interrupts */
#warning Missing logic

  /* Setup the TX timeout watchdog (perhaps restarting the timer) */

  (void)wd_start(priv->eth_txtimeout, LPC54_TXTIMEOUT,
                 lpc54_eth_txtimeout_expiry, 1, (wdparm_t)priv);
  return OK;
}

/****************************************************************************
 * Name: lpc54_eth_txpoll
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

static int lpc54_eth_txpoll(FAR struct net_driver_s *dev)
{
  FAR struct lpc54_ethdriver_s *priv = (FAR struct lpc54_ethdriver_s *)dev->d_private;

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  if (priv->eth_dev.d_len > 0)
    {
      /* Look up the destination MAC address and add it to the Ethernet
       * header.
       */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
      if (IFF_IS_IPv4(priv->eth_dev.d_flags))
#endif
        {
          arp_out(&priv->eth_dev);
        }
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
      else
#endif
        {
          neighbor_out(&priv->eth_dev);
        }
#endif /* CONFIG_NET_IPv6 */

      /* Send the packet */

      lpc54_eth_transmit(priv);

      /* Check if there is room in the device to hold another packet. If not,
       * return a non-zero value to terminate the poll.
       */
    }

  /* If zero is returned, the polling will continue until all connections have
   * been examined.
   */

  return 0;
}

/****************************************************************************
 * Name: lpc54_eth_receive
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

static void lpc54_eth_receive(FAR struct lpc54_ethdriver_s *priv)
{
  do
    {
      /* Check for errors and update statistics */
#warning Missing logic

      /* Check if the packet is a valid size for the network buffer
       * configuration.
       */
#warning Missing logic

      /* Copy the data data from the hardware to priv->eth_dev.d_buf.  Set
       * amount of data in priv->eth_dev.d_len
       */
#warning Missing logic

#ifdef CONFIG_NET_PKT
      /* When packet sockets are enabled, feed the frame into the packet tap */

       pkt_input(&priv->eth_dev);
#endif

      /* We only accept IP packets of the configured type and ARP packets */

#ifdef CONFIG_NET_IPv4
      if (BUF->type == HTONS(ETHTYPE_IP))
        {
          ninfo("IPv4 frame\n");
          NETDEV_RXIPV4(&priv->eth_dev);

          /* Handle ARP on input then give the IPv4 packet to the network
           * layer
           */

          arp_ipin(&priv->eth_dev);
          ipv4_input(&priv->eth_dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, the field  d_len will set to a value > 0.
           */

          if (priv->eth_dev.d_len > 0)
            {
              /* Update the Ethernet header with the correct MAC address */

#ifdef CONFIG_NET_IPv6
              if (IFF_IS_IPv4(priv->eth_dev.d_flags))
#endif
                {
                  arp_out(&priv->eth_dev);
                }
#ifdef CONFIG_NET_IPv6
              else
                {
                  neighbor_out(&kel->eth_dev);
                }
#endif

              /* And send the packet */

              lpc54_eth_transmit(priv);
            }
        }
      else
#endif
#ifdef CONFIG_NET_IPv6
      if (BUF->type == HTONS(ETHTYPE_IP6))
        {
          ninfo("Iv6 frame\n");
          NETDEV_RXIPV6(&priv->eth_dev);

          /* Give the IPv6 packet to the network layer */

          ipv6_input(&priv->eth_dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, the field  d_len will set to a value > 0.
           */

          if (priv->eth_dev.d_len > 0)
           {
              /* Update the Ethernet header with the correct MAC address */

#ifdef CONFIG_NET_IPv4
              if (IFF_IS_IPv4(priv->eth_dev.d_flags))
                {
                  arp_out(&priv->eth_dev);
                }
              else
#endif
#ifdef CONFIG_NET_IPv6
                {
                  neighbor_out(&priv->eth_dev);
                }
#endif

              /* And send the packet */

              lpc54_eth_transmit(priv);
            }
        }
      else
#endif
#ifdef CONFIG_NET_ARP
      if (BUF->type == htons(ETHTYPE_ARP))
        {
          arp_arpin(&priv->eth_dev);
          NETDEV_RXARP(&priv->eth_dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, the field  d_len will set to a value > 0.
           */

          if (priv->eth_dev.d_len > 0)
            {
              lpc54_eth_transmit(priv);
            }
        }
      else
#endif
        {
          NETDEV_RXDROPPED(&priv->eth_dev);
        }
    }
  while (); /* While there are more packets to be processed */
}

/****************************************************************************
 * Name: lpc54_eth_txdone
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

static void lpc54_eth_txdone(FAR struct lpc54_ethdriver_s *priv)
{
  int delay;

  /* Check for errors and update statistics */
#warning Missing logic

  NETDEV_TXDONE(priv->eth_dev);

  /* Check if there are pending transmissions */
#warning Missing logic

  /* If no further transmissions are pending, then cancel the TX timeout and
   * disable further Tx interrupts.
   */

  wd_cancel(priv->eth_txtimeout);

  /* And disable further TX interrupts. */
#warning Missing logic

  /* In any event, poll the network for new TX data */

  (void)devif_poll(&priv->eth_dev, lpc54_eth_txpoll);
}

/****************************************************************************
 * Name: lpc54_eth_interrupt_work
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

static void lpc54_eth_interrupt_work(FAR void *arg)
{
  FAR struct lpc54_ethdriver_s *priv = (FAR struct lpc54_ethdriver_s *)arg;

  /* Lock the network and serialize driver operations if necessary.
   * NOTE: Serialization is only required in the case where the driver work
   * is performed on an LP worker thread and where more than one LP worker
   * thread has been configured.
   */

  net_lock();

  /* Process pending Ethernet interrupts */
#warning Missing logic

  /* Get and clear interrupt status bits */
#warning Missing logic

  /* Handle interrupts according to status bit settings */
#warning Missing logic

  /* Check if we received an incoming packet, if so, call lpc54_eth_receive() */
#warning Missing logic

  lpc54_eth_receive(priv);

  /* Check if a packet transmission just completed.  If so, call lpc54_eth_txdone.
   * This may disable further Tx interrupts if there are no pending
   * transmissions.
   */
#warning Missing logic

  lpc54_eth_txdone(priv);
  net_unlock();

  /* Re-enable Ethernet interrupts */

  up_enable_irq(LPC54_IRQ_ETHERNET);
}

/****************************************************************************
 * Name: lpc54_eth_interrupt
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

static int lpc54_eth_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct lpc54_ethdriver_s *priv = (FAR struct lpc54_ethdriver_s *)arg;

  DEBUGASSERT(priv != NULL);

  /* Disable further Ethernet interrupts.  Because Ethernet interrupts are
   * also disabled if the TX timeout event occurs, there can be no race
   * condition here.
   */

  up_disable_irq(LPC54_IRQ_ETHERNET);

  /* TODO: Determine if a TX transfer just completed */
#warning Missing logic

    {
      /* If a TX transfer just completed, then cancel the TX timeout so
       * there will be no race condition between any subsequent timeout
       * expiration and the deferred interrupt processing.
       */

       wd_cancel(priv->eth_txtimeout);
    }

  /* Schedule to perform the interrupt processing on the worker thread. */

  work_queue(ETHWORK, &priv->eth_irqwork, lpc54_eth_interrupt_work, priv, 0);
  return OK;
}

/****************************************************************************
 * Name: lpc54_eth_txtimeout_work
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

static void lpc54_eth_txtimeout_work(FAR void *arg)
{
  FAR struct lpc54_ethdriver_s *priv = (FAR struct lpc54_ethdriver_s *)arg;

  /* Lock the network and serialize driver operations if necessary.
   * NOTE: Serialization is only required in the case where the driver work
   * is performed on an LP worker thread and where more than one LP worker
   * thread has been configured.
   */

  net_lock();

  /* Increment statistics and dump debug info */

  NETDEV_TXTIMEOUTS(priv->eth_dev);

  /* Then reset the hardware */
#warning Missing logic

  /* Then poll the network for new XMIT data */

  (void)devif_poll(&priv->eth_dev, lpc54_eth_txpoll);
  net_unlock();
}

/****************************************************************************
 * Name: lpc54_eth_txtimeout_expiry
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

static void lpc54_eth_txtimeout_expiry(int argc, wdparm_t arg, ...)
{
  FAR struct lpc54_ethdriver_s *priv = (FAR struct lpc54_ethdriver_s *)arg;

  /* Disable further Ethernet interrupts.  This will prevent some race
   * conditions with interrupt work.  There is still a potential race
   * condition with interrupt work that is already queued and in progress.
   */

  up_disable_irq(LPC54_IRQ_ETHERNET);

  /* Schedule to perform the TX timeout processing on the worker thread. */

  work_queue(ETHWORK, &priv->eth_irqwork, lpc54_eth_txtimeout_work, priv, 0);
}

/****************************************************************************
 * Name: lpc54_eth_poll_process
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

static inline void lpc54_eth_poll_process(FAR struct lpc54_ethdriver_s *priv)
{
#warning Missing logic
}

/****************************************************************************
 * Name: lpc54_eth_poll_work
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

static void lpc54_eth_poll_work(FAR void *arg)
{
  FAR struct lpc54_ethdriver_s *priv = (FAR struct lpc54_ethdriver_s *)arg;

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
#warning Missing logic

  /* If so, update TCP timing states and poll the network for new XMIT data.
   * Hmmm.. might be bug here.  Does this mean if there is a transmit in
   * progress, we will missing TCP time state updates?
   */

  (void)devif_timer(&priv->eth_dev, lpc54_eth_txpoll);

  /* Setup the watchdog poll timer again */

  (void)wd_start(priv->eth_txpoll, LPC54_WDDELAY, lpc54_eth_poll_expiry, 1,
                 (wdparm_t)priv);
  net_unlock();
}

/****************************************************************************
 * Name: lpc54_eth_poll_expiry
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

static void lpc54_eth_poll_expiry(int argc, wdparm_t arg, ...)
{
  FAR struct lpc54_ethdriver_s *priv = (FAR struct lpc54_ethdriver_s *)arg;

  /* Schedule to perform the interrupt processing on the worker thread. */

  work_queue(ETHWORK, &priv->eth_pollwork, lpc54_eth_poll_work, priv, 0);
}

/****************************************************************************
 * Name: lpc54_eth_ifup
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

static int lpc54_eth_ifup(FAR struct net_driver_s *dev)
{
  FAR struct lpc54_ethdriver_s *priv = (FAR struct lpc54_ethdriver_s *)dev->d_private;
  uintptr_t base;
  uint32_t regval;
  uint32_t burstlen;
  int i;

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

  /* Initialize the PHY */

  ret = lpc54_phy_autonegotiate(priv);
  if (ret < 0)
    {
      nerr("ERROR: lpc54_phy_autonegotiate failed: %d\n", ret);
      return ret;
    }

  /* Initialize the Ethernet interface */
  /* Initialize Ethernet DMA */
  /* Reset DMA */

  regval  = getreg32(LPC54_ETH_DMA_MODE);
  regval |= ETH_DMA_MODE_SWR;
  putreg32(regval, LPC54_ETH_DMA_MODE);

  /* Wait for the reset bit to be cleared at the completion of the reset */

  while ((getreg32(LPC54_ETH_DMA_MODE) & ETH_DMA_MODE_SWR) != 0)
    {
    }

  /* Set the burst length for each DMA descriptor ring */

  for (i = 0; i < ENET_RING_NUM_MAX; i++)
    {
      base = LPC54_ETH_DMA_CH_CTRL_BASE(i);

      /* REVISIT: burstlen setting for the case of multi-queuing.
       * REVISIT: Additional logic needed if burstlen > 32.
       */

      burstlen = 1;  /* DMA burst length = 1 */

      /* REVISIT: We would need to set ETH_DMACH_CTRL_PBLx8 in LPC54_ETH_DMACH_CTRL
       * is required for the burst length setting.
       */

      putreg32(0, base + LPC54_ETH_DMACH_CTRL_OFFSET);

      regval  = getreg(base + LPC54_ETH_DMACH_TX_CTRL_OFFSET);
      regval &= ~ETH_DMACH_TX_CTRL_TxPBL_MASK;
      regval |= ETH_DMACH_TX_CTRL_TxPBL(burstlen);
      putreg32(regval, base + LPC54_ETH_DMACH_TX_CTRL_OFFSET);


      regval = getreg(base + LPC54_ETH_DMACH_RX_CTRL_OFFSET);
      regval &= ~ETH_DMACH_RX_CTRL_RxPBL_MASK;
      regval |= ETH_DMACH_RX_CTRL_RxPBL(burstlen);
      putreg32(regval, base + LPC54_ETH_DMACH_RX_CTRL_OFFSET);
    }

  /* Initializes the Ethernet MTL */
#warning Missing logic

  /* Initialize the Ethernet MAC */
#warning Missing logic

  /* Setup up Ethernet interrupts */
#warning Missing logic

  /* Set the sideband flow control for each channel */
#warning Missing logic

  /* Instantiate the MAC address from priv->eth_dev.d_mac.ether.ether_addr_octet */
#warning Missing logic

#ifdef CONFIG_NET_ICMPv6
  /* Set up IPv6 multicast address filtering */

  lpc54_eth_ipv6multicast(priv);
#endif

  /* Set and activate a timer process */

  (void)wd_start(priv->eth_txpoll, LPC54_WDDELAY, lpc54_eth_poll_expiry, 1,
                 (wdparm_t)priv);

  /* Enable the Ethernet interrupt */

  priv->eth_bifup = true;
  up_enable_irq(LPC54_IRQ_ETHERNET);
  return OK;
}

/****************************************************************************
 * Name: lpc54_eth_ifdown
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

static int lpc54_eth_ifdown(FAR struct net_driver_s *dev)
{
  FAR struct lpc54_ethdriver_s *priv = (FAR struct lpc54_ethdriver_s *)dev->d_private;
  irqstate_t flags;
  uint32_t regval;

  /* Disable the Ethernet interrupt */

  flags = enter_critical_section();
  up_disable_irq(LPC54_IRQ_ETHERNET);

  /* Cancel the TX poll timer and TX timeout timers */

  wd_cancel(priv->eth_txpoll);
  wd_cancel(priv->eth_txtimeout);

  /* Put the EMAC in its reset, non-operational state.  This should be
   * a known configuration that will guarantee the lpc54_eth_ifup() always
   * successfully brings the interface back up.
   *
   * Reset the Ethernet interface.
   */

  lpc54_reset_eth();

  /* Select MII or RMII mode */

  regval  = getreg32(LPC54_SYSCON_ETHPHYSEL);
  regval &= ~SYSCON_ETHPHYSEL_MASK;
#ifdef CONFIG_LPC54_MII
  retval |= SYSCON_ETHPHYSEL_MII;
#else
  retval |= SYSCON_ETHPHYSEL_RMII;
#endif
  putreg32(regval, LPC54_SYSCON_ETHPHYSEL);

  /* Reset the PHY and bring it to an operational state.  We must be capable
   * of handling PHY ioctl commands while the network is down.
   */

  ret = lpc54_phy_reset(priv);
  if (ret < 0)
    {
      nerr("ERROR: lpc54_phy_reset failed: %d\n", ret);
      return ret;
    }

  /* Mark the device "down" */

  priv->eth_bifup = false;
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: lpc54_eth_txavail_work
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

static void lpc54_eth_txavail_work(FAR void *arg)
{
  FAR struct lpc54_ethdriver_s *priv = (FAR struct lpc54_ethdriver_s *)arg;

  /* Lock the network and serialize driver operations if necessary.
   * NOTE: Serialization is only required in the case where the driver work
   * is performed on an LP worker thread and where more than one LP worker
   * thread has been configured.
   */

  net_lock();

  /* Ignore the notification if the interface is not yet up */

  if (priv->eth_bifup)
    {
      /* Check if there is room in the hardware to hold another outgoing packet. */
#warning Missing logic

      /* If so, then poll the network for new XMIT data */

      (void)devif_poll(&priv->eth_dev, lpc54_eth_txpoll);
    }

  net_unlock();
}

/****************************************************************************
 * Name: lpc54_eth_txavail
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

static int lpc54_eth_txavail(FAR struct net_driver_s *dev)
{
  FAR struct lpc54_ethdriver_s *priv = (FAR struct lpc54_ethdriver_s *)dev->d_private;

  /* Is our single work structure available?  It may not be if there are
   * pending interrupt actions and we will have to ignore the Tx
   * availability action.
   */

  if (work_available(&priv->eth_pollwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      work_queue(ETHWORK, &priv->eth_pollwork, lpc54_eth_txavail_work, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: lpc54_eth_addmac
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
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_IGMP) || defined(CONFIG_NET_ICMPv6)
static int lpc54_eth_addmac(FAR struct net_driver_s *dev, FAR const uint8_t *mac)
{
  FAR struct lpc54_ethdriver_s *priv = (FAR struct lpc54_ethdriver_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */
#warning Missing logic

  return OK;
}
#endif

/****************************************************************************
 * Name: lpc54_eth_rmmac
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
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IGMP
static int lpc54_eth_rmmac(FAR struct net_driver_s *dev, FAR const uint8_t *mac)
{
  FAR struct lpc54_ethdriver_s *priv = (FAR struct lpc54_ethdriver_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */
#warning Missing logic

  return OK;
}
#endif

/****************************************************************************
 * Name: lpc54_eth_ipv6multicast
 *
 * Description:
 *   Configure the IPv6 multicast MAC address.
 *
 * Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6
static void lpc54_eth_ipv6multicast(FAR struct lpc54_ethdriver_s *priv)
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

  (void)lpc54_eth_addmac(dev, mac);

#ifdef CONFIG_NET_ICMPv6_AUTOCONF
  /* Add the IPv6 all link-local nodes Ethernet address.  This is the
   * address that we expect to receive ICMPv6 Router Advertisement
   * packets.
   */

  (void)lpc54_eth_addmac(dev, g_ipv6_ethallnodes.ether_addr_octet);

#endif /* CONFIG_NET_ICMPv6_AUTOCONF */

#ifdef CONFIG_NET_ICMPv6_ROUTER
  /* Add the IPv6 all link-local routers Ethernet address.  This is the
   * address that we expect to receive ICMPv6 Router Solicitation
   * packets.
   */

  (void)lpc54_eth_addmac(dev, g_ipv6_ethallrouters.ether_addr_octet);

#endif /* CONFIG_NET_ICMPv6_ROUTER */
}
#endif /* CONFIG_NET_ICMPv6 */

/****************************************************************************
 * Name: lpc54_eth_ioctl
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
static int lpc54_eth_ioctl(FAR struct net_driver_s *dev, int cmd,
                      unsigned long arg)
{
  FAR struct lpc54_ethdriver_s *priv = (FAR struct lpc54_ethdriver_s *)dev->d_private;
  int ret;

  /* Decode and dispatch the driver-specific IOCTL command */

  switch (cmd)
    {
     case SIOCGMIIPHY: /* Get MII PHY address */
        {
          struct mii_ioctl_data_s *req = (struct mii_ioctl_data_s *)((uintptr_t)arg);
          req->phy_id = CONFIG_LPC54_PHYADDR;
          ret = OK;
        }
        break;

      case SIOCGMIIREG: /* Get register from MII PHY */
        {
          struct mii_ioctl_data_s *req = (struct mii_ioctl_data_s *)((uintptr_t)arg);
          req->val_out = lpc54_phy_read(priv, req->reg_num);
          ret = OK
        }
        break;

      case SIOCSMIIREG: /* Set register in MII PHY */
        {
          struct mii_ioctl_data_s *req = (struct mii_ioctl_data_s *)((uintptr_t)arg);
          lpc54_phy_write(priv, req->reg_num, req->val_in);
          ret = OK
        }
        break;

      default:
        nerr("ERROR: Unrecognized IOCTL command: %d\n", command);
        return -ENOTTY;  /* Special return value for this case */
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: lpc54_set_csrdiv
 *
 * Description:
 *   Set the CSR clock divider.  The MDC clock derives from the divided down
 *   CSR clock (aka core clock or main clock).
 *
 * Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lpc54_set_csrdiv(void)
{
  uint32_t srcclk = BOARD_MAIN_CLK / 1000000;
  uint32_t regval;

  regval = getreg32(LPC54_ETH_MAC_MDIO_ADDR);
  regval &= ~ETH_MAC_MDIO_ADDR_CR_MASK;

  if (srcclk < 35)
    {
      regval |= ETH_MAC_MDIO_ADDR_CR_DIV16;    /* CSR=20-35 MHz; MDC=CSR/16 */
    }
  else if (srcclk < 60)
    {
      regval |= ETH_MAC_MDIO_ADDR_CR_DIV26;    /* CSR=35-60 MHz; MDC=CSR/26 */
    }
  else if (srcclk < 100)
    {
      regval |= ETH_MAC_MDIO_ADDR_CR_ DIV42;   /* CSR=60-100 MHz; MDC=CSR/42 */
    }
  else /* if (srcclk < 150) */
    {
      regval |= ETH_MAC_MDIO_ADDR_CR_DIV62;    /* CSR=100-150 MHz; MDC=CSR/62 */
    }

  putreg32(regval, LPC54_ETH_MAC_MDIO_ADDR);
}

/****************************************************************************
 * Name: lpc54_phy_read
 *
 * Description:
 *   Read the content from one PHY register.
 *
 * Parameters:
 *   priv   - Reference to the driver state structure
 *   phyreg - The 5-bit PHY address to read
 *
 * Returned Value:
 *   The 16-bit value read from the specified PHY register
 *
 ****************************************************************************/

static uint16_t lpc54_phy_read(FAR struct lpc54_ethdriver_s *priv,
                               uint8_t phyreg)
{
  uint32_t regval = base->MAC_MDIO_ADDR & ENET_MAC_MDIO_ADDR_CR_MASK;

  /* Set the MII read command. */

  regval  = getreg32(LPC54_ETH_MAC_MDIO_ADDR);
  regval &= ETH_MAC_MDIO_ADDR_CR_MASK;
  regval |= ETH_MAC_MDIO_ADDR_MOC_READ | ETH_MAC_MDIO_ADDR_RDA(phyreg) |
            ETH_MAC_MDIO_ADDR_PA(CONFIG_LPC54_PHYADDR);
  putreg32(regval, LPC54_ETH_MAC_MDIO_ADDR);

  /* Initiate the read */

  regval |= ETH_MAC_MDIO_ADDR_MB;
  putreg32(regval, LPC54_ETH_MAC_MDIO_ADDR);

  /* Wait until the SMI is no longer busy with the read */

  while ((getreg32(LPC54_ETH_MAC_MDIO_ADDR) & ETH_MAC_MDIO_ADDR_MB) != 0)
    {
    }

  return (uint16_t)getreg32(LPC54_ETH_MAC_MDIO_DATA);
}

/****************************************************************************
 * Name: lpc54_phy_write
 *
 * Description:
 *   Write a new value to of one PHY register.
 *
 * Parameters:
 *   priv   - Reference to the driver state structure
 *   phyreg - The 5-bit PHY address to write
 *   phyval - The 16-bit value to write to the PHY register
 *
 * Returned Value:
 *   The 16-bit value read from the specified PHY register
 *
 ****************************************************************************/

static void lpc54_phy_write(FAR struct lpc54_ethdriver_s *priv,
                            uint8_t phyreg, uint16_t phyval)
{
  uint32_t regval = base->MAC_MDIO_ADDR & ENET_MAC_MDIO_ADDR_CR_MASK;

  /* Set the MII write command. */

  regval  = getreg32(LPC54_ETH_MAC_MDIO_ADDR);
  regval &= ETH_MAC_MDIO_ADDR_CR_MASK;
  regval |= ETH_MAC_MDIO_ADDR_MOC_WRITE | ETH_MAC_MDIO_ADDR_RDA(phyreg) |
            ETH_MAC_MDIO_ADDR_PA(CONFIG_LPC54_PHYADDR);
  putreg32(regval, LPC54_ETH_MAC_MDIO_ADDR);

  /* Set the write data */

  putreg32((uint32_t)phyval, LPC54_ETH_MAC_MDIO_DATA);

  /* Initiate the write */

  regval |= ETH_MAC_MDIO_ADDR_MB;
  putreg32(regval, LPC54_ETH_MAC_MDIO_ADDR);

  /* Wait until the SMI is no longer busy with the write */

  while ((getreg32(LPC54_ETH_MAC_MDIO_ADDR) & ETH_MAC_MDIO_ADDR_MB) != 0)
    {
    }
}

/****************************************************************************
 * Name: lpc54_phy_linkstatus
 *
 * Description:
 *   Read the MII status register and return tru if the link is up.
 *
 * Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   true if the link is up
 *
 ****************************************************************************/

static inline bool lpc54_phy_linkstatus(ENET_Type *base)
{
  /* Read the status register and return tru of the linkstatus bit is set. */

  return ((lpc54_phy_read(priv, MII_MSR) & MII_MSR_LINKSTATUS) != 0);
}

/****************************************************************************
 * Name: lpc54_phy_autonegotiate
 *
 * Description:
 *   Initialize the PHY.
 *
 * Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

static int lpc54_phy_autonegotiate(FAR struct lpc54_ethdriver_s *priv)
{
  volatile int32_t timeout;
  uint16_t phyid1;
  uint16_t phyval;

  /* Advertise our cabilities. */

  phyval = (MII_ADVERTISE_CSMA | MII_ADVERTISE_10BASETXHALF |
            MII_ADVERTISE_10BASETXFULL | MII_ADVERTISE_100BASETXHALF |
            MII_ADVERTISE_100BASETXFULL);
  lpc54_phy_write(priv, MII_ADVERTISE, phyval);

  /* Start Auto negotiation and wait until auto negotiation completion */

  phyval = (MII_MCR_ANENABLE | MII_MCR_ANRESTART);
  lpc54_phy_write(priv, MII_MCR, phyval);

  /* Wait for the completion of autonegotiation. */

#ifdef CONFIG_ETH0_PHY_LAN8720
  timeout = LPC54_PHY_TIMEOUT;
  do
    {
      if (timeout-- <= 0)
        {
          return -ETIMEDOUT;
        }

      phyval = lpc54_phy_read(priv, MII_LAN8720_SCSR);

    }
  while ((phyval & MII_LAN8720_SPSCR_ANEGDONE) == 0);
#else
#  error Unrecognized PHY
#endif

  /* Wait for the link to be in the UP state */

  timeout = LPC54_PHY_TIMEOUT;
  do
    {
      if (timeout-- <= 0)
        {
          return -ETIMEDOUT;
        }
    }
  while (!lpc54_phy_linkstatus(priv));

  /* Get the negotiate PHY link mode. */

#ifdef CONFIG_ETH0_PHY_LAN8720
  /* Read the LAN8720 SPCR register. */

  phyval               = lpc54_phy_read(priv, MII_LAN8720_SCSR);
  priv->eth_fullduplex = ((phyval & MII_LAN8720_SPSCR_DUPLEX) != 0);
  priv->eth_100mbps    = ((phyval & MII_LAN8720_SPSCR_100MBPS) != 0);
#else
#  error Unrecognized PHY
#endif

  return OK;
}

/****************************************************************************
 * Name: lpc54_phy_reset
 *
 * Description:
 *   Reset the PHY and bring it to the operational status
 *
 * Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

static int lpc54_phy_reset(FAR struct lpc54_ethdriver_s *priv)
{
  volatile int32_t timeout;
  uint16_t phyid1;
  uint16_t phyval;

  /* Read and verify the PHY ID1 register */

  timeout = LPC54_PHY_TIMEOUT;
  do
    {
      if (timeout-- <= 0)
        {
          return -ETIMEDOUT;
        }

      phyid1 = lpc54_phy_read(priv, MII_PHYID1);
    }
  while (phyid1 != LPC54_PHYID1_VAL);

  /* Reset PHY and wait until completion. */

  lpc54_phy_write(priv, MII_MCR, MII_MCR_RESET);

  timeout = LPC54_PHY_TIMEOUT;
  do
    {
      if (timeout-- <= 0)
        {
          return -ETIMEDOUT;
        }

      phyval = lpc54_phy_read(base, CONFIG_LPC54_PHYADDR, MII_MCR, &reg);
    }
  while ((phyval & MII_MCR_RESET) != 0);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_netinitialize
 *
 * Description:
 *   Initialize the Ethernet controller and driver.
 *
 *   This is the "standard" network initialization logic called from the
 *   low-level initialization logic in up_initialize.c.
 *
 * Parameters:
 *   intf - In the case where there are multiple EMACs, this value
 *          identifies which EMAC is to be initialized.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int up_netinitialize(int intf)
{
  FAR struct lpc54_ethdriver_s *priv;

  /* Get the interface structure associated with this interface number. */

  DEBUGASSERT(intf == 0);
  priv = &g_ethdriver;

  /* Attach the IRQ to the driver */

  if (irq_attach(LPC54_IRQ_ETHERNET, lpc54_eth_interrupt, priv))
    {
      /* We could not attach the ISR to the interrupt */

      nerr("ERROR:  irq_attach failed\n");
      return -EAGAIN;
    }

  /* Initialize the driver structure */

  memset(priv, 0, sizeof(struct lpc54_ethdriver_s));
  priv->eth_dev.d_buf     = g_pktbuf;      /* Single packet buffer */
  priv->eth_dev.d_ifup    = lpc54_eth_ifup;     /* I/F up (new IP address) callback */
  priv->eth_dev.d_ifdown  = lpc54_eth_ifdown;   /* I/F down callback */
  priv->eth_dev.d_txavail = lpc54_eth_txavail;  /* New TX data callback */
#ifdef CONFIG_NET_IGMP
  priv->eth_dev.d_addmac  = lpc54_eth_addmac;   /* Add multicast MAC address */
  priv->eth_dev.d_rmmac   = lpc54_eth_rmmac;    /* Remove multicast MAC address */
#endif
#ifdef CONFIG_NETDEV_IOCTL
  priv->eth_dev.d_ioctl   = lpc54_eth_ioctl;    /* Handle network IOCTL commands */
#endif
  priv->eth_dev.d_private = (FAR void *)g_ethdriver; /* Used to recover private state from dev */

  /* Create a watchdog for timing polling for and timing of transmisstions */

  priv->eth_txpoll        = wd_create();        /* Create periodic poll timer */
  priv->eth_txtimeout     = wd_create();        /* Create TX timeout timer */

  DEBUGASSERT(priv->eth_txpoll != NULL && priv->eth_txtimeout != NULL);

  /* Configure GPIO pins to support Ethernet */
  /* Common MIIM interface */

  lpc54_gpio_config(GPIO_ENET_MDIO);    /* Ethernet MIIM data input and output */
  lpc54_gpio_config(GPIO_ENET_MDC);     /* Ethernet MIIM clock */

#ifdef CONFIG_LPC54_MII
  /* MII interface */

  lpc54_gpio_config(GPIO_ENET_RXD0);    /* Ethernet receive data 0-3 */
  lpc54_gpio_config(GPIO_ENET_RXD1);
  lpc54_gpio_config(GPIO_ENET_RXD2);
  lpc54_gpio_config(GPIO_ENET_RXD3);
  lpc54_gpio_config(GPIO_ENET_TXD0);    /* Ethernet transmit data 0-3 */
  lpc54_gpio_config(GPIO_ENET_TXD1);
  lpc54_gpio_config(GPIO_ENET_TXD2);
  lpc54_gpio_config(GPIO_ENET_TXD3);
  lpc54_gpio_config(GPIO_ENET_COL);     /* Ethernet collision detect */
  lpc54_gpio_config(GPIO_ENET_CRS);     /* Ethernet carrier sense */
  lpc54_gpio_config(GPIO_ENET_RX_ER);   /* Ethernet transmit error */
  lpc54_gpio_config(GPIO_ENET_TX_CLK);  /* Ethernet transmit clock */
  lpc54_gpio_config(GPIO_ENET_RX_CLK);  /* Ethernet receive clock */
  lpc54_gpio_config(GPIO_ENET_TX_ER);   /* Ethernet receive error */
  lpc54_gpio_config(GPIO_ENET_TX_EN);   /* Ethernet transmit enable */
#else
  /* RMII interface.
   *
   *   REF_CLK may be available in some implementations
   *   RX_ER is optional on switches
   */

  lpc54_gpio_config(GPIO_ENET_RXD0);    /* Ethernet receive data 0-1 */
  lpc54_gpio_config(GPIO_ENET_RXD1);
  lpc54_gpio_config(GPIO_ENET_TXD0);    /* Ethernet transmit data 0-1 */
  lpc54_gpio_config(GPIO_ENET_TXD1);
  lpc54_gpio_config(GPIO_ENET_RX_DV);   /* Ethernet receive data valid */
  lpc54_gpio_config(GPIO_ENET_TX_EN);   /* Ethernet transmit data enable */
#endif

  /* Enable clocking to the Ethernet peripheral */

  lpc54_eth_enableclk();

  /* Set the CSR clock divider */

  lpc43_set_crsdiv();

  /* Put the interface in the down state.  This amounts to resetting the
   * device by calling lpc54_eth_ifdown().
   */

  ret = lpc54_eth_ifdown(&priv->eth_dev);
  if (ret < 0)
    {
      nerr("ERROR:  lpc54_eth_ifdown failed: %d\n", ret);
      goto errout_with_clock;
    }

  /* Register the device with the OS so that socket IOCTLs can be performed */

  ret = netdev_register(&priv->eth_dev, NET_LL_ETHERNET);
  if (ret < 0)
    {
      nerr("ERROR:  netdev_register failed: %d\n", ret);
      goto errout_with_clock:
    }

  return OK;

errout_with_clock:
  lpc54_eth_disableclk();
  return ret;
}

#endif /* CONFIG_LPC54_ETHERNET */
