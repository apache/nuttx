/****************************************************************************
 * arch/arm/src/lpc54xx/lpx54_ethernet.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Some of the logic in this file was developed using sample code provided by
 * NXP that has a compatible BSD license:
 *
 *   Copyright (c) 2016, Freescale Semiconductor, Inc.
 *   Copyright 2016-2017 NXP
 *   All rights reserved.
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
#include <queue.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <arpa/inet.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>
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

#define LPC54_PHY_TIMEOUT 0x00ffffff  /* Timeout for PHY register accesses */

#ifdef CONFIG_ETH0_PHY_LAN8720
#  define LPC54_PHYID1_VAL MII_PHYID1_LAN8720
#else
#  error Unrecognized PHY selection
#endif

/* MTL-related definitions */

#define LPC54_MTL_QUEUE_UNIT    256
#define LPC54_MTL_RXQUEUE_UNITS 8     /* Rx queue size = 2048 bytes */
#define LPC54_MTL_TXQUEUE_UNITS 8     /* Tx queue size = 2048 bytes */

/* MAC-related definitinons */

#define LPC54_MAC_HALFDUPLEX_IPG ETH_MAC_CONFIG_IPG_64 /* Default half-duplex IPG */

/* Packet-buffer definitions */

#ifdef CONFIG_LPC54_ETH_MULTIQUEUE
#  define LPC54_NBUFFERS        (CONFIG_LPC54_ETH_NRXDESC0 + \
                                 CONFIG_LPC54_ETH_NRXDESC1 + \
                                 CONFIG_LPC54_ETH_NTXDESC0 + \
                                 CONFIG_LPC54_ETH_NTXDESC1)
#else
#  define LPC54_NBUFFERS        (CONFIG_LPC54_ETH_NRXDESC0 + \
                                 CONFIG_LPC54_ETH_NTXDESC0)
#endif

#define LPC54_BUFFER_SIZE       MAX_NET_DEV_MTU
#define LPC54_BUFFER_ALLOC      ((MAX_NET_DEV_MTU + CONFIG_NET_GUARDSIZE + 3) & ~3)
#define LPC54_BUFFER_WORDS      ((MAX_NET_DEV_MTU + CONFIG_NET_GUARDSIZE + 3) >> 2)

/* DMA descriptor definitions */

#define LPC54_MIN_RINGLEN       4     /* Min length of a ring */
#define LPC54_MAX_RINGLEN       1023  /* Max length of a ring */
#define LPC54_MAX_RINGS         2     /* Max number of tx/rx descriptor rings */
#ifdef CONFIG_LPC54_ETH_MULTIQUEUE
#  define LPC54_NRINGS          2     /* Use 2 Rx and Tx rings */
#else
#  define LPC54_NRINGS          1     /* Use 1 Rx and 1 Tx ring */
#endif

/* Interrupt masks */

#define LPC54_ABNORM_INTMASK    (ETH_DMACH_INT_TS  | ETH_DMACH_INT_RBU | \
                                 ETH_DMACH_INT_RS  | ETH_DMACH_INT_RWT | \
                                 ETH_DMACH_INT_FBE | ETH_DMACH_INT_ETI | \
                                 ETH_DMACH_INT_AI)
#define LPC54_TXERR_INTMASK     (ETH_DMACH_INT_TS  | ETH_DMACH_INT_ETI)
#define LPC54_RXERR_INTMASK     (ETH_DMACH_INT_RBU | ETH_DMACH_INT_RS  | \
                                 ETH_DMACH_INT_RWT)
#define LPC54_NORM_INTMASK      (ETH_DMACH_INT_TI  | ETH_DMACH_INT_TBU | \
                                 ETH_DMACH_INT_RI  | ETH_DMACH_INT_ERI | \
                                 ETH_DMACH_INT_NI)

/* This is a helper pointer for accessing the contents of the Ethernet
 * header.
 */

#define BUF ((struct eth_hdr_s *)priv->eth_dev.d_buf)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Describes the state of one Tx descriptor ring */

struct lpc54_txring_s
{
  struct enet_txdesc_s *tr_desc; /* Tx descriptor base address */
  uint16_t tr_genidx;            /* Tx generate index */
  uint16_t tr_consumidx;         /* Tx consume index */
  volatile uint16_t tr_inuse;    /* Number of Tx descriptors in-used */
  uint16_t tr_ndesc;             /* Number or descriptors in the Tx ring */
  uint32_t **tr_buffers;         /* Packet buffers assigned to the Rx ring */
};

/* Describes the state of one Rx descriptor ring */

struct lpc54_rxring_s
{
  struct enet_rxdesc_s *rr_desc; /* Rx descriptor base address */
  uint16_t rr_genidx;            /* Available Rx descriptor index */
  uint16_t rr_ndesc;             /* Number or descriptors in the Rx ring */
  uint32_t **rr_buffers;         /* Packet buffers assigned to the Rx ring */
};

/* The lpc54_ethdriver_s encapsulates all state information for a single
 * Ethernet interface
 */

struct lpc54_ethdriver_s
{
  bool eth_bifup;                /* true:ifup false:ifdown */
  bool eth_fullduplex;           /* true:Full duplex false:Half duplex mode */
  bool eth_100mbps;              /* true:100mbps false:10mbps */
  WDOG_ID eth_txpoll;            /* TX poll timer */
  WDOG_ID eth_txtimeout;         /* TX timeout timer */
  struct work_s eth_irqwork;     /* For deferring interupt work to the work queue */
  struct work_s eth_pollwork;    /* For deferring poll work to the work queue */
  struct sq_queue_s eth_freebuf; /* Free packet buffers */

  /* Ring state */

  struct lpc54_txring_s eth_txring[LPC54_NRINGS];
  struct lpc54_rxring_s eth_rxring[LPC54_NRINGS];

  /* This holds the information visible to the NuttX network */

  struct net_driver_s eth_dev;   /* Interface understood by the network */
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

/* Rx DMA descriptors */

static struct enet_rxdesc_s g_ch0_rxdesc[CONFIG_LPC54_ETH_NRXDESC0];
#ifdef CONFIG_LPC54_ETH_MULTIQUEUE
static struct enet_rxdesc_s g_ch1_rxdesc[CONFIG_LPC54_ETH_NRXDESC1];
#endif

/* Tx DMA descriptors */

static struct enet_txdesc_s g_ch0_txdesc[CONFIG_LPC54_ETH_NTXDESC0];
#ifdef CONFIG_LPC54_ETH_MULTIQUEUE
static struct enet_txdesc_s g_ch1_txdesc[CONFIG_LPC54_ETH_NTXDESC1];
#endif

/* Preallocated packet buffers */

static uint32_t g_prealloc_buffers[LPC54_NBUFFERS * LPC54_BUFFER_WORDS];

/* Packet buffers assigned to Rx and Tx descriptors.  The packet buffer
 * addresses are lost in the DMA due to write-back from the DMA harware.
 * So we have to remember the buffer assignments explicitly.
 */

static uint32_t *g_rxbuffers0[CONFIG_LPC54_ETH_NRXDESC0];
static uint32_t *g_txbuffers0[CONFIG_LPC54_ETH_NTXDESC0];
#ifdef CONFIG_LPC54_ETH_MULTIQUEUE
static uint32_t *g_rxbuffers1[CONFIG_LPC54_ETH_NRXDESC1];
static uint32_t *g_txbuffers1[CONFIG_LPC54_ETH_NTXDESC1];
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Common TX logic */

static int  lpc54_eth_transmit(struct lpc54_ethdriver_s *priv);
static int  lpc54_eth_txpoll(struct net_driver_s *dev);

/* Interrupt handling */

static void lpc54_eth_receive(struct lpc54_ethdriver_s *priv,
              unsigned int chan);
static void lpc54_eth_txdone(struct lpc54_ethdriver_s *priv,
              unsigned int chan);

static void lpc54_eth_interrupt_work(void *arg);
static int  lpc54_eth_interrupt(int irq, void *context, void *arg);
#if 0 /* Not used */
static int  lpc54_pmt_interrupt(int irq, void *context, void *arg);
static int  lpc54_mac_interrupt(int irq, void *context, void *arg);
#endif

/* Watchdog timer expirations */

static void lpc54_eth_txtimeout_work(void *arg);
static void lpc54_eth_txtimeout_expiry(int argc, wdparm_t arg, ...);

static void lpc54_eth_poll_work(void *arg);
static void lpc54_eth_poll_expiry(int argc, wdparm_t arg, ...);

/* NuttX callback functions */

static int  lpc54_eth_ifup(struct net_driver_s *dev);
static int  lpc54_eth_ifdown(struct net_driver_s *dev);

static void lpc54_eth_txavail_work(void *arg);
static int  lpc54_eth_txavail(struct net_driver_s *dev);

#if defined(CONFIG_NET_IGMP) || defined(CONFIG_NET_ICMPv6)
static int  lpc54_eth_addmac(struct net_driver_s *dev,
              const uint8_t *mac);
#ifdef CONFIG_NET_IGMP
static int  lpc54_eth_rmmac(struct net_driver_s *dev,
              const uint8_t *mac);
#endif
#ifdef CONFIG_NET_ICMPv6
static void lpc54_eth_ipv6multicast(struct lpc54_ethdriver_s *priv);
#endif
#endif
#ifdef CONFIG_NETDEV_IOCTL
static int  lpc54_eth_ioctl(struct net_driver_s *dev, int cmd,
              unsigned long arg);
#endif

/* Packet buffers */

static void lpc54_pktbuf_initialize(struct lpc54_ethdriver_s *priv);
static inline uint32_t *lpc54_pktbuf_alloc(struct lpc54_ethdriver_s *priv);
static inline void lpc54_pktbuf_free(struct lpc54_ethdriver_s *priv,
              uint32_t *pktbuf);

/* DMA descriptor rings */

static void lpc54_txring_initialize(struct lpc54_ethdriver_s *priv,
              unsigned int chan);
static void lpc54_rxring_initialize(struct lpc54_ethdriver_s *priv,
              unsigned int chan);
static void lpc54_ring_initialize(struct lpc54_ethdriver_s *priv);

/* Initialization/PHY control */

static void lpc54_set_csrdiv(void);
static uint16_t lpc54_phy_read(struct lpc54_ethdriver_s *priv,
              uint8_t phyreg);
static void lpc54_phy_write(struct lpc54_ethdriver_s *priv,
              uint8_t phyreg, uint16_t phyval);
static inline bool lpc54_phy_linkstatus(ENET_Type *base);
static int  lpc54_phy_autonegotiate(struct lpc54_ethdriver_s *priv);
static int  lpc54_phy_reset(struct lpc54_ethdriver_s *priv);

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

static int lpc54_eth_transmit(struct lpc54_ethdriver_s *priv)
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

static int lpc54_eth_txpoll(struct net_driver_s *dev)
{
  struct lpc54_ethdriver_s *priv = (struct lpc54_ethdriver_s *)dev->d_private;

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
 *   chan - The channel with the completed Rx transfer
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static void lpc54_eth_receive(struct lpc54_ethdriver_s *priv,
                              unsigned int chan)
{
  do
    {
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
 *   chan - The channel with the completed Tx transfer
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static void lpc54_eth_txdone(struct lpc54_ethdriver_s *priv,
                             unsigned int chan)
{
  struct lpc54_txring_s *txring;
  struct enet_txdesc_s *txdesc;
  uint32_t *pktbuf;
  int delay;

  /* Reclaim the compled Tx descriptor */

  txring = &priv->eth_txring[channel];
  txdesc = txring->desc + txring->tr_consumidx;

  /* Update the first index for transmit buffer free. */

  while (txring->tr_inuse > 0 &&
         (txdesc->ctrlstat & ENET_TXDESCRIP_RD_OWN_MASK) == 0)
    {
      /* Update statistics */

      NETDEV_TXDONE(priv->eth_dev);

      /* Free the Tx buffer assigned to the descriptor */

      pktbuf = txring->tr_buffers[txring->tr_consumidx];
      DEBUGASSERT(pktbuf != NULL);
      if (pktbuf != NULL)
        {
          lpc54_pktbuf_free(pktbuf);
          txring->tr_buffers[txring->tr_consumidx] = NULL;
        }

      /* One less Tx descriptor in use */

      txring->tr_inuse--;

      /* Update the consume index and the descriptor pointer. */

      if (++(txring->tr_consumidx) >= txring->tr_ndesc)
        {
          txring->tr_consumidx = 0;
        }

      txdesc = txring->desc + txring->tr_consumidx;
    }

  /* If no further transmissions are pending, then cancel the TX timeout and
   * disable further Tx interrupts.
   */

  if (txring->tr_inuse == 0)
    {
      wd_cancel(priv->eth_txtimeout);

      /* And disable further TX interrupts. */
#warning Missing logic
    }

  /* In any event, poll the network for new TX data */

  (void)devif_poll(&priv->eth_dev, lpc54_eth_txpoll);
}

/****************************************************************************
 * Name: lpc54_eth_channel_work
 *
 * Description:
 *   Perform interrupt related work for a channel DMA interrupt
 *
 * Parameters:
 *   priv - Reference to the driver state structure
 *   chan - The channel that received the interrupt event.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static void lpc54_eth_channel_work(void *arg)
{
  uintptr_t regaddr;
  uint32_t status;

  /* Read the DMA status register for this channel */

  regaddr = LPC54_ETH_DMACH_STAT(chan)
  status  = getre32(regaddr);

  /* Check for abnormal interrupts */

  if ((status & LPC54_ABNORM_INTMASK) != 0)
    {
      /* Acknowledge the normal receive interrupt */

      putreg32(LPC54_ABNORM_INTMASK, regaddr);

      /* Handle the incoming packet */

      nerr("ERROR: Abnormal interrupt received: %08lx\n", (unsigned long)status);
      status &= ~LPC54_ABNORM_INTMASK;

      /* Check for Tx/Rx related errors and update statistics */

      if ((status & LPC54_RXERR_INTMASK) != 0)
        {
          NETDEV_RXERRORS(priv->eth_dev);
        }

      if ((status & LPC54_TXERR_INTMASK) != 0)
        {
          NETDEV_TXERRORS(priv->eth_dev);
        }
    }

  /* Check for a receive interrupt */

  if ((status & ETH_DMACH_INT_RI) != 0)
    {
      /* Acknowledge the normal receive interrupt */

      putreg32(ETH_DMACH_INT_RI | ETH_DMACH_INT_NI, regaddr);
      status &= ~(ETH_DMACH_INT_RI | ETH_DMACH_INT_NI);

      /* Update statistics */

      NETDEV_RXPACKETS(priv->eth_dev);

      /* Handle the incoming packet */

      lpc54_eth_receive(priv, channel);
    }

  /* Check for a transmit interrupt */

  if ((status & ETH_DMACH_INT_TI) != 0)
    {
      /* Acknowledge the normal receive interrupt */

      putreg32(ETH_DMACH_INT_TI | ETH_DMACH_INT_NI, regaddr);
      status &= ~(ETH_DMACH_INT_TI | ETH_DMACH_INT_NI);

      /* Handle the Tx completion event.  Reclaim the completed Tx
       * descriptors, free packet buffers, and check if we can start a new
       * transmissin.
       */

      lpc54_eth_txdone(priv, channel);
    }

  /* Check for unhandled interrupts */

  if (status != 0)
    {
      nwarn("WARNING: Unhandled interrupts: %08lx\n",
            (unsigned int)status);
      putreg32(status, regaddr);
    }
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

static void lpc54_eth_interrupt_work(void *arg)
{
  struct lpc54_ethdriver_s *priv = (struct lpc54_ethdriver_s *)arg;
  uint32_t intrstat;
  uint32_t chstat;

  /* Lock the network to serialize driver operations. */

  net_lock();

  /* Check if interrupt is from DMA channel 0. */

  intrstat = getreg32(LPC54_ETH_DMA_INTR_STAT);
  if ((intrstat & ETH_DMA_INTR_STAT_DC0IS) != 0)
    {
      lpc54_eth_channel_work(priv, 0);
    }

  /* Check if interrupt is from DMA channel 1. */

  intrstat = getreg32(LPC54_ETH_DMA_INTR_STAT);
  if ((intrstat & ETH_DMA_INTR_STAT_DC1IS) != 0)
    {
      lpc54_eth_channel_work(priv, 1);
    }

  /* Un-lock the network and re-enable Ethernet interrupts */

  net_unlock();
  up_enable_irq(LPC54_IRQ_ETHERNET);
}

/****************************************************************************
 * Name: lpc54_eth_interrupt
 *
 * Description:
 *   Ethernet interrupt handler
 *
 * Parameters:
 *   irq     - Number of the IRQ that generated the interrupt
 *   context - Interrupt register state save info (architecture-specific)
 *
 * Returned Value:
 *   OK on success
 *
 ****************************************************************************/

static int lpc54_eth_interrupt(int irq, void *context, void *arg)
{
  struct lpc54_ethdriver_s *priv = (struct lpc54_ethdriver_s *)arg;

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
 * Name: lpc54_pmt_interrupt
 *
 * Description:
 *   Ethernet power management interrupt handler
 *
 * Parameters:
 *   irq     - Number of the IRQ that generated the interrupt
 *   context - Interrupt register state save info (architecture-specific)
 *
 * Returned Value:
 *   OK on success
 *
 ****************************************************************************/

#if 0 /* Not used */
static int  lpc54_pmt_interrupt(int irq, void *context, void *arg)
{
  return OK;
}
#endif

/****************************************************************************
 * Name: lpc54_mac_interrupt
 *
 * Description:
 *   Ethernet MAC interrupt handler
 *
 * Parameters:
 *   irq     - Number of the IRQ that generated the interrupt
 *   context - Interrupt register state save info (architecture-specific)
 *
 * Returned Value:
 *   OK on success
 *
 ****************************************************************************/

#if 0 /* Not used */
static int lpc54_mac_interrupt(int irq, void *context, void *arg)
{
  return OK;
}
#endif

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

static void lpc54_eth_txtimeout_work(void *arg)
{
  struct lpc54_ethdriver_s *priv = (struct lpc54_ethdriver_s *)arg;

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
  struct lpc54_ethdriver_s *priv = (struct lpc54_ethdriver_s *)arg;

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

static inline void lpc54_eth_poll_process(struct lpc54_ethdriver_s *priv)
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

static void lpc54_eth_poll_work(void *arg)
{
  struct lpc54_ethdriver_s *priv = (struct lpc54_ethdriver_s *)arg;

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
  struct lpc54_ethdriver_s *priv = (struct lpc54_ethdriver_s *)arg;

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

static int lpc54_eth_ifup(struct net_driver_s *dev)
{
  struct lpc54_ethdriver_s *priv = (struct lpc54_ethdriver_s *)dev->d_private;
  uint8_t *mptr;
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

  /* Initialize the PHY *****************************************************/

  ret = lpc54_phy_autonegotiate(priv);
  if (ret < 0)
    {
      nerr("ERROR: lpc54_phy_autonegotiate failed: %d\n", ret);
      return ret;
    }

  /* Initialize Ethernet DMA ************************************************/
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

#ifdef CONFIG_LPC54_ETH_MULTIQUEUE
      /* REVISIT: burstlen setting for the case of multi-queuing. */
#  warning Missing logic
#else
      /* REVISIT: Additional logic needed if burstlen > 32 */

      burstlen = 1;  /* DMA burst length = 1 */
#endif

      /* REVISIT: We would need to set ETH_DMACH_CTRL_PBLx8 in LPC54_ETH_DMACH_CTRL
       * is required for the burst length setting.
       */

      putreg32(0, base + LPC54_ETH_DMACH_CTRL_OFFSET);

      regval  = getreg(base + LPC54_ETH_DMACH_TX_CTRL_OFFSET);
      regval &= ~ETH_DMACH_TX_CTRL_TxPBL_MASK;
      regval |= ETH_DMACH_TX_CTRL_TxPBL(burstlen);
      putreg32(regval, base + LPC54_ETH_DMACH_TX_CTRL_OFFSET);


      regval  = getreg(base + LPC54_ETH_DMACH_RX_CTRL_OFFSET);
      regval &= ~ETH_DMACH_RX_CTRL_RxPBL_MASK;
      regval |= ETH_DMACH_RX_CTRL_RxPBL(burstlen);
      putreg32(regval, base + LPC54_ETH_DMACH_RX_CTRL_OFFSET);
    }

  /* Initializes the Ethernet MTL *******************************************/
  /* Set transmit operation mode
   *
   * FTQ   - Set to flush the queue
   * TSF   - Depends on configuration
   * TXQEN - Queue 0 disabled; queue 1 enabled
   * TTC   - Set to 32 bytes (ignored if TSF set)
   * TQS   - Set to 2048 bytes
   */

#ifdef CONFIG_LPC54_ETH_TX_STRFWD
  regval = ETH_MTL_TXQ_OP_MODE_TSF;
#else
  regval = 0;
#endif

  regval |= ETH_MTL_TXQ_OP_MODE_FTQ | ETH_MTL_TXQ_OP_MODE_TTC_32 |
            ETH_MTL_TXQ_OP_MODE_TQS(LPC54_MTL_TXQUEUE_UNITS);
  putreg (regval | ETH_MTL_TXQ_OP_MODE_TXQEN_DISABLE,
          LPC54_ETH_MTL_TXQ_OP_MODE(0));
  putreg (regval | ETH_MTL_TXQ_OP_MODE_TXQEN_ENABLE,
          LPC54_ETH_MTL_TXQ_OP_MODE(1));

  /* Set receive operation mode (queue 0 only)
   *
   * RTC        - Set to 64 bytes (ignored if RSF selected)
   * FUP        - enabled
   * FEP        - disabled
   * RSF        - Depends on configuration
   * DIS_TCP_EF - Not disabled
   * RQS        - Set to 2048 bytes
   */

#ifdef CONFIG_LPC54_ETH_RX_STRFWD
  regval = ETH_MTL_RXQ_OP_MODE_RSF;
#else
  regval = 0;
#endif

  regval |= ETH_MTL_RXQ_OP_MODE_RTC_64 | ETH_MTL_RXQ_OP_MODE_FUP |
            ETH_MTL_RXQ_OP_MODE_RQS(LPC54_MTL_RXQUEUE_UNITS);
  putreg (regval, LPC54_ETH_MTL_RXQ_OP_MODE(0));

#ifdef CONFIG_LPC54_ETH_MULTIQUEUE
  /* Set the schedule/arbitration(set for multiple queues) */
      /* Set the rx queue mapping to dma channel */
      /* Set the tx/rx queue weight. */
  /* REVISIT:  Missing multi-queue configuration here. */
#  warning Missing Logic
#endif

  /* Initialize the Ethernet MAC ********************************************/
  /* Instantiate the MAC address that appliation logic should have set in the
   * device structure:
   */

  mptr   = (uint8_t *)priv->eth_dev.d_mac.ether.ether_addr_octet;
  regval = ((uint32_t)mptr[3] << 24) | ((uint32_t)mptr[2] << 16) |
           ((uint32_t)mptr[1] << 8)  | ((uint32_t)mptr[0]);
  putreg32(regval, LPC54_ETH_MAC_ADDR_LOW);

  regval = ((uint32_t)mptr[5] << 8)  | ((uint32_t)mptr[4]);
  putreg32(regval, LPC54_ETH_MAC_ADDR_LOW);

  /* Set the receive address filter */

  regval  = ETH_MAC_FRAME_FILTER_PCF_NONE;
#ifdef CONFIG_LPC54_ETH_RX_PROMISCUOUS
  regval |= ETH_MAC_FRAME_FILTER_PR;
#endif
#ifndef CONFIG_LPC54_ETH_RX_BROADCAST
  regval |= ETH_MAC_FRAME_FILTER_DBF;
#endif
#ifdef CONFIG_LPC54_ETH_RX_ALLMULTICAST
  regval |= ETH_MAC_FRAME_FILTER_PM;
#endif
  putreg32(regval, LPC54_ETH_MAC_FRAME_FILTER).

#ifdef CONFIG_LPC54_ETH_FLOWCONTROL
  /* Configure flow control */

  regval = ETH_MAC_RX_FLOW_CTRL_RFE | ETH_MAC_RX_FLOW_CTRL_UP;
  putreg32(regval, LPC54_ETH_MAC_RX_FLOW_CTRL);

  regval = ETH_MAC_TX_FLOW_CTRL_Q_PT(CONFIG_LPC54_ETH_TX_PAUSETIME);
  putreg32(regval, LPC54_ETH_MAC_TX_FLOW_CTRL_Q0);
  putreg32(regval, LPC54_ETH_MAC_TX_FLOW_CTRL_Q1);
#endif

  /* Set the 1uS tick counter*/

  regval = ETH_MAC_1US_TIC_COUNTR(BOARD_MAIN_CLK / USEC_PER_SEC);
  putreg32(regval, LPC54_ETH_MAC_1US_TIC_COUNTR);

  /* Set the speed and duplex using the values previously determined through
   * autonegotiaion.
   */

  regval = ETH_MAC_CONFIG_ECRSFD | ETH_MAC_CONFIG_PS;

#ifdef CONFIG_LPC54_ETH_8023AS2K
  regval |= ENET_MAC_CONFIG_S2KP;
#endif

  if (priv->eth_fullduplex)
    {
      regval |= ETH_MAC_CONFIG_DM;
    }
  else
    {
      regval |= LPC54_MAC_HALFDUPLEX_IPG;
    }

  if (priv->eth_100mbps)
    {
      regval |= ETH_MAC_CONFIG_FES;
    }

  putreg32(regval, LPC54_ETH_MAC_CONFIG);

  /* Set the SYSCON sideband flow control for each channel (see UserManual) */
#warning Missing logic

  /* Enable Rx queues  */

  regval = ETH_MAC_RXQ_CTRL0_RXQ0EN_ENABLE | ETH_MAC_RXQ_CTRL0_RXQ1EN_ENABLE;
  putreg32(regval, LPC54_ETH_MAC_RXQ_CTRL0);

  /* Setup up Ethernet interrupts */

  regval = LPC54_NORM_INTMASK | LPC54_ABNORM_INTMASK;
  putreg32(regval, LPC54_ETH_DMACH_INT_EN(0));
  putreg32(regval, LPC54_ETH_DMACH_INT_EN(1));

  putreg32(0, LPC54_ETH_MAC_INTR_EN);

#ifdef CONFIG_NET_ICMPv6
  /* Set up IPv6 multicast address filtering */

  lpc54_eth_ipv6multicast(priv);
#endif

  /* Initialize packet buffers */

  lpc54_pktbuf_initialize(priv);

  /* Initialize descriptors */

  lpc54_ring_initialize(priv);

  /* Activate DMA on channel 0 */

  regval  = getreg32(LPC54_ETH_DMACH_RX_CTRL(0));
  regval |= ETH_DMACH_RX_CTRL_SR;
  putreg32(regval, LPC54_ETH_DMACH_RX_CTRL(0));

  regval  = getreg32(LPC54_ETH_DMACH_TX_CTRL(0));
  regval |= ETH_DMACH_TX_CTRL_ST;
  putreg32(regval, LPC54_ETH_DMACH_TX_CTRL(0));

  /* Then enable the Rx/Tx */

  regval  = getreg32(LPC54_ETH_MAC_CONFIG);
  regval |= ETH_MAC_CONFIG_RE;
  putreg32(regval, LPC54_ETH_MAC_CONFIG);

  regval |= ETH_MAC_CONFIG_TE;
  putreg32(regval, LPC54_ETH_MAC_CONFIG);

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

static int lpc54_eth_ifdown(struct net_driver_s *dev)
{
  struct lpc54_ethdriver_s *priv = (struct lpc54_ethdriver_s *)dev->d_private;
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
#ifdef CONFIG_LPC54_ETH_MII
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

static void lpc54_eth_txavail_work(void *arg)
{
  struct lpc54_ethdriver_s *priv = (struct lpc54_ethdriver_s *)arg;

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

static int lpc54_eth_txavail(struct net_driver_s *dev)
{
  struct lpc54_ethdriver_s *priv = (struct lpc54_ethdriver_s *)dev->d_private;

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
static int lpc54_eth_addmac(struct net_driver_s *dev, const uint8_t *mac)
{
  struct lpc54_ethdriver_s *priv = (struct lpc54_ethdriver_s *)dev->d_private;

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
static int lpc54_eth_rmmac(struct net_driver_s *dev, const uint8_t *mac)
{
  struct lpc54_ethdriver_s *priv = (struct lpc54_ethdriver_s *)dev->d_private;

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
static void lpc54_eth_ipv6multicast(struct lpc54_ethdriver_s *priv)
{
  struct net_driver_s *dev;
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
static int lpc54_eth_ioctl(struct net_driver_s *dev, int cmd,
                           unsigned long arg)
{
  struct lpc54_ethdriver_s *priv = (struct lpc54_ethdriver_s *)dev->d_private;
  int ret;

  /* Decode and dispatch the driver-specific IOCTL command */

  switch (cmd)
    {
     case SIOCGMIIPHY: /* Get MII PHY address */
        {
          struct mii_ioctl_data_s *req = (struct mii_ioctl_data_s *)((uintptr_t)arg);
          req->phy_id = CONFIG_LPC54_ETH_PHYADDR;
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
 * Name: lpc54_pktbuf_initialize
 *
 * Description:
 *   Initialize packet buffers my placing all of the pre-allocated packet
 *   buffers into a free list.
 *
 * Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lpc54_pktbuf_initialize(struct lpc54_ethdriver_s *priv)
{
  uint32_t *pktbuf;
  int i;

  for (i = 0, pktbuf = g_prealloc_buffers;
       i < LPC54_NBUFFERS;
       i++, pktbuf += LPC54_BUFFER_WORDS)
    {
      sq_addlast((sq_entry_t *)pktbuf, priv->&eth_freebuf);
    }
}

/****************************************************************************
 * Name: lpc54_pktbuf_alloc
 *
 * Description:
 *   Allocate one packet buffer by removing it from the free list.
 *
 * Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   A pointer to the allocated packet buffer on succes; NULL is returned if
 *   there are no available packet buffers.
 *
 ****************************************************************************/

static inline uint32_t *lpc54_pktbuf_alloc(struct lpc54_ethdriver_s *priv)
{
  return (uint32_t *)sq_remfirst(priv->&eth_freebuf);
}

/****************************************************************************
 * Name: lpc54_pktbuf_free
 *
 * Description:
 *   Allocate one packet buffer by removing it from the free list.
 *
 * Parameters:
 *   priv   - Reference to the driver state structure
 *   pktbuf - The packet buffer to be freed
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void lpc54_pktbuf_free(struct lpc54_ethdriver_s *priv,
                                     uint32_t *pktbuf)
{
  sq_addlast((sq_entry_t *)pktbuf, priv->&eth_freebuf);
}

/****************************************************************************
 * Name: lpc54_txring_initialize
 *
 * Description:
 *   Initialize one Tx descriptor ring.
 *
 * Parameters:
 *   priv   - Reference to the driver state structure
 *   chan   - Channel being initialized
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lpc54_txring_initialize(struct lpc54_ethdriver_s *priv,
                                    unsigned int chan);
{
  struct lpc54_txring_s *txring;
  struct enet_txdesc_s *txdesc;
  uint32_t control;
  uint32_t regval;
  int i;

  txring  = &priv->eth_txring[chan];
  txdesc  = txring->tr_desc;

  /* Set the word-aligned Tx descriptor start/tail pointers. */

  regval  = (uint32_t)txdesc;
  putreg32(regval, LPC54_ETH_DMACH_TXDESC_LIST_ADDR(ch));

  regval += txring->tr_ndesc * sizeof(struct enet_txdesc_s);
  putreg32(regval, LPC54_ETH_DMACH_TXDESC_TAIL_PTR(ch));

  /* Set the Tx ring length */

  regval = ETH_DMACH_TXDESC_RING_LENGTH(txring->tr_ndesc);
  putreg32(regval, LPC54_ETH_DMACH_TXDESC_RING_LENGTH(ch));

  /* Inituialize the Tx desriptors . */

  for (i = 0; i < txring->tr_ndesc; i++, txdesc++)
    {
      txdesc->buffer1  = 0;
      txdesc->buffer2  = 0;
      txdesc->buflen   = ETH_TXDES2_IOC;
      txdesc->ctrlstat = 0;
    }
}

/****************************************************************************
 * Name: lpc54_rxring_initialize
 *
 * Description:
 *   Initialize one Rx descriptor ring.
 *
 * Parameters:
 *   priv   - Reference to the driver state structure
 *   chan   - Channel being initialized
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lpc54_rxring_initialize(struct lpc54_ethdriver_s *priv,
                                    unsigned int chan)
{
  struct lpc54_rxring_s *rxring;
  struct enet_txdesc_s *txdesc;
  uint32_t regval;
  int i;
  int j;

  rxring  = &priv->eth_rxring[chan];
  rxdesc  = rxring->rr_desc;

  /* Set the word-aligned Rx descriptor start/tail pointers. */

  regval  = (uint32_t)rxdesc;
  putreg32(regval, LPC54_ETH_DMACH_RXDESC_LIST_ADDR(chan));

  regval += rxring->rr_ndesc * sizeof(struct enet_rxdesc_s);
  putreg32(regval, LPC54_ETH_DMACH_RXDESC_TAIL_PTR(chan));

  /* Set the Rx ring length */

  regval = ETH_DMACH_RXDESC_RING_LENGTH(rxring->rr_ndesc);
  putreg32(regval, LPC54_ETH_DMACH_RXDESC_RING_LENGTH(ch));

  /* Set the receive buffer size (in words) in the Rx control register */

  regval  = getreg32(LPC54_ETH_DMACH_RX_CTRL(chan));
  regval &= ~ETH_DMACH_RX_CTRL_RBSZ_MASK;
  regval |= ETH_DMACH_RX_CTRL_RBSZ(LPC54_BUFFER_SIZE >> 2);
  putreg32(regval, LPC54_ETH_DMACH_RX_CTRL(chan));

  /* Initialize the Rx descriptor ring. */

  regval = ETH_RXDES3_BUF1V | ETH_RXDES3_IOC | ETH_RXDES3_OWN;
#ifdef CONFIG_LPC54_ETH_RX_DOUBLEBUFFER
  regval |= ETH_RXDES3_BUF2V;
#endif

  for (i = 0; i < rxring->rr_ndesc; i++, rxdesc++)
    {
      /* Allocate the first Rx packet buffer */

      rxdesc->buffer1 = (uint32_t)lpc54_pktbuf_alloc(priv);
      DEBUGASSERT(rxdesc->buffer1 != NULL);
      priv->eth_rxbuffers[0][i] = rxdesc->buffer1;

#ifdef CONFIG_LPC54_ETH_RX_DOUBLEBUFFER
      /* Allocate the second Rx packet buffer */

      rxdesc->buffer2 = (uint32_t)lpc54_pktbuf_alloc(priv);
      DEBUGASSERT(rxdesc->buffer2 != NULL);
      priv->eth_rxbuffers[1][i] = rxdesc->buffer2;

#else
      /* The second buffer is not used */

      rxdesc->buffer2 = 0;
#endif

      /* Set the valid and DMA own flag.*/

      rxdesc->ctrl = regval;
    }
}

/****************************************************************************
 * Name: lpc54_ring_initialize
 *
 * Description:
 *   Initialize the Rx and Tx rings for every channel.
 *
 * Parameters:
 *   priv   - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lpc54_ring_initialize(struct lpc54_ethdriver_s *priv)
{
  /* Initialize ring descriptions */

  memset(priv->eth_txring, 0, LPC54_NRINGS * sizeof(struct lpc54_txring_s));
  memset(priv->eth_rxring, 0, LPC54_NRINGS * sizeof(struct lpc54_rxring_s));

  /* Initialize channel 0 rings */

  memset(g_txbuffers0, 0, CONFIG_LPC54_ETH_NTXDESC0 * sizeof(uint32_t *));
  memset(g_rxbuffers0, 0, CONFIG_LPC54_ETH_NRXDESC0 * sizeof(uint32_t *));

  priv->eth_txring[0].tr_desc    = g_ch0_txdesc;
  priv->eth_txring[0].tr_ndesc   = CONFIG_LPC54_ETH_NTXDESC0;
  priv->eth_txring[0].tr_buffers = g_txbuffers0;
  lpc54_txring_initialize(priv, 0);

  priv->eth_rxring[0].rr_desc    = g_ch0_rxdesc;
  priv->eth_rxring[0].rr_ndesc   = CONFIG_LPC54_ETH_NRXDESC0;
  priv->eth_rxring[0].rr_buffers = g_rxbuffers0;
  lpc54_rxring_initialize(priv, 0);

#ifdef CONFIG_LPC54_ETH_MULTIQUEUE
  /* Initialize channel 1 rings */

  memset(g_txbuffers1, 0, CONFIG_LPC54_ETH_NTXDESC1 * sizeof(uint32_t *));
  memset(g_rxbuffers1, 0, CONFIG_LPC54_ETH_NRXDESC1 * sizeof(uint32_t *));

  priv->eth_txring[1].tr_desc    = g_ch1_txdesc;
  priv->eth_txring[1].tr_ndesc   = CONFIG_LPC54_ETH_NTXDESC1;
  priv->eth_txring[0].tr_buffers = g_txbuffers1;
  lpc54_txring_initialize(priv, 1);

  priv->eth_rxring[0].rr_desc    = g_ch1_rxdesc;
  priv->eth_rxring[0].rr_ndesc   = CONFIG_LPC54_ETH_NRXDESC1;
  priv->eth_rxring[0].rr_buffers = g_rxbuffers1;
  lpc54_rxring_initialize(priv, 0);
#endif
}

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

static uint16_t lpc54_phy_read(struct lpc54_ethdriver_s *priv,
                               uint8_t phyreg)
{
  uint32_t regval = base->MAC_MDIO_ADDR & ENET_MAC_MDIO_ADDR_CR_MASK;

  /* Set the MII read command. */

  regval  = getreg32(LPC54_ETH_MAC_MDIO_ADDR);
  regval &= ETH_MAC_MDIO_ADDR_CR_MASK;
  regval |= ETH_MAC_MDIO_ADDR_MOC_READ | ETH_MAC_MDIO_ADDR_RDA(phyreg) |
            ETH_MAC_MDIO_ADDR_PA(CONFIG_LPC54_ETH_PHYADDR);
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

static void lpc54_phy_write(struct lpc54_ethdriver_s *priv,
                            uint8_t phyreg, uint16_t phyval)
{
  uint32_t regval = base->MAC_MDIO_ADDR & ENET_MAC_MDIO_ADDR_CR_MASK;

  /* Set the MII write command. */

  regval  = getreg32(LPC54_ETH_MAC_MDIO_ADDR);
  regval &= ETH_MAC_MDIO_ADDR_CR_MASK;
  regval |= ETH_MAC_MDIO_ADDR_MOC_WRITE | ETH_MAC_MDIO_ADDR_RDA(phyreg) |
            ETH_MAC_MDIO_ADDR_PA(CONFIG_LPC54_ETH_PHYADDR);
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

static int lpc54_phy_autonegotiate(struct lpc54_ethdriver_s *priv)
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

static int lpc54_phy_reset(struct lpc54_ethdriver_s *priv)
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

      phyval = lpc54_phy_read(base, CONFIG_LPC54_ETH_PHYADDR, MII_MCR);
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
  struct lpc54_ethdriver_s *priv;

  /* Get the interface structure associated with this interface number. */

  DEBUGASSERT(intf == 0);
  priv = &g_ethdriver;

  /* Attach the three Ethernet-related IRQs to the handlers */

  if (irq_attach(LPC54_IRQ_ETHERNET, lpc54_eth_interrupt, priv))
    {
      /* We could not attach the ISR to the interrupt */

      nerr("ERROR:  irq_attach failed\n");
      return -EAGAIN;
    }

#if 0 /* Not used */
  if (irq_attach(LPC54_IRQ_ETHERNETPMT, lpc54_pmt_interrupt, priv))
    {
      /* We could not attach the ISR to the interrupt */

      nerr("ERROR:  irq_attach for PMTfailed\n");
      return -EAGAIN;
    }

  if (irq_attach(LPC54_IRQ_ETHERNETMACLP, lpc54_mac_interrupt, priv))
    {
      /* We could not attach the ISR to the interrupt */

      nerr("ERROR:  irq_attach for MAC failed\n");
      return -EAGAIN;
    }
#endif

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
  priv->eth_dev.d_private = (void *)g_ethdriver; /* Used to recover private state from dev */

  /* Create a watchdog for timing polling for and timing of transmisstions */

  priv->eth_txpoll        = wd_create();        /* Create periodic poll timer */
  priv->eth_txtimeout     = wd_create();        /* Create TX timeout timer */

  DEBUGASSERT(priv->eth_txpoll != NULL && priv->eth_txtimeout != NULL);

  /* Configure GPIO pins to support Ethernet */
  /* Common MIIM interface */

  lpc54_gpio_config(GPIO_ENET_MDIO);    /* Ethernet MIIM data input and output */
  lpc54_gpio_config(GPIO_ENET_MDC);     /* Ethernet MIIM clock */

#ifdef CONFIG_LPC54_ETH_MII
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

  lpc54_set_crsdiv();

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
