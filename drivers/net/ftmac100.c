/****************************************************************************
 * drivers/net/ftmac100.c
 * Faraday FTMAC100 Ethernet MAC Driver
 *
 *   Copyright (C) 2015 Anton D. Kachalov. All rights reserved.
 *   Author: Anton D. Kachalov <mouse@yandex.ru>
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_FTMAC100)

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>
#include <crc32.h>

#include <arpa/inet.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>
#include <nuttx/net/arp.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/ftmac100.h>

#ifdef CONFIG_NET_PKT
#  include <nuttx/net/pkt.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* If processing is not done at the interrupt level, then work queue support
 * is required.
 */

#if !defined(CONFIG_SCHED_WORKQUEUE)
#  error Work queue support is required in this configuration (CONFIG_SCHED_WORKQUEUE)
#else

  /* Use the low priority work queue if possible */

#  if defined(CONFIG_FTMAC100_HPWORK)
#    define FTMAWORK HPWORK
#  elif defined(CONFIG_FTMAC100_LPWORK)
#    define FTMAWORK LPWORK
#  else
#    error Neither CONFIG_FTMAC100_HPWORK nor CONFIG_FTMAC100_LPWORK defined
#  endif
#endif

/* CONFIG_FTMAC100_NINTERFACES determines the number of physical interfaces
 * that will be supported.
 */

#ifndef CONFIG_FTMAC100_NINTERFACES
# define CONFIG_FTMAC100_NINTERFACES 1
#endif

/* TX poll delay = 1 seconds. CLK_TCK is the number of clock ticks per second */

#define FTMAC100_WDDELAY   (1*CLK_TCK)

/* TX timeout = 1 minute */

#define FTMAC100_TXTIMEOUT (60*CLK_TCK)

/* This is a helper pointer for accessing the contents of the Ethernet header */

#define BUF ((struct eth_hdr_s *)priv->ft_dev.d_buf)

/* RX/TX buffer alignment */

#define MAX_PKT_SIZE  1536
#define RX_BUF_SIZE   2044

#define ETH_ZLEN 60

#if defined(CONFIG_NET_IGMP) || defined(CONFIG_NET_ICMPv6)
# define MACCR_ENABLE_ALL (FTMAC100_MACCR_XMT_EN  | \
                           FTMAC100_MACCR_RCV_EN  | \
                           FTMAC100_MACCR_XDMA_EN | \
                           FTMAC100_MACCR_RDMA_EN | \
                           FTMAC100_MACCR_CRC_APD | \
                           FTMAC100_MACCR_FULLDUP | \
                           FTMAC100_MACCR_RX_RUNT | \
                           FTMAC100_MACCR_HT_MULTI_EN | \
                           FTMAC100_MACCR_RX_BROADPKT)
#else
# define MACCR_ENABLE_ALL (FTMAC100_MACCR_XMT_EN  | \
                           FTMAC100_MACCR_RCV_EN  | \
                           FTMAC100_MACCR_XDMA_EN | \
                           FTMAC100_MACCR_RDMA_EN | \
                           FTMAC100_MACCR_CRC_APD | \
                           FTMAC100_MACCR_FULLDUP | \
                           FTMAC100_MACCR_RX_RUNT | \
                           FTMAC100_MACCR_RX_BROADPKT)
#endif

#define MACCR_DISABLE_ALL 0

#define INT_MASK_ALL_ENABLED (FTMAC100_INT_RPKT_FINISH | \
                          FTMAC100_INT_NORXBUF | \
                          FTMAC100_INT_XPKT_OK | \
                          FTMAC100_INT_XPKT_LOST | \
                          FTMAC100_INT_RPKT_LOST | \
                          FTMAC100_INT_AHB_ERR | \
                          FTMAC100_INT_PHYSTS_CHG)

#define INT_MASK_ALL_DISABLED 0

#define putreg32(v, x) (*(volatile uint32_t*)(x) = (v))
#define getreg32(x) (*(uint32_t *)(x))

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The ftmac100_driver_s encapsulates all state information for a single hardware
 * interface
 */

struct ftmac100_driver_s
{
  struct ftmac100_txdes_s txdes[CONFIG_FTMAC100_TX_DESC];
  struct ftmac100_rxdes_s rxdes[CONFIG_FTMAC100_RX_DESC];
  int rx_pointer;
  int tx_pointer;
  int tx_clean_pointer;
  int tx_pending;
  uint32_t iobase;

  /* NuttX net data */

  bool ft_bifup;               /* true:ifup false:ifdown */
  WDOG_ID ft_txpoll;           /* TX poll timer */
  WDOG_ID ft_txtimeout;        /* TX timeout timer */
  unsigned int status;         /* Last ISR status */
  struct work_s ft_irqwork;    /* For deferring work to the work queue */
  struct work_s ft_pollwork;   /* For deferring work to the work queue */

  /* This holds the information visible to the NuttX network */

  struct net_driver_s ft_dev;  /* Interface understood by the network */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* A single packet buffer is used */

static uint8_t g_pktbuf[MAX_NETDEV_PKTSIZE + CONFIG_NET_GUARDSIZE];

/* Driver state structure. */

static struct ftmac100_driver_s g_ftmac100[CONFIG_FTMAC100_NINTERFACES]
  __attribute__((aligned(16)));

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Common TX logic */

static int  ftmac100_transmit(FAR struct ftmac100_driver_s *priv);
static int  ftmac100_txpoll(struct net_driver_s *dev);

/* Interrupt handling */

static void ftmac100_reset(FAR struct ftmac100_driver_s *priv);
static void ftmac100_receive(FAR struct ftmac100_driver_s *priv);
static void ftmac100_txdone(FAR struct ftmac100_driver_s *priv);

static void ftmac100_interrupt_work(FAR void *arg);
static int  ftmac100_interrupt(int irq, FAR void *context, FAR void *arg);

/* Watchdog timer expirations */

static void ftmac100_txtimeout_work(FAR void *arg);
static void ftmac100_txtimeout_expiry(int argc, uint32_t arg, ...);

static void ftmac100_poll_work(FAR void *arg);
static void ftmac100_poll_expiry(int argc, uint32_t arg, ...);

/* NuttX callback functions */

static int ftmac100_ifup(FAR struct net_driver_s *dev);
static int ftmac100_ifdown(FAR struct net_driver_s *dev);

static void ftmac100_txavail_work(FAR void *arg);
static int ftmac100_txavail(FAR struct net_driver_s *dev);

#if defined(CONFIG_NET_IGMP) || defined(CONFIG_NET_ICMPv6)
static int ftmac100_addmac(FAR struct net_driver_s *dev, FAR const uint8_t *mac);
#ifdef CONFIG_NET_IGMP
static int ftmac100_rmmac(FAR struct net_driver_s *dev, FAR const uint8_t *mac);
#endif
#ifdef CONFIG_NET_ICMPv6
static void ftmac100_ipv6multicast(FAR struct ftmac100_driver_s *priv);
#endif
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ftmac100_transmit
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
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/

static FAR struct ftmac100_rxdes_s *
ftmac100_current_rxdes(FAR struct ftmac100_driver_s *priv)
{
  return &priv->rxdes[priv->rx_pointer];
}

static FAR struct ftmac100_txdes_s *
ftmac100_current_txdes(FAR struct ftmac100_driver_s *priv)
{
  return &priv->txdes[priv->tx_pointer];
}

static FAR struct ftmac100_txdes_s *
ftmac100_current_clean_txdes(FAR struct ftmac100_driver_s *priv)
{
  return &priv->txdes[priv->tx_clean_pointer];
}

static int ftmac100_transmit(FAR struct ftmac100_driver_s *priv)
{
  FAR struct ftmac100_register_s *iobase = (FAR struct ftmac100_register_s *)priv->iobase;
  FAR struct ftmac100_txdes_s *txdes;
  int len = priv->ft_dev.d_len;
//irqstate_t flags;
//flags = enter_critical_section();
//ninfo("flags=%08x\n", flags);

  txdes = ftmac100_current_txdes(priv);

  /* Verify that the hardware is ready to send another packet.  If we get
   * here, then we are committed to sending a packet; Higher level logic
   * must have assured that there is no transmission in progress.
   */

  len = len < ETH_ZLEN ? ETH_ZLEN : len;

  /* Send the packet: address=priv->ft_dev.d_buf, length=priv->ft_dev.d_len */

//memcpy((void *)txdes->txdes2, priv->ft_dev.d_buf, len);
  txdes->txdes2  = (unsigned int)priv->ft_dev.d_buf;
  txdes->txdes1 &= FTMAC100_TXDES1_EDOTR;
  txdes->txdes1 |= (FTMAC100_TXDES1_FTS |
                    FTMAC100_TXDES1_LTS |
                    FTMAC100_TXDES1_TXIC |
                    FTMAC100_TXDES1_TXBUF_SIZE(len));
  txdes->txdes0 |= FTMAC100_TXDES0_TXDMA_OWN;

  ninfo("ftmac100_transmit[%x]: copy %08x to %08x %04x\n",
        priv->tx_pointer, priv->ft_dev.d_buf, txdes->txdes2, len);

  priv->tx_pointer = (priv->tx_pointer + 1) & (CONFIG_FTMAC100_TX_DESC - 1);
  priv->tx_pending++;

  /* Enable Tx polling */
  /* FIXME: enable interrupts */

  putreg32(1, &iobase->txpd);

  /* Setup the TX timeout watchdog (perhaps restarting the timer) */

  (void)wd_start(priv->ft_txtimeout, FTMAC100_TXTIMEOUT,
                 ftmac100_txtimeout_expiry, 1, (wdparm_t)priv);

//leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: ftmac100_txpoll
 *
 * Description:
 *   The transmitter is available, check if the network has any outgoing packets
 *   ready to send.  This is a callback from devif_poll().  devif_poll() may
 *   be called:
 *
 *   1. When the preceding TX packet send is complete,
 *   2. When the preceding TX packet send timesout and the interface is
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
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/

static int ftmac100_txpoll(struct net_driver_s *dev)
{
  FAR struct ftmac100_driver_s *priv = (FAR struct ftmac100_driver_s *)dev->d_private;

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  if (priv->ft_dev.d_len > 0)
    {
      /* Look up the destination MAC address and add it to the Ethernet
       * header.
       */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
      if (IFF_IS_IPv4(priv->ft_dev.d_flags))
#endif
        {
          arp_out(&priv->ft_dev);
        }
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
      else
#endif
        {
          neighbor_out(&priv->ft_dev);
        }
#endif /* CONFIG_NET_IPv6 */

      if (!devif_loopback(&priv->ft_dev))
        {
          /* Send the packet */

          ftmac100_transmit(priv);

          /* Check if there is room in the device to hold another packet. If not,
           * return a non-zero value to terminate the poll.
           */
        }
    }

  /* If zero is returned, the polling will continue until all connections have
   * been examined.
   */

  return 0;
}

/****************************************************************************
 * Name: ftmac100_reset
 *
 * Description:
 *   Do the HW reset
 *
 * Input Parameters:
 *   priv - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static void ftmac100_reset(FAR struct ftmac100_driver_s *priv)
{
  FAR struct ftmac100_register_s *iobase = (FAR struct ftmac100_register_s *)priv->iobase;

  ninfo("%s(): iobase=%p\n", __func__, iobase);

  putreg32 (FTMAC100_MACCR_SW_RST, &iobase->maccr);

  while (getreg32 (&iobase->maccr) & FTMAC100_MACCR_SW_RST)
    ;
}

/****************************************************************************
 * Name: ftmac100_init
 *
 * Description:
 *   Perform HW initialization
 *
 * Input Parameters:
 *   priv - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static void ftmac100_init(FAR struct ftmac100_driver_s *priv)
{
  FAR struct ftmac100_register_s *iobase = (FAR struct ftmac100_register_s *)priv->iobase;
  FAR struct ftmac100_txdes_s *txdes = priv->txdes;
  FAR struct ftmac100_rxdes_s *rxdes = priv->rxdes;
  FAR unsigned char *kmem;
  int i;

  nerr ("%s()\n", __func__);

  /* Disable all interrupts */

  putreg32(0, &iobase->imr);

  /* Initialize descriptors */

  priv->rx_pointer = 0;
  priv->tx_pointer = 0;
  priv->tx_clean_pointer = 0;
  priv->tx_pending = 0;

  rxdes[CONFIG_FTMAC100_RX_DESC - 1].rxdes1 = FTMAC100_RXDES1_EDORR;

  kmem = memalign(RX_BUF_SIZE, CONFIG_FTMAC100_RX_DESC * RX_BUF_SIZE);

  ninfo("KMEM=%08x\n", kmem);

  for (i = 0; i < CONFIG_FTMAC100_RX_DESC; i++)
    {
      /* RXBUF_BADR */

      rxdes[i].rxdes0 = FTMAC100_RXDES0_RXDMA_OWN;
      rxdes[i].rxdes1 |= FTMAC100_RXDES1_RXBUF_SIZE(RX_BUF_SIZE);
      rxdes[i].rxdes2 = (unsigned int)(kmem + i * RX_BUF_SIZE);
      rxdes[i].rxdes3 = (unsigned int)(rxdes + i + 1); /* Next ring */
    }

  rxdes[CONFIG_FTMAC100_RX_DESC - 1].rxdes3 = (unsigned int)rxdes; /* Next ring */

  for (i = 0; i < CONFIG_FTMAC100_TX_DESC; i++)
    {
      /* TXBUF_BADR */

      txdes[i].txdes0 = 0;
      txdes[i].txdes1 = 0;
      txdes[i].txdes2 = 0;
      txdes[i].txdes3 = 0;
//    txdes[i].txdes3 = (unsigned int)(txdes + i + 1); /* Next ring */
    }

  txdes[CONFIG_FTMAC100_TX_DESC - 1].txdes1 = FTMAC100_TXDES1_EDOTR;
//txdes[CONFIG_FTMAC100_TX_DESC - 1].txdes3 = (unsigned int)txdes; /* Next ring */

  /* transmit ring */

  ninfo("priv=%08x txdes=%08x rxdes=%08x\n", priv, txdes, rxdes);

  putreg32 ((unsigned int)txdes, &iobase->txr_badr);

  /* receive ring */

  putreg32 ((unsigned int)rxdes, &iobase->rxr_badr);

  /* set RXINT_THR and TXINT_THR */

//putreg32 (FTMAC100_ITC_RXINT_THR(1) | FTMAC100_ITC_TXINT_THR(1), &iobase->itc);

  /* poll receive descriptor automatically */

  putreg32 (FTMAC100_APTC_RXPOLL_CNT(1), &iobase->aptc);

#if 1
  /* Set DMA burst length */

  putreg32 (FTMAC100_DBLAC_RXFIFO_LTHR(2) |
            FTMAC100_DBLAC_RXFIFO_HTHR(6) |
            FTMAC100_DBLAC_RX_THR_EN, &iobase->dblac);

//putreg32 (getreg32(&iobase->fcr) | 0x1, &iobase->fcr);
//putreg32 (getreg32(&iobase->bpr) | 0x1, &iobase->bpr);
#endif

  /* enable transmitter, receiver */

  putreg32 (MACCR_ENABLE_ALL, &iobase->maccr);

  /* enable Rx, Tx interrupts */

  putreg32 (INT_MASK_ALL_ENABLED, &iobase->imr);
}

/****************************************************************************
 * Name: ftmac100_mdio_read
 *
 * Description:
 *   Read MII registers
 *
 * Input Parameters:
 *   iobase - Pointer to the driver's registers base
 *      reg - MII register number
 *
 * Returned Value:
 *   Register value
 *
 * Assumptions:
 *
 *
 ****************************************************************************/

static uint32_t ftmac100_mdio_read(FAR struct ftmac100_register_s *iobase, int reg)
{
  int i;
  uint32_t phycr = FTMAC100_PHYCR_PHYAD(1)   |
                   FTMAC100_PHYCR_REGAD(reg) |
                   FTMAC100_PHYCR_MIIRD;

  putreg32(phycr, &iobase->phycr);

  for (i = 0; i < 10; i++)
    {
      phycr = getreg32(&iobase->phycr);
      ninfo("%02x %d phycr=%08x\n", reg, i, phycr);

      if ((phycr & FTMAC100_PHYCR_MIIRD) == 0)
        {
          break;
        }
    }

  return phycr & 0xffff;
}

/****************************************************************************
 * Name: ftmac100_set_mac
 *
 * Description:
 *   Set the MAC address
 *
 * Input Parameters:
 *   priv - Reference to the NuttX driver state structure
 *    mac - Six bytes MAC address
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void ftmac100_set_mac(FAR struct ftmac100_driver_s *priv,
                             FAR const unsigned char *mac)
{
  FAR struct ftmac100_register_s *iobase = (FAR struct ftmac100_register_s *)priv->iobase;
  unsigned int maddr = mac[0] << 8 | mac[1];
  unsigned int laddr = mac[2] << 24 | mac[3] << 16 | mac[4] << 8 | mac[5];

  ninfo("%s(%x %x)\n", __func__, maddr, laddr);

  putreg32(maddr, &iobase->mac_madr);
  putreg32(laddr, &iobase->mac_ladr);
}

/****************************************************************************
 * Name: ftmac100_receive
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
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static void ftmac100_receive(FAR struct ftmac100_driver_s *priv)
{
  FAR struct ftmac100_rxdes_s *rxdes;
  FAR uint8_t *data;
  uint32_t len;
  int found;

  do
    {
      found = false;
      rxdes = ftmac100_current_rxdes(priv);

      while (!(rxdes->rxdes0 & FTMAC100_RXDES0_RXDMA_OWN))
        {
          if (rxdes->rxdes0 & FTMAC100_RXDES0_FRS)
            {
              found = true;
              break;
            }

          /* Clear status bits */

          rxdes->rxdes0 = FTMAC100_RXDES0_RXDMA_OWN;

          priv->rx_pointer = (priv->rx_pointer + 1) & (CONFIG_FTMAC100_RX_DESC - 1);
          rxdes = ftmac100_current_rxdes(priv);
        }

      if (!found)
        {
          ninfo("\nNOT FOUND\nCurrent RX %d rxdes0=%08x\n",
                priv->rx_pointer, rxdes->rxdes0);
          return;
        }

      len = FTMAC100_RXDES0_RFL(rxdes->rxdes0);
      data = (uint8_t *)rxdes->rxdes2;

      ninfo ("RX buffer %d (%08x), %x received (%d)\n",
             priv->rx_pointer, data, len, (rxdes->rxdes0 & FTMAC100_RXDES0_LRS));

      /* Copy the data data from the hardware to priv->ft_dev.d_buf.  Set
       * amount of data in priv->ft_dev.d_len
       */

      memcpy(priv->ft_dev.d_buf, data, len);
      priv->ft_dev.d_len = len;

#ifdef CONFIG_NET_PKT
      /* When packet sockets are enabled, feed the frame into the packet tap */

      pkt_input(&priv->ft_dev);
#endif

      /* We only accept IP packets of the configured type and ARP packets */

#ifdef CONFIG_NET_IPv4
      if (BUF->type == HTONS(ETHTYPE_IP))
        {
          ninfo("IPv4 frame\n");

          /* Handle ARP on input then give the IPv4 packet to the network
           * layer
           */

          arp_ipin(&priv->ft_dev);
          ipv4_input(&priv->ft_dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, the field  d_len will set to a value > 0.
           */

          if (priv->ft_dev.d_len > 0)
            {
              /* Update the Ethernet header with the correct MAC address */

#ifdef CONFIG_NET_IPv6
              if (IFF_IS_IPv4(priv->ft_dev.d_flags))
#endif
                {
                  arp_out(&priv->ft_dev);
                }
#ifdef CONFIG_NET_IPv6
              else
                {
                  neighbor_out(&priv->ft_dev);
                }
#endif

              /* And send the packet */

              ftmac100_transmit(priv);
            }
        }
      else
#endif
#ifdef CONFIG_NET_IPv6
      if (BUF->type == HTONS(ETHTYPE_IP6))
        {
          ninfo("Iv6 frame\n");

          /* Give the IPv6 packet to the network layer */

          ipv6_input(&priv->ft_dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, the field  d_len will set to a value > 0.
           */

          if (priv->ft_dev.d_len > 0)
           {
              /* Update the Ethernet header with the correct MAC address */

#ifdef CONFIG_NET_IPv4
              if (IFF_IS_IPv4(priv->ft_dev.d_flags))
                {
                  arp_out(&priv->ft_dev);
                }
              else
#endif
#ifdef CONFIG_NET_IPv6
                {
                  neighbor_out(&priv->ft_dev);
                }
#endif

              /* And send the packet */

              ftmac100_transmit(priv);
            }
        }
      else
#endif
#ifdef CONFIG_NET_ARP
      if (BUF->type == htons(ETHTYPE_ARP))
        {
          arp_arpin(&priv->ft_dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, the field  d_len will set to a value > 0.
           */

          if (priv->ft_dev.d_len > 0)
            {
              ftmac100_transmit(priv);
            }
        }
#endif
      priv->rx_pointer = (priv->rx_pointer + 1) & (CONFIG_FTMAC100_RX_DESC - 1);

      rxdes->rxdes1 &= FTMAC100_RXDES1_EDORR;
      rxdes->rxdes1 |= FTMAC100_RXDES1_RXBUF_SIZE(RX_BUF_SIZE);
      rxdes->rxdes0 |= FTMAC100_RXDES0_RXDMA_OWN;
    }
  while (true); /* While there are more packets to be processed */
}

/****************************************************************************
 * Name: ftmac100_txdone
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
 *   Global interrupts are disabled by the watchdog logic.
 *
 ****************************************************************************/

static void ftmac100_txdone(FAR struct ftmac100_driver_s *priv)
{
  FAR struct ftmac100_txdes_s *txdes;

  /* Check if a Tx was pending */

  while (priv->tx_pending)
    {
      txdes = ftmac100_current_clean_txdes(priv);

      /* txdes owned by dma */

      if (txdes->txdes0 & FTMAC100_TXDES0_TXDMA_OWN)
        {
          break;
        }

      /* TODO: check for excessive and late collisions */

      /* txdes reset */

      txdes->txdes0 = 0;
      txdes->txdes1 &= FTMAC100_TXDES1_EDOTR;
      txdes->txdes2 = 0;
      txdes->txdes3 = 0;

      priv->tx_clean_pointer = (priv->tx_clean_pointer + 1) & (CONFIG_FTMAC100_TX_DESC - 1);

      priv->tx_pending--;
    }

  /* If no further xmits are pending, then cancel the TX timeout and
   * disable further Tx interrupts.
   */

  ninfo("txpending=%d\n", priv->tx_pending);

  /* Cancel the TX timeout */

  wd_cancel(priv->ft_txtimeout);

  /* Then poll the network for new XMIT data */

  (void)devif_poll(&priv->ft_dev, ftmac100_txpoll);
}

/****************************************************************************
 * Name: ftmac100_interrupt_work
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
 *   Ethernet interrupts are disabled
 *
 ****************************************************************************/

static void ftmac100_interrupt_work(FAR void *arg)
{
  FAR struct ftmac100_driver_s *priv = (FAR struct ftmac100_driver_s *)arg;
  FAR struct ftmac100_register_s *iobase = (FAR struct ftmac100_register_s *)priv->iobase;
  unsigned int status;
  unsigned int phycr;

  /* Process pending Ethernet interrupts */

  net_lock();
  status = priv->status;

  ninfo("status=%08x(%08x) BASE=%p ISR=%p PHYCR=%p\n",
        status, getreg32(&iobase->isr), iobase, &iobase->isr, &iobase->phycr);

  if (!status)
    {
      goto out;
    }

  /* Handle interrupts according to status bit settings */

  /* Check if we received an incoming packet, if so, call ftmac100_receive() */

  if (status & FTMAC100_INT_RPKT_SAV)
    {
      putreg32(1, &iobase->rxpd);
    }

  if (status & (FTMAC100_INT_RPKT_FINISH | FTMAC100_INT_NORXBUF))
    {
      ftmac100_receive(priv);
    }

  /* Check if a packet transmission just completed.  If so, call ftmac100_txdone.
   * This may disable further Tx interrupts if there are no pending
   * transmissions.
   */

  if (status & (FTMAC100_INT_XPKT_OK))
    {
      ninfo("\n\nTXDONE\n\n");
      ftmac100_txdone(priv);
    }

  if (status & FTMAC100_INT_PHYSTS_CHG)
    {
      /* PHY link status change */

      phycr = ftmac100_mdio_read(iobase, 1);
      if (phycr & 0x04)
        {
          priv->ft_bifup = true;
        }
      else
        {
          priv->ft_bifup = false;
        }

      ninfo("Link: %s\n", priv->ft_bifup ? "UP" : "DOWN");
      ftmac100_mdio_read(iobase, 5);
    }

#if 0
#define REG(x) (*(volatile uint32_t *)(x))
  ninfo("\n=============================================================\n");
  ninfo("TM CNTL=%08x INTRS=%08x MASK=%08x LOAD=%08x COUNT=%08x M1=%08x\n",
        REG(0x98400030), REG(0x98400034), REG(0x98400038), REG(0x98400004),
        REG(0x98400000), REG(0x98400008));
  ninfo("IRQ STATUS=%08x MASK=%08x MODE=%08x LEVEL=%08x\n",
        REG(0x98800014), REG(0x98800004), REG(0x9880000C), REG(0x98800010));
  ninfo("FIQ STATUS=%08x MASK=%08x MODE=%08x LEVEL=%08x\n", REG(0x98800034),
        REG(0x98800024), REG(0x9880002C), REG(0x98800020));
  ninfo("=============================================================\n");
#endif

out:
  putreg32 (INT_MASK_ALL_ENABLED, &iobase->imr);

  ninfo("ISR-done\n");
  net_unlock();

  /* Re-enable Ethernet interrupts */

  up_enable_irq(CONFIG_FTMAC100_IRQ);
}

/****************************************************************************
 * Name: ftmac100_interrupt
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

static int ftmac100_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct ftmac100_driver_s *priv = &g_ftmac100[0];
  FAR struct ftmac100_register_s *iobase = (FAR struct ftmac100_register_s *)priv->iobase;

  /* Disable further Ethernet interrupts.  Because Ethernet interrupts are
   * also disabled if the TX timeout event occurs, there can be no race
   * condition here.
   */

  priv->status = getreg32(&iobase->isr);

  up_disable_irq(CONFIG_FTMAC100_IRQ);

  putreg32 (INT_MASK_ALL_DISABLED, &iobase->imr);

  /* TODO: Determine if a TX transfer just completed */

  ninfo("===> status=%08x\n", priv->status);

  if (priv->status & (FTMAC100_INT_XPKT_OK))
    {
      /* If a TX transfer just completed, then cancel the TX timeout so
       * there will be do race condition between any subsequent timeout
       * expiration and the deferred interrupt processing.
       */

      ninfo("\n\nTXDONE 0\n\n");
      wd_cancel(priv->ft_txtimeout);
    }

  /* Schedule to perform the interrupt processing on the worker thread. */

  work_queue(FTMAWORK, &priv->ft_irqwork, ftmac100_interrupt_work, priv, 0);

  return OK;
}

/****************************************************************************
 * Name: ftmac100_txtimeout_work
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
 * Assumptions:
 *   Ethernet interrupts are disabled
 *
 ****************************************************************************/

static void ftmac100_txtimeout_work(FAR void *arg)
{
  FAR struct ftmac100_driver_s *priv = (FAR struct ftmac100_driver_s *)arg;

  ninfo("TXTIMEOUT\n");

  /* Process pending Ethernet interrupts */

  net_lock();

  /* Then poll the network for new XMIT data */

  (void)devif_poll(&priv->ft_dev, ftmac100_txpoll);
  net_unlock();
}

/****************************************************************************
 * Name: ftmac100_txtimeout_expiry
 *
 * Description:
 *   Our TX watchdog timed out.  Called from the timer interrupt handler.
 *   The last TX never completed.  Reset the hardware and start again.
 *
 * Input Parameters:
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

static void ftmac100_txtimeout_expiry(int argc, uint32_t arg, ...)
{
  FAR struct ftmac100_driver_s *priv = (FAR struct ftmac100_driver_s *)arg;

  /* Disable further Ethernet interrupts.  This will prevent some race
   * conditions with interrupt work.  There is still a potential race
   * condition with interrupt work that is already queued and in progress.
   */

  up_disable_irq(CONFIG_FTMAC100_IRQ);

  /* Schedule to perform the TX timeout processing on the worker thread. */

  work_queue(FTMAWORK, &priv->ft_irqwork, ftmac100_txtimeout_work, priv, 0);
}

/****************************************************************************
 * Name: ftmac100_poll_work
 *
 * Description:
 *   Perform periodic polling from the worker thread
 *
 * Input Parameters:
 *   arg - The argument passed when work_queue() as called.
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *   Ethernet interrupts are disabled
 *
 ****************************************************************************/

static void ftmac100_poll_work(FAR void *arg)
{
  FAR struct ftmac100_driver_s *priv = (FAR struct ftmac100_driver_s *)arg;

  /* Perform the poll */

  net_lock();

  /* Check if there is room in the send another TX packet.  We cannot perform
   * the TX poll if he are unable to accept another packet for transmission.
   */

  /* If so, update TCP timing states and poll the network for new XMIT data. Hmmm..
   * might be bug here.  Does this mean if there is a transmit in progress,
   * we will missing TCP time state updates?
   */

  (void)devif_timer(&priv->ft_dev, ftmac100_txpoll);

  /* Setup the watchdog poll timer again */

  (void)wd_start(priv->ft_txpoll, FTMAC100_WDDELAY, ftmac100_poll_expiry, 1,
                 (wdparm_t)priv);
  net_unlock();
}

/****************************************************************************
 * Name: ftmac100_poll_expiry
 *
 * Description:
 *   Periodic timer handler.  Called from the timer interrupt handler.
 *
 * Input Parameters:
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

static void ftmac100_poll_expiry(int argc, uint32_t arg, ...)
{
  FAR struct ftmac100_driver_s *priv = (FAR struct ftmac100_driver_s *)arg;

  /* Schedule to perform the interrupt processing on the worker thread. */

  work_queue(FTMAWORK, &priv->ft_pollwork, ftmac100_poll_work, priv, 0);
}

/****************************************************************************
 * Name: ftmac100_ifup
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

static int ftmac100_ifup(struct net_driver_s *dev)
{
  FAR struct ftmac100_driver_s *priv = (FAR struct ftmac100_driver_s *)dev->d_private;

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

  /* Initialize PHYs, the Ethernet interface, and setup up Ethernet
   * interrupts.
   */

  ftmac100_init(priv);

  /* Instantiate the MAC address from priv->ft_dev.d_mac.ether.ether_addr_octet */

  ftmac100_set_mac(priv, priv->ft_dev.d_mac.ether.ether_addr_octet);

#ifdef CONFIG_NET_ICMPv6
  /* Set up IPv6 multicast address filtering */

  ftmac100_ipv6multicast(priv);
#endif

  /* Set and activate a timer process */

  (void)wd_start(priv->ft_txpoll, FTMAC100_WDDELAY, ftmac100_poll_expiry, 1,
                 (wdparm_t)priv);

  /* Enable the Ethernet interrupt */

  priv->ft_bifup = true;
  up_enable_irq(CONFIG_FTMAC100_IRQ);
  return OK;
}

/****************************************************************************
 * Name: ftmac100_ifdown
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

static int ftmac100_ifdown(struct net_driver_s *dev)
{
  FAR struct ftmac100_driver_s *priv = (FAR struct ftmac100_driver_s *)dev->d_private;
  FAR struct ftmac100_register_s *iobase = (FAR struct ftmac100_register_s *)priv->iobase;
  irqstate_t flags;

  /* Disable the Ethernet interrupt */

  flags = enter_critical_section();
  up_disable_irq(CONFIG_FTMAC100_IRQ);

  /* Cancel the TX poll timer and TX timeout timers */

  wd_cancel(priv->ft_txpoll);
  wd_cancel(priv->ft_txtimeout);

  /* Put the EMAC in its reset, non-operational state.  This should be
   * a known configuration that will guarantee the ftmac100_ifup() always
   * successfully brings the interface back up.
   */

  putreg32 (0, &iobase->maccr);

  /* Mark the device "down" */

  priv->ft_bifup = false;
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: ftmac100_txavail_work
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

static void ftmac100_txavail_work(FAR void *arg)
{
  FAR struct ftmac100_driver_s *priv = (FAR struct ftmac100_driver_s *)arg;

  /* Perform the poll */

  net_lock();

  /* Ignore the notification if the interface is not yet up */

  if (priv->ft_bifup)
    {
      /* Check if there is room in the hardware to hold another outgoing packet. */

      /* If so, then poll the network for new XMIT data */

      (void)devif_poll(&priv->ft_dev, ftmac100_txpoll);
    }

  net_unlock();
}

/****************************************************************************
 * Name: ftmac100_txavail
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

static int ftmac100_txavail(struct net_driver_s *dev)
{
  FAR struct ftmac100_driver_s *priv = (FAR struct ftmac100_driver_s *)dev->d_private;

  /* Is our single work structure available?  It may not be if there are
   * pending interrupt actions and we will have to ignore the Tx
   * availability action.
   */

  if (work_available(&priv->ft_pollwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      work_queue(FTMAWORK, &priv->ft_pollwork, ftmac100_txavail_work, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: ftmac100_addmac
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

#if defined(CONFIG_NET_IGMP) || defined(CONFIG_NET_ICMPv6)
static int ftmac100_addmac(struct net_driver_s *dev, FAR const uint8_t *mac)
{
  FAR struct ftmac100_driver_s *priv = (FAR struct ftmac100_driver_s *)dev->d_private;
  FAR struct ftmac100_register_s *iobase = (FAR struct ftmac100_register_s *)priv->iobase;
  uint32_t hash_value, hash_reg, hash_bit;
  uint32_t mta;

  /* Calculate Ethernet CRC32 for MAC */

  hash_value = crc32part(mac, 6, ~0L);

  /* The HASH Table  is a register array of 2 32-bit registers.
   * It is treated like an array of 64 bits.  We want to set
   * bit BitArray[hash_value]. So we figure out what register
   * the bit is in, read it, OR in the new bit, then write
   * back the new value.  The register is determined by the
   * upper 7 bits of the hash value and the bit within that
   * register are determined by the lower 5 bits of the value.
   */

  hash_reg = (hash_value >> 31) & 0x1;
  hash_bit = (hash_value >> 26) & 0x1f;

  /* Add the MAC address to the hardware multicast routing table */

  mta = getreg32(&iobase->maht0 + hash_reg);

  mta |= (1 << hash_bit);

  putreg32(mta, &iobase->maht0 + hash_reg);

  return OK;
}
#endif

/****************************************************************************
 * Name: ftmac100_rmmac
 *
 * Description:
 *   NuttX Callback: Remove the specified MAC address from the hardware multicast
 *   address filtering
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

#ifdef CONFIG_NET_IGMP
static int ftmac100_rmmac(struct net_driver_s *dev, FAR const uint8_t *mac)
{
  FAR struct ftmac100_driver_s *priv = (FAR struct ftmac100_driver_s *)dev->d_private;
  FAR struct ftmac100_register_s *iobase = (FAR struct ftmac100_register_s *)priv->iobase;
  uint32_t hash_value, hash_reg, hash_bit;
  uint32_t mta;

  /* Calculate Ethernet CRC32 for MAC */

  hash_value = crc32part(mac, 6, ~0L);

  hash_reg = (hash_value >> 31) & 0x1;
  hash_bit = (hash_value >> 26) & 0x1f;

  /* Remove the MAC address to the hardware multicast routing table */

  mta = getreg32(&iobase->maht0 + hash_reg);

  mta &= ~(1 << hash_bit);

  putreg32(mta, &iobase->maht0 + hash_reg);

  return OK;
}
#endif

/****************************************************************************
 * Name: ftmac100_ipv6multicast
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
static void ftmac100_ipv6multicast(FAR struct ftmac100_driver_s *priv)
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

  dev    = &priv->ft_dev;
  tmp16  = dev->d_ipv6addr[6];
  mac[2] = 0xff;
  mac[3] = tmp16 >> 8;

  tmp16  = dev->d_ipv6addr[7];
  mac[4] = tmp16 & 0xff;
  mac[5] = tmp16 >> 8;

  ninfo("IPv6 Multicast: %02x:%02x:%02x:%02x:%02x:%02x\n",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  (void)ftmac100_addmac(dev, mac);

#ifdef CONFIG_NET_ICMPv6_AUTOCONF
  /* Add the IPv6 all link-local nodes Ethernet address.  This is the
   * address that we expect to receive ICMPv6 Router Advertisement
   * packets.
   */

  (void)ftmac100_addmac(dev, g_ipv6_ethallnodes.ether_addr_octet);

#endif /* CONFIG_NET_ICMPv6_AUTOCONF */
#ifdef CONFIG_NET_ICMPv6_ROUTER
  /* Add the IPv6 all link-local routers Ethernet address.  This is the
   * address that we expect to receive ICMPv6 Router Solicitation
   * packets.
   */

  (void)ftmac100_addmac(dev, g_ipv6_ethallrouters.ether_addr_octet);

#endif /* CONFIG_NET_ICMPv6_ROUTER */
}
#endif /* CONFIG_NET_ICMPv6 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ftmac100_initialize
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

int ftmac100_initialize(int intf)
{
  struct ftmac100_driver_s *priv;

  /* Get the interface structure associated with this interface number. */

  DEBUGASSERT(intf < CONFIG_FTMAC100_NINTERFACES);
  priv = &g_ftmac100[intf];

  /* Attach the IRQ to the driver */

  if (irq_attach(CONFIG_FTMAC100_IRQ, ftmac100_interrupt, NULL))
    {
      /* We could not attach the ISR to the interrupt */

      return -EAGAIN;
    }

  /* Initialize the driver structure */

  memset(priv, 0, sizeof(struct ftmac100_driver_s));
  priv->ft_dev.d_buf     = g_pktbuf;          /* Single packet buffer */
  priv->ft_dev.d_ifup    = ftmac100_ifup;     /* I/F up (new IP address) callback */
  priv->ft_dev.d_ifdown  = ftmac100_ifdown;   /* I/F down callback */
  priv->ft_dev.d_txavail = ftmac100_txavail;  /* New TX data callback */
#ifdef CONFIG_NET_IGMP
  priv->ft_dev.d_addmac  = ftmac100_addmac;   /* Add multicast MAC address */
  priv->ft_dev.d_rmmac   = ftmac100_rmmac;    /* Remove multicast MAC address */
#endif
  priv->ft_dev.d_private = (FAR void *)g_ftmac100; /* Used to recover private state from dev */

  /* Create a watchdog for timing polling for and timing of transmissions */

  priv->ft_txpoll       = wd_create();        /* Create periodic poll timer */
  priv->ft_txtimeout    = wd_create();        /* Create TX timeout timer */

  priv->iobase          = CONFIG_FTMAC100_BASE;

  /* Put the interface in the down state.  This usually amounts to resetting
   * the device and/or calling ftmac100_ifdown().
   */
  ftmac100_reset(priv);

  /* Read the MAC address from the hardware into priv->ft_dev.d_mac.ether.ether_addr_octet */

  memcpy(priv->ft_dev.d_mac.ether.ether_addr_octet, (void *)(CONFIG_FTMAC100_MAC0_ENV_ADDR), 6);

  /* Register the device with the OS so that socket IOCTLs can be performed */

  (void)netdev_register(&priv->ft_dev, NET_LL_ETHERNET);
  return OK;
}

#endif /* CONFIG_NET && CONFIG_NET_FTMAC100 */
