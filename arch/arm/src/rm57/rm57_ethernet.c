/****************************************************************************
 * arch/arm/src/rm57/rm57_ethernet.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

/* RM57L843 EMAC (10/100 CPSW-lite Ethernet MAC) driver.
 *
 * There is exactly one EMAC on this device (a single "core 0"), so unlike
 * this port's DCAN driver this file uses one static instance rather than
 * an instance table, and needs no separate front-end split - it registers
 * directly with the network stack via netdev_register().
 *
 * CPPI descriptor rings live in the EMAC's internal 8KB CPPI RAM
 * (RM57_EMAC_CPPI_RAM_BASE), which this port's MPU maps as strongly-
 * ordered/non-cacheable device memory (see rm57_mpuinit.h) - so
 * descriptor fields need no D-cache maintenance.  Packet buffers live in
 * ordinary cacheable SRAM and are explicitly clean/invalidated around
 * DMA, following the same discipline as rm57_serial.c's DMA buffers.
 *
 * TX uses a small zero-copy free-buffer pool (buffers are swapped into
 * dev->d_buf, not memcpy'd) so devif_poll() can build packets directly
 * into a buffer that is then handed to the descriptor ring.  RX buffers
 * are fixed 1:1 with RX descriptors and are never swapped: a completed
 * packet is processed synchronously (net_lock held) with dev->d_buf
 * pointed directly at the descriptor's buffer, then the same descriptor
 * is immediately re-armed with the same buffer.
 *
 * Both TX and RX descriptor rings are managed as CPPI "append to a
 * running queue" chains per SPNU562A Section 31.2.6.2: a descriptor is
 * linked after the current tail (or written directly to TXHDP/RXHDP if
 * the channel is idle); completion is detected by polling the OWNER bit
 * clear and acknowledged by writing the completed descriptor's address
 * to TXCP/RXCP, with EOQ-race recovery (restarting the channel from the
 * next linked descriptor, or marking the channel idle) exactly as
 * described in that section.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <errno.h>
#include <debug.h>

#include <arpa/inet.h>

#include <arch/barriers.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>
#include <nuttx/queue.h>
#include <nuttx/signal.h>
#include <nuttx/cache.h>

#include <nuttx/net/mii.h>
#include <nuttx/net/net.h>
#include <nuttx/net/ethernet.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/ioctl.h>

#ifdef CONFIG_ARCH_PHY_INTERRUPT
#  include <nuttx/net/phy.h>
#endif

#ifdef CONFIG_NET_PKT
#  include <nuttx/net/pkt.h>
#endif

#include <arch/board/board.h>

#include "arm_internal.h"
#include "hardware/rm57_ethernet.h"
#include "rm57_ethernet.h"
#include "rm57_mdio.h"

#ifdef CONFIG_RM57_EMAC_REGDEBUG
#  include "hardware/rm57_iomm.h"
#  include "hardware/rm57_sys.h"
#endif

#ifdef CONFIG_RM57_EMAC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ETHWORK LPWORK

/* Packet buffer size: CONFIG_NET_ETH_PKTSIZE plus room for the 4-byte
 * CRC, rounded up to a multiple of the 32-byte D-cache line size so that
 * up_invalidate_dcache() on this buffer can never touch a neighbor's
 * data.  This also becomes RXMAXLEN, so hardware can never DMA past the
 * end of a fixed RX buffer.
 */

#define RM57_EMAC_BUFSIZE   (((CONFIG_NET_ETH_PKTSIZE + 4) + 31) & ~31)

#define RM57_EMAC_NTXDESC   CONFIG_RM57_EMAC_NTXDESC
#define RM57_EMAC_NRXDESC   CONFIG_RM57_EMAC_NRXDESC
#define RM57_EMAC_NTXBUF    (RM57_EMAC_NTXDESC + 1)

/* CPPI RAM partition: TX ring first, RX ring immediately after */

#define RM57_EMAC_TXRING  \
  ((FAR struct rm57_emac_desc_s *)(uintptr_t)RM57_EMAC_CPPI_RAM_BASE)
#define RM57_EMAC_RXRING  (RM57_EMAC_TXRING + RM57_EMAC_NTXDESC)

#if (RM57_EMAC_NTXDESC + RM57_EMAC_NRXDESC) > RM57_EMAC_MAX_DESC
#  error RM57_EMAC_NTXDESC + RM57_EMAC_NRXDESC exceeds the 8KB CPPI RAM
#endif

/* TX timeout: 60 seconds is generous; a real stuck-transmitter condition
 * should be detected far sooner via a link/host-error interrupt, this is
 * only the last-resort backstop.
 */

#define RM57_EMAC_TXTIMEOUT   (60 * CLOCKS_PER_SEC)

/* Bounded loops guard against a wedged descriptor chain hanging a
 * worker forever, matching the RM57_DCAN_ISR_BOUND idiom in
 * rm57_dcan.c.
 */

#define RM57_EMAC_RXDONE_BOUND  (RM57_EMAC_NRXDESC)
#define RM57_EMAC_TXDONE_BOUND  (RM57_EMAC_NTXDESC)

/* Autonegotiation poll: 10ms steps, ~5 second bound (TRM notes a PHY can
 * take up to 3 seconds to negotiate a link).
 */

#define RM57_EMAC_ANEG_POLL_MS    10
#define RM57_EMAC_ANEG_MAX_POLLS  500

/* Settle poll for rm57emac_linkupdate(): covers the small gap between BMSR
 * reporting LINKSTATUS and reporting ANEGCOMPLETE.  1ms steps; longer than
 * this means autonegotiation is not running at all.
 */

#define RM57_EMAC_ANEG_LATCH_POLLS 20

/* ALIVE poll: the MDIO module's PHY-detect state machine needs to
 * complete at least one full sweep of all 32 addresses after CONTROL.
 * ENABLE is set before ALIVE reflects a connected PHY (SPNU562A
 * 31.2.8.2.1 step 3: "The MDIO PHY alive status register (ALIVE) can
 * be read in polling fashion until a PHY connected to the system
 * responded") - a single immediate read races that sweep and always
 * reports 0.  1ms steps, 100ms bound is generous for one sweep at the
 * slowest permitted MDIO clock.
 */

#define RM57_EMAC_ALIVE_POLL_MS    1
#define RM57_EMAC_ALIVE_MAX_POLLS  100

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rm57_ethmac_s
{
  bool     ifup;      /* true: interface up */
  bool     mbps100;   /* true: 100Mbps, false: 10Mbps */
  bool     fduplex;   /* true: full duplex */
  uint8_t  phyaddr;   /* PHY address in use */

  struct wdog_s txtimeout;
  struct work_s irqwork;     /* Deferred work for C0_MISC_PULSE */
  struct work_s rxwork;      /* Deferred work for C0_RX_PULSE */
  struct work_s txwork;      /* Deferred work for C0_TX_PULSE */
  struct work_s pollwork;    /* Deferred work for d_txavail */
  struct work_s timeoutwork; /* Deferred work for the TX timeout watchdog */

#ifdef CONFIG_ARCH_PHY_INTERRUPT
  /* The MDIO module has a single LINKINTENB bit per PHY, which this
   * driver keeps permanently enabled (set in rm57emac_ifup()) for its
   * own netdev_carrier_on/off tracking - regardless of whether a
   * phy_notify subscriber is attached.  phynotify_armed is therefore a
   * separate software gate satisfying arch_phy_irq()'s documented
   * contract (disabled until the caller explicitly re-enables it via
   * the returned phy_enable_t) without disturbing that shared hardware
   * bit.
   */

  xcpt_t   phyhandler;
  FAR void *phyarg;
  bool     phynotify_armed;
#endif

  struct net_driver_s dev; /* Interface understood by the network layer */

  /* CPPI descriptor rings (in CPPI RAM - see RM57_EMAC_TXRING/RXRING) */

  FAR struct rm57_emac_desc_s *txnext;    /* Next free TX descriptor to fill */
  FAR struct rm57_emac_desc_s *txtail;    /* Last desc. appended to the active TX queue, NULL if idle */
  FAR struct rm57_emac_desc_s *txreclaim; /* Oldest not-yet-reclaimed TX descriptor */
  uint16_t txused;                        /* # TX descriptors currently in flight */

  FAR struct rm57_emac_desc_s *rxnext;    /* Next RX descriptor expected to complete */
  FAR struct rm57_emac_desc_s *rxtail;    /* Last desc. appended to the active RX queue */

  sq_queue_t txfreeb;  /* Free TX packet buffer pool (RM57_EMAC_NTXBUF entries) */

  uint8_t  mcrefcnt[64]; /* Per-hash-bit multicast reference counts */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct rm57_ethmac_s g_rm57emac;

/* TX packet buffer pool - one extra slot beyond the descriptor count so
 * devif_poll() always has somewhere to build the next packet while all
 * NTXDESC descriptors are in flight.
 */

static uint8_t g_rm57emac_txbuf[RM57_EMAC_NTXBUF][RM57_EMAC_BUFSIZE]
  aligned_data(32);

/* RX packet buffers - fixed 1:1 with RX descriptors, never swapped */

static uint8_t g_rm57emac_rxbuf[RM57_EMAC_NRXDESC][RM57_EMAC_BUFSIZE]
  aligned_data(32);

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static FAR uint8_t *rm57emac_allocbuffer(FAR struct rm57_ethmac_s *priv);
static void rm57emac_freebuffer(FAR struct rm57_ethmac_s *priv,
                                FAR uint8_t *buf);

static void rm57emac_enqueue(uintptr_t headreg,
                             FAR struct rm57_emac_desc_s **tailp,
                             FAR struct rm57_emac_desc_s *desc);

static int  rm57emac_transmit(FAR struct rm57_ethmac_s *priv);
static int  rm57emac_txpoll(FAR struct net_driver_s *dev);
static void rm57emac_dopoll(FAR struct rm57_ethmac_s *priv);

static void rm57emac_dispatch(FAR struct rm57_ethmac_s *priv);
static void rm57emac_receive(FAR struct rm57_ethmac_s *priv);
static void rm57emac_txreap(FAR struct rm57_ethmac_s *priv);
static void rm57emac_txdone(FAR struct rm57_ethmac_s *priv);

static void rm57emac_rx_work(FAR void *arg);
static void rm57emac_tx_work(FAR void *arg);
static void rm57emac_misc_work(FAR void *arg);
static int  rm57emac_interrupt(int irq, FAR void *context, FAR void *arg);

static void rm57emac_txtimeout_work(FAR void *arg);
static void rm57emac_txtimeout_expiry(wdparm_t arg);

static void rm57emac_pollwork(FAR void *arg);
static int  rm57emac_txavail(FAR struct net_driver_s *dev);

#ifdef CONFIG_NET_MCASTGROUP
static int  rm57emac_addmac(FAR struct net_driver_s *dev,
                            FAR const uint8_t *mac);
static int  rm57emac_rmmac(FAR struct net_driver_s *dev,
                           FAR const uint8_t *mac);
#endif

#ifdef CONFIG_NETDEV_IOCTL
static int  rm57emac_ioctl(FAR struct net_driver_s *dev, int cmd,
                           unsigned long arg);
#endif

static int  rm57emac_phyread(uint16_t phyaddr, uint8_t regaddr,
                             FAR uint16_t *value);
static int  rm57emac_phywrite(uint16_t phyaddr, uint8_t regaddr,
                              uint16_t value);
static int  rm57emac_phyfind(FAR struct rm57_ethmac_s *priv);
static int  rm57emac_phyinit(FAR struct rm57_ethmac_s *priv);
static void rm57emac_linkupdate(FAR struct rm57_ethmac_s *priv);

static void rm57emac_setmacaddr(uint32_t channel, FAR const uint8_t *mac,
                                uint32_t matchfilt);
static void rm57emac_macaddress(FAR struct rm57_ethmac_s *priv);
static void rm57emac_stats_update(FAR struct rm57_ethmac_s *priv);

static void rm57emac_txdescinit(FAR struct rm57_ethmac_s *priv);
static void rm57emac_rxdescinit(FAR struct rm57_ethmac_s *priv);
static void rm57emac_reset(void);
static int  rm57emac_configure(FAR struct rm57_ethmac_s *priv);

static int  rm57emac_ifup(FAR struct net_driver_s *dev);
static int  rm57emac_ifdown(FAR struct net_driver_s *dev);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rm57emac_allocbuffer / rm57emac_freebuffer
 ****************************************************************************/

static FAR uint8_t *rm57emac_allocbuffer(FAR struct rm57_ethmac_s *priv)
{
  return (FAR uint8_t *)sq_remfirst(&priv->txfreeb);
}

static void rm57emac_freebuffer(FAR struct rm57_ethmac_s *priv,
                                FAR uint8_t *buf)
{
  sq_addlast((FAR sq_entry_t *)buf, &priv->txfreeb);
}

/****************************************************************************
 * Name: rm57emac_enqueue
 *
 * Description:
 *   Append 'desc' to the active hardware descriptor chain rooted at
 *   headreg (TXHDP(0) or RXHDP(0)), starting the channel directly if it
 *   was idle (*tailp == NULL).  desc->next is left NULL, matching TRM
 *   Section 31.2.6.4.1's "0 = last descriptor in the queue" convention.
 *
 ****************************************************************************/

static void rm57emac_enqueue(uintptr_t headreg,
                             FAR struct rm57_emac_desc_s **tailp,
                             FAR struct rm57_emac_desc_s *desc)
{
  FAR struct rm57_emac_desc_s *tail = *tailp;

  desc->next = NULL;
  UP_DSB();

  if (tail == NULL)
    {
      putreg32((uint32_t)(uintptr_t)desc, headreg);
    }
  else
    {
      tail->next = desc;
      UP_DSB();
    }

  *tailp = desc;
}

/****************************************************************************
 * Name: rm57emac_transmit
 *
 * Description:
 *   Queue dev->d_buf/d_len (already built by devif_poll()) onto the TX
 *   ring.  Caller (rm57emac_txpoll) has already verified the ring is not
 *   full.
 *
 ****************************************************************************/

static int rm57emac_transmit(FAR struct rm57_ethmac_s *priv)
{
  FAR struct net_driver_s *dev = &priv->dev;
  FAR struct rm57_emac_desc_s *desc = priv->txnext;
  size_t idx = desc - RM57_EMAC_TXRING;
  uint16_t txlen = dev->d_len;

  /* The 802.3 minimum frame is 64 bytes on the wire, i.e. 60 bytes of
   * header+payload before the 4-byte CRC the EMAC appends itself
   * (PASSCRC is left clear).  ARP requests/replies are only 42 bytes
   * and nothing pads them out upstream, so without this a short frame
   * either gets silently dropped as a runt by the far end or, worse,
   * transmits whatever stale bytes happen to follow the packet in the
   * buffer.  The buffer is sized for a full MTU (RM57_EMAC_BUFSIZE), so
   * there is always room to extend it.
   */

  if (txlen < 60)
    {
      memset(dev->d_buf + txlen, 0, 60 - txlen);
      txlen = 60;
    }

  desc->buffer = (FAR uint8_t *)dev->d_buf;
  desc->bufoff = 0;
  desc->buflen = txlen;

  up_clean_dcache((uintptr_t)dev->d_buf,
                  (uintptr_t)dev->d_buf + txlen);

  desc->flags_pktlen = EMAC_DESC_SOP | EMAC_DESC_EOP | EMAC_DESC_OWNER |
                       (txlen & EMAC_DESC_PKTLEN_MASK);

  rm57emac_enqueue(RM57_EMAC_TXHDP(0), &priv->txtail, desc);

  priv->txused++;
  priv->txnext = &RM57_EMAC_TXRING[(idx + 1) % RM57_EMAC_NTXDESC];

  wd_start(&priv->txtimeout, RM57_EMAC_TXTIMEOUT,
           rm57emac_txtimeout_expiry, (wdparm_t)priv);

  NETDEV_TXPACKETS(dev);

  /* The descriptor owns this buffer now, so drop the network layer's
   * reference.  Left set, the -EBUSY path in rm57emac_txpoll() returns with
   * d_buf still pointing at it, devif_poll() restores that pointer and
   * rm57emac_dopoll() frees it while the DMA is still reading - then
   * rm57emac_txdone() frees it a second time, which loops the free list onto
   * itself and leaves every descriptor sharing one buffer.  Must stay after
   * NETDEV_TXPACKETS(), which reads d_len for tx_bytes.
   */

  dev->d_buf = NULL;
  dev->d_len = 0;

  return OK;
}

/****************************************************************************
 * Name: rm57emac_txpoll
 *
 * Description:
 *   devif_poll() callback: dev->d_len > 0 means a packet was built into
 *   dev->d_buf and should be sent.  Always tries to leave dev->d_buf
 *   holding a fresh buffer for the next packet devif_poll() may build;
 *   returning -EBUSY/-ENOMEM stops the poll for this pass.
 *
 ****************************************************************************/

static int rm57emac_txpoll(FAR struct net_driver_s *dev)
{
  FAR struct rm57_ethmac_s *priv =
    (FAR struct rm57_ethmac_s *)dev->d_private;

  if (dev->d_len > 0)
    {
      /* devif_poll() has already resolved L2 addressing (arp_out()/
       * neighbor_out()) before invoking this callback - see
       * net/devif/devif_poll.c.
       */

      rm57emac_transmit(priv);

      DEBUGASSERT(dev->d_len == 0 && dev->d_buf == NULL);

      if (priv->txused >= RM57_EMAC_NTXDESC)
        {
          return -EBUSY;
        }

      dev->d_buf = rm57emac_allocbuffer(priv);
      if (dev->d_buf == NULL)
        {
          return -ENOMEM;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: rm57emac_dopoll
 ****************************************************************************/

static void rm57emac_dopoll(FAR struct rm57_ethmac_s *priv)
{
  FAR struct net_driver_s *dev = &priv->dev;

  if (priv->txused < RM57_EMAC_NTXDESC)
    {
      dev->d_buf = rm57emac_allocbuffer(priv);
      if (dev->d_buf != NULL)
        {
          devif_poll(dev, rm57emac_txpoll);

          if (dev->d_buf != NULL)
            {
              rm57emac_freebuffer(priv, dev->d_buf);
              dev->d_buf = NULL;
              dev->d_len = 0;
            }
        }
    }
}

/****************************************************************************
 * Name: rm57emac_dispatch
 *
 * Description:
 *   dev->d_buf/d_len hold one received frame (priv->dev.d_buf points
 *   directly into the RX descriptor's fixed buffer).  Demux by
 *   ethertype and hand off to the appropriate _input() function; if that
 *   produces a reply, transmit it synchronously.
 *
 ****************************************************************************/

static void rm57emac_dispatch(FAR struct rm57_ethmac_s *priv)
{
  FAR struct net_driver_s *dev = &priv->dev;
  FAR struct eth_hdr_s *eth = (FAR struct eth_hdr_s *)dev->d_buf;

#ifdef CONFIG_NET_PKT
  pkt_input(dev);
#endif

#ifdef CONFIG_NET_IPv4
  if (eth->type == HTONS(ETHTYPE_IP))
    {
      NETDEV_RXIPV4(dev);
      ipv4_input(dev);
    }
  else
#endif
#ifdef CONFIG_NET_IPv6
  if (eth->type == HTONS(ETHTYPE_IP6))
    {
      NETDEV_RXIPV6(dev);
      ipv6_input(dev);
    }
  else
#endif
#ifdef CONFIG_NET_ARP
  if (eth->type == HTONS(ETHTYPE_ARP))
    {
      NETDEV_RXARP(dev);
      arp_input(dev);
    }
  else
#endif
    {
      NETDEV_RXDROPPED(dev);
      return;
    }

  /* ipv4_input()/ipv6_input()/arp_input() build a direct reply in place
   * (swapping src/dst) without needing arp_out()/neighbor_out() - those
   * are only for NEW outbound packets originated via devif_poll(), which
   * already calls them itself (net/devif/devif_poll.c) before invoking
   * rm57emac_txpoll().
   *
   * A synchronous reply was built into the still-borrowed RX buffer.
   * dev->d_buf still points at the RX descriptor's fixed buffer here;
   * rm57emac_transmit() copies its pointer into a TX descriptor (not
   * this driver's buffer pool), which would corrupt the RX ring once
   * this descriptor is re-armed.  Route the reply through the TX free
   * buffer pool instead.
   */

  if (dev->d_len > 0)
    {
      FAR uint8_t *txbuf;

      /* Reclaim finished descriptors before calling the ring full.  All EMAC
       * work shares one LPWORK thread, so rm57emac_tx_work() cannot run
       * until this RX burst ends and the ring would fill even though the
       * hardware is long done with it.  rm57emac_txreap() is used rather
       * than rm57emac_txdone() because it does not call rm57emac_dopoll(),
       * which would re-enter devif_poll() from inside a dispatch.
       */

      if (priv->txused >= RM57_EMAC_NTXDESC)
        {
          rm57emac_txreap(priv);
        }

      txbuf = rm57emac_allocbuffer(priv);

      if (txbuf != NULL && priv->txused < RM57_EMAC_NTXDESC)
        {
          memcpy(txbuf, dev->d_buf, dev->d_len);
          dev->d_buf = txbuf;
          rm57emac_transmit(priv);
          dev->d_buf = NULL;
        }
      else
        {
          if (txbuf != NULL)
            {
              rm57emac_freebuffer(priv, txbuf);
            }

          /* The stack believes this packet was sent, so a silent drop costs
           * a full RTO before TCP notices.  A recurring warning here means
           * the TX ring is not draining.
           */

          nwarn("WARNING: TX pool exhausted, dropped %u bytes "
                "(txused=%u/%u, buf=%s)\n",
                (unsigned)dev->d_len, (unsigned)priv->txused,
                (unsigned)RM57_EMAC_NTXDESC,
                txbuf != NULL ? "ok" : "none");

          NETDEV_TXERRORS(dev);
        }

      dev->d_len = 0;
    }
}

/****************************************************************************
 * Name: rm57emac_receive
 *
 * Description:
 *   Walk the RX ring from priv->rxnext while descriptors have completed
 *   (OWNER clear), dispatching each good frame and immediately re-arming
 *   and re-appending the same descriptor/buffer.
 *
 *   Handles the same EOQ race as the TX path (SPNU562A Section
 *   31.2.6.4.9): a completed descriptor that halted the channel has to
 *   restart it explicitly, otherwise reception stops for good.
 *
 ****************************************************************************/

static void rm57emac_receive(FAR struct rm57_ethmac_s *priv)
{
  FAR struct net_driver_s *dev = &priv->dev;
  FAR struct rm57_emac_desc_s *desc;
  FAR volatile struct rm57_emac_desc_s *next;
  bool eoq;
  size_t idx;
  int bound;

  for (bound = 0; bound < RM57_EMAC_RXDONE_BOUND; bound++)
    {
      desc = priv->rxnext;

      if ((desc->flags_pktlen & EMAC_DESC_OWNER) != 0)
        {
          break;
        }

      putreg32((uint32_t)(uintptr_t)desc, RM57_EMAC_RXCP(0));

      /* Latch the EOQ state now: flags_pktlen is overwritten by the
       * re-arm below and next by the rm57emac_enqueue() after it.
       */

      eoq  = (desc->flags_pktlen & EMAC_DESC_EOQ) != 0;
      next = desc->next;

      if ((desc->flags_pktlen & (EMAC_DESC_SOP | EMAC_DESC_EOP)) ==
          (EMAC_DESC_SOP | EMAC_DESC_EOP) &&
          (desc->flags_pktlen &
           (EMAC_DESC_RX_CRCERROR | EMAC_DESC_RX_ALIGNERROR |
            EMAC_DESC_RX_CODEERROR | EMAC_DESC_RX_OVERRUN |
            EMAC_DESC_RX_FRAGMENT | EMAC_DESC_RX_JABBER)) == 0)
        {
          uint16_t pktlen = desc->flags_pktlen & EMAC_DESC_PKTLEN_MASK;

          up_invalidate_dcache((uintptr_t)desc->buffer,
                               (uintptr_t)desc->buffer +
                               RM57_EMAC_BUFSIZE);

          dev->d_len = pktlen;
          dev->d_buf = (FAR uint8_t *)desc->buffer;

          NETDEV_RXPACKETS(dev);

          rm57emac_dispatch(priv);

          dev->d_buf = NULL;
          dev->d_len = 0;
        }
      else
        {
          NETDEV_RXERRORS(dev);
        }

      /* Re-arm this descriptor with the same (fixed) buffer.  netdev_input()
       * copies the stack's reply back into this buffer (iob_copyout()), and
       * SRAM is write-back, so those dirty lines must go before the DMA owns
       * the buffer again - otherwise they are evicted over the next frame.
       * Discarding is safe: rm57emac_dispatch() has already copied any reply
       * into a TX pool buffer.  The invalidate above is still needed as
       * well; it drops lines the core may have speculatively filled.
       */

      up_invalidate_dcache((uintptr_t)desc->buffer,
                           (uintptr_t)desc->buffer + RM57_EMAC_BUFSIZE);

      desc->bufoff = 0;
      desc->buflen = RM57_EMAC_BUFSIZE;
      desc->flags_pktlen = EMAC_DESC_OWNER;

      idx = desc - RM57_EMAC_RXRING;
      priv->rxnext = &RM57_EMAC_RXRING[(idx + 1) % RM57_EMAC_NRXDESC];

      /* If this descriptor halted the channel, restart it: either at the
       * descriptor that had already been linked behind it while the halt
       * was in flight, or - when it was the last one in the queue - by
       * dropping the tail so that the rm57emac_enqueue() below writes
       * RXHDP again with this just re-armed descriptor.  Without this the
       * queue never restarts: rxtail would stay non-NULL forever and
       * RXHDP would keep the stale value written by rm57emac_rxdescinit().
       */

      if (eoq)
        {
          if (next != NULL)
            {
              putreg32((uint32_t)(uintptr_t)next, RM57_EMAC_RXHDP(0));
            }
          else
            {
              priv->rxtail = NULL;
            }
        }

      rm57emac_enqueue(RM57_EMAC_RXHDP(0), &priv->rxtail, desc);
    }
}

/****************************************************************************
 * Name: rm57emac_txreap
 *
 * Description:
 *   Walk the TX ring from priv->txreclaim while descriptors have
 *   completed (OWNER clear), acknowledging each and freeing its buffer.
 *   Handles the EOQ race described in SPNU562A Section 31.2.6.4.9: if a
 *   completed descriptor halted the channel (EOQ set) but already had a
 *   descriptor linked after it (appended while the halt was in flight),
 *   restart the channel there.
 *
 *   Deliberately does not poll the network layer, so that it is safe to
 *   call from rm57emac_dispatch().  See rm57emac_txdone().
 *
 ****************************************************************************/

static void rm57emac_txreap(FAR struct rm57_ethmac_s *priv)
{
  FAR struct rm57_emac_desc_s *desc;
  size_t idx;
  int bound;

  for (bound = 0; bound < RM57_EMAC_TXDONE_BOUND && priv->txused > 0;
       bound++)
    {
      desc = priv->txreclaim;

      if ((desc->flags_pktlen & EMAC_DESC_OWNER) != 0)
        {
          break;
        }

      putreg32((uint32_t)(uintptr_t)desc, RM57_EMAC_TXCP(0));

      if ((desc->flags_pktlen & EMAC_DESC_EOQ) != 0)
        {
          if (desc->next != NULL)
            {
              putreg32((uint32_t)(uintptr_t)desc->next, RM57_EMAC_TXHDP(0));
            }
          else
            {
              priv->txtail = NULL;
            }
        }

      rm57emac_freebuffer(priv, (FAR uint8_t *)desc->buffer);

      /* Count the completion, not just the enqueue; otherwise ifconfig's TX
       * "Sent" column stays at zero however much traffic passes.
       */

      NETDEV_TXDONE(&priv->dev);

      idx = desc - RM57_EMAC_TXRING;
      priv->txreclaim = &RM57_EMAC_TXRING[(idx + 1) % RM57_EMAC_NTXDESC];
      priv->txused--;
    }
}

/****************************************************************************
 * Name: rm57emac_txdone
 *
 * Description:
 *   Reclaim completed TX descriptors, then re-evaluate the TX watchdog and
 *   give the network layer another poll.
 *
 ****************************************************************************/

static void rm57emac_txdone(FAR struct rm57_ethmac_s *priv)
{
  rm57emac_txreap(priv);

  /* Only disarm the watchdog once the ring has drained; re-arm it while
   * frames are still in flight.  Cancelling unconditionally left no recovery
   * path for a missed completion: txused would stay non-zero and the ring
   * would look permanently full from then on.
   */

  if (priv->txused == 0)
    {
      wd_cancel(&priv->txtimeout);
    }
  else
    {
      wd_start(&priv->txtimeout, RM57_EMAC_TXTIMEOUT,
               rm57emac_txtimeout_expiry, (wdparm_t)priv);
    }

  if (priv->txused < RM57_EMAC_NTXDESC)
    {
      rm57emac_dopoll(priv);
    }
}

/****************************************************************************
 * Name: rm57emac_stats_update
 *
 * Description:
 *   Drain the 36 EMAC network statistics registers (STATPEND fires when
 *   any approaches overflow, SPNU562A Section 31.2.17.1.3), folding the
 *   error-class counters into dev->d_statistics and discarding the rest
 *   (this driver tracks its own rx/tx packet counts in the data path).
 *
 ****************************************************************************/

static void rm57emac_stats_update(FAR struct rm57_ethmac_s *priv)
{
  static const uintptr_t rxerr_regs[] =
  {
    RM57_EMAC_RXCRCERRORS, RM57_EMAC_RXALIGNCODEERRORS,
    RM57_EMAC_RXOVERSIZED, RM57_EMAC_RXJABBER, RM57_EMAC_RXUNDERSIZED,
    RM57_EMAC_RXFRAGMENTS, RM57_EMAC_RXSOFOVERRUNS,
    RM57_EMAC_RXMOFOVERRUNS, RM57_EMAC_RXDMAOVERRUNS
  };

  static const uintptr_t txerr_regs[] =
  {
    RM57_EMAC_TXUNDERRUN, RM57_EMAC_TXLATECOLL,
    RM57_EMAC_TXEXCESSIVECOLL, RM57_EMAC_TXCARRIERSENSE
  };

  static const uintptr_t other_regs[] =
  {
    RM57_EMAC_RXGOODFRAMES, RM57_EMAC_RXBCASTFRAMES,
    RM57_EMAC_RXMCASTFRAMES, RM57_EMAC_RXPAUSEFRAMES,
    RM57_EMAC_RXFILTERED, RM57_EMAC_RXQOSFILTERED, RM57_EMAC_RXOCTETS,
    RM57_EMAC_TXGOODFRAMES, RM57_EMAC_TXBCASTFRAMES,
    RM57_EMAC_TXMCASTFRAMES, RM57_EMAC_TXPAUSEFRAMES,
    RM57_EMAC_TXDEFERRED, RM57_EMAC_TXCOLLISION, RM57_EMAC_TXSINGLECOLL,
    RM57_EMAC_TXMULTICOLL, RM57_EMAC_TXOCTETS, RM57_EMAC_FRAME64,
    RM57_EMAC_FRAME65T127, RM57_EMAC_FRAME128T255,
    RM57_EMAC_FRAME256T511, RM57_EMAC_FRAME512T1023,
    RM57_EMAC_FRAME1024TUP, RM57_EMAC_NETOCTETS
  };

  FAR struct net_driver_s *dev = &priv->dev;
  uint32_t sum;
  size_t i;

  UNUSED(dev);

  sum = 0;
  for (i = 0; i < sizeof(rxerr_regs) / sizeof(rxerr_regs[0]); i++)
    {
      sum += getreg32(rxerr_regs[i]);
      putreg32(0, rxerr_regs[i]);
    }

#ifdef CONFIG_NETDEV_STATISTICS
  dev->d_statistics.rx_errors += sum;
#endif

  sum = 0;
  for (i = 0; i < sizeof(txerr_regs) / sizeof(txerr_regs[0]); i++)
    {
      sum += getreg32(txerr_regs[i]);
      putreg32(0, txerr_regs[i]);
    }

#ifdef CONFIG_NETDEV_STATISTICS
  dev->d_statistics.tx_errors += sum;
#endif

  for (i = 0; i < sizeof(other_regs) / sizeof(other_regs[0]); i++)
    {
      putreg32(0, other_regs[i]);
    }
}

/****************************************************************************
 * Name: rm57emac_rx_work / rm57emac_tx_work / rm57emac_misc_work
 *
 * Description:
 *   Deferred (LPWORK) processing for the three serviced VIM channels.
 *   Each follows SPNU562A Section 31.2.17.3: the top-half ISR already
 *   cleared the relevant C0xxEN bit; here we do the real work, then
 *   acknowledge via MACEOIVECTOR and re-enable C0xxEN last, so a
 *   still-pending condition regenerates a fresh pulse rather than being
 *   silently missed.
 *
 ****************************************************************************/

static void rm57emac_rx_work(FAR void *arg)
{
  FAR struct rm57_ethmac_s *priv = (FAR struct rm57_ethmac_s *)arg;

  net_lock();
  rm57emac_receive(priv);
  net_unlock();

  putreg32(EMAC_MACEOIVECTOR_C0RX, RM57_EMAC_MACEOIVECTOR);
  putreg32(EMAC_CTRL_C0CHEN(0), RM57_EMAC_CTRL_C0RXEN);
}

static void rm57emac_tx_work(FAR void *arg)
{
  FAR struct rm57_ethmac_s *priv = (FAR struct rm57_ethmac_s *)arg;

  net_lock();
  rm57emac_txdone(priv);
  net_unlock();

  putreg32(EMAC_MACEOIVECTOR_C0TX, RM57_EMAC_MACEOIVECTOR);
  putreg32(EMAC_CTRL_C0CHEN(0), RM57_EMAC_CTRL_C0TXEN);
}

static void rm57emac_misc_work(FAR void *arg)
{
  FAR struct rm57_ethmac_s *priv = (FAR struct rm57_ethmac_s *)arg;
  uint32_t macintstat;
  uint32_t linkintraw;

  macintstat = getreg32(RM57_EMAC_MACINTSTATMASKED);

  if ((macintstat & EMAC_MACINT_HOSTPEND) != 0)
    {
      uint32_t macstatus = getreg32(RM57_EMAC_MACSTATUS);
      uint32_t txerrcode = (macstatus & EMAC_MACSTATUS_TXERRCODE_MASK) >>
                           EMAC_MACSTATUS_TXERRCODE_SHIFT;

      nerr("ERROR: EMAC host error, MACSTATUS=%08" PRIx32
           " TXERRCODE=%" PRIu32 " TXERRCH=%" PRIu32
           " RXERRCODE=%" PRIu32 " RXERRCH=%" PRIu32
           " - resetting\n", macstatus, txerrcode,
           (macstatus & EMAC_MACSTATUS_TXERRCH_MASK) >>
             EMAC_MACSTATUS_TXERRCH_SHIFT,
           (macstatus & EMAC_MACSTATUS_RXERRCODE_MASK) >>
             EMAC_MACSTATUS_RXERRCODE_SHIFT,
           (macstatus & EMAC_MACSTATUS_RXERRCH_MASK) >>
             EMAC_MACSTATUS_RXERRCH_SHIFT);

      net_lock();
      rm57emac_ifdown(&priv->dev);
      rm57emac_ifup(&priv->dev);
      net_unlock();

      /* rm57emac_ifup() already re-armed C0MISCEN as part of its normal
       * setup sequence; still need to acknowledge the pulse that got us
       * here so the EMAC Control module's pulse generator for this
       * channel isn't left latched.
       */

      putreg32(EMAC_MACEOIVECTOR_C0MISC, RM57_EMAC_MACEOIVECTOR);
      return;
    }

  if ((macintstat & EMAC_MACINT_STATPEND) != 0)
    {
      net_lock();
      rm57emac_stats_update(priv);
      net_unlock();
    }

  linkintraw = getreg32(RM57_MDIO_LINKINTRAW);
  if ((linkintraw & MDIO_LINKINT_USERPHY0) != 0)
    {
      putreg32(MDIO_LINKINT_USERPHY0, RM57_MDIO_LINKINTMASKED);

      net_lock();
      rm57emac_linkupdate(priv);
      net_unlock();

#ifdef CONFIG_ARCH_PHY_INTERRUPT
      if (priv->phyhandler != NULL && priv->phynotify_armed)
        {
          priv->phynotify_armed = false;
          priv->phyhandler(RM57_REQ_EMACMISC, NULL, priv->phyarg);
        }
#endif
    }

  putreg32(EMAC_MACEOIVECTOR_C0MISC, RM57_EMAC_MACEOIVECTOR);
  putreg32(EMAC_CTRL_C0MISC_USERINT0EN | EMAC_CTRL_C0MISC_LINKINT0EN |
           EMAC_CTRL_C0MISC_HOSTPENDEN | EMAC_CTRL_C0MISC_STATPENDEN,
           RM57_EMAC_CTRL_C0MISCEN);
}

/****************************************************************************
 * Name: rm57emac_interrupt
 ****************************************************************************/

static int rm57emac_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct rm57_ethmac_s *priv = (FAR struct rm57_ethmac_s *)arg;

  switch (irq)
    {
      case RM57_REQ_EMACRX:
        putreg32(0, RM57_EMAC_CTRL_C0RXEN);
        if (work_available(&priv->rxwork))
          {
            work_queue(ETHWORK, &priv->rxwork, rm57emac_rx_work, priv, 0);
          }
        break;

      case RM57_REQ_EMACTX:
        putreg32(0, RM57_EMAC_CTRL_C0TXEN);
        if (work_available(&priv->txwork))
          {
            work_queue(ETHWORK, &priv->txwork, rm57emac_tx_work, priv, 0);
          }
        break;

      case RM57_REQ_EMACMISC:
        putreg32(0, RM57_EMAC_CTRL_C0MISCEN);
        if (work_available(&priv->irqwork))
          {
            work_queue(ETHWORK, &priv->irqwork, rm57emac_misc_work, priv,
                       0);
          }
        break;

      case RM57_REQ_EMACTHRESH:
      default:
        /* Not enabled by this driver (see rm57emac_configure()); mask
         * defensively and ack in case it ever fires.
         */

        putreg32(0, RM57_EMAC_CTRL_C0RXTHRESHEN);
        putreg32(EMAC_MACEOIVECTOR_C0RXTHRESH, RM57_EMAC_MACEOIVECTOR);
        break;
    }

  return OK;
}

/****************************************************************************
 * Name: rm57emac_txtimeout_work / rm57emac_txtimeout_expiry
 ****************************************************************************/

static void rm57emac_txtimeout_work(FAR void *arg)
{
  FAR struct rm57_ethmac_s *priv = (FAR struct rm57_ethmac_s *)arg;

  nerr("ERROR: EMAC TX timeout - resetting\n");

  /* Dump the collision counters before the reset below re-initialises the
   * MAC.  Non-zero late/excessive collisions mean the MAC is half duplex
   * against a full-duplex peer; all-zero points at the descriptor ring.
   * Otherwise these are only sampled on the rare STATPEND interrupt.
   */

  nerr("ERROR:   MACCONTROL=%08" PRIx32 " txused=%u/%u\n",
       getreg32(RM57_EMAC_MACCONTROL), (unsigned)priv->txused,
       (unsigned)RM57_EMAC_NTXDESC);
  nerr("ERROR:   coll=%" PRId32 " single=%" PRId32 " multi=%" PRId32
       " late=%" PRId32 " excess=%" PRId32 " carrier=%" PRId32
       " underrun=%" PRId32 "\n",
       getreg32(RM57_EMAC_TXCOLLISION), getreg32(RM57_EMAC_TXSINGLECOLL),
       getreg32(RM57_EMAC_TXMULTICOLL), getreg32(RM57_EMAC_TXLATECOLL),
       getreg32(RM57_EMAC_TXEXCESSIVECOLL),
       getreg32(RM57_EMAC_TXCARRIERSENSE), getreg32(RM57_EMAC_TXUNDERRUN));

  net_lock();
  NETDEV_TXTIMEOUTS(&priv->dev);
  rm57emac_ifdown(&priv->dev);
  rm57emac_ifup(&priv->dev);
  rm57emac_dopoll(priv);
  net_unlock();
}

static void rm57emac_txtimeout_expiry(wdparm_t arg)
{
  FAR struct rm57_ethmac_s *priv = (FAR struct rm57_ethmac_s *)arg;

  up_disable_irq(RM57_REQ_EMACTX);
  up_disable_irq(RM57_REQ_EMACRX);
  up_disable_irq(RM57_REQ_EMACMISC);

  if (work_available(&priv->timeoutwork))
    {
      work_queue(ETHWORK, &priv->timeoutwork, rm57emac_txtimeout_work, priv,
                0);
    }
}

/****************************************************************************
 * Name: rm57emac_pollwork / rm57emac_txavail
 ****************************************************************************/

static void rm57emac_pollwork(FAR void *arg)
{
  FAR struct rm57_ethmac_s *priv = (FAR struct rm57_ethmac_s *)arg;

  net_lock();
  if (priv->ifup)
    {
      rm57emac_dopoll(priv);
    }

  net_unlock();
}

static int rm57emac_txavail(FAR struct net_driver_s *dev)
{
  FAR struct rm57_ethmac_s *priv =
    (FAR struct rm57_ethmac_s *)dev->d_private;

  if (work_available(&priv->pollwork))
    {
      work_queue(ETHWORK, &priv->pollwork, rm57emac_pollwork, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: rm57emac_addmac / rm57emac_rmmac
 *
 * Description:
 *   Program the 64-bit MACHASH1/2 group-address hash table per
 *   SPNU562A Section 31.5.37: fold the 48-bit destination address into a
 *   6-bit hash via 8-way XOR of each bit position, then set/clear that
 *   bit.  mcrefcnt tracks how many joined addresses currently hash to
 *   each bit, since two different multicast addresses can collide.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_MCASTGROUP
static uint8_t rm57emac_hash(FAR const uint8_t *mac)
{
  uint8_t hash = 0;
  int bit;
  int byte;

  for (bit = 0; bit < 6; bit++)
    {
      uint8_t acc = 0;

      for (byte = 0; byte < 6; byte++)
        {
          acc ^= (mac[byte] >> bit) & 1;
        }

      hash |= (acc & 1) << bit;
    }

  return hash;
}

static void rm57emac_sethash(FAR struct rm57_ethmac_s *priv, uint8_t hash,
                             bool set)
{
  uintptr_t reg = (hash < 32) ? RM57_EMAC_MACHASH1 : RM57_EMAC_MACHASH2;
  uint32_t bit = 1u << (hash & 31);
  uint32_t regval = getreg32(reg);

  if (set)
    {
      regval |= bit;
    }
  else
    {
      regval &= ~bit;
    }

  putreg32(regval, reg);
}

static int rm57emac_addmac(FAR struct net_driver_s *dev,
                           FAR const uint8_t *mac)
{
  FAR struct rm57_ethmac_s *priv =
    (FAR struct rm57_ethmac_s *)dev->d_private;
  uint8_t hash = rm57emac_hash(mac);

  if (priv->mcrefcnt[hash] == 0)
    {
      rm57emac_sethash(priv, hash, true);
    }

  if (priv->mcrefcnt[hash] < UINT8_MAX)
    {
      priv->mcrefcnt[hash]++;
    }

  return OK;
}

static int rm57emac_rmmac(FAR struct net_driver_s *dev,
                          FAR const uint8_t *mac)
{
  FAR struct rm57_ethmac_s *priv =
    (FAR struct rm57_ethmac_s *)dev->d_private;
  uint8_t hash = rm57emac_hash(mac);

  if (priv->mcrefcnt[hash] > 0 && --priv->mcrefcnt[hash] == 0)
    {
      rm57emac_sethash(priv, hash, false);
    }

  return OK;
}
#endif /* CONFIG_NET_MCASTGROUP */

/****************************************************************************
 * Name: rm57emac_ioctl
 ****************************************************************************/

#ifdef CONFIG_NETDEV_IOCTL
static int rm57emac_ioctl(FAR struct net_driver_s *dev, int cmd,
                          unsigned long arg)
{
  int ret;

  switch (cmd)
    {
#ifdef CONFIG_NETDEV_PHY_IOCTL
      case SIOCGMIIPHY:
        {
          FAR struct rm57_ethmac_s *priv =
            (FAR struct rm57_ethmac_s *)dev->d_private;
          FAR struct mii_ioctl_data_s *req =
            (FAR struct mii_ioctl_data_s *)((uintptr_t)arg);

          req->phy_id = priv->phyaddr;
          ret = OK;
        }
        break;

      case SIOCGMIIREG:
        {
          FAR struct mii_ioctl_data_s *req =
            (FAR struct mii_ioctl_data_s *)((uintptr_t)arg);

          ret = rm57emac_phyread(req->phy_id, req->reg_num, &req->val_out);
        }
        break;

      case SIOCSMIIREG:
        {
          FAR struct mii_ioctl_data_s *req =
            (FAR struct mii_ioctl_data_s *)((uintptr_t)arg);

          ret = rm57emac_phywrite(req->phy_id, req->reg_num, req->val_in);
        }
        break;

#ifdef CONFIG_ARCH_PHY_INTERRUPT
      case SIOCMIINOTIFY:
        {
          FAR struct mii_ioctl_notify_s *req =
            (FAR struct mii_ioctl_notify_s *)((uintptr_t)arg);

          ret = phy_notify_subscribe(dev->d_ifname, req->pid, &req->event);
          if (ret == OK)
            {
              rm57_mdio_linkint(
                ((FAR struct rm57_ethmac_s *)dev->d_private)->phyaddr,
                true);
            }
        }
        break;
#endif
#endif /* CONFIG_NETDEV_PHY_IOCTL */

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}
#endif /* CONFIG_NETDEV_IOCTL */

/****************************************************************************
 * Name: rm57emac_phyread / rm57emac_phywrite
 ****************************************************************************/

static int rm57emac_phyread(uint16_t phyaddr, uint8_t regaddr,
                            FAR uint16_t *value)
{
  return rm57_mdio_read((uint8_t)phyaddr, regaddr, value);
}

static int rm57emac_phywrite(uint16_t phyaddr, uint8_t regaddr,
                             uint16_t value)
{
  return rm57_mdio_write((uint8_t)phyaddr, regaddr, value);
}

/****************************************************************************
 * Name: rm57emac_phyfind
 *
 * Description:
 *   Confirm a PHY answers at CONFIG_RM57_EMAC_PHYADDR (via MDIO_ALIVE)
 *   and, if a specific PHY model is selected in the drivers/net/Kconfig
 *   PHY choice, compare its ID - a mismatch only warns, since a wrong
 *   Kconfig selection shouldn't block an otherwise-working link.
 *
 ****************************************************************************/

static int rm57emac_phyfind(FAR struct rm57_ethmac_s *priv)
{
  uint32_t alive = 0;
  uint16_t id1;
  uint16_t id2;
  int i;

  priv->phyaddr = CONFIG_RM57_EMAC_PHYADDR;

  /* ALIVE only reflects a connected PHY once the MDIO module's
   * autodetect state machine has swept all 32 addresses at least once
   * since CONTROL.ENABLE was set - poll rather than sampling it once.
   */

  for (i = 0; i < RM57_EMAC_ALIVE_MAX_POLLS; i++)
    {
      alive = rm57_mdio_alive();
      if ((alive & MDIO_ALIVE_PHY(priv->phyaddr)) != 0)
        {
          break;
        }

      up_mdelay(RM57_EMAC_ALIVE_POLL_MS);
    }

  if ((alive & MDIO_ALIVE_PHY(priv->phyaddr)) == 0)
    {
      uint32_t control = getreg32(RM57_MDIO_CONTROL);

      nerr("ERROR: No PHY responding at address %d (MDIO_ALIVE=%08"
           PRIx32 ")\n", priv->phyaddr, alive);

      /* CONTROL.FAULT means the module drove the MDIO line and read
       * back something else, i.e. a genuine physical-layer problem
       * rather than simply nobody answering.
       */

      if ((control & MDIO_CONTROL_FAULT) != 0)
        {
          nerr("ERROR: MDIO physical layer fault (MDIO_CONTROL=%08"
               PRIx32 ")\n", control);
        }

      /* The background sweep is not the only way to reach a PHY: fall
       * back to explicit reads, the way HALCoGen's EMACHWInit() probes.
       * This distinguishes "the bus is dead" from "the PHY is at an
       * address we did not expect" - a wrong strap latch
       * shows up here as a hit at some other address.
       */

      for (i = 0; i < 32; i++)
        {
          uint16_t probe = 0xffff;

          if (rm57emac_phyread(i, MII_PHYID1, &probe) == OK &&
              probe != 0xffff && probe != 0x0000)
            {
              nerr("ERROR: but a PHY answered at address %d "
                   "(MII_PHYID1=%04x) - check CONFIG_RM57_EMAC_PHYADDR\n",
                   i, probe);
              break;
            }
        }

      return -ENODEV;
    }

  if (rm57emac_phyread(priv->phyaddr, MII_PHYID1, &id1) < 0 ||
      rm57emac_phyread(priv->phyaddr, MII_PHYID2, &id2) < 0)
    {
      nerr("ERROR: Failed to read PHY ID at address %d\n", priv->phyaddr);
      return -ENODEV;
    }

  ninfo("PHY ID: %04x:%04x\n", id1, id2);

#if defined(CONFIG_ETH0_PHY_DP83630)
  if (id1 != MII_PHYID1_DP83630 ||
      (id2 & ~MII_PHYID2_DP83630_REV_MASK) !=
      (MII_PHYID2_DP83630 & ~MII_PHYID2_DP83630_REV_MASK))
    {
      nwarn("WARNING: PHY ID %04x:%04x does not match configured "
            "DP83630 (%04x:%04x)\n", id1, id2, MII_PHYID1_DP83630,
            MII_PHYID2_DP83630);
    }
#endif

  return OK;
}

/****************************************************************************
 * Name: rm57emac_linkupdate
 *
 * Description:
 *   Read the negotiated (or forced) link state from the PHY and update
 *   netdev_carrier + MACCONTROL's FULLDUPLEX/RMIISPEED bits to match.
 *   Called both after the initial autonegotiation wait in
 *   rm57emac_phyinit() and from the LINKINT0 path in rm57emac_misc_work.
 *
 ****************************************************************************/

static void rm57emac_linkupdate(FAR struct rm57_ethmac_s *priv)
{
  FAR struct net_driver_s *dev = &priv->dev;
  uint16_t msr;
  uint32_t macctrl;

  if (rm57emac_phyread(priv->phyaddr, MII_MSR, &msr) < 0 ||
      (msr & MII_MSR_LINKSTATUS) == 0)
    {
      netdev_carrier_off(dev);
      return;
    }

#ifdef CONFIG_RM57_EMAC_AUTONEG
  {
    uint16_t adv = 0;
    uint16_t lpa = 0;
    int retry;

    /* Do not latch a result until autonegotiation has finished.  LINKSTATUS
     * can come up a moment before ANEGCOMPLETE, and the LINKINT0 handler
     * enters here at an arbitrary instant, so the ANAR/ANLPAR intersection
     * below could still read zero - which fell through to the terminal else
     * and quietly programmed 10 Mbit half duplex.  There is no periodic link
     * poll, so that would stand until the next link change.
     */

    for (retry = 0; retry < RM57_EMAC_ANEG_LATCH_POLLS; retry++)
      {
        if ((msr & MII_MSR_ANEGCOMPLETE) != 0)
          {
            break;
          }

        up_mdelay(1);

        if (rm57emac_phyread(priv->phyaddr, MII_MSR, &msr) < 0)
          {
            nerr("ERROR: MII_MSR read failed while waiting for autoneg\n");
            netdev_carrier_off(dev);
            return;
          }
      }

    /* A failed MDIO read leaves adv/lpa at 0, indistinguishable from
     * "nothing negotiated".  Bail out instead of programming a duplex from
     * it; MACCONTROL keeps whatever it already had.
     */

    if (rm57emac_phyread(priv->phyaddr, MII_ADVERTISE, &adv) < 0 ||
        rm57emac_phyread(priv->phyaddr, MII_LPA, &lpa) < 0)
      {
        nerr("ERROR: PHY ANAR/ANLPAR read failed - link unchanged\n");
        netdev_carrier_off(dev);
        return;
      }

    adv &= lpa;

    /* An incomplete negotiation means the partner is forced, and 802.3
     * parallel detection resolves to half duplex - what the fallback below
     * selects anyway.  Say so, since arriving there by accident is exactly
     * what this function used to hide.
     */

    if ((msr & MII_MSR_ANEGCOMPLETE) == 0)
      {
        nwarn("WARNING: Autonegotiation incomplete (MSR=%04x) - "
              "falling back to half duplex\n", msr);
      }

    if ((adv & MII_ADVERTISE_100BASETXFULL) != 0)
      {
        priv->mbps100 = true;
        priv->fduplex = true;
      }
    else if ((adv & MII_ADVERTISE_100BASETXHALF) != 0)
      {
        priv->mbps100 = true;
        priv->fduplex = false;
      }
    else if ((adv & MII_ADVERTISE_10BASETXFULL) != 0)
      {
        priv->mbps100 = false;
        priv->fduplex = true;
      }
    else
      {
        priv->mbps100 = false;
        priv->fduplex = false;
      }
  }
#else
#  ifdef CONFIG_RM57_EMAC_ETH100MBPS
  priv->mbps100 = true;
#  else
  priv->mbps100 = false;
#  endif
#  ifdef CONFIG_RM57_EMAC_ETHFD
  priv->fduplex = true;
#  else
  priv->fduplex = false;
#  endif
#endif

  macctrl = getreg32(RM57_EMAC_MACCONTROL);
  macctrl &= ~(EMAC_MACCONTROL_FULLDUPLEX | EMAC_MACCONTROL_RMIISPEED);

  if (priv->fduplex)
    {
      macctrl |= EMAC_MACCONTROL_FULLDUPLEX;
    }

  if (priv->mbps100)
    {
      macctrl |= EMAC_MACCONTROL_RMIISPEED;
    }

  putreg32(macctrl, RM57_EMAC_MACCONTROL);

  /* Warning level on purpose: the resolved duplex is the most useful fact
   * about the link, and ninfo is compiled out of most builds.  MACCONTROL is
   * read back so this reports what the hardware holds, not what was written.
   */

  nwarn("Link up: %s Mbps %s duplex (MACCONTROL=%08" PRIx32 ")\n",
        priv->mbps100 ? "100" : "10", priv->fduplex ? "full" : "half",
        getreg32(RM57_EMAC_MACCONTROL));

  netdev_carrier_on(dev);
}

/****************************************************************************
 * Name: rm57emac_phyinit
 *
 * Description:
 *   Reset the PHY, start autonegotiation (or force the configured
 *   speed/duplex), wait for it to complete, then call
 *   rm57emac_linkupdate() to program MACCONTROL and raise the carrier.
 *   Runs in thread context (from rm57emac_ifup()), so it may block.
 *
 ****************************************************************************/

static int rm57emac_phyinit(FAR struct rm57_ethmac_s *priv)
{
  uint16_t mcr;
  int ret;
  int i;

  ret = rm57emac_phyfind(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Soft reset and wait for it to self-clear */

  rm57emac_phywrite(priv->phyaddr, MII_MCR, MII_MCR_RESET);

  for (i = 0; i < 100; i++)
    {
      uint16_t mcrval = 0;

      up_mdelay(1);
      if (rm57emac_phyread(priv->phyaddr, MII_MCR, &mcrval) == OK &&
          (mcrval & MII_MCR_RESET) == 0)
        {
          break;
        }
    }

#ifdef CONFIG_RM57_EMAC_AUTONEG
  rm57emac_phywrite(priv->phyaddr, MII_ADVERTISE,
                    MII_ADVERTISE_CSMA | MII_ADVERTISE_10BASETXHALF |
                    MII_ADVERTISE_10BASETXFULL |
                    MII_ADVERTISE_100BASETXHALF |
                    MII_ADVERTISE_100BASETXFULL);

  mcr = MII_MCR_ANENABLE | MII_MCR_ANRESTART;
  rm57emac_phywrite(priv->phyaddr, MII_MCR, mcr);

  for (i = 0; i < RM57_EMAC_ANEG_MAX_POLLS; i++)
    {
      uint16_t msr = 0;

      up_mdelay(RM57_EMAC_ANEG_POLL_MS);
      if (rm57emac_phyread(priv->phyaddr, MII_MSR, &msr) == OK &&
          (msr & MII_MSR_ANEGCOMPLETE) != 0)
        {
          break;
        }
    }

  if (i >= RM57_EMAC_ANEG_MAX_POLLS)
    {
      nwarn("WARNING: Autonegotiation did not complete\n");
    }
#else
  mcr = 0;
#  ifdef CONFIG_RM57_EMAC_ETHFD
  mcr |= MII_MCR_FULLDPLX;
#  endif
#  ifdef CONFIG_RM57_EMAC_ETH100MBPS
  mcr |= MII_MCR_SPEED100;
#  endif

  rm57emac_phywrite(priv->phyaddr, MII_MCR, mcr);
#endif

  rm57emac_linkupdate(priv);

  /* Always enable the MDIO link-change interrupt for this driver's own
   * netdev_carrier_on/off tracking in rm57emac_misc_work() - independent
   * of whether a phy_notify subscriber (CONFIG_ARCH_PHY_INTERRUPT) is
   * also attached; see the phynotify_armed field comment.
   */

  rm57_mdio_linkint(priv->phyaddr, true);

  return OK;
}

/****************************************************************************
 * Name: rm57emac_setmacaddr
 *
 * Description:
 *   Program one of the 8 per-channel MAC address match registers, using
 *   the MACINDEX/MACADDRHI/MACADDRLO write order of TI's HALCoGen
 *   EMACMACAddrSet() (SPNU562A Section 31.5.43-31.5.45).
 *
 *   The byte order is NOT HALCoGen's, though: that function documents
 *   its argument as "array[0] shall be the LSB of the MAC address", i.e.
 *   an address stored backwards relative to wire order, which is not
 *   what dev->d_mac holds.  The register fields are numbered by position
 *   in the 48-bit address, not by position on the wire - Section 31.5.44
 *   pins this down by placing the group bit, which is the low bit of the
 *   first byte transmitted, in MACADDR5 (bit 40).  So MACADDR5, the low
 *   byte of MACADDRHI, takes mac[0] and the fields run down to MACADDR0,
 *   the high byte of MACADDRLO, taking mac[5].  Feeding a wire-order
 *   address through HALCoGen's shifts instead programs the reversed
 *   address: broadcast keeps working (RXBROADEN bypasses this filter)
 *   while every unicast frame is counted in RXFILTERED and dropped.
 *
 ****************************************************************************/

static void rm57emac_setmacaddr(uint32_t channel, FAR const uint8_t *mac,
                                uint32_t matchfilt)
{
  putreg32(channel, RM57_EMAC_MACINDEX);

  putreg32((uint32_t)mac[0] | ((uint32_t)mac[1] << 8) |
           ((uint32_t)mac[2] << 16) | ((uint32_t)mac[3] << 24),
           RM57_EMAC_MACADDRHI);

  putreg32((uint32_t)mac[4] | ((uint32_t)mac[5] << 8) | matchfilt |
           EMAC_MACADDRLO_CHANNEL(channel),
           RM57_EMAC_MACADDRLO);
}

static void rm57emac_macaddress(FAR struct rm57_ethmac_s *priv)
{
  FAR const uint8_t *mac = priv->dev.d_mac.ether.ether_addr_octet;
  uint32_t ch;

  /* Same field numbering as MACADDRHI/LO above (Section 31.5.41-31.5.42) */

  putreg32((uint32_t)mac[0] | ((uint32_t)mac[1] << 8) |
           ((uint32_t)mac[2] << 16) | ((uint32_t)mac[3] << 24),
           RM57_EMAC_MACSRCADDRHI);
  putreg32((uint32_t)mac[4] | ((uint32_t)mac[5] << 8),
           RM57_EMAC_MACSRCADDRLO);

  /* Program the same address into all 8 slots, whether the channel is
   * used or not (SPNU562A Section 31.2.16.4 step 5: "Be sure to program
   * all eight MAC address registers ... Duplicate the same MAC address
   * across all unused channels"), but mark only channel 0 - the one this
   * driver receives on - VALID.  Section 31.5.43 is explicit that VALID
   * "should be cleared to 0 for unused address channels", and CHANNEL
   * "determines which receive channel a valid address match will be
   * transferred to": leaving all eight valid makes an incoming unicast
   * frame match every slot, and a slot other than 0 then steers it to a
   * channel that has neither descriptors nor an RXUNICASTSET bit, so the
   * frame is dropped.  Broadcast is unaffected - RXBROADEN/RXBROADCH
   * route it without consulting the address filter - which is why the
   * symptom was DHCP and ARP working while every unicast frame (ping
   * replies, DNS answers) vanished.
   */

  for (ch = 0; ch < 8; ch++)
    {
      rm57emac_setmacaddr(ch, mac,
                          ch == 0 ?
                          EMAC_MACADDRLO_VALID | EMAC_MACADDRLO_MATCHFILT :
                          0);
    }
}

/****************************************************************************
 * Name: rm57emac_txdescinit / rm57emac_rxdescinit
 ****************************************************************************/

static void rm57emac_txdescinit(FAR struct rm57_ethmac_s *priv)
{
  int i;

  sq_init(&priv->txfreeb);

  for (i = 0; i < RM57_EMAC_NTXBUF; i++)
    {
      sq_addlast((FAR sq_entry_t *)g_rm57emac_txbuf[i], &priv->txfreeb);
    }

  for (i = 0; i < RM57_EMAC_NTXDESC; i++)
    {
      memset((FAR void *)&RM57_EMAC_TXRING[i], 0,
             sizeof(struct rm57_emac_desc_s));
    }

  priv->txnext    = &RM57_EMAC_TXRING[0];
  priv->txtail    = NULL;
  priv->txreclaim = &RM57_EMAC_TXRING[0];
  priv->txused    = 0;
}

static void rm57emac_rxdescinit(FAR struct rm57_ethmac_s *priv)
{
  int i;

  /* Drop cached lines covering the RX pool before the DMA takes ownership.
   * These buffers are .bss, and on an ifdown/ifup cycle they still hold the
   * previous session's data, so dirty lines can exist here and would be
   * evicted over the first frames received.
   */

  up_invalidate_dcache((uintptr_t)g_rm57emac_rxbuf,
                       (uintptr_t)g_rm57emac_rxbuf +
                       sizeof(g_rm57emac_rxbuf));

  for (i = 0; i < RM57_EMAC_NRXDESC; i++)
    {
      FAR struct rm57_emac_desc_s *desc = &RM57_EMAC_RXRING[i];

      desc->next        = NULL;
      desc->buffer       = g_rm57emac_rxbuf[i];
      desc->bufoff       = 0;
      desc->buflen       = RM57_EMAC_BUFSIZE;
      desc->flags_pktlen = EMAC_DESC_OWNER;
    }

  /* Pre-link the whole ring so the DMA engine has RM57_EMAC_NRXDESC
   * buffers available from the start; the last descriptor's next stays
   * NULL (it is the initial tail).
   */

  for (i = 0; i < RM57_EMAC_NRXDESC - 1; i++)
    {
      RM57_EMAC_RXRING[i].next = &RM57_EMAC_RXRING[i + 1];
    }

  priv->rxnext = &RM57_EMAC_RXRING[0];
  priv->rxtail = &RM57_EMAC_RXRING[RM57_EMAC_NRXDESC - 1];

  putreg32((uint32_t)(uintptr_t)&RM57_EMAC_RXRING[0], RM57_EMAC_RXHDP(0));
}

/****************************************************************************
 * Name: rm57emac_reset
 *
 * Description:
 *   Soft-reset the EMAC module and poll until the reset self-clears
 *   (SPNU562A Section 31.2.15.1).  The EMAC Control and MDIO modules
 *   have no software reset of their own on this device.
 *
 ****************************************************************************/

#ifdef CONFIG_RM57_EMAC_REGDEBUG

/****************************************************************************
 * Name: rm57emac_regdump
 *
 * Description:
 *   Dump the registers that separate the usual EMAC bring-up failures
 *   from each other, so a bad board can be told apart from a bad
 *   configuration without a debugger.
 *
 *   MDIO_REVID reading its reset value proves the module is powered and
 *   clocked (PENA, the PCR2 power-down clears and VCLK3 are all good),
 *   which narrows a silent MDIO bus down to the pin mux or the PHY.
 *   PINMMR87 selects which ball the MDIO input is taken from and
 *   PINMMR160 selects MII vs RMII; both are easy to get wrong and
 *   neither reports an error when it is.  SYSPC1/ECPCNTL matter on
 *   boards that clock their PHY from the MCU's ECLK1 terminal.
 *
 ****************************************************************************/

static void rm57emac_regdump(void)
{
  ninfo("MDIO:  REVID=%08" PRIx32 " CONTROL=%08" PRIx32
        " ALIVE=%08" PRIx32 " LINK=%08" PRIx32 "\n",
        getreg32(RM57_MDIO_REVID), getreg32(RM57_MDIO_CONTROL),
        getreg32(RM57_MDIO_ALIVE), getreg32(RM57_MDIO_LINK));

  ninfo("EMAC:  CTRL_REVID=%08" PRIx32 " MACCONTROL=%08" PRIx32 "\n",
        getreg32(RM57_EMAC_CTRL_REVID), getreg32(RM57_EMAC_MACCONTROL));

  ninfo("IOMM:  PINMMR87=%08" PRIx32 " PINMMR160=%08" PRIx32 "\n",
        getreg32(RM57_IOMM_PINMMR(87)), getreg32(RM57_IOMM_PINMMR(160)));

  ninfo("SYS:   SYSPC1=%08" PRIx32 " ECPCNTL=%08" PRIx32 "\n",
        getreg32(RM57_SYS_SYSPC1), getreg32(RM57_SYS_ECPCNTL));
}

#else
#  define rm57emac_regdump()
#endif

static void rm57emac_reset(void)
{
  int i;

  /* Reset the EMAC control module first, then the EMAC itself - the
   * order HALCoGen's EMACInit() uses.  The control module reset also
   * clears the CPPI descriptor RAM, so it has to happen
   * before rm57emac_txdescinit()/rm57emac_rxdescinit() populate it.
   */

  putreg32(EMAC_CTRL_SOFTRESET_RESET, RM57_EMAC_CTRL_SOFTRESET);

  for (i = 0; i < 1000; i++)
    {
      if ((getreg32(RM57_EMAC_CTRL_SOFTRESET) &
           EMAC_CTRL_SOFTRESET_RESET) == 0)
        {
          break;
        }
    }

  putreg32(EMAC_SOFTRESET_RESET, RM57_EMAC_SOFTRESET);

  for (i = 0; i < 1000; i++)
    {
      if ((getreg32(RM57_EMAC_SOFTRESET) & EMAC_SOFTRESET_RESET) == 0)
        {
          break;
        }
    }
}

/****************************************************************************
 * Name: rm57emac_configure
 *
 * Description:
 *   The 17-step EMAC bring-up sequence from SPNU562A Section 31.2.16.4,
 *   up to (but not including) enabling TX/RX DMA and GMIIEN - those are
 *   left to rm57emac_ifup() so descriptor rings are populated first.
 *
 ****************************************************************************/

static int rm57emac_configure(FAR struct rm57_ethmac_s *priv)
{
  int ch;

  /* Steps 1-2: mask all EMAC Control interrupts, clear MAC/RX/TX control */

  putreg32(0, RM57_EMAC_CTRL_C0RXTHRESHEN);
  putreg32(0, RM57_EMAC_CTRL_C0RXEN);
  putreg32(0, RM57_EMAC_CTRL_C0TXEN);
  putreg32(0, RM57_EMAC_CTRL_C0MISCEN);

  putreg32(0, RM57_EMAC_MACCONTROL);
  putreg32(0, RM57_EMAC_RXCONTROL);
  putreg32(0, RM57_EMAC_TXCONTROL);

  /* Step 3: clear all head descriptor pointers */

  for (ch = 0; ch < 8; ch++)
    {
      putreg32(0, RM57_EMAC_RXHDP(ch));
      putreg32(0, RM57_EMAC_TXHDP(ch));
    }

  /* Step 4: statistics registers - cleared as a side effect of
   * rm57emac_reset() above (soft reset resets them to default).
   */

  /* Step 5: MAC address on all 8 channels */

  rm57emac_macaddress(priv);

  /* Step 6: flow control thresholds - not used by this driver */

  /* Step 7: no multicast addresses joined yet */

  putreg32(0, RM57_EMAC_MACHASH1);
  putreg32(0, RM57_EMAC_MACHASH2);

  /* Step 8: no leading buffer offset */

  putreg32(0, RM57_EMAC_RXBUFFEROFFSET);

  /* Cap accepted frame size at the actual RX buffer size so a
   * misbehaving or oversized frame can never overrun a fixed RX buffer.
   */

  putreg32(RM57_EMAC_BUFSIZE, RM57_EMAC_RXMAXLEN);

  /* Step 9: enable unicast reception on channel 0 only */

  putreg32(0xff, RM57_EMAC_RXUNICASTCLEAR);
  putreg32(EMAC_RXUNICAST_CH(0), RM57_EMAC_RXUNICASTSET);

  /* Step 10: broadcast + (initially disabled) multicast on channel 0.
   * Under CONFIG_RM57_EMAC_PROMISC, RXCAFEN additionally copies every
   * frame that fails the address filter to the promiscuous channel.
   * Those arrive with EMAC_DESC_RX_NOMATCH set, which rm57emac_receive()
   * deliberately does not treat as an error.
   */

  putreg32(EMAC_RXMBPENABLE_RXBROADEN |
           EMAC_RXMBPENABLE_RXBROADCH(0) |
#ifdef CONFIG_RM57_EMAC_PROMISC
           EMAC_RXMBPENABLE_RXCAFEN |
           EMAC_RXMBPENABLE_RXPROMCH(0) |
#endif
#ifdef CONFIG_NET_MCASTGROUP
           EMAC_RXMBPENABLE_RXMULTEN |
#endif
           EMAC_RXMBPENABLE_RXMULTCH(0),
           RM57_EMAC_RXMBPENABLE);

  /* Step 11: MACCONTROL (GMIIEN deliberately left clear here; set by
   * rm57emac_ifup() only after the descriptor rings are ready).
   * RMIISPEED/FULLDUPLEX are set later by rm57emac_linkupdate().
   */

#ifdef CONFIG_RM57_EMAC_LOOPBACK
  putreg32(EMAC_MACCONTROL_LOOPBACK, RM57_EMAC_MACCONTROL);
#endif

  /* Steps 12-13: clear then arm TX/RX channel 0 interrupt masks, plus
   * HOSTPEND/STATPEND at the MAC level.
   */

  putreg32(0xffffffff, RM57_EMAC_RXINTMASKCLEAR);
  putreg32(0xffffffff, RM57_EMAC_TXINTMASKCLEAR);
  putreg32(EMAC_RXCH(0), RM57_EMAC_RXINTMASKSET);
  putreg32(EMAC_TXCH(0), RM57_EMAC_TXINTMASKSET);

  putreg32(0xffffffff, RM57_EMAC_MACINTMASKCLEAR);
  putreg32(EMAC_MACINT_STATPEND | EMAC_MACINT_HOSTPEND,
           RM57_EMAC_MACINTMASKSET);

  return OK;
}

/****************************************************************************
 * Name: rm57emac_ifup
 ****************************************************************************/

static int rm57emac_ifup(FAR struct net_driver_s *dev)
{
  FAR struct rm57_ethmac_s *priv =
    (FAR struct rm57_ethmac_s *)dev->d_private;
  int ret;

  rm57emac_reset();

  ret = rm57_mdio_initialize(BOARD_VCLK3_FREQUENCY,
                             CONFIG_RM57_EMAC_MDIO_FREQ);
  if (ret < 0)
    {
      nerr("ERROR: rm57_mdio_initialize failed: %d\n", ret);
      return ret;
    }

  rm57emac_regdump();

  ret = rm57emac_configure(priv);
  if (ret < 0)
    {
      return ret;
    }

  rm57emac_txdescinit(priv);
  rm57emac_rxdescinit(priv);

  /* Attach and enable the four EMAC VIM interrupts before releasing the
   * RX/TX DMA engines, matching this port's convention (see rm57_dma.h)
   * of attaching from ifup rather than early boot, to avoid an ordering
   * hazard with up_irqinitialize().
   */

  irq_attach(RM57_REQ_EMACRX, rm57emac_interrupt, priv);
  irq_attach(RM57_REQ_EMACTX, rm57emac_interrupt, priv);
  irq_attach(RM57_REQ_EMACMISC, rm57emac_interrupt, priv);
  irq_attach(RM57_REQ_EMACTHRESH, rm57emac_interrupt, priv);

  up_enable_irq(RM57_REQ_EMACRX);
  up_enable_irq(RM57_REQ_EMACTX);
  up_enable_irq(RM57_REQ_EMACMISC);

  putreg32(EMAC_CTRL_C0CHEN(0), RM57_EMAC_CTRL_C0RXEN);
  putreg32(EMAC_CTRL_C0CHEN(0), RM57_EMAC_CTRL_C0TXEN);
  putreg32(EMAC_CTRL_C0MISC_LINKINT0EN | EMAC_CTRL_C0MISC_HOSTPENDEN |
           EMAC_CTRL_C0MISC_STATPENDEN, RM57_EMAC_CTRL_C0MISCEN);

  putreg32(EMAC_RXCONTROL_RXEN, RM57_EMAC_RXCONTROL);
  putreg32(EMAC_TXCONTROL_TXEN, RM57_EMAC_TXCONTROL);

  putreg32(getreg32(RM57_EMAC_MACCONTROL) | EMAC_MACCONTROL_GMIIEN,
           RM57_EMAC_MACCONTROL);

  priv->ifup = true;

#ifdef CONFIG_RM57_EMAC_LOOPBACK
  /* Loopback needs no PHY - bring the carrier up unconditionally so the
   * ring/interrupt path can be exercised standalone.
   */

  netdev_carrier_on(dev);
#else
  ret = rm57emac_phyinit(priv);
  if (ret < 0)
    {
      nerr("ERROR: rm57emac_phyinit failed: %d\n", ret);

      /* Leave the MAC running - a PHY that shows up later (hot-plugged
       * cable, slow-to-enumerate PHY) will be picked up via LINKINT0.
       */
    }
#endif

  return OK;
}

/****************************************************************************
 * Name: rm57emac_ifdown
 ****************************************************************************/

static int rm57emac_ifdown(FAR struct net_driver_s *dev)
{
  FAR struct rm57_ethmac_s *priv =
    (FAR struct rm57_ethmac_s *)dev->d_private;
  irqstate_t flags;

  flags = enter_critical_section();

  up_disable_irq(RM57_REQ_EMACRX);
  up_disable_irq(RM57_REQ_EMACTX);
  up_disable_irq(RM57_REQ_EMACMISC);
  up_disable_irq(RM57_REQ_EMACTHRESH);

  irq_detach(RM57_REQ_EMACRX);
  irq_detach(RM57_REQ_EMACTX);
  irq_detach(RM57_REQ_EMACMISC);
  irq_detach(RM57_REQ_EMACTHRESH);

  wd_cancel(&priv->txtimeout);

  putreg32(0, RM57_EMAC_MACCONTROL);
  putreg32(0, RM57_EMAC_RXCONTROL);
  putreg32(0, RM57_EMAC_TXCONTROL);

  rm57emac_reset();
  rm57_mdio_shutdown();

  priv->ifup = false;

  leave_critical_section(flags);

  netdev_carrier_off(dev);

  return OK;
}

/****************************************************************************
 * Name: rm57emac_phy_enable
 *
 * Description:
 *   phy_enable_t callback returned by arch_phy_irq(): flips the
 *   phynotify_armed software gate rm57emac_misc_work() checks before
 *   invoking the phy_notify handler.  Does not touch the shared MDIO
 *   LINKINTENB bit - see the phynotify_armed field comment.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_PHY_INTERRUPT
static void rm57emac_phy_enable(bool enable)
{
  g_rm57emac.phynotify_armed = enable;
}
#endif

/****************************************************************************
 * Name: arch_phy_irq
 *
 * Description:
 *   Chip-level PHY interrupt registration (see include/nuttx/arch.h).
 *   The RM57L843's PHY link-change interrupt is generated internally by
 *   the MDIO module (LINKINT0, routed through C0_MISC_PULSE) rather than
 *   a board GPIO, so this is implemented here rather than at board
 *   level - matching how tm4c_ethernet.c handles an internal PHY.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_PHY_INTERRUPT
int arch_phy_irq(FAR const char *intf, xcpt_t handler, FAR void *arg,
                 FAR phy_enable_t *enable)
{
  FAR struct rm57_ethmac_s *priv = &g_rm57emac;
  irqstate_t flags;

  DEBUGASSERT(intf != NULL);

  if (strcmp(intf, priv->dev.d_ifname) != 0)
    {
      nerr("ERROR: Unsupported interface: %s\n", intf);
      return -EINVAL;
    }

  flags = enter_critical_section();

  priv->phyhandler      = handler;
  priv->phyarg          = arg;
  priv->phynotify_armed = false;

  if (enable != NULL)
    {
      *enable = handler != NULL ? rm57emac_phy_enable : NULL;
    }

  leave_critical_section(flags);
  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rm57_ethinitialize
 ****************************************************************************/

int rm57_ethinitialize(int intf, FAR const uint8_t *macaddr)
{
  FAR struct rm57_ethmac_s *priv = &g_rm57emac;
  static const uint8_t default_mac[6] =
  {
    0x02, 0x00, 0x00, 0x00, 0x00, 0x01
  };

  UNUSED(intf);

  memset(priv, 0, sizeof(struct rm57_ethmac_s));

  memcpy(priv->dev.d_mac.ether.ether_addr_octet,
         macaddr != NULL ? macaddr : default_mac, 6);

  priv->dev.d_ifup    = rm57emac_ifup;
  priv->dev.d_ifdown  = rm57emac_ifdown;
  priv->dev.d_txavail = rm57emac_txavail;
#ifdef CONFIG_NET_MCASTGROUP
  priv->dev.d_addmac  = rm57emac_addmac;
  priv->dev.d_rmmac   = rm57emac_rmmac;
#endif
#ifdef CONFIG_NETDEV_IOCTL
  priv->dev.d_ioctl   = rm57emac_ioctl;
#endif
  priv->dev.d_private = priv;

  return netdev_register(&priv->dev, NET_LL_ETHERNET);
}

#endif /* CONFIG_RM57_EMAC */
