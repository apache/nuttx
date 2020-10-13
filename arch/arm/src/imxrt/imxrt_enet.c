/****************************************************************************
 * arch/arm/src/imxrt/imxrt_enet.c
 *
 *   Copyright (C) 2018-2019 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
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
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include <arpa/inet.h>

#include <nuttx/wdog.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/signal.h>
#include <nuttx/net/mii.h>
#include <nuttx/net/arp.h>
#include <nuttx/net/phy.h>
#include <nuttx/net/netdev.h>

#ifdef CONFIG_NET_PKT
#  include <nuttx/net/pkt.h>
#endif

#include "arm_arch.h"
#include "chip.h"
#include "imxrt_config.h"
#include "hardware/imxrt_enet.h"
#include "hardware/imxrt_ccm.h"
#include "hardware/imxrt_pinmux.h"
#include "imxrt_periphclks.h"
#include "imxrt_gpio.h"
#include "imxrt_enet.h"

#ifdef CONFIG_IMXRT_ENET

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* If processing is not done at the interrupt level, then work queue support
 * is required.
 */

#if !defined(CONFIG_SCHED_WORKQUEUE)
#  error Work queue support is required
#else

  /* Select work queue.  Always use the LP work queue if available.  If not,
   * then LPWORK will re-direct to the HP work queue.
   *
   * NOTE:  However, the network should NEVER run on the high priority work
   * queue!  That queue is intended only to service short back end interrupt
   * processing that never suspends.  Suspending the high priority work queue
   * may bring the system to its knees!
   */

#  define ETHWORK LPWORK
#endif

/* CONFIG_IMXRT_ENET_NETHIFS determines the number of physical interfaces
 * that will be supported.
 */

#if CONFIG_IMXRT_ENET_NETHIFS != 1
#  error "CONFIG_IMXRT_ENET_NETHIFS must be one for now"
#endif

#if CONFIG_IMXRT_ENET_NTXBUFFERS < 1
#  error "Need at least one TX buffer"
#endif

#if CONFIG_IMXRT_ENET_NRXBUFFERS < 1
#  error "Need at least one RX buffer"
#endif

#define NENET_NBUFFERS \
  (CONFIG_IMXRT_ENET_NTXBUFFERS + CONFIG_IMXRT_ENET_NRXBUFFERS)

/* Normally you would clean the cache after writing new values to the DMA
 * memory so assure that the dirty cache lines are flushed to memory
 * before the DMA occurs.  And you would invalid the cache after a data is
 * received via DMA so that you fetch the actual content of the data from
 * the cache.
 *
 * These conditions are not fully supported here.  If the write-throuch
 * D-Cache is enabled, however, then many of these issues go away:  The
 * cache clean operation does nothing (because there are not dirty cache
 * lines) and the cache invalid operation is innocuous (because there are
 * never dirty cache lines to be lost; valid data will always be reloaded).
 *
 * At present, we simply insist that write through cache be enabled.
 */

#if defined(CONFIG_ARMV7M_DCACHE) && !defined(CONFIG_ARMV7M_DCACHE_WRITETHROUGH)
#  error Write back D-Cache not yet supported
#endif

/* TX poll delay = 1 seconds. CLK_TCK is the number of clock ticks per
 * second.
 */

#define IMXRT_WDDELAY     (1 * CLK_TCK)

/* Align assuming that the D-Cache is enabled (probably 32-bytes).
 *
 * REVISIT: The size of descriptors and buffers must also be in even units
 * of the cache line size  That is because the operations to clean and
 * invalidate the cache will operate on a full 32-byte cache line.  If
 * CONFIG_ENET_ENHANCEDBD is selected, then the size of the descriptor is
 * 32-bytes (and probably already the correct size for the cache line);
 * otherwise, the size of the descriptors much smaller, only 8 bytes.
 */

#define ENET_ALIGN        ARMV7M_DCACHE_LINESIZE
#define ENET_ALIGN_MASK   (ENET_ALIGN - 1)
#define ENET_ALIGN_UP(n)  (((n) + ENET_ALIGN_MASK) & ~ENET_ALIGN_MASK)

/* TX timeout = 1 minute */

#define IMXRT_TXTIMEOUT   (60 * CLK_TCK)
#define MII_MAXPOLLS      (0x1ffff)
#define LINK_WAITUS       (500 * 1000)
#define LINK_NLOOPS       (10)

/* PHY definitions.
 *
 * The selected PHY must be selected from the drivers/net/Kconfig PHY menu.
 * A description of the PHY must be provided here.  That description must
 * include:
 *
 * 1. BOARD_PHY_NAME: A PHY name string (for debug output),
 * 2. BOARD_PHYID1 and BOARD_PHYID2: The PHYID1 and PHYID2 values (from
 *    include/nuttx/net/mii.h)
 * 3. BOARD_PHY_STATUS:  The address of the status register to use when
 *    querying link status (from include/nuttx/net/mii.h)
 * 4. BOARD_PHY_ADDRThe PHY broadcast address of 0 is selected.  This
 *    should be fine as long as there is only a single PHY.
 * 5. BOARD_PHY_10BASET:  A macro that can convert the status register
 *    value into a boolean: true=10Base-T, false=Not 10Base-T
 * 6. BOARD_PHY_100BASET:  A macro that can convert the status register
 *    value into a boolean: true=100Base-T, false=Not 100Base-T
 * 7. BOARD_PHY_ISDUPLEX:  A macro that can convert the status register
 *    value into a boolean: true=duplex mode, false=half-duplex mode
 *
 * The imxrt1050-evk board uses a KSZ8081 PHY
 * The Versiboard2 uses a LAN8720 PHY
 *
 * ...and further PHY descriptions here.
 */

#if defined(CONFIG_ETH0_PHY_KSZ8081)
#  define BOARD_PHY_NAME        "KSZ8081"
#  define BOARD_PHYID1          MII_PHYID1_KSZ8081
#  define BOARD_PHYID2          MII_PHYID2_KSZ8081
#  define BOARD_PHY_STATUS      MII_KSZ8081_PHYCTRL1
#  define BOARD_PHY_ADDR        (0)
#  define BOARD_PHY_10BASET(s)  (((s) & MII_PHYCTRL1_MODE_10HDX) != 0)
#  define BOARD_PHY_100BASET(s) (((s) & MII_PHYCTRL1_MODE_100HDX) != 0)
#  define BOARD_PHY_ISDUPLEX(s) (((s) & MII_PHYCTRL1_MODE_DUPLEX) != 0)
#elif defined(CONFIG_ETH0_PHY_LAN8720)
#  define BOARD_PHY_NAME        "LAN8720"
#  define BOARD_PHYID1          MII_PHYID1_LAN8720
#  define BOARD_PHYID2          MII_PHYID2_LAN8720
#  define BOARD_PHY_STATUS      MII_LAN8720_SCSR
#  define BOARD_PHY_ADDR        (1)
#  define BOARD_PHY_10BASET(s)  (((s)&MII_LAN8720_SPSCR_10MBPS) != 0)
#  define BOARD_PHY_100BASET(s) (((s)&MII_LAN8720_SPSCR_100MBPS) != 0)
#  define BOARD_PHY_ISDUPLEX(s) (((s)&MII_LAN8720_SPSCR_DUPLEX) != 0)
#else
#  error "Unrecognized or missing PHY selection"
#endif

/* Estimate the MII_SPEED in order to get an MDC close to 2.5MHz,
 * based on the internal module (ENET) clock:
 *
 *   MII_SPEED = ENET_FREQ/5000000 -1
 *
 * For example, if ENET_FREQ_MHZ=120 (MHz):
 *
 *   MII_SPEED = 120000000/5000000 -1
 *             = 23
 */

#define IMXRT_MII_SPEED  0x38 /* 100Mbs. Revisit and remove hardcoded value */
#if IMXRT_MII_SPEED > 63
#  error "IMXRT_MII_SPEED is out-of-range"
#endif

/* Interrupt groups */

#define RX_INTERRUPTS     (ENET_INT_RXF | ENET_INT_RXB)
#define TX_INTERRUPTS      ENET_INT_TXF
#define ERROR_INTERRUPTS  (ENET_INT_UN    | ENET_INT_RL   | ENET_INT_LC | \
                           ENET_INT_EBERR | ENET_INT_BABT | ENET_INT_BABR)

/* The subset of errors that require us to reset the hardware - this list
 * may need to be revisited if it's found that some error above leads to a
 * locking up of the Ethernet interface.
 */

#define CRITICAL_ERROR    (ENET_INT_UN | ENET_INT_RL | ENET_INT_EBERR)

/* This is a helper pointer for accessing
 * the contents of the Ethernet header
 */

#define BUF ((struct eth_hdr_s *)priv->dev.d_buf)

#define IMXRT_BUF_SIZE  ENET_ALIGN_UP(CONFIG_NET_ETH_PKTSIZE)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The imxrt_driver_s encapsulates all state information for
 * a single hardware interface
 */

struct imxrt_driver_s
{
  bool bifup;                  /* true:ifup false:ifdown */
  uint8_t txtail;              /* The oldest busy TX descriptor */
  uint8_t txhead;              /* The next TX descriptor to use */
  uint8_t rxtail;              /* The next RX descriptor to use */
  uint8_t phyaddr;             /* Selected PHY address */
  struct wdog_s txpoll;        /* TX poll timer */
  struct wdog_s txtimeout;     /* TX timeout timer */
  struct work_s irqwork;       /* For deferring interrupt work to the work queue */
  struct work_s pollwork;      /* For deferring poll work to the work queue */
  struct enet_desc_s *txdesc;  /* A pointer to the list of TX descriptor */
  struct enet_desc_s *rxdesc;  /* A pointer to the list of RX descriptors */

  /* This holds the information visible to the NuttX network */

  struct net_driver_s dev;     /* Interface understood by the network */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct imxrt_driver_s g_enet[CONFIG_IMXRT_ENET_NETHIFS];

/* The DMA descriptors.  A unaligned uint8_t is used to allocate the
 * memory; 16 is added to assure that we can meet the descriptor alignment
 * requirements.
 */

static uint8_t g_desc_pool[NENET_NBUFFERS * sizeof(struct enet_desc_s)]
               __attribute__((aligned(ENET_ALIGN)));

/* The DMA buffers.  Again, A unaligned uint8_t is used to allocate the
 * memory; 16 is added to assure that we can meet the descriptor alignment
 * requirements.
 */

static uint8_t g_buffer_pool[NENET_NBUFFERS * IMXRT_BUF_SIZE]
               __attribute__((aligned(ENET_ALIGN)));

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Utility functions */

#ifndef IMXRT_BUFFERS_SWAP
#  define imxrt_swap32(value) (value)
#  define imxrt_swap16(value) (value)
#else
#if 0 /* Use builtins if the compiler supports them */
static inline uint32_t imxrt_swap32(uint32_t value);
static inline uint16_t imxrt_swap16(uint16_t value);
#else
#  define imxrt_swap32 __builtin_bswap32
#  define imxrt_swap16 __builtin_bswap16
#endif
#endif

/* Common TX logic */

static bool imxrt_txringfull(FAR struct imxrt_driver_s *priv);
static int  imxrt_transmit(FAR struct imxrt_driver_s *priv);
static int  imxrt_txpoll(struct net_driver_s *dev);

/* Interrupt handling */

static void imxrt_dispatch(FAR struct imxrt_driver_s *priv);
static void imxrt_receive(FAR struct imxrt_driver_s *priv);
static void imxrt_txdone(FAR struct imxrt_driver_s *priv);

static void imxrt_enet_interrupt_work(FAR void *arg);
static int  imxrt_enet_interrupt(int irq, FAR void *context, FAR void *arg);

/* Watchdog timer expirations */

static void imxrt_txtimeout_work(FAR void *arg);
static void imxrt_txtimeout_expiry(wdparm_t arg);

static void imxrt_poll_work(FAR void *arg);
static void imxrt_polltimer_expiry(wdparm_t arg);

/* NuttX callback functions */

static int  imxrt_ifup(struct net_driver_s *dev);
static int  imxrt_ifdown(struct net_driver_s *dev);

static void imxrt_txavail_work(FAR void *arg);
static int  imxrt_txavail(struct net_driver_s *dev);

/* Internal ifup function that allows phy reset to be optional */

static int imxrt_ifup_action(struct net_driver_s *dev, bool resetphy);

#ifdef CONFIG_NET_MCASTGROUP
static int  imxrt_addmac(struct net_driver_s *dev,
              FAR const uint8_t *mac);
static int  imxrt_rmmac(struct net_driver_s *dev, FAR const uint8_t *mac);
#endif

#ifdef CONFIG_NETDEV_IOCTL
static int  imxrt_ioctl(struct net_driver_s *dev, int cmd,
            unsigned long arg);
#endif

/* PHY/MII support */

#if defined(CONFIG_NETDEV_PHY_IOCTL) && defined(CONFIG_ARCH_PHY_INTERRUPT)
static int imxrt_phyintenable(struct imxrt_driver_s *priv);
#endif
static inline void imxrt_initmii(struct imxrt_driver_s *priv);
static int imxrt_writemii(struct imxrt_driver_s *priv, uint8_t phyaddr,
             uint8_t regaddr, uint16_t data);
static int imxrt_readmii(struct imxrt_driver_s *priv, uint8_t phyaddr,
             uint8_t regaddr, uint16_t *data);
static int imxrt_initphy(struct imxrt_driver_s *priv, bool renogphy);

/* Initialization */

static void imxrt_initbuffers(struct imxrt_driver_s *priv);
static void imxrt_reset(struct imxrt_driver_s *priv);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: imxrt_swap16/32
 *
 * Description:
 *   The descriptors are represented by structures  Unfortunately, when the
 *   structures are overlaid on the data, the bytes are reversed because
 *   the underlying hardware writes the data in big-endian byte order.
 *
 * Input Parameters:
 *   value  - The value to be byte swapped
 *
 * Returned Value:
 *   The byte swapped value
 *
 ****************************************************************************/

#if 0 /* Use builtins if the compiler supports them */
#ifndef CONFIG_ENDIAN_BIG
static inline uint32_t imxrt_swap32(uint32_t value)
{
  uint32_t result = 0;

  __asm__ __volatile__
  (
    "rev %0, %1"
    :"=r" (result)
    : "r"(value)
  );
  return result;
}

static inline uint16_t imxrt_swap16(uint16_t value)
{
  uint16_t result = 0;

  __asm__ __volatile__
  (
    "revsh %0, %1"
    :"=r" (result)
    : "r"(value)
  );
  return result;
}
#endif
#endif

/****************************************************************************
 * Function: imxrt_txringfull
 *
 * Description:
 *   Check if all of the TX descriptors are in use.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   true is the TX ring is full; false if there are free slots at the
 *   head index.
 *
 ****************************************************************************/

static bool imxrt_txringfull(FAR struct imxrt_driver_s *priv)
{
  uint8_t txnext;

  /* Check if there is room in the hardware to hold another outgoing
   * packet.  The ring is full if incrementing the head pointer would
   * collide with the tail pointer.
   */

  txnext = priv->txhead + 1;
  if (txnext >= CONFIG_IMXRT_ENET_NTXBUFFERS)
    {
      txnext = 0;
    }

  return priv->txtail == txnext;
}

/****************************************************************************
 * Function: imxrt_transmit
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

static int imxrt_transmit(FAR struct imxrt_driver_s *priv)
{
  struct enet_desc_s *txdesc;
  irqstate_t flags;
  uint32_t regval;
  uint8_t *buf;

  /* Since this can be called from imxrt_receive, it is possible that
   * the transmit queue is full, so check for that now.  If this is the
   * case, the outgoing packet will be dropped (e.g. an ARP reply)
   */

  if (imxrt_txringfull(priv))
    {
      return -EBUSY;
    }

  /* When we get here the TX descriptor should show that the previous
   * transfer has  completed.  If we get here, then we are committed to
   * sending a packet; Higher level logic must have assured that there is
   * no transmission in progress.
   */

  txdesc = &priv->txdesc[priv->txhead];
  priv->txhead++;
  if (priv->txhead >= CONFIG_IMXRT_ENET_NTXBUFFERS)
    {
      priv->txhead = 0;
    }

#ifdef CONFIG_DEBUG_ASSERTIONS
  up_invalidate_dcache((uintptr_t)txdesc,
                       (uintptr_t)txdesc + sizeof(struct enet_desc_s));

  DEBUGASSERT(priv->txtail != priv->txhead &&
             (txdesc->status1 & TXDESC_R) == 0);
#endif

  /* Increment statistics */

  NETDEV_TXPACKETS(&priv->dev);

  /* Setup the buffer descriptor for transmission: address=priv->dev.d_buf,
   * length=priv->dev.d_len
   */

  txdesc->length   = imxrt_swap16(priv->dev.d_len);
#ifdef CONFIG_IMXRT_ENETENHANCEDBD
  txdesc->bdu      = 0x00000000;
  txdesc->status2  = TXDESC_INT | TXDESC_TS; /* | TXDESC_IINS | TXDESC_PINS; */
#endif
  txdesc->status1 |= (TXDESC_R | TXDESC_L | TXDESC_TC);

  buf = (uint8_t *)imxrt_swap32((uint32_t)priv->dev.d_buf);
  if (priv->rxdesc[priv->rxtail].data == buf)
    {
      struct enet_desc_s *rxdesc = &priv->rxdesc[priv->rxtail];

      /* Data was written into the RX buffer, so swap the TX and RX buffers */

      DEBUGASSERT((rxdesc->status1 & RXDESC_E) == 0);
      rxdesc->data = txdesc->data;
      txdesc->data = buf;
    }
  else
    {
      DEBUGASSERT(txdesc->data == buf);
    }

  /* Make the following operations atomic */

  flags = spin_lock_irqsave();

  /* Enable TX interrupts */

  regval  = getreg32(IMXRT_ENET_EIMR);
  regval |= TX_INTERRUPTS;
  putreg32(regval, IMXRT_ENET_EIMR);

  /* Setup the TX timeout watchdog (perhaps restarting the timer) */

  wd_start(&priv->txtimeout, IMXRT_TXTIMEOUT,
           imxrt_txtimeout_expiry, (wdparm_t)priv);

  /* Start the TX transfer (if it was not already waiting for buffers) */

  putreg32(ENET_TDAR, IMXRT_ENET_TDAR);

  spin_unlock_irqrestore(flags);
  return OK;
}

/****************************************************************************
 * Function: imxrt_txpoll
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

static int imxrt_txpoll(struct net_driver_s *dev)
{
  FAR struct imxrt_driver_s *priv =
    (FAR struct imxrt_driver_s *)dev->d_private;

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  if (priv->dev.d_len > 0)
    {
      /* Look up the destination MAC address and add it to the Ethernet
       * header.
       */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
      if (IFF_IS_IPv4(priv->dev.d_flags))
#endif
        {
          arp_out(&priv->dev);
        }
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
      else
#endif
        {
          neighbor_out(&priv->dev);
        }
#endif /* CONFIG_NET_IPv6 */

      if (!devif_loopback(&priv->dev))
        {
          /* Send the packet */

          imxrt_transmit(priv);
          priv->dev.d_buf = (uint8_t *)
            imxrt_swap32((uint32_t)priv->txdesc[priv->txhead].data);

          /* Check if there is room in the device to hold another packet. If
           * not, return a non-zero value to terminate the poll.
           */

          if (imxrt_txringfull(priv))
            {
              return -EBUSY;
            }
        }
    }

  /* If zero is returned, the polling will continue until
   * all connections have been examined.
   */

  return 0;
}

/****************************************************************************
 * Function: imxrt_dispatch
 *
 * Description:
 *   A new Rx packet was received; dispatch that packet to the network layer
 *   as necessary.
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

static inline void imxrt_dispatch(FAR struct imxrt_driver_s *priv)
{
  /* Update statistics */

  NETDEV_RXPACKETS(&priv->dev);

#ifdef CONFIG_NET_PKT
  /* When packet sockets are enabled, feed the frame into the tap */

  pkt_input(&priv->dev);
#endif

#ifdef CONFIG_NET_IPv4
  /* Check for an IPv4 packet */

  if (BUF->type == HTONS(ETHTYPE_IP))
    {
      ninfo("IPv4 frame\n");
      NETDEV_RXIPV4(&priv->dev);

      /* Handle ARP on input then give the IPv4 packet to the network
       * layer
       */

      arp_ipin(&priv->dev);
      ipv4_input(&priv->dev);

      /* If the above function invocation resulted in data that should be
       * sent out on the network, d_len field will set to a value > 0.
       */

      if (priv->dev.d_len > 0)
        {
          /* Update the Ethernet header with the correct MAC address */

#ifdef CONFIG_NET_IPv6
          if (IFF_IS_IPv4(priv->dev.d_flags))
#endif
            {
              arp_out(&priv->dev);
            }
#ifdef CONFIG_NET_IPv6
          else
            {
              neighbor_out(&priv->dev);
            }
#endif

          /* And send the packet */

          imxrt_transmit(priv);
        }
    }
  else
#endif
#ifdef CONFIG_NET_IPv6
  /* Check for an IPv6 packet */

  if (BUF->type == HTONS(ETHTYPE_IP6))
    {
      ninfo("IPv6 frame\n");
      NETDEV_RXIPV6(&priv->dev);

      /* Give the IPv6 packet to the network layer */

      ipv6_input(&priv->dev);

      /* If the above function invocation resulted in data that should be
       * sent out on the network, d_len field will set to a value > 0.
       */

      if (priv->dev.d_len > 0)
        {
          /* Update the Ethernet header with the correct MAC address */

#ifdef CONFIG_NET_IPv4
          if (IFF_IS_IPv4(priv->dev.d_flags))
            {
              arp_out(&priv->dev);
            }
          else
#endif
#ifdef CONFIG_NET_IPv6
            {
              neighbor_out(&priv->dev);
            }
#endif

          /* And send the packet */

          imxrt_transmit(priv);
        }
    }
  else
#endif
#ifdef CONFIG_NET_ARP
  /* Check for an ARP packet */

  if (BUF->type == htons(ETHTYPE_ARP))
    {
      NETDEV_RXARP(&priv->dev);
      arp_arpin(&priv->dev);

      /* If the above function invocation resulted in data that should
       * be sent out on the network, the field  d_len will set to a
       * value > 0.
       */

      if (priv->dev.d_len > 0)
        {
          imxrt_transmit(priv);
        }
    }
#endif
  else
    {
      NETDEV_RXDROPPED(&priv->dev);
    }
}

/****************************************************************************
 * Function: imxrt_receive
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

static void imxrt_receive(FAR struct imxrt_driver_s *priv)
{
  struct enet_desc_s *rxdesc;
  bool received;

  /* Loop while there are received packets to be processed */

  do
    {
      /* Invalidate the Rx descriptor.  Since it has been modified via DMA,
       * we must assure that we must invalid any cached values and re-read
       * the descriptor from the memory.
       */

      rxdesc = &priv->rxdesc[priv->rxtail];
      up_invalidate_dcache((uintptr_t)rxdesc,
                           (uintptr_t)rxdesc + sizeof(struct enet_desc_s));

      /* Check if the data buffer associated with the descriptor has
       * been filled with valid data.
       */

      received = ((rxdesc->status1 & RXDESC_E) == 0);
      if (received)
        {
          /* Copy the buffer pointer to priv->dev.d_buf.  Set amount of data
           * in priv->dev.d_len
           */

          priv->dev.d_len = imxrt_swap16(rxdesc->length);
          priv->dev.d_buf = (uint8_t *)imxrt_swap32((uint32_t)rxdesc->data);

          /* Invalidate the buffer so that the correct packet will be re-read
           * from memory when the packet content is accessed.
           */

          up_invalidate_dcache((uintptr_t)priv->dev.d_buf,
                               (uintptr_t)priv->dev.d_buf + priv->dev.d_len);

          /* Dispatch (or drop) the newly received packet */

          imxrt_dispatch(priv);

          /* Point the packet buffer back to the next Tx buffer that will be
           * used during the next write.  If the write queue is full, then
           * this will point at an active buffer, which must not be written
           * to.  This is OK because devif_poll won't be called unless the
           * queue is not full.
           */

          priv->dev.d_buf = (uint8_t *)
            imxrt_swap32((uint32_t)priv->txdesc[priv->txhead].data);
          rxdesc->status1 |= RXDESC_E;

          /* Update the index to the next descriptor */

          priv->rxtail++;
          if (priv->rxtail >= CONFIG_IMXRT_ENET_NRXBUFFERS)
            {
              priv->rxtail = 0;
            }

          /* Indicate that there have been empty receive buffers produced */

          putreg32(ENET_RDAR, IMXRT_ENET_RDAR);
        }
    }
  while (received);
}

/****************************************************************************
 * Function: imxrt_txdone
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
 *   The network is locked.
 *
 ****************************************************************************/

static void imxrt_txdone(FAR struct imxrt_driver_s *priv)
{
  struct enet_desc_s *txdesc;
  uint32_t regval;
  bool txdone;

  /* We are here because a transmission completed, so the watchdog can be
   * canceled.
   */

  wd_cancel(&priv->txtimeout);

  /* Verify that the oldest descriptor descriptor completed */

  do
    {
      /* Invalidate the Tx descriptor.  Since status information has been
       * modified via DMA, we must assure that we must invalid any cached
       * values and re-read the descriptor from the memory.
       */

      txdesc = &priv->txdesc[priv->txtail];
      up_invalidate_dcache((uintptr_t)txdesc,
                           (uintptr_t)txdesc + sizeof(struct enet_desc_s));

      txdone = false;
      if ((txdesc->status1 & TXDESC_R) == 0 && priv->txtail != priv->txhead)
        {
          /* Yes.. bump up the tail pointer, making space for a new TX
           * descriptor.
           */

          priv->txtail++;
          if (priv->txtail >= CONFIG_IMXRT_ENET_NTXBUFFERS)
            {
              priv->txtail = 0;
            }

          /* Update statistics */

          NETDEV_TXDONE(&priv->dev);
          txdone = true;
        }
    }
  while (txdone);

  /* Are there other transmissions queued? */

  if (priv->txtail == priv->txhead)
    {
      /* No.. Cancel the TX timeout and disable further Tx interrupts. */

      wd_cancel(&priv->txtimeout);

      regval  = getreg32(IMXRT_ENET_EIMR);
      regval &= ~TX_INTERRUPTS;
      putreg32(regval, IMXRT_ENET_EIMR);
    }

  /* There should be space for a new TX in any event.  Poll the network for
   * new XMIT data
   */

  devif_poll(&priv->dev, imxrt_txpoll);
}

/****************************************************************************
 * Function: imxrt_enet_interrupt_work
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
 *   The network is locked.
 *
 ****************************************************************************/

static void imxrt_enet_interrupt_work(FAR void *arg)
{
  FAR struct imxrt_driver_s *priv = (FAR struct imxrt_driver_s *)arg;
  uint32_t pending;
#ifdef CONFIG_NET_MCASTGROUP
  uint32_t gaurstore;
  uint32_t galrstore;
#endif

  /* Process pending Ethernet interrupts */

  net_lock();

  /* Get the set of unmasked, pending interrupt. */

  pending = getreg32(IMXRT_ENET_EIR) & getreg32(IMXRT_ENET_EIMR);

  /* Clear the pending interrupts */

  putreg32(pending, IMXRT_ENET_EIR);

  /* Check for errors */

  if (pending & ERROR_INTERRUPTS)
    {
      /* An error has occurred, update statistics */

      NETDEV_ERRORS(&priv->dev);

      nerr("ERROR: Network interface error occurred (0x%08X)\n",
           (pending & ERROR_INTERRUPTS));
    }

  if (pending & CRITICAL_ERROR)
    {
      nerr("Critical error, restarting Ethernet interface\n");

      /* Bring the Ethernet chip down and back up but with no need to
       * reset/renegotiate the phy.
       */

#ifdef CONFIG_NET_MCASTGROUP
      /* Just before we pull the rug lets make sure we retain the
       * multicast hash table.
       */

      gaurstore = getreg32(IMXRT_ENET_GAUR);
      galrstore = getreg32(IMXRT_ENET_GALR);
#endif

      imxrt_ifdown(&priv->dev);
      imxrt_ifup_action(&priv->dev, false);

#ifdef CONFIG_NET_MCASTGROUP
      /* Now write the multicast table back */

      putreg32(gaurstore, IMXRT_ENET_GAUR);
      putreg32(galrstore, IMXRT_ENET_GALR);
#endif

      /* Then poll the network for new XMIT data */

      devif_poll(&priv->dev, imxrt_txpoll);
    }
  else
    {
      /* Check for the receipt of a packet */

      if ((pending & ENET_INT_RXF) != 0)
        {
          /* A packet has been received, call imxrt_receive() to handle the
           * packet.
           */

          imxrt_receive(priv);
        }

      /* Check if a packet transmission has completed */

      if ((pending & ENET_INT_TXF) != 0)
        {
          /* Call imxrt_txdone to handle the end of transfer even.  NOTE
           * that this may disable further Tx interrupts if there are no
           * pending transmissions.
           */

          imxrt_txdone(priv);
        }
    }

  net_unlock();

  /* Re-enable Ethernet interrupts */

#if 0
  up_enable_irq(IMXRT_IRQ_EMACTMR);
#endif
  up_enable_irq(IMXRT_IRQ_ENET);
}

/****************************************************************************
 * Function: imxrt_enet_interrupt
 *
 * Description:
 *   Three interrupt sources will vector to this function:
 *   1. Ethernet MAC transmit interrupt handler
 *   2. Ethernet MAC receive interrupt handler
 *   3.
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

static int imxrt_enet_interrupt(int irq, FAR void *context, FAR void *arg)
{
  register FAR struct imxrt_driver_s *priv = &g_enet[0];

  /* Disable further Ethernet interrupts.  Because Ethernet interrupts are
   * also disabled if the TX timeout event occurs, there can be no race
   * condition here.
   */

  up_disable_irq(IMXRT_IRQ_ENET);

  /* Schedule to perform the interrupt processing on the worker thread. */

  work_queue(ETHWORK, &priv->irqwork, imxrt_enet_interrupt_work, priv, 0);
  return OK;
}

/****************************************************************************
 * Function: imxrt_txtimeout_work
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
 *
 ****************************************************************************/

static void imxrt_txtimeout_work(FAR void *arg)
{
  FAR struct imxrt_driver_s *priv = (FAR struct imxrt_driver_s *)arg;

  /* Increment statistics and dump debug info */

  net_lock();
  nerr("Resetting interface\n");

  NETDEV_TXTIMEOUTS(&priv->dev);

  /* Take the interface down and bring it back up.  That is the most
   * aggressive hardware reset.
   */

  imxrt_ifdown(&priv->dev);
  imxrt_ifup_action(&priv->dev, false);

  /* Then poll the network for new XMIT data */

  devif_poll(&priv->dev, imxrt_txpoll);
  net_unlock();
}

/****************************************************************************
 * Function: imxrt_txtimeout_expiry
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

static void imxrt_txtimeout_expiry(wdparm_t arg)
{
  FAR struct imxrt_driver_s *priv = (FAR struct imxrt_driver_s *)arg;

  /* Disable further Ethernet interrupts.  This will prevent some race
   * conditions with interrupt work.  There is still a potential race
   * condition with interrupt work that is already queued and in progress.
   */

  up_disable_irq(IMXRT_IRQ_ENET);

  /* Schedule to perform the TX timeout processing on the worker thread,
   * canceling any pending interrupt work.
   */

  work_queue(ETHWORK, &priv->irqwork, imxrt_txtimeout_work, priv, 0);
}

/****************************************************************************
 * Function: imxrt_poll_work
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
 *   The network is locked.
 *
 ****************************************************************************/

static void imxrt_poll_work(FAR void *arg)
{
  FAR struct imxrt_driver_s *priv = (FAR struct imxrt_driver_s *)arg;

  /* Check if there is there is a transmission in progress.
   * We cannot perform the TX poll if he are unable to accept
   * another packet for transmission.
   */

  net_lock();
  if (!imxrt_txringfull(priv))
    {
      /* If so, update TCP timing states and poll the network for new XMIT
       * data. Hmmm.. might be bug here.  Does this mean if there is a
       * transmit in progress, we will missing TCP time state updates?
       */

      devif_timer(&priv->dev, IMXRT_WDDELAY, imxrt_txpoll);
    }

  /* Setup the watchdog poll timer again in any case */

  wd_start(&priv->txpoll, IMXRT_WDDELAY,
           imxrt_polltimer_expiry, (wdparm_t)priv);
  net_unlock();
}

/****************************************************************************
 * Function: imxrt_polltimer_expiry
 *
 * Description:
 *   Periodic timer handler.  Called from the timer interrupt handler.
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

static void imxrt_polltimer_expiry(wdparm_t arg)
{
  FAR struct imxrt_driver_s *priv = (FAR struct imxrt_driver_s *)arg;

  /* Schedule to perform the poll processing on the worker thread. */

  work_queue(ETHWORK, &priv->pollwork, imxrt_poll_work, priv, 0);
}

/****************************************************************************
 * Function: imxrt_ifup_action
 *
 * Description:
 *   Internal action routine to bring up the Ethernet interface
 *   which makes the resetting of the phy (which takes considerable time)
 *   optional.
 *
 * Input Parameters:
 *   dev      - Reference to the NuttX driver state structure
 *   resetphy - Flag indicating if Phy is to be reset. If not then the
 *              phy configuration is just re-loaded into the ethernet
 *              interface
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int imxrt_ifup_action(struct net_driver_s *dev, bool resetphy)
{
  FAR struct imxrt_driver_s *priv =
    (FAR struct imxrt_driver_s *)dev->d_private;
  uint8_t *mac = dev->d_mac.ether.ether_addr_octet;
  uint32_t regval;
  int ret;

  ninfo("Bringing up: %d.%d.%d.%d\n",
        dev->d_ipaddr & 0xff, (dev->d_ipaddr >> 8) & 0xff,
        (dev->d_ipaddr >> 16) & 0xff, dev->d_ipaddr >> 24);

  /* Initialize ENET buffers */

  imxrt_initbuffers(priv);

  /* Configure the MII interface */

  imxrt_initmii(priv);

  /* Set the MAC address */

  putreg32((mac[0] << 24) | (mac[1] << 16) | (mac[2] << 8) | mac[3],
           IMXRT_ENET_PALR);
  putreg32((mac[4] << 24) | (mac[5] << 16), IMXRT_ENET_PAUR);

  /* Configure the PHY */

  ret = imxrt_initphy(priv, resetphy);
  if (ret < 0)
    {
      nerr("ERROR: Failed to configure the PHY: %d\n", ret);
      return ret;
    }

  /* Handle promiscuous mode */

#ifdef CONFIG_NET_PROMISCUOUS
  regval = getreg32(IMXRT_ENET_RCR);
  regval |= ENET_RCR_PROM;
  putreg32(regval, IMXRT_ENET_RCR);
#endif

  /* Select legacy of enhanced buffer descriptor format */

#ifdef CONFIG_IMXRT_ENETENHANCEDBD
  putreg32(ENET_ECR_EN1588, IMXRT_ENET_ECR);
#else
  putreg32(0, IMXRT_ENET_ECR);
#endif

  /* Set the RX buffer size */

  putreg32(IMXRT_BUF_SIZE, IMXRT_ENET_MRBR);

  /* Point to the start of the circular RX buffer descriptor queue */

  putreg32((uint32_t)priv->rxdesc, IMXRT_ENET_RDSR);

  /* Point to the start of the circular TX buffer descriptor queue */

  putreg32((uint32_t)priv->txdesc, IMXRT_ENET_TDSR);

  /* And enable the MAC itself */

  regval  = getreg32(IMXRT_ENET_ECR);
  regval |= ENET_ECR_ETHEREN
#ifdef IMXRT_USE_DBSWAP
         | ENET_ECR_DBSWP
#endif
        ;
  putreg32(regval, IMXRT_ENET_ECR);

  /* Indicate that there have been empty receive buffers produced */

  putreg32(ENET_RDAR, IMXRT_ENET_RDAR);

  /* Set and activate a timer process */

  wd_start(&priv->txpoll, IMXRT_WDDELAY,
           imxrt_polltimer_expiry, (wdparm_t)priv);

  /* Clear all pending ENET interrupt */

  putreg32(RX_INTERRUPTS | ERROR_INTERRUPTS | TX_INTERRUPTS, IMXRT_ENET_EIR);

  /* Enable RX and error interrupts at the controller (TX interrupts are
   * still disabled).
   */

  putreg32(RX_INTERRUPTS | ERROR_INTERRUPTS,
           IMXRT_ENET_EIMR);

  /* Mark the interrupt "up" and enable interrupts at the NVIC */

  priv->bifup = true;

#if 0
  up_enable_irq(IMXRT_IRQ_EMACTMR);
#endif
  up_enable_irq(IMXRT_IRQ_ENET);

  return OK;
}

/****************************************************************************
 * Function: imxrt_ifup
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

static int imxrt_ifup(struct net_driver_s *dev)
{
  /* The externally available ifup action includes resetting the phy */

  return imxrt_ifup_action(dev, true);
}

/****************************************************************************
 * Function: imxrt_ifdown
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

static int imxrt_ifdown(struct net_driver_s *dev)
{
  FAR struct imxrt_driver_s *priv =
    (FAR struct imxrt_driver_s *)dev->d_private;
  irqstate_t flags;

  ninfo("Taking down: %d.%d.%d.%d\n",
        dev->d_ipaddr & 0xff, (dev->d_ipaddr >> 8) & 0xff,
        (dev->d_ipaddr >> 16) & 0xff, dev->d_ipaddr >> 24);

  /* Flush and disable the Ethernet interrupts at the NVIC */

  flags = enter_critical_section();

  up_disable_irq(IMXRT_IRQ_ENET);
  putreg32(0, IMXRT_ENET_EIMR);

  /* Cancel the TX poll timer and TX timeout timers */

  wd_cancel(&priv->txpoll);
  wd_cancel(&priv->txtimeout);

  /* Put the EMAC in its reset, non-operational state.  This should be
   * a known configuration that will guarantee the imxrt_ifup() always
   * successfully brings the interface back up.
   */

  imxrt_reset(priv);

  /* Mark the device "down" */

  priv->bifup = false;
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Function: imxrt_txavail_work
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

static void imxrt_txavail_work(FAR void *arg)
{
  FAR struct imxrt_driver_s *priv = (FAR struct imxrt_driver_s *)arg;

  /* Ignore the notification if the interface is not yet up */

  net_lock();
  if (priv->bifup)
    {
      /* Check if there is room in the hardware to hold another outgoing
       * packet.
       */

      if (!imxrt_txringfull(priv))
        {
          /* No, there is space for another transfer.  Poll the network for
           * new XMIT data.
           */

          devif_poll(&priv->dev, imxrt_txpoll);
        }
    }

  net_unlock();
}

/****************************************************************************
 * Function: imxrt_txavail
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

static int imxrt_txavail(struct net_driver_s *dev)
{
  FAR struct imxrt_driver_s *priv =
    (FAR struct imxrt_driver_s *)dev->d_private;

  /* Is our single work structure available?  It may not be if there are
   * pending interrupt actions and we will have to ignore the Tx
   * availability action.
   */

  if (work_available(&priv->pollwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      work_queue(ETHWORK, &priv->pollwork, imxrt_txavail_work, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Function: imxrt_calcethcrc
 *
 * Description:
 *   Function to calculate the CRC used by IMXRT to check an Ethernet frame
 *
 * Input Parameters:
 *   data   - the data to be checked
 *   length - length of the data
 *
 * Returned Value:
 *   crc32
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_MCASTGROUP
static uint32_t imxrt_calcethcrc(const uint8_t *data, size_t length)
{
  uint32_t crc    = 0xffffffffu;
  uint32_t count1 = 0;
  uint32_t count2 = 0;

  /* Calculates the CRC-32 polynomial on the multicast group address. */

  for (count1 = 0; count1 < length; count1++)
    {
      uint8_t c = data[count1];

      for (count2 = 0; count2 < 0x08u; count2++)
        {
          if ((c ^ crc) & 1U)
            {
              crc >>= 1U;
              c   >>= 1U;
              crc  ^= 0xedb88320u;
            }
          else
            {
              crc >>= 1U;
              c   >>= 1U;
            }
        }
    }

  return crc;
}
#endif

/****************************************************************************
 * Function: imxrt_enet_hash_index
 *
 * Description:
 *   Function to find the hash index for multicast address filter
 *
 * Input Parameters:
 *   mac  - The MAC address
 *
 * Returned Value:
 *   hash index
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_MCASTGROUP
static uint32_t imxrt_enet_hash_index(const uint8_t *mac)
{
  uint32_t crc;
  uint32_t hashindex;

  ninfo("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  crc = imxrt_calcethcrc(mac, 6);
  hashindex = (crc >> 26) & 0x3f;

  return hashindex;
}
#endif

/****************************************************************************
 * Function: imxrt_addmac
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
static int imxrt_addmac(struct net_driver_s *dev, FAR const uint8_t *mac)
{
  uint32_t hashindex;
  uint32_t temp;
  uint32_t registeraddress;

  hashindex = imxrt_enet_hash_index(mac);

  /* Add the MAC address to the hardware multicast routing table */

  if (hashindex > 31)
    {
      registeraddress = IMXRT_ENET_GAUR;
      hashindex      -= 32;
    }
  else
    {
      registeraddress = IMXRT_ENET_GALR;
    }

  temp  = getreg32(registeraddress);
  temp |= 1 << hashindex;
  putreg32(temp, registeraddress);

  return OK;
}
#endif

/****************************************************************************
 * Function: imxrt_rmmac
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
static int imxrt_rmmac(struct net_driver_s *dev, FAR const uint8_t *mac)
{
  uint32_t hashindex;
  uint32_t temp;
  uint32_t registeraddress;

  /* Remove the MAC address from the hardware multicast routing table */

  hashindex = imxrt_enet_hash_index(mac);

  if (hashindex > 31)
    {
      registeraddress = IMXRT_ENET_GAUR;
      hashindex      -= 32;
    }
  else
    {
      registeraddress = IMXRT_ENET_GALR;
    }

  temp  = getreg32(registeraddress);
  temp &= ~(1 << hashindex);
  putreg32(temp, registeraddress);

  return OK;
}
#endif

/****************************************************************************
 * Function: imxrt_ioctl
 *
 * Description:
 *   PHY ioctl command handler
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   cmd  - ioctl command
 *   arg  - Argument accompanying the command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NETDEV_IOCTL
static int imxrt_ioctl(struct net_driver_s *dev, int cmd, unsigned long arg)
{
#ifdef CONFIG_NETDEV_PHY_IOCTL
  FAR struct imxrt_driver_s *priv =
    (FAR struct imxrt_driver_s *)dev->d_private;
#endif
  int ret;

  switch (cmd)
    {
#ifdef CONFIG_NETDEV_PHY_IOCTL
#ifdef CONFIG_ARCH_PHY_INTERRUPT
      case SIOCMIINOTIFY: /* Set up for PHY event notifications */
        {
          struct mii_ioctl_notify_s *req =
            (struct mii_ioctl_notify_s *)((uintptr_t)arg);

          ret = phy_notify_subscribe(dev->d_ifname, req->pid, &req->event);
          if (ret == OK)
            {
              /* Enable PHY link up/down interrupts */

              ret = imxrt_phyintenable(priv);
            }
        }
        break;
#endif

      case SIOCGMIIPHY: /* Get MII PHY address */
        {
          struct mii_ioctl_data_s *req =
            (struct mii_ioctl_data_s *)((uintptr_t)arg);
          req->phy_id = priv->phyaddr;
          ret = OK;
        }
        break;

      case SIOCGMIIREG: /* Get register from MII PHY */
        {
          struct mii_ioctl_data_s *req =
            (struct mii_ioctl_data_s *)((uintptr_t)arg);
          ret = imxrt_readmii(priv, req->phy_id,
                              req->reg_num, &req->val_out);
        }
        break;

      case SIOCSMIIREG: /* Set register in MII PHY */
        {
          struct mii_ioctl_data_s *req =
            (struct mii_ioctl_data_s *)((uintptr_t)arg);
          ret = imxrt_writemii(priv, req->phy_id, req->reg_num, req->val_in);
        }
        break;
#endif /* CONFIG_NETDEV_PHY_IOCTL */

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}
#endif /* CONFIG_NETDEV_IOCTL */

/****************************************************************************
 * Function: imxrt_phyintenable
 *
 * Description:
 *  Enable link up/down PHY interrupts.  The interrupt protocol is like this:
 *
 *  - Interrupt status is cleared when the interrupt is enabled.
 *  - Interrupt occurs.  Interrupt is disabled (at the processor level) when
 *    is received.
 *  - Interrupt status is cleared when the interrupt is re-enabled.
 *
 * Input Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno (-ETIMEDOUT) on failure.
 *
 ****************************************************************************/

#if defined(CONFIG_NETDEV_PHY_IOCTL) && defined(CONFIG_ARCH_PHY_INTERRUPT)
static int imxrt_phyintenable(struct imxrt_driver_s *priv)
{
#if defined(CONFIG_ETH0_PHY_KSZ8051) || defined(CONFIG_ETH0_PHY_KSZ8061) || \
    defined(CONFIG_ETH0_PHY_KSZ8081)
  uint16_t phyval;
  int ret;

  /* Read the interrupt status register in order to clear any pending
   * interrupts
   */

  ret = imxrt_readmii(priv, priv->phyaddr, MII_KSZ8081_INT, &phyval);
  if (ret == OK)
    {
      /* Enable link up/down interrupts */

      ret = imxrt_writemii(priv, priv->phyaddr, MII_KSZ8081_INT,
                           (MII_KSZ80X1_INT_LDEN | MII_KSZ80X1_INT_LUEN));
    }

  return ret;
#else
#  error Unrecognized PHY
  return -ENOSYS;
#endif
}
#endif

/****************************************************************************
 * Function: imxrt_initmii
 *
 * Description:
 *   Configure the MII interface
 *
 * Input Parameters:
 *   priv - Reference to the private ENET driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void imxrt_initmii(struct imxrt_driver_s *priv)
{
  /* Speed is based on the peripheral (bus) clock; hold time is 2 module
   * clock.  This hold time value may need to be increased on some platforms
   */

  putreg32(ENET_MSCR_HOLDTIME_2CYCLES |
           IMXRT_MII_SPEED << ENET_MSCR_MII_SPEED_SHIFT,
           IMXRT_ENET_MSCR);
}

/****************************************************************************
 * Function: imxrt_writemii
 *
 * Description:
 *   Write a 16-bit value to a PHY register.
 *
 * Input Parameters:
 *   priv - Reference to the private ENET driver state structure
 *   phyaddr - The PHY address
 *   regaddr - The PHY register address
 *   data    - The data to write to the PHY register
 *
 * Returned Value:
 *   Zero on success, a negated errno value on failure.
 *
 ****************************************************************************/

static int imxrt_writemii(struct imxrt_driver_s *priv, uint8_t phyaddr,
                            uint8_t regaddr, uint16_t data)
{
  int timeout;

  /* Clear the MII interrupt bit */

  putreg32(ENET_INT_MII, IMXRT_ENET_EIR);

  /* Initiate the MII Management write */

  putreg32(data |
           2 << ENET_MMFR_TA_SHIFT |
           (uint32_t)regaddr << ENET_MMFR_RA_SHIFT |
           (uint32_t)phyaddr << ENET_MMFR_PA_SHIFT |
           ENET_MMFR_OP_WRMII |
           1 << ENET_MMFR_ST_SHIFT,
           IMXRT_ENET_MMFR);

  /* Wait for the transfer to complete */

  for (timeout = 0; timeout < MII_MAXPOLLS; timeout++)
    {
      if ((getreg32(IMXRT_ENET_EIR) & ENET_INT_MII) != 0)
        {
          break;
        }
    }

  /* Check for a timeout */

  if (timeout == MII_MAXPOLLS)
    {
      return -ETIMEDOUT;
    }

  /* Clear the MII interrupt bit */

  putreg32(ENET_INT_MII, IMXRT_ENET_EIR);
  return OK;
}

/****************************************************************************
 * Function: imxrt_reademii
 *
 * Description:
 *   Read a 16-bit value from a PHY register.
 *
 * Input Parameters:
 *   priv    - Reference to the private ENET driver state structure
 *   phyaddr - The PHY address
 *   regaddr - The PHY register address
 *   data    - A pointer to the location to return the data
 *
 * Returned Value:
 *   Zero on success, a negated errno value on failure.
 *
 ****************************************************************************/

static int imxrt_readmii(struct imxrt_driver_s *priv, uint8_t phyaddr,
                           uint8_t regaddr, uint16_t *data)
{
  int timeout;

  /* Clear the MII interrupt bit */

  putreg32(ENET_INT_MII, IMXRT_ENET_EIR);

  /* Initiate the MII Management read */

  putreg32(2 << ENET_MMFR_TA_SHIFT |
           (uint32_t)regaddr << ENET_MMFR_RA_SHIFT |
           (uint32_t)phyaddr << ENET_MMFR_PA_SHIFT |
           ENET_MMFR_OP_RDMII |
           1 << ENET_MMFR_ST_SHIFT,
           IMXRT_ENET_MMFR);

  /* Wait for the transfer to complete */

  for (timeout = 0; timeout < MII_MAXPOLLS; timeout++)
    {
      if ((getreg32(IMXRT_ENET_EIR) & ENET_INT_MII) != 0)
        {
          break;
        }
    }

  /* Check for a timeout */

  if (timeout >= MII_MAXPOLLS)
    {
      nerr("ERROR: Timed out waiting for transfer to complete\n");
      return -ETIMEDOUT;
    }

  /* Clear the MII interrupt bit */

  putreg32(ENET_INT_MII, IMXRT_ENET_EIR);

  /* And return the MII data */

  *data = (uint16_t)(getreg32(IMXRT_ENET_MMFR) & ENET_MMFR_DATA_MASK);
  return OK;
}

/****************************************************************************
 * Function: imxrt_initphy
 *
 * Description:
 *   Configure the PHY
 *
 * Input Parameters:
 *   priv     - Reference to the private ENET driver state structure
 *   renogphy - Flag indicating if to perform negotiation of the link
 *
 * Returned Value:
 *   Zero (OK) returned on success; a negated errno value is returned on any
 *   failure;
 *
 * Assumptions:
 *
 ****************************************************************************/

static inline int imxrt_initphy(struct imxrt_driver_s *priv, bool renogphy)
{
  uint32_t rcr;
  uint32_t tcr;
  uint32_t racc;
  uint16_t phydata;
  uint8_t phyaddr    = BOARD_PHY_ADDR;
  int retries;
  int ret;

  if (renogphy)
    {
      /* Loop (potentially infinitely?) until we successfully communicate
       * with the PHY. This is 'standard stuff' that should work for any PHY
       * - we are not communicating with it's 'special' registers
       * at this point.
       */

      ninfo("%s: Try phyaddr: %u\n", BOARD_PHY_NAME, phyaddr);

      /* Try to read PHYID1 few times using this address */

      retries = 0;
      do
        {
          nxsig_usleep(LINK_WAITUS);

          ninfo("%s: Read PHYID1, retries=%d\n",
                BOARD_PHY_NAME, retries + 1);

          phydata = 0xffff;
          ret     = imxrt_readmii(priv, phyaddr, MII_PHYID1, &phydata);
        }
      while ((ret < 0 || phydata == 0xffff) && ++retries < 3);

      if (retries >= 3)
        {
          nerr("ERROR: Failed to read %s PHYID1 at address %d\n",
               BOARD_PHY_NAME, phyaddr);
          return -ENOENT;
        }

      ninfo("%s: Using PHY address %u\n", BOARD_PHY_NAME, phyaddr);
      priv->phyaddr = phyaddr;

      /* Verify PHYID1.  Compare OUI bits 3-18 */

      ninfo("%s: PHYID1: %04x\n", BOARD_PHY_NAME, phydata);
      if (phydata != BOARD_PHYID1)
        {
          nerr("ERROR: PHYID1=%04x incorrect for %s.  Expected %04x\n",
               phydata, BOARD_PHY_NAME, BOARD_PHYID1);
          return -ENXIO;
        }

      /* Read PHYID2 */

      ret = imxrt_readmii(priv, phyaddr, MII_PHYID2, &phydata);
      if (ret < 0)
        {
          nerr("ERROR: Failed to read %s PHYID2: %d\n", BOARD_PHY_NAME, ret);
          return ret;
        }

      ninfo("%s: PHYID2: %04x\n", BOARD_PHY_NAME, phydata);

      /* Verify PHYID2:  Compare OUI bits 19-24 and the 6-bit model number
       * (ignoring the 4-bit revision number).
       */

      if ((phydata & 0xfff0) != (BOARD_PHYID2 & 0xfff0))
        {
          nerr("ERROR: PHYID2=%04x incorrect for %s.  Expected %04x\n",
               (phydata & 0xfff0), BOARD_PHY_NAME, (BOARD_PHYID2 & 0xfff0));
          return -ENXIO;
        }

#ifdef CONFIG_ETH0_PHY_KSZ8081
      /* Reset PHY */

      imxrt_writemii(priv, phyaddr, MII_MCR, MII_MCR_RESET);

      /* Set RMII mode */

      ret = imxrt_readmii(priv, phyaddr, MII_KSZ8081_PHYCTRL2, &phydata);
      if (ret < 0)
        {
          nerr("ERROR: Failed to read MII_KSZ8081_PHYCTRL2\n");
          return ret;
        }

      /* Indicate 50MHz clock */

      imxrt_writemii(priv, phyaddr, MII_KSZ8081_PHYCTRL2,
                     (phydata | (1 << 7)));

      /* Switch off NAND Tree mode (in case it was set via pinning) */

      ret = imxrt_readmii(priv, phyaddr, MII_KSZ8081_OMSO, &phydata);
      if (ret < 0)
        {
          nerr("ERROR: Failed to read MII_KSZ8081_OMSO: %d\n", ret);
          return ret;
        }

      imxrt_writemii(priv, phyaddr, MII_KSZ8081_OMSO,
                     (phydata & ~(1 << 5)));

      /* Set Ethernet led to green = activity and yellow = link and  */

      ret = imxrt_readmii(priv, phyaddr, MII_KSZ8081_PHYCTRL2, &phydata);
      if (ret < 0)
        {
          nerr("ERROR: Failed to read MII_KSZ8081_PHYCTRL2\n");
          return ret;
        }

      imxrt_writemii(priv, phyaddr, MII_KSZ8081_PHYCTRL2,
                     (phydata | (1 << 4)));

      imxrt_writemii(priv, phyaddr, MII_ADVERTISE,
                     MII_ADVERTISE_100BASETXFULL |
                     MII_ADVERTISE_100BASETXHALF |
                     MII_ADVERTISE_10BASETXFULL |
                     MII_ADVERTISE_10BASETXHALF |
                     MII_ADVERTISE_CSMA);

#elif defined (CONFIG_ETH0_PHY_LAN8720)
      /* Make sure that PHY comes up in correct mode when it's reset */

      imxrt_writemii(priv, phyaddr, MII_LAN8720_MODES,
                     MII_LAN8720_MODES_RESV | MII_LAN8720_MODES_ALL |
                     MII_LAN8720_MODES_PHYAD(BOARD_PHY_ADDR));

      /* ...and reset PHY */

      imxrt_writemii(priv, phyaddr, MII_MCR, MII_MCR_RESET);
#endif

      /* Start auto negotiation */

      ninfo("%s: Start Autonegotiation...\n",  BOARD_PHY_NAME);
      imxrt_writemii(priv, phyaddr, MII_MCR,
                     (MII_MCR_ANRESTART | MII_MCR_ANENABLE));

      /* Wait for auto negotiation to complete */

      for (retries = 0; retries < LINK_NLOOPS; retries++)
        {
          ret = imxrt_readmii(priv, phyaddr, MII_MSR, &phydata);
          if (ret < 0)
            {
              nerr("ERROR: Failed to read %s MII_MSR: %d\n",
                    BOARD_PHY_NAME, ret);
              return ret;
            }

          if (phydata & MII_MSR_ANEGCOMPLETE)
            {
              break;
            }

          nxsig_usleep(LINK_WAITUS);
        }

      if (phydata & MII_MSR_ANEGCOMPLETE)
        {
          ninfo("%s: Autonegotiation complete\n",  BOARD_PHY_NAME);
          ninfo("%s: MII_MSR: %04x\n", BOARD_PHY_NAME, phydata);
        }
      else
        {
          /* TODO: Autonegotiation has right now failed. Maybe the Eth cable
           * is not connected.  PHY chip have mechanisms to configure link
           * OK. We should leave autconf on, and find a way to re-configure
           * MCU whenever the link is ready.
           */

          ninfo("%s: Autonegotiation failed [%d] (is cable plugged-in ?), "
                "default to 10Mbs mode\n", \
                BOARD_PHY_NAME, retries);

          /* Stop auto negotiation */

          imxrt_writemii(priv, phyaddr, MII_MCR, 0);
        }
    }

  /* When we get here we have a (negotiated) speed and duplex. This is also
   * the point we enter if renegotiation is turned off, so have multiple
   * attempts at reading the status register in case the PHY isn't awake
   * properly.
   */

  retries = 0;
  do
    {
      phydata = 0xffff;
      ret = imxrt_readmii(priv, phyaddr, BOARD_PHY_STATUS, &phydata);
    }
  while ((ret < 0 || phydata == 0xffff) && ++retries < 3);

  /* If we didn't successfully read anything and we haven't tried a physical
   * renegotiation then lets do that
   */

  if (retries >= 3)
    {
      if (renogphy == false)
        {
          /* Give things one more chance with renegotiation turned on */

          return imxrt_initphy(priv, true);
        }
      else
        {
          /* That didn't end well, just give up */

          nerr("ERROR: Failed to read %s BOARD_PHY_STATUS[%02x]: %d\n",
               BOARD_PHY_NAME, BOARD_PHY_STATUS, ret);
          return ret;
        }
    }

  ninfo("%s: BOARD_PHY_STATUS: %04x\n", BOARD_PHY_NAME, phydata);

  /* Set up the transmit and receive control registers based on the
   * configuration and the auto negotiation results.
   */

#ifdef CONFIG_IMXRT_ENETUSEMII
  rcr = ENET_RCR_CRCFWD |
        CONFIG_NET_ETH_PKTSIZE << ENET_RCR_MAX_FL_SHIFT |
        ENET_RCR_MII_MODE;
#else
  rcr = ENET_RCR_RMII_MODE | ENET_RCR_CRCFWD |
        CONFIG_NET_ETH_PKTSIZE << ENET_RCR_MAX_FL_SHIFT |
        ENET_RCR_MII_MODE;
#endif
  tcr = 0;

  putreg32(rcr, IMXRT_ENET_RCR);
  putreg32(tcr, IMXRT_ENET_TCR);

  /* Enable Discard Of Frames With MAC Layer Errors.
   * Enable Discard Of Frames With Wrong Protocol Checksum.
   * Bit 1: Enable discard of frames with wrong IPv4 header checksum.
   */

  racc = ENET_RACC_PRODIS | ENET_RACC_LINEDIS | ENET_RACC_IPDIS;
  putreg32(racc, IMXRT_ENET_RACC);

  /* Setup half or full duplex */

  if (BOARD_PHY_ISDUPLEX(phydata))
    {
      /* Full duplex */

      ninfo("%s: Full duplex\n",  BOARD_PHY_NAME);
      tcr |= ENET_TCR_FDEN;
    }
  else
    {
      /* Half duplex */

      ninfo("%s: Half duplex\n",  BOARD_PHY_NAME);
      rcr |= ENET_RCR_DRT;
    }

  if (BOARD_PHY_10BASET(phydata))
    {
      /* 10 Mbps */

      ninfo("%s: 10 Base-T\n",  BOARD_PHY_NAME);
      rcr |= ENET_RCR_RMII_10T;
    }
  else if (BOARD_PHY_100BASET(phydata))
    {
      /* 100 Mbps */

      ninfo("%s: 100 Base-T\n",  BOARD_PHY_NAME);
    }
  else
    {
      /* This might happen if Autonegotiation did not complete(?) */

      nerr("ERROR: Neither 10- nor 100-BaseT reported: PHY STATUS=%04x\n",
           phydata);
      return -EIO;
    }

  putreg32(rcr, IMXRT_ENET_RCR);
  putreg32(tcr, IMXRT_ENET_TCR);
  return OK;
}

/****************************************************************************
 * Function: imxrt_initbuffers
 *
 * Description:
 *   Initialize ENET buffers and descriptors
 *
 * Input Parameters:
 *   priv - Reference to the private ENET driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void imxrt_initbuffers(struct imxrt_driver_s *priv)
{
  uintptr_t addr;
  int i;

  /* Get an aligned TX descriptor (array) address */

  addr         = (uintptr_t)g_desc_pool;
  priv->txdesc = (struct enet_desc_s *)addr;

  /* Get an aligned RX descriptor (array) address */

  addr        +=  CONFIG_IMXRT_ENET_NTXBUFFERS * sizeof(struct enet_desc_s);
  priv->rxdesc = (struct enet_desc_s *)addr;

  /* Get the beginning of the first aligned buffer */

  addr         = (uintptr_t)g_buffer_pool;

  /* Then fill in the TX descriptors */

  for (i = 0; i < CONFIG_IMXRT_ENET_NTXBUFFERS; i++)
    {
      priv->txdesc[i].status1 = 0;
      priv->txdesc[i].length  = 0;
      priv->txdesc[i].data    = (uint8_t *)imxrt_swap32((uint32_t)addr);
#ifdef CONFIG_IMXRT_ENETENHANCEDBD
      priv->txdesc[i].status2 = TXDESC_IINS | TXDESC_PINS;
#endif
      addr                   += IMXRT_BUF_SIZE;
    }

  /* Then fill in the RX descriptors */

  for (i = 0; i < CONFIG_IMXRT_ENET_NRXBUFFERS; i++)
    {
      priv->rxdesc[i].status1 = RXDESC_E;
      priv->rxdesc[i].length  = 0;
      priv->rxdesc[i].data    = (uint8_t *)imxrt_swap32((uint32_t)addr);
#ifdef CONFIG_IMXRT_ENETENHANCEDBD
      priv->rxdesc[i].bdu     = 0;
      priv->rxdesc[i].status2 = RXDESC_INT;
#endif
      addr                   += IMXRT_BUF_SIZE;
    }

  /* Set the wrap bit in the last descriptors to form a ring */

  priv->txdesc[CONFIG_IMXRT_ENET_NTXBUFFERS - 1].status1 |= TXDESC_W;
  priv->rxdesc[CONFIG_IMXRT_ENET_NRXBUFFERS - 1].status1 |= RXDESC_W;

  /* We start with RX descriptor 0 and with no TX descriptors in use */

  priv->txtail = 0;
  priv->txhead = 0;
  priv->rxtail = 0;

  /* Initialize the packet buffer, which is used when sending */

  priv->dev.d_buf =
    (uint8_t *)imxrt_swap32((uint32_t)priv->txdesc[priv->txhead].data);
}

/****************************************************************************
 * Function: imxrt_reset
 *
 * Description:
 *   Put the EMAC in the non-operational, reset state
 *
 * Input Parameters:
 *   priv - Reference to the private ENET driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void imxrt_reset(struct imxrt_driver_s *priv)
{
  unsigned int i;

  /* Set the reset bit and clear the enable bit */

  putreg32(ENET_ECR_RESET, IMXRT_ENET_ECR);

  /* Wait at least 8 clock cycles */

  for (i = 0; i < 10; i++)
    {
      asm volatile ("nop");
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: imxrt_netinitialize
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

int imxrt_netinitialize(int intf)
{
  struct imxrt_driver_s *priv;
#ifdef CONFIG_NET_ETHERNET
  uint32_t uidl;
  uint32_t uidml;
  uint8_t *mac;
#endif
  uint32_t regval;
  int ret;

  /* Get the interface structure associated with this interface number. */

  DEBUGASSERT(intf < CONFIG_IMXRT_ENET_NETHIFS);
  priv = &g_enet[intf];

  /* Enable ENET1_TX_CLK_DIR (Provides 50MHz clk OUT to PHY) */

  regval = getreg32(IMXRT_IOMUXC_GPR_GPR1);
  regval |= GPR_GPR1_ENET1_TX_CLK_OUT_EN;
  putreg32(regval, IMXRT_IOMUXC_GPR_GPR1);

  /* Enable the ENET clock.  Clock is on during all modes,
   * except STOP mode.
   */

  imxrt_clockall_enet();

  /* Configure all ENET/MII pins */

  imxrt_config_gpio(GPIO_ENET_MDIO);
  imxrt_config_gpio(GPIO_ENET_MDC);
  imxrt_config_gpio(GPIO_ENET_RX_EN);
  imxrt_config_gpio(GPIO_ENET_RX_DATA00);
  imxrt_config_gpio(GPIO_ENET_RX_DATA01);
  imxrt_config_gpio(GPIO_ENET_TX_DATA00);
  imxrt_config_gpio(GPIO_ENET_TX_DATA01);
  imxrt_config_gpio(GPIO_ENET_TX_CLK);
  imxrt_config_gpio(GPIO_ENET_TX_EN);
#ifdef GPIO_ENET_RX_ER
  imxrt_config_gpio(GPIO_ENET_RX_ER);
#endif

  /* Attach the Ethernet MAC IEEE 1588 timer interrupt handler */

#if 0
  if (irq_attach(IMXRT_IRQ_EMACTMR, imxrt_tmrinterrupt, NULL))
    {
      /* We could not attach the ISR to the interrupt */

      nerr("ERROR: Failed to attach EMACTMR IRQ\n");
      return -EAGAIN;
    }
#endif

  /* Attach the Ethernet interrupt handler */

  if (irq_attach(IMXRT_IRQ_ENET, imxrt_enet_interrupt, NULL))
    {
      /* We could not attach the ISR to the interrupt */

      nerr("ERROR: Failed to attach EMACTX IRQ\n");
      return -EAGAIN;
    }

  /* Initialize the driver structure */

  memset(priv, 0, sizeof(struct imxrt_driver_s));
  priv->dev.d_ifup    = imxrt_ifup;     /* I/F up (new IP address) callback */
  priv->dev.d_ifdown  = imxrt_ifdown;   /* I/F down callback */
  priv->dev.d_txavail = imxrt_txavail;  /* New TX data callback */
#ifdef CONFIG_NET_MCASTGROUP
  priv->dev.d_addmac  = imxrt_addmac;   /* Add multicast MAC address */
  priv->dev.d_rmmac   = imxrt_rmmac;    /* Remove multicast MAC address */
#endif
#ifdef CONFIG_NETDEV_IOCTL
  priv->dev.d_ioctl   = imxrt_ioctl;    /* Support PHY ioctl() calls */
#endif
  priv->dev.d_private = g_enet;         /* Used to recover private state from dev */

#ifdef CONFIG_NET_ETHERNET
  /* Determine a semi-unique MAC address from MCU UID
   * We use UID Low and Mid Low registers to get 64 bits, from which we keep
   * 48 bits.  We then force unicast and locally administered bits
   * (b0 and b1, 1st octet)
   */

  /* hardcoded offset: todo: need proper header file */

  uidl   = getreg32(IMXRT_OCOTP_BASE + 0x410);
  uidml  = getreg32(IMXRT_OCOTP_BASE + 0x420);
  mac    = priv->dev.d_mac.ether.ether_addr_octet;

  uidml |= 0x00000200;
  uidml &= 0x0000feff;

  mac[0] = (uidml & 0x0000ff00) >> 8;
  mac[1] = (uidml & 0x000000ff);
  mac[2] = (uidl &  0xff000000) >> 24;
  mac[3] = (uidl &  0x00ff0000) >> 16;
  mac[4] = (uidl &  0x0000ff00) >> 8;
  mac[5] = (uidl &  0x000000ff);
#endif

#ifdef CONFIG_IMXRT_ENET_PHYINIT
  /* Perform any necessary, one-time, board-specific PHY initialization */

  ret = imxrt_phy_boardinitialize(0);
  if (ret < 0)
    {
      nerr("ERROR: Failed to initialize the PHY: %d\n", ret);
      return ret;
    }
#endif

  /* Put the interface in the down state.  This usually amounts to resetting
   * the device and/or calling imxrt_ifdown().
   */

  imxrt_ifdown(&priv->dev);

  /* Register the device with the OS so that socket IOCTLs can be performed */

  netdev_register(&priv->dev, NET_LL_ETHERNET);

  UNUSED(ret);
  return OK;
}

/****************************************************************************
 * Name: arm_netinitialize
 *
 * Description:
 *   Initialize the first network interface.  If there are more than one
 *   interface in the chip, then board-specific logic will have to provide
 *   this function to determine which, if any, Ethernet controllers should
 *   be initialized.
 *
 ****************************************************************************/

#if CONFIG_IMXRT_ENET_NETHIFS == 1 && !defined(CONFIG_NETDEV_LATEINIT)
void arm_netinitialize(void)
{
  imxrt_netinitialize(0);
}
#endif

#endif /* CONFIG_IMXRT_ENET */
