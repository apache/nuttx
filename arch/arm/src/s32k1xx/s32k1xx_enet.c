/****************************************************************************
 * arch/arm/src/s32k1xx/s32k1xx_enet.c
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
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <endian.h>

#include <arpa/inet.h>

#include <nuttx/wdog.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/spinlock.h>
#include <nuttx/wqueue.h>
#include <nuttx/signal.h>
#include <nuttx/net/mii.h>
#include <nuttx/net/arp.h>
#include <nuttx/net/phy.h>
#include <nuttx/net/netdev.h>

#ifdef CONFIG_NET_PKT
#  include <nuttx/net/pkt.h>
#endif

#include "arm_internal.h"
#include "chip.h"
#include "s32k1xx_config.h"
#include "hardware/s32k1xx_enet.h"
#include "hardware/s32k1xx_pinmux.h"
#include "s32k1xx_periphclocks.h"
#include "s32k1xx_pin.h"
#include "s32k1xx_enet.h"

#ifdef CONFIG_S32K1XX_ENET

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

/* CONFIG_S32K1XX_ENET_NETHIFS determines the number of physical interfaces
 * that will be supported.
 */

#if CONFIG_S32K1XX_ENET_NETHIFS != 1
#  error "CONFIG_S32K1XX_ENET_NETHIFS must be one for now"
#endif

#if CONFIG_S32K1XX_ENET_NTXBUFFERS < 1
#  error "Need at least one TX buffer"
#endif

#if CONFIG_S32K1XX_ENET_NRXBUFFERS < 1
#  error "Need at least one RX buffer"
#endif

#define NENET_NBUFFERS \
  (CONFIG_S32K1XX_ENET_NTXBUFFERS+CONFIG_S32K1XX_ENET_NRXBUFFERS)

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

#define S32K1XX_TXTIMEOUT   (60*CLK_TCK)
#define MII_MAXPOLLS      (0x1ffff)
#define LINK_WAITUS       (500*1000)
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
 * The s32k1xx1050-evk board uses a KSZ8081 PHY
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
#elif defined(CONFIG_ETH0_PHY_TJA1101)
#  define BOARD_PHY_NAME        "TJA1101"
#  define BOARD_PHYID1          MII_PHYID1_TJA1101
#  define BOARD_PHYID2          MII_PHYID2_TJA1101
#  define BOARD_PHY_STATUS      MII_TJA110X_BSR
#  define BOARD_PHY_ADDR        (0)
#  define BOARD_PHY_10BASET(s)  0 /* PHY only supports 100BASE-T1 */
#  define BOARD_PHY_100BASET(s) 1 /* PHY only supports 100BASE-T1 */
#  define BOARD_PHY_ISDUPLEX(s) 1 /* PHY only supports fullduplex */
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

#define S32K1XX_MII_SPEED  0x0f /* 100Mbs. Revisit and remove hardcoded value */
#if S32K1XX_MII_SPEED > 63
#  error "S32K1XX_MII_SPEED is out-of-range"
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

#define CRITICAL_ERROR    (ENET_INT_UN | ENET_INT_RL | ENET_INT_EBERR )

/* This is a helper pointer for accessing the contents of Ethernet header */

#define BUF ((struct eth_hdr_s *)priv->dev.d_buf)

#define S32K1XX_BUF_SIZE  ENET_ALIGN_UP(CONFIG_NET_ETH_PKTSIZE)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The s32k1xx_driver_s encapsulates all state information for a single
 * hardware interface
 */

struct s32k1xx_driver_s
{
  bool bifup;                  /* true:ifup false:ifdown */
  uint8_t txtail;              /* The oldest busy TX descriptor */
  uint8_t txhead;              /* The next TX descriptor to use */
  uint8_t rxtail;              /* The next RX descriptor to use */
  uint8_t phyaddr;             /* Selected PHY address */
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

static struct s32k1xx_driver_s g_enet[CONFIG_S32K1XX_ENET_NETHIFS];

/* The DMA descriptors.  A unaligned uint8_t is used to allocate the
 * memory; 16 is added to assure that we can meet the descriptor alignment
 * requirements.
 */

static uint8_t g_desc_pool[NENET_NBUFFERS * sizeof(struct enet_desc_s)]
               aligned_data(ENET_ALIGN);

/* The DMA buffers.  Again, A unaligned uint8_t is used to allocate the
 * memory; 16 is added to assure that we can meet the descriptor alignment
 * requirements.
 */

static uint8_t g_buffer_pool[NENET_NBUFFERS * S32K1XX_BUF_SIZE]
               aligned_data(ENET_ALIGN);

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Utility functions */

#ifndef S32K1XX_BUFFERS_SWAP
#  define s32k1xx_swap32(value) (value)
#  define s32k1xx_swap16(value) (value)
#else
#if 0 /* Use builtins if the compiler supports them */
static inline uint32_t s32k1xx_swap32(uint32_t value);
static inline uint16_t s32k1xx_swap16(uint16_t value);
#else
#  define s32k1xx_swap32 swap32
#  define s32k1xx_swap16 swap16
#endif
#endif

/* Common TX logic */

static bool s32k1xx_txringfull(struct s32k1xx_driver_s *priv);
static int  s32k1xx_transmit(struct s32k1xx_driver_s *priv);
static int  s32k1xx_txpoll(struct net_driver_s *dev);

/* Interrupt handling */

static void s32k1xx_dispatch(struct s32k1xx_driver_s *priv);
static void s32k1xx_receive(struct s32k1xx_driver_s *priv);
static void s32k1xx_txdone(struct s32k1xx_driver_s *priv);

static void s32k1xx_enet_interrupt_work(void *arg);
static int  s32k1xx_enet_interrupt(int irq, void *context,
                                   void *arg);

/* Watchdog timer expirations */

static void s32k1xx_txtimeout_work(void *arg);
static void s32k1xx_txtimeout_expiry(wdparm_t arg);

/* NuttX callback functions */

static int  s32k1xx_ifup(struct net_driver_s *dev);
static int  s32k1xx_ifdown(struct net_driver_s *dev);

static void s32k1xx_txavail_work(void *arg);
static int  s32k1xx_txavail(struct net_driver_s *dev);

/* Internal ifup function that allows phy reset to be optional */

static int s32k1xx_ifup_action(struct net_driver_s *dev, bool resetphy);

#ifdef CONFIG_NET_MCASTGROUP
static int  s32k1xx_addmac(struct net_driver_s *dev, const uint8_t *mac);
static int  s32k1xx_rmmac(struct net_driver_s *dev, const uint8_t *mac);
#endif

#ifdef CONFIG_NETDEV_IOCTL
static int  s32k1xx_ioctl(struct net_driver_s *dev, int cmd,
            unsigned long arg);
#endif

/* PHY/MII support */

#if defined(CONFIG_NETDEV_PHY_IOCTL) && defined(CONFIG_ARCH_PHY_INTERRUPT)
static int s32k1xx_phyintenable(struct s32k1xx_driver_s *priv);
#endif
static inline void s32k1xx_initmii(struct s32k1xx_driver_s *priv);
static int s32k1xx_writemii(struct s32k1xx_driver_s *priv, uint8_t phyaddr,
             uint8_t regaddr, uint16_t data);
static int s32k1xx_readmii(struct s32k1xx_driver_s *priv, uint8_t phyaddr,
             uint8_t regaddr, uint16_t *data);
static int s32k1xx_initphy(struct s32k1xx_driver_s *priv, bool renogphy);

/* Initialization */

static void s32k1xx_initbuffers(struct s32k1xx_driver_s *priv);
static void s32k1xx_reset(struct s32k1xx_driver_s *priv);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: s32k1xx_swap16/32
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
static inline uint32_t s32k1xx_swap32(uint32_t value)
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

static inline uint16_t s32k1xx_swap16(uint16_t value)
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
 * Function: s32k1xx_txringfull
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

static bool s32k1xx_txringfull(struct s32k1xx_driver_s *priv)
{
  uint8_t txnext;

  /* Check if there is room in the hardware to hold another outgoing
   * packet.  The ring is full if incrementing the head pointer would
   * collide with the tail pointer.
   */

  txnext = priv->txhead + 1;
  if (txnext >= CONFIG_S32K1XX_ENET_NTXBUFFERS)
    {
      txnext = 0;
    }

  return priv->txtail == txnext;
}

/****************************************************************************
 * Function: s32k1xx_transmit
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

static int s32k1xx_transmit(struct s32k1xx_driver_s *priv)
{
  struct enet_desc_s *txdesc;
  irqstate_t flags;
  uint32_t regval;
  uint8_t *buf;

  /* Since this can be called from s32k1xx_receive, it is possible that
   * the transmit queue is full, so check for that now.  If this is the
   * case, the outgoing packet will be dropped (e.g. an ARP reply)
   */

  if (s32k1xx_txringfull(priv))
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
  if (priv->txhead >= CONFIG_S32K1XX_ENET_NTXBUFFERS)
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

  txdesc->length   = s32k1xx_swap16(priv->dev.d_len);
#ifdef CONFIG_S32K1XX_ENETENHANCEDBD
  txdesc->bdu      = 0x00000000;
  txdesc->status2  = TXDESC_INT | TXDESC_TS; /* | TXDESC_IINS | TXDESC_PINS; */
#endif
  txdesc->status1 |= (TXDESC_R | TXDESC_L | TXDESC_TC);

  buf = (uint8_t *)s32k1xx_swap32((uint32_t)priv->dev.d_buf);
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

  flags = spin_lock_irqsave(NULL);

  /* Enable TX interrupts */

  regval  = getreg32(S32K1XX_ENET_EIMR);
  regval |= TX_INTERRUPTS;
  putreg32(regval, S32K1XX_ENET_EIMR);

  /* Setup the TX timeout watchdog (perhaps restarting the timer) */

  wd_start(&priv->txtimeout, S32K1XX_TXTIMEOUT,
           s32k1xx_txtimeout_expiry, (wdparm_t)priv);

  /* Start the TX transfer (if it was not already waiting for buffers) */

  putreg32(ENET_TDAR, S32K1XX_ENET_TDAR);

  spin_unlock_irqrestore(NULL, flags);
  return OK;
}

/****************************************************************************
 * Function: s32k1xx_txpoll
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

static int s32k1xx_txpoll(struct net_driver_s *dev)
{
  struct s32k1xx_driver_s *priv =
    (struct s32k1xx_driver_s *)dev->d_private;

  /* Send the packet */

  s32k1xx_transmit(priv);
  priv->dev.d_buf = (uint8_t *)
    s32k1xx_swap32((uint32_t)priv->txdesc[priv->txhead].data);

  /* Check if there is room in the device to hold another packet. If
   * not, return a non-zero value to terminate the poll.
   */

  if (s32k1xx_txringfull(priv))
    {
      return -EBUSY;
    }

  /* If zero is returned, the polling will continue until all connections
   * have been examined.
   */

  return 0;
}

/****************************************************************************
 * Function: s32k1xx_dispatch
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

static inline void s32k1xx_dispatch(struct s32k1xx_driver_s *priv)
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

      /* Receive an IPv4 packet from the network device */

      ipv4_input(&priv->dev);

      /* If the above function invocation resulted in data that should be
       * sent out on the network, d_len field will set to a value > 0.
       */

      if (priv->dev.d_len > 0)
        {
          /* And send the packet */

          s32k1xx_transmit(priv);
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
          /* And send the packet */

          s32k1xx_transmit(priv);
        }
    }
  else
#endif
#ifdef CONFIG_NET_ARP
  /* Check for an ARP packet */

  if (BUF->type == HTONS(ETHTYPE_ARP))
    {
      NETDEV_RXARP(&priv->dev);
      arp_arpin(&priv->dev);

      /* If the above function invocation resulted in data that should
       * be sent out on the network, the field  d_len will set to a
       * value > 0.
       */

      if (priv->dev.d_len > 0)
        {
          s32k1xx_transmit(priv);
        }
    }
#endif
  else
    {
      NETDEV_RXDROPPED(&priv->dev);
    }
}

/****************************************************************************
 * Function: s32k1xx_receive
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

static void s32k1xx_receive(struct s32k1xx_driver_s *priv)
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

          priv->dev.d_len = s32k1xx_swap16(rxdesc->length);
          priv->dev.d_buf =
            (uint8_t *)s32k1xx_swap32((uint32_t)rxdesc->data);

          /* Invalidate the buffer so that the correct packet will be re-read
           * from memory when the packet content is accessed.
           */

          up_invalidate_dcache((uintptr_t)priv->dev.d_buf,
                               (uintptr_t)priv->dev.d_buf + priv->dev.d_len);

          /* Dispatch (or drop) the newly received packet */

          s32k1xx_dispatch(priv);

          /* Point the packet buffer back to the next Tx buffer that will be
           * used during the next write.  If the write queue is full, then
           * this will point at an active buffer, which must not be written
           * to.  This is OK because devif_poll won't be called unless the
           * queue is not full.
           */

          priv->dev.d_buf = (uint8_t *)
            s32k1xx_swap32((uint32_t)priv->txdesc[priv->txhead].data);
          rxdesc->status1 |= RXDESC_E;

          /* Update the index to the next descriptor */

          priv->rxtail++;
          if (priv->rxtail >= CONFIG_S32K1XX_ENET_NRXBUFFERS)
            {
              priv->rxtail = 0;
            }

          /* Indicate that there have been empty receive buffers produced */

          putreg32(ENET_RDAR, S32K1XX_ENET_RDAR);
        }
    }
  while (received);
}

/****************************************************************************
 * Function: s32k1xx_txdone
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

static void s32k1xx_txdone(struct s32k1xx_driver_s *priv)
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
          if (priv->txtail >= CONFIG_S32K1XX_ENET_NTXBUFFERS)
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

      regval  = getreg32(S32K1XX_ENET_EIMR);
      regval &= ~TX_INTERRUPTS;
      putreg32(regval, S32K1XX_ENET_EIMR);
    }

  /* There should be space for a new TX in any event.  Poll the network for
   * new XMIT data
   */

  devif_poll(&priv->dev, s32k1xx_txpoll);
}

/****************************************************************************
 * Function: s32k1xx_enet_interrupt_work
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

static void s32k1xx_enet_interrupt_work(void *arg)
{
  struct s32k1xx_driver_s *priv = (struct s32k1xx_driver_s *)arg;
  uint32_t pending;
#ifdef CONFIG_NET_MCASTGROUP
  uint32_t gaurstore;
  uint32_t galrstore;
#endif

  /* Process pending Ethernet interrupts */

  net_lock();

  /* Get the set of unmasked, pending interrupt. */

  pending = getreg32(S32K1XX_ENET_EIR) & getreg32(S32K1XX_ENET_EIMR);

  /* Clear the pending interrupts */

  putreg32(pending, S32K1XX_ENET_EIR);

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

      gaurstore = getreg32(S32K1XX_ENET_GAUR);
      galrstore = getreg32(S32K1XX_ENET_GALR);
#endif

      s32k1xx_ifdown(&priv->dev);
      s32k1xx_ifup_action(&priv->dev, false);

#ifdef CONFIG_NET_MCASTGROUP
      /* Now write the multicast table back */

      putreg32(gaurstore, S32K1XX_ENET_GAUR);
      putreg32(galrstore, S32K1XX_ENET_GALR);
#endif

      /* Then poll the network for new XMIT data */

      devif_poll(&priv->dev, s32k1xx_txpoll);
    }
  else
    {
      /* Check for the receipt of a packet */

      if ((pending & ENET_INT_RXF) != 0)
        {
          /* A packet has been received, call s32k1xx_receive() to handle the
           * packet.
           */

          s32k1xx_receive(priv);
        }

      /* Check if a packet transmission has completed */

      if ((pending & ENET_INT_TXF) != 0)
        {
          /* Call s32k1xx_txdone to handle the end of transfer even.  NOTE
           * that this may disable further Tx interrupts if there are no
           * pending transmissions.
           */

          s32k1xx_txdone(priv);
        }
    }

  net_unlock();

  /* Re-enable Ethernet interrupts */

#if 0
  up_enable_irq(S32K1XX_IRQ_EMACTMR);
#endif
  up_enable_irq(S32K1XX_IRQ_ENET_TXDONE);
  up_enable_irq(S32K1XX_IRQ_ENET_RXDONE);
}

/****************************************************************************
 * Function: s32k1xx_enet_interrupt
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

static int s32k1xx_enet_interrupt(int irq, void *context, void *arg)
{
  register struct s32k1xx_driver_s *priv = &g_enet[0];

  /* Disable further Ethernet interrupts.  Because Ethernet interrupts are
   * also disabled if the TX timeout event occurs, there can be no race
   * condition here.
   */

  up_disable_irq(S32K1XX_IRQ_ENET_TXDONE);
  up_disable_irq(S32K1XX_IRQ_ENET_RXDONE);

  /* Schedule to perform the interrupt processing on the worker thread. */

  work_queue(ETHWORK, &priv->irqwork, s32k1xx_enet_interrupt_work, priv, 0);
  return OK;
}

/****************************************************************************
 * Function: s32k1xx_txtimeout_work
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

static void s32k1xx_txtimeout_work(void *arg)
{
  struct s32k1xx_driver_s *priv = (struct s32k1xx_driver_s *)arg;

  /* Increment statistics and dump debug info */

  net_lock();
  nerr("Resetting interface\n");

  NETDEV_TXTIMEOUTS(&priv->dev);

  /* Take the interface down and bring it back up.  That is the most
   * aggressive hardware reset.
   */

  s32k1xx_ifdown(&priv->dev);
  s32k1xx_ifup_action(&priv->dev, false);

  /* Then poll the network for new XMIT data */

  devif_poll(&priv->dev, s32k1xx_txpoll);
  net_unlock();
}

/****************************************************************************
 * Function: s32k1xx_txtimeout_expiry
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

static void s32k1xx_txtimeout_expiry(wdparm_t arg)
{
  struct s32k1xx_driver_s *priv = (struct s32k1xx_driver_s *)arg;

  /* Disable further Ethernet interrupts.  This will prevent some race
   * conditions with interrupt work.  There is still a potential race
   * condition with interrupt work that is already queued and in progress.
   */

  up_disable_irq(S32K1XX_IRQ_ENET_TXDONE);
  up_disable_irq(S32K1XX_IRQ_ENET_RXDONE);

  /* Schedule to perform the TX timeout processing on the worker thread,
   * canceling any pending interrupt work.
   */

  work_queue(ETHWORK, &priv->irqwork, s32k1xx_txtimeout_work, priv, 0);
}

/****************************************************************************
 * Function: s32k1xx_ifup_action
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

static int s32k1xx_ifup_action(struct net_driver_s *dev, bool resetphy)
{
  struct s32k1xx_driver_s *priv =
    (struct s32k1xx_driver_s *)dev->d_private;
  uint8_t *mac = dev->d_mac.ether.ether_addr_octet;
  uint32_t regval;
  int ret;

  ninfo("Bringing up: %d.%d.%d.%d\n",
        (int)(dev->d_ipaddr & 0xff), (int)((dev->d_ipaddr >> 8) & 0xff),
        (int)((dev->d_ipaddr >> 16) & 0xff), (int)(dev->d_ipaddr >> 24));

  /* Initialize ENET buffers */

  s32k1xx_initbuffers(priv);

  /* Configure the MII interface */

  s32k1xx_initmii(priv);

  /* Set the MAC address */

  putreg32((mac[0] << 24) | (mac[1] << 16) | (mac[2] << 8) | mac[3],
           S32K1XX_ENET_PALR);
  putreg32((mac[4] << 24) | (mac[5] << 16), S32K1XX_ENET_PAUR);

  /* Configure the PHY */

  ret = s32k1xx_initphy(priv, resetphy);
  if (ret < 0)
    {
      nerr("ERROR: Failed to configure the PHY: %d\n", ret);
      return ret;
    }

  /* Handle promiscuous mode */

#ifdef CONFIG_NET_PROMISCUOUS
  regval = getreg32(S32K1XX_ENET_RCR);
  regval |= ENET_RCR_PROM;
  putreg32(regval, S32K1XX_ENET_RCR);
#endif

  /* Select legacy of enhanced buffer descriptor format */

#ifdef CONFIG_S32K1XX_ENETENHANCEDBD
  putreg32(ENET_ECR_EN1588, S32K1XX_ENET_ECR);
#else
  putreg32(0, S32K1XX_ENET_ECR);
#endif

  /* Set the RX buffer size */

  putreg32(S32K1XX_BUF_SIZE, S32K1XX_ENET_MRBR);

  /* Point to the start of the circular RX buffer descriptor queue */

  putreg32((uint32_t)priv->rxdesc, S32K1XX_ENET_RDSR);

  /* Point to the start of the circular TX buffer descriptor queue */

  putreg32((uint32_t)priv->txdesc, S32K1XX_ENET_TDSR);

  /* And enable the MAC itself */

  regval  = getreg32(S32K1XX_ENET_ECR);
  regval |= ENET_ECR_ETHEREN
#ifdef S32K1XX_USE_DBSWAP
         | ENET_ECR_DBSWP
#endif
        ;
  putreg32(regval, S32K1XX_ENET_ECR);

  /* Indicate that there have been empty receive buffers produced */

  putreg32(ENET_RDAR, S32K1XX_ENET_RDAR);

  /* Clear all pending ENET interrupt */

  putreg32(RX_INTERRUPTS | ERROR_INTERRUPTS | TX_INTERRUPTS,
           S32K1XX_ENET_EIR);

  /* Enable RX and error interrupts at the controller (TX interrupts are
   * still disabled).
   */

  putreg32(RX_INTERRUPTS | ERROR_INTERRUPTS,
           S32K1XX_ENET_EIMR);

  /* Mark the interrupt "up" and enable interrupts at the NVIC */

  priv->bifup = true;

#if 0
  up_enable_irq(S32K1XX_IRQ_EMACTMR);
#endif
  up_enable_irq(S32K1XX_IRQ_ENET_TXDONE);
  up_enable_irq(S32K1XX_IRQ_ENET_RXDONE);

  return OK;
}

/****************************************************************************
 * Function: s32k1xx_ifup
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

static int s32k1xx_ifup(struct net_driver_s *dev)
{
  /* The externally available ifup action includes resetting the phy */

  return s32k1xx_ifup_action(dev, true);
}

/****************************************************************************
 * Function: s32k1xx_ifdown
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

static int s32k1xx_ifdown(struct net_driver_s *dev)
{
  struct s32k1xx_driver_s *priv =
    (struct s32k1xx_driver_s *)dev->d_private;
  irqstate_t flags;

  ninfo("Taking down: %d.%d.%d.%d\n",
        (int)(dev->d_ipaddr & 0xff), (int)((dev->d_ipaddr >> 8) & 0xff),
        (int)((dev->d_ipaddr >> 16) & 0xff), (int)(dev->d_ipaddr >> 24));

  /* Flush and disable the Ethernet interrupts at the NVIC */

  flags = enter_critical_section();

  up_disable_irq(S32K1XX_IRQ_ENET_TXDONE);
  up_disable_irq(S32K1XX_IRQ_ENET_RXDONE);
  putreg32(0, S32K1XX_ENET_EIMR);

  /* Cancel the TX timeout timers */

  wd_cancel(&priv->txtimeout);

  /* Put the EMAC in its reset, non-operational state.  This should be
   * a known configuration that will guarantee the s32k1xx_ifup() always
   * successfully brings the interface back up.
   */

  s32k1xx_reset(priv);

  /* Mark the device "down" */

  priv->bifup = false;
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Function: s32k1xx_txavail_work
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

static void s32k1xx_txavail_work(void *arg)
{
  struct s32k1xx_driver_s *priv = (struct s32k1xx_driver_s *)arg;

  /* Ignore the notification if the interface is not yet up */

  net_lock();
  if (priv->bifup)
    {
      /* Check if there is room in the hardware to hold another outgoing
       * packet.
       */

      if (!s32k1xx_txringfull(priv))
        {
          /* No, there is space for another transfer.  Poll the network for
           * new XMIT data.
           */

          devif_poll(&priv->dev, s32k1xx_txpoll);
        }
    }

  net_unlock();
}

/****************************************************************************
 * Function: s32k1xx_txavail
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

static int s32k1xx_txavail(struct net_driver_s *dev)
{
  struct s32k1xx_driver_s *priv =
    (struct s32k1xx_driver_s *)dev->d_private;

  /* Is our single work structure available?  It may not be if there are
   * pending interrupt actions and we will have to ignore the Tx
   * availability action.
   */

  if (work_available(&priv->pollwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      work_queue(ETHWORK, &priv->pollwork, s32k1xx_txavail_work, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Function: s32k1xx_calcethcrc
 *
 * Description:
 *   Function to calculate the CRC used by S32K1XX to check an Ethernet frame
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
static uint32_t s32k1xx_calcethcrc(const uint8_t *data, size_t length)
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
 * Function: s32k1xx_enet_hash_index
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
static uint32_t s32k1xx_enet_hash_index(const uint8_t *mac)
{
  uint32_t crc;
  uint32_t hashindex;

  ninfo("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  crc = s32k1xx_calcethcrc(mac, 6);
  hashindex = (crc >> 26) & 0x3f;

  return hashindex;
}
#endif

/****************************************************************************
 * Function: s32k1xx_addmac
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
static int s32k1xx_addmac(struct net_driver_s *dev, const uint8_t *mac)
{
  uint32_t crc;
  uint32_t hashindex;
  uint32_t temp;
  uint32_t registeraddress;

  hashindex = s32k1xx_enet_hash_index(mac);

  /* Add the MAC address to the hardware multicast routing table */

  if (hashindex > 31)
    {
      registeraddress = S32K1XX_ENET_GAUR;
      hashindex      -= 32;
    }
  else
    {
      registeraddress = S32K1XX_ENET_GALR;
    }

  temp  = getreg32(registeraddress);
  temp |= 1 << hashindex;
  putreg32(temp, registeraddress);

  return OK;
}
#endif

/****************************************************************************
 * Function: s32k1xx_rmmac
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
static int s32k1xx_rmmac(struct net_driver_s *dev, const uint8_t *mac)
{
  uint32_t crc;
  uint32_t hashindex;
  uint32_t temp;
  uint32_t registeraddress;

  /* Remove the MAC address from the hardware multicast routing table */

  hashindex = s32k1xx_enet_hash_index(mac);

  if (hashindex > 31)
    {
      registeraddress = S32K1XX_ENET_GAUR;
      hashindex      -= 32;
    }
  else
    {
      registeraddress = S32K1XX_ENET_GALR;
    }

  temp  = getreg32(registeraddress);
  temp &= ~(1 << hashindex);
  putreg32(temp, registeraddress);

  return OK;
}
#endif

/****************************************************************************
 * Function: s32k1xx_ioctl
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
static int s32k1xx_ioctl(struct net_driver_s *dev, int cmd,
                         unsigned long arg)
{
#ifdef CONFIG_NETDEV_PHY_IOCTL
  struct s32k1xx_driver_s *priv =
    (struct s32k1xx_driver_s *)dev->d_private;
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

              ret = s32k1xx_phyintenable(priv);
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
          ret =
            s32k1xx_readmii(priv, req->phy_id, req->reg_num, &req->val_out);
        }
        break;

      case SIOCSMIIREG: /* Set register in MII PHY */
        {
          struct mii_ioctl_data_s *req =
            (struct mii_ioctl_data_s *)((uintptr_t)arg);
          ret =
            s32k1xx_writemii(priv, req->phy_id, req->reg_num, req->val_in);
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
 * Function: s32k1xx_phyintenable
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
static int s32k1xx_phyintenable(struct s32k1xx_driver_s *priv)
{
#if defined(CONFIG_ETH0_PHY_KSZ8051) || defined(CONFIG_ETH0_PHY_KSZ8061) || \
    defined(CONFIG_ETH0_PHY_KSZ8081)
  uint16_t phyval;
  int ret;

  /* Read the interrupt status register in order to clear any pending
   * interrupts
   */

  ret = s32k1xx_readmii(priv, priv->phyaddr, MII_KSZ8081_INT, &phyval);
  if (ret == OK)
    {
      /* Enable link up/down interrupts */

      ret = s32k1xx_writemii(priv, priv->phyaddr, MII_KSZ8081_INT,
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
 * Function: s32k1xx_initmii
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

static void s32k1xx_initmii(struct s32k1xx_driver_s *priv)
{
  /* Speed is based on the peripheral (bus) clock; hold time is 2 module
   * clock.  This hold time value may need to be increased on some platforms
   */

  putreg32(S32K1XX_MII_SPEED << ENET_MSCR_MII_SPEED_SHIFT,
           S32K1XX_ENET_MSCR);
}

/****************************************************************************
 * Function: s32k1xx_writemii
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

static int s32k1xx_writemii(struct s32k1xx_driver_s *priv, uint8_t phyaddr,
                            uint8_t regaddr, uint16_t data)
{
  int timeout;

  /* Clear the MII interrupt bit */

  putreg32(ENET_INT_MII, S32K1XX_ENET_EIR);

  /* Initiate the MII Management write */

  putreg32(data |
           2 << ENET_MMFR_TA_SHIFT |
           (uint32_t)regaddr << ENET_MMFR_RA_SHIFT |
           (uint32_t)phyaddr << ENET_MMFR_PA_SHIFT |
           ENET_MMFR_OP_WRMII |
           1 << ENET_MMFR_ST_SHIFT,
           S32K1XX_ENET_MMFR);

  /* Wait for the transfer to complete */

  for (timeout = 0; timeout < MII_MAXPOLLS; timeout++)
    {
      if ((getreg32(S32K1XX_ENET_EIR) & ENET_INT_MII) != 0)
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

  putreg32(ENET_INT_MII, S32K1XX_ENET_EIR);
  return OK;
}

/****************************************************************************
 * Function: s32k1xx_reademii
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

static int s32k1xx_readmii(struct s32k1xx_driver_s *priv, uint8_t phyaddr,
                           uint8_t regaddr, uint16_t *data)
{
  int timeout;

  /* Clear the MII interrupt bit */

  putreg32(ENET_INT_MII, S32K1XX_ENET_EIR);

  /* Initiate the MII Management read */

  putreg32(2 << ENET_MMFR_TA_SHIFT |
           (uint32_t)regaddr << ENET_MMFR_RA_SHIFT |
           (uint32_t)phyaddr << ENET_MMFR_PA_SHIFT |
           ENET_MMFR_OP_RDMII |
           1 << ENET_MMFR_ST_SHIFT,
           S32K1XX_ENET_MMFR);

  /* Wait for the transfer to complete */

  for (timeout = 0; timeout < MII_MAXPOLLS; timeout++)
    {
      if ((getreg32(S32K1XX_ENET_EIR) & ENET_INT_MII) != 0)
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

  putreg32(ENET_INT_MII, S32K1XX_ENET_EIR);

  /* And return the MII data */

  *data = (uint16_t)(getreg32(S32K1XX_ENET_MMFR) & ENET_MMFR_DATA_MASK);
  return OK;
}

/****************************************************************************
 * Function: s32k1xx_initphy
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

static inline int s32k1xx_initphy(struct s32k1xx_driver_s *priv,
                                  bool renogphy)
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
       * - we are not communicating with it's 'special' registers at this
       * point.
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
          ret     = s32k1xx_readmii(priv, phyaddr, MII_PHYID1, &phydata);
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

      ret = s32k1xx_readmii(priv, phyaddr, MII_PHYID2, &phydata);
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

      s32k1xx_writemii(priv, phyaddr, MII_MCR, MII_MCR_RESET);

      /* Set RMII mode */

      ret = s32k1xx_readmii(priv, phyaddr, MII_KSZ8081_PHYCTRL2, &phydata);
      if (ret < 0)
        {
          nerr("ERROR: Failed to read MII_KSZ8081_PHYCTRL2\n");
          return ret;
        }

      /* Indicate 50MHz clock */

      s32k1xx_writemii(priv, phyaddr, MII_KSZ8081_PHYCTRL2,
                     (phydata | (1 << 7)));

      /* Switch off NAND Tree mode (in case it was set via pinning) */

      ret = s32k1xx_readmii(priv, phyaddr, MII_KSZ8081_OMSO, &phydata);
      if (ret < 0)
        {
          nerr("ERROR: Failed to read MII_KSZ8081_OMSO: %d\n", ret);
          return ret;
        }

      s32k1xx_writemii(priv, phyaddr, MII_KSZ8081_OMSO,
                     (phydata & ~(1 << 5)));

      /* Set Ethernet led to green = activity and yellow = link and  */

      ret = s32k1xx_readmii(priv, phyaddr, MII_KSZ8081_PHYCTRL2, &phydata);
      if (ret < 0)
        {
          nerr("ERROR: Failed to read MII_KSZ8081_PHYCTRL2\n");
          return ret;
        }

      s32k1xx_writemii(priv, phyaddr, MII_KSZ8081_PHYCTRL2,
                     (phydata | (1 << 4)));

#elif defined (CONFIG_ETH0_PHY_LAN8720)

      /* Make sure that PHY comes up in correct mode when it's reset */

      s32k1xx_writemii(priv, phyaddr, MII_LAN8720_MODES,
             MII_LAN8720_MODES_RESV | MII_LAN8720_MODES_ALL |
             MII_LAN8720_MODES_PHYAD(BOARD_PHY_ADDR));

      /* ...and reset PHY */

      s32k1xx_writemii(priv, phyaddr, MII_MCR, MII_MCR_RESET);
#endif

      /* Start auto negotiation */

      ninfo("%s: Start Autonegotiation...\n",  BOARD_PHY_NAME);
      s32k1xx_writemii(priv, phyaddr, MII_MCR,
                     (MII_MCR_ANRESTART | MII_MCR_ANENABLE));

      /* Wait for auto negotiation to complete */

      for (retries = 0; retries < LINK_NLOOPS; retries++)
        {
          ret = s32k1xx_readmii(priv, phyaddr, MII_MSR, &phydata);
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

          s32k1xx_writemii(priv, phyaddr, MII_MCR, 0);
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
      ret = s32k1xx_readmii(priv, phyaddr, BOARD_PHY_STATUS, &phydata);
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

          return s32k1xx_initphy(priv, true);
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

#ifdef CONFIG_S32K1XX_ENETUSEMII
  rcr = ENET_RCR_CRCFWD |
        CONFIG_NET_ETH_PKTSIZE << ENET_RCR_MAX_FL_SHIFT |
        ENET_RCR_MII_MODE;
#else
  rcr = ENET_RCR_RMII_MODE | ENET_RCR_CRCFWD |
        CONFIG_NET_ETH_PKTSIZE << ENET_RCR_MAX_FL_SHIFT |
        ENET_RCR_MII_MODE;
#endif
  tcr = 0;

  putreg32(rcr, S32K1XX_ENET_RCR);
  putreg32(tcr, S32K1XX_ENET_TCR);

  /* Do not Discard Of Frames With MAC Layer Errors.
   * Enable Discard Of Frames With Wrong Protocol Checksum.
   * Bit 1: Enable discard of frames with wrong IPv4 header checksum.
   */

  racc = ENET_RACC_PRODIS | ENET_RACC_IPDIS;
  putreg32(racc, S32K1XX_ENET_RACC);

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

#if defined(CONFIG_ETH0_PHY_TJA1101)
/* The NXP TJA110X PHY is an automotive 100BASE-T1 PHY
 * Which requires additional initialization
 */

  /* select mode TJA110X */

  s32k1xx_writemii(priv, phyaddr, MII_TJA110X_EXT_CNTRL,
                  (MII_EXT_CNTRL_NORMAL | MII_EXT_CNTRL_CONFIG_EN |
                  MII_EXT_CNTRL_CONFIG_INH));

#  if defined(CONFIG_PHY_100BASE_T1_MASTER)
  /* Set TJA110X in master mode */

  s32k1xx_writemii(priv, phyaddr, MII_TJA110X_CONFIG1,
                  (MII_CONFIG1_MASTER | MII_CONFIG1_TX_1250MV |
                  MII_CONFIG1_RMII_25MHZ | MII_CONFIG1_LED_EN));
#  else
  /* Set TJA110X in slave mode */

  s32k1xx_writemii(priv, phyaddr, MII_TJA110X_CONFIG1,
                  (MII_CONFIG1_TX_1250MV | MII_CONFIG1_RMII_25MHZ |
                  MII_CONFIG1_LED_EN));
#  endif

  s32k1xx_writemii(priv, phyaddr, MII_TJA110X_CONFIG2,
                  (MII_CONFIG2_SNR_AV64 | MII_CONFIG2_WLIM_D |
                   MII_CONFIG2_SNR_F_NL | MII_CONFIG2_SLP_T_1));

  /* Select normal mode TJA110X */

  s32k1xx_writemii(priv, phyaddr, MII_TJA110X_EXT_CNTRL,
                  (MII_EXT_CNTRL_NORMAL | MII_EXT_CNTRL_CONFIG_INH));

  s32k1xx_writemii(priv, phyaddr, MII_TJA110X_EXT_CNTRL,
                  (MII_EXT_CNTRL_LINK_CNTRL | MII_EXT_CNTRL_NORMAL |
                  MII_EXT_CNTRL_CONFIG_INH));
#endif

  putreg32(rcr, S32K1XX_ENET_RCR);
  putreg32(tcr, S32K1XX_ENET_TCR);
  return OK;
}

/****************************************************************************
 * Function: s32k1xx_initbuffers
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

static void s32k1xx_initbuffers(struct s32k1xx_driver_s *priv)
{
  uintptr_t addr;
  int i;

  /* Get an aligned TX descriptor (array) address */

  addr         = (uintptr_t)g_desc_pool;
  priv->txdesc = (struct enet_desc_s *)addr;

  /* Get an aligned RX descriptor (array) address */

  addr        +=  CONFIG_S32K1XX_ENET_NTXBUFFERS *
                  sizeof(struct enet_desc_s);
  priv->rxdesc = (struct enet_desc_s *)addr;

  /* Get the beginning of the first aligned buffer */

  addr         = (uintptr_t)g_buffer_pool;

  /* Then fill in the TX descriptors */

  for (i = 0; i < CONFIG_S32K1XX_ENET_NTXBUFFERS; i++)
    {
      priv->txdesc[i].status1 = 0;
      priv->txdesc[i].length  = 0;
      priv->txdesc[i].data    = (uint8_t *)s32k1xx_swap32((uint32_t)addr);
#ifdef CONFIG_S32K1XX_ENETENHANCEDBD
      priv->txdesc[i].status2 = TXDESC_IINS | TXDESC_PINS;
#endif
      addr                   += S32K1XX_BUF_SIZE;
    }

  /* Then fill in the RX descriptors */

  for (i = 0; i < CONFIG_S32K1XX_ENET_NRXBUFFERS; i++)
    {
      priv->rxdesc[i].status1 = RXDESC_E;
      priv->rxdesc[i].length  = 0;
      priv->rxdesc[i].data    = (uint8_t *)s32k1xx_swap32((uint32_t)addr);
#ifdef CONFIG_S32K1XX_ENETENHANCEDBD
      priv->rxdesc[i].bdu     = 0;
      priv->rxdesc[i].status2 = RXDESC_INT;
#endif
      addr                   += S32K1XX_BUF_SIZE;
    }

  /* Set the wrap bit in the last descriptors to form a ring */

  priv->txdesc[CONFIG_S32K1XX_ENET_NTXBUFFERS - 1].status1 |= TXDESC_W;
  priv->rxdesc[CONFIG_S32K1XX_ENET_NRXBUFFERS - 1].status1 |= RXDESC_W;

  /* We start with RX descriptor 0 and with no TX descriptors in use */

  priv->txtail = 0;
  priv->txhead = 0;
  priv->rxtail = 0;

  /* Initialize the packet buffer, which is used when sending */

  priv->dev.d_buf =
    (uint8_t *)s32k1xx_swap32((uint32_t)priv->txdesc[priv->txhead].data);
}

/****************************************************************************
 * Function: s32k1xx_reset
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

static void s32k1xx_reset(struct s32k1xx_driver_s *priv)
{
  unsigned int i;

  /* Set the reset bit and clear the enable bit */

  putreg32(ENET_ECR_RESET, S32K1XX_ENET_ECR);

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
 * Function: s32k1xx_netinitialize
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

int s32k1xx_netinitialize(int intf)
{
  struct s32k1xx_driver_s *priv;
#ifdef CONFIG_NET_ETHERNET
  uint32_t uidl;
  uint32_t uidml;
  uint8_t *mac;
#endif
  int ret;

  /* Get the interface structure associated with this interface number. */

  DEBUGASSERT(intf < CONFIG_S32K1XX_ENET_NETHIFS);
  priv = &g_enet[intf];

  /* NOTE:  ENET clocking was enabled and configured by board-specific logic
   * when other clocking was configured.
   */

  /* Configure all ENET/RMII pins */

  s32k1xx_pinconfig(PIN_RMII_MDC);
  s32k1xx_pinconfig(PIN_RMII_MDIO);
  s32k1xx_pinconfig(PIN_RMII_RX_DV);
  s32k1xx_pinconfig(PIN_RMII_RX_ER);
  s32k1xx_pinconfig(PIN_RMII_RXD0);
  s32k1xx_pinconfig(PIN_RMII_RXD1);
  s32k1xx_pinconfig(PIN_RMII_TX_CLK);
  s32k1xx_pinconfig(PIN_RMII_TX_EN);
  s32k1xx_pinconfig(PIN_RMII_TXD0);
  s32k1xx_pinconfig(PIN_RMII_TXD1);

#if 0
  /* Configure all ENET/MII pins */

  s32k1xx_pinconfig(PIN_MII_COL);
  s32k1xx_pinconfig(PIN_MII_CRS);
  s32k1xx_pinconfig(PIN_MII_MDC);
  s32k1xx_pinconfig(PIN_MII_MDIO);
  s32k1xx_pinconfig(PIN_MII_RX_DV);
  s32k1xx_pinconfig(PIN_MII_RX_ER);
  s32k1xx_pinconfig(PIN_MII_RX_CLK);
  s32k1xx_pinconfig(PIN_MII_RXD1);
  s32k1xx_pinconfig(PIN_MII_RXD2);
  s32k1xx_pinconfig(PIN_MII_RXD3);
  s32k1xx_pinconfig(PIN_MII_TX_CLK);
  s32k1xx_pinconfig(PIN_MII_TX_EN);
  s32k1xx_pinconfig(PIN_MII_TX_ER);
  s32k1xx_pinconfig(PIN_MII_TXD1);
  s32k1xx_pinconfig(PIN_MII_TXD2);
  s32k1xx_pinconfig(PIN_MII_TXD3);
#endif

  /* Attach the Ethernet MAC IEEE 1588 timer interrupt handler */

#if 0
  if (irq_attach(S32K1XX_IRQ_EMACTMR, s32k1xx_tmrinterrupt, NULL))
    {
      /* We could not attach the ISR to the interrupt */

      nerr("ERROR: Failed to attach EMACTMR IRQ\n");
      return -EAGAIN;
    }
#endif

  /* Attach the Ethernet interrupt handler */

  if (irq_attach(S32K1XX_IRQ_ENET_TXDONE, s32k1xx_enet_interrupt, NULL))
    {
      /* We could not attach the ISR to the interrupt */

      nerr("ERROR: Failed to attach EMACTX IRQ\n");
      return -EAGAIN;
    }

  if (irq_attach(S32K1XX_IRQ_ENET_RXDONE, s32k1xx_enet_interrupt, NULL))
    {
      /* We could not attach the ISR to the interrupt */

      nerr("ERROR: Failed to attach EMACRX IRQ\n");
      return -EAGAIN;
    }

  /* Initialize the driver structure */

  memset(priv, 0, sizeof(struct s32k1xx_driver_s));
  priv->dev.d_ifup    = s32k1xx_ifup;     /* I/F up (new IP address) callback */
  priv->dev.d_ifdown  = s32k1xx_ifdown;   /* I/F down callback */
  priv->dev.d_txavail = s32k1xx_txavail;  /* New TX data callback */
#ifdef CONFIG_NET_MCASTGROUP
  priv->dev.d_addmac  = s32k1xx_addmac;   /* Add multicast MAC address */
  priv->dev.d_rmmac   = s32k1xx_rmmac;    /* Remove multicast MAC address */
#endif
#ifdef CONFIG_NETDEV_IOCTL
  priv->dev.d_ioctl   = s32k1xx_ioctl;    /* Support PHY ioctl() calls */
#endif
  priv->dev.d_private = g_enet;           /* Used to recover private state from dev */

#ifdef CONFIG_NET_ETHERNET
  /* Determine a semi-unique MAC address from MCU UID
   * We use UID Low and Mid Low registers to get 64 bits, from which we keep
   * 48 bits.  We then force unicast and locally administered bits (b0 and
   * b1, 1st octet)
   */

  /* hardcoded offset: todo: need proper header file */

  uidl   = getreg32(S32K1XX_SIM_BASE + 0x60);
  uidml  = getreg32(S32K1XX_SIM_BASE + 0x5c);
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

#ifdef CONFIG_S32K1XX_ENET_PHYINIT
  /* Perform any necessary, one-time, board-specific PHY initialization */

  ret = s32k1xx_phy_boardinitialize(0);
  if (ret < 0)
    {
      nerr("ERROR: Failed to initialize the PHY: %d\n", ret);
      return ret;
    }
#endif

  /* Put the interface in the down state.  This usually amounts to resetting
   * the device and/or calling s32k1xx_ifdown().
   */

  s32k1xx_ifdown(&priv->dev);

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

#if CONFIG_S32K1XX_ENET_NETHIFS == 1 && !defined(CONFIG_NETDEV_LATEINIT)
void arm_netinitialize(void)
{
  s32k1xx_netinitialize(0);
}
#endif

#endif /* CONFIG_S32K1XX_ENET */
