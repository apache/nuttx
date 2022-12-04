/****************************************************************************
 * arch/arm/src/lpc54xx/lpc54_ethernet.c
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
 * 1. Timestamps not supported
 *
 * 2. Multi-queuing not fully; supported.  The second queue is intended to
 *    support QVLAN, AVB type packets which have an 18-byte IEEE 802.1q
 *    Ethernet header.  I propose handling this case with a new network
 *    interface qvlan_input().
 *
 * 3. Multicast address filtering.  Unlike other hardware, this Ethernet
 *    does not seem to support explicit Multicast address filtering as
 *    needed for ICMPv6 and for IGMP.  In these cases, I am simply accepting
 *    all multicast packets.  I am not sure if that is the right thing to
 *    do.
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
#include <nuttx/queue.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>
#include <nuttx/net/mii.h>
#include <nuttx/net/arp.h>
#include <nuttx/net/netdev.h>

#ifdef CONFIG_NET_PKT
#  include <nuttx/net/pkt.h>
#endif

#include "arm_internal.h"
#include "hardware/lpc54_syscon.h"
#include "hardware/lpc54_pinmux.h"
#include "hardware/lpc54_ethernet.h"
#include "lpc54_enableclk.h"
#include "lpc54_reset.h"
#include "lpc54_gpio.h"

#include <arch/board/board.h>

#ifdef CONFIG_LPC54_ETHERNET

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Work queue support is required. */

#if !defined(CONFIG_SCHED_WORKQUEUE)
#  error Work queue support is required in this configuration (CONFIG_SCHED_WORKQUEUE)
#endif

/* Multicast address filtering.  Unlike other hardware, this Ethernet does
 * not seem to support explicit Multicast address filtering as needed for
 * ICMPv6 and for IGMP.  In these cases, I am simply accepting all multicast
 * packets.
 */

#undef LPC54_ACCEPT_ALLMULTICAST
#if  defined(CONFIG_LPC54_ETH_RX_ALLMULTICAST) || \
     defined(CONFIG_NET_MCASTGROUP) || defined(CONFIG_NET_ICMPv6)
#  define LPC54_ACCEPT_ALLMULTICAST 1
#endif

/* The low priority work queue is preferred.  If it is not enabled, LPWORK
 * will be the same as HPWORK.
 *
 * NOTE:  However, the network should NEVER run on the high priority work
 * queue!  That queue is intended only to service short back end interrupt
 * processing that never suspends.  Suspending the high priority work queue
 * may bring the system to its knees!
 */

#define ETHWORK LPWORK

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

#define LPC54_MTL_QUEUE_UNIT      256
#define LPC54_MTL_RXQUEUE_UNITS   8     /* Rx queue size = 2048 bytes */
#define LPC54_MTL_TXQUEUE_UNITS   8     /* Tx queue size = 2048 bytes */

#ifdef CONFIG_LPC54_ETH_TXRR
#  define LPC54_MTL_OPMODE_SCHALG ETH_MTL_OP_MODE_SHALG_WSP
#else
#  define LPC54_MTL_OPMODE_SCHALG ETH_MTL_OP_MODE_SHALG_SP
#endif

#ifdef CONFIG_LPC54_ETH_RXRR
#  define LPC54_MTL_OPMODE_RAA    ETH_MTL_OP_MODE_RAA_WSP
#else
#  define LPC54_MTL_OPMODE_RAA    ETH_MTL_OP_MODE_RAA_SP
#endif

/* MAC-related definitions */

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

#define LPC54_BUFFER_SIZE       MAX_NETDEV_PKTSIZE
#define LPC54_BUFFER_ALLOC      ((MAX_NETDEV_PKTSIZE + CONFIG_NET_GUARDSIZE + 3) & ~3)
#define LPC54_BUFFER_WORDS      ((MAX_NETDEV_PKTSIZE + CONFIG_NET_GUARDSIZE + 3) >> 2)
#define LPC54_BUFFER_MAX        16384

/* DMA and DMA descriptor definitions */

#define LPC54_MIN_RINGLEN       4     /* Min length of a ring */
#define LPC54_MAX_RINGLEN       1023  /* Max length of a ring */
#define LPC54_MAX_RINGS         2     /* Max number of tx/rx descriptor rings */
#ifdef CONFIG_LPC54_ETH_MULTIQUEUE
#  define LPC54_NRINGS          2     /* Use 2 Rx and Tx rings */
#else
#  define LPC54_NRINGS          1     /* Use 1 Rx and 1 Tx ring */
#endif

#ifndef CONFIG_LPC54_ETH_BURSTLEN
#  define CONFIG_LPC54_ETH_BURSTLEN 1
#endif

#if CONFIG_LPC54_ETH_BURSTLEN < 2
#  define LPC54_BURSTLEN        1
#  define LPC54_PBLX8           0
#elif CONFIG_LPC54_ETH_BURSTLEN < 4
#  define LPC54_BURSTLEN        2
#  define LPC54_PBLX8           0
#elif CONFIG_LPC54_ETH_BURSTLEN < 8
#  define LPC54_BURSTLEN        4
#  define LPC54_PBLX8           0
#elif CONFIG_LPC54_ETH_BURSTLEN < 16
#  define LPC54_BURSTLEN        8
#  define LPC54_PBLX8           0
#elif CONFIG_LPC54_ETH_BURSTLEN < 32
#  define LPC54_BURSTLEN        16
#  define LPC54_PBLX8           0
#elif CONFIG_LPC54_ETH_BURSTLEN < 64
#  define LPC54_BURSTLEN        32
#  define LPC54_PBLX8           0
#elif CONFIG_LPC54_ETH_BURSTLEN < 128
#  define LPC54_BURSTLEN        8
#  define LPC54_PBLX8           ETH_DMACH_CTRL_PBLx8
#elif CONFIG_LPC54_ETH_BURSTLEN < 256
#  define LPC54_BURSTLEN        16
#  define LPC54_PBLX8           ETH_DMACH_CTRL_PBLx8
#else
#  define LPC54_BURSTLEN        32
#  define LPC54_PBLX8           ETH_DMACH_CTRL_PBLx8
#endif

#ifdef CONFIG_LPC54_ETH_DYNAMICMAP
#  define LPC54_QUEUEMAP        (ETH_MTL_RXQ_DMA_MAP_Q0DDMACH | \
                                 ETH_MTL_RXQ_DMA_MAP_Q1DDMACH)
#else
#  define LPC54_QUEUEMAP        ETH_MTL_RXQ_DMA_MAP_Q1MDMACH
#endif

/* Interrupt masks */

#define LPC54_ABNORM_INTMASK    (ETH_DMACH_INT_TS  | ETH_DMACH_INT_RBU | \
                                 ETH_DMACH_INT_RS  | ETH_DMACH_INT_RWT | \
                                 ETH_DMACH_INT_FBE | ETH_DMACH_INT_AI)
#define LPC54_TXERR_INTMASK     (ETH_DMACH_INT_TS)
#define LPC54_RXERR_INTMASK     (ETH_DMACH_INT_RBU | ETH_DMACH_INT_RS  | \
                                 ETH_DMACH_INT_RWT)
#define LPC54_NORM_INTMASK      (ETH_DMACH_INT_TI  | ETH_DMACH_INT_RI  | \
                                 ETH_DMACH_INT_NI)

/* This is a helper pointer for accessing the contents of the Ethernet
 * header.
 */

#define ETH8021QWBUF ((struct eth_8021qhdr_s *)priv->eth_dev.d_buf)

/* This is a helper pointer for accessing the contents of the Ethernet
 * header
 */

#define BUF ((FAR struct eth_hdr_s *)&dev->d_buf[0])

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Describes the state of one Tx descriptor ring */

struct lpc54_txring_s
{
  struct enet_txdesc_s *tr_desc; /* Tx descriptor base address */
  uint16_t tr_supply;            /* Tx supplier ring index */
  uint16_t tr_consume;           /* Tx consumer ring index */
  uint16_t tr_ndesc;             /* Number or descriptors in the Tx ring */
  uint16_t tr_inuse;             /* Number of Tx descriptors in-use */
  uint32_t **tr_buffers;         /* Packet buffers assigned to the Rx ring */
};

/* Describes the state of one Rx descriptor ring */

struct lpc54_rxring_s
{
  struct enet_rxdesc_s *rr_desc; /* Rx descriptor base address */
  uint16_t rr_supply;            /* Available Rx descriptor ring index */
  uint16_t rr_ndesc;             /* Number or descriptors in the Rx ring */
  uint32_t **rr_buffers;         /* Packet buffers assigned to the Rx ring */
};

/* The lpc54_ethdriver_s encapsulates all state information for a single
 * Ethernet interface
 */

struct lpc54_ethdriver_s
{
  uint8_t eth_bifup : 1;         /* 1:ifup 0:ifdown */
  uint8_t eth_fullduplex : 1;    /* 1:Full duplex 0:Half duplex mode */
  uint8_t eth_100mbps : 1;       /* 1:100mbps 0:10mbps */
  uint8_t eth_rxdiscard : 1;     /* 1:Discarding Rx data */
  struct wdog_s eth_txtimeout;   /* TX timeout timer */
  struct work_s eth_irqwork;     /* For deferring interrupt work to the work queue */
  struct work_s eth_pollwork;    /* For deferring poll work to the work queue */
  struct work_s eth_timeoutwork; /* For deferring timeout work to the work queue */
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
 * addresses are lost in the DMA due to write-back from the DMA hardware.
 * So we have to remember the buffer assignments explicitly.
 *
 * REVISIT:  According to the User manual, buffer1 and buffer2 addresses
 * will be overwritten by DMA write-back data.  However, I see that the
 * Rx buffer1 and buffer2 addresses are, indeed, used by the NXP example
 * code after completion of DMA so the user manual must be wrong.  We
 * could eliminate this extra array of saved allocation addresses.
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

/* Register level debug hooks */

#ifdef CONFIG_LPC54_ETH_REGDEBUG
static uint32_t lpc54_getreg(uintptr_t addr);
static void lpc54_putreg(uint32_t val, uintptr_t addr);
#else
# define lpc54_getreg(addr)      getreg32(addr)
# define lpc54_putreg(val,addr)  putreg32(val,addr)
#endif

/* Common TX logic */

static int  lpc54_eth_transmit(struct lpc54_ethdriver_s *priv,
              unsigned int chan);
static unsigned int lpc54_eth_getring(struct lpc54_ethdriver_s *priv);
static int  lpc54_eth_txpoll(struct net_driver_s *dev);

/* Interrupt handling */

static void lpc54_eth_reply(struct lpc54_ethdriver_s *priv);
static void lpc54_eth_rxdispatch(struct lpc54_ethdriver_s *priv);
static int  lpc54_eth_receive(struct lpc54_ethdriver_s *priv,
              unsigned int chan);
static void lpc54_eth_txdone(struct lpc54_ethdriver_s *priv,
              unsigned int chan);

static void lpc54_eth_channel_work(struct lpc54_ethdriver_s *priv,
              unsigned int chan);
static void lpc54_eth_interrupt_work(void *arg);
static int  lpc54_eth_interrupt(int irq, void *context, void *arg);
#if 0 /* Not used */
static int  lpc54_pmt_interrupt(int irq, void *context, void *arg);
static int  lpc54_mac_interrupt(int irq, void *context, void *arg);
#endif

/* Watchdog timer expirations */

static void lpc54_eth_dopoll(struct lpc54_ethdriver_s *priv);

static void lpc54_eth_txtimeout_work(void *arg);
static void lpc54_eth_txtimeout_expiry(wdparm_t arg);

/* NuttX callback functions */

static int  lpc54_eth_ifup(struct net_driver_s *dev);
static int  lpc54_eth_ifdown(struct net_driver_s *dev);

static void lpc54_eth_txavail_work(void *arg);
static int  lpc54_eth_txavail(struct net_driver_s *dev);

#ifdef CONFIG_NET_MCASTGROUP
static int  lpc54_eth_addmac(struct net_driver_s *dev,
              const uint8_t *mac);
static int  lpc54_eth_rmmac(struct net_driver_s *dev,
              const uint8_t *mac);
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
static inline bool lpc54_phy_linkstatus(struct lpc54_ethdriver_s *priv);
static int  lpc54_phy_autonegotiate(struct lpc54_ethdriver_s *priv);
static int  lpc54_phy_reset(struct lpc54_ethdriver_s *priv);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc54_getreg
 *
 * Description:
 *   This function may to used to intercept an monitor all register accesses.
 *   Clearly this is nothing you would want to do unless you are debugging
 *   this driver.
 *
 * Input Parameters:
 *   addr - The register address to read
 *
 * Returned Value:
 *   The value read from the register
 *
 ****************************************************************************/

#ifdef CONFIG_LPC54_ETH_REGDEBUG
static uint32_t lpc54_getreg(uintptr_t addr)
{
  static uintptr_t prevaddr = 0;
  static uint32_t preval    = 0;
  static uint32_t count     = 0;

  /* Read the value from the register */

  uint32_t val = getreg32(addr);

  /* Is this the same value that we read from the same register last time?
   * Are we polling the register?  If so, suppress some of the output.
   */

  if (addr == prevaddr && val == preval)
    {
      if (count == 0xffffffff || ++count > 3)
        {
          if (count == 4)
            {
              ninfo("...\n");
            }

          return val;
        }
    }

  /* No this is a new address or value */

  else
    {
      /* Did we print "..." for the previous value? */

      if (count > 3)
        {
          /* Yes.. then show how many times the value repeated */

          ninfo("[repeats %d more times]\n", count - 3);
        }

      /* Save the new address, value, and count */

      prevaddr = addr;
      preval   = val;
      count    = 1;
    }

  /* Show the register value read */

  ninfo("%08x->%08x\n", addr, val);
  return val;
}
#endif

/****************************************************************************
 * Name: lpc54_putreg
 *
 * Description:
 *   This function may to used to intercept an monitor all register accesses.
 *   Clearly this is nothing you would want to do unless you are debugging
 *   this driver.
 *
 * Input Parameters:
 *   val - The value to write to the register
 *   addr - The register address to read
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_LPC54_ETH_REGDEBUG
static void lpc54_putreg(uint32_t val, uintptr_t addr)
{
  /* Show the register value being written */

  ninfo("%08x<-%08x\n", addr, val);

  /* Write the value */

  putreg32(val, addr);
}
#endif

/****************************************************************************
 * Name: lpc54_eth_transmit
 *
 * Description:
 *   Start hardware transmission.  Called either from the txdone interrupt
 *   handling or from watchdog based polling.
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *   chan - The channel to send the packet on
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static int lpc54_eth_transmit(struct lpc54_ethdriver_s *priv,
                              unsigned int chan)
{
  struct lpc54_txring_s *txring;
  struct enet_txdesc_s *txdesc;
  uint8_t *buffer;
  uint32_t regval;
  unsigned int buflen;

  /* Verify that the hardware is ready to send another packet.  If we get
   * here, then we are committed to sending a packet; Higher level logic
   * must have assured that we have the resources available to perform the
   * send.
   */

  txring = &priv->eth_txring[chan];

  DEBUGASSERT(priv->eth_dev.d_buf != 0 && priv->eth_dev.d_len > 0 &&
              priv->eth_dev.d_len <= LPC54_BUFFER_SIZE &&
              txring->tr_inuse < txring->tr_ndesc);

  /* Fill the descriptor. */

  txdesc = txring->tr_desc + txring->tr_supply;
  buffer = priv->eth_dev.d_buf;
  buflen = priv->eth_dev.d_len;

  priv->eth_dev.d_buf = NULL;
  priv->eth_dev.d_len = 0;

  if (buflen <= LPC54_BUFFER_MAX)
    {
      /* Prepare the Tx descriptor for transmission */

      txdesc->buffer1 = (uint32_t)buffer;
      txdesc->buffer2 = 0;

      /* One buffer, no timestamp, interrupt on completion */

      regval = ETH_TXDES2_B1L(buflen) | ETH_TXDES2_B2L(0) | ETH_TXDES2_IOC;
      txdesc->buflen = regval;

      /* Full packet length, last descriptor, first descriptor, owned by
       * DMA.
       */

      regval = ETH_TXDES3_FL(buflen) | ETH_TXDES3_LD | ETH_TXDES3_FD |
               ETH_TXDES3_OWN;
      txdesc->ctrlstat = regval;
    }
#if LPC54_BUFFER_SIZE > LPC54_BUFFER_MAX
  else
    {
      unsigned int buf2len = buflen - LPC54_BUFFER_MAX;
      uint8_t *buffer2     = buffer + LPC54_BUFFER_MAX;

      DEBUGASSERT(buf2len <= LPC54_BUFFER_MAX);

      /* Prepare the Tx descriptor for transmission */

      txdesc->buffer1 = (uint32_t)buffer;
      txdesc->buffer2 = (uint32_t)buffer2;

      /* Two buffers, no timestamp, interrupt on completion */

      regval = ETH_TXDES2_B1L(LPC54_BUFFER_MAX) |
               ETH_TXDES2_B2L(buf2len) | ETH_TXDES2_IOC;
      txdesc->buflen = regval;

      /* Full packet length, last descriptor, first descriptor, owned by
       * DMA.
       */

      regval = ETH_TXDES3_FL(buflen) | ETH_TXDES3_LD | ETH_TXDES3_FD |
               ETH_TXDES3_OWN;
      txdesc->ctrlstat = regval;
    }
#endif

  /* Increase the index */

  if (++(txring->tr_supply) >= txring->tr_ndesc)
    {
      txring->tr_supply = 0;
    }

  /* Increment the number of descriptors in-use. */

  txring->tr_inuse++;

  /* Update the transmit tail address. */

  txdesc = txring->tr_desc + txring->tr_supply;
  if (txring->tr_supply == 0)
    {
      txdesc = txring->tr_desc + txring->tr_ndesc;
    }

  /* Update the DMA tail pointer */

  lpc54_putreg((uint32_t)txdesc, LPC54_ETH_DMACH_TXDESC_TAIL_PTR(chan));

  /* Setup the TX timeout watchdog (perhaps restarting the timer) */

  wd_start(&priv->eth_txtimeout, LPC54_TXTIMEOUT,
           lpc54_eth_txtimeout_expiry, (wdparm_t)priv);
  return OK;
}

/****************************************************************************
 * Name: lpc54_eth_getring
 *
 * Description:
 *   An output message is ready to send, but which queue should we send it
 *   on?  The rule is this:
 *
 *   "Normal" packets (or CONFIG_LPC54_ETH_MULTIQUEUE not selected):
 *     Always send on ring 0
 *   8021QVLAN AVB packets (and CONFIG_LPC54_ETH_MULTIQUEUE not selected):
 *     Always send on ring 1
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   The ring to use when sending the packet.
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static unsigned int lpc54_eth_getring(struct lpc54_ethdriver_s *priv)
{
  unsigned int ring = 0;

#ifdef CONFIG_LPC54_ETH_MULTIQUEUE
  /* Choose the ring ID for different types of frames.  For 802.1q VLAN AVB
   * frames, uses ring 1.  Everything else goes on ring 0.
   */

  if (ETH8021QWBUF->tpid == HTONS(TPID_8021QVLAN) &&
      ETH8021QWBUF->type == HTONS(ETHTYPE_AVBTP))
    {
      ring = 1;
    }
#endif

  return ring;
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

static int lpc54_eth_txpoll(struct net_driver_s *dev)
{
  struct lpc54_ethdriver_s *priv;
  struct lpc54_txring_s *txring;
  struct lpc54_txring_s *txring0;
#ifdef CONFIG_LPC54_ETH_MULTIQUEUE
  struct lpc54_txring_s *txring1;
#endif
  unsigned int chan;

  DEBUGASSERT(dev->d_private != NULL && dev->d_buf != NULL);
  priv = (struct lpc54_ethdriver_s *)dev->d_private;

  /* Send the packet */

  chan   = lpc54_eth_getring(priv);
  txring = &priv->eth_txring[chan];

  (txring->tr_buffers)[txring->tr_supply] =
    (uint32_t *)priv->eth_dev.d_buf;

  lpc54_eth_transmit(priv, chan);

  txring0 = &priv->eth_txring[0];
#ifdef CONFIG_LPC54_ETH_MULTIQUEUE
  txring1 = &priv->eth_txring[1];

  /* We cannot perform the Tx poll now if all of the Tx descriptors
   * for both channels are in-use.
   */

  if (txring0->tr_inuse >= txring0->tr_ndesc ||
      txring1->tr_inuse >= txring1->tr_ndesc)
#else
  /* We cannot continue the Tx poll now if all of the Tx descriptors
   * for this channel 0 are in-use.
   */

  if (txring0->tr_inuse >= txring0->tr_ndesc)
#endif
    {
      /* Stop the poll.. no more Tx descriptors */

      return 1;
    }

  /* There is a free descriptor in the ring, allocate a new Tx buffer
   * to perform the poll.
   */

  priv->eth_dev.d_buf = (uint8_t *)lpc54_pktbuf_alloc(priv);
  if (priv->eth_dev.d_buf == NULL)
    {
      /* Stop the poll.. no more packet buffers */

      return 1;
    }

  /* If zero is returned, the polling will continue until all connections
   * have been examined.  If there is nothing to be sent, we will return to
   * the caller of devif_poll() with an allocated packet buffer.
   */

  return 0;
}

/****************************************************************************
 * Name: lpc54_eth_reply
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

static void lpc54_eth_reply(struct lpc54_ethdriver_s *priv)
{
  struct lpc54_txring_s *txring;
  unsigned int chan;

  /* If the packet dispatch resulted in data that should be sent out on the
   * network, the field d_len will set to a value > 0.
   */

  if (priv->eth_dev.d_len > 0)
    {
      /* Update the Ethernet header with the correct MAC address */

#ifdef CONFIG_LPC54_ETH_MULTIQUEUE
      /* Check for an outgoing 802.1q VLAN packet */
#warning Missing Logic
#endif

      /* And send the packet */

      chan   = lpc54_eth_getring(priv);
      txring = &priv->eth_txring[chan];

      (txring->tr_buffers)[txring->tr_supply] =
        (uint32_t *)priv->eth_dev.d_buf;

      lpc54_eth_transmit(priv, chan);
    }
}

/****************************************************************************
 * Name: lpc54_eth_rxdispatch
 *
 * Description:
 *   A new packet has been received and will be forwarded to the network
 *   stack.
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

static void lpc54_eth_rxdispatch(struct lpc54_ethdriver_s *priv)
{
  struct net_driver_s *dev = &priv->eth_dev;

#ifdef CONFIG_NET_PKT
  /* When packet sockets are enabled, feed the frame into the tap */

  pkt_input(dev);
#endif

  /* We only accept IP packets of the configured type and ARP packets */

#ifdef CONFIG_NET_IPv4
  if (BUF->type == HTONS(ETHTYPE_IP))
    {
      ninfo("IPv4 packet\n");
      NETDEV_RXIPV4(dev);

      /* Receive an IPv4 packet from the network device */

      ipv4_input(dev);

      /* Check for a reply to the IPv4 packet */

      lpc54_eth_reply(priv);
    }
  else
#endif
#ifdef CONFIG_NET_IPv6
  if (BUF->type == HTONS(ETHTYPE_IP6))
    {
      ninfo("IPv6 packet\n");
      NETDEV_RXIPV6(dev);

      /* Dispatch IPv6 packet to the network layer */

      ipv6_input(dev);

      /* Check for a reply to the IPv6 packet */

      lpc54_eth_reply(priv);
    }
  else
#endif
#ifdef CONFIG_LPC54_ETH_MULTIQUEUE
  if (ETH8021QWBUF->tpid == HTONS(TPID_8021QVLAN))
    {
      ninfo("IEEE 802.1q packet\n");
      NETDEV_RXQVLAN(dev);

      /* Dispatch the 802.1q VLAN packet to the network layer */

      qvlan_input(dev);

      /* Check for a reply to the 802.1q VLAN packet */

      lpc54_eth_reply(priv);
    }
  else
#endif
#ifdef CONFIG_NET_ARP
  if (BUF->type == HTONS(ETHTYPE_ARP))
    {
      struct lpc54_txring_s *txring;
      unsigned int chan;

      /* Dispatch the ARP packet to the network layer */

      arp_input(dev);
      NETDEV_RXARP(dev);

      /* If the above function invocation resulted in data that should be
       * sent out on the network, d_len field will set to a value > 0.
       */

      if (dev->d_len > 0)
        {
          chan   = lpc54_eth_getring(priv);
          txring = &priv->eth_txring[chan];

          (txring->tr_buffers)[txring->tr_supply] =
            (uint32_t *)dev->d_buf;

          lpc54_eth_transmit(priv, chan);
        }
    }
  else
#endif
    {
      NETDEV_RXDROPPED(dev);
    }

  /* On entry, d_buf refers to the receive buffer as set by logic in
   * lpc54_eth_receive().  If lpc54_eth_transmit() was called to reply
   * with an outgoing packet, then that packet was removed for transmission
   * and d_buf was nullified.  Otherwise, d_buf still holds the stale
   * receive buffer and we will need to dispose of it here.
   */

  if (dev->d_buf != NULL)
    {
      lpc54_pktbuf_free(priv, (uint32_t *)dev->d_buf);
    }

  dev->d_buf = NULL;
  dev->d_len = 0;
}

/****************************************************************************
 * Name: lpc54_eth_receive
 *
 * Description:
 *   An interrupt was received indicating the availability of a new RX packet
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *   chan - The channel with the completed Rx transfer
 *
 * Returned Value:
 *   The number of Rx descriptors processed
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static int lpc54_eth_receive(struct lpc54_ethdriver_s *priv,
                              unsigned int chan)
{
  struct lpc54_rxring_s *rxring;
  struct enet_rxdesc_s *rxdesc;
  unsigned int framelen;
  unsigned int pktlen;
  unsigned int supply;
  uint32_t regval;
  bool suspend;
  int ndesc;

  /* Get the Rx ring associated with this channel */

  rxring  = &priv->eth_rxring[chan];

  /* If no Rx descriptor is available, then suspend for now */

  regval  = lpc54_getreg(LPC54_ETH_DMACH_STAT(chan));
  suspend = ((regval & ETH_DMACH_INT_RBU) != 0);

  /* Loop until the next full frame is encountered or until we encounter a
   * descriptor still owned by the DMA.
   */

  pktlen = 0;
  ndesc  = 0;

  for (; ; )
    {
      /* Get the last Rx descriptor in the ring */

      supply = rxring->rr_supply;
      rxdesc = rxring->rr_desc + supply;

      /* Is this frame still owned by the DMA? */

      if ((rxdesc->ctrl & ETH_RXDES3_OWN) != 0)
        {
          /* Yes.. then bail */

          return ndesc;
        }

      ndesc++;

      /* Set the supplier index to the next descriptor */

      if (++(rxring->rr_supply) > rxring->rr_ndesc)
        {
          rxring->rr_supply = 0;
        }

      /* Is this the last descriptor of the frame? */

      if ((rxdesc->ctrl & ETH_RXDES3_LD) != 0)
        {
          /* Have we been discarding Rx data?  If so, that was the last
           * packet to be discarded.
           */

          if (priv->eth_rxdiscard)
            {
              priv->eth_rxdiscard = 0;
            }
          else
            {
              /* Last frame encountered.  This is a valid packet */

              framelen  = (rxdesc->ctrl & ETH_RXDES3_PL_MASK);
              pktlen   += framelen;

              if (pktlen > 0)
                {
                  /* Recover the buffer.
                   *
                   * REVISIT:  According to the User manual, buffer1 and
                   * buffer2 addresses were overwritten by the write-back
                   * data.  However, I see that the buffer1 and buffer2
                   * addresses are, indeed, used by the NXP example code
                   * so the user manual must be wrong.  We could eliminate
                   * this extra array of saved allocation addresses.
                   */

                  priv->eth_dev.d_buf = (uint8_t *)
                    (rxring->rr_buffers)[supply];
                  (rxring->rr_buffers)[supply] = NULL;
                  DEBUGASSERT(priv->eth_dev.d_buf != NULL);

                  priv->eth_dev.d_len = pktlen;

                  /* REVISIT: What should we do if there is no Tx buffer
                   * available.  In receiving the packet, we could also
                   * generate a new outgoing packet that could only be
                   * handled if there is an available Tx descriptor.
                   */

                  lpc54_eth_rxdispatch(priv);

                  /* Allocate a new Rx buffer and update the Rx buffer
                   * descriptor.
                   */

                  rxdesc->buffer1 = (uint32_t)lpc54_pktbuf_alloc(priv);
                  DEBUGASSERT(rxdesc->buffer1 != 0);
                  (rxring->rr_buffers)[supply] = (uint32_t *)rxdesc->buffer1;

#if LPC54_BUFFER_SIZE > LPC54_BUFFER_MAX
                  rxdesc->buffer2 = rxdesc->buffer1 + LPC54_BUFFER_MAX;
#else
                  /* The second buffer is not used */

                  rxdesc->buffer2  = 0;
#endif
                  rxdesc->reserved = 0;

                  /* Buffer1 (and maybe 2) valid, interrupt on completion,
                   * owned by DMA.
                   */

                  regval  = ETH_RXDES3_BUF1V | ETH_RXDES3_IOC |
                            ETH_RXDES3_OWN;
#if LPC54_BUFFER_SIZE > LPC54_BUFFER_MAX
                  regval |= ETH_RXDES3_BUF2V;
#endif
                  rxdesc->ctrl = regval;
                }

              return ndesc;
            }
        }
      else if (!priv->eth_rxdiscard)
        {
          /* Not the last Rx descriptor of the packet.
           *
           * We are attempting to receive a large packet spanning multiple
           * Rx descriptors.  We cannot support that in this design.  We
           * would like to:
           *
           *   1. Accumulate the data in yet another Rx buffer,
           *   2. Accumulate the size in the 'pktlen' local variable, then
           *   3. Dispatch that extra Rx buffer when the last frame is
           *      encountered.
           *
           * The assumption here is that this will never happen if our MTU
           * properly advertised.
           */

          NETDEV_RXDROPPED(&priv->eth_dev);
          priv->eth_rxdiscard = 1;
        }
    }

  /* Restart the receiver and clear the RBU status if it was suspended. */

  if (suspend)
    {
      uintptr_t regaddr = LPC54_ETH_DMACH_RXDESC_TAIL_PTR(chan);

      /* Clear the RBU status */

      lpc54_putreg(ETH_DMACH_INT_RBU, LPC54_ETH_DMACH_STAT(chan));

      /* Writing to the tail pointer register
       * will restart the Rx processing
       */

      regval = lpc54_getreg(regaddr);
      lpc54_putreg(regval, regaddr);
    }

  return ndesc;
}

/****************************************************************************
 * Name: lpc54_eth_txdone
 *
 * Description:
 *   An interrupt was received indicating that the last TX packet(s) is done
 *
 * Input Parameters:
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
#ifdef CONFIG_LPC54_ETH_MULTIQUEUE
  struct lpc54_txring_s *txring0;
  struct lpc54_txring_s *txring1;
#endif
  struct enet_txdesc_s *txdesc;
  uint32_t *pktbuf;

  /* Reclaim the compled Tx descriptor */

  txring = &priv->eth_txring[chan];
  txdesc = txring->tr_desc + txring->tr_consume;

  /* Update the first index for transmit buffer free. */

  while (txring->tr_inuse > 0 && (txdesc->ctrlstat & ETH_TXDES3_OWN) == 0)
    {
      /* Update statistics */

      NETDEV_TXDONE(priv->eth_dev);

      /* Free the Tx buffer assigned to the descriptor */

      pktbuf = txring->tr_buffers[txring->tr_consume];
      DEBUGASSERT(pktbuf != NULL);
      if (pktbuf != NULL)
        {
          lpc54_pktbuf_free(priv, pktbuf);
          txring->tr_buffers[txring->tr_consume] = NULL;
        }

      /* One less Tx descriptor in use */

      txring->tr_inuse--;

      /* Update the consume index and the descriptor pointer. */

      if (++(txring->tr_consume) >= txring->tr_ndesc)
        {
          txring->tr_consume = 0;
        }

      txdesc = txring->tr_desc + txring->tr_consume;
    }

  /* If no further transmissions are pending, then cancel the TX timeout. */

#ifdef CONFIG_LPC54_ETH_MULTIQUEUE
  txring0 = &priv->eth_txring[0];
  txring1 = &priv->eth_txring[1];

  if (txring0->tr_inuse == 0 && txring1->tr_inuse == 0)

#else
  if (txring->tr_inuse == 0)
#endif
    {
      wd_cancel(&priv->eth_txtimeout);
      work_cancel(ETHWORK, &priv->eth_timeoutwork);
    }

  /* Poll the network for new TX data. */

  lpc54_eth_dopoll(priv);
}

/****************************************************************************
 * Name: lpc54_eth_channel_work
 *
 * Description:
 *   Perform interrupt related work for a channel DMA interrupt
 *
 * Input Parameters:
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

static void lpc54_eth_channel_work(struct lpc54_ethdriver_s *priv,
                                   unsigned int chan)
{
  uintptr_t regaddr;
  uint32_t status;
  uint32_t pending;

  /* Read the DMA status register for this channel */

  regaddr = LPC54_ETH_DMACH_STAT(chan);
  status  = lpc54_getreg(regaddr);
  pending = status & lpc54_getreg(LPC54_ETH_DMACH_INT_EN(chan));

  /* Check for abnormal interrupts */

  if ((pending & LPC54_ABNORM_INTMASK) != 0)
    {
      /* Acknowledge the abnormal interrupt interrupts except for RBU...
       * that is a special case where the status will be cleared in
       * lpc54_eth_receive().  See comments below.
       */

      lpc54_putreg((LPC54_ABNORM_INTMASK & ~ETH_DMACH_INT_RBU), regaddr);

      /* Handle the incoming packet */

      nerr("ERROR: Abnormal interrupt received: %08lx (%08lx)\n",
           (unsigned long)pending, (unsigned long)status);

      /* Check for Tx/Rx related errors and update statistics */

      if ((pending & LPC54_RXERR_INTMASK) != 0)
        {
          NETDEV_RXERRORS(priv->eth_dev);
        }

      if ((pending & LPC54_TXERR_INTMASK) != 0)
        {
          NETDEV_TXERRORS(priv->eth_dev);
        }

      /* The Receive Buffer Unavailable (RBU) error is a special case.  It
       * means that we have an Rx overrun condition:  All of the Rx buffers
       * have been filled with packet data and there are no Rx descriptors
       * available to receive the next packet.
       *
       * Often RBU is accompanied by RI but we need to force that condition
       * in all cases.  In the case of RBU, we need to perform receive
       * processing in order to recover from the situation and to resume.
       *
       * This is really a configuration problem:  It really means that we
       * have not assigned enough Rx buffers for the environment and
       * addressing filtering options that we have selected.
       */

      if ((pending & ETH_DMACH_INT_RBU) != 0)
        {
          pending |= ETH_DMACH_INT_RI;
        }

      pending &= ~LPC54_ABNORM_INTMASK;
    }

  /* Check for a receive interrupt */

  if ((pending & ETH_DMACH_INT_RI) != 0)
    {
      int ndesc;

      /* Acknowledge the normal receive interrupt */

      lpc54_putreg(ETH_DMACH_INT_RI | ETH_DMACH_INT_NI, regaddr);
      pending &= ~(ETH_DMACH_INT_RI | ETH_DMACH_INT_NI);

      /* Loop until all available Rx packets
       * in the ring have been processed
       */

      for (; ; )
        {
          /* Dispatch the next packet from the Rx ring */

          ndesc = lpc54_eth_receive(priv, chan);
          if (ndesc > 0)
            {
              /* Update statistics if a packet was dispatched */

              NETDEV_RXPACKETS(priv->eth_dev);
            }
          else
            {
              break;
            }
        }
    }

  /* Check for a transmit interrupt */

  if ((pending & ETH_DMACH_INT_TI) != 0)
    {
      /* Acknowledge the normal receive interrupt */

      lpc54_putreg(ETH_DMACH_INT_TI | ETH_DMACH_INT_NI, regaddr);
      pending &= ~(ETH_DMACH_INT_TI | ETH_DMACH_INT_NI);

      /* Handle the Tx completion event.  Reclaim the completed Tx
       * descriptors, free packet buffers, and check if we can start a new
       * transmission.
       */

      lpc54_eth_txdone(priv, chan);
    }

  /* Check for unhandled interrupts (shouldn't be any) */

  if (pending != 0)
    {
      nwarn("WARNING: Unhandled interrupts: %08lx (%08lx)\n",
           (unsigned long)pending, (unsigned long)status);
      lpc54_putreg(pending, regaddr);
    }
}

/****************************************************************************
 * Name: lpc54_eth_interrupt_work
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

static void lpc54_eth_interrupt_work(void *arg)
{
  struct lpc54_ethdriver_s *priv = (struct lpc54_ethdriver_s *)arg;
  uint32_t intrstat;

  /* Lock the network to serialize driver operations. */

  net_lock();

  /* Check if interrupt is from DMA channel 0. */

  intrstat = lpc54_getreg(LPC54_ETH_DMA_INTR_STAT);
  if ((intrstat & ETH_DMA_INTR_STAT_DC0IS) != 0)
    {
      lpc54_eth_channel_work(priv, 0);
    }

  /* Check if interrupt is from DMA channel 1. */

  intrstat = lpc54_getreg(LPC54_ETH_DMA_INTR_STAT);
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
 * Input Parameters:
 *   irq     - Number of the IRQ that generated the interrupt
 *   context - Interrupt register state save info (architecture-specific)
 *
 * Returned Value:
 *   Runs in the context of a the Ethernet interrupt handler.  Local
 *   interrupts are disabled by the interrupt logic.
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

  /* Note:  We have a race condition which, I believe is handled OK.  If
   * there is a Tx timeout in place, then that timeout could expire
   * anytime and queue additional work to handle the timeout.
   */

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
 * Input Parameters:
 *   irq     - Number of the IRQ that generated the interrupt
 *   context - Interrupt register state save info (architecture-specific)
 *
 * Returned Value:
 *   Runs in the context of a the Ethernet PMT interrupt handler.  Local
 *   interrupts are disabled by the interrupt logic.
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
 * Input Parameters:
 *   irq     - Number of the IRQ that generated the interrupt
 *   context - Interrupt register state save info (architecture-specific)
 *
 * Returned Value:
 *   Runs in the context of a the Ethernet MAC handler.  Local
 *   interrupts are disabled by the interrupt logic.
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
 * Input Parameters:
 *   arg - The argument passed when work_queue() as called.
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *   Runs on a worker thread.
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

  /* Then reset the hardware by bringing it down and taking it back up
   * again.
   */

  lpc54_eth_ifdown(&priv->eth_dev);
  lpc54_eth_ifup(&priv->eth_dev);

  /* Then poll the network for new XMIT data */

  lpc54_eth_dopoll(priv);
  net_unlock();
}

/****************************************************************************
 * Name: lpc54_eth_txtimeout_expiry
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

static void lpc54_eth_txtimeout_expiry(wdparm_t arg)
{
  struct lpc54_ethdriver_s *priv = (struct lpc54_ethdriver_s *)arg;

  /* Disable further Ethernet interrupts.  This will prevent some race
   * conditions with interrupt work.  There is still a potential race
   * condition with interrupt work that is already queued and in progress.
   */

  up_disable_irq(LPC54_IRQ_ETHERNET);

  /* Schedule to perform the TX timeout processing on the worker thread. */

  work_queue(ETHWORK, &priv->eth_timeoutwork, lpc54_eth_txtimeout_work,
             priv, 0);
}

/****************************************************************************
 * Name: lpc54_eth_dopoll
 *
 * Description:
 *   Check if there are Tx descriptors available and, if so, allocate a Tx
 *   then perform the normal Tx poll
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

static void lpc54_eth_dopoll(struct lpc54_ethdriver_s *priv)
{
  struct lpc54_txring_s *txring0;
#ifdef CONFIG_LPC54_ETH_MULTIQUEUE
  struct lpc54_txring_s *txring1;
#endif

  DEBUGASSERT(priv->eth_dev.d_buf == NULL);

  txring0 = &priv->eth_txring[0];
#ifdef CONFIG_LPC54_ETH_MULTIQUEUE
  txring1 = &priv->eth_txring[1];

  /* We cannot perform the Tx poll now if all of the Tx descriptors for both
   * channels are in-use.
   */

  if (txring0->tr_inuse < txring0->tr_ndesc &&
      txring1->tr_inuse < txring1->tr_ndesc)
#else
  /* We cannot perform the Tx poll now if all of the Tx descriptors for this
   * channel 0 are in-use.
   */

  if (txring0->tr_inuse < txring0->tr_ndesc)
#endif
    {
      /* There is a free descriptor in the ring, allocate a new Tx buffer
       * to perform the poll.
       */

      priv->eth_dev.d_buf = (uint8_t *)lpc54_pktbuf_alloc(priv);
      if (priv->eth_dev.d_buf != NULL)
        {
          devif_poll(&priv->eth_dev, lpc54_eth_txpoll);

          /* Make sure that the Tx buffer remaining after the poll is
           * freed.
           */

          if (priv->eth_dev.d_buf != NULL)
            {
              lpc54_pktbuf_free(priv, (uint32_t *)priv->eth_dev.d_buf);
              priv->eth_dev.d_buf = NULL;
            }
        }
    }
}

/****************************************************************************
 * Name: lpc54_eth_ifup
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

static int lpc54_eth_ifup(struct net_driver_s *dev)
{
  struct lpc54_ethdriver_s *priv =
    (struct lpc54_ethdriver_s *)dev->d_private;
  uint8_t *mptr;
  uintptr_t base;
  uint32_t regval;
  int ret;
  int i;

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

  /* Initialize the PHY *****************************************************/

  ret = lpc54_phy_autonegotiate(priv);
  if (ret < 0)
    {
      nerr("ERROR: lpc54_phy_autonegotiate failed: %d\n", ret);
      return ret;
    }

  /* Initialize Ethernet DMA ************************************************/

  /* Reset DMA.  Resets the logic and all internal registers of the OMA, MTL,
   * and MAC.  This bit is automatically cleared after the reset operation
   * is complete in all Ethernet Block clock domains.
   */

  regval  = lpc54_getreg(LPC54_ETH_DMA_MODE);
  regval |= ETH_DMA_MODE_SWR;
  lpc54_putreg(regval, LPC54_ETH_DMA_MODE);

  /* Wait for the reset bit to be cleared at the completion of the reset */

  while ((lpc54_getreg(LPC54_ETH_DMA_MODE) & ETH_DMA_MODE_SWR) != 0)
    {
    }

  /* Set the burst length for each DMA descriptor ring */

  for (i = 0; i < LPC54_NRINGS; i++)
    {
      base = LPC54_ETH_DMACH_BASE(i);
      lpc54_putreg(LPC54_PBLX8, base + LPC54_ETH_DMACH_CTRL_OFFSET);

      regval  = lpc54_getreg(base + LPC54_ETH_DMACH_TX_CTRL_OFFSET);
      regval &= ~ETH_DMACH_TX_CTRL_TXPBL_MASK;
      regval |= ETH_DMACH_TX_CTRL_TXPBL(LPC54_BURSTLEN);
      lpc54_putreg(regval, base + LPC54_ETH_DMACH_TX_CTRL_OFFSET);

      regval  = lpc54_getreg(base + LPC54_ETH_DMACH_RX_CTRL_OFFSET);
      regval &= ~ETH_DMACH_RX_CTRL_RXPBL_MASK;
      regval |= ETH_DMACH_RX_CTRL_RXPBL(LPC54_BURSTLEN);
      lpc54_putreg(regval, base + LPC54_ETH_DMACH_RX_CTRL_OFFSET);
    }

  /* Initializes the Ethernet MTL *******************************************/
#ifdef CONFIG_LPC54_ETH_MULTIQUEUE
  /* Set the schedule/arbitration for multiple queues */

  lpc54_putreg(LPC54_MTL_OPMODE_SCHALG | LPC54_MTL_OPMODE_RAA,
               LPC54_ETH_MTL_OP_MODE);

  /* Set the Rx queue mapping to DMA channel. */

  lpc54_putreg(LPC54_QUEUEMAP, LPC54_ETH_MTL_RXQ_DMA_MAP);
#endif

  /* Set transmit queue operation mode
   *
   * FTQ   - Set to flush the queue
   * TSF   - Depends on configuration
   * TXQEN - Queue 0 enabled; queue 1 may be disabled
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
  lpc54_putreg(regval | ETH_MTL_TXQ_OP_MODE_TXQEN_ENABLE,
               LPC54_ETH_MTL_TXQ_OP_MODE(0));
#ifdef CONFIG_LPC54_ETH_MULTIQUEUE
  lpc54_putreg(regval | ETH_MTL_TXQ_OP_MODE_TXQEN_ENABLE,
               LPC54_ETH_MTL_TXQ_OP_MODE(1));
#else
  lpc54_putreg(regval | ETH_MTL_TXQ_OP_MODE_TXQEN_DISABLE,
               LPC54_ETH_MTL_TXQ_OP_MODE(1));
#endif

  /* Set receive receive operation mode
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
  lpc54_putreg(regval, LPC54_ETH_MTL_RXQ_OP_MODE(0));
#ifdef CONFIG_LPC54_ETH_MULTIQUEUE
  lpc54_putreg(regval, LPC54_ETH_MTL_RXQ_OP_MODE(1));

  /* Set the Tx/Rx queue weights. */

  lpc54_putreg(CONFIG_LPC54_ETH_TXQ0WEIGHT, LPC54_ETH_MTL_TXQ_QNTM_WGHT(0));
  lpc54_putreg(CONFIG_LPC54_ETH_TXQ1WEIGHT, LPC54_ETH_MTL_TXQ_QNTM_WGHT(1));

  lpc54_putreg(CONFIG_LPC54_ETH_RXQ0WEIGHT, LPC54_ETH_MTL_RXQ_CTRL(0));
  lpc54_putreg(CONFIG_LPC54_ETH_RXQ1WEIGHT, LPC54_ETH_MTL_RXQ_CTRL(1));
#endif

  /* Initialize the Ethernet MAC ********************************************/

  /* Instantiate the MAC address that application logic should have set in
   * the device structure.
   *
   * "Note that the first DA byte that is received on the MII interface
   *  corresponds to the LS Byte (bits 7:0) of the MAC address low register.
   *  For example, if 0x1122 3344 5566 is received (0x11 is the first byte)
   *  on the MII as the destination address, then the MAC address
   *  register[47:0] is compared with 0x6655 4433 2211."
   */

  mptr   = (uint8_t *)priv->eth_dev.d_mac.ether.ether_addr_octet;
  regval = ((uint32_t)mptr[3] << 24) | ((uint32_t)mptr[2] << 16) |
           ((uint32_t)mptr[1] << 8)  | ((uint32_t)mptr[0]);
  lpc54_putreg(regval, LPC54_ETH_MAC_ADDR_LOW);

  regval = ((uint32_t)mptr[5] << 8)  | ((uint32_t)mptr[4]);
  lpc54_putreg(regval, LPC54_ETH_MAC_ADDR_HIGH);

  /* Set the receive address filter */

  regval  = ETH_MAC_FRAME_FILTER_PCF_NONE;
#ifdef CONFIG_LPC54_ETH_RX_PROMISCUOUS
  regval |= ETH_MAC_FRAME_FILTER_PR;
#endif
#ifndef CONFIG_LPC54_ETH_RX_BROADCAST
  regval |= ETH_MAC_FRAME_FILTER_DBF;
#endif
#ifdef LPC54_ACCEPT_ALLMULTICAST
  regval |= ETH_MAC_FRAME_FILTER_PM;
#endif
  lpc54_putreg(regval, LPC54_ETH_MAC_FRAME_FILTER);

#ifdef CONFIG_LPC54_ETH_FLOWCONTROL
  /* Configure flow control */

  regval = ETH_MAC_RX_FLOW_CTRL_RFE | ETH_MAC_RX_FLOW_CTRL_UP;
  lpc54_putreg(regval, LPC54_ETH_MAC_RX_FLOW_CTRL);

  regval = ETH_MAC_TX_FLOW_CTRL_Q_PT(CONFIG_LPC54_ETH_TX_PAUSETIME);
  lpc54_putreg(regval, LPC54_ETH_MAC_TX_FLOW_CTRL_Q0);
  lpc54_putreg(regval, LPC54_ETH_MAC_TX_FLOW_CTRL_Q1);
#endif

  /* Set the 1uS tick counter */

  regval = ETH_MAC_1US_TIC_COUNTR(BOARD_MAIN_CLK / USEC_PER_SEC);
  lpc54_putreg(regval, LPC54_ETH_MAC_1US_TIC_COUNTR);

  /* Set the speed and duplex using the values previously determined through
   * autonegotiaion.
   */

  regval  = ETH_MAC_CONFIG_ECRSFD | ETH_MAC_CONFIG_PS;

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

  lpc54_putreg(regval, LPC54_ETH_MAC_CONFIG);

  /* REVISIT:  The User Manual says we need to set the SYSCON sideband flow
   * control for each channel.  But it is not clear to me what setting that
   * refers to nor do I see any such settings in the NXP sample code.
   */

  /* Enable Rx queues  */

  regval = ETH_MAC_RXQ_CTRL0_RXQ0EN_ENABLE |
           ETH_MAC_RXQ_CTRL0_RXQ1EN_ENABLE;
  lpc54_putreg(regval, LPC54_ETH_MAC_RXQ_CTRL0);

  /* Setup up Ethernet interrupts */

  regval = LPC54_NORM_INTMASK | LPC54_ABNORM_INTMASK;
  lpc54_putreg(regval, LPC54_ETH_DMACH_INT_EN(0));
  lpc54_putreg(regval, LPC54_ETH_DMACH_INT_EN(1));

  lpc54_putreg(0, LPC54_ETH_MAC_INTR_EN);

  /* Initialize packet buffers */

  lpc54_pktbuf_initialize(priv);

  /* Initialize descriptors */

  lpc54_ring_initialize(priv);

  /* Activate DMA on channel 0 */

  regval  = lpc54_getreg(LPC54_ETH_DMACH_RX_CTRL(0));
  regval |= ETH_DMACH_RX_CTRL_SR;
  lpc54_putreg(regval, LPC54_ETH_DMACH_RX_CTRL(0));

  regval  = lpc54_getreg(LPC54_ETH_DMACH_TX_CTRL(0));
  regval |= ETH_DMACH_TX_CTRL_ST;
  lpc54_putreg(regval, LPC54_ETH_DMACH_TX_CTRL(0));

  /* Then enable the Rx/Tx */

  regval  = lpc54_getreg(LPC54_ETH_MAC_CONFIG);
  regval |= ETH_MAC_CONFIG_RE;
  lpc54_putreg(regval, LPC54_ETH_MAC_CONFIG);

  regval |= ETH_MAC_CONFIG_TE;
  lpc54_putreg(regval, LPC54_ETH_MAC_CONFIG);

  /* Enable the Ethernet interrupt */

  priv->eth_bifup = 1;
  up_enable_irq(LPC54_IRQ_ETHERNET);
  return OK;
}

/****************************************************************************
 * Name: lpc54_eth_ifdown
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

static int lpc54_eth_ifdown(struct net_driver_s *dev)
{
  struct lpc54_ethdriver_s *priv =
    (struct lpc54_ethdriver_s *)dev->d_private;
  irqstate_t flags;
  uint32_t regval;
  int ret;

  /* Disable the Ethernet interrupt */

  flags = enter_critical_section();
  up_disable_irq(LPC54_IRQ_ETHERNET);

  /* Cancel the TX timeout timers */

  wd_cancel(&priv->eth_txtimeout);

  /* Put the EMAC in its post-reset, non-operational state.  This should be
   * a known configuration that will guarantee the lpc54_eth_ifup() always
   * successfully brings the interface back up.
   *
   * Reset the Ethernet interface.
   */

  lpc54_reset_eth();

  /* Set the CSR clock divider */

  lpc54_set_csrdiv();

  /* Select MII or RMII mode */

  regval  = lpc54_getreg(LPC54_SYSCON_ETHPHYSEL);
  regval &= ~SYSCON_ETHPHYSEL_MASK;
#ifdef CONFIG_LPC54_ETH_MII
  regval |= SYSCON_ETHPHYSEL_MII;
#else
  regval |= SYSCON_ETHPHYSEL_RMII;
#endif
  lpc54_putreg(regval, LPC54_SYSCON_ETHPHYSEL);

  /* Reset the PHY and bring it to an operational state.  We must be capable
   * of handling PHY ioctl commands while the network is down.
   */

  ret = lpc54_phy_reset(priv);
  if (ret < 0)
    {
      nerr("ERROR: lpc54_phy_reset failed: %d\n", ret);
      leave_critical_section(flags);
      return ret;
    }

  /* Mark the device "down" */

  priv->eth_bifup = 0;
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: lpc54_eth_txavail_work
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
      /* Poll the network for new XMIT data. */

      lpc54_eth_dopoll(priv);
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

static int lpc54_eth_txavail(struct net_driver_s *dev)
{
  struct lpc54_ethdriver_s *priv =
    (struct lpc54_ethdriver_s *)dev->d_private;

  /* Is our single work structure available?  It may not be if there are
   * pending interrupt actions and we will have to ignore the Tx
   * availability action.
   */

  if (work_available(&priv->eth_pollwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      work_queue(ETHWORK, &priv->eth_pollwork,
                 lpc54_eth_txavail_work, priv, 0);
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
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be added
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_MCASTGROUP
static int lpc54_eth_addmac(struct net_driver_s *dev, const uint8_t *mac)
{
  /* Unlike other Ethernet hardware, the LPC54xx does not seem to support
   * explicit Multicast address filtering as needed for ICMPv6 and for IGMP.
   * In these cases, I am simply accepting all multicast packets.
   */

  return OK;
}
#endif

/****************************************************************************
 * Name: lpc54_eth_rmmac
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
static int lpc54_eth_rmmac(struct net_driver_s *dev, const uint8_t *mac)
{
  /* Unlike other Ethernet hardware, the LPC54xx does not seem to support
   * explicit Multicast address filtering as needed for ICMPv6 and for IGMP.
   * In these cases, I am simply accepting all multicast packets.
   */

  return -ENOSYS;
}
#endif

/****************************************************************************
 * Name: lpc54_eth_ioctl
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
static int lpc54_eth_ioctl(struct net_driver_s *dev, int cmd,
                           unsigned long arg)
{
#ifdef CONFIG_NETDEV_PHY_IOCTL
  struct lpc54_ethdriver_s *priv =
    (struct lpc54_ethdriver_s *)dev->d_private;
#endif
  int ret;

  /* Decode and dispatch the driver-specific IOCTL command */

  switch (cmd)
    {
#ifdef CONFIG_NETDEV_PHY_IOCTL
     case SIOCGMIIPHY: /* Get MII PHY address */
        {
          struct mii_ioctl_data_s *req =
        (struct mii_ioctl_data_s *)((uintptr_t)arg);
          req->phy_id = CONFIG_LPC54_ETH_PHYADDR;
          ret = OK;
        }
        break;

      case SIOCGMIIREG: /* Get register from MII PHY */
        {
          struct mii_ioctl_data_s *req =
        (struct mii_ioctl_data_s *)((uintptr_t)arg);
          req->val_out = lpc54_phy_read(priv, req->reg_num);
          ret = OK
        }
        break;

      case SIOCSMIIREG: /* Set register in MII PHY */
        {
          struct mii_ioctl_data_s *req =
        (struct mii_ioctl_data_s *)((uintptr_t)arg);
          lpc54_phy_write(priv, req->reg_num, req->val_in);
          ret = OK
        }
        break;
#endif /* ifdef CONFIG_NETDEV_PHY_IOCTL */

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
 * Input Parameters:
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
      sq_addlast((sq_entry_t *)pktbuf, &priv->eth_freebuf);
    }
}

/****************************************************************************
 * Name: lpc54_pktbuf_alloc
 *
 * Description:
 *   Allocate one packet buffer by removing it from the free list.
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   A pointer to the allocated packet buffer on success; NULL is returned if
 *   there are no available packet buffers.
 *
 * Assumptions:
 *   The network must be locked.  Mutually exclusive access to the free list
 *   is maintained by locking the network.
 *
 ****************************************************************************/

static inline uint32_t *lpc54_pktbuf_alloc(struct lpc54_ethdriver_s *priv)
{
  return (uint32_t *)sq_remfirst(&priv->eth_freebuf);
}

/****************************************************************************
 * Name: lpc54_pktbuf_free
 *
 * Description:
 *   Allocate one packet buffer by removing it from the free list.
 *
 * Input Parameters:
 *   priv   - Reference to the driver state structure
 *   pktbuf - The packet buffer to be freed
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network must be locked.  Mutually exclusive access to the free list
 *   is maintained by locking the network.
 *
 ****************************************************************************/

static inline void lpc54_pktbuf_free(struct lpc54_ethdriver_s *priv,
                                     uint32_t *pktbuf)
{
  sq_addlast((sq_entry_t *)pktbuf, &priv->eth_freebuf);
}

/****************************************************************************
 * Name: lpc54_txring_initialize
 *
 * Description:
 *   Initialize one Tx descriptor ring.
 *
 * Input Parameters:
 *   priv   - Reference to the driver state structure
 *   chan   - Channel being initialized
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lpc54_txring_initialize(struct lpc54_ethdriver_s *priv,
                                    unsigned int chan)
{
  struct lpc54_txring_s *txring;
  struct enet_txdesc_s *txdesc;
  uint32_t regval;
  int i;

  txring  = &priv->eth_txring[chan];
  txdesc  = txring->tr_desc;

  /* Set the word-aligned Tx descriptor start/tail pointers. */

  regval  = (uint32_t)txdesc;
  lpc54_putreg(regval, LPC54_ETH_DMACH_TXDESC_LIST_ADDR(chan));

  regval += txring->tr_ndesc * sizeof(struct enet_txdesc_s);
  lpc54_putreg(regval, LPC54_ETH_DMACH_TXDESC_TAIL_PTR(chan));

  /* Set the Tx ring length */

  regval = ETH_DMACH_TXDESC_RING_LENGTH(txring->tr_ndesc);
  lpc54_putreg(regval, LPC54_ETH_DMACH_TXDESC_RING_LENGTH(chan));

  /* Inituialize the Tx descriptors . */

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
 * Input Parameters:
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
  struct enet_rxdesc_s *rxdesc;
  uint32_t regval;
  int i;

  rxring  = &priv->eth_rxring[chan];
  rxdesc  = rxring->rr_desc;

  /* Set the word-aligned Rx descriptor start/tail pointers. */

  regval  = (uint32_t)rxdesc;
  lpc54_putreg(regval, LPC54_ETH_DMACH_RXDESC_LIST_ADDR(chan));

  regval += rxring->rr_ndesc * sizeof(struct enet_rxdesc_s);
  lpc54_putreg(regval, LPC54_ETH_DMACH_RXDESC_TAIL_PTR(chan));

  /* Set the Rx ring length */

  regval = ETH_DMACH_RXDESC_RING_LENGTH(rxring->rr_ndesc);
  lpc54_putreg(regval, LPC54_ETH_DMACH_RXDESC_RING_LENGTH(chan));

  /* Set the receive buffer size (in words) in the Rx control register */

  regval  = lpc54_getreg(LPC54_ETH_DMACH_RX_CTRL(chan));
  regval &= ~ETH_DMACH_RX_CTRL_RBSZ_MASK;
  regval |= ETH_DMACH_RX_CTRL_RBSZ(LPC54_BUFFER_SIZE >> 2);
  lpc54_putreg(regval, LPC54_ETH_DMACH_RX_CTRL(chan));

  /* Initialize the Rx descriptor ring. */

  regval  = ETH_RXDES3_BUF1V | ETH_RXDES3_IOC | ETH_RXDES3_OWN;
#if LPC54_BUFFER_SIZE > LPC54_BUFFER_MAX
  regval |= ETH_RXDES3_BUF2V;
#endif

  for (i = 0; i < rxring->rr_ndesc; i++, rxdesc++)
    {
      /* Allocate the first Rx packet buffer */

      rxdesc->buffer1 = (uint32_t)lpc54_pktbuf_alloc(priv);
      DEBUGASSERT(rxdesc->buffer1 != 0);
      (rxring->rr_buffers)[i] = (uint32_t *)rxdesc->buffer1;

#if LPC54_BUFFER_SIZE > LPC54_BUFFER_MAX
      /* Configure the second part of a large packet buffer as buffer2 */

      rxdesc->buffer2 = rxdesc->buffer1 + LPC54_BUFFER_MAX;
#else
      /* The second buffer is not used */

      rxdesc->buffer2 = 0;
#endif

      /* Buffer1 and maybe 2 valid, interrupt on completion, owned by DMA. */

      rxdesc->ctrl = regval;
    }
}

/****************************************************************************
 * Name: lpc54_ring_initialize
 *
 * Description:
 *   Initialize the Rx and Tx rings for every channel.
 *
 * Input Parameters:
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

  priv->eth_txring[0].tr_desc     = g_ch0_txdesc;
  priv->eth_txring[0].tr_ndesc    = CONFIG_LPC54_ETH_NTXDESC0;
  priv->eth_txring[0].tr_buffers  = g_txbuffers0;
  lpc54_txring_initialize(priv, 0);

  priv->eth_rxring[0].rr_desc     = g_ch0_rxdesc;
  priv->eth_rxring[0].rr_ndesc    = CONFIG_LPC54_ETH_NRXDESC0;
  priv->eth_rxring[0].rr_buffers  = g_rxbuffers0;
  lpc54_rxring_initialize(priv, 0);

#ifdef CONFIG_LPC54_ETH_MULTIQUEUE
  /* Initialize channel 1 rings */

  memset(g_txbuffers1, 0, CONFIG_LPC54_ETH_NTXDESC1 * sizeof(uint32_t *));
  memset(g_rxbuffers1, 0, CONFIG_LPC54_ETH_NRXDESC1 * sizeof(uint32_t *));

  priv->eth_txring[1].tr_desc     = g_ch1_txdesc;
  priv->eth_txring[1].tr_ndesc    = CONFIG_LPC54_ETH_NTXDESC1;
  priv->eth_txring[1].tr_buffers  = g_txbuffers1;
  lpc54_txring_initialize(priv, 1);

  priv->eth_rxring[1].rr_desc     = g_ch1_rxdesc;
  priv->eth_rxring[1].rr_ndesc    = CONFIG_LPC54_ETH_NRXDESC1;
  priv->eth_rxring[1].rr_buffers  = g_rxbuffers1;
  lpc54_rxring_initialize(priv, 1);
#endif
}

/****************************************************************************
 * Name: lpc54_set_csrdiv
 *
 * Description:
 *   Set the CSR clock divider.  The MDC clock derives from the divided down
 *   CSR clock (aka core clock or main clock).
 *
 * Input Parameters:
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

  regval = lpc54_getreg(LPC54_ETH_MAC_MDIO_ADDR);
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
      regval |= ETH_MAC_MDIO_ADDR_CR_DIV42;    /* CSR=60-100 MHz; MDC=CSR/42 */
    }
  else /* if (srcclk < 150) */
    {
      regval |= ETH_MAC_MDIO_ADDR_CR_DIV62;    /* CSR=100-150 MHz; MDC=CSR/62 */
    }

  lpc54_putreg(regval, LPC54_ETH_MAC_MDIO_ADDR);
}

/****************************************************************************
 * Name: lpc54_phy_read
 *
 * Description:
 *   Read the content from one PHY register.
 *
 * Input Parameters:
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
  uint32_t regval;

  /* Set the MII read command. */

  regval  = lpc54_getreg(LPC54_ETH_MAC_MDIO_ADDR);
  regval &= ETH_MAC_MDIO_ADDR_CR_MASK;
  regval |= ETH_MAC_MDIO_ADDR_MOC_READ | ETH_MAC_MDIO_ADDR_RDA(phyreg) |
            ETH_MAC_MDIO_ADDR_PA(CONFIG_LPC54_ETH_PHYADDR);
  lpc54_putreg(regval, LPC54_ETH_MAC_MDIO_ADDR);

  /* Initiate the read */

  regval |= ETH_MAC_MDIO_ADDR_MB;
  lpc54_putreg(regval, LPC54_ETH_MAC_MDIO_ADDR);

  /* Wait until the SMI is no longer busy with the read */

  while ((lpc54_getreg(LPC54_ETH_MAC_MDIO_ADDR) & ETH_MAC_MDIO_ADDR_MB) != 0)
    {
    }

  return (uint16_t)lpc54_getreg(LPC54_ETH_MAC_MDIO_DATA);
}

/****************************************************************************
 * Name: lpc54_phy_write
 *
 * Description:
 *   Write a new value to of one PHY register.
 *
 * Input Parameters:
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
  uint32_t regval;

  /* Set the MII write command. */

  regval  = lpc54_getreg(LPC54_ETH_MAC_MDIO_ADDR);
  regval &= ETH_MAC_MDIO_ADDR_CR_MASK;
  regval |= ETH_MAC_MDIO_ADDR_MOC_WRITE | ETH_MAC_MDIO_ADDR_RDA(phyreg) |
            ETH_MAC_MDIO_ADDR_PA(CONFIG_LPC54_ETH_PHYADDR);
  lpc54_putreg(regval, LPC54_ETH_MAC_MDIO_ADDR);

  /* Set the write data */

  lpc54_putreg((uint32_t)phyval, LPC54_ETH_MAC_MDIO_DATA);

  /* Initiate the write */

  regval |= ETH_MAC_MDIO_ADDR_MB;
  lpc54_putreg(regval, LPC54_ETH_MAC_MDIO_ADDR);

  /* Wait until the SMI is no longer busy with the write */

  while ((lpc54_getreg(LPC54_ETH_MAC_MDIO_ADDR) & ETH_MAC_MDIO_ADDR_MB) != 0)
    {
    }
}

/****************************************************************************
 * Name: lpc54_phy_linkstatus
 *
 * Description:
 *   Read the MII status register and return tru if the link is up.
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   true if the link is up
 *
 ****************************************************************************/

static inline bool lpc54_phy_linkstatus(struct lpc54_ethdriver_s *priv)
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
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

static int lpc54_phy_autonegotiate(struct lpc54_ethdriver_s *priv)
{
  volatile int32_t timeout;
  uint16_t phyval;

  /* Advertise our capabilities. */

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
          nerr("ERROR: Autonegotiation timed out\n");
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
          nerr("ERROR: Link status UP timed out\n");
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
 * Input Parameters:
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
          nerr("ERROR: PHY start up timed out\n");
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
          nerr("ERROR: PHY reset timed out\n");
          return -ETIMEDOUT;
        }

      phyval = lpc54_phy_read(priv, MII_MCR);
    }
  while ((phyval & MII_MCR_RESET) != 0);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_netinitialize
 *
 * Description:
 *   Initialize the Ethernet controller and driver.
 *
 *   This is the "standard" network initialization logic called from the
 *   low-level initialization logic in arm_initialize.c.
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

void arm_netinitialize(void)
{
  struct lpc54_ethdriver_s *priv;
  int ret;

  /* Get the interface structure associated with this interface number. */

  priv = &g_ethdriver;

  /* Attach the three Ethernet-related IRQs to the handlers */

  ret = irq_attach(LPC54_IRQ_ETHERNET, lpc54_eth_interrupt, priv);
  if (ret < 0)
    {
      /* We could not attach the ISR to the interrupt */

      nerr("ERROR: irq_attach failed: %d\n", ret);
      return;
    }

#if 0 /* Not used */
  ret = irq_attach(LPC54_IRQ_ETHERNETPMT, lpc54_pmt_interrupt, priv);
  if (ret < 0)
    {
      /* We could not attach the ISR to the interrupt */

      nerr("ERROR:  irq_attach for PMT failed: %d\n", ret);
      return;
    }

  ret = irq_attach(LPC54_IRQ_ETHERNETMACLP, lpc54_mac_interrupt, priv);
  if (ret < 0)
    {
      /* We could not attach the ISR to the interrupt */

      nerr("ERROR:  irq_attach for MAC failed: %d\n", ret);
      return;
    }
#endif

  /* Initialize the driver structure */

  memset(priv, 0, sizeof(struct lpc54_ethdriver_s));
  priv->eth_dev.d_ifup    = lpc54_eth_ifup;     /* I/F up (new IP address) callback */
  priv->eth_dev.d_ifdown  = lpc54_eth_ifdown;   /* I/F down callback */
  priv->eth_dev.d_txavail = lpc54_eth_txavail;  /* New TX data callback */
#ifdef CONFIG_NET_MCASTGROUP
  priv->eth_dev.d_addmac  = lpc54_eth_addmac;   /* Add multicast MAC address */
  priv->eth_dev.d_rmmac   = lpc54_eth_rmmac;    /* Remove multicast MAC address */
#endif
#ifdef CONFIG_NETDEV_IOCTL
  priv->eth_dev.d_ioctl   = lpc54_eth_ioctl;    /* Handle network IOCTL commands */
#endif
  priv->eth_dev.d_private = &g_ethdriver;       /* Used to recover private state from dev */

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
   *   REF_CLK may be available in some implementations.  Clocking from
   *     PHY appears to be necessary for DMA reset operations.
   *   RX_ER is optional on switches.
   */

  lpc54_gpio_config(GPIO_ENET_RXD0);    /* Ethernet receive data 0-1 */
  lpc54_gpio_config(GPIO_ENET_RXD1);
  lpc54_gpio_config(GPIO_ENET_TXD0);    /* Ethernet transmit data 0-1 */
  lpc54_gpio_config(GPIO_ENET_TXD1);
  lpc54_gpio_config(GPIO_ENET_RX_DV);   /* Ethernet receive data valid */
  lpc54_gpio_config(GPIO_ENET_TX_EN);   /* Ethernet transmit data enable */
  lpc54_gpio_config(GPIO_ENET_REF_CLK); /* PHY reference clock */
#endif

  /* Enable clocking to the Ethernet peripheral */

  lpc54_eth_enableclk();

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
      goto errout_with_clock;
    }

  return;

errout_with_clock:
  lpc54_eth_disableclk();
}

#endif /* CONFIG_LPC54_ETHERNET */
