/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_ethernet.c
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

#include <inttypes.h>
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
#include <nuttx/wqueue.h>
#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/net/mii.h>
#include <nuttx/net/gmii.h>
#include <nuttx/net/arp.h>
#include <nuttx/net/netdev.h>

#ifdef CONFIG_NET_PKT
#  include <nuttx/net/pkt.h>
#endif

#include "riscv_internal.h"
#include "mpfs_memorymap.h"
#include "mpfs_ethernet.h"

#if defined(CONFIG_NET) && defined(CONFIG_MPFS_ETHMAC)

#ifndef OK
#  define OK 0
#endif

#define MPFS_PMPCFG_ETH0_0   (MPFS_MPUCFG_BASE + 0x400)
#define MPFS_PMPCFG_ETH0_1   (MPFS_MPUCFG_BASE + 0x408)
#define MPFS_PMPCFG_ETH0_2   (MPFS_MPUCFG_BASE + 0x410)
#define MPFS_PMPCFG_ETH0_3   (MPFS_MPUCFG_BASE + 0x418)
#define MPFS_PMPCFG_ETH1_0   (MPFS_MPUCFG_BASE + 0x500)
#define MPFS_PMPCFG_ETH1_1   (MPFS_MPUCFG_BASE + 0x508)
#define MPFS_PMPCFG_ETH1_2   (MPFS_MPUCFG_BASE + 0x510)
#define MPFS_PMPCFG_ETH1_3   (MPFS_MPUCFG_BASE + 0x518)

#if defined(CONFIG_MPFS_ETHMAC_0) && defined(CONFIG_MPFS_ETHMAC_1)
#  warning "Using 2 MACs is not yet supported."
#  define MPFS_NETHERNET     (2)
#else
#  define MPFS_NETHERNET     (1)
#endif

#define MPFS_MAC_QUEUE_COUNT (4)

#if !defined(CONFIG_SCHED_WORKQUEUE)
#  error Work queue support is required
#else

/* Select work queue */

#  if defined(CONFIG_MPFS_ETHMAC_HPWORK)
#    define ETHWORK HPWORK
#  elif defined(CONFIG_MPFS_ETHMAC_LPWORK)
#    define ETHWORK LPWORK
#  else
#    define ETHWORK LPWORK
#  endif
#endif

#ifndef CONFIG_MPFS_PHYADDR
#  error "CONFIG_MPFS_PHYADDR must be defined in the NuttX configuration"
#endif

#if !defined(CONFIG_MPFS_MAC_SGMII) && !defined(CONFIG_MPFS_MAC_GMII)
#  warning "Neither CONFIG_MPFS_MAC_SGMII nor CONFIG_MPFS_MAC_GMII defined"
#endif

#if defined(CONFIG_MPFS_MAC_SGMII) && defined(CONFIG_MPFS_MAC_GMII)
#  error "Both CONFIG_MPFS_MAC_SGMII and CONFIG_MPFS_MAC_GMII defined"
#endif

/* Number of buffers for RX */

#ifndef CONFIG_MPFS_ETHMAC_NRXBUFFERS
#  define CONFIG_MPFS_ETHMAC_NRXBUFFERS  (16)
#endif

/* Number of buffers for TX */

#ifndef CONFIG_MPFS_ETHMAC_NTXBUFFERS
#  define CONFIG_MPFS_ETHMAC_NTXBUFFERS  (8)
#endif

/* GMAC buffer sizes
 * REVISIT: do we want to make GMAC_RX_UNITSIZE configurable?
 * issue when using the MTU size receive block
 */

#define GMAC_RX_UNITSIZE  (512)                  /* Fixed size for RX buffer */
#define GMAC_TX_UNITSIZE  CONFIG_NET_ETH_PKTSIZE /* MAX size for Ethernet packet */

/* The MAC can support frame lengths up to 1536 bytes */

#define GMAC_MAX_FRAMELEN (1536)
#if CONFIG_NET_ETH_PKTSIZE > GMAC_MAX_FRAMELEN
#  error CONFIG_NET_ETH_PKTSIZE is too large
#endif

/* for DMA dma_rxbuf_size */

#define MPFS_MAC_RX_BUF_VALUE ((GMAC_RX_UNITSIZE + 63) / 64)

/* We need at least one more free buffer than transmit buffers */

#define MPFS_GMAC_NFREEBUFFERS (CONFIG_MPFS_ETHMAC_NTXBUFFERS + 1)

#ifdef CONFIG_NET_DUMPPACKET
#  define mpfs_dumppacket(m,a,n) lib_dumpbuffer(m,a,n)
#else
#  define mpfs_dumppacket(m,a,n)
#endif

#ifdef CONFIG_MPFS_HAVE_CORERMII
#  define CORE_RMII_RESET       (1 << 15)
#  define CORE_RMII_LOOPBACK    (1 << 2)
#  define CORE_RMII_FULL_DUPLEX (1 << 1)
#  define CORE_RMII_100MBIT     (1 << 0)
#endif

/* Timing *******************************************************************/

/* TX timeout = 1 minute */

#define MPFS_TXTIMEOUT   (60 * CLK_TCK)

/* PHY reset tim in loop counts */

#define PHY_RESET_WAIT_COUNT (10)

/* PHY re-try coount in loop counts */

#define PHY_RETRY_MAX   300000

/* This is a helper pointer for accessing the contents of the GMAC header */

#define BUF ((struct eth_hdr_s *)priv->dev.d_buf)

#define upper_32_bits(n) ((uint32_t)(((n) >> 16) >> 16))
#define lower_32_bits(n) ((uint32_t)(n))

/* Interrupt flags */

#define INT_RX     (GEM_INT_RECEIVE_COMPLETE)
#define INT_TX     (GEM_INT_TRANSMIT_COMPLETE)
#define INT_ERRORS (GEM_INT_TRANSMIT_BUFFER_UNDERRUN | \
                    GEM_INT_RECEIVE_OVERRUN | \
                    GEM_INT_AMBA_ERROR | GEM_INT_RESP_NOT_OK)
#define INT_ALL    (INT_RX | INT_TX | INT_ERRORS)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct gmac_rxdesc_s
{
  uint32_t addr;    /* Buffer address */
  uint32_t status;  /* RX status and controls */
#if defined(CONFIG_MPFS_ETHMAC_64BIT_ADDRESS_MODE)
  uint32_t addr_hi; /* Buffer address high 32-bits */
  uint32_t unused;
#endif
};

/* Transmit buffer descriptor */

struct gmac_txdesc_s
{
  uint32_t addr;    /* Buffer address */
  uint32_t status;  /* TX status and controls */
#if defined(CONFIG_MPFS_ETHMAC_64BIT_ADDRESS_MODE)
  uint32_t addr_hi; /* Buffer address high 32-bits */
  uint32_t unused;  /*  */
#endif
};

static uint8_t g_pktbuf[MPFS_NETHERNET]
                       [MAX_NETDEV_PKTSIZE + CONFIG_NET_GUARDSIZE];

#if defined(CONFIG_MPFS_GMAC_PREALLOCATE)
static uint8_t g_txbuffer[CONFIG_MPFS_ETHMAC_NTXBUFFERS * GMAC_TX_UNITSIZE]
               aligned_data(8);
static uint8_t g_rxbuffer[CONFIG_MPFS_ETHMAC_NRXBUFFERS * GMAC_RX_UNITSIZE]
               aligned_data(8);
#endif

struct mpfs_mac_queue_s
{
#if defined(CONFIG_MPFS_GMAC_PREALLOCATE)
  struct gmac_txdesc_s tx_desc_tab[CONFIG_MPFS_ETHMAC_NTXBUFFERS];  /* Transmit descriptor table */
  struct gmac_rxdesc_s rx_desc_tab[CONFIG_MPFS_ETHMAC_NRXBUFFERS];  /* Receive descriptor table */
#else
  struct gmac_txdesc_s *tx_desc_tab;     /* Transmit descriptor table */
  struct gmac_rxdesc_s *rx_desc_tab;     /* Receive descriptor table */
  uint8_t              *rxbuffer;        /* Allocated RX buffers */
  uint8_t              *txbuffer;        /* Allocated TX buffers */
#endif
  uint32_t             txhead;           /* Circular buffer head index */
  uint32_t             txtail;           /* Circular buffer tail index */
  uint32_t             rxndx;            /* RX index for current processing RX descriptor */
  volatile uint32_t    *int_status;      /* interrupt status */
  volatile uint32_t    *int_mask;        /* interrupt mask */
  volatile uint32_t    *int_enable;      /* interrupt enable */
  volatile uint32_t    *int_disable;     /* interrupt disable */
  volatile uint32_t    *rx_q_ptr;        /* RX queue pointer */
  volatile uint32_t    *tx_q_ptr;        /* TX queue pointer */
  volatile uint32_t    *dma_rxbuf_size;  /* RX queue buffer size */
};

/* The mpfs_ethmac_s encapsulates all state information for a single
 * hardware interface
 */

struct mpfs_ethmac_s
{
  uintptr_t     regbase;                         /* mac base address */
  irq_t         mac_q_int[MPFS_MAC_QUEUE_COUNT]; /* irq numbers */
  uint8_t       ifup : 1;                        /* true:ifup false:ifdown */
  uint8_t       intf;                            /* Ethernet interface number */
  uint8_t       phyaddr;                         /* PHY address */
  struct wdog_s txtimeout;                       /* TX timeout timer */
  struct work_s irqwork;                         /* For deferring interrupt work to the work queue */
  struct work_s pollwork;                        /* For deferring poll work to the work queue */

  /* This holds the information visible to the NuttX network */

  struct net_driver_s     dev;  /* Interface understood by the network */
  struct mpfs_mac_queue_s queue[MPFS_MAC_QUEUE_COUNT];
};

/* These are the pre-allocated Ethernet device structures */

static struct mpfs_ethmac_s g_mpfsethmac[MPFS_NETHERNET];

static uintptr_t g_regbases[MPFS_NETHERNET] =
{
#ifdef CONFIG_MPFS_ETHMAC_0
  MPFS_GEM0_LO_BASE,
#endif
#ifdef CONFIG_MPFS_ETHMAC_1
  MPFS_GEM1_LO_BASE,
#endif
  /* if support for emacs is added later
   * (MPFS_GEM0_LO_BASE + 0x1000),
   * (MPFS_GEM1_LO_BASE + 0x1000),'
   */
};

static const irq_t g_irq_numbers[MPFS_NETHERNET][MPFS_MAC_QUEUE_COUNT] =
{
#ifdef CONFIG_MPFS_ETHMAC_0
  { MPFS_IRQ_MAC0_INT, MPFS_IRQ_MAC0_QUEUE1,
    MPFS_IRQ_MAC0_QUEUE2, MPFS_IRQ_MAC0_QUEUE3 },
#endif
#ifdef CONFIG_MPFS_ETHMAC_1
  { MPFS_IRQ_MAC1_INT, MPFS_IRQ_MAC1_QUEUE1,
    MPFS_IRQ_MAC1_QUEUE2, MPFS_IRQ_MAC1_QUEUE3 },
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register operations ******************************************************/

static uint16_t mpfs_txinuse(struct mpfs_ethmac_s *priv, unsigned int queue);
static uint16_t mpfs_txfree(struct mpfs_ethmac_s *priv, unsigned int queue);
static int mpfs_buffer_initialize(struct mpfs_ethmac_s *priv,
                                  unsigned int queue);
static void mpfs_buffer_free(struct mpfs_ethmac_s *priv, unsigned int queue);

static int  mpfs_transmit(struct mpfs_ethmac_s *priv, unsigned int queue);
static int  mpfs_txpoll(struct net_driver_s *dev);
static void mpfs_dopoll(struct mpfs_ethmac_s *priv);

/* NuttX callback functions */

static int  mpfs_ifup(struct net_driver_s *dev);
static int  mpfs_ifdown(struct net_driver_s *dev);

static void mpfs_txavail_work(void *arg);
static int  mpfs_txavail(struct net_driver_s *dev);

#ifdef CONFIG_NETDEV_PHY_IOCTL
static int  mpfs_ioctl(struct net_driver_s *dev, int cmd,
                       unsigned long arg);
#endif

#if defined(CONFIG_NET_MCASTGROUP) || defined(CONFIG_NET_ICMPv6)
static unsigned int mpfs_hashindx(const uint8_t *mac);
static int  mpfs_addmac(struct net_driver_s *dev, const uint8_t *mac);
#endif
#ifdef CONFIG_NET_MCASTGROUP
static int mpfs_rmmac(struct net_driver_s *dev, const uint8_t *mac);
#endif

#ifdef CONFIG_NETDEV_IOCTL
static int mpfs_ioctl(struct net_driver_s *dev, int cmd, unsigned long arg);
#endif

/* PHY Initialization */

static void mpfs_enablemdio(struct mpfs_ethmac_s *priv);
static void mpfs_disablemdio(struct mpfs_ethmac_s *priv);
static int  mpfs_phyreset(struct mpfs_ethmac_s *priv);
static int  mpfs_phyinit(struct mpfs_ethmac_s *priv);
static int  mpfs_phyread(struct mpfs_ethmac_s *priv, uint8_t phyaddr,
                         uint8_t regaddr, uint16_t *phyval);
static int  mpfs_phywrite(struct mpfs_ethmac_s *priv, uint8_t phyaddr,
                          uint8_t regaddr, uint16_t phyval);
static int  mpfs_phywait(struct mpfs_ethmac_s *priv);
static int  mpfs_phyfind(struct mpfs_ethmac_s *priv, uint8_t *phyaddr);
#ifdef CONFIG_MPFS_MAC_AUTONEG
static int  mpfs_autonegotiate(struct mpfs_ethmac_s *priv);
#else
static void mpfs_linkspeed(struct mpfs_ethmac_s *priv);
#endif

#if defined(CONFIG_DEBUG_NET) && defined(CONFIG_DEBUG_INFO)
static void mpfs_phydump(struct mpfs_ethmac_s *priv);
#else
#  define mpfs_phydump(priv)
#endif

/* MAC/DMA Initialization */

static void mpfs_txreset(struct mpfs_ethmac_s *priv);
static void mpfs_rxreset(struct mpfs_ethmac_s *priv);
static int  mpfs_macenable(struct mpfs_ethmac_s *priv);
static int  mpfs_ethconfig(struct mpfs_ethmac_s *priv);
static void mpfs_ethreset(struct mpfs_ethmac_s *priv);
#ifdef CONFIG_NET_ICMPv6
static void mpfs_ipv6multicast(struct sam_gmac_s *priv);
#endif

static void mpfs_interrupt_work(void *arg);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mac_putreg
 *
 * Description:
 *  Write a value to an MAC register
 *
 * Input Parameters:
 *   val - The value to write to the register
 *   offset - The mac register address offset to write
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void mac_putreg(struct mpfs_ethmac_s *priv,
                              uint32_t offset, uint32_t val)
{
  uint32_t *addr = (uint32_t *)(priv->regbase + offset);
#if defined(CONFIG_DEBUG_NET) && defined(CONFIG_MPFS_ETHMAC_REGDEBUG)
  ninfo("0x%08x <- 0x%08x\n", addr, val);
#endif
  putreg32(val, addr);
}

/****************************************************************************
 * Name: mac_getreg
 *
 * Description:
 *   Read a value from an MAC register
 *
 * Input Parameters:
 *   priv -
 *   offset - The register address offset to read
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline uint32_t mac_getreg(struct mpfs_ethmac_s *priv,
                                  uint32_t offset)
{
  uint32_t *addr = (uint32_t *)(priv->regbase + offset);
  uint32_t value = getreg32(addr);
#if defined(CONFIG_DEBUG_NET) && defined(CONFIG_MPFS_ETHMAC_REGDEBUG)
  ninfo("0x%08x -> 0x%08x\n", addr, value);
#endif
  return value;
}

static int mpfs_interrupt_0(int irq, void *context, void *arg)
{
  struct mpfs_ethmac_s *priv = (struct mpfs_ethmac_s *)arg;
  uint32_t isr;
  UNUSED(irq);
  UNUSED(context);

  /* Disable further Ethernet interrupts. */

  *priv->queue[0].int_disable = 0xffffffff;
  isr = *priv->queue[0].int_status;
  ninfo("isr=0x%" PRIx32 "\n", isr);

  /* clear all pending... */

  *priv->queue[0].int_status = isr;

  if ((isr & GEM_INT_TRANSMIT_COMPLETE) != 0)
    {
      /* If a TX transfer just completed, then cancel the TX timeout */

      nwarn("TX complete: cancel timeout\n");
      wd_cancel(&priv->txtimeout);
    }

  /* Schedule to perform the interrupt processing on the worker thread. */

  work_queue(ETHWORK, &priv->irqwork, mpfs_interrupt_work, priv, 0);

  return OK;
}

static int mpfs_interrupt_1(int irq, void *context, void *arg)
{
  struct mpfs_ethmac_s *priv = (struct mpfs_ethmac_s *)arg;
  UNUSED(priv);
  UNUSED(irq);
  UNUSED(context);

  ninfo("IRQ-1");
  return 0;
}

static int mpfs_interrupt_2(int irq, void *context, void *arg)
{
  struct mpfs_ethmac_s *priv = (struct mpfs_ethmac_s *)arg;
  UNUSED(priv);
  UNUSED(irq);
  UNUSED(context);

  ninfo("IRQ-2");
  return 0;
}

static int mpfs_interrupt_3(int irq, void *context, void *arg)
{
  struct mpfs_ethmac_s *priv = (struct mpfs_ethmac_s *)arg;
  UNUSED(priv);
  UNUSED(irq);
  UNUSED(context);

  ninfo("IRQ-3");
  return 0;
}

/****************************************************************************
 * Function: mpfs_txdone
 *
 * Description:
 *   An interrupt was received indicating that one or more frames have
 *   completed transmission.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *   queue - The queue number that completed
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by the watchdog logic.
 *
 ****************************************************************************/

static void mpfs_txdone(struct mpfs_ethmac_s *priv, unsigned int queue)
{
  struct gmac_txdesc_s *txdesc;

  /* Are there any outstanding transmissions?  Loop until either (1) all of
   * the TX descriptors have been examined, or (2) until we encounter the
   * first descriptor that is still in use by the hardware.
   */

  while (priv->queue[queue].txhead != priv->queue[queue].txtail)
    {
      /* Yes. check the next buffer at the tail of the list */

      txdesc = &priv->queue[queue].tx_desc_tab[priv->queue[queue].txtail];

      /* Is this TX descriptor done transmitting?
       * First TX descriptor in chain has GEM_TX_DMA_USED = 1
       */

      if ((txdesc->status & GEM_TX_DMA_USED) == 0)
        {
          break;
        }

      /* Increment the tail index */

      if (++priv->queue[queue].txtail >= CONFIG_MPFS_ETHMAC_NTXBUFFERS)
        {
          /* Wrap to the beginning of the TX descriptor list */

          priv->queue[queue].txtail = 0;
        }

      /* At least one TX descriptor is available.  Re-enable RX interrupts.
       * RX interrupts may previously have been disabled when we ran out of
       * TX descriptors (see comments in mpfs_transmit()).
       */

      *priv->queue[queue].int_enable = INT_RX;
    }

  /* Then poll the network for new XMIT data */

  mpfs_dopoll(priv);
}

/****************************************************************************
 * Function: mpfs_recvframe
 *
 * Description:
 *   The function is called when a frame is received. It scans the RX
 *   descriptors of the received frame and assembles the full packet/
 *
 *   NOTE: This function will silently discard any packets containing errors.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *   qi    - Queue index number
 *
 * Returned Value:
 *   OK if a packet was successfully returned; -EAGAIN if there are no
 *   further packets available
 *
 * Assumptions:
 *   - Global interrupts are disabled by interrupt handling logic.
 *   - The RX descriptor D-cache list has been invalided to force fetching
 *     from RAM.
 *
 ****************************************************************************/

static int mpfs_recvframe(struct mpfs_ethmac_s *priv, unsigned int qi)
{
  volatile struct gmac_rxdesc_s *rxdesc;
  struct net_driver_s *dev;
  uintptr_t addr;
  uint8_t  *dest;
  uint32_t rxndx;
  uint32_t pktlen;
  uint16_t copylen;
  bool isframe;

  /* Process received RX descriptor.  The ownership bit is set by the GMAC
   * once it has successfully written a frame to memory.
   */

  dev        = &priv->dev;
  dev->d_len = 0;
  dest       = dev->d_buf;
  pktlen     = 0;
  rxndx      = priv->queue[qi].rxndx;
  rxdesc     = &priv->queue[qi].rx_desc_tab[rxndx];
  isframe    = false;

  ninfo("rxndx: %" PRId32 "\n", rxndx);

  while ((rxdesc->addr & GEM_RX_DMA_ADDR_OWNER) != 0)
    {
      /* The start of frame bit indicates the beginning of a frame.  Discard
       * any previous fragments.
       */

      if ((rxdesc->status & GEM_RX_DMA_STATUS_SOF) != 0)
        {
          /* Skip previous fragments */

          while (rxndx != priv->queue[qi].rxndx)
            {
              /* Give ownership back to the GMAC */

              rxdesc = &priv->queue[qi].
                        rx_desc_tab[priv->queue[qi].rxndx];
              rxdesc->addr &= ~GEM_RX_DMA_ADDR_OWNER;

              /* Increment the RX index */

              if (++priv->queue[qi].rxndx >= CONFIG_MPFS_ETHMAC_NRXBUFFERS)
                {
                  priv->queue[qi].rxndx = 0;
                }
            }

          /* Reset the packet data pointer and packet length */

          dest   = dev->d_buf;
          pktlen = 0;

          /* Start to gather buffers into the packet buffer */

          isframe = true;
        }

      /* Increment the working index */

      if (++rxndx >= CONFIG_MPFS_ETHMAC_NRXBUFFERS)
        {
          rxndx = 0;
        }

      /* Copy data into the packet buffer */

      if (isframe)
        {
          if (rxndx == priv->queue[qi].rxndx)
            {
              nerr("ERROR: No EOF (Invalid or buffers too small)\n");
              do
                {
                  /* Give ownership back to the GMAC */

                  rxdesc = &priv->queue[qi].
                            rx_desc_tab[priv->queue[qi].rxndx];
                  rxdesc->addr &= ~GEM_RX_DMA_ADDR_OWNER;

                  /* Increment the RX index */

                  if (++priv->queue[qi].rxndx
                      >= CONFIG_MPFS_ETHMAC_NRXBUFFERS)
                    {
                      priv->queue[qi].rxndx = 0;
                    }
                }
              while (rxndx != priv->queue[qi].rxndx);
              return -EIO;
            }

          /* Get the number of bytes to copy from the buffer */

          copylen = GMAC_RX_UNITSIZE;
          if ((pktlen + copylen) > CONFIG_NET_ETH_PKTSIZE)
            {
              copylen = CONFIG_NET_ETH_PKTSIZE - pktlen;
            }

          /* And do the copy */

          addr = (uintptr_t)(rxdesc->addr & GEM_RX_DMA_ADDR_MASK);
#if defined(CONFIG_MPFS_ETHMAC_64BIT_ADDRESS_MODE)
          addr += (uintptr_t)(rxdesc->addr_hi) << 32;
#endif
          memcpy(dest, (const void *)addr, copylen);
          dest   += copylen;
          pktlen += copylen;

          /* If the end of frame has been received, return the data */

          if ((rxdesc->status & GEM_RX_DMA_STATUS_EOF) != 0)
            {
              /* Frame size from the GMAC */

              dev->d_len = rxdesc->status & GEM_RX_DMA_STATUS_FRAME_LEN_MASK;
              ninfo("packet %d-%" PRId32 " (%d)\n",
                    priv->queue[qi].rxndx, rxndx, dev->d_len);

              /* All data have been copied in the application frame buffer,
               * release the RX descriptor
               */

              while (priv->queue[qi].rxndx != rxndx)
                {
                  /* Give ownership back to the GMAC */

                  rxdesc = &priv->queue[qi].
                            rx_desc_tab[priv->queue[qi].rxndx];
                  rxdesc->addr &= ~GEM_RX_DMA_ADDR_OWNER;

                  /* Increment the RX index */

                  if (++priv->queue[qi].rxndx
                      >= CONFIG_MPFS_ETHMAC_NRXBUFFERS)
                    {
                      priv->queue[qi].rxndx = 0;
                    }
                }

              /* Check if the device packet buffer was large enough to accept
               * all of the data.
               */

              ninfo("rxndx: %d d_len: %d\n",
                    priv->queue[qi].rxndx, dev->d_len);

              if (pktlen < dev->d_len)
                {
                  nerr("ERROR: Buffer size %d; frame size %" PRId32 "\n",
                       dev->d_len, pktlen);
                  return -E2BIG;
                }

              return OK;
            }
        }

      /* We have not encountered the SOF yet... discard this fragment
       * and keep looking
       */

      else
        {
          /* Give ownership back to the GMAC */

          rxdesc->addr &= ~GEM_RX_DMA_ADDR_OWNER;
          priv->queue[qi].rxndx = rxndx;
        }

      /* Process the next buffer */

      rxdesc = &priv->queue[qi].rx_desc_tab[rxndx];
    }

  /* isframe indicates that we have found a SOF. If we've received a SOF,
   * but not an EOF in the sequential buffers we own, it must mean that we
   * have a partial packet. This should only happen if there was a Buffer
   * Not Available (BNA) error.  When bursts of data come in, quickly
   * filling the available buffers, before our interrupts can even service
   * them. Eventually, the ring buffer loops back on itself and the
   * peripheral sees it cannot write the next fragment of the packet.
   *
   * In this case, we keep the rxndx at the start of the last frame, since
   * the peripheral will finish writing the packet there next.
   */

  if (!isframe)
    {
      priv->queue[qi].rxndx = rxndx;
    }

  ninfo("rxndx: %d\n", priv->queue[qi].rxndx);
  return -EAGAIN;
}

/****************************************************************************
 * Function: mpfs_receive
 *
 * Description:
 *   An interrupt was received indicating the availability of onr or more
 *   new RX packets in FIFO memory.
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

static void mpfs_receive(struct mpfs_ethmac_s *priv, unsigned int queue)
{
  struct net_driver_s *dev = &priv->dev;

  /* Loop while while mpfs_recvframe() successfully retrieves valid
   * GMAC frames.
   */

  while (mpfs_recvframe(priv, queue) == OK)
    {
      mpfs_dumppacket("Received packet", dev->d_buf, dev->d_len);

      /* Check if the packet is a valid size for the network buffer
       * configuration (this should not happen)
       */

      if (dev->d_len > CONFIG_NET_ETH_PKTSIZE)
        {
          nwarn("WARNING: Dropped, Too big: %d\n", dev->d_len);
          continue;
        }

  #ifdef CONFIG_NET_PKT
      /* When packet sockets are enabled, feed the frame into the packet
       * tap
       */

      pkt_input(&priv->dev);
  #endif

      /* We only accept IP packets of the configured type and ARP packets */

  #ifdef CONFIG_NET_IPv4
      if (BUF->type == HTONS(ETHTYPE_IP))
        {
          ninfo("IPv4 frame\n");

          /* Receive an IPv4 packet from the network device */

          ipv4_input(&priv->dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, the field d_len will set to a
           * value > 0.
           */

          if (priv->dev.d_len > 0)
            {
              /* And send the packet */

              mpfs_transmit(priv, queue);
            }
        }
      else
  #endif

  #ifdef CONFIG_NET_IPv6
      if (BUF->type == HTONS(ETHTYPE_IP6))
        {
          ninfo("IPv6 frame\n");

          /* Give the IPv6 packet to the network layer */

          ipv6_input(&priv->dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, the field  d_len will set to a
           * value > 0.
           */

          if (priv->dev.d_len > 0)
            {
              /* And send the packet */

              mpfs_transmit(priv, queue);
            }
        }
      else
  #endif

  #ifdef CONFIG_NET_ARP
      if (BUF->type == htons(ETHTYPE_ARP))
        {
          ninfo("ARP frame\n");

          /* Handle ARP packet */

          arp_arpin(&priv->dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, the field  d_len will set to a
           * value > 0.
           */

          if (priv->dev.d_len > 0)
            {
              mpfs_transmit(priv, queue);
            }
        }
      else
  #endif
        {
          nwarn("WARNING: Dropped, Unknown type: %04x\n", BUF->type);
        }
    }

    ninfo("receive done.\n");
}

/****************************************************************************
 * Function: mpfs_interrupt_work
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

static void mpfs_interrupt_work(void *arg)
{
  struct mpfs_ethmac_s *priv = (struct mpfs_ethmac_s *)arg;
  uint32_t isr;
  uint32_t rsr;
  uint32_t tsr;
  uint32_t regval;
  uint32_t queue = 0; /* todo: get from arg */

  /* Process pending Ethernet interrupts */

  net_lock();
  isr = *priv->queue[queue].int_status;
  rsr = mac_getreg(priv, RECEIVE_STATUS);
  tsr = mac_getreg(priv, TRANSMIT_STATUS);

  ninfo("isr: %08" PRIx32 "\n", isr);

  /* Check for the completion of a transmission.  This should be done before
   * checking for received data (because receiving can cause another
   * transmission before we had a chance to handle the last one).
   *
   * ISR:TCOMP is set when a frame has been transmitted. Cleared on read.
   * TSR:COMP is set when a frame has been transmitted. Cleared by writing a
   *   one to this bit.
   */

  if (tsr != 0)
    {
      ninfo("TX tsr=0x%X\n", tsr);
      uint32_t tx_error = 0;

      /* Check for Retry Limit Exceeded  */

      if ((tsr & TRANSMIT_STATUS_RETRY_LIMIT_EXCEEDED) != 0)
        {
          ++tx_error;
          nerr("ERROR: Retry Limit Exceeded TSR: %08" PRIx32 "\n", tsr);
        }

      /* Check Collision Occurred  */

      if ((tsr & TRANSMIT_STATUS_COLLISION_OCCURED) != 0)
        {
          nerr("ERROR: Collision occurred TSR: %08" PRIx32 "\n", tsr);
          ++tx_error;
        }

      /* Check for Transmit Frame Corruption due to AMBA error */

      if ((tsr & TRANSMIT_STATUS_AMBA_ERROR) != 0)
        {
          nerr("ERROR: AMBA error TSR: %08" PRIx32 "\n", tsr);
          ++tx_error;
        }

      /* Check for Transmit Underrun (UND)
       *
       * ISR:UND is set transmit DMA was not able to read data from memory,
       *   either because the bus was not granted in time, because a not
       *   OK hresp(bus error) was returned or because a used bit was read
       *   midway through frame transmission. If this occurs, the
       *   transmitter forces bad CRC. Cleared by writing a one to this bit.
       */

      if ((tsr & TRANSMIT_STATUS_UNDERRUN) != 0)
        {
          nerr("ERROR: Transmit Underrun TSR: %08" PRIx32 "\n", tsr);
          ++tx_error;
        }

      /* Check for HRESP not OK */

      if ((tsr & TRANSMIT_STATUS_RESP_NOT_OK) != 0)
        {
          nerr("ERROR: TX HRESP not OK: %08" PRIx32 "\n", tsr);
          ++tx_error;
        }

      /* Check for Late Collisions */

      if ((tsr & TRANSMIT_STATUS_LATE_COLLISION) != 0)
        {
          nerr("ERROR: Late collision: %08" PRIx32 "\n", tsr);
          ++tx_error;
        }

      /* Clear status */

      mac_putreg(priv, TRANSMIT_STATUS, tsr);

      /* Handle errors */

      if (tx_error != 0)
        {
          mpfs_txreset(priv);
          nerr("TX ERROR: reset\n");

          regval = mac_getreg(priv, NETWORK_CONTROL);
          regval |= NETWORK_CONTROL_ENABLE_TRANSMIT;
          mac_putreg(priv, NETWORK_CONTROL, regval);
        }

      /* And handle the TX done event */

      if ((tsr & TRANSMIT_STATUS_TRANSMIT_COMPLETE) != 0)
        {
          ninfo("TXCOMP: cancel timeout\n");
          wd_cancel(&priv->txtimeout);

          mpfs_txdone(priv, queue);
        }
    }

  /* Check for the receipt of an RX packet.
   *
   * RXCOMP indicates that a packet has been received and stored in memory.
   * RSR:REC indicates that one or more frames have been received and placed
   *   in memory. This indication is cleared by writing a one to this bit.
   */

  if (rsr != 0)
    {
      uint32_t rx_error = 0;
      ninfo("RX: rsr=0x%X\n", rsr);

      if ((rsr & RECEIVE_STATUS_FRAME_RECEIVED) != 0)
        {
          /* Handle the received packet */

          mpfs_receive(priv, queue);
        }

      /* Check for Receive Overrun */

      if ((rsr & RECEIVE_STATUS_RECEIVE_OVERRUN) != 0)
        {
          ++rx_error;
          nerr("ERROR: Receiver overrun RSR: %08" PRIx32 "\n", rsr);
        }

      /* Check for buffer not available (BNA) */

      if ((rsr & RECEIVE_STATUS_BUFFER_NOT_AVAILABLE) != 0)
        {
          ++rx_error;
          nerr("ERROR: Buffer not available RSR: %08" PRIx32 "\n", rsr);
        }

      /* Check for HRESP not OK */

      if ((rsr &  RECEIVE_STATUS_RESP_NOT_OK) != 0)
        {
          ++rx_error;
          nerr("ERROR: HRESP not OK: %08" PRIx32 "\n", rsr);
        }

      /* Clear status */

      mac_putreg(priv, RECEIVE_STATUS, rsr);

      if (rx_error != 0)
        {
          nerr("RX ERROR: reset\n");
          mpfs_rxreset(priv);
          *priv->queue[queue].int_status = 0xffffffff;
          mac_putreg(priv, RECEIVE_STATUS, 0xffffffff);

          /* rxreset disables reveiver so re-enable it */

          regval = mac_getreg(priv, NETWORK_CONTROL);
          regval |= NETWORK_CONTROL_ENABLE_RECEIVE;
          mac_putreg(priv, NETWORK_CONTROL, regval);
        }
    }

  net_unlock();

  /* Re-enable Ethernet interrupts */

  regval = INT_ALL;
  *priv->queue[queue].int_enable = regval;
}

/****************************************************************************
 * Function: mpfs_txreset
 *
 * Description:
 *  Reset the transmit logic
 *
 * Input Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *
 ****************************************************************************/

static void mpfs_txreset(struct mpfs_ethmac_s *priv)
{
  uint8_t *txbuffer;
  struct gmac_txdesc_s *txdesc;
  uintptr_t bufaddr;
  uint32_t regval;
  int qi;
  int ndx;

  /* Disable TX */

  regval  = mac_getreg(priv, NETWORK_CONTROL);
  regval &= ~NETWORK_CONTROL_ENABLE_TRANSMIT;
  mac_putreg(priv, NETWORK_CONTROL, regval);

  /* Configure the TX descriptors. */

  /* first set the common upper32-bits of 64 bit address */

  bufaddr = (uintptr_t)priv->queue[0].tx_desc_tab;
  mac_putreg(priv, UPPER_TX_Q_BASE_ADDR, upper_32_bits(bufaddr));

  for (qi = 0; qi < MPFS_MAC_QUEUE_COUNT; qi++)
    {
      txbuffer = priv->queue[qi].txbuffer;
      txdesc   = priv->queue[qi].tx_desc_tab;
      priv->queue[qi].txhead = 0;
      priv->queue[qi].txtail = 0;

      for (ndx = 0; ndx < CONFIG_MPFS_ETHMAC_NTXBUFFERS; ndx++)
        {
          bufaddr = (uintptr_t)&txbuffer[ndx * GMAC_TX_UNITSIZE];

          /* Set the buffer address and mark the descriptor as in used by
           * firmware.
           */

          txdesc[ndx].addr    = lower_32_bits(bufaddr);
          txdesc[ndx].status  = GEM_TX_DMA_USED;
#ifdef CONFIG_MPFS_ETHMAC_64BIT_ADDRESS_MODE
          txdesc[ndx].addr_hi = upper_32_bits(bufaddr);
#endif
        }

      /* Mark the final descriptor in the list */

      txdesc[CONFIG_MPFS_ETHMAC_NTXBUFFERS - 1].status = GEM_TX_DMA_USED |
                                                         GEM_TX_DMA_WRAP;

      /* Set the Transmit Buffer Queue Base Register and Disable queue */

      *priv->queue[qi].tx_q_ptr = lower_32_bits((uintptr_t)txdesc) | 1u;
    }

  /* REVISIT: for now enable only queue 0 for DMA operation */

  *priv->queue[0].tx_q_ptr = lower_32_bits((uintptr_t)
                                           priv->queue[0].tx_desc_tab);
}

/****************************************************************************
 * Function: mpfs_rxreset
 *
 * Description:
 *  Reset the receive logic
 *
 * Input Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *
 ****************************************************************************/

static void mpfs_rxreset(struct mpfs_ethmac_s *priv)
{
  uint8_t *rxbuffer;
  struct gmac_rxdesc_s *rxdesc;
  uintptr_t bufaddr;
  uint32_t regval;
  int qi;
  int ndx;

  /* Disable RX */

  regval  = mac_getreg(priv, NETWORK_CONTROL);
  regval &= ~NETWORK_CONTROL_ENABLE_RECEIVE;
  mac_putreg(priv, NETWORK_CONTROL, regval);

  /* first set the common upper32-bits of 64 bit address */

  bufaddr = (uintptr_t)priv->queue[0].rx_desc_tab;
  mac_putreg(priv, UPPER_RX_Q_BASE_ADDR, upper_32_bits(bufaddr));

  /* Configure the RX descriptors. */

  for (qi = 0; qi < MPFS_MAC_QUEUE_COUNT; qi++)
    {
      rxbuffer = priv->queue[qi].rxbuffer;
      rxdesc   = priv->queue[qi].rx_desc_tab;
      priv->queue[qi].rxndx = 0;

      for (ndx = 0; ndx < CONFIG_MPFS_ETHMAC_NRXBUFFERS; ndx++)
        {
          bufaddr = (uintptr_t)&rxbuffer[ndx * GMAC_RX_UNITSIZE];

          /* Set the buffer address and remove GMACRXD_ADDR_OWNER and
           * GMACRXD_ADDR_WRAP.
           */

          rxdesc[ndx].addr    = lower_32_bits(bufaddr);
          rxdesc[ndx].status  = 0;
#ifdef CONFIG_MPFS_ETHMAC_64BIT_ADDRESS_MODE
          rxdesc[ndx].addr_hi = upper_32_bits(bufaddr);
#endif
        }

      /* Mark the final descriptor in the list */

      rxdesc[CONFIG_MPFS_ETHMAC_NRXBUFFERS - 1].addr |= GEM_RX_DMA_ADDR_WRAP;

      /* Set the Receive Buffer Queue Base Register and disable queue */

      *priv->queue[qi].rx_q_ptr = lower_32_bits((uintptr_t)rxdesc) | 1u;
    }

  /* REVISIT: enable only queue 0 for DMA operation */

  *priv->queue[0].rx_q_ptr = lower_32_bits((uintptr_t)
                                           priv->queue[0].rx_desc_tab);
  *priv->queue[0].int_status = 0xffffffff;
}

/****************************************************************************
 * Function: mpfs_txinuse
 *
 * Description:
 *   Return the number of TX buffers in-use
 *
 * Input Parameters:
 *   priv - The GMAC driver state
 *
 * Returned Value:
 *   The number of TX buffers in-use
 *
 ****************************************************************************/

static uint16_t mpfs_txinuse(struct mpfs_ethmac_s *priv, unsigned int queue)
{
  uint32_t txhead32 = priv->queue[queue].txhead;
  if (priv->queue[queue].txtail > txhead32)
    {
      txhead32 += CONFIG_MPFS_ETHMAC_NTXBUFFERS;
    }

  return (uint16_t)(txhead32 - priv->queue[queue].txtail);
}

/****************************************************************************
 * Function: mpfs_txfree
 *
 * Description:
 *   Return the number of TX buffers available
 *
 * Input Parameters:
 *   priv - The GMAC driver state
 *
 * Returned Value:
 *   The number of TX buffers available
 *
 ****************************************************************************/

static uint16_t mpfs_txfree(struct mpfs_ethmac_s *priv, unsigned int queue)
{
  /* The number available is equal to the total number of buffers, minus the
   * number of buffers in use.  Notice that that actual number of buffers is
   * the configured size minus 1.
   */

  return (CONFIG_MPFS_ETHMAC_NTXBUFFERS - 1) - mpfs_txinuse(priv, queue);
}

/****************************************************************************
 * Function: mpfs_macaddress
 *
 * Description:
 *   Configure the selected MAC address.
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

static void mpfs_macaddress(struct mpfs_ethmac_s *priv)
{
  struct net_driver_s *dev = &priv->dev;
  uint32_t regval;

  ninfo("%s MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
        dev->d_ifname,
        dev->d_mac.ether.ether_addr_octet[0],
        dev->d_mac.ether.ether_addr_octet[1],
        dev->d_mac.ether.ether_addr_octet[2],
        dev->d_mac.ether.ether_addr_octet[3],
        dev->d_mac.ether.ether_addr_octet[4],
        dev->d_mac.ether.ether_addr_octet[5]);

  /* Set the MAC address */

  regval = (uint32_t)dev->d_mac.ether.ether_addr_octet[0] |
           (uint32_t)dev->d_mac.ether.ether_addr_octet[1] << 8 |
           (uint32_t)dev->d_mac.ether.ether_addr_octet[2] << 16 |
           (uint32_t)dev->d_mac.ether.ether_addr_octet[3] << 24;
  mac_putreg(priv, SPEC_ADD1_BOTTOM, regval);

  regval = (uint32_t)dev->d_mac.ether.ether_addr_octet[4] |
           (uint32_t)dev->d_mac.ether.ether_addr_octet[5] << 8;
  mac_putreg(priv, SPEC_ADD1_TOP, regval);
}

/****************************************************************************
 * Function: mpfa_txpoll
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

static int mpfs_txpoll(struct net_driver_s *dev)
{
  struct mpfs_ethmac_s *priv = (struct mpfs_ethmac_s *)dev->d_private;

  /* Send the packet */

  mpfs_transmit(priv, 0);

  /* Check if there are any free TX descriptors.  We cannot perform
   * the TX poll if we do not have buffering for another packet.
   */

  if (mpfs_txfree(priv, 0) == 0)
    {
      /* We have to terminate the poll if we have no more descriptors
       * available for another transfer.
       */

      return -EBUSY;
    }

  /* If zero is returned, the polling will continue until all connections
   * have been examined.
   */

  return 0;
}

/****************************************************************************
 * Function: mpfs_dopoll
 *
 * Description:
 *   The function is called in order to perform an out-of-sequence TX poll.
 *   This is done:
 *
 *   1. After completion of a transmission (mpfs_txdone),
 *   2. When new TX data is available (mpfs_txavail), and
 *   3. After a TX timeout to restart the sending process
 *      (mpfs_txtimeout_expiry).
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

static void mpfs_dopoll(struct mpfs_ethmac_s *priv)
{
  struct net_driver_s *dev = &priv->dev;

  /* Check if there are any free TX descriptors.  We cannot perform the
   * TX poll if we do not have buffering for another packet.
   */

  if (mpfs_txfree(priv, 0) > 0)
    {
      /* If we have the descriptor,
       * then poll the network for new XMIT data.
       */

      devif_poll(dev, mpfs_txpoll);
    }
}

/****************************************************************************
 * Function: mpfs_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the Ethernet interface when an IP address is
 *   provided
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int mpfs_ifup(struct net_driver_s *dev)
{
  struct mpfs_ethmac_s *priv = (struct mpfs_ethmac_s *)dev->d_private;
  int ret;

#ifdef CONFIG_NET_IPv4
  ninfo("Bringing up: %d.%d.%d.%d\n",
        (int)(dev->d_ipaddr & 0xff), (int)((dev->d_ipaddr >> 8) & 0xff),
        (int)((dev->d_ipaddr >> 16) & 0xff), (int)(dev->d_ipaddr >> 24));
#endif
#ifdef CONFIG_NET_IPv6
  ninfo("Bringing up: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        dev->d_ipv6addr[0], dev->d_ipv6addr[1], dev->d_ipv6addr[2],
        dev->d_ipv6addr[3], dev->d_ipv6addr[4], dev->d_ipv6addr[5],
        dev->d_ipv6addr[6], dev->d_ipv6addr[7]);
#endif

  /* Configure the Ethernet interface for DMA operation. */

  ret = mpfs_ethconfig(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Set the MAC address (should have been configured while we were down) */

  mpfs_macaddress(priv);

#ifdef CONFIG_NET_ICMPv6
  /* Set up IPv6 multicast address filtering */

  mpfs_ipv6multicast(priv);
#endif

  /* Initialize for PHY access */

  ret = mpfs_phyinit(priv);
  if (ret < 0)
    {
      nerr("ERROR: mpfs_phyinit failed: %d\n", ret);
      return ret;
    }

#ifdef CONFIG_MPFS_MAC_AUTONEG

  ret = mpfs_autonegotiate(priv);
  if (ret < 0)
    {
      nerr("ERROR: mpfs_autonegotiate failed: %d\n", ret);
      return ret;
    }
#else
  /* Just force the configured link speed */

  mpfs_linkspeed(priv);
#endif

  /* Enable normal MAC operation */

  ninfo("Enable normal operation\n");

  /* Enable the Ethernet interrupts */

  priv->ifup = true;
  up_enable_irq(priv->mac_q_int[0]);
  up_enable_irq(priv->mac_q_int[1]);
  up_enable_irq(priv->mac_q_int[2]);
  up_enable_irq(priv->mac_q_int[3]);

  return OK;
}

/****************************************************************************
 * Function: mpfs_ifdown
 *
 * Description:
 *   NuttX Callback: Stop the interface.
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int mpfs_ifdown(struct net_driver_s *dev)
{
  struct mpfs_ethmac_s *priv = (struct mpfs_ethmac_s *)dev->d_private;
  irqstate_t flags;

  ninfo("Taking the network down\n");

  /* Disable the Ethernet interrupt */

  flags = enter_critical_section();
  up_disable_irq(priv->mac_q_int[0]);
  up_disable_irq(priv->mac_q_int[1]);
  up_disable_irq(priv->mac_q_int[2]);
  up_disable_irq(priv->mac_q_int[3]);

  *priv->queue[0].int_disable = 0xffffffff;
  *priv->queue[1].int_disable = 0xffffffff;
  *priv->queue[2].int_disable = 0xffffffff;
  *priv->queue[3].int_disable = 0xffffffff;

  /* Cancel the TX timeout timers */

  wd_cancel(&priv->txtimeout);

  /* Put the MAC in its reset, non-operational state.  This should be
   * a known configuration that will guarantee the mpfs_ifup() always
   * successfully brings the interface back up.
   */

  mpfs_ethreset(priv);

  /* Mark the device "down" */

  priv->ifup = false;
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Function: mpfs_txavail_work
 *
 * Description:
 *   Perform an out-of-cycle poll on the worker thread.
 *
 * Parameters:
 *   arg  - Reference to the NuttX driver state structure (cast to void*)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called on the higher priority worker thread.
 *
 ****************************************************************************/

static void mpfs_txavail_work(void *arg)
{
  struct mpfs_ethmac_s *priv = (struct mpfs_ethmac_s *)arg;

  ninfo("ifup: %d\n", priv->ifup);

  /* Ignore the notification if the interface is not yet up */

  net_lock();
  if (priv->ifup)
    {
      /* Poll the network for new XMIT data */

      mpfs_dopoll(priv);
    }

  net_unlock();
}

/****************************************************************************
 * Function: mpfs_txavail
 *
 * Description:
 *   Driver callback invoked when new TX data is available.  This is a
 *   stimulus perform an out-of-cycle poll and, thereby, reduce the TX
 *   latency.
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called in normal user mode
 *
 ****************************************************************************/

static int mpfs_txavail(struct net_driver_s *dev)
{
  struct mpfs_ethmac_s *priv = (struct mpfs_ethmac_s *)dev->d_private;

  /* Is our single work structure available?  It may not be if there are
   * pending interrupt actions and we will have to ignore the Tx
   * availability action.
   */

  if (work_available(&priv->pollwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      work_queue(ETHWORK, &priv->pollwork, mpfs_txavail_work, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: mpfs_hashindx
 *
 * Description:
 *   Cacuclate the hash address register index.  The hash address register
 *   is 64 bits long and takes up two locations in the memory map. The
 *   destination address is reduced to a 6-bit index into the 64-bit Hash
 *   Register using the following hash function: The hash function is an XOR
 *   of every sixth bit of the destination address.
 *
 *   ndx:05 = da:05 ^ da:11 ^ da:17 ^ da:23 ^ da:29 ^ da:35 ^ da:41 ^ da:47
 *   ndx:04 = da:04 ^ da:10 ^ da:16 ^ da:22 ^ da:28 ^ da:34 ^ da:40 ^ da:46
 *   ndx:03 = da:03 ^ da:09 ^ da:15 ^ da:21 ^ da:27 ^ da:33 ^ da:39 ^ da:45
 *   ndx:02 = da:02 ^ da:08 ^ da:14 ^ da:20 ^ da:26 ^ da:32 ^ da:38 ^ da:44
 *   ndx:01 = da:01 ^ da:07 ^ da:13 ^ da:19 ^ da:25 ^ da:31 ^ da:37 ^ da:43
 *   ndx:00 = da:00 ^ da:06 ^ da:12 ^ da:18 ^ da:24 ^ da:30 ^ da:36 ^ da:42
 *
 *   Where da:00 represents the least significant bit of the first byte
 *   received and da:47 represents the most significant bit of the last byte
 *   received.
 *
 * Input Parameters:
 *   mac - The multicast address to be hashed
 *
 * Returned Value:
 *   The 6-bit hash table index
 *
 ****************************************************************************/

#if defined(CONFIG_NET_MCASTGROUP) || defined(CONFIG_NET_ICMPv6)
static unsigned int mpfs_hashindx(const uint8_t *mac)
{
  unsigned int ndx;

  ndx = mac[0];
  ndx ^= (mac[1] << 2) | (mac[0] >> 6);
  ndx ^= (mac[2] << 4) | (mac[1] >> 4);
  ndx ^= (mac[2] >> 2);
  ndx ^= mac[3];
  ndx ^= (mac[4] << 2) | (mac[3] >> 6);
  ndx ^= (mac[5] << 4) | (mac[4] >> 4);
  ndx ^= (mac[5] >> 2);

  return ndx & 0x3f;
}
#endif /* CONFIG_NET_MCASTGROUP || CONFIG_NET_ICMPv6 */

/****************************************************************************
 * Function: mpfs_addmac
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
static int mpfs_addmac(struct net_driver_s *dev, const uint8_t *mac)
{
  struct mpfs_ethmac_s *priv = (struct mpfs_ethmac_s *)dev->d_private;
  uint32_t regoffset;
  uint32_t regval;
  unsigned int ndx;
  UNUSED(priv);

  ninfo("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  /* Calculate the 6-bit has table index */

  ndx = mpfs_hashindx(mac);

  /* Add the multicast address to the hardware multicast hash table */

  regoffset = (ndx >= 32) ? HASH_TOP : HASH_BOTTOM;
  regval    = mac_getreg(priv, regoffset);
  regval   |= 1 << (ndx % 32);
  mac_putreg(priv, regoffset, regval);

  /* The unicast hash enable and the multicast hash enable bits in the
   * Network Configuration Register enable the reception of hash matched
   * frames:
   *
   * - A multicast match will be signalled if the multicast hash enable bit
   *   is set, da:00 is logic 1 and the hash index points to a bit set in
   *   the Hash Register.
   * - A unicast match will be signalled if the unicast hash enable bit is
   *   set, da:00 is logic 0 and the hash index points to a bit set in the
   *   Hash Register.
   */

  regval  = mac_getreg(priv, NETWORK_CONFIG);
  regval &= ~NETWORK_CONFIG_UNICAST_HASH_ENABLE;
  regval |= NETWORK_CONFIG_MULTICAST_HASH_ENABLE;
  mac_putreg(priv, NETWORK_CONFIG, regval);

  return OK;
}
#endif

/****************************************************************************
 * Function: mpfs_rmmac
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
static int mpfs_rmmac(struct net_driver_s *dev, const uint8_t *mac)
{
  struct mpfs_ethmac_s *priv = (struct mpfs_ethmac_s *)dev->d_private;
  uint32_t regval;
  uint32_t regaddr1;
  uint32_t regaddr2;
  unsigned int ndx;
  UNUSED(priv);

  ninfo("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  /* Calculate the 6-bit has table index */

  ndx = mpfs_hashindx(mac);

  /* Remove the multicast address to the hardware multicast hast table */

  if (ndx >= 32)
    {
      regaddr1 = HASH_TOP;        /* Hash Register Top [63:32] Register */
      regaddr2 = HASH_BOTTOM;     /* Hash Register Bottom [31:0] Register */
    }
  else
    {
      regaddr1 = HASH_BOTTOM;     /* Hash Register Bottom [31:0] Register */
      regaddr2 = HASH_TOP;        /* Hash Register Top [63:32] Register */
    }

  regval  = mac_getreg(priv, regaddr1);
  regval &= ~(1 << (ndx % 32));
  mac_putreg(priv, regaddr1, regval);

  /* The unicast hash enable and the multicast hash enable bits in the
   * Network Configuration Register enable the reception of hash matched
   * frames:
   *
   * - A multicast match will be signalled if the multicast hash enable bit
   *   is set, da:00 is logic 1 and the hash index points to a bit set in
   *   the Hash Register.
   * - A unicast match will be signalled if the unicast hash enable bit is
   *   set, da:00 is logic 0 and the hash index points to a bit set in the
   *   Hash Register.
   */

  /* Are all multicast address matches disabled? */

  if (regval == 0 && mac_getreg(priv, regaddr2) == 0)
    {
      /* Yes.. disable all address matching */

      regval  = mac_getreg(priv, NETWORK_CONFIG);
      regval &= ~(NETWORK_CONFIG_UNICAST_HASH_ENABLE |
                  NETWORK_CONFIG_MULTICAST_HASH_ENABLE);
      mac_putreg(priv, NETWORK_CONFIG, regval);
    }

  return OK;
}
#endif

/****************************************************************************
 * Function: mpfs_enablemdio
 *
 * Description:
 *  Enable the management port
 *
 * Input Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_enablemdio(struct mpfs_ethmac_s *priv)
{
  uint32_t regval;
  uint32_t enables;

  /* Enable management port */

  regval  = mac_getreg(priv, NETWORK_CONTROL);
  enables = regval & (NETWORK_CONTROL_ENABLE_RECEIVE |
                      NETWORK_CONTROL_ENABLE_TRANSMIT);

  regval &= ~(NETWORK_CONTROL_ENABLE_RECEIVE |
              NETWORK_CONTROL_ENABLE_TRANSMIT);
  regval |= NETWORK_CONTROL_MAN_PORT_EN;
  mac_putreg(priv, NETWORK_CONTROL, regval);

  regval |= enables;
  mac_putreg(priv, NETWORK_CONTROL, regval);
}

/****************************************************************************
 * Function: mpfs_disablemdio
 *
 * Description:
 *  Disable the management port
 *
 * Input Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_disablemdio(struct mpfs_ethmac_s *priv)
{
  uint32_t regval;
  uint32_t enables;

  /* Disable management port */

  regval  = mac_getreg(priv, NETWORK_CONTROL);
  enables = regval & (NETWORK_CONTROL_ENABLE_RECEIVE |
                      NETWORK_CONTROL_ENABLE_TRANSMIT);

  regval &= ~(NETWORK_CONTROL_ENABLE_RECEIVE |
              NETWORK_CONTROL_ENABLE_TRANSMIT);
  mac_putreg(priv, NETWORK_CONTROL, regval);

  regval &= ~NETWORK_CONTROL_MAN_PORT_EN;
  mac_putreg(priv, NETWORK_CONTROL, regval);

  regval |= enables;
  mac_putreg(priv, NETWORK_CONTROL, regval);
}

/****************************************************************************
 * Function: mpfs_phyread
 *
 * Description:
 *  Read a PHY register.
 *
 * Input Parameters:
 *   priv - A reference to the private driver state structure
 *   phyaddr - The PHY device address
 *   regaddr - The PHY register address
 *   phyval - The location to return the 16-bit PHY register value.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int mpfs_phyread(struct mpfs_ethmac_s *priv, uint8_t phyaddr,
                        uint8_t regaddr, uint16_t *phyval)
{
  uint32_t regval;
  int ret;

  /* Make sure that the PHY is idle */

  ret = mpfs_phywait(priv);
  if (ret < 0)
    {
      nerr("ERROR: mpfs_phywait failed: %d\n", ret);
      return ret;
    }

  /* Write the PHY Maintenance register */

  regval = PHY_MANAGEMENT_PHY_DATA(0) | PHY_MANAGEMENT_WRITE10 |
           PHY_MANAGEMENT_REG_ADDRESS(regaddr) |
           PHY_MANAGEMENT_PHY_ADDRESS(phyaddr) |
           PHY_MANAGEMENT_OPERATION_READ |
           PHY_MANAGEMENT_WRITE_1;

  mac_putreg(priv, PHY_MANAGEMENT, regval);

  /* Wait until the PHY is again idle */

  ret = mpfs_phywait(priv);
  if (ret < 0)
    {
      nerr("ERROR: mpfs_phywait failed: %d\n", ret);
      return ret;
    }

  /* Return the PHY data */

  *phyval = (uint16_t)(mac_getreg(priv, PHY_MANAGEMENT) &
            PHY_MANAGEMENT_PHY_DATA_MASK);
  return OK;
}

/****************************************************************************
 * Function: mpfs_phywrite
 *
 * Description:
 *  Write to a PHY register.
 *
 * Input Parameters:
 *   priv - A reference to the private driver state structure
 *   phyaddr - The PHY device address
 *   regaddr - The PHY register address
 *   phyval - The 16-bit value to write to the PHY register.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int mpfs_phywrite(struct mpfs_ethmac_s *priv, uint8_t phyaddr,
                         uint8_t regaddr, uint16_t phyval)
{
  uint32_t regval;
  int ret;

  /* Make sure that the PHY is idle */

  ret = mpfs_phywait(priv);
  if (ret < 0)
    {
      nerr("ERROR: mpfs_phywait failed: %d\n", ret);
      return ret;
    }

  /* Write the PHY Maintenance register */

  regval = PHY_MANAGEMENT_PHY_DATA(phyval) | PHY_MANAGEMENT_WRITE10 |
           PHY_MANAGEMENT_REG_ADDRESS(regaddr) |
           PHY_MANAGEMENT_PHY_ADDRESS(phyaddr) |
           PHY_MANAGEMENT_OPERATION_WRITE |
           PHY_MANAGEMENT_WRITE_1;
  mac_putreg(priv,  PHY_MANAGEMENT, regval);

  /* Wait until the PHY is again IDLE */

  ret = mpfs_phywait(priv);
  if (ret < 0)
    {
      nerr("ERROR: mpfs_phywait failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Function: mpfs_phywait
 *
 * Description:
 *  Wait for the PHY to become IDLE
 *
 * Input Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno (-ETIMEDOUT) on failure.
 *
 ****************************************************************************/

static int mpfs_phywait(struct mpfs_ethmac_s *priv)
{
  unsigned int retries;

  /* Loop for the configured number of attempts */

  for (retries = 0; retries < PHY_RETRY_MAX; retries++)
    {
      /* Is the PHY IDLE */

      if ((mac_getreg(priv, NETWORK_STATUS) & NETWORK_STATUS_MAN_DONE) != 0)
        {
          return OK;
        }
    }

  return -ETIMEDOUT;
}

/****************************************************************************
 * Function: mpfs_phyfind
 *
 * Description:
 *  Verify the PHY address and, if it is bad, try to one that works.
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

static int mpfs_phyfind(struct mpfs_ethmac_s *priv, uint8_t *phyaddr)
{
  uint16_t phyval;
  uint8_t candidate;
  unsigned int offset;
  int ret = -ESRCH;

  ninfo("Find a valid PHY address\n");

  /* Enable management port */

  mpfs_enablemdio(priv);

#ifdef CONFIG_MPFS_HAVE_CORERMII
  ninfo("coreRMII: reset @address: %d\n", CONFIG_MPFS_CORERMII_ADDRESS);

  ret = mpfs_phywrite(priv, CONFIG_MPFS_CORERMII_ADDRESS, GMII_MCR,
                      CORE_RMII_RESET | CORE_RMII_FULL_DUPLEX |
                      CORE_RMII_100MBIT);
  if (ret != OK)
    {
      nerr("Core RMII reset write failed!\n");
    }

  ret = mpfs_phyread(priv, CONFIG_MPFS_CORERMII_ADDRESS, GMII_MCR, &phyval);
  ninfo("CORE-RMII after reset MCR=%d\n", phyval);
  if ((phyval != 0x03) || (ret != OK))
    {
      nerr("Core RMII reset read failed! val=%d\n", phyval);
    }
#endif

  /* Check initial candidate address */

  candidate = *phyaddr;

  ret = mpfs_phyread(priv, candidate, GMII_PHYID1, &phyval);
  if (ret == OK && phyval != 0xffff)
    {
      *phyaddr = candidate;
      ret = OK;
    }

  /* The current address does not work... try another */

  else
    {
      nerr("ERROR: mpfs_phyread failed for PHY address %02x: %d\n",
           candidate, ret);

      for (offset = 0; offset < 32; offset++)
        {
          /* Get the next candidate PHY address */

          candidate = (candidate + 1) & 0x1f;

          /* Try reading the PHY ID from the candidate PHY address */

          ret = mpfs_phyread(priv, candidate, GMII_PHYID1, &phyval);
          if (ret == OK && phyval != 0xffff)
            {
              ret = OK;
              break;
            }
        }
    }

  if (ret == OK)
    {
      ninfo("  PHYID1: %04x PHY addr: %d\n", phyval, candidate);
      *phyaddr = candidate;
    }

  /* Disable management port */

  mpfs_disablemdio(priv);
  return ret;
}

/****************************************************************************
 * Function: mpfs_phydump
 *
 * Description:
 *   Dump the contents of PHY registers
 *
 * Input Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_DEBUG_NET) && defined(CONFIG_DEBUG_INFO)
static void mpfs_phydump(struct mpfs_ethmac_s *priv)
{
  uint16_t phyval;

  /* Enable management port */

  mpfs_enablemdio(priv);

  ninfo("GMII Registers (Address %02x)\n", priv->phyaddr);
  mpfs_phyread(priv, priv->phyaddr, GMII_MCR, &phyval);
  ninfo("       MCR: %04x\n", phyval);
  mpfs_phyread(priv, priv->phyaddr, GMII_MSR, &phyval);
  ninfo("       MSR: %04x\n", phyval);
  mpfs_phyread(priv, priv->phyaddr, GMII_ADVERTISE, &phyval);
  ninfo(" ADVERTISE: %04x\n", phyval);
  mpfs_phyread(priv, priv->phyaddr, GMII_LPA, &phyval);
  ninfo("       LPR: %04x\n", phyval);
  mpfs_phyread(priv, priv->phyaddr, GMII_1000BTCR, &phyval);
  ninfo("  1000BTCR: %04x\n", phyval);
  mpfs_phyread(priv, priv->phyaddr, GMII_1000BTSR, &phyval);
  ninfo("  1000BTSR: %04x\n", phyval);
  mpfs_phyread(priv, priv->phyaddr, GMII_ESTATUS, &phyval);
  ninfo("   ESTATUS: %04x\n", phyval);

  /* Disable management port */

  mpfs_disablemdio(priv);
}
#endif

/****************************************************************************
 * Function: mpfs_autonegotiate
 *
 * Description:
 *  Autonegotiate speed and duplex.
 *
 * Input Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_MPFS_MAC_AUTONEG
static int mpfs_autonegotiate(struct mpfs_ethmac_s *priv)
{
  uint32_t regval;
  uint32_t control;
  uint32_t linkmode;
  uint16_t phyval;
  uint16_t phyid1;
  uint16_t phyid2;
  uint16_t advertise;
  uint16_t lpa;
  int timeout;
  int ret;

#ifndef CONFIG_MPFS_MAC_AUTONEG_DISABLE_1000MBPS
  uint16_t btsr;
  uint16_t btcr;
#endif

  /* Enable management port */

  mpfs_enablemdio(priv);

  /* Read the MSB bits of the OUI from the PHYID1 register */

  ret = mpfs_phyread(priv, priv->phyaddr, GMII_PHYID1, &phyid1);
  if (ret < 0)
    {
      nerr("ERROR: Failed to read PHYID1 register\n");
      goto errout;
    }

  ninfo("PHYID1: %04x PHY address: %02x\n", phyid1, priv->phyaddr);

  /* Read the LS bits of the OUI from the PHYID2 register */

  ret = mpfs_phyread(priv, priv->phyaddr, GMII_PHYID2, &phyid2);
  if (ret < 0)
    {
      nerr("ERROR: Failed to read PHYID2 register\n");
      goto errout;
    }

  ninfo("PHYID2: %04x PHY address: %02x\n", phyid2, priv->phyaddr);

  if ((phyid1 != 0xffff) && (phyid2 != 0xffff))
    {
      ninfo("  Vendor Model Number:   %04x\n",
            (phyid2 & GMII_PHYID2_MODEL_MASK) >> GMII_PHYID2_MODEL_SHIFT);
      ninfo("  Model Revision Number: %04x\n",
            (phyid2 & GMII_PHYID2_REV_MASK) >> GMII_PHYID2_REV_SHIFT);
    }

  /* Set the Auto_negotiation Advertisement Register, MII advertising for
   * Next page 100BaseTxFD and HD, 10BaseTxFD and HD, IEEE 802.3
   */

  advertise = GMII_ADVERTISE_100BASETXFULL | GMII_ADVERTISE_100BASETXHALF |
              GMII_ADVERTISE_10BASETXFULL | GMII_ADVERTISE_10BASETXHALF |
              GMII_ADVERTISE_8023;

  ret = mpfs_phywrite(priv, priv->phyaddr, GMII_ADVERTISE, advertise);
  if (ret < 0)
    {
      nerr("ERROR: Failed to write ADVERTISE register\n");
      goto errout;
    }

#ifndef CONFIG_MPFS_MAC_AUTONEG_DISABLE_1000MBPS
  /* Modify the 1000Base-T control register to advertise 1000Base-T full
   * and half duplex support.
   */

  ret = mpfs_phyread(priv, priv->phyaddr, GMII_1000BTCR, &btcr);
  if (ret < 0)
    {
      nerr("ERROR: Failed to read 1000BTCR register: %d\n", ret);
      goto errout;
    }

  btcr |= GMII_1000BTCR_1000BASETFULL | GMII_1000BTCR_1000BASETHALF;

  ret = mpfs_phywrite(priv, priv->phyaddr, GMII_1000BTCR, btcr);
  if (ret < 0)
    {
      nerr("ERROR: Failed to write 1000BTCR register: %d\n", ret);
      goto errout;
    }

#endif

  /* Restart Auto_negotiation */

  ret = mpfs_phyread(priv, priv->phyaddr, GMII_MCR, &phyval);
  if (ret < 0)
    {
      nerr("ERROR: Failed to read MCR register: %d\n", ret);
      goto errout;
    }

  phyval |= GMII_MCR_ANRESTART;

  ret = mpfs_phywrite(priv, priv->phyaddr, GMII_MCR, phyval);
  if (ret < 0)
    {
      nerr("ERROR: Failed to write MCR register: %d\n", ret);
      goto errout;
    }

  ret = mpfs_phyread(priv, priv->phyaddr, GMII_MCR, &phyval);
  if (ret < 0)
    {
      nerr("ERROR: Failed to read MCR register: %d\n", ret);
      goto errout;
    }

  ninfo(" MCR: 0x%X\n", phyval);

  /* Wait for autonegotiation to complete */

  timeout = 0;
  for (; ; )
    {
      ret = mpfs_phyread(priv, priv->phyaddr, GMII_MSR, &phyval);
      if (ret < 0)
        {
          nerr("ERROR: Failed to read MSR register: %d\n", ret);
          goto errout;
        }

      /* Check for completion of autonegotiation */

      if ((phyval & GMII_MSR_ANEGCOMPLETE) != 0)
        {
          /* Yes break out of the loop */

          ninfo("AutoNegotiate complete\n");
          break;
        }

      /* No check for a timeout */

      if (++timeout >= PHY_RETRY_MAX)
        {
          nerr("ERROR: TimeOut\n");
          mpfs_phydump(priv);
          ret = -ETIMEDOUT;
          goto errout;
        }
    }

  /* Setup the GMAC local link speed */

  linkmode = 0;  /* 10Base-T Half-Duplex */
  timeout  = 0;

  for (; ; )
    {
  #ifndef CONFIG_MPFS_MAC_AUTONEG_DISABLE_1000MBPS
      ret = mpfs_phyread(priv, priv->phyaddr, GMII_1000BTSR, &btsr);
      if (ret < 0)
        {
          nerr("ERROR: Failed to read 1000BTSR register: %d\n", ret);
          goto errout;
        }

      /* Setup the GMAC link speed */

      if ((btsr & GMII_1000BTSR_LP1000BASETFULL) != 0 &&
          (btcr & GMII_1000BTCR_1000BASETHALF) != 0)
        {
          /* Set RGMII for 1000BaseTX and Full Duplex */

          ninfo("Link: FD - 1000\n");
          linkmode = NETWORK_CONFIG_FULL_DUPLEX |
                     NETWORK_CONFIG_GIGABIT_MODE_ENABLE;
          break;
        }
      else if ((btsr & GMII_1000BTSR_LP1000BASETHALF) != 0 &&
               (btcr & GMII_1000BTCR_1000BASETFULL) != 0)
        {
          /* Set RGMII for 1000BaseT and Half Duplex */

          ninfo("Link: HD - 1000\n");
          linkmode = NETWORK_CONFIG_GIGABIT_MODE_ENABLE;
          break;
        }
  #endif

      /* Get the Autonegotiation Link partner base page */

      ret  = mpfs_phyread(priv, priv->phyaddr, GMII_LPA, &lpa);
      if (ret < 0)
        {
          nerr("ERROR: Failed to read LPA register: %d\n", ret);
          goto errout;
        }

      /* Setup the GMAC link speed */

      if ((advertise & GMII_ADVERTISE_100BASETXFULL) != 0 &&
          (lpa & GMII_LPA_100BASETXFULL) != 0)
        {
          /* Set RGMII for 100BaseTX and Full Duplex */

          ninfo("Link: FD - 100\n");
          linkmode = (NETWORK_CONFIG_SPEED | NETWORK_CONFIG_FULL_DUPLEX);
          break;
        }
      else if ((advertise & GMII_ADVERTISE_10BASETXFULL) != 0 &&
               (lpa & GMII_LPA_10BASETXFULL) != 0)
        {
          /* Set RGMII for 10BaseT and Full Duplex */

          ninfo("Link: FD - 10\n");
          linkmode = NETWORK_CONFIG_FULL_DUPLEX;
          break;
        }
      else if ((advertise & GMII_ADVERTISE_100BASETXHALF) != 0 &&
               (lpa & GMII_LPA_100BASETXHALF) != 0)
        {
          /* Set RGMII for 100BaseTX and half Duplex */

          ninfo("Link: HD - 100\n");
          linkmode = NETWORK_CONFIG_SPEED;
          break;
        }
      else if ((advertise & GMII_ADVERTISE_10BASETXHALF) != 0 &&
               (lpa & GMII_LPA_10BASETXHALF) != 0)
        {
          /* Set RGMII for 10BaseT and half Duplex */

          ninfo("Link: HD - 10\n");
          break;
        }

      /* Check for a timeout */

      if (++timeout >= PHY_RETRY_MAX)
        {
          nerr("ERROR: TimeOut\n");
          mpfs_phydump(priv);
          ret = -ETIMEDOUT;
          goto errout;
        }
    }

  /* Disable RX and TX momentarily */

  control = mac_getreg(priv, NETWORK_CONTROL);
  mac_putreg(priv, NETWORK_CONTROL,
             control & ~(NETWORK_CONTROL_ENABLE_RECEIVE |
                         NETWORK_CONTROL_ENABLE_TRANSMIT));

  /* Modify the NETWORK_CONFIG register based on the negotiated
   * speed and duplex
   */

  regval  = mac_getreg(priv, NETWORK_CONFIG);
  regval &= ~(NETWORK_CONFIG_SPEED | NETWORK_CONFIG_FULL_DUPLEX |
              NETWORK_CONFIG_GIGABIT_MODE_ENABLE);
  regval |= linkmode;
  mac_putreg(priv, NETWORK_CONFIG, regval);
  mac_putreg(priv, NETWORK_CONTROL, control);

errout:

  /* Disable the management port */

  mpfs_disablemdio(priv);
  return ret;
}
#endif

/****************************************************************************
 * Function: mpfs_linkspeed
 *
 * Description:
 *  If autonegotiation is not configured, then just force the configuration
 *  mode
 *
 * Input Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_MPFS_MAC_AUTONEG
static void mpfs_linkspeed(struct mpfs_ethmac_s *priv)
{
  uint32_t regval;
  uint32_t ncr;

  /* Disable RX and TX momentarily */

  ncr = mac_getreg(priv, NETWORK_CONTROL);
  mac_putreg(priv, NETWORK_CONTROL,
             ncr & ~(NETWORK_CONTROL_ENABLE_RECEIVE |
                     NETWORK_CONTROL_ENABLE_TRANSMIT));

  /* Modify the NETWORK_CONFIG register based on the configured
   * speed and duplex
   */

  regval = mac_getreg(priv, NETWORK_CONFIG);
  regval &= ~(NETWORK_CONFIG_SPEED | NETWORK_CONFIG_FULL_DUPLEX |
              NETWORK_CONFIG_GIGABIT_MODE_ENABLE);

#ifdef CONFIG_MPFS_MAC_ETHFD
  regval |= NETWORK_CONFIG_FULL_DUPLEX;
#endif

#if defined(CONFIG_MPFS_MAC_ETH100MBPS)
  regval |= NETWORK_CONFIG_SPEED;
#elif defined(CONFIG_MPFS_MAC_ETH1000MBPS)
  regval |= NETWORK_CONFIG_GIGABIT_MODE_ENABLE;
#endif

  ninfo("set linkspeed: NETWORK_CONFIG=0x%x\n", regval);

  mac_putreg(priv, NETWORK_CONFIG, regval);
  mac_putreg(priv, NETWORK_CONTROL, ncr);
}
#endif

/****************************************************************************
 * Function: mpfs_ioctl
 *
 * Description:
 *  Handles driver ioctl calls:
 *
 *  SIOCMIINOTIFY - Set up to received notifications from PHY interrupting
 *    events.
 *
 *  SIOCGMIIPHY, SIOCGMIIREG, and SIOCSMIIREG:
 *    Executes the SIOCxMIIxxx command and responds using the request struct
 *    that must be provided as its 2nd parameter.
 *
 *    When called with SIOCGMIIPHY it will get the PHY address for the device
 *    and write it to the req->phy_id field of the request struct.
 *
 *    When called with SIOCGMIIREG it will read a register of the PHY that is
 *    specified using the req->reg_no struct field and then write its output
 *    to the req->val_out field.
 *
 *    When called with SIOCSMIIREG it will write to a register of the PHY
 *    that is specified using the req->reg_no struct field and use
 *    req->val_in as its input.
 *
 * Input Parameters:
 *   dev - Ethernet device structure
 *   cmd - SIOCxMIIxxx command code
 *   arg - Request structure also used to return values
 *
 * Returned Value: Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NETDEV_IOCTL
static int mpfs_ioctl(struct net_driver_s *dev, int cmd, unsigned long arg)
{
#ifdef CONFIG_NETDEV_PHY_IOCTL
  struct mpfs_ethmac_s *priv = (struct mpfs_ethmac_s *)dev->d_private;
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

              ret = mpfs_phyintenable(priv);
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

          /* Enable the management port */

          mpfs_enablemdio(priv);

          /* Read from the requested register */

          ret = mpfs_phyread(priv, req->phy_id, req->reg_num, &req->val_out);

          /* Disable the management port */

          mpfs_disablemdio(priv);
        }
        break;

      case SIOCSMIIREG: /* Set register in MII PHY */
        {
          struct mii_ioctl_data_s *req =
                  (struct mii_ioctl_data_s *)((uintptr_t)arg);

          /* Enable the management port */

          mpfs_enablemdio(priv);

          /* Write to the requested register */

          ret = mpfs_phywrite(priv, req->phy_id, req->reg_num, req->val_in);

          /* Disable the management port */

          mpfs_disablemdio(priv);
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
 * Function: mpfs_buffer_initialize
 *
 * Description:
 *   Allocate aligned TX and RX descriptors and buffers.  For the case of
 *   pre-allocated structures, the function degenerates to a few assignments.
 *
 * Input Parameters:
 *   priv - The GMAC driver state
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *   Called very early in the initialization sequence.
 *
 ****************************************************************************/

static int mpfs_buffer_initialize(struct mpfs_ethmac_s *priv,
                                  unsigned int queue)
{
#ifdef CONFIG_MPFS_ETHMAC_PREALLOCATE
  /* Use pre-allocated buffers */

  priv->txdesc   = g_txdesc;
  priv->rxdesc   = g_rxdesc;
  priv->txbuffer = g_txbuffer;
  priv->rxbuffer = g_rxbuffer;

#else
  size_t allocsize;

  /* Allocate buffers */

  allocsize = CONFIG_MPFS_ETHMAC_NTXBUFFERS * sizeof(struct gmac_txdesc_s);
  priv->queue[queue].tx_desc_tab = (struct gmac_txdesc_s *)
                                   kmm_memalign(8, allocsize);
  if (priv->queue[queue].tx_desc_tab == NULL)
    {
      nerr("ERROR: Failed to allocate TX descriptors\n");
      return -ENOMEM;
    }

  memset(priv->queue[queue].tx_desc_tab, 0, allocsize);

  allocsize = CONFIG_MPFS_ETHMAC_NRXBUFFERS * sizeof(struct gmac_rxdesc_s);
  priv->queue[queue].rx_desc_tab = kmm_memalign(8, allocsize);
  if (priv->queue[queue].rx_desc_tab == NULL)
    {
      nerr("ERROR: Failed to allocate RX descriptors\n");
      mpfs_buffer_free(priv, queue);
      return -ENOMEM;
    }

  memset(priv->queue[queue].rx_desc_tab, 0, allocsize);

  allocsize = CONFIG_MPFS_ETHMAC_NTXBUFFERS * GMAC_TX_UNITSIZE;
  priv->queue[queue].txbuffer = (uint8_t *)kmm_memalign(8, allocsize);
  if (priv->queue[queue].txbuffer == NULL)
    {
      nerr("ERROR: Failed to allocate TX buffer\n");
      mpfs_buffer_free(priv, queue);
      return -ENOMEM;
    }

  allocsize = CONFIG_MPFS_ETHMAC_NRXBUFFERS * GMAC_RX_UNITSIZE;
  priv->queue[queue].rxbuffer = (uint8_t *)kmm_memalign(8, allocsize);
  if (priv->queue[queue].rxbuffer == NULL)
    {
      nerr("ERROR: Failed to allocate RX buffer\n");
      mpfs_buffer_free(priv, queue);
      return -ENOMEM;
    }

#endif

  DEBUGASSERT(((uintptr_t)priv->queue[queue].rx_desc_tab & 7) == 0 &&
              ((uintptr_t)priv->queue[queue].rxbuffer    & 7) == 0 &&
              ((uintptr_t)priv->queue[queue].tx_desc_tab & 7) == 0 &&
              ((uintptr_t)priv->queue[queue].txbuffer    & 7) == 0);
  return OK;
}

/****************************************************************************
 * Function: mpfs_buffer_free
 *
 * Description:
 *   Free aligned TX and RX descriptors and buffers.  For the case of
 *   pre-allocated structures, the function does nothing.
 *
 * Input Parameters:
 *   priv - The GMAC driver state
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_buffer_free(struct mpfs_ethmac_s *priv, unsigned int queue)
{
#ifndef CONFIG_MPFS_GMAC_PREALLOCATE
  /* Free allocated buffers */

  if (priv->queue[queue].rx_desc_tab != NULL)
    {
      kmm_free(priv->queue[queue].rx_desc_tab);
      priv->queue[queue].rx_desc_tab = NULL;
    }

  if (priv->queue[queue].rx_desc_tab != NULL)
    {
      kmm_free(priv->queue[queue].rx_desc_tab);
      priv->queue[queue].rx_desc_tab = NULL;
    }

  if (priv->queue[queue].txbuffer != NULL)
    {
      kmm_free(priv->queue[queue].txbuffer);
      priv->queue[queue].txbuffer = NULL;
    }

  if (priv->queue[queue].txbuffer != NULL)
    {
      kmm_free(priv->queue[queue].txbuffer);
      priv->queue[queue].txbuffer = NULL;
    }
#endif
}

/****************************************************************************
 * Function: mpfs_txtimeout_work
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

static void mpfs_txtimeout_work(void *arg)
{
  struct mpfs_ethmac_s *priv = (struct mpfs_ethmac_s *)arg;

  nerr("ERROR: TX-Timeout!\n");

  /* Reset the hardware.  Just take the interface down, then back up again. */

  net_lock();
  mpfs_ifdown(&priv->dev);
  mpfs_ifup(&priv->dev);

  /* Then poll the network for new XMIT data */

  mpfs_dopoll(priv);
  net_unlock();
}

/****************************************************************************
 * Function: mpfs_txtimeout_expiry
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

static void mpfs_txtimeout_expiry(wdparm_t arg)
{
  struct mpfs_ethmac_s *priv = (struct mpfs_ethmac_s *)arg;
  unsigned int qi;

  /* Disable further Ethernet interrupts.  This will prevent some race
   * conditions with interrupt work.  There is still a potential race
   * condition with interrupt work that is already queued and in progress.
   */

  for (qi = 0; qi < MPFS_MAC_QUEUE_COUNT; qi++)
    {
      *priv->queue[qi].int_disable = 0xffffffff;
    }

  /* Schedule to perform the TX timeout processing on the worker thread. */

  work_queue(ETHWORK, &priv->irqwork, mpfs_txtimeout_work, priv, 0);
}

/****************************************************************************
 * Function: mpfs_transmit
 *
 * Description:
 *   Start hardware transmission.  Called either from the TX done interrupt
 *   handling or from watchdog based polling.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *  queue  - The queue to send from
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

static int mpfs_transmit(struct mpfs_ethmac_s *priv, unsigned int queue)
{
  struct net_driver_s *dev = &priv->dev;
  volatile struct gmac_txdesc_s *txdesc;
  uintptr_t addr;
  uint32_t status;
  uint32_t regval;

  ninfo("d_len: %d txhead: %d txtail: %d\n",
        dev->d_len, priv->queue[queue].txhead, priv->queue[queue].txtail);
  mpfs_dumppacket("Transmit packet", dev->d_buf, dev->d_len);

  /* Check parameter */

  if (dev->d_len > GMAC_TX_UNITSIZE)
    {
      nerr("ERROR: Packet too big: %d\n", dev->d_len);
      return -EINVAL;
    }

  /* Pointer to the current TX descriptor */

  txdesc = &priv->queue[queue].tx_desc_tab[priv->queue[queue].txhead];

  /* If no free TX descriptor, buffer can't be sent */

  if (mpfs_txfree(priv, queue) < 1)
    {
      nerr("ERROR: No free TX descriptors\n");
      return -EBUSY;
    }

  /* Mark buffer as used temporarily so DMA doesn't operate on it */

  status = GEM_TX_DMA_USED | GEM_TX_DMA_LAST;
  txdesc->status = status;

  /* Setup/Copy data to transmission buffer */

  if (dev->d_len > 0)
    {
      addr = txdesc->addr;
#ifdef MPFS_ETHMAC_64BIT_ADDRESS_MODE
      addr += (uintptr_t)(txdesc->addr_hi) << 32;
#endif
      memcpy((void *)addr, dev->d_buf, dev->d_len);
    }

  /* Update TX descriptor status. */

  status = (dev->d_len & GEM_DMA_STATUS_LENGTH_MASK) | GEM_TX_DMA_LAST;

  if (priv->queue[queue].txhead == CONFIG_MPFS_ETHMAC_NTXBUFFERS - 1)
    {
      status |= GEM_TX_DMA_WRAP;
    }

  /* Update the descriptor status */

  txdesc->status = status;

  /* Increment the head index */

  if (++priv->queue[queue].txhead >= CONFIG_MPFS_ETHMAC_NTXBUFFERS)
    {
      priv->queue[queue].txhead = 0;
    }

  /* Now start transmission (if it is not already done) */

  regval  = mac_getreg(priv, NETWORK_CONTROL);
  regval |= NETWORK_CONTROL_TRANSMIT_START;
  mac_putreg(priv, NETWORK_CONTROL, regval);

  /* Set up the TX timeout watchdog (perhaps restarting the timer) */

  wd_start(&priv->txtimeout, MPFS_TXTIMEOUT,
           mpfs_txtimeout_expiry, (wdparm_t)priv);

  /* Set d_len to zero meaning that the d_buf[] packet buffer is again
   * available.
   */

  dev->d_len = 0;

  /* If we have no more available TX descriptors, then we must disable the
   * RCOMP interrupt to stop further RX processing.  Why?  Because EACH RX
   * packet that is dispatched is also an opportunity to reply with a TX
   * packet.  So, if we cannot handle an RX packet reply, then we disable
   * all RX packet processing.
   */

  if (mpfs_txfree(priv, queue) < 1)
    {
      ninfo("Disabling RX interrupts\n");
      *priv->queue[queue].int_disable = INT_RX;
    }

  return OK;
}

/****************************************************************************
 * Function: mpfs_ethreset
 *
 * Description:
 *  Reset the Ethernet block.
 *
 * Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *
 ****************************************************************************/

static void mpfs_ethreset(struct mpfs_ethmac_s *priv)
{
  int qi;

  /* if we are supporting PHY IOCTLs, then do not reset the MAC. */

#ifndef CONFIG_NETDEV_PHY_IOCTL
  if (priv->regbase == MPFS_GEM0_LO_BASE)
    {
      /* reset */

      modifyreg32(MPFS_SYSREG_BASE + MPFS_SYSREG_SOFT_RESET_CR_OFFSET,
                  0, SYSREG_SOFT_RESET_CR_MAC0);
      modifyreg32(MPFS_SYSREG_BASE + MPFS_SYSREG_SOFT_RESET_CR_OFFSET,
                  SYSREG_SOFT_RESET_CR_MAC0, 0);
    }

  if (priv->regbase == MPFS_GEM1_LO_BASE)
    {
      /* reset */

      modifyreg32(MPFS_SYSREG_BASE + MPFS_SYSREG_SOFT_RESET_CR_OFFSET,
                  0, SYSREG_SOFT_RESET_CR_MAC1);
      modifyreg32(MPFS_SYSREG_BASE + MPFS_SYSREG_SOFT_RESET_CR_OFFSET,
                  SYSREG_SOFT_RESET_CR_MAC1, 0);
    }

#endif

  /* Disable all GMAC interrupts */

  for (qi = 0; qi < MPFS_MAC_QUEUE_COUNT; qi++)
    {
      *priv->queue[qi].int_disable = 0xffffffff;
      *priv->queue[qi].int_status  = 0xffffffff;
    }

  /* Reset RX and TX logic */

  mpfs_rxreset(priv);
  mpfs_txreset(priv);
}

/****************************************************************************
 * Function: mpfs_macconfig
 *
 * Description:
 *  Configure the Ethernet MAC for DMA operation.
 *
 * Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int mpfs_macconfig(struct mpfs_ethmac_s *priv)
{
  uint32_t net_config = 0;
  uint32_t net_control = 0;
  uint32_t dma_config = 0;
  uintptr_t addr;
  unsigned int qi;

  net_control = NETWORK_CONTROL_CLEAR_ALL_STATS_REGS;

  net_config = (((uint32_t)1) << NETWORK_CONFIG_DATA_BUS_WIDTH_SHIFT) |
               ((2 & NETWORK_CONFIG_MDC_CLOCK_DIVISOR_MASK)
                  << NETWORK_CONFIG_MDC_CLOCK_DIVISOR_SHIFT);

#ifdef CONFIG_MPFS_MAC_SGMII
  net_config |= NETWORK_CONFIG_SGMII_MODE_ENABLE | NETWORK_CONFIG_PCS_SELECT;
#endif

#ifdef CONFIG_NET_LOOPBACK
  net_control |= NETWORK_CONTROL_LOOPBACK_LOCAL;
#endif

#ifdef CONFIG_NET_PROMISCUOUS
  net_config |= NETWORK_CONFIG_COPY_ALL_FRAMES;
#endif

#ifdef CONFIG_MPFS_MAC_NO_BROADCAST
  net_config |= NETWORK_CONFIG_NO_BROADCAST;
#endif

  mac_putreg(priv, NETWORK_CONTROL, net_control);
  mac_putreg(priv, NETWORK_CONFIG,  net_config);

  mac_putreg(priv, RECEIVE_STATUS,  0xffffffff);
  mac_putreg(priv, TRANSMIT_STATUS, 0xffffffff);

  /* Disable and clear all ints */

  for (qi = 0; qi < MPFS_MAC_QUEUE_COUNT; qi++)
    {
      *priv->queue[qi].int_disable = 0xffffffff;
      *priv->queue[qi].int_status  = 0xffffffff;
    }

  /* TODO: If using only queue0 use all memory for that.
   * TX_Q_SEG_ALLOC_Q_LOWER xxx
   */

#ifdef CONFIG_MPFS_MAC_SGMII
  /* Reset PCS */

  mac_putreg(priv, PCS_CONTROL, PCS_CONTROL_SOFTWARE_RESET);
#endif

  /* Configure MAC Network DMA Config register */

  dma_config = (MPFS_MAC_RX_BUF_VALUE << DMA_CONFIG_RX_BUF_SIZE_SHIFT) |
                DMA_CONFIG_TX_PBUF_SIZE |
                (((uint32_t)0x3) << DMA_CONFIG_RX_PBUF_SIZE_SHIFT) |
                (((uint32_t)0x04 & DMA_CONFIG_AMBA_BURST_LENGTH_MASK));

#if defined(CONFIG_MPFS_ETHMAC_64BIT_ADDRESS_MODE)
  dma_config |= DMA_CONFIG_DMA_ADDR_BUS_WIDTH_1;
#endif

  mac_putreg(priv, DMA_CONFIG, dma_config);

  for (qi = 1; qi < MPFS_MAC_QUEUE_COUNT; qi++)
    {
      *priv->queue[qi].dma_rxbuf_size = MPFS_MAC_RX_BUF_VALUE;
    }

  /* Disable the other queues as the GEM reset leaves them enabled with an
   * address pointer of 0. Setting b0 of the queue pointer disables a queue.
   */

  addr = (uintptr_t)priv->queue[0].tx_desc_tab;
  mac_putreg(priv, UPPER_TX_Q_BASE_ADDR, upper_32_bits(addr));
  addr = (uintptr_t)priv->queue[0].rx_desc_tab;
  mac_putreg(priv, UPPER_RX_Q_BASE_ADDR, upper_32_bits(addr));

  mac_putreg(priv, TRANSMIT_Q1_PTR,
              ((uint32_t)(uint64_t)priv->queue[0].tx_desc_tab) | 1U);
  mac_putreg(priv, TRANSMIT_Q2_PTR,
              ((uint32_t)(uint64_t)priv->queue[0].tx_desc_tab) | 1U);
  mac_putreg(priv, TRANSMIT_Q3_PTR,
              ((uint32_t)(uint64_t)priv->queue[0].tx_desc_tab) | 1U);
  mac_putreg(priv, RECEIVE_Q1_PTR,
              ((uint32_t)(uint64_t)priv->queue[0].rx_desc_tab) | 1U);
  mac_putreg(priv, RECEIVE_Q2_PTR,
              ((uint32_t)(uint64_t)priv->queue[0].rx_desc_tab) | 1U);
  mac_putreg(priv, RECEIVE_Q3_PTR,
              ((uint32_t)(uint64_t)priv->queue[0].rx_desc_tab) | 1U);

  return OK;
}

/****************************************************************************
 * Function: mpfs_macenable
 *
 * Description:
 *  Enable normal MAC operation.
 *
 * Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int mpfs_macenable(struct mpfs_ethmac_s *priv)
{
  uint32_t regval;

  /* Reset TX and RX */

  mpfs_rxreset(priv);
  mpfs_txreset(priv);

  /* enable queue-0 DMA */

  uint32_t r = *priv->queue[0].rx_q_ptr & ~1u;
  *priv->queue[0].rx_q_ptr = r;

  /* enable global irqs */

  up_enable_irq(priv->mac_q_int[0]);
  up_enable_irq(priv->mac_q_int[1]);
  up_enable_irq(priv->mac_q_int[2]);
  up_enable_irq(priv->mac_q_int[3]);

  /* Enable Rx and Tx, enable the statistics registers. */

  regval  = mac_getreg(priv, NETWORK_CONTROL);
  regval |= (NETWORK_CONTROL_ENABLE_RECEIVE |
             NETWORK_CONTROL_ENABLE_TRANSMIT |
             NETWORK_CONTROL_STATS_WRITE_EN);
  mac_putreg(priv, NETWORK_CONTROL, regval);

  /* enable queue-0 irqs */

  *priv->queue[0].int_status = 0xffffffff;
  *priv->queue[0].int_enable = INT_ALL;

  return OK;
}

/****************************************************************************
 * Function: mpfs_mdcclock
 *
 * Description:
 *  Configure the MDC clocking
 *
 * Input Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_mdcclock(struct mpfs_ethmac_s *priv)
{
  uint32_t ncfgr;
  uint32_t ncr;
  uint32_t mck;

  /* Disable RX and TX momentarily */

  ncr = mac_getreg(priv, NETWORK_CONTROL);
  mac_putreg(priv, NETWORK_CONTROL, ncr &
             ~(NETWORK_CONTROL_ENABLE_RECEIVE |
              NETWORK_CONTROL_ENABLE_TRANSMIT));

  /* Modify the NCFGR register based on the configured board MCK frequency */

  ncfgr  = mac_getreg(priv, NETWORK_CONFIG);
  ncfgr &= ~(NETWORK_CONFIG_MDC_CLOCK_DIVISOR_MASK <<
             NETWORK_CONFIG_MDC_CLOCK_DIVISOR_SHIFT);

  mck = CONFIG_MPFS_ETHMAC_MDC_CLOCK_SOURCE_HZ;
  DEBUGASSERT(mck <= 240000000);

  if (mck <= 20000000)
    {
      ncfgr |= NETWORK_CONFIG_MDC_CLOCK_DIVISOR_8;
    }
  else if (mck <= 40000000)
    {
      ncfgr |= NETWORK_CONFIG_MDC_CLOCK_DIVISOR_16;
    }
  else if (mck <= 80000000)
    {
      ncfgr |= NETWORK_CONFIG_MDC_CLOCK_DIVISOR_32;
    }
  else if (mck <= 120000000)
    {
      ncfgr |= NETWORK_CONFIG_MDC_CLOCK_DIVISOR_48;
    }
  else if (mck <= 160000000)
    {
      ncfgr |= NETWORK_CONFIG_MDC_CLOCK_DIVISOR_64;
    }
  else
    {
      ncfgr |= NETWORK_CONFIG_MDC_CLOCK_DIVISOR_96;
    }

  mac_putreg(priv, NETWORK_CONFIG, ncfgr);

  /* Restore RX and TX enable settings */

  mac_putreg(priv, NETWORK_CONTROL, ncr);
}

/****************************************************************************
 * Function: mpfs_phyinit
 *
 * Description:
 *  Configure the PHY and determine the link speed/duplex.
 *
 * Input Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

static int mpfs_phyinit(struct mpfs_ethmac_s *priv)
{
  int ret;

  /* Configure PHY clocking */

  mpfs_mdcclock(priv);

  /* Check the PHY Address */

  priv->phyaddr = CONFIG_MPFS_PHYADDR;
  ret = mpfs_phyfind(priv, &priv->phyaddr);
  if (ret < 0)
    {
      nerr("ERROR: mpfs_phyfind failed: %d\n", ret);
      return ret;
    }

  /* We have a PHY address.  Reset the PHY */

  mpfs_phyreset(priv);
  return OK;
}

/****************************************************************************
 * Function: mpfs_phyreset
 *
 * Description:
 *  Reset the PHY
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

static int mpfs_phyreset(struct mpfs_ethmac_s *priv)
{
  uint16_t mcr;
  int timeout;
  int ret;

  ninfo(" mpfs_phyreset\n");

  /* Enable management port */

  mpfs_enablemdio(priv);

  /* Reset the PHY */

  ret = mpfs_phywrite(priv, priv->phyaddr, GMII_MCR,
                      GMII_MCR_RESET);
  if (ret < 0)
    {
      nerr("ERROR: mpfs_phywrite failed: %d\n", ret);
    }

  /* Wait for the PHY reset to complete */

  ret = -ETIMEDOUT;
  for (timeout = 0; timeout < PHY_RESET_WAIT_COUNT; timeout++)
    {
      mcr = GMII_MCR_RESET;
      int result = mpfs_phyread(priv, priv->phyaddr,
                                GMII_MCR, &mcr);
      if (result < 0)
        {
          nerr("ERROR: Failed to read the MCR register: %d\n", ret);
          ret = result;
        }
      else if ((mcr & GMII_MCR_RESET) == 0)
        {
          ret = OK;
          break;
        }
    }

  /* Disable management port */

  mpfs_disablemdio(priv);
  return ret;
}

/****************************************************************************
 * Function: mpfs_ethconfig
 *
 * Description:
 *  Configure the Ethernet interface for DMA operation.
 *
 * Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int mpfs_ethconfig(struct mpfs_ethmac_s *priv)
{
  int ret;

  ninfo("Entry\n");

#ifdef CONFIG_MPFS_PHYINIT
  /* Perform any necessary, board-specific PHY initialization */

  ret = mpfs_phy_boardinitialize(priv->intf);
  if (ret < 0)
    {
      nerr("ERROR: Failed to initialize the PHY: %d\n", ret);
      return ret;
    }
#endif

  /* Reset the Ethernet block */

  ninfo("Reset the Ethernet block\n");
  mpfs_ethreset(priv);

  /* Initialize the PHY */

  ninfo("Initialize the PHY\n");
  ret = mpfs_phyinit(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Initialize the MAC and DMA */

  ninfo("Initialize the MAC and DMA\n");
  ret = mpfs_macconfig(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Enable normal MAC operation */

  ninfo("Enable normal operation\n");
  return mpfs_macenable(priv);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: mpfs_ethinitialize
 *
 * Description:
 *   Initialize the Ethernet driver for one interface.  If the STM32 chip
 *   supports multiple Ethernet controllers, then board specific logic
 *   must implement arm_netinitialize() and call this function to initialize
 *   the desired interfaces.
 *
 * Parameters:
 *   intf - In the case where there are multiple EMACs, this value
 *          identifies which GMAC is to be initialized.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int mpfs_ethinitialize(int intf)
{
  struct mpfs_ethmac_s *priv;
  int ret = OK;
  uintptr_t base;

  ninfo("intf: %d\n", intf);

  /* Get the interface structure associated with this interface number. */

  DEBUGASSERT(intf < MPFS_NETHERNET);
  priv = &g_mpfsethmac[intf];

  /* Initialize the driver structure */

  memset(priv, 0, sizeof(struct mpfs_ethmac_s));
  priv->dev.d_buf     = g_pktbuf[intf]; /* Single packet buffer */
  priv->dev.d_ifup    = mpfs_ifup;      /* I/F up (new IP address) callback */
  priv->dev.d_ifdown  = mpfs_ifdown;    /* I/F down callback */
  priv->dev.d_txavail = mpfs_txavail;   /* New TX data callback */
#ifdef CONFIG_NET_MCASTGROUP
  priv->dev.d_addmac  = mpfs_addmac;    /* Add multicast MAC address */
  priv->dev.d_rmmac   = mpfs_rmmac;     /* Remove multicast MAC address */
#endif
#ifdef CONFIG_NETDEV_IOCTL
  priv->dev.d_ioctl   = mpfs_ioctl;     /* Support PHY ioctl() calls */
#endif
  priv->dev.d_private = priv;           /* Used to recover private state */
  priv->intf          = intf;           /* Remember the interface number */
  priv->regbase       = g_regbases[intf];
  priv->mac_q_int[0]  = g_irq_numbers[intf][0];
  priv->mac_q_int[1]  = g_irq_numbers[intf][1];
  priv->mac_q_int[2]  = g_irq_numbers[intf][2];
  priv->mac_q_int[3]  = g_irq_numbers[intf][3];
  ninfo("mac @ 0x%" PRIx64 "\n", priv->regbase);

  base = priv->regbase;
  priv->queue[0].int_status     = (uint32_t *)(base + INT_STATUS);
  priv->queue[1].int_status     = (uint32_t *)(base + INT_Q1_STATUS);
  priv->queue[2].int_status     = (uint32_t *)(base + INT_Q2_STATUS);
  priv->queue[3].int_status     = (uint32_t *)(base + INT_Q3_STATUS);
  priv->queue[0].int_mask       = (uint32_t *)(base + INT_MASK);
  priv->queue[1].int_mask       = (uint32_t *)(base + INT_Q1_MASK);
  priv->queue[2].int_mask       = (uint32_t *)(base + INT_Q2_MASK);
  priv->queue[3].int_mask       = (uint32_t *)(base + INT_Q3_MASK);
  priv->queue[0].int_enable     = (uint32_t *)(base + INT_ENABLE);
  priv->queue[1].int_enable     = (uint32_t *)(base + INT_Q1_ENABLE);
  priv->queue[2].int_enable     = (uint32_t *)(base + INT_Q2_ENABLE);
  priv->queue[3].int_enable     = (uint32_t *)(base + INT_Q3_ENABLE);
  priv->queue[0].int_disable    = (uint32_t *)(base + INT_DISABLE);
  priv->queue[1].int_disable    = (uint32_t *)(base + INT_Q1_DISABLE);
  priv->queue[2].int_disable    = (uint32_t *)(base + INT_Q2_DISABLE);
  priv->queue[3].int_disable    = (uint32_t *)(base + INT_Q3_DISABLE);
  priv->queue[0].rx_q_ptr       = (uint32_t *)(base + RECEIVE_Q_PTR);
  priv->queue[1].rx_q_ptr       = (uint32_t *)(base + RECEIVE_Q1_PTR);
  priv->queue[2].rx_q_ptr       = (uint32_t *)(base + RECEIVE_Q2_PTR);
  priv->queue[3].rx_q_ptr       = (uint32_t *)(base + RECEIVE_Q3_PTR);
  priv->queue[0].tx_q_ptr       = (uint32_t *)(base + TRANSMIT_Q_PTR);
  priv->queue[1].tx_q_ptr       = (uint32_t *)(base + TRANSMIT_Q1_PTR);
  priv->queue[2].tx_q_ptr       = (uint32_t *)(base + TRANSMIT_Q2_PTR);
  priv->queue[3].tx_q_ptr       = (uint32_t *)(base + TRANSMIT_Q3_PTR);
  priv->queue[0].dma_rxbuf_size = (uint32_t *)(base + DMA_RXBUF_SIZE_Q1);
  priv->queue[1].dma_rxbuf_size = (uint32_t *)(base + DMA_RXBUF_SIZE_Q1);
  priv->queue[2].dma_rxbuf_size = (uint32_t *)(base + DMA_RXBUF_SIZE_Q2);
  priv->queue[3].dma_rxbuf_size = (uint32_t *)(base + DMA_RXBUF_SIZE_Q3);

  /* MPU hack for ETH DMA if not enabled by bootloader */

#ifdef CONFIG_MPFS_MPU_DMA_ENABLE
#  ifdef CONFIG_MPFS_ETHMAC_0
  putreg64(0x1f00000fffffffff, MPFS_PMPCFG_ETH0_0);
#  endif
#  ifdef CONFIG_MPFS_ETHMAC_1
  putreg64(0x1f00000fffffffff, MPFS_PMPCFG_ETH1_0);
#  endif
#endif

  /* Allocate buffers */

  ret = mpfs_buffer_initialize(priv, 0);
  if (ret < 0)
    {
      nerr("ERROR: mpfs_buffer_initialize failed: %d\n", ret);
      return ret;
    }

  /* Attach the IRQ to the driver */

  if (irq_attach(priv->mac_q_int[0], mpfs_interrupt_0, priv))
    {
      /* We could not attach the ISR to the interrupt */

      return -EAGAIN;
    }

  if (irq_attach(priv->mac_q_int[1], mpfs_interrupt_1, NULL))
    {
      /* We could not attach the ISR to the interrupt */

      return -EAGAIN;
    }

  if (irq_attach(priv->mac_q_int[2], mpfs_interrupt_2, NULL))
    {
      /* We could not attach the ISR to the interrupt */

      return -EAGAIN;
    }

  if (irq_attach(priv->mac_q_int[3], mpfs_interrupt_3, NULL))
    {
      /* We could not attach the ISR to the interrupt */

      return -EAGAIN;
    }

  /* Enable clocking to the GMAC peripheral (just for mpfs_ifdown()) */

  /* MAC HW clock enable and reset */

  if (priv->regbase == MPFS_GEM0_LO_BASE)
    {
      modifyreg32(MPFS_SYSREG_BASE + MPFS_SYSREG_SUBBLK_CLOCK_CR_OFFSET, 0,
                  SYSREG_SUBBLK_CLOCK_CR_MAC0);
      modifyreg32(MPFS_SYSREG_BASE + MPFS_SYSREG_SOFT_RESET_CR_OFFSET,
                  0, SYSREG_SOFT_RESET_CR_MAC0);
      modifyreg32(MPFS_SYSREG_BASE + MPFS_SYSREG_SOFT_RESET_CR_OFFSET,
                  SYSREG_SOFT_RESET_CR_MAC0, 0);
    }

  if (priv->regbase == MPFS_GEM1_LO_BASE)
    {
      modifyreg32(MPFS_SYSREG_BASE + MPFS_SYSREG_SUBBLK_CLOCK_CR_OFFSET, 0,
                  SYSREG_SUBBLK_CLOCK_CR_MAC1);
      modifyreg32(MPFS_SYSREG_BASE + MPFS_SYSREG_SOFT_RESET_CR_OFFSET,
                  0, SYSREG_SOFT_RESET_CR_MAC1);
      modifyreg32(MPFS_SYSREG_BASE + MPFS_SYSREG_SOFT_RESET_CR_OFFSET,
                  SYSREG_SOFT_RESET_CR_MAC1, 0);
    }

  /* Put the interface in the down state. */

  ret = mpfs_ifdown(&priv->dev);
  if (ret < 0)
    {
      nerr("ERROR: Failed to put the interface in the down state: %d\n",
           ret);
      goto errout_with_buffers;
    }

  /* Register the device with the OS so that socket IOCTLs can be performed */

  ret = netdev_register(&priv->dev, NET_LL_ETHERNET);
  if (ret >= 0)
    {
      return ret;
    }

  nerr("ERROR: netdev_register() failed: %d\n", ret);

errout_with_buffers:
  mpfs_buffer_free(priv, 0);
  return ret;
}

#endif /* CONFIG_NET && CONFIG_MPFS_ETHMAC */

/****************************************************************************
 * Function: riscv_netinitialize
 *
 * Description:
 *   This is the "standard" network initialization logic called from the
 *   low-level initialization logic in riscv_initialize.c. If MPFS_NETHERNET
 *   greater than one, then board specific logic will have to supply a
 *   version of riscv_netinitialize() that calls mpfs_ethinitialize() with
 *   the appropriate interface number.
 *
 * Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *
 ****************************************************************************/

#if !defined(CONFIG_NETDEV_LATEINIT)
void riscv_netinitialize(void)
{
  mpfs_ethinitialize(0);
}
#endif
