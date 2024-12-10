/****************************************************************************
 * arch/arm64/src/imx9/imx9_enet.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <inttypes.h>
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
#include <nuttx/net/phy.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/netdev.h>

#ifdef CONFIG_NET_PKT
#  include <nuttx/net/pkt.h>
#endif

#include <arch/barriers.h>
#include <arch/board/board.h>

#include "arm64_internal.h"
#include "chip.h"
#include "hardware/imx9_enet.h"
#include "imx9_enet.h"

#include "imx9_ccm.h"
#include "imx9_iomuxc.h"
#include "hardware/imx9_ccm.h"
#include "hardware/imx9_pinmux.h"
#include "hardware/imx9_blk_ctrl.h"

#ifdef CONFIG_IMX9_ENET

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* If processing is not done at the interrupt level, then work queue support
 * is required.
 */

#if !defined(CONFIG_SCHED_LPWORK)
#  error LPWORK queue support is required
#endif

#define ETHWORK LPWORK

/* We need at least two TX buffers for reliable operation */

#if CONFIG_IMX9_ENET_NTXBUFFERS < 1
#define IMX9_ENET_NTXBUFFERS 1
#else
#define IMX9_ENET_NTXBUFFERS CONFIG_IMX9_ENET_NTXBUFFERS
#endif

/* We need an even number of RX buffers, since RX descriptors are
 * freed for the DMA in pairs due to two descriptors always fitting
 * in one cache line (cahce line size is 64, descriptor size is 32)
 */

#if CONFIG_IMX9_ENET_NRXBUFFERS < 2
#define IMX9_ENET_NRXBUFFERS 2
#elif CONFIG_IMX9_ENET_NRXBUFFERS & 1
#define IMX9_ENET_NRXBUFFERS (CONFIG_IMX9_ENET_NRXBUFFERS + 1)
#else
#define IMX9_ENET_NRXBUFFERS CONFIG_IMX9_ENET_NRXBUFFERS
#endif

#define nitems(_a)    (sizeof(_a) / sizeof(0[(_a)]))

#define ALIGNED_BUFSIZE   ENET_ALIGN_UP(CONFIG_NET_ETH_PKTSIZE + \
                                      CONFIG_NET_GUARDSIZE)

/* TX timeout = 1 second */

#define IMX9_TXTIMEOUT    (CLK_TCK)
#define MII_MAXPOLLS      (0x1ffff)
#define LINK_WAITUS       (100 * 1000)
#define LINK_NLOOPS       (50)

/* PHY reset tim in loop counts */

#define PHY_RESET_WAIT_COUNT (10)

/* Estimate the MII_SPEED in order to get an MDC close to 2.5MHz,
 * based on the internal module (ENET) clock:

 * MII clock frequency = 133 MHz / ((26 + 1) x 2) = 2.5 MHz
 *
 * TODO: This is hard-coded for now, could be properly calculated
 */

#define IMX9_MII_SPEED  26

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

#define IMX93_OCOTP_UID_OFFSET 0xc0

#define MMD1                  1
#define MMD1_PMA_STATUS1      1
#define MMD1_PS1_RECEIVE_LINK_STATUS (1 << 2)

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum phy_type_t
{
  PHY_NONE  = 0,
  PHY_RMII  = 1,
  PHY_RGMII = 2,
};

/* The imx9_driver_s encapsulates all state information for
 * a single hardware interface
 */

struct imx9_driver_s
{
  struct net_driver_s          dev;         /* Interface understood by the network */
  const uint32_t               base;        /* Base address of ENET controller */
  const int                    clk_gate;    /* Enet clock gate */
  const int                    irq;         /* Enet interrupt */
  const struct phy_desc_s *    phy_list;    /* Supported PHYs for this IF */
  const int                    n_phys;      /* Number of supported PHYs */
  struct enet_txdesc_s * const txdesc;      /* A pointer to the list of TX descriptor */
  struct enet_desc_s * const   rxdesc;      /* A pointer to the list of RX descriptors */
  const uintptr_t              buffer_pool; /* DMA buffer pool */
#ifdef CONFIG_IMX9_ENET_USE_OTP_MAC
  const off_t                  otp_mac_off; /* MAC address offset in OTP */
#endif
  const bool                   promiscuous; /* Set promiscuous mode */
  const enum phy_type_t        phy_type;    /* PHY type */
  const bool                   autoneg;     /* Phy autonegotiation enabled */
  const bool                   force_speed; /* Disable autonegotiation and force speed */
  bool                         full_duplex; /* Manually set to full duplex mode */
  bool                         s_10mbps;    /* Manually set to 10 MBPS */
  bool                         s_100mbps;   /* Manually set to 100 M0BPS */
  bool                         s_1000mbps;  /* Manually set to 1GBPS */
  const struct phy_desc_s *    cur_phy;     /* Currently selected phy */
  bool                         bifup;       /* true:ifup false:ifdown */
  uint8_t                      txhead;      /* The next TX descriptor to use */
  uint8_t                      rxtail;      /* The next RX descriptor to use */
  uint8_t                      phyaddr;     /* Selected PHY address */
  struct wdog_s                txtimeout;   /* TX timeout timer */
  uint32_t                     ints;        /* Enabled interrupts */
  struct work_s                irqwork;     /* For deferring interrupt work to the work queue */
  struct work_s                pollwork;    /* For deferring poll work to the work queue */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Utility functions */

static inline uint32_t imx9_enet_getreg32(struct imx9_driver_s *priv,
                                          uint32_t offset);
static inline void imx9_enet_putreg32(struct imx9_driver_s *priv,
                                      uint32_t value, uint32_t offset);

static inline void imx9_enet_modifyreg32(struct imx9_driver_s *priv,
                                         unsigned int offset,
                                         uint32_t clearbits,
                                         uint32_t setbits);

/* Common TX logic */

static bool imx9_txringfull(struct imx9_driver_s *priv);
static int  imx9_transmit(struct imx9_driver_s *priv,
                          uint32_t *buf_swap);
static int  imx9_txpoll(struct net_driver_s *dev);

/* Interrupt handling */

static void imx9_dispatch(struct imx9_driver_s *priv);
static void imx9_receive(struct imx9_driver_s *priv);
static void imx9_txdone(struct imx9_driver_s *priv);

static void imx9_enet_interrupt_work(void *arg);
static int  imx9_enet_interrupt(int irq, void *context, void *arg);

/* Watchdog timer expirations */

static void imx9_txtimeout_work(void *arg);
static void imx9_txtimeout_expiry(wdparm_t arg);

/* NuttX callback functions */

static int  imx9_ifup(struct net_driver_s *dev);
static int  imx9_ifdown(struct net_driver_s *dev);

static void imx9_txavail_work(void *arg);
static int  imx9_txavail(struct net_driver_s *dev);

/* Internal ifup function that allows phy reset to be optional */

static int imx9_ifup_action(struct net_driver_s *dev, bool resetphy);

#ifdef CONFIG_NET_MCASTGROUP
static int  imx9_addmac(struct net_driver_s *dev, const uint8_t *mac);
static int  imx9_rmmac(struct net_driver_s *dev, const uint8_t *mac);
#endif

#ifdef CONFIG_NETDEV_IOCTL
static int  imx9_ioctl(struct net_driver_s *dev, int cmd,
                       unsigned long arg);
#endif

/* PHY/MII support */

static int imx9_phy_is(struct imx9_driver_s *priv, const char *name);
static int imx9_determine_phy(struct imx9_driver_s *priv);

#if defined(CONFIG_NETDEV_PHY_IOCTL) && defined(CONFIG_ARCH_PHY_INTERRUPT)
static int imx9_phyintenable(struct imx9_driver_s *priv);
#endif
static inline void imx9_initmii(struct imx9_driver_s *priv);
static int imx9_writemii(struct imx9_driver_s *priv, uint8_t regaddr,
                         uint16_t data);
static int imx9_readmii(struct imx9_driver_s *priv, uint8_t regaddr,
                        uint16_t *data);
static int imx9_initphy(struct imx9_driver_s *priv, bool renogphy);

static int imx9_readmmd(struct imx9_driver_s *priv, uint8_t mmd,
                        uint16_t regaddr, uint16_t *data);
#if 0
static int imx9_writemmd(struct imx9_driver_s *priv, uint8_t mmd,
                         uint16_t regaddr, uint16_t data);
#endif

/* Initialization */

static void imx9_initbuffers(struct imx9_driver_s *priv);
static void imx9_reset(struct imx9_driver_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_IMX9_ENET1

/* If the board didn't provide a list of known PHYs, we can still work with
 * autonegotiation disabled and setting the speed manually
 */

#ifndef BOARD_ENET1_PHY_LIST
#define BOARD_ENET1_PHY_LIST {}
#endif

static const struct phy_desc_s g_enet1_phy_list[] = BOARD_ENET1_PHY_LIST;

/* The DMA descriptors */

static struct enet_txdesc_s g_enet1_tx_desc_pool[IMX9_ENET_NTXBUFFERS]
                                                 aligned_data(ENET_ALIGN);
static struct enet_desc_s g_enet1_rx_desc_pool[IMX9_ENET_NRXBUFFERS]
                                               aligned_data(ENET_ALIGN);

/* The DMA buffers */

static uint8_t g_enet_1_buffer_pool
               [IMX9_ENET_NTXBUFFERS + IMX9_ENET_NRXBUFFERS][ALIGNED_BUFSIZE]
                aligned_data(ENET_ALIGN);
#endif

static struct imx9_driver_s g_enet[] =
{
#ifdef CONFIG_IMX9_ENET1
  {
    .dev =
      {
        .d_ifup    = imx9_ifup,
        .d_ifdown  = imx9_ifdown,
        .d_txavail = imx9_txavail,

#  ifdef CONFIG_NET_MCASTGROUP
        .d_addmac  = imx9_addmac,
        .d_rmmac   = imx9_rmmac,
#  endif

#  ifdef CONFIG_NETDEV_IOCTL
        .d_ioctl   = imx9_ioctl,
#  endif
      },
    .base         = IMX9_ENET_BASE,
    .clk_gate     = CCM_LPCG_ENET1,
    .irq          = IMX9_IRQ_ENET,
    .phy_list     = g_enet1_phy_list,
    .n_phys       = nitems(g_enet1_phy_list),
    .txdesc       = g_enet1_tx_desc_pool,
    .rxdesc       = g_enet1_rx_desc_pool,
    .buffer_pool  = (const uintptr_t)g_enet_1_buffer_pool,

#  ifdef CONFIG_IMX9_ENET_USE_OTP_MAC
    .otp_mac_off  = CONFIG_IMX9_ENET1_OTP_MAC_ADDR,
#  endif

#  ifdef CONFIG_IMX9_ENET1_PROMISCUOUS
    .promiscuous  = trued
#  endif

#    ifdef CONFIG_IMX9_ENET1_RGMII
    .phy_type     = PHY_RGMII,
#    elif defined(CONFIG_IMX9_ENET1_RMII)
    .phy_type     = PHY_RMII,
#    else
#      error PHY must be RGMII or RMII
#    endif

    /* Duplex: default to FD */

#  if defined(CONFIG_IMX9_ENET1_PHY_FD) || defined(CONFIG_IMX9_ENET1_PHY_AUTONEG)
    .full_duplex  = true,
#  endif

    /* 10 mbps, default to false */

#  ifdef CONFIG_IMX9_ENET1_PHY_10MBPS
    .s_10mbps     = true,
#  endif

    /* 100 mbps, default  to true */

#  if defined(CONFIG_IMX9_ENET1_PHY_100MBPS) || defined(CONFIG_IMX9_ENET1_PHY_AUTONEG)
    .s_100mbps    = true,
#  endif

    /* 1000 mbps, default to false */

#  ifdef CONFIG_IMX9_ENET1_PHY_1000MBPS
    .s_1000mbps   = true,
#  endif

#  ifdef CONFIG_IMX9_ENET1_PHY_AUTONEG
    .autoneg      = true,
#  else
#    ifdef CONFIG_IMX9_ENET1_PHY_FORCE_SPEED
    .force_speed  = true,
#    endif
#  endif
  },
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx9_enet_getreg32
 *
 * Description:
 *   Get the contents of the ENET register at offset
 *
 * Input Parameters:
 *   priv   - private ENET device structure
 *   offset - offset to the register of interest
 *
 * Returned Value:
 *   The contents of the 32-bit register
 *
 ****************************************************************************/

static inline uint32_t imx9_enet_getreg32(struct imx9_driver_s *priv,
                                          uint32_t offset)
{
  return getreg32(priv->base + offset);
}

/****************************************************************************
 * Name: imx9_enet_modifyreg32
 *
 * Description:
 *   Atomically modify the specified bits in a memory mapped register
 *
 * Input Parameters:
 *   priv      - private SPI device structure
 *   offset    - offset to the register of interest
 *   clearbits - the 32-bit value to be written as 0s
 *   setbits   - the 32-bit value to be written as 1s
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void imx9_enet_modifyreg32(struct imx9_driver_s *priv,
                                         unsigned int offset,
                                         uint32_t clearbits,
                                         uint32_t setbits)
{
  modifyreg32(priv->base + offset, clearbits, setbits);
}

/****************************************************************************
 * Name: imx9_enet_putreg32
 *
 * Description:
 *   Write a 16-bit value to the ENET register at offset
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *   value  - the 32-bit value to be written
 *   offset - offset to the register of interest
 *
 * Returned Value:
 *   The contents of the 32-bit register
 *
 ****************************************************************************/

static inline void imx9_enet_putreg32(struct imx9_driver_s *priv,
                                      uint32_t value, uint32_t offset)
{
  putreg32(value, priv->base + offset);
}

/****************************************************************************
 * Function: dump_descriptor
 *
 * Description:
 *   Can be used for debugging; dumps the content of a DMA descriptor
 *
 * Input Parameters:
 *   desc  - Pointer to DMA descriptor
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

inline static void dump_descriptor(struct enet_desc_s *desc)
{
  _alert("length %d\n", desc->length);
  _alert("status1 0x%04x\n", desc->status1);
  _alert("data 0x%08x\n", desc->data);
  _alert("status2 0x%08x\n", desc->status2);
  _alert("checksum 0x%08x\n", desc->checksum);
  _alert("lenproto 0x%08x\n", desc->lenproto);
  _alert("bdu 0x%08x\n", desc->bdu);
  _alert("timestamp 0x%08x\n", desc->timestamp);
}

/****************************************************************************
 * Function: imx9_txringfull
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

static bool imx9_txringfull(struct imx9_driver_s *priv)
{
  struct enet_desc_s *txdesc = &priv->txdesc[priv->txhead].d1;
  struct enet_desc_s *txdesc2 = &priv->txdesc[priv->txhead].d2;

  up_invalidate_dcache((uintptr_t)txdesc,
                       (uintptr_t)txdesc + sizeof(struct enet_txdesc_s));

  return (txdesc2->status1 & TXDESC_R) != 0;
}

/****************************************************************************
 * Function: imx9_transmit
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

static int imx9_transmit(struct imx9_driver_s *priv, uint32_t *buf_swap)
{
  struct enet_desc_s *txdesc = &priv->txdesc[priv->txhead].d1;
  struct enet_desc_s *txdesc2 = &priv->txdesc[priv->txhead].d2;
  int split;
  uint32_t buf = (uintptr_t)priv->dev.d_buf;
  int len = priv->dev.d_len;

  DEBUGASSERT(len > 0 && buf != 0);
  DEBUGASSERT((buf & ENET_ALIGN_MASK) == 0);

  if (imx9_txringfull(priv))
    {
      /* Ring is full; this can only happen if transmit is called directly
       * from the receive path. The buffer is lost.
       */

      nerr("TX ring full, packet lost\n");
      NETDEV_TXERRORS(&priv->dev);
      return -EBUSY;
    }

  if (len > ALIGNED_BUFSIZE)
    {
      nerr("TX frame too large %d, max %d\n", len,
           ALIGNED_BUFSIZE);
    }

  /* We are done with the provided buffer after transmit */

  priv->dev.d_buf = NULL;
  priv->dev.d_len = 0;

  /* Increment statistics */

  NETDEV_TXPACKETS(&priv->dev);

  /* Optimize the two-descriptor usage; if possible, align the second part
   * on 64-byte boundary. Note that 0-length buffers are not accepted by
   * the DMA, so we must put some data to both descriptors.
   */

  split = len > 64 ? 64 : 1;

  txdesc->length   = split;
  txdesc->status2  = TXDESC_TS /* | TXDESC_IINS | TXDESC_PINS */;
  txdesc->bdu      = 0x00000000;

  txdesc2->length   = len - split;
  txdesc2->status2  = TXDESC_TS | TXDESC_INT /* | TXDESC_IINS | TXDESC_PINS */;
  txdesc2->bdu      = 0x00000000;

  if (buf_swap)
    {
      /* Data was written into the RX buffer, so swap the TX and RX buffers */

      DEBUGASSERT(*buf_swap == buf);
      *buf_swap = txdesc->data;
      txdesc->data = buf;
    }
  else
    {
      DEBUGASSERT(txdesc->data == buf);
    }

  txdesc2->data = buf + split;

  UP_DSB();

  /* Make sure the buffer data is in memory */

  up_clean_dcache(buf, buf + len);

  /* Descriptors & buffer data are ready to send */

  txdesc2->status1 = (txdesc2->status1 & TXDESC_W) |
                     (TXDESC_TC | TXDESC_L | TXDESC_R);

  /* Proceed to next descriptors */

  priv->txhead++;
  if (priv->txhead >= IMX9_ENET_NTXBUFFERS)
    {
      priv->txhead = 0;
    }

  /* If all TX descriptors are in-flight, then we have to disable receive
   * interrupts too.  This is because receive events can trigger more un-
   * stoppable transmit events.
   */

  if (imx9_txringfull(priv))
    {
      priv->ints &= ~RX_INTERRUPTS;
    }

  /* Enable TX interrupts */

  priv->ints |= TX_INTERRUPTS;
  imx9_enet_putreg32(priv, priv->ints, IMX9_ENET_EIMR_OFFSET);

  /* The latter descriptor was update first. This ensures that the DMA
   * won't start before all the descriptor data has been updated and it
   * is safe to clean the cache
   */

  UP_DMB();
  txdesc->status1 = TXDESC_R;
  UP_DSB();

  /* Make sure the descriptors are written from cache to memory */

  up_clean_dcache((uintptr_t)txdesc,
                  (uintptr_t)txdesc + sizeof(struct enet_txdesc_s));

  /* Start the TX transfer (if it was not already waiting for buffers) */

  imx9_enet_putreg32(priv, ENET_TDAR, IMX9_ENET_TDAR_OFFSET);

  /* Setup the TX timeout watchdog (perhaps restarting the timer) */

  wd_start(&priv->txtimeout, IMX9_TXTIMEOUT,
           imx9_txtimeout_expiry, (wdparm_t)priv);

  return OK;
}

/****************************************************************************
 * Function: imx9_txpoll
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

static int imx9_txpoll(struct net_driver_s *dev)
{
  struct imx9_driver_s *priv = (struct imx9_driver_s *)dev;

  /* Send the packet */

  imx9_transmit(priv, NULL);

  /* Check if the next TX descriptor is owned by the Ethernet DMA or
   * CPU.  We cannot perform the TX poll if we are unable to accept
   * another packet for transmission.
   */

  if (imx9_txringfull(priv))
    {
      return -EBUSY;
    }

  /* Return 0 to continue polling */

  return 0;
}

/****************************************************************************
 * Function: imx9_dopoll
 *
 * Description:
 *   The function is called in order to perform an out-of-sequence TX poll.
 *   This is done:
 *
 *   1. After completion of a transmission (stm32_txdone),
 *   2. When new TX data is available (stm32_txavail_process), and
 *   3. After a TX timeout to restart the sending process
 *      (stm32_txtimeout_process).
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static void imx9_dopoll(struct imx9_driver_s *priv)
{
  struct net_driver_s *dev = &priv->dev;
  struct enet_desc_s *txdesc;
  struct enet_desc_s *txdesc2;

  /* Check if the next TX descriptor is owned by the Ethernet DMA or
   * CPU.  We cannot perform the TX poll if we are unable to accept
   * another packet for transmission.
   */

  if (!imx9_txringfull(priv))
    {
      DEBUGASSERT(dev->d_len == 0 && dev->d_buf == NULL);

      txdesc = &priv->txdesc[priv->txhead].d1;
      txdesc2 = &priv->txdesc[priv->txhead].d2;

      /* Debug: check for any errors in the previously sent descriptors.
       * Note: cache line was invalidated in the imx9_txringfull already
       */

      if ((txdesc->status2 & TXDESC_STATUS2_ERRORS) != 0)
        {
          nerr("d1 status1 %x, status2 %x\n", txdesc->status1 & TXDESC_TXE,
               txdesc->status2 & TXDESC_STATUS2_ERRORS);
        }

      if ((txdesc2->status2 & TXDESC_STATUS2_ERRORS) != 0)
        {
          nerr("d2 status1 %x, status2 %x\n", txdesc2->status1 & TXDESC_TXE,
               txdesc2->status2 & TXDESC_STATUS2_ERRORS);
        }

      /* Poll for new data */

      dev->d_buf = (uint8_t *)(uintptr_t)txdesc->data;
      devif_poll(dev, imx9_txpoll);
      dev->d_buf = NULL;
      dev->d_len = 0;
    }
  else
    {
      nerr("TX ring full\n");
    }
}

/****************************************************************************
 * Function: imx9_dispatch
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

static inline void imx9_dispatch(struct imx9_driver_s *priv)
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
    }
  else
#endif
#ifdef CONFIG_NET_ARP
  /* Check for an ARP packet */

  if (BUF->type == HTONS(ETHTYPE_ARP))
    {
      NETDEV_RXARP(&priv->dev);
      arp_input(&priv->dev);
    }
#endif
  else
    {
      priv->dev.d_buf = NULL;
      priv->dev.d_len = 0;
      NETDEV_RXDROPPED(&priv->dev);
    }
}

inline static bool imx9_rxdesc_full(struct enet_desc_s *rxdesc)
{
  up_invalidate_dcache((uintptr_t)rxdesc,
                       (uintptr_t)rxdesc + sizeof(struct enet_desc_s));

  /* Check if the data buffer associated with the descriptor has
   * been filled or reception terminated for errors
   */

  return (rxdesc->status1 & RXDESC_E) == 0;
}

/****************************************************************************
 * Function: imx9_receive
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

static void imx9_receive(struct imx9_driver_s *priv)
{
  static uint32_t swap_data[2];
  int tail = priv->rxtail;
  int swap_d_n;
  struct enet_desc_s *rxdesc;
  bool received;

  /* Loop while there are received packets to be processed */

  do
    {
      rxdesc = &priv->rxdesc[tail];
      received = imx9_rxdesc_full(rxdesc);
      if (received)
        {
          /* Copy the buffer pointer to priv->dev.d_buf.  Set amount of data
           * in priv->dev.d_len
           */

          DEBUGASSERT(priv->dev.d_buf == NULL);

          if ((rxdesc->status1 & RXDESC_STATUS1_ERRORS) != 0 ||
              (rxdesc->status2 & RXDESC_STATUS2_ERRORS) != 0)
            {
              nerr("status1 %x, status2 %x",
                   rxdesc->status1 & RXDESC_STATUS1_ERRORS,
                   rxdesc->status2 & RXDESC_STATUS2_ERRORS);
            }

          DEBUGASSERT(rxdesc->length > 0 &&
                      ((uintptr_t)rxdesc->data & ENET_ALIGN_MASK) == 0);

          priv->dev.d_len = rxdesc->length;
          priv->dev.d_buf = (uint8_t *)(uintptr_t)rxdesc->data;

          /* Invalidate the buffer so that the correct packet will be re-read
           * from memory when the packet content is accessed.
           */

          up_invalidate_dcache((uintptr_t)priv->dev.d_buf,
                               (uintptr_t)priv->dev.d_buf + priv->dev.d_len);

          /* Dispatch (or drop) the newly received packet */

          imx9_dispatch(priv);

          /* If the dispatch resulted in data that should
           * be sent out on the network, the field d_len will set to a
           * value > 0. In this case imx9_transmit will just directly use
           * the provided buffer to transmit, and swap the rx / tx buffers
           */

          swap_d_n = tail & 1;
          swap_data[swap_d_n] = rxdesc->data;
          if (priv->dev.d_len > 0)
            {
              /* And send the packet */

              imx9_transmit(priv, &swap_data[swap_d_n]);

              /* Assume that the upper levels didn't write to the tx buffer
               * beyond the d_len, so the transmit buffer cache is clean.
               * If this wouldn't be the case, we'd have to invalidate here!
               * up_invalidate_dcache(swap_data[swap_d_n],
               *                      swap_data[swap_d_n] + ALIGNED_BUFSIZE);
               */
            }

          /* We are done with the buffers - let's not leave the pointers
           * laying around
           */

          priv->dev.d_buf = NULL;
          priv->dev.d_len = 0;

          /* RX descriptor size is 32 bytes, but the cache line size is 64.
           * This means that we can only free the rx descriptors in pairs.
           * If we are the second descriptor of the pair, we update both
           */

          if (swap_d_n == 1)
            {
              /* First update the second descriptor - RX DMA may not start
               * before both are updated
               */

              rxdesc->data     = swap_data[1];
              rxdesc->length   = 0;
              rxdesc->status2  = RXDESC_INT;
              rxdesc->bdu      = 0x00000000;
              rxdesc->status1  = (rxdesc->status1 & RXDESC_W) | RXDESC_E;

              /* Now update the first descriptor of the pair */

              rxdesc -= 1;
              rxdesc->data     = swap_data[0];
              rxdesc->length   = 0;
              rxdesc->status2  = RXDESC_INT;
              rxdesc->bdu      = 0x00000000;

              /* Make sure both descriptors are fully updated before updating
               * the first descriptor's status1; this allows DMA to proceed
               * to this descriptor pair.
               */

              UP_DMB();
              rxdesc->status1  = RXDESC_E;
              UP_DSB();

              up_clean_dcache((uintptr_t)&rxdesc[(-1)],
                              (uintptr_t)&rxdesc[(-1)] +
                              2 * sizeof(rxdesc[0]));

              /* Indicate that we produced empty receive buffers */

              imx9_enet_putreg32(priv, ENET_RDAR, IMX9_ENET_RDAR_OFFSET);
            }

          tail++;
          if (tail >= IMX9_ENET_NRXBUFFERS)
            {
              tail = 0;
            }
        }
    }
  while (received);

  /* Update the index to the next empty descriptor */

  priv->rxtail = tail;
}

/****************************************************************************
 * Function: imx9_txdone
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

static void imx9_txdone(struct imx9_driver_s *priv)
{
  DEBUGASSERT(priv->dev.d_len == 0 && priv->dev.d_buf == NULL);

  /* Cancel the timeout watchdog */

  wd_cancel(&priv->txtimeout);

  /* Update statistics */

  NETDEV_TXDONE(&priv->dev);

  priv->ints |= RX_INTERRUPTS;
  imx9_enet_putreg32(priv, priv->ints, IMX9_ENET_EIMR_OFFSET);

  /* Poll the network for new XMIT data */

  imx9_dopoll(priv);
}

/****************************************************************************
 * Function: imx9_enet_interrupt_work
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

static void imx9_enet_interrupt_work(void *arg)
{
  struct imx9_driver_s *priv = (struct imx9_driver_s *)arg;
  uint32_t pending;
#ifdef CONFIG_NET_MCASTGROUP
  uint32_t gaurstore;
  uint32_t galrstore;
#endif

  /* Process pending Ethernet interrupts */

  net_lock();

  /* Get the set of unmasked, pending interrupt. */

  pending = imx9_enet_getreg32(priv, IMX9_ENET_EIR_OFFSET) & priv->ints;

  /* Clear the pending interrupts */

  imx9_enet_putreg32(priv, pending, IMX9_ENET_EIR_OFFSET);

  /* Check for errors */

  if (pending & ERROR_INTERRUPTS)
    {
      /* An error has occurred, update statistics */

      NETDEV_ERRORS(&priv->dev);

      nerr("pending %" PRIx32 " ints %" PRIx32 "\n", pending, priv->ints);
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

      gaurstore = imx9_enet_getreg32(priv, IMX9_ENET_GAUR_OFFSET);
      galrstore = imx9_enet_getreg32(priv, IMX9_ENET_GALR_OFFSET);
#endif

      imx9_ifdown(&priv->dev);
      imx9_ifup_action(&priv->dev, false);

#ifdef CONFIG_NET_MCASTGROUP
      /* Now write the multicast table back */

      imx9_enet_putreg32(priv, gaurstore, IMX9_ENET_GAUR_OFFSET);
      imx9_enet_putreg32(priv, galrstore, IMX9_ENET_GALR_OFFSET);
#endif

      /* Then poll the network for new XMIT data */

      imx9_dopoll(priv);
    }
  else
    {
      /* Check for the receipt of a packet */

      if ((pending & ENET_INT_RXF) != 0)
        {
          /* A packet has been received, call imx9_receive() to handle the
           * packet.
           */

          imx9_receive(priv);
        }

      /* Check if a packet transmission has completed */

      if ((pending & ENET_INT_TXF) != 0)
        {
          /* Call imx9_txdone to handle the end of transfer */

          imx9_txdone(priv);
        }
    }

  net_unlock();

  /* Re-enable Ethernet interrupts */

  imx9_enet_putreg32(priv, priv->ints, IMX9_ENET_EIMR_OFFSET);
}

/****************************************************************************
 * Function: imx9_enet_interrupt
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

static int imx9_enet_interrupt(int irq, void *context, void *arg)
{
  register struct imx9_driver_s *priv = (struct imx9_driver_s *)arg;

  /* Mask all the interrupts */

  imx9_enet_putreg32(priv, 0, IMX9_ENET_EIMR_OFFSET);

  /* Schedule to perform the interrupt processing on the worker thread. */

  work_queue(ETHWORK, &priv->irqwork, imx9_enet_interrupt_work, priv, 0);
  return OK;
}

/****************************************************************************
 * Function: imx9_txtimeout_work
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

static void imx9_txtimeout_work(void *arg)
{
  struct imx9_driver_s *priv = (struct imx9_driver_s *)arg;

  /* Increment statistics and dump debug info */

  nerr("Resetting interface\n");

  /* Take the interface down and bring it back up.  That is the most
   * aggressive hardware reset.
   */

  NETDEV_TXTIMEOUTS(&priv->dev);
  imx9_ifdown(&priv->dev);
  imx9_ifup_action(&priv->dev, false);

  /* Then poll the network for new XMIT data */

  net_lock();
  imx9_dopoll(priv);
  net_unlock();
}

/****************************************************************************
 * Function: imx9_txtimeout_expiry
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

static void imx9_txtimeout_expiry(wdparm_t arg)
{
  struct imx9_driver_s *priv = (struct imx9_driver_s *)arg;

  /* Disable further Ethernet interrupts.  This will prevent some race
   * conditions with interrupt work.
   */

  imx9_enet_putreg32(priv, 0, IMX9_ENET_EIMR_OFFSET);
  priv->ints = 0;

  /* Schedule to perform the TX timeout processing on the worker thread,
   * canceling any pending interrupt work.
   */

  work_queue(ETHWORK, &priv->irqwork, imx9_txtimeout_work, priv, 0);
}

/****************************************************************************
 * Function: imx9_ifup_action
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

static int imx9_ifup_action(struct net_driver_s *dev, bool resetphy)
{
  struct imx9_driver_s *priv = (struct imx9_driver_s *)dev;
  uint8_t *mac = dev->d_mac.ether.ether_addr_octet;
  uint32_t ecr;
  int ret;

  ninfo("Bringing up: %u.%u.%u.%u\n",
        ip4_addr1(dev->d_ipaddr), ip4_addr2(dev->d_ipaddr),
        ip4_addr3(dev->d_ipaddr), ip4_addr4(dev->d_ipaddr));

  /* Initialize ENET buffers */

  imx9_initbuffers(priv);

  /* Configure the MII interface */

  imx9_initmii(priv);

  /* Take MAC out of reset */

  ecr = ENET_ECR_EN1588 | ENET_ECR_DBSWP;
  imx9_enet_putreg32(priv, ecr, IMX9_ENET_ECR_OFFSET);

  /* Enable store and forward mode */

  imx9_enet_putreg32(priv, ENET_TFWR_STRFWD, IMX9_ENET_TFWR_OFFSET);

  /* Set the MAC address */

  imx9_enet_putreg32(priv, (mac[0] << 24) | (mac[1] << 16) |
                      (mac[2] << 8) | mac[3], IMX9_ENET_PALR_OFFSET);
  imx9_enet_putreg32(priv, (mac[4] << 24) | (mac[5] << 16),
                      IMX9_ENET_PAUR_OFFSET);

  /* Configure the PHY */

  ret = imx9_determine_phy(priv);
  if (ret < 0)
    {
      nwarn("Unrecognized PHY\n");
    }

  ret = imx9_initphy(priv, resetphy);
  if (ret < 0)
    {
      nerr("ERROR: Failed to configure the PHY: %d\n", ret);
      return ret;
    }

  /* Set the RX buffer size */

  imx9_enet_putreg32(priv, ALIGNED_BUFSIZE, IMX9_ENET_MRBR_OFFSET);

  /* Point to the start of the circular RX buffer descriptor queue */

  imx9_enet_putreg32(priv, (uint32_t)(uintptr_t)priv->rxdesc,
                     IMX9_ENET_RDSR_OFFSET);

  /* Point to the start of the circular TX buffer descriptor queue */

  imx9_enet_putreg32(priv, (uint32_t)(uintptr_t)priv->txdesc,
                     IMX9_ENET_TDSR_OFFSET);

  /* Mask and clear all ENET interrupts */

  imx9_enet_putreg32(priv, 0, IMX9_ENET_EIMR_OFFSET);

  imx9_enet_putreg32(priv, 0xffffffff, IMX9_ENET_EIR_OFFSET);

  /* Set 1GBPS if link is set to that */

  if (priv->s_1000mbps)
    {
      ecr |= ENET_ECR_SPEED;
    }

  /* And enable the MAC */

  ecr |= ENET_ECR_ETHEREN;

  imx9_enet_putreg32(priv, ecr, IMX9_ENET_ECR_OFFSET);

  /* Enable RX and error interrupts at the controller */

  priv->ints = RX_INTERRUPTS | ERROR_INTERRUPTS;
  imx9_enet_putreg32(priv, priv->ints, IMX9_ENET_EIMR_OFFSET);

  /* Mark the interface "up" and enable interrupts */

  priv->bifup = true;
  up_enable_irq(priv->irq);

  /* Indicate that there have been empty receive buffers produced */

  imx9_enet_putreg32(priv, ENET_RDAR, IMX9_ENET_RDAR_OFFSET);

  return OK;
}

/****************************************************************************
 * Function: imx9_ifup
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

static int imx9_ifup(struct net_driver_s *dev)
{
  /* The externally available ifup action includes resetting the phy */

  return imx9_ifup_action(dev, true);
}

/****************************************************************************
 * Function: imx9_ifdown
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

static int imx9_ifdown(struct net_driver_s *dev)
{
  struct imx9_driver_s *priv = (struct imx9_driver_s *)dev;

  ninfo("Taking down: %u.%u.%u.%u\n",
        ip4_addr1(dev->d_ipaddr), ip4_addr2(dev->d_ipaddr),
        ip4_addr3(dev->d_ipaddr), ip4_addr4(dev->d_ipaddr));

  /* Cancel the TX timeout timers */

  wd_cancel(&priv->txtimeout);

  /* Flush and disable the Ethernet interrupts */

  up_disable_irq(priv->irq);

  imx9_enet_putreg32(priv, 0, IMX9_ENET_EIMR_OFFSET);
  priv->ints = 0;

  /* Put the EMAC in its reset, non-operational state.  This should be
   * a known configuration that will guarantee the imx9_ifup() always
   * successfully brings the interface back up.
   */

  imx9_reset(priv);

  /* Clear any pending interrupts */

  imx9_enet_putreg32(priv, 0xffffffff, IMX9_ENET_EIR_OFFSET);

  /* Mark the device "down" */

  priv->bifup = false;

  return OK;
}

/****************************************************************************
 * Function: imx9_txavail_work
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

static void imx9_txavail_work(void *arg)
{
  struct imx9_driver_s *priv = (struct imx9_driver_s *)arg;

  /* Ignore the notification if the interface is not yet up */

  net_lock();
  if (priv->bifup)
    {
      /* Poll the network for new XMIT data */

      imx9_dopoll(priv);
    }

  net_unlock();
}

/****************************************************************************
 * Function: imx9_txavail
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

static int imx9_txavail(struct net_driver_s *dev)
{
  struct imx9_driver_s *priv = (struct imx9_driver_s *)dev;

  /* Is our single work structure available?  It may not be if there are
   * pending interrupt actions and we will have to ignore the Tx
   * availability action.
   */

  if (work_available(&priv->pollwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      work_queue(ETHWORK, &priv->pollwork, imx9_txavail_work, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Function: imx9_calcethcrc
 *
 * Description:
 *   Function to calculate the CRC used by IMX9 to check an Ethernet frame
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
static uint32_t imx9_calcethcrc(const uint8_t *data, size_t length)
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
 * Function: imx9_enet_hash_index
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
static uint32_t imx9_enet_hash_index(const uint8_t *mac)
{
  uint32_t crc;
  uint32_t hashindex;

  ninfo("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  crc = imx9_calcethcrc(mac, 6);
  hashindex = (crc >> 26) & 0x3f;

  return hashindex;
}
#endif

/****************************************************************************
 * Function: imx9_addmac
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
static int imx9_addmac(struct net_driver_s *dev, const uint8_t *mac)
{
  uint32_t hashindex;
  uint32_t temp;
  uint32_t registeraddress;
  struct imx9_driver_s *priv = (struct imx9_driver_s *)dev;

  hashindex = imx9_enet_hash_index(mac);

  /* Add the MAC address to the hardware multicast routing table */

  if (hashindex > 31)
    {
      registeraddress = IMX9_ENET_GAUR_OFFSET;
      hashindex      -= 32;
    }
  else
    {
      registeraddress = IMX9_ENET_GALR_OFFSET;
    }

  temp  = imx9_enet_getreg32(priv, registeraddress);
  temp |= 1 << hashindex;
  imx9_enet_putreg32(priv, temp, registeraddress);

  return OK;
}
#endif

/****************************************************************************
 * Function: imx9_rmmac
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
static int imx9_rmmac(struct net_driver_s *dev, const uint8_t *mac)
{
  uint32_t hashindex;
  uint32_t temp;
  uint32_t registeraddress;
  struct imx9_driver_s *priv = (struct imx9_driver_s *)dev;

  /* Remove the MAC address from the hardware multicast routing table */

  hashindex = imx9_enet_hash_index(mac);

  if (hashindex > 31)
    {
      registeraddress = IMX9_ENET_GAUR_OFFSET;
      hashindex      -= 32;
    }
  else
    {
      registeraddress = IMX9_ENET_GALR_OFFSET;
    }

  temp  = imx9_enet_getreg32(priv, registeraddress);
  temp &= ~(1 << hashindex);
  imx9_enet_putreg32(priv, temp, registeraddress);

  return OK;
}
#endif

/****************************************************************************
 * Function: imx9_ioctl
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
static int imx9_ioctl(struct net_driver_s *dev, int cmd, unsigned long arg)
{
#ifdef CONFIG_NETDEV_PHY_IOCTL
  struct imx9_driver_s *priv = (struct imx9_driver_s *)dev;
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

              ret = imx9_phyintenable(priv);
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
          if (priv->cur_phy && priv->cur_phy->clause == 45 &&
              MII_MSR == req->reg_num)
            {
              ret = imx9_readmmd(priv, MMD1, MMD1_PMA_STATUS1,
                                  &req->val_out);
            }
          else
            {
              ret = imx9_readmii(priv, req->reg_num, &req->val_out);
            }
        }
        break;

      case SIOCSMIIREG: /* Set register in MII PHY */
        {
          struct mii_ioctl_data_s *req =
            (struct mii_ioctl_data_s *)((uintptr_t)arg);
          ret = imx9_writemii(priv, req->reg_num, req->val_in);
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
 * Function: imx9_phyintenable
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
static int imx9_phyintenable(struct imx9_driver_s *priv)
{
  uint16_t phyval;
  int ret;
  uint16_t mask;
  uint8_t  rreg;
  uint8_t  wreg;

  if (imx9_phy_is(priv, MII_YT8512_NAME))
    {
      mask = MII_YT8512_IMR_LD_EN | MII_YT8512_IMR_LU_EN;
      rreg  = MII_YT8512_ISR;
      wreg  = MII_YT8512_IMR;
    }
  else if (imx9_phy_is(priv, MII_KSZ8051_NAME) ||
           imx9_phy_is(priv, MII_KSZ8061_NAME) ||
           imx9_phy_is(priv, MII_KSZ8081_NAME) ||
           imx9_phy_is(priv, MII_DP83825I_NAME))
    {
      mask = MII_KSZ80X1_INT_LDEN | MII_KSZ80X1_INT_LUEN;
      rreg = MII_KSZ8081_INT;
      wreg = rreg;
    }
  else
    {
      return -ENOSYS;
    }

  /* Read the interrupt status register in order to clear any pending
   * interrupts
   */

  ret = imx9_readmii(priv, rreg, &phyval);
  if (ret == OK)
    {
      /* Enable link up/down interrupts */

      ret = imx9_writemii(priv, wreg, mask);
    }

  return ret;
}
#endif

/****************************************************************************
 * Function: imx9_initmii
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

static void imx9_initmii(struct imx9_driver_s *priv)
{
  /* Speed is based on the peripheral (bus) clock; hold time is 2 module
   * clock.  This hold time value may need to be increased on some platforms
   */

  imx9_enet_putreg32(priv, ENET_MSCR_HOLDTIME_2CYCLES |
                      IMX9_MII_SPEED << ENET_MSCR_MII_SPEED_SHIFT,
                      IMX9_ENET_MSCR_OFFSET);
}

/****************************************************************************
 * Function: imx9_writemii
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

static int imx9_writemii(struct imx9_driver_s *priv,
                            uint8_t regaddr, uint16_t data)
{
  int timeout;

  /* Clear the MII interrupt bit */

  imx9_enet_putreg32(priv, ENET_INT_MII, IMX9_ENET_EIR_OFFSET);

  /* Initiate the MII Management write */

  imx9_enet_putreg32(priv, data |
                      2 << ENET_MMFR_TA_SHIFT |
                      (uint32_t)regaddr << ENET_MMFR_RA_SHIFT |
                      (uint32_t)priv->phyaddr << ENET_MMFR_PA_SHIFT |
                      ENET_MMFR_OP_WRMII |
                      1 << ENET_MMFR_ST_SHIFT,
                      IMX9_ENET_MMFR_OFFSET);

  /* Wait for the transfer to complete */

  for (timeout = 0; timeout < MII_MAXPOLLS; timeout++)
    {
      if ((imx9_enet_getreg32(priv, IMX9_ENET_EIR_OFFSET) &
                               ENET_INT_MII) != 0)
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

  imx9_enet_putreg32(priv, ENET_INT_MII, IMX9_ENET_EIR_OFFSET);
  return OK;
}

/****************************************************************************
 * Function: imx9_reademii
 *
 * Description:
 *   Read a 16-bit value from a PHY register.
 *
 * Input Parameters:
 *   priv    - Reference to the private ENET driver state structure
 *   regaddr - The PHY register address
 *   data    - A pointer to the location to return the data
 *
 * Returned Value:
 *   Zero on success, a negated errno value on failure.
 *
 ****************************************************************************/

static int imx9_readmii(struct imx9_driver_s *priv,
                           uint8_t regaddr, uint16_t *data)
{
  int timeout;

  /* Clear the MII interrupt bit */

  imx9_enet_putreg32(priv, ENET_INT_MII, IMX9_ENET_EIR_OFFSET);

  /* Initiate the MII Management read */

  imx9_enet_putreg32(priv, 2 << ENET_MMFR_TA_SHIFT |
                      (uint32_t)regaddr << ENET_MMFR_RA_SHIFT |
                      (uint32_t)priv->phyaddr << ENET_MMFR_PA_SHIFT |
                      ENET_MMFR_OP_RDMII |
                      1 << ENET_MMFR_ST_SHIFT,
                      IMX9_ENET_MMFR_OFFSET);

  /* Wait for the transfer to complete */

  for (timeout = 0; timeout < MII_MAXPOLLS; timeout++)
    {
      if ((imx9_enet_getreg32(priv, IMX9_ENET_EIR_OFFSET) &
                               ENET_INT_MII) != 0)
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

  imx9_enet_putreg32(priv, ENET_INT_MII, IMX9_ENET_EIR_OFFSET);

  /* And return the MII data */

  *data = (uint16_t)(imx9_enet_getreg32(priv, IMX9_ENET_MMFR_OFFSET) &
                                    ENET_MMFR_DATA_MASK);
  return OK;
}

/****************************************************************************
 * Function: imx9_read_phy_status
 *
 * Description:
 *   Read the phy status from the current phy
 *
 * Input Parameters:
 *   priv - Reference to the private ENET driver state structure
 *
 * Returned Value:
 *   0 on success, -1 on any error
 *
 ****************************************************************************/

int imx9_read_phy_status(struct imx9_driver_s *priv)
{
  int ret;
  int retries;
  uint16_t page = 0;
  uint16_t prev_page;
  uint16_t page_reg;
  uint16_t mask;
  uint16_t status;

  if (priv->cur_phy == NULL)
    {
      /* We don't support guessing the link speed based ou our and link
       * partner's capabilities. For now, user must manually set the
       * speed and duplex if the phy is unknown
       */

      nerr("Unknown PHY, can't read link speed\n");
      return ERROR;
    }

  /* Special handling for rtl8211f, which needs to chage page */

  if (imx9_phy_is(priv, GMII_RTL8211F_NAME))
    {
      page_reg = GMII_RTL8211F_PAGSR;
      page = 0xa43;
    }

  if (page)
    {
      /* Get current page */

      ret = imx9_readmii(priv, page_reg, &prev_page);

      /* Set page */

      if (ret >= 0)
        {
          ninfo("Changing PHY page from 0x%x to 0x%x\n", prev_page, page);
          ret = imx9_writemii(priv, page_reg, page);
          if (ret < 0)
            {
              return ERROR;
            }
        }
    }

  retries = 0;
  do
    {
      status = 0xffff;
      ret = imx9_readmii(priv, priv->cur_phy->status, &status);
    }
  while ((ret < 0 || status == 0xffff) && ++retries < 3);

  if (status != 0xffff)
    {
      ninfo("%s: PHY status %x: %04x\n", priv->cur_phy->name,
            priv->cur_phy->status, status);

      /* Set the current link information */

      mask = priv->cur_phy->speed_mask;

      priv->full_duplex = (status & priv->cur_phy->duplex) != 0;
      priv->s_10mbps = (status & mask) == priv->cur_phy->mbps10;
      priv->s_100mbps = (status & mask) == priv->cur_phy->mbps100;
      priv->s_1000mbps = (status & mask) == priv->cur_phy->mbps1000;
    }

  if (page)
    {
      /* Restore original page */

      ninfo("Restoring PHY page to 0x%x\n", prev_page);
      imx9_writemii(priv, page_reg, prev_page);
    }

  return OK;
}

/****************************************************************************
 * Function: imx9_determine_phy
 *
 * Description:
 *   Uses the board.h supplied PHY list to determine which PHY
 *   is populated on this board.
 *
 * Input Parameters:
 *   priv - Reference to the private ENET driver state structure
 *
 * Returned Value:
 *   Zero on success, a -ENOENT errno value on failure.
 *
 ****************************************************************************/

static int imx9_determine_phy(struct imx9_driver_s *priv)
{
  int i;
  uint16_t phydata     = 0xffff;
  uint8_t last_phyaddr = 0;
  int retries;
  int ret;

  for (i = 0; i < priv->n_phys; i++)
    {
      priv->phyaddr = (uint8_t)priv->phy_list[i].address_lo;
      last_phyaddr = priv->phy_list[i].address_high == 0xffff ?
        priv->phyaddr :
          (uint8_t)priv->phy_list[i].address_high;

      for (; priv->phyaddr <= last_phyaddr; priv->phyaddr++)
        {
          retries = 0;
          do
            {
              nxsig_usleep(100);
              phydata = 0xffff;
              ret = imx9_readmii(priv, MII_PHYID1, &phydata);
              ninfo("phy %s addr %d received PHYID1 %x\n",
                    priv->phy_list[i].name, priv->phyaddr,
                    phydata);
            }
          while ((ret < 0 || phydata == 0xffff) && ++retries < 3);

          if (retries <= 3 && ret == 0 &&
              phydata == priv->phy_list[i].id1)
            {
              do
                {
                  nxsig_usleep(100);
                  phydata = 0xffff;
                  ret = imx9_readmii(priv, MII_PHYID2, &phydata);
                  ninfo("phy %s addr %d received PHYID2 %x\n",
                        priv->phy_list[i].name, priv->phyaddr,
                        phydata);
                }
              while ((ret < 0 || phydata == 0xffff) && ++retries < 3);
              if (retries <= 3 && ret == 0 &&
                  (phydata & 0xfff0) ==
                  (priv->phy_list[i].id2 & 0xfff0))
                {
                  priv->cur_phy = & priv->phy_list[i];
                  ninfo("Found phy %s addr %d\n",
                        priv->cur_phy->name, priv->phyaddr);
                  return OK;
                }
            }
        }
    }

  nerr("No PHY found\n");

  priv->cur_phy = NULL;
  return -ENOENT;
}

/****************************************************************************
 * Function: imx9_phy_is
 *
 * Description:
 *   Compares the name with the current selected PHY's name
 *
 * Input Parameters:
 *   priv - Reference to the private ENET driver state structure
 *   name - a pointer to comapre to.
 *
 * Returned Value:
 *   1 on match, a 0 on no match.
 *
 ****************************************************************************/

static int imx9_phy_is(struct imx9_driver_s *priv, const char *name)
{
  return priv->cur_phy && strcmp(priv->cur_phy->name, name) == 0;
}

#if 0
/****************************************************************************
 * Function: imx9_writemmd
 *
 * Description:
 *   Write a 16-bit value to a the selected MMD PHY register.
 *
 * Input Parameters:
 *   priv - Reference to the private ENET driver state structure
 *   mmd     - The Selected MMD Space
 *   regaddr - The PHY register address
 *   data    - The data to write to the PHY register
 *
 * Returned Value:
 *   Zero on success, a negated errno value on failure.
 *
 ****************************************************************************/

static int imx9_writemmd(struct imx9_driver_s *priv,
                          uint8_t mmd, uint16_t regaddr, uint16_t data)
{
  int timeout;

  /* Clear the MII interrupt bit */

  imx9_enet_putreg32(priv, ENET_INT_MII, IMX9_ENET_EIR_OFFSET);

  /* Initiate the MMD Management write  - Address Phase */

  imx9_enet_putreg32(priv,
                      0 << ENET_MMFR_ST_SHIFT |
                      ENET_MMFR_OP_WRNOTMII |
                      (uint32_t)mmd << ENET_MMFR_RA_SHIFT |
                      (uint32_t)priv->phyaddr << ENET_MMFR_PA_SHIFT |
                      2 << ENET_MMFR_TA_SHIFT |
                      regaddr,
                      IMX9_ENET_MMFR_OFFSET);

  /* Wait for the transfer to complete */

  for (timeout = 0; timeout < MII_MAXPOLLS; timeout++)
    {
      if ((imx9_enet_getreg32(priv, IMX9_ENET_EIR_OFFSET) &
                               ENET_INT_MII) != 0)
        {
          break;
        }
    }

  imx9_enet_putreg32(priv, ENET_INT_MII, IMX9_ENET_EIR_OFFSET);

  /* Check for a timeout */

  if (timeout == MII_MAXPOLLS)
    {
      return -ETIMEDOUT;
    }

  /* Initiate the MMD Management write  - Data Phase */

  imx9_enet_putreg32(priv,
                      0 << ENET_MMFR_ST_SHIFT |
                      ENET_MMFR_OP_WRMII |
                      (uint32_t)mmd << ENET_MMFR_RA_SHIFT |
                      (uint32_t)priv->phyaddr << ENET_MMFR_PA_SHIFT |
                      2 << ENET_MMFR_TA_SHIFT |
                      data,
                      IMX9_ENET_MMFR_OFFSET);

  /* Wait for the transfer to complete */

  for (timeout = 0; timeout < MII_MAXPOLLS; timeout++)
    {
      if ((imx9_enet_getreg32(priv, IMX9_ENET_EIR_OFFSET) &
                               ENET_INT_MII) != 0)
        {
          break;
        }
    }

  /* Clear the MII interrupt bit */

  imx9_enet_putreg32(priv, ENET_INT_MII, IMX9_ENET_EIR_OFFSET);

  /* Check for a timeout */

  if (timeout == MII_MAXPOLLS)
    {
      return -ETIMEDOUT;
    }

  return OK;
}
#endif

/****************************************************************************
 * Function: imx9_reademmd
 *
 * Description:
 *   Read a 16-bit value from a PHY register.
 *
 * Input Parameters:
 *   priv - Reference to the private ENET driver state structure
 *   mmd     - The Selected MMD Space
 *   regaddr - The PHY register address
 *   data    - A pointer to the location to return the data
 *
 * Returned Value:
 *   Zero on success, a negated errno value on failure.
 *
 ****************************************************************************/

static int imx9_readmmd(struct imx9_driver_s *priv,
                         uint8_t mmd, uint16_t regaddr, uint16_t *data)
{
  int timeout;

  /* Clear the MII interrupt bit */

  imx9_enet_putreg32(priv, ENET_INT_MII, IMX9_ENET_EIR_OFFSET);

  /* Initiate the MMD Management read  - Address Phase */

  imx9_enet_putreg32(priv,
                      0 << ENET_MMFR_ST_SHIFT |
                      ENET_MMFR_OP_WRNOTMII   |
                      (uint32_t)mmd << ENET_MMFR_RA_SHIFT |
                      (uint32_t)priv->phyaddr << ENET_MMFR_PA_SHIFT |
                      2 << ENET_MMFR_TA_SHIFT |
                      regaddr,
                      IMX9_ENET_MMFR_OFFSET);

  /* Wait for the transfer to complete */

  for (timeout = 0; timeout < MII_MAXPOLLS; timeout++)
    {
      if ((imx9_enet_getreg32(priv, IMX9_ENET_EIR_OFFSET) &
                               ENET_INT_MII) != 0)
        {
          break;
        }
    }

  /* Clear the MII interrupt bit */

  imx9_enet_putreg32(priv, ENET_INT_MII, IMX9_ENET_EIR_OFFSET);

  /* Check for a timeout */

  if (timeout >= MII_MAXPOLLS)
    {
      nerr("ERROR: Timed out waiting for transfer to complete\n");
      return -ETIMEDOUT;
    }

  /* Initiate the MMD Management read - Data Phase */

  imx9_enet_putreg32(priv,
                      0 << ENET_MMFR_ST_SHIFT |
                      ENET_MMFR_OP_RDNOTMII |
                      (uint32_t)mmd << ENET_MMFR_RA_SHIFT |
                      (uint32_t)priv->phyaddr << ENET_MMFR_PA_SHIFT |
                      2 << ENET_MMFR_TA_SHIFT,
                      IMX9_ENET_MMFR_OFFSET);

  /* Wait for the transfer to complete */

  for (timeout = 0; timeout < MII_MAXPOLLS; timeout++)
    {
      if ((imx9_enet_getreg32(priv, IMX9_ENET_EIR_OFFSET) &
                               ENET_INT_MII) != 0)
        {
          break;
        }
    }

  /* Clear the MII interrupt bit */

  imx9_enet_putreg32(priv, ENET_INT_MII, IMX9_ENET_EIR_OFFSET);

  /* Check for a timeout */

  if (timeout == MII_MAXPOLLS)
    {
      return -ETIMEDOUT;
    }

  /* And return the MII data */

  *data = (uint16_t)(imx9_enet_getreg32(priv, IMX9_ENET_MMFR_OFFSET) &
                                    ENET_MMFR_DATA_MASK);
  return OK;
}

int imx9_reset_phy(struct imx9_driver_s *priv)
{
  int timeout;
  int ret;
  int result;
  uint16_t mcr;

  /* Reset the PHY */

  ret = imx9_writemii(priv, MII_MCR, MII_MCR_RESET);

  if (ret < 0)
    {
      nerr("ERROR: mpfs_phywrite failed: %d\n", ret);
    }

  /* Wait for the PHY reset to complete */

  ret = -ETIMEDOUT;
  for (timeout = 0; timeout < PHY_RESET_WAIT_COUNT; timeout++)
    {
      nxsig_usleep(100);
      result = imx9_readmii(priv, MII_MCR, &mcr);
      if (result < 0)
        {
          nerr("ERROR: Failed to read the MCR register: %d\n", ret);
          ret = result;
        }
      else if ((mcr & MII_MCR_RESET) == 0)
        {
          ninfo("MII reset complete: %x\n", mcr);
          ret = OK;
          break;
        }
      else
        {
          nerr("MCR data %x\n", mcr);
        }
    }

  return ret;
}

/****************************************************************************
 * Function: imx9_phy_set_speed
 *
 * Description:
 *   Set or start to autonegotiate the link speed
 *
 * Input Parameters:
 *   priv          - Reference to the private ENET driver state structure
 *   autonegotiate - true: autonegotiate with default advertisement
 *                   false: autonegotiate, but disable other speeds
 * Returned Value:
 *   Zero (OK) returned on success; a negated errno value is returned on any
 *   failure;
 *
 * Assumptions:
 *
 ****************************************************************************/

static int imx9_phy_set_speed(struct imx9_driver_s *priv, bool autonegotiate)
{
  uint16_t advertise;
  uint16_t mcr;
  uint16_t force_mcr;
  uint16_t btcr;
  int ret;

  /* Read initial MCR and take link down */

  ret = imx9_readmii(priv, MII_MCR, &mcr);

  if (ret < 0)
    {
      nerr("ERROR: Failed to read MCR register: %d\n", ret);
      return ret;
    }

  ret = imx9_writemii(priv, MII_MCR, mcr | MII_MCR_PDOWN);

  /* If we are trying to manually set the speed, disable advertisement of the
   * other ones. In case we want to force the speed setting on the PHY,
   * set the speed in MCR and disable autonegotiation completely.
   */

  force_mcr = mcr;
  if (!autonegotiate)
    {
      /* Read the Auto_negotiation Advertisement Register defaults */

      ret = imx9_readmii(priv, MII_ADVERTISE, &advertise);

      if (ret < 0)
        {
          nerr("ERROR: Failed to read ADVERTISE register: %d\n", ret);
          return ret;
        }

      if (priv->phy_type == PHY_RGMII)
        {
          ret = imx9_readmii(priv, GMII_1000BTCR, &btcr);
          if (ret < 0)
            {
              nerr("ERROR: Failed to read GMII_1000BTCR register\n");
              return ret;
            }
        }

      if (priv->full_duplex)
        {
          advertise &= ~(MII_ADVERTISE_1000XHALF |
                         MII_ADVERTISE_100BASETXHALF |
                         MII_ADVERTISE_10BASETXHALF);
          btcr &= ~GMII_1000BTCR_1000BASETHALF;
          force_mcr |= MII_MCR_FULLDPLX;
        }
      else
        {
          advertise &= ~(MII_ADVERTISE_1000XFULL |
                         MII_ADVERTISE_100BASETXFULL |
                         MII_ADVERTISE_10BASETXFULL);
          btcr &= ~GMII_1000BTCR_1000BASETFULL;
          force_mcr &= ~MII_MCR_FULLDPLX;
        }

      if (priv->s_10mbps)
        {
          advertise &= ~(MII_ADVERTISE_100BASETXFULL |
                         MII_ADVERTISE_100BASETXHALF |
                         MII_ADVERTISE_1000XFULL |
                         MII_ADVERTISE_1000XHALF);
          btcr &= ~(GMII_1000BTCR_1000BASETHALF |
                    GMII_1000BTCR_1000BASETFULL);
          force_mcr &= ~(MII_MCR_SPEED100 | GMII_MCR_SPEED1000);
        }

      if (priv->s_100mbps)
        {
          advertise &= ~(MII_ADVERTISE_10BASETXFULL |
                         MII_ADVERTISE_10BASETXHALF |
                         MII_ADVERTISE_1000XFULL |
                         MII_ADVERTISE_1000XHALF);
          btcr &= ~(GMII_1000BTCR_1000BASETHALF |
                    GMII_1000BTCR_1000BASETFULL);
          force_mcr &= ~GMII_MCR_SPEED1000;
          force_mcr |= MII_MCR_SPEED100;
        }

      if (priv->s_1000mbps)
        {
          advertise &= ~(MII_ADVERTISE_10BASETXFULL |
                         MII_ADVERTISE_10BASETXHALF |
                         MII_ADVERTISE_100BASETXFULL |
                         MII_ADVERTISE_100BASETXHALF);
          force_mcr &= ~MII_MCR_SPEED100;
          force_mcr |= GMII_MCR_SPEED1000;
        }

      ret = imx9_writemii(priv, MII_ADVERTISE, advertise);

      if (ret < 0)
        {
          nerr("ERROR: Failed to write ADVERTISE register\n");
          return ret;
        }

      if (priv->phy_type == PHY_RGMII)
        {
          ret = imx9_writemii(priv, GMII_1000BTCR, btcr);
          if (ret < 0)
            {
              nerr("ERROR: Failed to write GMII_1000BTCR register\n");
              return ret;
            }
        }
    }

  /* Enable autonegotiation and take link back up */

  if (!priv->force_speed)
    {
      mcr |= (MII_MCR_ANENABLE | MII_MCR_ANRESTART);
    }
  else
    {
      mcr = force_mcr & (~(MII_MCR_ANENABLE | MII_MCR_ANRESTART));
    }

  ret = imx9_writemii(priv, MII_MCR, mcr);
  if (ret < 0)
    {
      nerr("ERROR: Failed to write MCR register: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Function: imx9_phy_wait_autoneg_complete
 *
 * Description:
 *   Wait for autonegotiation to complete
 *
 * Input Parameters:
 *   priv     - Reference to the private ENET driver state structure
 *
 * Returned Value:
 *   Zero (OK) returned on success; a negated errno value is returned on any
 *   failure;
 *
 * Assumptions:
 *
 ****************************************************************************/

static int imx9_phy_wait_autoneg_complete(struct imx9_driver_s *priv)
{
  int ret;
  uint16_t msr;
  int timeout;

  /* Wait for autonegotiation to complete */

  for (timeout = 0; timeout < LINK_NLOOPS; timeout++)
    {
      ret = imx9_readmii(priv, MII_MSR, &msr);
      if (ret < 0)
        {
          nerr("ERROR: Failed to read MSR register: %d\n", ret);
          return ret;
        }

      /* Check for completion of autonegotiation */

      if ((msr & MII_MSR_ANEGCOMPLETE) != 0)
        {
          /* Yes break out of the loop */

          ninfo("Autonegotiate complete, MSR %x\n", msr);
          break;
        }

      nxsig_usleep(LINK_WAITUS);
    }

  if (timeout == LINK_NLOOPS)
    {
      ninfo("Autonegotiate failed, MSR %x\n", msr);
      return -ETIMEDOUT;
    }

  return imx9_read_phy_status(priv);
}

/****************************************************************************
 * Function: imx9_initphy
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

static inline int imx9_initphy(struct imx9_driver_s *priv, bool renogphy)
{
  uint32_t rcr;
  uint32_t tcr;
  uint32_t racc;
  uint16_t phydata;
  int retries;
  int ret;
  const char *phy_name = priv->cur_phy ? priv->cur_phy->name : "Unknown";

  if (renogphy)
    {
      /* Loop until we successfully communicate
       * with the PHY. This is 'standard stuff' that should work for any PHY
       * - we are not communicating with it's 'special' registers
       * at this point.
       */

      ninfo("%s: Try phyaddr: %u\n", phy_name, priv->phyaddr);

      /* Try to read PHYID1 few times using this address */

      retries = 0;
      do
        {
          nxsig_usleep(LINK_WAITUS);

          ninfo("%s: Read PHYID1, retries=%d\n", phy_name, retries + 1);

          phydata = 0xffff;
          ret = imx9_readmii(priv, MII_PHYID1, &phydata);
        }
      while ((ret < 0 || phydata == 0xffff) && ++retries < 3);

      if (retries >= 3)
        {
          nerr("ERROR: Failed to read %s PHYID1 at address %d\n",
               phy_name, priv->phyaddr);
          return -ENOENT;
        }

      ninfo("%s: Using PHY address %u\n", phy_name, priv->phyaddr);

      /* Verify PHYID1.  Compare OUI bits 3-18 */

      ninfo("%s: PHYID1: %04x\n", phy_name, phydata);
      if (priv->cur_phy && phydata != priv->cur_phy->id1)
        {
          nerr("ERROR: PHYID1=%04x incorrect for %s.  Expected %04x\n",
               phydata, phy_name, priv->cur_phy->id1);
          return -ENXIO;
        }

      /* Read PHYID2 */

      ret = imx9_readmii(priv, MII_PHYID2, &phydata);
      if (ret < 0)
        {
          nerr("ERROR: Failed to read %s PHYID2: %d\n", phy_name, ret);
          return ret;
        }

      ninfo("%s: PHYID2: %04x\n", phy_name, phydata);

      /* Verify PHYID2:  Compare OUI bits 19-24 and the 6-bit model number
       * (ignoring the 4-bit revision number).
       */

      if (priv->cur_phy &&
          (phydata & 0xfff0) != (priv->cur_phy->id2 & 0xfff0))
        {
          nerr("ERROR: PHYID2=%04x incorrect for %s.  Expected %04x\n",
               (phydata & 0xfff0), phy_name,
               (priv->cur_phy->id2 & 0xfff0));
          return -ENXIO;
        }

      if (imx9_phy_is(priv, MII_LAN8720_NAME) ||
          imx9_phy_is(priv, MII_LAN8742A_NAME))
        {
          /* Make sure that PHY comes up in correct mode when it's reset */

          imx9_writemii(priv, MII_LAN8720_MODES,
                        MII_LAN8720_MODES_RESV | MII_LAN8720_MODES_ALL |
                        MII_LAN8720_MODES_PHYAD(priv->phyaddr));
        }

      ret = imx9_reset_phy(priv);
      if (ret < 0)
        {
          nerr("ERROR: PHY reset failed: %d\n", ret);
          return ret;
        }

      ret = imx9_phy_set_speed(priv, priv->autoneg);

      if (ret < 0)
        {
          nerr("ERROR: PHY setting speed failed: %d\n", ret);
          return ret;
        }

      /* If this is an unknown phy, we can't read the current link speed. In
       * that case, just set the default speed and duplex settings.
       */

      if (!priv->cur_phy || !priv->autoneg)
        {
          nwarn("Can't read PHY status, using default speed and duplex\n");
          imx9_phy_set_speed(priv, false);
        }
      else
        {
          ret = imx9_phy_wait_autoneg_complete(priv);
          if (ret < 0)
            {
              return ret;
            }
        }
    }

  /* Set up the transmit and receive control registers based on the
   * configuration and the auto negotiation results.
   */

  rcr = (ENET_RCR_CRCFWD | ((CONFIG_NET_ETH_PKTSIZE + CONFIG_NET_GUARDSIZE)
                            << ENET_RCR_MAX_FL_SHIFT) |
         ENET_RCR_FCE | ENET_RCR_MII_MODE);

  if (priv->phy_type == PHY_RGMII)
    {
      rcr |= ENET_RCR_RGMII_EN;
    }
  else
    {
      rcr |= ENET_RCR_RMII_MODE;
    }

  if (priv->promiscuous)
    {
      rcr |= ENET_RCR_PROM;
    }

  tcr = 0;

  imx9_enet_putreg32(priv, tcr, IMX9_ENET_TCR_OFFSET);

  /* Enable Discard Of Frames With MAC Layer Errors.
   * Enable Discard Of Frames With Wrong Protocol Checksum.
   * Bit 1: Enable discard of frames with wrong IPv4 header checksum.
   */

  racc = ENET_RACC_PRODIS | ENET_RACC_LINEDIS | ENET_RACC_IPDIS;
  imx9_enet_putreg32(priv, racc, IMX9_ENET_RACC_OFFSET);

  /* Setup half or full duplex */

  if (priv->full_duplex)
    {
      /* Full duplex */

      ninfo("%s: Full duplex\n", phy_name);
      tcr |= ENET_TCR_FDEN;
    }
  else
    {
      /* Half duplex */

      ninfo("%s: Half duplex\n", phy_name);
      rcr |= ENET_RCR_DRT;
    }

  if (priv->s_10mbps)
    {
      /* 10 Mbps */

      ninfo("%s: 10 Base-T\n", phy_name);
      rcr |= ENET_RCR_RMII_10T;
    }
  else if (priv->s_100mbps)
    {
      /* 100 Mbps */

      ninfo("%s: 100 Base-T\n", phy_name);
    }
  else if (priv->s_1000mbps)
    {
      /* 1000 Mbps */

      ninfo("%s: 1000 Base-T\n", phy_name);
    }
  else
    {
      /* This might happen if Autonegotiation did not complete(?) */

      nerr("ERROR: No 10-, 100-, or 1000-BaseT reported: PHY STATUS=%04x\n",
           phydata);
      return -EIO;
    }

  imx9_enet_putreg32(priv, rcr, IMX9_ENET_RCR_OFFSET);
  imx9_enet_putreg32(priv, tcr, IMX9_ENET_TCR_OFFSET);
  return OK;
}

/****************************************************************************
 * Function: imx9_initbuffers
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

static void imx9_initbuffers(struct imx9_driver_s *priv)
{
  uintptr_t addr;
  int i;

  /* Get the beginning of the first aligned buffer */

  addr = priv->buffer_pool;

  /* Then fill in the TX descriptors */

  memset(priv->txdesc, 0, IMX9_ENET_NTXBUFFERS * sizeof(priv->txdesc[0]));

  for (i = 0; i < IMX9_ENET_NTXBUFFERS; i++)
    {
      priv->txdesc[i].d1.data = addr;
      priv->txdesc[i].d1.bdu  = TXDESC_BDU;

      priv->txdesc[i].d2.bdu  = TXDESC_BDU;
      addr += ALIGNED_BUFSIZE;
    }

  /* Then fill in the RX descriptors */

  memset(priv->rxdesc, 0, IMX9_ENET_NRXBUFFERS * sizeof(priv->rxdesc[0]));

  for (i = 0; i < IMX9_ENET_NRXBUFFERS; i++)
    {
      priv->rxdesc[i].status1 = RXDESC_E;
      priv->rxdesc[i].data    = addr;
      priv->rxdesc[i].status2 = RXDESC_INT;
      addr += ALIGNED_BUFSIZE;
    }

  /* Set the wrap bit in the last descriptors to form a ring */

  priv->txdesc[IMX9_ENET_NTXBUFFERS - 1].d2.status1 |= TXDESC_W;
  priv->rxdesc[IMX9_ENET_NRXBUFFERS - 1].status1 |= RXDESC_W;

  UP_DSB();

  up_clean_dcache((uintptr_t)priv->txdesc,
                  (uintptr_t)priv->txdesc +
                  IMX9_ENET_NTXBUFFERS * sizeof(priv->txdesc[0]));
  up_clean_dcache((uintptr_t)priv->rxdesc,
                  (uintptr_t)priv->rxdesc +
                  IMX9_ENET_NRXBUFFERS * sizeof(priv->rxdesc[0]));

  /* We start with RX descriptor 0 and with no TX descriptors in use */

  priv->txhead = 0;
  priv->rxtail = 0;
}

/****************************************************************************
 * Function: imx9_reset
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

static void imx9_reset(struct imx9_driver_s *priv)
{
  /* Set the reset bit and wait for the enable to clear */

  imx9_enet_putreg32(priv, ENET_ECR_RESET, IMX9_ENET_ECR_OFFSET);

  while (imx9_enet_getreg32(priv, IMX9_ENET_ECR_OFFSET) & ENET_ECR_ETHEREN)
    {
      asm volatile ("nop");
    }
}

/****************************************************************************
 * Function: imx9_enet_mux_io
 *
 * Description:
 *   Mux all the IO pins
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void imx9_enet_mux_io(void)
{
#ifdef CONFIG_IMX9_ENET1
  imx9_iomux_configure(MUX_ENET1_MDIO);
  imx9_iomux_configure(MUX_ENET1_MDC);

  imx9_iomux_configure(MUX_ENET1_RX_DATA00);
  imx9_iomux_configure(MUX_ENET1_RX_DATA01);

  imx9_iomux_configure(MUX_ENET1_TX_DATA00);
  imx9_iomux_configure(MUX_ENET1_TX_DATA01);

#  if defined(CONFIG_IMX9_ENET1_RGMII)
  imx9_iomux_configure(MUX_ENET1_RX_DATA02);
  imx9_iomux_configure(MUX_ENET1_RX_DATA03);
  imx9_iomux_configure(MUX_ENET1_TX_DATA02);
  imx9_iomux_configure(MUX_ENET1_TX_DATA03);
  imx9_iomux_configure(MUX_ENET1_RXC);
  imx9_iomux_configure(MUX_ENET1_TX_CTL);
  imx9_iomux_configure(MUX_ENET1_RX_CTL);
#  else /* RMII */
  imx9_iomux_configure(MUX_ENET1_TX_EN);
  imx9_iomux_configure(MUX_ENET1_REF_CLK);
  imx9_iomux_configure(MUX_ENET1_CRS_DV);
#  endif

#  ifdef MUX_ENET1_RX_ER
  imx9_iomux_configure(MUX_ENET1_RX_ER);
#  endif
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: imx9_netinitialize
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

int imx9_netinitialize(int intf)
{
  struct imx9_driver_s *priv;
  uint32_t uidl;
  uint32_t uidml;
  uint8_t *mac;
  int ret;

  /* Get the interface structure associated with this interface number. */

  priv = &g_enet[intf];

  /* Disable the ENET clock */

  imx9_ccm_gate_on(priv->clk_gate, false);

  /* Enet ref to 125 MHz */

  imx9_ccm_configure_root_clock(CCM_CR_ENETREF, SYS_PLL1PFD0DIV2, 2);

  /* Enet timer 1 to 125MHz */

  imx9_ccm_configure_root_clock(CCM_CR_ENETTIMER1, SYS_PLL1PFD0DIV2, 2);

  /* Enet ref clock to 25 MHz */

  imx9_ccm_configure_root_clock(CCM_CR_ENETREFPHY, SYS_PLL1PFD0DIV2, 20);

  /* Enet TX / ref clock direction */

#ifdef CONFIG_IMX9_ENET1_TX_CLOCK_IS_INPUT
  modifyreg32(IMX9_WAKUPMIX_ENET_CLK_SEL, WAKEUPMIX_ENET1_TX_CLK_SEL, 0);
#else
  modifyreg32(IMX9_WAKUPMIX_ENET_CLK_SEL, 0, WAKEUPMIX_ENET1_TX_CLK_SEL);
#endif

  /* Enable the ENET clock */

  imx9_ccm_gate_on(priv->clk_gate, true);

  /* Attach the Ethernet interrupt handler */

  if (irq_attach(priv->irq, imx9_enet_interrupt, priv))
    {
      /* We could not attach the ISR to the interrupt */

      nerr("ERROR: Failed to attach EMACTX IRQ\n");
      return -EAGAIN;
    }

  /* TODO: 1588 features */

  /* Attach the Ethernet MAC IEEE 1588 timer interrupt handler */

#if 0
  if (irq_attach(IMX9_IRQ_ENET_1588, imx9_enet_interrupt, priv))
    {
      /* We could not attach the ISR to the interrupt */

      nerr("ERROR: Failed to attach EMACTMR IRQ\n");
      return -EAGAIN;
    }
#endif

#ifdef CONFIG_IMX9_ENET_USE_OTP_MAC

  /* Boards like the imx93-evk have a unique (official)
   * MAC address stored in OTP.
   */

  uidl   = getreg32(IMX9_OCOTP_BASE + priv->otp_mac_off);
  uidml  = getreg32(IMX9_OCOTP_BASE + priv->otp_mac_off + 4);
  mac    = priv->dev.d_mac.ether.ether_addr_octet;

  mac[0] = (uidml & 0x0000ff00) >> 8;
  mac[1] = (uidml & 0x000000ff) >> 0;
  mac[2] = (uidl & 0xff000000) >> 24;
  mac[3] = (uidl & 0x00ff0000) >> 16;
  mac[4] = (uidl & 0x0000ff00) >> 8;
  mac[5] = (uidl & 0x000000ff) >> 0;

#else

  /* Determine a semi-unique MAC address from MCU UID
   * We use UID Low and Mid Low registers to get 64 bits, from which we keep
   * 40 bits.  We then force locally administered bits in mac[0] based on
   * interface number (0x2,0x6,0xa,0xe for if 0,1,2,3)
   */

  uidl   = getreg32(IMX9_OCOTP_BASE + IMX93_OCOTP_UID_OFFSET);
  uidml  = getreg32(IMX9_OCOTP_BASE + IMX93_OCOTP_UID_OFFSET + 4);
  mac    = priv->dev.d_mac.ether.ether_addr_octet;

  mac[0] = (0x2 | (intf << 2));
  mac[1] = (uidml & 0x000000ff);
  mac[2] = (uidl &  0xff000000) >> 24;
  mac[3] = (uidl &  0x00ff0000) >> 16;
  mac[4] = (uidl &  0x0000ff00) >> 8;
  mac[5] = (uidl &  0x000000ff);

#endif

#ifdef CONFIG_IMX9_ENET_PHYINIT
  /* Perform any necessary, one-time, board-specific PHY initialization */

  ret = imx9_phy_boardinitialize(intf);
  if (ret < 0)
    {
      nerr("ERROR: Failed to initialize the PHY: %d\n", ret);
      return ret;
    }
#endif

  /* Put the interface in the down state.  This usually amounts to resetting
   * the device and/or calling imx9_ifdown().
   */

  imx9_ifdown(&priv->dev);

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

#if !defined(CONFIG_NETDEV_LATEINIT)
void arm64_netinitialize(void)
{
  int i;

  /* Configure all ENET/MII pins */

  imx9_enet_mux_io();

  /* Initialize all IFs */

  for (i = 0; i < nitems(g_enet); i++)
    {
      imx9_netinitialize(i);
    }
}
#endif

#endif /* CONFIG_IMX9_ENET */
