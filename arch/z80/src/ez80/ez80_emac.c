/****************************************************************************
 * arch/z80/src/ez80/ez80_emac.c
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
 * References:
 *   eZ80F91 MCU Product Specification, PS019214-0808, Zilig, Inc., 2008.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#if defined(CONFIG_NET) && defined(CONFIG_EZ80_EMAC)

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>
#include <debug.h>
#include <errno.h>
#include <assert.h>

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

#include <arch/io.h>

#include "chip.h"
#include "z80_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* If processing is not done at the interrupt level, then work queue support
 * is required.
 */

#if !defined(CONFIG_SCHED_WORKQUEUE)
#  error Work queue support is required in this configuration (CONFIG_SCHED_WORKQUEUE)
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

/* The eZ80F92 has 16Kb of SRAM.  The base address of the SRAM is setup by
 * the eZ80 start-up logic by setting the RAM_ADDR_U register to the upper
 * 8 bits of the 24-bit address.  The EMAC RAM is at an offset of 0x00c000
 * into that region.
 */

extern uintptr_t _RAM_ADDR_U_INIT_PARAM;
#define ETH_RAMADDR (((uintptr_t)&_RAM_ADDR_U_INIT_PARAM << 16) + 0x00c000)

#if CONFIG_NET_ETH_PKTSIZE > 1518
#  error "MAXF size too big for this device"
#endif

/* The memory region shared between this driver and the EMAC hardware
 * is subdivided into a Transmit buffer and the Receive buffer.
 * The Transmit and Receive buffers are subdivided into packet buffers
 * of 32, 64, 128, or 256 bytes in size.
 */

#ifndef CONFIG_EZ80_PKTBUFSIZE
#  define CONFIG_EZ80_PKTBUFSIZE 64
#endif

#ifndef CONFIG_EZ80_NTXPKTBUFS
#  define CONFIG_EZ80_NTXPKTBUFS 64
#endif

#ifndef CONFIG_EZ80_NRXPKTBUFS
#  define CONFIG_EZ80_NRXPKTBUFS 64
#endif

#define EMAC_TXBUFSIZE (CONFIG_EZ80_PKTBUFSIZE * CONFIG_EZ80_NTXPKTBUFS)
#if EMAC_TXBUFSIZE > 8192
#  error "Too many TX buffers"
#endif

#define EMAC_RXBUFSIZE (CONFIG_EZ80_PKTBUFSIZE * CONFIG_EZ80_NRXPKTBUFS)
#if EMAC_RXBUFSIZE > 8192
#  error "Too many TX buffers"
#endif

#define EMAC_TOTAL_BUFSIZE (EMAC_TXBUFSIZE + EMAC_RXBUFSIZE)
#if EMAC_TOTAL_BUFSIZE > 8192
#  error "Too many TX+RX buffers"
#elif EMAC_TOTAL_BUFSIZE < 8192
#  error "Unused buffers!"
#endif

#if CONFIG_EZ80_PKTBUFSIZE == 256
#  define EMAC_BUFSZ          EMAC_BUFSZ_256b
#  define EMAC_PKTBUF_SHIFT   8
#elif CONFIG_EZ80_PKTBUFSIZE == 128
#  define EMAC_BUFSZ          EMAC_BUFSZ_128b
#  define EMAC_PKTBUF_SHIFT   7
#elif CONFIG_EZ80_PKTBUFSIZE == 64
#  define EMAC_BUFSZ          EMAC_BUFSZ_64b
#  define EMAC_PKTBUF_SHIFT   6
#elif CONFIG_EZ80_PKTBUFSIZE == 32
#  define EMAC_BUFSZ          EMAC_BUFSZ_32b
#  define EMAC_PKTBUF_SHIFT   5
#else
#  error "Unsupported CONFIG_EZ80_PKTBUFSIZE value"
#endif

#define EMAC_PKTBUF_MASK     (CONFIG_EZ80_PKTBUFSIZE - 1)
#define EMAC_PKTBUF_ALIGN(a) (((a) + EMAC_PKTBUF_MASK - 1) & ~EMAC_PKTBUF_MASK)

/* Am79c874 PHY configuration */

#define EZ80_EMAC_AUTONEG    0
#define EZ80_EMAC_100BFD     1
#define EZ80_EMAC_100BHD     2
#define EZ80_EMAC_10BFD      3
#define EZ80_EMAC_10BHD      4

#ifndef CONFIG_EZ80_PHYCONFIG
#  define CONFIG_EZ80_PHYCONFIG EZ80_EMAC_10BFD
#endif

/* Select the fastest MDC clock that does not exceed 25MHz.  The MDC
 * clock derives from the SCLK divided by 4, 6, 8, 10, 14, 20, or 28.
 */

#ifndef CONFIG_EZ80_MDCDIV
#  ifdef CONFIG_ETH0_PHY_AM79C874
#    define CONFIG_EZ80_MDCDIV EMAC_MDC_DIV20
#  else
#    define CONFIG_EZ80_MDCDIV EMAC_MDC_DIV4
#  endif
#endif

/* Select the Tx poll timer.  If not specified, a 10MS timer is set */

#ifndef CONFIG_EZ80_TXPOLLTIMERMS
#  define CONFIG_EZ80_TXPOLLTIMERMS 10
#endif

/* Misc. timing values and settings */

#define EMAC_MXPOLLLOOPS        1000
#define EMAC_CRCPOLY2           0xedb88320;

/* Default register settings */

#define EMAC_TPTV              0x1000      /* TPTV: Default 0x1000 slot time (1slot:512bit) */
#define EMAC_IPGT              0x12        /* IPGT: Back-to-back IPG default value */
#define EMAC_IPGR1             0x0c        /* IPGR1: Non-back-to-back IPG default value */
#define EMAC_IPGR2             0x12        /* IPGR2: Non-back-to-back IPG default value */
#define EMAC_MAXF              0x0600      /* Maximum packet length value (reset value) */
#define EMAC_LCOL              0x37        /* CFG2: Late collision window default value */
#define EMAC_RETRY             0x0f        /* CFG3: Maximum number of retry default value */

/* Poll timer setting.  The transmit poll timer is set in increments of
 * SYSCLCK / 256.  NOTE: The system clock frequency is defined in the
 * board.h file.
 */

#define EMAC_PTMR              (CONFIG_EZ80_TXPOLLTIMERMS * (ez80_systemclock / 1000) >> 8)

  /* EMAC system interrupts :
   *
   * EMAC_ISTAT_TXFSMERR - Bit 7: 1=Transmit state machine error interrupt
   *   A Transmit State Machine Error should never occur. However, if this
   *   bit is set, the entire transmitter module must be reset.
   * EMAC_ISTAT_MGTDONE  - Bit 6: 1=MII Mgmt done interrupt
   *   This bit is set when communicating to the PHY over the MII during
   *   a Read or Write operation.
   * EMAC_ISTAT_RXOVR    - Bit 2: 1=Receive overrun interrupt
   *   If this bit is set, all incoming packets are ignored until
   *   this bit is cleared by software.
   */

#define EMAC_ISTAT_SYSEVENTS \
  (EMAC_ISTAT_TXFSMERR | EMAC_ISTAT_MGTDONE | EMAC_ISTAT_RXOVR)

  /* EMAC Tx interrupts:
   *
   * EMAC_ISTAT_TXDONE   - Bit 0: 1=Transmit done interrupt
   *   Denotes when packet transmission is complete.
   * EMAC_ISTAT_TXCF     - Bit 1: 1=Transmit control frame interrupt
   *   Denotes when control frame transmission is complete.
   */

#define EMAC_ISTAT_TXEVENTS    (EMAC_ISTAT_TXDONE | EMAC_ISTAT_TXCF)

  /* EMAC Rx interrupts:
   *
   * EMAC_ISTAT_RXDONE   - Bit 3: 1=Receive done interrupt
   *   Denotes when packet reception is complete.
   * EMAC_ISTAT_RXPCF    - Bit 4: 1=Receive pause control frame interrupt
   *   Denotes when pause control frame reception is complete.
   * EMAC_ISTAT_RXCF     - Bit 5: 1=Receive control frame interrupt
   *   Denotes when control frame reception is complete.
   */

#define EMAC_ISTAT_RXEVENTS \
  (EMAC_ISTAT_RXDONE | EMAC_ISTAT_RXPCF | EMAC_ISTAT_RXCF)
#define EMAC_EIN_HANDLED \
  (EMAC_ISTAT_RXEVENTS | EMAC_ISTAT_TXEVENTS | EMAC_ISTAT_SYSEVENTS)

/* TX timeout = 1 minute */

#define EMAC_TXTIMEOUT         (60*CLK_TCK)

/* This is a helper pointer for accessing the contents of Ethernet header */

#define ETHBUF ((FAR struct eth_hdr_s *)priv->dev.d_buf)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* EMAC statistics (debug only) */

#if defined(CONFIG_DEBUG_FEATURES) && defined(CONFIG_DEBUG_NET)
struct ez80mac_statistics_s
{
  uint32_t rx_int;         /* Number of Rx interrupts received */
  uint32_t rx_packets;     /* Number of packets received (sum of the following): */
#ifdef CONFIG_NET_IPv4
  uint32_t rx_ip;          /*   Number of Rx IPv4 packets received */
#endif
#ifdef CONFIG_NET_IPv6
  uint32_t rx_ipv6;        /*   Number of Rx IPv6 packets received */
#endif
  uint32_t rx_arp;         /*   Number of Rx ARP packets received */
  uint32_t rx_dropped;     /*   Number of dropped, unsupported Rx packets */
  uint32_t rx_nok;         /*   Number of Rx packets received without OK bit */
  uint32_t rx_errors;      /* Number of Rx errors (rx_overerrors + rx_nok) */
  uint32_t rx_ovrerrors;   /*   Number of FIFO overrun errors */
  uint32_t tx_int;         /* Number of Tx interrupts received */
  uint32_t tx_packets;     /* Number of Tx descriptors queued */
  uint32_t tx_errors;      /* Number of Tx errors (sum of the following) */
  uint32_t tx_abterrors;   /*   Number of aborted Tx descriptors */
  uint32_t tx_fsmerrors;   /*   Number of Tx state machine errors */
  uint32_t tx_timeouts;    /*   Number of Tx timeout errors */
  uint32_t sys_int;        /* Number of system interrupts received */
};
#  define EMAC_STAT(priv,name)   priv->stat.name++
#else
#  define EMAC_STAT(priv,name)
#endif

/* Private driver data. The ez80emac_driver_s encapsulates all state
 * information for a single hardware interface
 */

struct ez80emac_driver_s
{
  /* Tx buffer management
   *
   * txstart:   The beginning of the Rx descriptor list (and also the
   *            beginning of Tx/Rx memory).
   * txhead:    Points to the oldest Tx descriptor queued for output (but for
   *            which output has not yet completed.  Initialized to NULL; set
   *            by ez80emac_transmit() when Tx is started and by
   *            ez80emac_txinterrupt() when Tx processing completes.  txhead
   *            == NULL is also a sure indication that there is no Tx in
   *            progress.
   * txnext:    Points to the next free Tx descriptor. Initialized to
   *            txstart; set when ez80emac_transmit() adds the descriptor;
   *            reset to txstart when the last Tx packet is sent.
   */

  FAR struct ez80emac_desc_s *txstart;
  FAR struct ez80emac_desc_s *txhead;
  FAR struct ez80emac_desc_s *txnext;

  /* Rx buffer management
   *
   * rxstart:   The beginning of the Rx descriptor list (and also the end of
   *            Tx buffer + 1).
   * rxnext:    The next of Rx descriptor available for receipt of a packet.
   *            Initialized to rxstart; rxnext is incremented by
   *            rmac_rxinterrupt() as part of Rx interrupt processing. rxnext
   *            wraps back to rxstart when rxnext exceeds rxendp1.
   * rxendp1:   The end of the Rx descriptor list + 1.
   */

  FAR struct ez80emac_desc_s *rxstart;
  FAR struct ez80emac_desc_s *rxnext;
  FAR struct ez80emac_desc_s *rxendp1;

  bool    bifup;            /* true:ifup false:ifdown */
  bool    blinkok;          /* true:successful MII autonegotiation */
  bool    bfullduplex;      /* true:full duplex */
  bool    b100mbs;          /* true:100Mbp */

  struct wdog_s txtimeout;  /* TX timeout timer */

  struct work_s txwork;     /* For deferring Tx-related work to the work queue */
  struct work_s rxwork;     /* For deferring Rx-related work to the work queue */
  struct work_s syswork;    /* For deferring system work to the work queue */

#if defined(CONFIG_DEBUG_FEATURES) && defined(CONFIG_DEBUG_NET)
  struct ez80mac_statistics_s stat;
#endif

  /* This holds the information visible to the NuttX network */

  struct net_driver_s dev;  /* Interface understood by the network */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* A single packet buffer is used */

static uint8_t g_pktbuf[MAX_NETDEV_PKTSIZE + CONFIG_NET_GUARDSIZE];

/* There is only a single instance of driver private data (because there is
 * only one EMAC interface.
 */

static struct ez80emac_driver_s g_emac;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* MII logic */

static void ez80emac_waitmiibusy(void);
static void ez80emac_miiwrite(FAR struct ez80emac_driver_s *priv,
              uint8_t offset, uint16_t value);
static uint16_t ez80emac_miiread(FAR struct ez80emac_driver_s *priv,
              uint32_t offset);
static bool ez80emac_miipoll(FAR struct ez80emac_driver_s *priv,
              uint32_t offset, uint16_t bits, bool bclear);
static int  ez80emac_miiconfigure(FAR struct ez80emac_driver_s *priv);

/* Multi-cast filtering */

#ifdef CONFIG_EZ80_MCFILTER
static void ez80emac_machash(FAR uint8_t *mac, FAR int *ndx, FAR int *bitno);
#endif

/* TX/RX logic */

static int  ez80emac_transmit(FAR struct ez80emac_driver_s *priv);
static int  ez80emac_txpoll(FAR struct net_driver_s *dev);

static inline FAR struct ez80emac_desc_s *ez80emac_rwp(void);
static inline FAR struct ez80emac_desc_s *ez80emac_rrp(void);
static int  ez80emac_receive(FAR struct ez80emac_driver_s *priv);

/* Interrupt handling */

static void ez80emac_txinterrupt_work(FAR void *arg);
static int  ez80emac_txinterrupt(int irq, FAR void *context,
              FAR void *arg);

static void ez80emac_rxinterrupt_work(FAR void *arg);
static int  ez80emac_rxinterrupt(int irq, FAR void *context,
              FAR void *arg);

static void ez80emac_sysinterrupt_work(FAR void *arg);
static int  ez80emac_sysinterrupt(int irq, FAR void *context,
              FAR void *arg);

/* Watchdog timer expirations */

static void ez80emac_txtimeout_work(FAR void *arg);
static void ez80emac_txtimeout_expiry(wdparm_t arg);

/* NuttX callback functions */

static int  ez80emac_ifup(FAR struct net_driver_s *dev);
static int  ez80emac_ifdown(FAR struct net_driver_s *dev);

static void ez80emac_txavail_work(FAR void *arg);
static int  ez80emac_txavail(FAR struct net_driver_s *dev);

#ifdef CONFIG_NET_MCASTGROUP
static int ez80emac_addmac(FAR struct net_driver_s *dev,
             FAR const uint8_t *mac);
static int ez80emac_rmmac(FAR struct net_driver_s *dev,
             FAR const uint8_t *mac);
#endif

/* Initialization */

static int  ez80_emacinitialize(void);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: ez80emac_waitmiibusy
 *
 * Description:
 *   Wait for the MII to become available.
 *
 * Input Parameters:
 *   priv   - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void ez80emac_waitmiibusy(void)
{
  /* Wait for any preceding MII management operation to complete */

  while ((inp(EZ80_EMAC_MIISTAT) & EMAC_MIISTAT_BUSY) != 0);
}

/****************************************************************************
 * Function: ez80emac_miiwrite
 *
 * Description:
 *   Write a signel MII register
 *
 * Input Parameters:
 *   priv   - Reference to the driver state structure
 *   offset - Register offset in PMD
 *   value  - Value to write
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void ez80emac_miiwrite(FAR struct ez80emac_driver_s *priv,
                              uint8_t offset, uint16_t value)
{
  uint8_t regval;

  /* Wait for any preceding MII management operation to complete */

  ez80emac_waitmiibusy();

  /* Set up PHY addressing */

  outp(EZ80_EMAC_FIAD, CONFIG_EZ80_FIAD & EMAC_FIAD_MASK);
  outp(EZ80_EMAC_RGAD, offset & EMAC_RGAD_MASK);

  /* Write the control data */

  outp(EZ80_EMAC_CTLD_H, value >> 8);
  outp(EZ80_EMAC_CTLD_L, value & 0xff);

  /* Send control data to  PHY */

  regval  = inp(EZ80_EMAC_MIIMGT);
  regval |= EMAC_MIIMGMT_LCTLD;
  outp(EZ80_EMAC_MIIMGT, regval);
}

/****************************************************************************
 * Function: ez80emac_miiread
 *
 * Description:
 *   Read a single MII register
 *
 * Input Parameters:
 *   priv   - Reference to the driver state structure
 *   offset - Register offset in PMD
 *
 * Returned Value:
 *   Value read from the register
 *
 ****************************************************************************/

static uint16_t ez80emac_miiread(FAR struct ez80emac_driver_s *priv,
                                 uint32_t offset)
{
  uint8_t regval;

  /* Wait for any preceding MII management operation to complete */

  ez80emac_waitmiibusy();

  /* Set up PHY addressing */

  outp(EZ80_EMAC_FIAD, CONFIG_EZ80_FIAD & EMAC_FIAD_MASK);
  outp(EZ80_EMAC_RGAD, offset & EMAC_RGAD_MASK);

  /* Read status from PHY */

  regval  = inp(EZ80_EMAC_MIIMGT);
  regval |= EMAC_MIIMGMT_RSTAT;
  outp(EZ80_EMAC_MIIMGT, regval);

  /* Wait for MII management operation to complete */

  ez80emac_waitmiibusy();
  return ((uint16_t)inp(EZ80_EMAC_PRSD_H) << 8 | inp(EZ80_EMAC_PRSD_L));
}

/****************************************************************************
 * Function: ez80emac_miipoll
 *
 * Description:
 *   Read an MII register until the bit has the specified polarity (or until
 *   the maximum number of retries occurs
 *
 * Input Parameters:
 *   priv   - Reference to the driver state structure
 *   offset - Register offset in PMD
 *   bits   - Selects set of bits to wait for
 *   bclear - true:Return true when all bits in 'bits' are 0
 *            false:Return true when one or more bits in 'bits' are 1
 *
 * Returned Value:
 *   true:Bit has requested polarity; false: EMAC_MXPOLLLOOPS exceeded
 *
 ****************************************************************************/

static bool ez80emac_miipoll(FAR struct ez80emac_driver_s *priv,
                             uint32_t offset, uint16_t bits, bool bclear)
{
  uint16_t value;
  int i;

  for (i = 0; i < EMAC_MXPOLLLOOPS; i++)
    {
      value = ez80emac_miiread(priv, offset);
      if (bclear)
        {
          if ((value & bits) == 0)
            {
              return true;
            }
        }
      else
        {
          if ((value & bits) != 0)
            {
              return true;
            }
        }

      up_mdelay(10);
    }

  return false;
}

/****************************************************************************
 * Function: ez80emac_miiconfigure
 *
 * Description:
 *   Dump all MII registers
 *
 * Input Parameters:
 *   priv   - Reference to the driver state structure
 *   offset - Register offset in PMD
 *   bits   - Selects set of bits to wait for
 *   bclear - true:Return true when all bits in 'bits' are 0
 *            false:Return true when one or more bits in 'bits' are 1
 *
 * Returned Value:
 *   true:Bit has requested polarity; false: EMAC_MXPOLLLOOPS exceeded
 *
 ****************************************************************************/

#ifdef CONFIG_ETH0_PHY_AM79C874
static int ez80emac_miiconfigure(FAR struct ez80emac_driver_s *priv)
{
  uint16_t phyval;
  bool bauto;
  int ret = OK;
  int i;

  /* Verify that the detect PHY is an AMD Am87c874 as expected */

#ifdef CONFIG_DEBUG_FEATURES /* Parameter checking only done when DEBUG is enabled */
  phyval = ez80emac_miiread(priv, MII_PHYID1);
  if (phyval != MII_PHYID1_AM79C874)
    {
      nerr("ERROR: Not an Am79c874 PHY: PHY1=%04x vs %04x\n",
           phyval, MII_PHYID1_AM79C874);
      ret = -ENODEV;
      goto dumpregs;
    }

  phyval = ez80emac_miiread(priv, MII_PHYID2);
  if (phyval != MII_PHYID2_AM79C874)
    {
      nerr("ERROR: Not an Am79c874 PHY: PHY2=%04x vs %04x\n",
           phyval, MII_PHYID2_AM79C874);
      ret = -ENODEV;
      goto dumpregs;
    }
#endif

  /* Check if the PHY can do auto-negotiation */

  phyval = ez80emac_miiread(priv, MII_MSR);
  if (phyval & MII_MSR_ANEGABLE)
    {
      phyval = MII_MCR_ANRESTART | MII_MCR_ANENABLE;
      bauto  = true;
    }
  else
    {
      phyval = 0;
      bauto = false;

      /* PADEN - EMAC pads all short frames by adding zeroes to the end of
       *         the data field. This bit is used in conjunction with ADPADN
       *         and VLPAD.
       * CRCEN - Append CRC to every frame regardless of padding options.
       */

      outp(EZ80_EMAC_CFG1, EMAC_CFG1_PADEN | EMAC_CFG1_CRCEN);
    }

#if CONFIG_EZ80_PHYCONFIG == EZ80_EMAC_AUTONEG
  /* Set the configured link capabilities */

  ninfo("Configure autonegotiation\n");
  if (bauto)
    {
      ez80emac_miiwrite(priv, MII_ADVERTISE,
                        MII_ADVERTISE_100BASETXFULL |
                        MII_ADVERTISE_100BASETXHALF |
                        MII_ADVERTISE_10BASETXFULL |
                        MII_ADVERTISE_10BASETXHALF |
                        MII_ADVERTISE_CSMA);
    }
  else
    {
      nerr("ERROR: Am79c784 is not capable of autonegotiation\n");
    }

#elif CONFIG_EZ80_PHYCONFIG == EZ80_EMAC_100BFD

  ninfo("100BASETX full duplex\n");
  phyval |= MII_MCR_SPEED100 | MII_MCR_FULLDPLX;
  ez80emac_miiwrite(priv, MII_ADVERTISE,
                    MII_ADVERTISE_100BASETXFULL |
                    MII_ADVERTISE_100BASETXHALF |
                    MII_ADVERTISE_10BASETXFULL |
                    MII_ADVERTISE_10BASETXHALF |
                    MII_ADVERTISE_CSMA);

#elif CONFIG_EZ80_PHYCONFIG == EZ80_EMAC_100BHD

  ninfo("100BASETX half duplex\n");
  phyval |= MII_MCR_SPEED100;
  ez80emac_miiwrite(priv, MII_ADVERTISE,
                    MII_ADVERTISE_100BASETXHALF |
                    MII_ADVERTISE_10BASETXFULL |
                    MII_ADVERTISE_10BASETXHALF |
                    MII_ADVERTISE_CSMA);

#elif CONFIG_EZ80_PHYCONFIG == EZ80_EMAC_10BFD

  ninfo("10BASETX full duplex\n");
  phyval |= MII_MCR_FULLDPLX;
  ez80emac_miiwrite(priv, MII_ADVERTISE,
                    MII_ADVERTISE_10BASETXFULL |
                    MII_ADVERTISE_10BASETXHALF |
                    MII_ADVERTISE_CSMA);

#elif CONFIG_EZ80_PHYCONFIG == EZ80_EMAC_10BHD

  ninfo("10BASETX half duplex\n");
  ez80emac_miiwrite(priv, MII_ADVERTISE,
                    MII_ADVERTISE_10BASETXHALF |
                    MII_ADVERTISE_CSMA);

#else
#  error "No recognized value of CONFIG_EZ80_PHYCONFIG"
#endif

  ez80emac_miiwrite(priv, MII_MCR, phyval);

  /* Wait for a link to be established */

  for (i = 0; i < 50; i++)
    {
      phyval = ez80emac_miiread(priv, MII_MSR);
      if ((phyval & (MII_MSR_ANEGCOMPLETE | MII_MSR_LINKSTATUS)) != 0)
        {
           break;
        }

      up_mdelay(10);
    }

  if ((phyval & MII_MSR_LINKSTATUS) == 0)
    {
      nerr("ERROR: Failed to establish link\n");
      ret = -ETIMEDOUT;
    }
  else
    {
      /* Read the Am79c874 diagnostics register for link settings */

      phyval = ez80emac_miiread(priv, MII_AM79C874_DIAGNOSTIC);
      if (phyval & AM79C874_DIAG_FULLDPLX)
        {
          outp(EZ80_EMAC_CFG1, EMAC_CFG1_PADEN | EMAC_CFG1_CRCEN |
                               EMAC_CFG1_FULLHD);
        }
      else
        {
          outp(EZ80_EMAC_CFG1, EMAC_CFG1_PADEN | EMAC_CFG1_CRCEN);
        }
    }

dumpregs:
  ninfo("Am79c874 MII registers (FIAD=%lx)\n",
        CONFIG_EZ80_FIAD);
  ninfo("  MII_MCR:         %04x\n",
        ez80emac_miiread(priv, MII_MCR));
  ninfo("  MII_MSR:         %04x\n",
        ez80emac_miiread(priv, MII_MSR));
  ninfo("  MII_PHYID1:      %04x\n",
        ez80emac_miiread(priv, MII_PHYID1));
  ninfo("  MII_PHYID2:      %04x\n",
        ez80emac_miiread(priv, MII_PHYID2));
  ninfo("  MII_ADVERTISE:   %04x\n",
        ez80emac_miiread(priv, MII_ADVERTISE));
  ninfo("  MII_LPA:         %04x\n",
        ez80emac_miiread(priv, MII_LPA));
  ninfo("  MII_EXPANSION:   %04x\n",
        ez80emac_miiread(priv, MII_EXPANSION));
  ninfo("  MII_DIAGNOSTICS: %04x\n",
        ez80emac_miiread(priv, MII_AM79C874_DIAGNOSTIC));
  ninfo("EMAC CFG1:         %02x\n",
        inp(EZ80_EMAC_CFG1));
  return ret;
}
#else
static int ez80emac_miiconfigure(FAR struct ez80emac_driver_s *priv)
{
  uint16_t advertise;
  uint16_t lpa;
  uint16_t mcr;
  uint8_t  regval;
  int      i;

  /* Start auto-negotiation */

  ez80emac_miiwrite(priv, MII_MCR, 0);
  ez80emac_miiwrite(priv,
           MII_MCR,
           MII_MCR_ANENABLE | MII_MCR_ANRESTART | MII_MCR_FULLDPLX |
           MII_MCR_SPEED100);

  /* Wait for auto-negotiation to complete and a link to be established */

  for (i = 0; i < 50; i++)
    {
      regval = ez80emac_miiread(priv, MII_MSR);
      if ((regval & (MII_MSR_ANEGCOMPLETE | MII_MSR_LINKSTATUS)) != 0)
        {
           break;
        }

      up_mdelay(50);
    }

  /* Wait link */

  if (!ez80emac_miipoll(priv, MII_MSR, MII_MSR_LINKSTATUS, false))
    {
      nwarn("WARNING: Link is down!\n");
      priv->blinkok = false;
    }
  else
    {
      priv->blinkok = true;
    }

  /* Read capable media type */

  up_udelay(500);
  advertise = ez80emac_miiread(priv, MII_ADVERTISE);
  lpa       = ez80emac_miiread(priv, MII_LPA);

  /* Check for 100BASETX full duplex */

  if ((advertise & MII_ADVERTISE_100BASETXFULL) &&
      (lpa & MII_LPA_100BASETXFULL))
    {
      ninfo("100BASETX full duplex\n");

      regval            = inp(EZ80_EMAC_CFG1);
      regval           |= EMAC_CFG1_FULLHD; /* Enable full duplex mode */
      outp(EZ80_EMAC_CFG1, regval);

      priv->b100mbs     = true;
      priv->bfullduplex = true;
    }

  /* Check for 100BASETX half duplex */

  else if ((advertise & MII_ADVERTISE_100BASETXHALF) &&
           (lpa & MII_LPA_100BASETXHALF))
    {
      ninfo("100BASETX half duplex\n");
      regval            = inp(EZ80_EMAC_CFG1);
      regval           &= ~EMAC_CFG1_FULLHD; /* Disable full duplex mode */
      outp(EZ80_EMAC_CFG1, regval);
      priv->b100mbs     = true;
      priv->bfullduplex = false;
    }

  /* Check for 10BASETX full duplex */

  else if ((advertise & MII_ADVERTISE_10BASETXFULL) &&
           (lpa & MII_LPA_10BASETXFULL))
    {
      ninfo("10BASETX full duplex\n");

      regval            = inp(EZ80_EMAC_CFG1);
      regval           |= EMAC_CFG1_FULLHD; /* Enable full duplex mode */
      outp(EZ80_EMAC_CFG1, regval);

      priv->b100mbs     = false;
      priv->bfullduplex = true;
    }

  /* Check for 10BASETX half duplex */

  else if ((advertise & MII_ADVERTISE_10BASETXHALF) &&
           (lpa & MII_LPA_10BASETXHALF))
    {
      ninfo("10BASETX half duplex\n");

      regval            = inp(EZ80_EMAC_CFG1);
      regval           &= ~EMAC_CFG1_FULLHD; /* Disable full duplex mode */
      outp(EZ80_EMAC_CFG1, regval);

      priv->b100mbs     = false;
      priv->bfullduplex = false;
    }
  else
    {
      nwarn("WARNING: No valid connection; force 10Mbps half-duplex.\n");

      regval            = inp(EZ80_EMAC_CFG1);
      regval           &= ~EMAC_CFG1_FULLHD; /* Disable full duplex mode */
      outp(EZ80_EMAC_CFG1, regval);
      priv->b100mbs     = false;
      priv->bfullduplex = false;
    }

  /* Set MII control */

  mcr = ez80emac_miiread(priv, MII_MCR);
  if (priv->bfullduplex)
    {
      mcr |= MII_MCR_FULLDPLX;
    }
  else
    {
      mcr &= ~MII_MCR_FULLDPLX;
    }

  if (priv->b100mbs)
    {
      mcr |= MII_MCR_SPEED100;
    }
  else
    {
      mcr &= ~MII_MCR_SPEED100;
    }

  mcr |= MII_MCR_ANENABLE;
  ez80emac_miiwrite(priv, MII_MCR, mcr);

  ninfo("MII registers (FIAD=%x)\n", CONFIG_EZ80_FIAD);
  ninfo("  MII_MCR:       %04x\n", ez80emac_miiread(priv, MII_MCR));
  ninfo("  MII_MSR:       %04x\n", ez80emac_miiread(priv, MII_MSR));
  ninfo("  MII_PHYID1:    %04x\n", ez80emac_miiread(priv, MII_PHYID1));
  ninfo("  MII_PHYID2:    %04x\n", ez80emac_miiread(priv, MII_PHYID2));
  ninfo("  MII_ADVERTISE: %04x\n", ez80emac_miiread(priv, MII_ADVERTISE));
  ninfo("  MII_LPA:       %04x\n", ez80emac_miiread(priv, MII_LPA));
  ninfo("  MII_EXPANSION: %04x\n", ez80emac_miiread(priv, MII_EXPANSION));
  ninfo("EMAC CFG1:         %02x\n", inp(EZ80_EMAC_CFG1));
  return OK;
}
#endif

/****************************************************************************
 * Function: ez80emac_machash
 *
 * Description:
 *   Given a MAC address, perform the CRC32 calculation and return the
 *   index and bit number for the multi-cast hash table.
 *
 * Input Parameters:
 *   priv   - Reference to the driver state structure
 *   mac    - The MAC address to add
 *   enable - true: Enable filtering on this address; false: disable
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_EZ80_MCFILTER
static void ez80emac_machash(FAR uint8_t *mac, FAR int *ndx, FAR int *bitno)
{
  uint32_t hash;
  uint32_t crc32;
  int i;
  int j;

  /* Calculate the CRC32 value on the MAC address */

  crc32 = 0xffffffff;
  for (i = 0; i < 6; i++)
    {
      crc32 ^= (uint32_t)mac[i] & 0x0f;
      for (j = 0; j < 4; j++)
        {
          if (crc32 & 1)
            {
              crc32 = (crc32 >> 1) ^ EMAC_CRCPOLY2;
            }
          else
            {
              crc32 >>= 1;
            }
        }

      crc32 ^= (uint32_t)mac[i] >> 4;
      for (j = 0; j < 4; j++)
        {
          if (crc32 & 1)
            {
              crc32 = (crc32 >> 1) ^ EMAC_CRCPOLY2;
            }
          else
            {
              crc32 >>= 1;
            }
        }
    }

  /* The normal CRC result would be the complement of crc32,
   * the following calculates the EMAC hash value
   *
   * This loop changes the bit ordering the for bits [23:28] of
   * the CRC32 value to -> [0:5]
   */

  crc32 &= 0x000001f8;
  hash = 0;

  for (j = 31; j >= 23; j--)
    {
      hash    = (hash << 1) + (crc32 & 1);
      crc32 >>= 1;
    }

  *ndx   = (hash >> 3) & 7;
  *bitno = hash & 7;
}
#endif

/****************************************************************************
 * Function: ez80emac_transmit
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
 *
 ****************************************************************************/

static int ez80emac_transmit(FAR struct ez80emac_driver_s *priv)
{
  FAR struct ez80emac_desc_s *txdesc;
  FAR struct ez80emac_desc_s *txnext;
  FAR uint8_t *psrc;
  FAR uint8_t *pdest;
  uint24_t     len;
  irqstate_t   flags;

  /* Careful:  This function can be called from outside of the interrupt
   * handler and, therefore, may be suspended when debug output is generated!
   */

  ninfo("txnext=%p {%06x, %u, %04x} trp=%02x%02x\n",
        priv->txnext, priv->txnext->np, priv->txnext->pktsize,
        priv->txnext->stat, inp(EZ80_EMAC_TRP_H), inp(EZ80_EMAC_TRP_L));

  /* Increment statistics */

  flags = enter_critical_section();
  EMAC_STAT(priv, tx_packets);

  /* The current packet to be sent is txnext; Calculate the new txnext and
   * set the ownership to host so that the EMAC does not try to transmit
   * the next packet.
   *
   * The new txnext will be the current txnext plus the size of the
   * descriptor header plus the size of the data to be transferred, aligned
   * up to the next packet buffer size.  NOTE: that there is no check to
   * see if we have overran the EMAC buffer -- i.e., if the next txnext has
   * not yet been transmitted.
   */

  txdesc = priv->txnext;

  len    = EMAC_PKTBUF_ALIGN(priv->dev.d_len + SIZEOF_EMACSDESC);
  txnext = (FAR struct ez80emac_desc_s *)((FAR uint8_t *)txdesc + len);

  /* Handle wraparound to the beginning of the TX region */

  if ((uint8_t *)txnext + SIZEOF_EMACSDESC >= (FAR uint8_t *)priv->rxstart)
    {
      txnext = (FAR struct ez80emac_desc_s *)
        ((FAR uint8_t *)priv->txstart +
        ((FAR uint8_t *)txnext - (uint8_t *)priv->rxstart));
    }

  priv->txnext    = txnext;
  txnext->np      = 0;
  txnext->pktsize = 0;
  txnext->stat    = 0; /* Bit 15: 0=Host (eZ80 CPU) owns, 1=EMAC owns. */

  /* Copy the data to the next packet in the Tx buffer
   * (handling wraparound)
   */

  psrc            = priv->dev.d_buf;
  pdest           = (FAR uint8_t *)txdesc + SIZEOF_EMACSDESC;
  len             = (FAR uint8_t *)priv->rxstart - pdest;
  if (len >= priv->dev.d_len)
    {
      /* The entire packet will fit into the EMAC SRAM without wrapping */

      memcpy(pdest, psrc, priv->dev.d_len);
    }
  else
    {
      /* Handle wrap to the beginning of the buffer */

      memcpy(pdest, psrc, len);
      memcpy(priv->txstart, &psrc[len], (priv->dev.d_len - len));
    }

  if (!priv->txhead)
    {
      /* There are no pending TX actions.  This descriptor is the new head */

      priv->txhead = txdesc;
    }

  /* Then, give ownership of the descriptor to the hardware.  It should
   * perform the transmission on its next polling cycle.
   */

  txdesc->np      = (uint24_t)priv->txnext;
  txdesc->pktsize = priv->dev.d_len;
  txdesc->stat    = EMAC_TXDESC_OWNER;

  /* Enable the TX poll timer.  The poll timer may already be running.  In
   * that case, this will force the hardware to poll again now
   */

  outp(EZ80_EMAC_PTMR, EMAC_PTMR);
  leave_critical_section(flags);

  ninfo("txdesc=%p {%06x, %u, %04x}\n",
        txdesc, txdesc->np, txdesc->pktsize, txdesc->stat);
  ninfo("txnext=%p {%06x, %u, %04x} trp=%02x%02x\n",
        txnext, txnext->np, txnext->pktsize, txnext->stat,
        inp(EZ80_EMAC_TRP_H), inp(EZ80_EMAC_TRP_L));

  /* Setup the TX timeout watchdog (perhaps restarting the timer) */

  wd_start(&priv->txtimeout, EMAC_TXTIMEOUT,
           ez80emac_txtimeout_expiry, (wdparm_t)priv);
  return OK;
}

/****************************************************************************
 * Function: ez80emac_txpoll
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
 *
 ****************************************************************************/

static int ez80emac_txpoll(FAR struct net_driver_s *dev)
{
  FAR struct ez80emac_driver_s *priv =
    (FAR struct ez80emac_driver_s *)dev->d_private;
  int ret = 0;

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  ninfo("Poll result: d_len=%d\n", priv->dev.d_len);
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
          /* Send the packet.  ez80emac_transmit() will return zero if the
           * packet was successfully handled.
           */

          ret = ez80emac_transmit(priv);
        }
    }

  /* If zero is returned, the polling will continue until all connections
   * have been examined.
   */

  return ret;
}

/****************************************************************************
 * Function: ez80emac_rwp
 *
 * Description:
 *   Get the eZ80 RWP value
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The current RWP value expressed as a pointer to a descriptor
 *
 ****************************************************************************/

static inline FAR struct ez80emac_desc_s *ez80emac_rwp(void)
{
  return (FAR struct ez80emac_desc_s *)
    (ETH_RAMADDR +
    ((uint24_t)inp(EZ80_EMAC_RWP_H) << 8) + (uint24_t)inp(EZ80_EMAC_RWP_L));
}

/****************************************************************************
 * Function: ez80emac_rrp
 *
 * Description:
 *   Get the eZ80 RRP value
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The current RRP value expressed as a pointer to a descriptor
 *
 ****************************************************************************/

static inline FAR struct ez80emac_desc_s *ez80emac_rrp(void)
{
  return (FAR struct ez80emac_desc_s *)
    (ETH_RAMADDR +
    ((uint24_t)inp(EZ80_EMAC_RRP_H) << 8) + (uint24_t)inp(EZ80_EMAC_RRP_L));
}

/****************************************************************************
 * Function: ez80emac_receive
 *
 * Description:
 *   Process received packets pending in the RX buffer
 *
 * Input Parameters:
 *   priv  - Driver data instance
 *
 * Returned Value:
 *   0: Success, but nothing received
 *  >0: Success, number of packets received
 *  <0: ERROR, negated error number
 *
 * Returned Value:
 *  Interrupts are disabled
 *
 ****************************************************************************/

static int ez80emac_receive(FAR struct ez80emac_driver_s *priv)
{
  FAR struct ez80emac_desc_s *rxdesc = priv->rxnext;
  FAR struct ez80emac_desc_s *rwp;
  FAR uint8_t *psrc;
  FAR uint8_t *pdest;
  int          pktlen;
  int          npackets;
  uint8_t      pktgood;

  /* The RRP register points to where the next Receive packet is read from.
   * The read-only EMAC Receive Write Pointer (RWP) register reports the
   * current RxDMA Receive Write pointer.  The RxDMA block uses the RRP[12:5]
   * to compare to RWP[12:5] for determining how many buffers remain. The
   * result is the BLKSLFT register.
   */

  rwp = ez80emac_rwp();
  ninfo("rxnext=%p {%06x, %u, %04x} rrp=%p rwp=%p blkslft=%02x%02x\n",
        rxdesc, rxdesc->np, rxdesc->pktsize, rxdesc->stat,
        ez80emac_rrp(), rwp,
        inp(EZ80_EMAC_BLKSLFT_H), inp(EZ80_EMAC_BLKSLFT_L));

  /* The RxDMA reads the data from the RxFIFO and stores it in the EMAC
   * memory Receive buffer. When the end of the packet is detected, the
   * RxDMA reads the next two bytes from the RxFIFO and writes them into
   * the Rx descriptor status LSB and MSB. The packet length counter is
   * stored into the descriptor table packet length field, the descriptor
   * table next pointer is written into the Rx descriptor table and finally
   * the Rx_DONE_STAT bit in the EMAC Interrupt Status Register register is
   * set to 1.
   */

  npackets = 0;
  while (rxdesc != rwp)
    {
      DEBUGASSERT(rxdesc == ez80emac_rrp());
      EMAC_STAT(priv, rx_packets);

      if ((rxdesc->stat & EMAC_RXDESC_OK) != 0)
        {
          /* We have a good packet. Check if the packet is a valid size
           * for the network buffer configuration.
           */

          pktgood = 1;

          if (rxdesc->pktsize > CONFIG_NET_ETH_PKTSIZE)
            {
              ninfo("Truncated oversize RX pkt: %d->%d\n",
                    rxdesc->pktsize, CONFIG_NET_ETH_PKTSIZE);
              pktlen = CONFIG_NET_ETH_PKTSIZE;
            }
          else
            {
              pktlen = rxdesc->pktsize;
            }

          /* Copy the data data from the hardware to priv->dev.d_buf */

          psrc  = (FAR uint8_t *)priv->rxnext + SIZEOF_EMACSDESC;
          pdest =  priv->dev.d_buf;

          /* Check for wraparound */

          if ((FAR uint8_t *)(psrc + pktlen) > (FAR uint8_t *)priv->rxendp1)
            {
              int nbytes = (int)((FAR uint8_t *)priv->rxendp1 -
                                 (FAR uint8_t *)psrc);

              ninfo("RX wraps after %d bytes\n", nbytes + SIZEOF_EMACSDESC);

              memcpy(pdest, psrc, nbytes);
              memcpy(&pdest[nbytes], priv->rxstart, pktlen - nbytes);
            }
          else
            {
              memcpy(pdest, psrc, pktlen);
            }

          /* Set the amount of data in priv->dev.d_len */

          priv->dev.d_len = pktlen;
        }
      else
        {
          /* The packet is bad, but the Rx descriptor reclaim still needs
           * to be done. Account for the bad packet here, where rxdesc is
           * still valid.
           */

          ninfo("Skipping bad RX pkt: %04x\n", rxdesc->stat);
          EMAC_STAT(priv, rx_errors);
          EMAC_STAT(priv, rx_nok);
          pktgood = 0;
        }

      /* Reclaim the Rx descriptor */

      priv->rxnext    = (FAR struct ez80emac_desc_s *)rxdesc->np;

      rxdesc->np      = 0;
      rxdesc->pktsize = 0;
      rxdesc->stat    = 0;

      /* Update pointers */

      rxdesc          = priv->rxnext;
      rwp             = ez80emac_rwp();

      /* Update the RRP to match our rxnext pointer: "For the hardware flow
       * control to function properly, the software must update the hardware
       * RRP (EmacRrp) pointer whenever the software version is updated.
       * The RxDMA uses RWP and the RRP to determine how many packets remain
       * in the Rx buffer.
       */

      outp(EZ80_EMAC_RRP_L, (uint8_t)((uint24_t)rxdesc & 0xff));
      outp(EZ80_EMAC_RRP_H, (uint8_t)(((uint24_t)rxdesc >> 8) & 0xff));

      ninfo("rxnext=%p {%06x, %u, %04x} rrp=%p rwp=%p blkslft=%02x%02x\n",
            rxdesc, rxdesc->np, rxdesc->pktsize, rxdesc->stat,
            ez80emac_rrp(), rwp,
            inp(EZ80_EMAC_BLKSLFT_H), inp(EZ80_EMAC_BLKSLFT_L));

      /* That's as far as we go processing bad packets */

      if (pktgood == 0)
        {
          continue;
        }

#ifdef CONFIG_NET_PKT
      /* When packet sockets are enabled, feed the frame into the tap */

      pkt_input(&priv->dev);
#endif

      /* We only accept IP packets of the configured type and ARP packets */

#ifdef CONFIG_NET_IPv4
      if (ETHBUF->type == HTONS(ETHTYPE_IP))
        {
          ninfo("IPv4 frame\n");

          /* Handle ARP on input then give the IPv4 packet to the network
           * layer
           */

          EMAC_STAT(priv, rx_ip);
          arp_ipin(&priv->dev);
          ipv4_input(&priv->dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, the field  d_len will set to a value >
           * 0.
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

              ez80emac_transmit(priv);
            }
        }
      else
#endif
#ifdef CONFIG_NET_IPv6
      if (ETHBUF->type == HTONS(ETHTYPE_IP6))
        {
          ninfo("IPv6 frame\n");

          /* Give the IPv6 packet to the network layer */

          EMAC_STAT(priv, rx_ip);
          ipv6_input(&priv->dev);

          /* If the above function invocation resulted in data that should
           * be sent out on the network, the field  d_len will set to a
           * value > 0.
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

              ez80emac_transmit(priv);
            }
        }
      else
#endif
#ifdef CONFIG_NET_ARP
      if (ETHBUF->type == HTONS(ETHTYPE_ARP))
        {
          ninfo("ARP packet received (%02x)\n", ETHBUF->type);
          EMAC_STAT(priv, rx_arp);

          arp_arpin(&priv->dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, the field  d_len will set to a value >
           * 0.
           */

          if (priv->dev.d_len > 0)
            {
              ez80emac_transmit(priv);
            }
        }
      else
#endif
        {
          ninfo("Unsupported packet type dropped (%02x)\n", ETHBUF->type);
          EMAC_STAT(priv, rx_dropped);
        }

      npackets++;
    }

  return npackets;
}

/****************************************************************************
 * Function: ez80emac_txinterrupt_work
 *
 * Description:
 *   Perform Tx interrupt related work from the worker thread
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

static void ez80emac_txinterrupt_work(FAR void *arg)
{
  FAR struct ez80emac_driver_s *priv = (FAR struct ez80emac_driver_s *)arg;
  FAR struct ez80emac_desc_s *txhead = priv->txhead;
  uint8_t istat;

  /* Process pending Ethernet Tx interrupts */

  net_lock();

  /* EMAC Tx interrupts:
   *
   * EMAC_ISTAT_TXDONE   - Bit 0: 1=Transmit done interrupt
   *   Denotes when packet transmission is complete.
   * EMAC_ISTAT_TXCF     - Bit 1: 1=Transmit control frame interrupt
   *   Denotes when control frame transmission is complete.
   */

  /* Get and clear Tx interrupt status bits */

  istat = inp(EZ80_EMAC_ISTAT) & EMAC_ISTAT_TXEVENTS;
  outp(EZ80_EMAC_ISTAT, istat);

  EMAC_STAT(priv, tx_int);

  /* All events are packet/control frame transmit complete events */

  ninfo("txhead=%p {%06x, %u, %04x} trp=%02x%02x istat=%02x\n",
        txhead, txhead->np, txhead->pktsize, txhead->stat,
        inp(EZ80_EMAC_TRP_H), inp(EZ80_EMAC_TRP_L), istat);

  /* Handle all packets in the list that are no longer owned by hardware */

  while (txhead && (txhead->stat & EMAC_TXDESC_OWNER) == 0)
    {
      if ((txhead->stat & EMAC_TXDESC_ABORT) != 0)
        {
          nwarn("WARNING: Descriptor %p aborted {%06x, %u, %04x} "
                "trp=%02x%02x\n",
                txhead, txhead->np, txhead->pktsize, txhead->stat,
                inp(EZ80_EMAC_TRP_H), inp(EZ80_EMAC_TRP_L));

          EMAC_STAT(priv, tx_errors);
          EMAC_STAT(priv, tx_abterrors);
        }

      /* Get the address of the next Tx descriptor in the list (if any) */

      txhead = (FAR struct ez80emac_desc_s *)txhead->np;
      if (txhead)
        {
          ninfo("txhead=%p {%06x, %u, %04x} trp=%02x%02x\n",
                txhead, txhead->np, txhead->pktsize, txhead->stat,
                inp(EZ80_EMAC_TRP_H), inp(EZ80_EMAC_TRP_L));
        }
    }

  /* Save the new head.  If it is NULL, then we have read all the way to
   * the terminating description with np==NULL.
   */

  priv->txhead = txhead;
  if (!priv->txhead)
    {
      ninfo("No pending Tx.. Stopping XMIT function.\n");

      /* Stop the Tx poll timer. (It will get restarted when we have
       * something to send
       */

      outp(EZ80_EMAC_PTMR, 0);

      /* Reset the transmit function.  That should force the TRP to be
       * the same as TDLP which is then set to txstart.
       */

#if 0 // Seems to reset RWP as well ???
      priv->txnext = priv->txstart;

      regval  = inp(EZ80_EMAC_RST);
      regval |= EMAC_RST_HRTFN;
      outp(EZ80_EMAC_RST, regval);
      regval &= ~EMAC_RST_HRTFN;
      outp(EZ80_EMAC_RST, regval);
#endif

      /* Cancel any pending the TX timeout */

      wd_cancel(&priv->txtimeout);
    }

  net_unlock();

  /* Re-enable Ethernet Tx interrupts */

  outp(EZ80_EMAC_IEN, EMAC_EIN_HANDLED); /* Enable all interrupts */
}

/****************************************************************************
 * Function: ez80emac_txinterrupt
 *
 * Description:
 *   Process Tx-related interrupt events
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

static int ez80emac_txinterrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct ez80emac_driver_s *priv = &g_emac;
  uint8_t istat;

  /* Disable further Ethernet Tx interrupts.  Because Ethernet interrupts are
   * also disabled if the TX timeout event occurs, there can be no race
   * condition here.
   */

  outp(EZ80_EMAC_IEN, 0);        /* Disable all interrupts */

  /* Determine if a TX transfer just completed */

  istat = inp(EZ80_EMAC_ISTAT);
  if ((istat & EMAC_ISTAT_TXDONE) != 0)
    {
      /* If a TX transfer just completed, then cancel the TX timeout so
       * there will be no race condition between any subsequent timeout
       * expiration and the deferred interrupt processing.
       */

       wd_cancel(&priv->txtimeout);
    }

  /* Schedule to perform the Tx interrupt processing on the worker thread. */

  work_queue(ETHWORK, &priv->txwork, ez80emac_txinterrupt_work, priv, 0);

  return OK;
}

/****************************************************************************
 * Function: ez80emac_rxinterrupt_work
 *
 * Description:
 *   Perform Rx interrupt related work from the worker thread
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

static void ez80emac_rxinterrupt_work(FAR void *arg)
{
  FAR struct ez80emac_driver_s *priv = (FAR struct ez80emac_driver_s *)arg;
  uint8_t  istat;

  /* Process pending Ethernet Rx interrupts */

  net_lock();

  /* EMAC Rx interrupts:
   *
   * EMAC_ISTAT_RXDONE   - Bit 3: 1=Receive done interrupt
   *   Denotes when packet reception is complete.
   * EMAC_ISTAT_RXPCF    - Bit 4: 1=Receive pause control frame interrupt
   *   Denotes when pause control frame reception is complete.
   * EMAC_ISTAT_RXCF     - Bit 5: 1=Receive control frame interrupt
   *   Denotes when control frame reception is complete.
   */

  /* Get and clear Rx interrupt status bits */

  istat = inp(EZ80_EMAC_ISTAT) & EMAC_ISTAT_RXEVENTS;
  outp(EZ80_EMAC_ISTAT, istat);

  EMAC_STAT(priv, rx_int);

  /* Process any RX packets pending the RX buffer */

  ez80emac_receive(priv);
  net_unlock();

  /* Re-enable Ethernet Rx interrupts */

  outp(EZ80_EMAC_IEN, EMAC_EIN_HANDLED); /* Enable all interrupts */
}

/****************************************************************************
 * Function: ez80emac_rxinterrupt
 *
 * Description:
 *   Process Rx-related interrupt events
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

static int ez80emac_rxinterrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct ez80emac_driver_s *priv = &g_emac;

  /* Disable further Ethernet Rx interrupts.  Because Ethernet interrupts are
   * also disabled if the TX timeout event occurs, there can be no race
   * condition here.
   */

  outp(EZ80_EMAC_IEN, 0);        /* Disable all interrupts */

  /* Schedule to perform the Rx interrupt processing on the worker thread. */

  work_queue(ETHWORK, &priv->rxwork,  ez80emac_rxinterrupt_work, priv, 0);
  return OK;
}

/****************************************************************************
 * Function: ez80emac_sysinterrupt_work
 *
 * Description:
 *   Perform system interrupt related work from the worker thread
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

static void ez80emac_sysinterrupt_work(FAR void *arg)
{
  FAR struct ez80emac_driver_s *priv = (FAR struct ez80emac_driver_s *)arg;
  uint8_t istat;

  /* Process pending system interrupts */

  net_lock();

  /* EMAC system interrupts :
   *
   * EMAC_ISTAT_TXFSMERR - Bit 7: 1=Transmit state machine error interrupt
   *   A Transmit State Machine Error should never occur. However, if this
   *   bit is set, the entire transmitter module must be reset.
   * EMAC_ISTAT_MGTDONE  - Bit 6: 1=MII Mgmt done interrupt
   *   This bit is set when communicating to the PHY over the MII during
   *   a Read or Write operation.
   * EMAC_ISTAT_RXOVR    - Bit 2: 1=Receive overrun interrupt
   *   If this bit is set, all incoming packets are ignored until
   *   this bit is cleared by software.
   */

  EMAC_STAT(priv, sys_int);

  /* Get and clear system interrupt status bits */

  istat = inp(EZ80_EMAC_ISTAT) & EMAC_ISTAT_SYSEVENTS;
  outp(EZ80_EMAC_ISTAT, istat);

  /* Check for transmit state machine error */

  if ((istat & EMAC_ISTAT_TXFSMERR) != 0)
    {
      nwarn("WARNING: Tx FSMERR txhead=%p {%06x, %u, %04x} trp=%02x%02x "
            "istat=%02x\n",
            priv->txhead, priv->txhead->np, priv->txhead->pktsize,
            priv->txhead->stat, inp(EZ80_EMAC_TRP_H), inp(EZ80_EMAC_TRP_L),
            istat);

      /* Increment statistics */

      EMAC_STAT(priv, tx_errors);
      EMAC_STAT(priv, tx_fsmerrors);

      /* Really need to reset the transmitter module here */
    }

  /* Check for Rx overrun error */

  if ((istat & EMAC_ISTAT_RXOVR) != 0)
    {
      nwarn("WARNING: Rx OVR rxnext=%p {%06x, %u, %04x} "
            "rrp=%02x%02x rwp=%02x%02x blkslft=%02x%02x istat=%02x\n",
           priv->rxnext, priv->rxnext->np, priv->rxnext->pktsize,
           priv->rxnext->stat,
           inp(EZ80_EMAC_RRP_H), inp(EZ80_EMAC_RRP_L),
           inp(EZ80_EMAC_RWP_H), inp(EZ80_EMAC_RWP_L),
           inp(EZ80_EMAC_BLKSLFT_H), inp(EZ80_EMAC_BLKSLFT_L),
           istat);

      /* Increment statistics */

      EMAC_STAT(priv, rx_errors);
      EMAC_STAT(priv, rx_ovrerrors);
    }

  net_unlock();

  /* Re-enable Ethernet system interrupts */

  outp(EZ80_EMAC_IEN, EMAC_EIN_HANDLED); /* Enable all interrupts */
}

/****************************************************************************
 * Function: ez80emac_sysinterrupt
 *
 * Description:
 *   System interrupt handler
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

static int ez80emac_sysinterrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct ez80emac_driver_s *priv = &g_emac;

  /* Disable further Ethernet interrupts.  Because Ethernet interrupts are
   * also disabled if the TX timeout event occurs, there can be no race
   * condition here.
   */

  outp(EZ80_EMAC_IEN, 0);

  /* Schedule to perform the interrupt processing on the worker thread. */

  work_queue(ETHWORK, &priv->syswork,  ez80emac_sysinterrupt_work, priv, 0);
  return OK;
}

/****************************************************************************
 * Function: ez80emac_txtimeout_work
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
 *   The network is locked.
 *
 ****************************************************************************/

static void ez80emac_txtimeout_work(FAR void *arg)
{
  FAR struct ez80emac_driver_s *priv = (FAR struct ez80emac_driver_s *)arg;
  irqstate_t flags;

  /* Process pending Ethernet interrupts */

  net_lock();

  /* Increment statistics and dump debug info */

  EMAC_STAT(priv, tx_errors);
  EMAC_STAT(priv, tx_timeouts);

  /* Then reset the hardware */

  flags = enter_critical_section();
  ez80emac_ifdown(&priv->dev);
  ez80emac_ifup(&priv->dev);
  leave_critical_section(flags);

  /* Then poll the network for new XMIT data */

  devif_poll(&priv->dev, ez80emac_txpoll);
  net_unlock();
}

/****************************************************************************
 * Function: ez80emac_txtimeout_expiry
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

static void ez80emac_txtimeout_expiry(wdparm_t arg)
{
  FAR struct ez80emac_driver_s *priv = (FAR struct ez80emac_driver_s *)arg;

  /* Disable further Ethernet Tx interrupts.  This will prevent some race
   * conditions with interrupt work.  There is still a potential race
   * condition with interrupt work that is already queued and in progress.
   */

  outp(EZ80_EMAC_IEN, 0);

  /* Schedule to perform the TX timeout processing on the worker thread. */

  work_queue(ETHWORK, &priv->txwork, ez80emac_txtimeout_work, priv, 0);
}

/****************************************************************************
 * Function: ez80emac_ifup
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

static int ez80emac_ifup(FAR struct net_driver_s *dev)
{
  FAR struct ez80emac_driver_s *priv =
    (FAR struct ez80emac_driver_s *)dev->d_private;
  uint8_t regval;
  int ret;

  ninfo("Bringing up: MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
        dev->d_mac.ether.ether_addr_octet[0],
        dev->d_mac.ether.ether_addr_octet[1],
        dev->d_mac.ether.ether_addr_octet[2],
        dev->d_mac.ether.ether_addr_octet[3],
        dev->d_mac.ether.ether_addr_octet[4],
        dev->d_mac.ether.ether_addr_octet[5]);
  ninfo("             IP  %d.%d.%d.%d\n",
        dev->d_ipaddr >> 24,       (dev->d_ipaddr >> 16) & 0xff,
       (dev->d_ipaddr >> 8) & 0xff, dev->d_ipaddr & 0xff);

  /* Bring up the interface -- Must be down right now */

  DEBUGASSERT((inp(EZ80_EMAC_CFG4) & EMAC_CFG4_RXEN) == 0);

  /* Reset hardware */

  ret = ez80_emacinitialize();
  if (ret == 0)
    {
      /* EMAC_AFR_BC - Accept broadcast messages
       * EMAC_AFR_MC - Accept any multicast message
       * EMAC_AFR_QMC - Accept only qualified multicast messages
       */

#ifdef CONFIG_EZ80_MCFILTER
      outp(EZ80_EMAC_AFR, EMAC_AFR_BC | EMAC_AFR_QMC | EMAC_AFR_MC);
#else
      outp(EZ80_EMAC_AFR, EMAC_AFR_BC | EMAC_AFR_MC);
#endif

      /* Set the MAC address */

      outp(EZ80_EMAC_STAD_0, priv->dev.d_mac.ether.ether_addr_octet[0]);
      outp(EZ80_EMAC_STAD_1, priv->dev.d_mac.ether.ether_addr_octet[1]);
      outp(EZ80_EMAC_STAD_2, priv->dev.d_mac.ether.ether_addr_octet[2]);
      outp(EZ80_EMAC_STAD_3, priv->dev.d_mac.ether.ether_addr_octet[3]);
      outp(EZ80_EMAC_STAD_4, priv->dev.d_mac.ether.ether_addr_octet[4]);
      outp(EZ80_EMAC_STAD_5, priv->dev.d_mac.ether.ether_addr_octet[5]);

      /* Enable/disable promiscuous mode */

      regval = inp(EZ80_EMAC_AFR);
#if defined(CONFIG_EZ80_EMACPROMISC)
      regval |= EMAC_AFR_PROM;
#else
      regval &= ~EMAC_AFR_PROM;
#endif
      outp(EZ80_EMAC_AFR, regval);

      /* Enable Rx */

      regval = inp(EZ80_EMAC_CFG4);
      regval |= EMAC_CFG4_RXEN;
      outp(EZ80_EMAC_CFG4, regval);

      /* Turn on interrupts */

      outp(EZ80_EMAC_ISTAT, 0xff);           /* Clear all pending interrupts */
      outp(EZ80_EMAC_IEN, EMAC_EIN_HANDLED); /* Enable all interrupts */

      /* Enable the Ethernet interrupts */

      priv->bifup = true;
      outp(EZ80_EMAC_IEN, EMAC_EIN_HANDLED); /* Enable all interrupts */
      ret = OK;
    }

  return ret;
}

/****************************************************************************
 * Function: ez80emac_ifdown
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

static int ez80emac_ifdown(FAR struct net_driver_s *dev)
{
  FAR struct ez80emac_driver_s *priv =
    (FAR struct ez80emac_driver_s *)dev->d_private;
  irqstate_t flags;
  uint8_t regval;

  /* Disable the Ethernet interrupt */

  flags = enter_critical_section();
  outp(EZ80_EMAC_IEN, 0);        /* Disable all interrupts */

  /* Cancel the TX poll timer and TX timeout timers */

  wd_cancel(&priv->txtimeout);

  /* Disable Rx */

  regval = inp(EZ80_EMAC_CFG4);
  regval &= ~EMAC_CFG4_RXEN;
  outp(EZ80_EMAC_CFG4, regval);

  /* Disable the Tx poll timer */

  outp(EZ80_EMAC_PTMR, 0);

  priv->bifup = false;
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Function: ez80emac_txavail_work
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

static void ez80emac_txavail_work(FAR void *arg)
{
  FAR struct ez80emac_driver_s *priv = (FAR struct ez80emac_driver_s *)arg;

  /* Ignore the notification if the interface is not yet up */

  net_lock();
  if (priv->bifup)
    {
      /* Check if there is room in the hardware to hold another packet. */

      /* If so, then poll the network for new XMIT data */

      devif_poll(&priv->dev, ez80emac_txpoll);
    }

  net_unlock();
}

/****************************************************************************
 * Function: ez80emac_txavail
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

static int ez80emac_txavail(FAR struct net_driver_s *dev)
{
  FAR struct ez80emac_driver_s *priv =
    (FAR struct ez80emac_driver_s *)dev->d_private;

  /* Is our single work structure available?  It may not be if there are
   * pending interrupt actions and we will have to ignore the Tx
   * availability action.
   */

  if (work_available(&priv->syswork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      work_queue(ETHWORK, &priv->syswork, ez80emac_txavail_work, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Function: ez80emac_addmac
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
static int ez80emac_addmac(FAR struct net_driver_s *dev,
                           FAR const uint8_t *mac)
{
  FAR struct ez80emac_driver_s *priv =
    (FAR struct ez80emac_driver_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

  /* MISSING LOGIC!!! */

  return OK;
}
#endif

/****************************************************************************
 * Function: ez80emac_rmmac
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
static int ez80emac_rmmac(FAR struct net_driver_s *dev,
                          FAR const uint8_t *mac)
{
  FAR struct ez80emac_driver_s *priv =
    (FAR struct ez80emac_driver_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

  /* MISSING LOGIC!!! */

  return OK;
}
#endif

/****************************************************************************
 * Function: ez80emac_initialize
 *
 * Description:
 *   Initialize the Ethernet driver
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

static int ez80_emacinitialize(void)
{
  FAR struct ez80emac_driver_s *priv = &g_emac;
  uint24_t addr;
  uint8_t regval;
  int ret;

  /* Reset the EMAC hardware */

  outp(EZ80_EMAC_IEN, 0);        /* Disable all interrupts */
  outp(EZ80_EMAC_RST, 0);        /* Reset everything */
  outp(EZ80_EMAC_RST, 0xff);
  outp(EZ80_EMAC_RST, 0);

  /* The ez80 has a fixed 8kb of EMAC SRAM memory (+ 8kb of
   * general purpose SRAM) located in the high address space.
   * Configure the GP and EMAC SRAM.
   *
   * EZ80_RAM_CTL and EZ80_RAM_ADDR_U where configured by ez80 start-up
   * logic.  We need only enable EMAC RAM here.
   */

  outp(EZ80_RAM_CTL, (RAMCTL_ERAMEN | RAMCTL_GPRAMEN));
  outp(EZ80_EMAC_BP_U, (ETH_RAMADDR >> 16));

  /* The EMAC memory is broken into two parts: the Tx buffer and the Rx
   * buffer.
   *
   * The TX buffer lies at the beginning of the EMAC memory.
   * The Transmit Lower Boundary Pointer Register, TLBP, holds the
   * least significant 12-bits of the starting address of the Tx buffer.
   * The Transmit Write Pointer, TRP, will be set to the TLBP.
   */

  addr                  = ETH_RAMADDR;
  outp(EZ80_EMAC_TLBP_L, (uint8_t)(addr & 0xff));
  outp(EZ80_EMAC_TLBP_H, (uint8_t)((addr >> 8) & 0xff));

  priv->txstart         = (FAR struct ez80emac_desc_s *)(addr);
  priv->txnext          = priv->txstart;
  priv->txhead          = NULL;

  priv->txnext->np      = 0;
  priv->txnext->pktsize = 0;
  priv->txnext->stat    = 0;

  ninfo("txnext=%p {%06x, %u, %04x} tlbp=%02x%02x trp=%02x%02x\n",
        priv->txnext, priv->txnext->np, priv->txnext->pktsize,
        priv->txnext->stat, inp(EZ80_EMAC_TLBP_H), inp(EZ80_EMAC_TLBP_L),
        inp(EZ80_EMAC_TRP_H), inp(EZ80_EMAC_TRP_L));

  /* The Boundary Pointer Register, EMAC_BP, points to the start of the Rx
   * buffer (end of Tx buffer + 1).  Only bits EMAC_BP[12:5] of are
   * write-able.
   */

  addr                 += EMAC_TXBUFSIZE;
  outp(EZ80_EMAC_BP_L, (uint8_t)(addr & 0xff));
  outp(EZ80_EMAC_BP_H, (uint8_t)((addr >> 8) & 0xff));

  priv->rxstart         = (FAR struct ez80emac_desc_s *)(addr);
  priv->rxnext          = priv->rxstart;

  priv->rxnext->np      = 0;
  priv->rxnext->pktsize = 0;
  priv->rxnext->stat    = 0;

  ninfo("rxnext=%p {%06x, %u, %04x} bp=%02x%02x\n",
        priv->rxnext, priv->rxnext->np, priv->rxnext->pktsize,
        priv->rxnext->stat, inp(EZ80_EMAC_BP_H), inp(EZ80_EMAC_BP_L));

  /* The EMAC Receive Read Pointer (RRP) register(s) should be initialized
   * to the start of the Receive buffer. The RRP register points to where the
   * next Receive packet is read from. The EMAC_BP[12:5] is loaded into this
   * register whenever the EMAC_RST [(HRRFN) is set to 1. The RxDMA block
   * uses the RRP[12:5] to compare to RWP[12:5] for determining how many
   * buffers remain. The result equates to the BLKSLFT register.
   *
   * The read-only EMAC Receive Write Pointer (RWP) registers report the
   * current RxDMA Receive Write pointer. This pointer gets initialized to
   * EMAC_BP whenever EMAC_RST bits SRST or HRRTN are set. Because the size
   * of the packet is limited to a minimum of 32 bytes, the last five bits
   * are always zero.
   */

  outp(EZ80_EMAC_RRP_L, (uint8_t)(addr & 0xff));
  outp(EZ80_EMAC_RRP_H, (uint8_t)((addr >> 8) & 0xff));

  ninfo("rrp=%02x%02x rwp=%02x%02x\n",
        inp(EZ80_EMAC_RRP_H), inp(EZ80_EMAC_RRP_L),
        inp(EZ80_EMAC_RWP_H), inp(EZ80_EMAC_RWP_L));

  /* The Receive High Boundary Pointer Register, EMAC_RHBP, points to the end
   * of the Rx buffer + 1.  Only bits EMAC_RHBP[12:5] are write-able.
   */

  addr         += EMAC_RXBUFSIZE;
  outp(EZ80_EMAC_RHBP_L, (uint8_t)(addr & 0xff));
  outp(EZ80_EMAC_RHBP_H, (uint8_t)((addr >> 8) & 0xff));
  priv->rxendp1 = (FAR struct ez80emac_desc_s *)addr;

  ninfo("rxendp1=%p rhbp=%02x%02x\n",
        priv->rxendp1,
        inp(EZ80_EMAC_RHBP_H), inp(EZ80_EMAC_RHBP_L));

  /* The Tx and Receive buffers are divided into packet buffers of either
   * 256, 128, 64, or 32 bytes selected by BufSize register bits 7 and 6.
   */

  outp(EZ80_EMAC_BUFSZ, EMAC_BUFSZ);
  ninfo("bufsz=%02x blksleft=%02x%02x\n",
        inp(EZ80_EMAC_BUFSZ), inp(EZ80_EMAC_BLKSLFT_H),
        inp(EZ80_EMAC_BLKSLFT_L));

  /* Software reset */

  outp(EZ80_EMAC_ISTAT, 0xff); /* Clear any pending interrupts */
  regval  = inp(EZ80_EMAC_RST);
  regval |= EMAC_RST_SRST;
  outp(EZ80_EMAC_RST, regval);
  regval &= ~EMAC_RST_SRST;
  outp(EZ80_EMAC_RST, regval);

  ninfo("After soft reset: rwp=%02x%02x trp=%02x%02x\n",
        inp(EZ80_EMAC_RWP_H), inp(EZ80_EMAC_RWP_L),
        inp(EZ80_EMAC_TRP_H), inp(EZ80_EMAC_TRP_L));

  /* Select the fastest MDC clock divider.  The MDC clock derives
   * from the SCLK divided by 4, 6, 8, 10, 14, 20, or 28.
   *
   * This needs to be done before trying to use the MII.
   */

  outp(EZ80_EMAC_MIIMGT, CONFIG_EZ80_MDCDIV);

  /* PHY reset */

  up_udelay(500);
  ez80emac_miiwrite(priv, MII_MCR, MII_MCR_RESET);
  if (!ez80emac_miipoll(priv, MII_MCR, MII_MCR_RESET, true))
    {
      nerr("ERROR: PHY reset error.\n");
    }

  /*  Initialize MAC */

  /* Set only the default late collision bytes in CFG2 */

  outp(EZ80_EMAC_CFG2, EMAC_LCOL);

  /* Set only the retry count in CFG3 */

  outp(EZ80_EMAC_CFG3, EMAC_RETRY);

  /* EMAC_CFG4_TXFC - Pause control frames are allowed to be transmitted
   * EMAC_CFG4_RXFC - Act on received pause control frames
   */

  outp(EZ80_EMAC_CFG4, EMAC_CFG4_TXFC | EMAC_CFG4_RXFC);

  /* EMAC_CFG1_CRCEN + EMAC_CFG1_PADEN + EMAC_CFG1_VLPAD + EMAC_CFG1_ADPADN =
   *   if VLAN not detected, pad to 60, add CRC
   *   if VLAN detected, pad to 64, add CRC
   */

  outp(EZ80_EMAC_CFG1, EMAC_CFG1_CRCEN | EMAC_CFG1_PADEN | EMAC_CFG1_ADPADN |
                       EMAC_CFG1_VLPAD);

  outp(EZ80_EMAC_IPGT, EMAC_IPGT);
  outp(EZ80_EMAC_IPGR1, EMAC_IPGR1);
  outp(EZ80_EMAC_IPGR2, EMAC_IPGR2);

  outp(EZ80_EMAC_MAXF_L, EMAC_MAXF & 0xff);
  outp(EZ80_EMAC_MAXF_H, EMAC_MAXF >> 8);

  /* Clear the new hash table */

  outp(EZ80_EMAC_HTBL_0, 0);
  outp(EZ80_EMAC_HTBL_1, 0);
  outp(EZ80_EMAC_HTBL_2, 0);
  outp(EZ80_EMAC_HTBL_3, 0);
  outp(EZ80_EMAC_HTBL_4, 0);
  outp(EZ80_EMAC_HTBL_5, 0);
  outp(EZ80_EMAC_HTBL_6, 0);
  outp(EZ80_EMAC_HTBL_7, 0);

  /* PHY reset */

  ez80emac_miiwrite(priv, MII_MCR, MII_MCR_RESET);
  if (!ez80emac_miipoll(priv, MII_MCR, MII_MCR_RESET, true))
    {
      nerr("ERROR: PHY reset error.\n");
      ret = -EIO;
      goto errout;
    }

  /* Configure the PHY */

  ret = ez80emac_miiconfigure(priv);

  /* Initialize DMA / FIFO */

  outp(EZ80_EMAC_TPTV_L, EMAC_TPTV & 0xff);
  outp(EZ80_EMAC_TPTV_H, EMAC_TPTV >> 8);
  return OK;

errout:
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: ez80emac_initialize
 *
 * Description:
 *   Initialize the Ethernet driver
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int up_netinitialize(void)
{
  FAR struct ez80emac_driver_s *priv = &g_emac;
  int ret;

  /* Disable all interrupts */

  outp(EZ80_EMAC_IEN, 0);

  /* Attach IRQs */

  ret = irq_attach(EZ80_EMACSYS_IRQ, ez80emac_sysinterrupt, NULL);
  if (ret < 0)
    {
      nerr("ERROR: Unable to attach IRQ %d\n", EZ80_EMACSYS_IRQ);
      ret = -EAGAIN;
      goto errout;
    }

  ret = irq_attach(EZ80_EMACRX_IRQ, ez80emac_rxinterrupt, NULL);
  if (ret < 0)
    {
      nerr("ERROR: Unable to attach IRQ %d\n", EZ80_EMACRX_IRQ);
      ret = -EAGAIN;
      goto errout;
    }

  ret = irq_attach(EZ80_EMACTX_IRQ, ez80emac_txinterrupt, NULL);
  if (ret < 0)
    {
      nerr("ERROR: Unable to attach IRQ %d\n", EZ80_EMACTX_IRQ);
      ret = -EAGAIN;
      goto errout;
    }

  /* Initialize the driver structure */

  memset(&g_emac, 0, sizeof(struct ez80emac_driver_s));
  priv->dev.d_buf     = g_pktbuf;            /* Single packet buffer */
  priv->dev.d_ifup    = ez80emac_ifup;       /* I/F down callback */
  priv->dev.d_ifdown  = ez80emac_ifdown;     /* I/F up (new IP address) callback */
  priv->dev.d_txavail = ez80emac_txavail;    /* New TX data callback */
#ifdef CONFIG_NET_MCASTGROUP
  priv->dev.d_addmac  = ez80emac_addmac;     /* Add multicast MAC address */
  priv->dev.d_rmmac   = ez80emac_rmmac;      /* Remove multicast MAC address */
#endif
  priv->dev.d_private = &g_emac;             /* Used to recover private state from dev */

  /* Read the MAC address from the hardware into
   * priv->dev.d_mac.ether.ether_addr_octet
   */

  /* Register the device with the OS so that socket IOCTLs can be performed */

  netdev_register(&priv->dev, NET_LL_ETHERNET);
  return OK;

errout:
  up_netuninitialize();
  return ret;
}

/****************************************************************************
 * Function: up_multicastfilter
 *
 * Description:
 *   Add one MAC address to the multi-cast hash table
 *
 * Input Parameters:
 *   dev    - Reference to the network driver state structure
 *   mac    - The MAC address to add
 *   enable - true: Enable filtering on this address; false: disable
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_MCFILTER
int up_multicastfilter(FAR struct net_driver_s *dev, FAR uint8_t *mac,
                       bool enable)
{
  FAR struct ez80emac_driver_s *priv =
    (FAR struct ez80emac_driver_s *)dev->priv;
  uint8_t regval;
  int ndx;
  int bit;
  int i;

  /* The EMAC Hash Table Registers represent an 8x8 hash table matrix.
   * This table is used as an option to select between different multi-cast
   * addresses.  If a multicast address is received, the first 6 bits of the
   * CRC decoded and to a table that points to a single bit in the hash
   * table matrix.  if the selected bit is '1', then multicast packet is
   * accepted.  If the bit is '0', the multicast packet is rejected.
   */

  /* Apply the hash algorithm to the hash table index and bit number
   * corresponding to this MAC
   */

  ez80emac_machash(mclist->dmi_addr, &ndx, &bit);

  /* And set/clear that bit in that array element */

  regval = inp(EZ80_EMAC_HTBL_0 + ndx);
  if (enable)
    {
       regval |= (1 << bit);
    }
  else
    {
       regval &= ~(1 << bit);
    }

  outp(EZ80_EMAC_HTBL_0 + ndx, regval);
  return OK;
}
#endif

/****************************************************************************
 * Function: up_netuninitialize
 *
 * Description:
 *   Un-initialize the Ethernet driver
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

void up_netuninitialize(void)
{
  FAR struct ez80emac_driver_s *priv = &g_emac;

  ez80emac_ifdown(&priv->dev);
#if 0
  netdev_unregister(priv->dev); /* No such interface yet */
#endif

  priv->txnext = priv->txstart;
  priv->txhead = NULL;

  irq_detach(EZ80_EMACRX_IRQ);
  irq_detach(EZ80_EMACTX_IRQ);
  irq_detach(EZ80_EMACSYS_IRQ);
}

#endif /* CONFIG_NET && CONFIG_EZ80_EMAC */
