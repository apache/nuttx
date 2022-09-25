/****************************************************************************
 * arch/renesas/src/rx65n/rx65n_eth.c
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

#if defined(CONFIG_NET) && defined(CONFIG_RX65N_EMAC)

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
#include <nuttx/queue.h>
#include <nuttx/wqueue.h>
#include <nuttx/net/phy.h>
#include <nuttx/net/mii.h>
#include <nuttx/net/arp.h>
#include <nuttx/net/netdev.h>

#if defined(CONFIG_ARCH_PHY_INTERRUPT)
#  include <nuttx/net/phy.h>
#endif

#if defined(CONFIG_NET_PKT)
#  include <nuttx/net/pkt.h>
#endif

#include "up_internal.h"
#include "chip.h"
#include "rx65n_definitions.h"
#include "rx65n_eth.h"
#include "rx65n_cmt.h"
#include "rx65n_cmtw.h"
#include "rx65n_cmtw0.h"

#include <arch/board/board.h>
#include <arch/board/rx65n_gpio.h>

#if RX65N_NETHERNET > 0

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if RX65N_NETHERNET > 1
#  error "Logic to support multiple Ethernet interfaces is incomplete"
#endif

#define RX65N_EMAC0_DEVNAME "eth0"

/* Work queue support is required. */

#if !defined (CONFIG_SCHED_WORKQUEUE)
#  error Work queue support is required
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

#ifndef CONFIG_RX65N_EMAC0_PHYADDR
#  error "CONFIG_RX65N_EMAC0_PHYADDR must be defined in the NuttX configuration"
#endif

#if !defined(CONFIG_RX65N_EMAC0_MII) && !defined(CONFIG_RX65N_EMAC0_RMII)
#  warning "Neither CONFIG_RX65N_EMAC0_MII nor CONFIG_RX65N_EMAC0_RMII defined"
#endif

#if defined(CONFIG_RX65N_EMAC0_MII) && defined(CONFIG_RX65N_EMAC0_RMII)
#  error "Both CONFIG_RX65N_EMAC0_MII and CONFIG_RX65N_EMAC0_RMII defined"
#endif

#ifdef CONFIG_RX65N_EMAC0_AUTONEG
#  ifndef CONFIG_RX65N_EMAC0_PHYSR
#    error "CONFIG_RX65N_EMAC0_PHYSR must be defined in the NuttX configuration"
#  endif
#  ifdef CONFIG_RX65N_EMAC0_PHYSR_ALTCONFIG
#    ifndef CONFIG_RX65N_EMAC0_PHYSR_ALTMODE
#      error "CONFIG_RX65N_EMAC0_PHYSR_ALTMODE must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_RX65N_EMAC0_PHYSR_10HD
#      error "CONFIG_RX65N_EMAC0_PHYSR_10HD must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_RX65N_EMAC0_PHYSR_100HD
#      error "CONFIG_RX65N_EMAC0_PHYSR_100HD must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_RX65N_EMAC0_PHYSR_10FD
#      error "CONFIG_RX65N_EMAC0_PHYSR_10FD must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_RX65N_EMAC0_PHYSR_100FD
#      error "CONFIG_RX65N_EMAC0_PHYSR_100FD must be defined in the NuttX configuration"
#    endif
#  else
#    ifndef CONFIG_RX65N_EMAC0_PHYSR_SPEED
#      error "CONFIG_RX65N_EMAC0_PHYSR_SPEED must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_RX65N_EMAC0_PHYSR_100MBPS
#      error "CONFIG_RX65N_EMAC0_PHYSR_100MBPS must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_RX65N_EMAC0_PHYSR_MODE
#      error "CONFIG_RX65N_EMAC0_PHYSR_MODE must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_RX65N_EMAC0_PHYSR_FULLDUPLEX
#      error "CONFIG_RX65N_EMAC0_PHYSR_FULLDUPLEX must be defined in the NuttX configuration"
#    endif
#  endif
#endif

/* This driver does not use enhanced descriptors.  Enhanced descriptors must
 * be used, however, if time stamping and/or IPv4 checksum offload is
 * supported.
 */

#undef CONFIG_RX65N_ETH_ENHANCEDDESC
#undef CONFIG_RX65N_ETH_HWCHECKSUM

/* Add 4 to the configured buffer size to account for the 2 byte checksum
 * memory needed at the end of the maximum size packet.  Buffer sizes must
 * be an even multiple of 4, 8, 16 or 32bytes (depending on buswidth).  We
 * will use the 32-byte alignment in all cases.
 */

#define OPTIMAL_ETH_BUFSIZE ((CONFIG_NET_ETH_PKTSIZE + 4 + 31) & ~31)

#ifndef CONFIG_RX65N_ETH_BUFSIZE
#  define CONFIG_RX65N_ETH_BUFSIZE OPTIMAL_ETH_BUFSIZE
#endif

#if (CONFIG_RX65N_ETH_BUFSIZE & 31) != 0
#  error "CONFIG_RX65N_ETH_BUFSIZE must be aligned"
#endif

#if CONFIG_RX65N_ETH_BUFSIZE != OPTIMAL_ETH_BUFSIZE
#  warning "You using an incomplete/untested configuration"
#endif

#ifndef CONFIG_RX65N_ETH_NRXDESC
#  define CONFIG_RX65N_ETH_NRXDESC 16
#endif
#ifndef CONFIG_RX65N_ETH_NTXDESC
#  define CONFIG_RX65N_ETH_NTXDESC 12
#endif

/* We need at least one more free buffer than transmit buffers */

#define RX65N_ETH_NFREEBUFFERS (CONFIG_RX65N_ETH_NTXDESC+1)

/* Extremely detailed register debug that you would normally never want
 * enabled.
 */

#ifndef CONFIG_DEBUG_NET
#  undef CONFIG_RX65N_EMAC_REGDEBUG
#endif

/* Helpers */

/* This is a helper pointer for accessing the contents of the Ethernet
 * header
 */

#define BUF ((FAR struct eth_hdr_s *)priv->dev.d_buf)

/* PHY return definitions */

#define R_PHY_OK        (0)
#define R_PHY_ERROR     (-1)

#define ETHER_FLAG_OFF   (0)

/* PHY Mode Selection */

#define PHY_MII_SET_MODE        (0)
#define PHY_RMII_SET_MODE       (1)

/* Pause frame disable/enable */

#define PAUSE_FRAME_DISABLE             (0)
#define PAUSE_FRAME_ENABLE              (1)

/* Media Independent Interface */

#define PHY_MII_ST                      (1)
#define PHY_MII_READ                    (2)
#define PHY_MII_WRITE                   (1)

/* Define the access timing of MII/RMII register */

#define ETHER_CFG_PHY_MII_WAIT                      (8)     /* define the value of 1 or more */

/* Define the waiting time for reset completion of PHY-LSI */

#define ETHER_CFG_PHY_DELAY_RESET                   (0x00020000L)
#define ETHER_PHY_STATUS_CHECK_DELAY                (20000)

/* Group AL1 interrupt priority level.
 * This definition is not used when EINT interrupt
 * is assigned to Peripheral
 * interrupt.
 */

#define ETHER_CFG_AL1_INT_PRIORITY                  (15)

/* Use LINKSTA signal for detect link status changes
 * 0 = unused  (use PHY-LSI status register)
 * 1 = use     (use LINKSTA signal)
 */

/* This setting is reflected in all channels */

#define ETHER_CFG_USE_LINKSTA           (1)

#define ETHER_LINKUP                    (1)
#define ETHER_LINKDOWN                  (0)

/* Multicast filter */

#define ETHER_MC_FILTER_OFF             (0) /* Multicast frame filter disable */
#define ETHER_MC_FILTER_ON              (1) /* Multicast frame filter enable */

/* Standard PHY Registers */

#define PHY_REG_CONTROL                 (0)
#define PHY_REG_STATUS                  (1)
#define PHY_REG_IDENTIFIER1             (2)
#define PHY_REG_IDENTIFIER2             (3)
#define PHY_REG_AN_ADVERTISEMENT        (4)
#define PHY_REG_AN_LINK_PARTNER         (5)
#define PHY_REG_AN_EXPANSION            (6)

/* Phy Interrupt register */

#define PHY_REG_MICR (17)
#define PHY_REG_MISR (18)
#define PHY_MICR_TINT (1<<2)
#define PHY_MICR_INTEN (1<<1)
#define PHY_MICR_INT_OE (1<<0)
#define PHY_MISR_LINK_LQ_INT_EN (1<<7)
#define PHY_MISR_LINK_ED_INT_EN (1<<6)
#define PHY_MISR_LINK_INT_EN (1<<5)

/* Basic Mode Control Register Bit Definitions */

#define PHY_CONTROL_RESET               (1 << 15)
#define PHY_CONTROL_LOOPBACK            (1 << 14)
#define PHY_CONTROL_100_MBPS            (1 << 13)
#define PHY_CONTROL_AN_ENABLE           (1 << 12)
#define PHY_CONTROL_POWER_DOWN          (1 << 11)
#define PHY_CONTROL_ISOLATE             (1 << 10)
#define PHY_CONTROL_AN_RESTART          (1 << 9)
#define PHY_CONTROL_FULL_DUPLEX         (1 << 8)
#define PHY_CONTROL_COLLISION           (1 << 7)
#define PHY_VALID_LINK                  (1 << 2)
#define PHY_AUTO_NEG_DONE               (1 << 5)

/* Auto Negotiation Advertisement Bit Definitions */

#define PHY_AN_ADVERTISEMENT_NEXT_PAGE  (1 << 15)
#define PHY_AN_ADVERTISEMENT_RM_FAULT   (1 << 13)
#define PHY_AN_ADVERTISEMENT_ASM_DIR    (1 << 11)
#define PHY_AN_ADVERTISEMENT_PAUSE      (1 << 10)
#define PHY_AN_ADVERTISEMENT_100_T4     (1 << 9)
#define PHY_AN_ADVERTISEMENT_100F       (1 << 8)
#define PHY_AN_ADVERTISEMENT_100H       (1 << 7)
#define PHY_AN_ADVERTISEMENT_10F        (1 << 6)
#define PHY_AN_ADVERTISEMENT_10H        (1 << 5)
#define PHY_AN_ADVERTISEMENT_SELECTOR   (1 << 0)

/* Bit definitions of status member of DescriptorS */

#define  TACT               (0x80000000)
#define  TDLE               (0x40000000)
#define  RACT               (0x80000000)
#define  RDLE               (0x40000000)
#define  TFP1               (0x20000000)
#define  RFP1               (0x20000000)
#define  TFP0               (0x10000000)
#define  RFP0               (0x10000000)
#define  TFE                (0x08000000)
#define  RFE                (0x08000000)

#define  RFS9_RFOVER        (0x00000200)
#define  RFS8_RAD           (0x00000100)
#define  RFS7_RMAF          (0x00000080)
#define  RFS4_RRF           (0x00000010)
#define  RFS3_RTLF          (0x00000008)
#define  RFS2_RTSF          (0x00000004)
#define  RFS1_PRE           (0x00000002)
#define  RFS0_CERF          (0x00000001)

#define  TWBI               (0x04000000)
#define  TFS8_TAD           (0x00000100)
#define  TFS3_CND           (0x00000008)
#define  TFS2_DLC           (0x00000004)
#define  TFS1_CD            (0x00000002)
#define  TFS0_TRO           (0x00000001)

/* DMA descriptor buffer alignment to 32 bytes */

#define NX_ALIGN32 aligned_data(32)

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/* Ethernet TX DMA Descriptor */

struct eth_txdesc_s
{
  /* Normal DMA descriptor words */

  volatile uint32_t tdes0;   /* Indicates transfer frame setting and the status after transmission */
  volatile uint32_t tdes1;   /* Used to set the valid byte length of the transmit buffer */
  volatile uint32_t tdes2;   /* Used to set the start address of transmit buffer */
  volatile uint32_t tdes3;   /* next descriptor address pointer */
};

/* Ethernet RX DMA Descriptor */

struct eth_rxdesc_s
{
  volatile uint32_t rdes0;   /* Indicates receive frame status */

  /* Indicates receive buffer length when reception is completed,
   * the receive frame length is return back
   */

  volatile uint32_t rdes1;
  volatile uint32_t rdes2;   /* Indicates the start address of  the receive buffer  */
  volatile uint32_t rdes3;   /* next descriptor address pointer */
};

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The rx65n_ethmac_s encapsulates all state information for a single
 * hardware interface
 */

struct rx65n_ethmac_s
{
  /* Descriptor allocations */

  struct eth_rxdesc_s rxtable[CONFIG_RX65N_ETH_NRXDESC];
  struct eth_txdesc_s txtable[CONFIG_RX65N_ETH_NTXDESC];

  /* Buffer allocations */

  uint8_t rxbuffer[CONFIG_RX65N_ETH_NRXDESC*CONFIG_RX65N_ETH_BUFSIZE];
  uint8_t alloc[RX65N_ETH_NFREEBUFFERS*CONFIG_RX65N_ETH_BUFSIZE];
  uint8_t              ifup    : 1; /* true:ifup false:ifdown */
  uint8_t              mbps100 : 1; /* 100MBps operation (vs 10 MBps) */
  uint8_t              fduplex : 1; /* Full (vs. half) duplex */
  struct work_s        irqwork;     /* For deferring interrupt work to the work queue */
  struct work_s        pollwork;    /* For deferring poll work to the work queue */

  /* This holds the information visible to the NuttX network */

  struct net_driver_s  dev;         /* Interface understood by the network */

  /* Used to track transmit and receive descriptors */

  struct eth_txdesc_s *txhead;      /* Next available TX descriptor */
  struct eth_rxdesc_s *rxhead;      /* Next available RX descriptor */

  struct eth_txdesc_s *txtail;      /* First "in_flight" TX descriptor */
  struct eth_rxdesc_s *rxcurr;      /* First RX descriptor of the segment */
  uint16_t             segments;    /* RX segment count */
  uint16_t             inflight;    /* Number of TX transfers "in_flight" */
  sq_queue_t           freeb;       /* The free buffer list */

  uint32_t             prevlinkstatus; /* Previous link status to ignore multiple link change interrupt (specific to GR-Rose) */
  uint8_t              mc_filter_flag; /* Multicast filter */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

NX_ALIGN32 static struct rx65n_ethmac_s g_rx65nethmac[RX65N_NETHERNET];

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register operations */

#if defined(CONFIG_RX65N_EMAC_REGDEBUG) && defined(CONFIG_DEBUG_FEATURES)
static uint32_t rx65n_getreg(uint32_t addr);
static void rx65n_putreg(uint32_t val, uint32_t addr);
static void rx65n_checksetup(void);
#else
# define rx65n_getreg(addr)      getreg32(addr)
# define rx65n_putreg(val,addr)  putreg32(val,addr)
# define rx65n_checksetup()
#endif

/* Debug */

/* Extra, in-depth debug output that is only available if
 * CONFIG_NETDEV_PHY_DEBUG us defined.
 */

#ifdef CONFIG_NETDEV_PHY_DEBUG
#  define phyerr    _err
#  define phywarn   _warn
#  define phyinfo   _info
#else
#  define phyerr(x...)
#  define phywarn(x...)
#  define phyinfo(x...)
#endif

/* Free buffer management */

static void rx65n_initbuffer(FAR struct rx65n_ethmac_s *priv);
static inline uint8_t *rx65n_allocbuffer(FAR struct rx65n_ethmac_s *priv);
static inline void rx65n_freebuffer(FAR struct rx65n_ethmac_s *priv,
              uint8_t *buffer);
static inline bool rx65n_isfreebuffer(FAR struct rx65n_ethmac_s *priv);

/* Common TX logic */

static int  rx65n_transmit(FAR struct rx65n_ethmac_s *priv);
static int  rx65n_txpoll(struct net_driver_s *dev);
static void rx65n_dopoll(FAR struct rx65n_ethmac_s *priv);

/* Interrupt handling */

static void rx65n_enableint(FAR struct rx65n_ethmac_s *priv,
              uint32_t ierbit);
static void rx65n_disableint(FAR struct rx65n_ethmac_s *priv,
              uint32_t ierbit);

static void rx65n_freesegment(FAR struct rx65n_ethmac_s *priv,
              FAR struct eth_rxdesc_s *rxfirst, int segments);
static int  rx65n_recvframe(FAR struct rx65n_ethmac_s *priv);
static void rx65n_receive(FAR struct rx65n_ethmac_s *priv);
static void rx65n_freeframe(FAR struct rx65n_ethmac_s *priv);
static void rx65n_txdone(FAR struct rx65n_ethmac_s *priv);

static void rx65n_interrupt_work(FAR void *arg);
static int  rx65n_interrupt(int irq, FAR void *context, FAR void *arg);

/* Timer expirations */

static void rx65n_txtimeout_work(FAR void *arg);

/* NuttX callback functions */

static int  rx65n_ifup(struct net_driver_s *dev);
static int  rx65n_ifdown(struct net_driver_s *dev);

static void rx65n_txavail_work(FAR void *arg);
static int  rx65n_txavail(struct net_driver_s *dev);

#if defined(CONFIG_NET_MCASTGROUP) || defined(CONFIG_NET_ICMPv6)
static int  rx65n_addmac(struct net_driver_s *dev, FAR const uint8_t *mac);
#endif
#ifdef CONFIG_NET_MCASTGROUP
static int  rx65n_rmmac(struct net_driver_s *dev, FAR const uint8_t *mac);
#endif
#ifdef CONFIG_NETDEV_IOCTL
static int  rx65n_ioctl(struct net_driver_s *dev, int cmd,
              unsigned long arg);
#endif

/* Descriptor Initialization */

static void rx65n_txdescinit(FAR struct rx65n_ethmac_s *priv);
static void rx65n_rxdescinit(FAR struct rx65n_ethmac_s *priv);

/* PHY Initialization */

#if defined(CONFIG_NETDEV_PHY_IOCTL) && defined(CONFIG_ARCH_PHY_INTERRUPT)
static void  rx65n_phyintenable(bool enable);
#endif
#if defined(CONFIG_ARCH_PHY_INTERRUPT)
int arch_phy_irq(FAR const char *intf, xcpt_t handler, void *arg,
                 phy_enable_t *enable);
#endif
static int  rx65n_phyinit(FAR struct rx65n_ethmac_s *priv);

/* MAC/DMA Initialization */

static int  rx65n_ethreset(FAR struct rx65n_ethmac_s *priv);
static int  rx65n_macconfig(FAR struct rx65n_ethmac_s *priv);
static void rx65n_macaddress(FAR struct rx65n_ethmac_s *priv);
static int  rx65n_ethconfig(FAR struct rx65n_ethmac_s *priv);

static void rx65n_phy_preamble (void);
static void rx65n_phy_trans_zto0 (void);
static void rx65n_phy_trans_1to0 (void);
static void rx65n_phy_reg_set(uint8_t phydevaddr, uint16_t reg_addr,
                              int32_t option);
static void rx65n_phy_reg_write (uint16_t data);
static void rx65n_phy_mii_write1 (void);
static void rx65n_phy_mii_write0 (void);

void rx65n_ether_enable_icu(void);
void rx65n_power_on_control (void);
void rx65n_ether_set_phy_mode(uint8_t mode);
void rx65n_ether_interrupt_init(void);

static int rx65n_phywrite (uint8_t phydevaddr, uint16_t reg_addr,
                           uint16_t data);
static uint16_t rx65n_phyread (uint8_t phydevaddr, uint16_t reg_addr,
                               uint16_t *value);

void up_enable_irq(int irq);
void up_disable_irq(int irq);

#if defined(CONFIG_ARCH_PHY_INTERRUPT)
struct phylinknotification_t
{
        xcpt_t phandler;
        void *penable;
        struct phy_notify_s *pclient;
};

struct phylinknotification_t phylinknotification =
{
  NULL, NULL, NULL
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rx65n_getreg
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

#ifdef CONFIG_RX65N_EMAC_REGDEBUG
static uint32_t rx65n_getreg(uint32_t addr)
{
  static uint32_t prevaddr = 0;
  static uint32_t preval   = 0;
  static uint32_t count    = 0;

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

          ninfo("[repeats %" PRId32 " more times]\n", count - 3);
        }

      /* Save the new address, value, and count */

      prevaddr = addr;
      preval   = val;
      count    = 1;
    }

  /* Show the register value read */

  ninfo("%08" PRIx32 "->%08" PRIx32 "\n", addr, val);
  return val;
}
#endif

/****************************************************************************
 * Name: rx65n_putreg
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

#if defined(CONFIG_RX65N_EMAC_REGDEBUG) && defined(CONFIG_DEBUG_FEATURES)
static void rx65n_putreg(uint32_t val, uint32_t addr)
{
  /* Show the register value being written */

  ninfo("%08" PRIx32 "<-%08" PRIx32 "\n", addr, val);

  /* Write the value */

  putreg32(val, addr);
}
#endif

/****************************************************************************
 * Name: rx65n_checksetup
 *
 * Description:
 *   Show the state of critical configuration registers.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_RX65N_EMAC_REGDEBUG
static void rx65n_checksetup(void)
{
}
#endif

/****************************************************************************
 * Function: rx65n_initbuffer
 *
 * Description:
 *   Initialize the free buffer list.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called during early driver initialization before Ethernet interrupts
 *   are enabled.
 *
 ****************************************************************************/

static void rx65n_initbuffer(FAR struct rx65n_ethmac_s *priv)
{
  uint8_t *buffer;
  int i;

  /* Initialize the head of the free buffer list */

  sq_init(&priv->freeb);

  /* Add all of the pre-allocated buffers to the free buffer list */

  for (i = 0, buffer = priv->alloc;
       i < RX65N_ETH_NFREEBUFFERS;
       i++, buffer += CONFIG_RX65N_ETH_BUFSIZE)
    {
      sq_addlast((FAR sq_entry_t *)buffer, &priv->freeb);
    }
}

/****************************************************************************
 * Function: rx65n_allocbuffer
 *
 * Description:
 *   Allocate one buffer from the free buffer list.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   Pointer to the allocated buffer on success; NULL on failure
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/

static inline uint8_t *rx65n_allocbuffer(FAR struct rx65n_ethmac_s *priv)
{
  /* Allocate a buffer by returning the head of the free buffer list */

  return (uint8_t *)sq_remfirst(&priv->freeb);
}

/****************************************************************************
 * Function: rx65n_freebuffer
 *
 * Description:
 *   Return a buffer to the free buffer list.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *   buffer - A pointer to the buffer to be freed
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/

static inline void rx65n_freebuffer(FAR struct rx65n_ethmac_s *priv,
                                    uint8_t *buffer)
{
  /* Free the buffer by adding it to the end of the free buffer list */

  sq_addlast((FAR sq_entry_t *)buffer, &priv->freeb);
}

/****************************************************************************
 * Function: rx65n_isfreebuffer
 *
 * Description:
 *   Return TRUE if the free buffer list is not empty.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   True if there are one or more buffers in the free buffer list;
 *   false if the free buffer list is empty
 *
 * Assumptions:
 *   None.
 *
 ****************************************************************************/

static inline bool rx65n_isfreebuffer(FAR struct rx65n_ethmac_s *priv)
{
  /* Return TRUE if the free buffer list is not empty */

  return !sq_empty(&priv->freeb);
}

/****************************************************************************
 * Function: rx65n_transmit
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

static int rx65n_transmit(FAR struct rx65n_ethmac_s *priv)
{
  struct eth_txdesc_s *txdesc;
  struct eth_txdesc_s *txfirst;
  uint32_t regval;

  /* The internal (optimal) network buffer size may be configured to be
   * larger than the Ethernet buffer size.
   */

#if OPTIMAL_ETH_BUFSIZE > CONFIG_RX65N_ETH_BUFSIZE
  uint8_t *buffer;
  int bufcount;
  int lastsize;
  int i;
#endif

  /* Verify that the hardware is ready to send another packet.  If we get
   * here, then we are committed to sending a packet; Higher level logic
   * must have assured that there is no transmission in progress.
   */

  txdesc  = priv->txhead;
  txfirst = txdesc;

  ninfo("d_len: %d d_buf: %p txhead: %p tdes0: %08" PRIx32 "\n",
        priv->dev.d_len, priv->dev.d_buf, txdesc, txdesc->tdes0);

  DEBUGASSERT(txdesc && (txdesc->tdes0 & TACT) == 0);

  /* Is the size to be sent greater than the size of the Ethernet buffer? */

  DEBUGASSERT(priv->dev.d_len > 0 && priv->dev.d_buf != NULL);

  /* Increment statistics */

#if defined(CONFIG_NETDEV_STATISTICS)
  (priv->dev.d_statistics.tx_packets)++;
#endif

#if OPTIMAL_ETH_BUFSIZE > CONFIG_RX65N_ETH_BUFSIZE
  if (priv->dev.d_len > CONFIG_RX65N_ETH_BUFSIZE)
    {
      /* Yes... how many buffers will be need to send the packet? */

      bufcount = (priv->dev.d_len +
                 (CONFIG_RX65N_ETH_BUFSIZE - 1)) / CONFIG_RX65N_ETH_BUFSIZE;
      lastsize = priv->dev.d_len - (bufcount - 1) * CONFIG_RX65N_ETH_BUFSIZE;

      ninfo("bufcount: %d lastsize: %d\n", bufcount, lastsize);

      /* Set the first segment bit in the first TX descriptor */

      txdesc->tdes0 |= TFP1;

      /* Set up all but the last TX descriptor */

      buffer = priv->dev.d_buf;

      for (i = 0; i < bufcount; i++)
        {
          /* This could be a normal event but the design does not handle it */

          DEBUGASSERT((txdesc->tdes0 & TACT) == 0);

          /* Set the Buffer1 address pointer */

          txdesc->tdes2 = (uint32_t)buffer;

          /* Set the buffer size in all TX descriptors */

          if (i == (bufcount - 1))
            {
              /* This is the last segment.  Set the last segment bit in the
               * last TX descriptor and ask for an interrupt when this
               * segment transfer completes.
               */

              txdesc->tdes0 |= (TFP0 | TWBI);

              /* This segment is, most likely, of fractional buffersize */

              txdesc->tdes1  = (lastsize << 16);
              buffer        += lastsize;
            }
          else
            {
              /* This is not the last segment.  We don't want an interrupt
               * when this segment transfer completes.
               */

              txdesc->tdes0 &= ~TWBI;

              /* The size of the transfer is the whole buffer */

              txdesc->tdes1  = (CONFIG_RX65N_ETH_BUFSIZE << 16);
              buffer        += CONFIG_RX65N_ETH_BUFSIZE;
            }

          /* Give the descriptor to DMA */

          txdesc->tdes0 |= TACT;
          regval  = rx65n_getreg(RX65N_ETHD_EDTRR);
          regval |= (ETHD_EDRRR_TR);
          rx65n_putreg(regval, RX65N_ETHD_EDTRR);

          txdesc = (struct eth_txdesc_s *)txdesc->tdes3;
        }
    }
  else
#endif
    {
      /* The single descriptor is both the first and last segment.  And we do
       * want an interrupt when the transfer completes.
       */

      txdesc->tdes0 |= (TFP0 | TFP1 | TWBI);

      /* Set frame size */

      DEBUGASSERT(priv->dev.d_len <= CONFIG_NET_ETH_PKTSIZE);
      txdesc->tdes1 = ((priv->dev.d_len) << 16);

      /* Set the Buffer1 address pointer */

      txdesc->tdes2 = (uint32_t)priv->dev.d_buf;

      /* Set OWN bit of the TX descriptor tdes0.  This gives the buffer to
       * Ethernet DMA
       */

      txdesc->tdes0 |= TACT;
      regval  = rx65n_getreg(RX65N_ETHD_EDTRR);
      regval |= (ETHD_EDRRR_TR);
      rx65n_putreg(regval, RX65N_ETHD_EDTRR);

      /* Point to the next available TX descriptor */

      txdesc = (struct eth_txdesc_s *)txdesc->tdes3;
    }

  /* Remember where we left off in the TX descriptor chain */

  priv->txhead = txdesc;

  /* Detach the buffer from priv->dev structure.  That buffer is now
   * "in-flight".
   */

  priv->dev.d_buf = NULL;
  priv->dev.d_len = 0;

  /* If there is no other TX buffer, in flight, then remember the location
   * of the TX descriptor.  This is the location to check for TX done events.
   */

  if (!priv->txtail)
    {
      DEBUGASSERT(priv->inflight == 0);
      priv->txtail = txfirst;
    }

  /* Increment the number of TX transfer in-flight */

  priv->inflight++;

  ninfo("txhead: %p txtail: %p inflight: %d\n",
        priv->txhead, priv->txtail, priv->inflight);

  /* If all TX descriptors are in-flight, then we
   * have to disable receive interrupts
   * too.  This is because receive events
   * can trigger more un-stoppable transmit
   * events.
   */

  if (priv->inflight >= CONFIG_RX65N_ETH_NTXDESC)
    {
      rx65n_disableint(priv, ETHD_EESIPR_FRIP);
    }

  /* Enable TX interrupts */

  rx65n_enableint(priv, ETHD_EESIPR_TCIP);

  /* Setup the TX timeout (perhaps restarting the timer) */

  rx65n_cmtw0_start(rx65n_cmtw0_timeout,
                    RX65N_CMTW0_COUNT_VALUE_FOR_TXTIMEOUT);
  return OK;
}

/****************************************************************************
 * Function: rx65n_txpoll
 *
 * Description:
 *   The transmitter is available, check if the network
 *   has any outgoing packets ready
 *   to send.  This is a callback from devif_poll().
 *    devif_poll() may be called:
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

static int rx65n_txpoll(struct net_driver_s *dev)
{
  FAR struct rx65n_ethmac_s *priv = (FAR struct rx65n_ethmac_s *)dev->
                                     d_private;

  DEBUGASSERT(priv->dev.d_buf != NULL);

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

          rx65n_transmit(priv);
          DEBUGASSERT(dev->d_len == 0 && dev->d_buf == NULL);

          /* Check if the next TX descriptor is owned by the Ethernet DMA or
           * CPU. We cannot perform the TX poll if we are unable to accept
           * another packet fo transmission.
           *
           * In a race condition, TACT may be cleared BUT still not available
           * because rx65n_freeframe() has not yet run. If rx65n_freeframe()
           * has run, the buffer1 pointer (tdes2) will be nullified (and
           * inflight should be CONFIG_RX65N_ETH_NTXDESC).
           */

          if ((priv->txhead->tdes0 & TACT) != 0 ||
               priv->txhead->tdes2 != 0)
            {
              /* We have to terminate the poll if we have no more descriptors
               * available for another transfer.
               */

              return -EBUSY;
            }

          /* We have the descriptor, we can continue the poll. Allocate a new
           * buffer for the poll.
           */

          dev->d_buf = rx65n_allocbuffer(priv);

          /* We can't continue the poll if we have no buffers */

          if (dev->d_buf == NULL)
            {
              /* Terminate the poll. */

              return -ENOMEM;
            }
        }
    }

  /* If zero is returned, the polling will continue until all connections
   * have been examined.
   */

  return 0;
}

/****************************************************************************
 * Function: rx65n_dopoll
 *
 * Description:
 *   The function is called in order to perform an out-of-sequence TX poll.
 *   This is done:
 *
 *   1. After completion of a transmission (rx65n_txdone),
 *   2. When new TX data is available (rx65n_txavail_process), and
 *   3. After a TX timeout to restart the sending process
 *      (rx65n_txtimeout_process).
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

static void rx65n_dopoll(FAR struct rx65n_ethmac_s *priv)
{
  FAR struct net_driver_s *dev = &priv->dev;

  /* Check if the next TX descriptor is owned by the Ethernet DMA or
   * CPU.  We cannot perform the TX poll if we are unable to accept
   * another packet for transmission.
   *
   * In a race condition, TACT may be cleared BUT still not available
   * because rx65n_freeframe() has not yet run. If rx65n_freeframe()
   * has run, the buffer1 pointer (tdes2) will be nullified (and
   * inflight should be < CONFIG_RX65N_ETH_NTXDESC).
   */

  if ((priv->txhead->tdes0 & TACT) == 0 &&
       priv->txhead->tdes2 == 0)
    {
      /* If we have the descriptor, then poll the network for new XMIT data.
       * Allocate a buffer for the poll.
       */

      DEBUGASSERT(dev->d_len == 0 && dev->d_buf == NULL);
      dev->d_buf = rx65n_allocbuffer(priv);

      /* We can't poll if we have no buffers */

      if (dev->d_buf)
        {
          devif_poll(dev, rx65n_txpoll);

          /* We will, most likely end up with a buffer to be freed.  But it
           * might not be the same one that we allocated above.
           */

          if (dev->d_buf)
            {
              DEBUGASSERT(dev->d_len == 0);
              rx65n_freebuffer(priv, dev->d_buf);
              dev->d_buf = NULL;
            }
        }
    }
}

/****************************************************************************
 * Function: rx65n_enableint
 *
 * Description:
 *   Enable interrupt
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 ****************************************************************************/

static void rx65n_enableint(FAR struct rx65n_ethmac_s *priv, uint32_t ierbit)
{
  uint32_t regval;

  /* ETHERC/EDMAC enabling Status Interrupt */

  regval  = rx65n_getreg(RX65N_ETHD_EESIPR);
  regval |= (ierbit); /* enabling ETHD_EESIPR_FRIP or ETHD_EESIPR_TCIP */

  rx65n_putreg(regval, RX65N_ETHD_EESIPR);
}

/****************************************************************************
 * Function: rx65n_disableint
 *
 * Description:
 *   Disable a normal interrupt.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 ****************************************************************************/

static void rx65n_disableint(FAR struct rx65n_ethmac_s *priv,
                             uint32_t ierbit)
{
  uint32_t regval;

  /* ETHERC status interrupt request is disabled */

  regval  = rx65n_getreg(RX65N_ETHD_EESIPR);
  regval &= (~(ierbit)); /* setting ETHD_EESIPR_FRIP or ETHD_EESIPR_TCIP */

  rx65n_putreg(regval, RX65N_ETHD_EESIPR);
}

/****************************************************************************
 * Function: rx65n_freesegment
 *
 * Description:
 *   The function is called when a frame is received using the DMA receive
 *   interrupt.  It scans the RX descriptors to the received frame.
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

static void rx65n_freesegment(FAR struct rx65n_ethmac_s *priv,
                              FAR struct eth_rxdesc_s *rxfirst, int segments)
{
  struct eth_rxdesc_s *rxdesc;
  int i;
  uint32_t regval;

  ninfo("rxfirst: %p segments: %d\n", rxfirst, segments);

  /* Set RACT bit in RX descriptors.  This gives the buffers back to EDMA */

  rxdesc = rxfirst;
  for (i = 0; i < segments; i++)
    {
      /* Check last descriptor */

      if (rxdesc->rdes0 & RDLE)
        {
          rxdesc->rdes0 |= RACT;
        }
      else
        {
          rxdesc->rdes0 = RACT;
        }

      rxdesc = (struct eth_rxdesc_s *)rxdesc->rdes3;
    }

  /* Resume transmission if stopped */

  if (!(rx65n_getreg(RX65N_ETHD_EDRRR)))
    {
      regval  = rx65n_getreg(RX65N_ETHD_EDRRR);
      regval |= (ETHD_EDRRR_RR);
      rx65n_putreg(regval, RX65N_ETHD_EDRRR);
    }

  /* Reset the segment management logic */

  priv->rxcurr   = NULL;
  priv->segments = 0;
}

/****************************************************************************
 * Function: rx65n_recvframe
 *
 * Description:
 *   The function is called when a frame is received using the DMA receive
 *   interrupt.  It scans the RX descriptors of the received frame.
 *
 *   NOTE: This function will silently discard any packets containing errors.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   OK if a packet was successfully returned; -EAGAIN if there are no
 *   further packets available
 *
 * Assumptions:
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static int rx65n_recvframe(FAR struct rx65n_ethmac_s *priv)
{
  struct eth_rxdesc_s *rxdesc;
  struct eth_rxdesc_s *rxcurr;
  uint8_t *buffer;
  int i;

  ninfo("rxhead: %p rxcurr: %p segments: %d\n",
        priv->rxhead, priv->rxcurr, priv->segments);

  /* Check if there are free buffers.  We cannot receive new frames in
   * this design unless there is at least one free buffer.
   */

  if (!rx65n_isfreebuffer(priv))
    {
      nerr("ERROR: No free buffers\n");
      return -ENOMEM;
    }

  /* Scan descriptors owned by the CPU.  Scan until:
   *
   *   1) We find a descriptor still owned by the DMA,
   *   2) We have examined all of the RX descriptors, or
   *   3) All of the TX descriptors are in flight.
   *
   * This last case is obscure.  It is due to that fact that each packet
   * that we receive can generate an unstoppable transmisson.  So we have
   * to stop receiving when we can not longer transmit.  In this case, the
   * transmit logic should also have disabled further RX interrupts.
   */

  rxdesc = priv->rxhead;
  for (i = 0;
       (rxdesc->rdes0 & RACT) == 0 &&
        i < CONFIG_RX65N_ETH_NRXDESC &&
        priv->inflight < CONFIG_RX65N_ETH_NTXDESC;
       i++)
    {
      /* Check if this is the first segment in the frame */

      if ((rxdesc->rdes0 & RFP1) != 0 &&
          (rxdesc->rdes0 & RFP0) == 0)
        {
          priv->rxcurr   = rxdesc;
          priv->segments = 1;
        }

      /* Check if this is an intermediate segment in the frame */

      else if (((rxdesc->rdes0 & RFP0) == 0) &&
               ((rxdesc->rdes0 & RFP1) == 0))
        {
          priv->segments++;
        }

      /* Otherwise, it is the last segment in the frame */

      else
        {
          priv->segments++;

          /* Check if there is only one segment in the frame */

          if (priv->segments == 1)
            {
              rxcurr = rxdesc;
            }
          else
            {
              rxcurr = priv->rxcurr;
            }

          ninfo("rxhead: %p rxcurr: %p segments: %d\n",
              priv->rxhead, priv->rxcurr, priv->segments);

          /* Check if any errors are reported in the frame */

          if ((rxdesc->rdes0 & RFE) == 0)
            {
              struct net_driver_s *dev = &priv->dev;

              /* Get the Frame Length of the received packet: substruct 4
               * bytes of the CRC
               */

              dev->d_len = ((rxdesc->rdes1 & 0x0000ffff));

              /* Get a buffer from the free list.  We don't even check if
               * this is successful because we already assure the free
               * list is not empty above.
               */

              buffer = rx65n_allocbuffer(priv);

              /* Take the buffer from the RX descriptor of the first free
               * segment, put it into the network device structure,
               * then replace the buffer in the RX descriptor with the
               * newly allocated buffer.
               */

              DEBUGASSERT(dev->d_buf == NULL);
              dev->d_buf    = (uint8_t *)rxcurr->rdes2;
              rxcurr->rdes2 = (uint32_t)buffer;

              /* Increment statistics */

#if defined(CONFIG_NETDEV_STATISTICS)
  (priv->dev.d_statistics.rx_packets)++;
#endif

              /* Return success, remembering where we should re-start
               * scanning and resetting the segment scanning logic
               */

              priv->rxhead   = (struct eth_rxdesc_s *)rxdesc->rdes3;
              rx65n_freesegment(priv, rxcurr, priv->segments);

              ninfo("rxhead: %p d_buf: %p d_len: %d\n",
                    priv->rxhead, dev->d_buf, dev->d_len);

              return OK;
            }
          else
            {
              /* Increment statistics */

#if defined(CONFIG_NETDEV_STATISTICS)
  (priv->dev.d_statistics.rx_errors)++;
#endif

              /* Drop the frame that contains the errors, reset the segment
               * scanning logic, and continue scanning with the next frame.
               */

              nerr("ERROR: Dropped, RX descriptor errors: %08" PRIx32 "\n",
                                rxdesc->rdes0);
              rx65n_freesegment(priv, rxcurr, priv->segments);
            }
        }

      /* Try the next descriptor */

      rxdesc = (struct eth_rxdesc_s *)rxdesc->rdes3;
    }

  /* We get here after all of the descriptors have been scanned or when
   * rxdesc points to the first descriptor owned by the DMA. Remember
   * where we left off.
   */

  priv->rxhead = rxdesc;

  ninfo("rxhead: %p rxcurr: %p segments: %d\n",
        priv->rxhead, priv->rxcurr, priv->segments);

  return -EAGAIN;
}

/****************************************************************************
 * Function: rx65n_receive
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

static void rx65n_receive(FAR struct rx65n_ethmac_s *priv)
{
  struct net_driver_s *dev = &priv->dev;

  /* Loop while while rx65n_recvframe() successfully retrieves valid
   * Ethernet frames.
   */

  while (rx65n_recvframe(priv) == OK)
    {
#ifdef CONFIG_NET_PKT

      /* When packet sockets are enabled, feed the frame into the packet
       * tap
       */

     pkt_input(&priv->dev);
#endif

      /* Check if the packet is a valid size for the network
       * buffer configuration (this should not happen)
       */

      if (dev->d_len > CONFIG_NET_ETH_PKTSIZE)
        {
          nerr("ERROR: Dropped, Too big: %d\n", dev->d_len);

          /* Free dropped packet buffer */

          if (dev->d_buf)
            {
              rx65n_freebuffer(priv, dev->d_buf);
              dev->d_buf = NULL;
              dev->d_len = 0;
            }

          continue;
        }

      /* We only accept IP packets of the configured type and ARP packets */

#ifdef CONFIG_NET_IPv4
      if (BUF->type == HTONS(ETHTYPE_IP))
        {
          ninfo("IPv4 frame\n");

          /* Handle ARP on input then give the IPv4 packet to the network
           * layer
           */

          /* Increment statistics */

#if defined(CONFIG_NETDEV_STATISTICS)
  (priv->dev.d_statistics.rx_ipv4)++;
#endif

          arp_ipin(&priv->dev);
          ipv4_input(&priv->dev);

          /* If the above function invocation resulted in data
           * that should be
           * sent out on the network, the field  d_len will set
           * to a value > 0.
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

              rx65n_transmit(priv);
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

          /* If the above function invocation resulted in data
           * that should be sent out on the network, the field
           * d_len will set to a value > 0.
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

              rx65n_transmit(priv);
            }
        }
      else
#endif
#ifdef CONFIG_NET_ARP
  if (BUF->type == HTONS(ETHTYPE_ARP))
        {
          ninfo("ARP frame\n");

          /* Increment statistics */

#if defined(CONFIG_NETDEV_STATISTICS)
          (priv->dev.d_statistics.rx_arp)++;
#endif

          /* Handle ARP packet */

          arp_arpin(&priv->dev);

          /* If the above function invocation resulted in data
           * that should be sent out on the network, the field
           * d_len will set to a value > 0.
           */

          if (priv->dev.d_len > 0)
            {
              rx65n_transmit(priv);
            }
        }
      else
#endif
        {
          nerr("ERROR: Dropped, Unknown type: %04x\n", BUF->type);
#if defined(CONFIG_NETDEV_STATISTICS)
                  (priv->dev.d_statistics.rx_dropped)++;
#endif
        }

      /* We are finished with the RX buffer.  NOTE:  If the buffer is
       * re-used for transmission, the dev->d_buf field will have been
       * nullified.
       */

      if (dev->d_buf)
        {
          /* Free the receive packet buffer */

          rx65n_freebuffer(priv, dev->d_buf);
          dev->d_buf = NULL;
          dev->d_len = 0;
        }
    }
}

/****************************************************************************
 * Function: rx65n_freeframe
 *
 * Description:
 *   Scans the TX descriptors and frees the buffers of completed TX
 *   transfers.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static void rx65n_freeframe(FAR struct rx65n_ethmac_s *priv)
{
  struct eth_txdesc_s *txdesc;
  int i;

  ninfo("txhead: %p txtail: %p inflight: %d\n",
        priv->txhead, priv->txtail, priv->inflight);

  /* Scan for "in-flight" descriptors owned by the CPU */

  txdesc = priv->txtail;
  if (txdesc)
    {
      DEBUGASSERT(priv->inflight > 0);

      for (i = 0; (txdesc->tdes0 & TACT) == 0; i++)
        {
          /* There should be a buffer assigned to all in-flight
           * TX descriptors.
           */

          ninfo("txtail: %p tdes0: %08" PRIx32
                " tdes2: %08" PRIx32 " tdes3: %08" PRIx32 "\n",
                txdesc, txdesc->tdes0, txdesc->tdes2, txdesc->tdes3);

          DEBUGASSERT(txdesc->tdes2 != 0);

          /* Check if this is the first segment of a TX frame. */

          if ((txdesc->tdes0 & TFP1) != 0)
            {
              /* Yes.. Free the buffer */

              rx65n_freebuffer(priv, (uint8_t *)txdesc->tdes2);
            }

          /* In any event, make sure that TDES2 is nullified. */

          txdesc->tdes2 = 0;

          /* Check if this is the last segment of a TX frame */

          if ((txdesc->tdes0 & TFP0) != 0)
            {
              /* Yes.. Decrement the number of frames "in-flight". */

              priv->inflight--;

              /* If all of the TX descriptors were in-flight,
               * then RX interrupts may have been disabled...
               * we can re-enable them now.
               */

              /* Need to check this and update the arguments of the
               * rx65n_enableint function
               */

              rx65n_enableint(priv, ETHD_EESIPR_FRIP);

              /* If there are no more frames in-flight, then bail. */

              if (priv->inflight <= 0)
                {
                  priv->txtail   = NULL;
                  priv->inflight = 0;
                  return;
                }
            }

          /* Try the next descriptor in the TX chain */

          txdesc = (struct eth_txdesc_s *)txdesc->tdes3;
        }

      /* We get here if (1) there are still frames "in-flight". Remember
       * where we left off.
       */

      priv->txtail = txdesc;

      ninfo("txhead: %p txtail: %p inflight: %d\n",
            priv->txhead, priv->txtail, priv->inflight);
    }
}

/****************************************************************************
 * Function: rx65n_txdone
 *
 * Description:
 *   An interrupt was received indicating that the last TX packet
 *   transfer(s) are complete.
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

static void rx65n_txdone(FAR struct rx65n_ethmac_s *priv)
{
  DEBUGASSERT(priv->txtail != NULL);

  /* Scan the TX descriptor change, returning buffers to free list */

  rx65n_freeframe(priv);

  /* If no further xmits are pending, then cancel the TX timeout */

  if (priv->inflight <= 0)
    {
      /* Cancel the TX timeout */

          rx65n_cmtw0_stop(rx65n_cmtw0_timeout);

      /* And disable further TX interrupts. */

      rx65n_disableint(priv, ETHD_EESIPR_TCIP);
    }

  /* Then poll the network for new XMIT data */

  rx65n_dopoll(priv);
}

/****************************************************************************
 * Function: rx65n_interrupt_work
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

static void rx65n_interrupt_work(FAR void *arg)
{
  FAR struct rx65n_ethmac_s *priv = (FAR struct rx65n_ethmac_s *)arg;
  uint32_t edmasr;
  uint32_t regval;

  uint32_t ethsr;
#if defined(CONFIG_ARCH_PHY_INTERRUPT)
  uint16_t phyreg;
  int phyreg_read_status;
  int irqno = 0;
#endif
  DEBUGASSERT(priv);

  /* Process pending Ethernet interrupts */

  net_lock();

  /* Check and clear the Link Signal Change Flag */

  ethsr  = rx65n_getreg(RX65N_ETH_ECSR);
  ethsr  &= rx65n_getreg(RX65N_ETH_ECSIPR);
  if ((ethsr & ETH_ECSR_LCHNG) != 0)
    {
      regval  = rx65n_getreg(RX65N_ETH_ECSR);
      regval |= ((ETH_ECSR_LCHNG)); /* Write 1 to clear flag */
      rx65n_putreg(regval, RX65N_ETH_ECSR);

#if defined(CONFIG_ARCH_PHY_INTERRUPT)
    phyreg_read_status = rx65n_phyread(CONFIG_RX65N_EMAC0_PHYADDR,
                                         PHY_STS_READ_REG, &phyreg);
      if (OK == phyreg_read_status)
        {
          regval = (uint32_t)(phyreg & PHY_STS_BIT_MASK)
                    >> PHY_STS_SHIFT_COUNT;
        }

      if (regval != priv->prevlinkstatus) /* Check link status by 0th bit */
        {
          /* Link UP or DOWN status */

          if (phylinknotification.phandler != NULL)
            {
              phylinknotification.phandler(irqno, (FAR void *)NULL,
              (FAR void *)phylinknotification.pclient);
              priv->prevlinkstatus = regval;
            }
        }
#endif
    }

  /* Get the interrupt status bits (ETHERC/EDMAC interrupt status check ) */

  edmasr = rx65n_getreg(RX65N_ETHD_EESR);

  /* Mask only enabled interrupts.  This depends on the
   * fact that the interrupt
   * related bits (0-16) correspond in these two registers.
   */

  edmasr &= rx65n_getreg(RX65N_ETHD_EESIPR);

  /* Yes.. Check if we received an incoming packet,
   * if so, call
   * rx65n_receive()
   */

  if ((edmasr & ETHD_EESR_FR) != 0)
    {
      /* Clear the pending receive interrupt */

      rx65n_putreg(ETHD_EESR_FR, RX65N_ETHD_EESR);

      /* Handle the received package */

      rx65n_receive(priv);
    }

  /* Check if a packet transmission just completed.
   * If so, call rx65n_txdone(). This may disable further
   * TX interrupts if there are no pending transmissions.
   */

  if ((edmasr & ETHD_EESR_TC) != 0)
    {
      /* Clear the pending transmit interrupt */

      rx65n_putreg(ETHD_EESR_TC, RX65N_ETHD_EESR);

      /* Increment statistics */

#if defined(CONFIG_NETDEV_STATISTICS)
  (priv->dev.d_statistics.tx_done)++;
#endif

      /* Check if there are pending transmissions */

      rx65n_txdone(priv);
    }

  net_unlock();

  /* Re-enable Ethernet interrupts */

  up_enable_irq(RX65N_ETH_IRQ);
}

/****************************************************************************
 * Function: rx65n_interrupt
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

static int rx65n_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct rx65n_ethmac_s *priv = &g_rx65nethmac[0];
  uint32_t edmasr;

  /* Get the interrupt status bits (ETHERC/EDMAC interrupt status check ) */

  edmasr = rx65n_getreg(RX65N_ETHD_EESR);
  if (edmasr != 0)
    {
      /* Disable further Ethernet interrupts.  Because Ethernet interrupts
       * are also disabled if the TX timeout event occurs, there can be no
       * race condition here.
       */

      up_disable_irq(RX65N_ETH_IRQ);

      /* Check if a packet transmission just completed. */

      if ((edmasr & ETHD_EESR_TC) != 0)
        {
          /* If a TX transfer just completed, then cancel the TX timeout so
           * there will be no race condition between any subsequent timeout
           * expiration and the deferred interrupt processing.
           */

          rx65n_cmtw0_stop(rx65n_cmtw0_timeout);
        }

      /* Schedule to perform the interrupt processing on the worker thread. */

      work_queue(ETHWORK, &priv->irqwork, rx65n_interrupt_work, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Function: rx65n_txtimeout_work
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

static void rx65n_txtimeout_work(FAR void *arg)
{
  FAR struct rx65n_ethmac_s *priv = (FAR struct rx65n_ethmac_s *)arg;

  /* Reset the hardware.Just take the interface down, then back up again. */

  net_lock();

  /* Increment statistics */

#if defined(CONFIG_NETDEV_STATISTICS)
  (priv->dev.d_statistics.tx_errors)++;
  (priv->dev.d_statistics.tx_timeouts)++;
#endif

  rx65n_ifdown(&priv->dev);
  rx65n_ifup(&priv->dev);

  /* Then poll for new XMIT data */

  rx65n_dopoll(priv);
  net_unlock();
}

/****************************************************************************
 * Function: rx65n_txtimeout_expiry
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

void rx65n_txtimeout_expiry(wdparm_t arg)
{
  FAR struct rx65n_ethmac_s *priv = (FAR struct rx65n_ethmac_s *)arg;
  nerr("ERROR: Timeout!\n");

  /* Disable further Ethernet interrupts.  This will prevent some race
   * conditions with interrupt work.  There is still a potential race
   * condition with interrupt work that is already queued and in progress.
   *
   * Interrupts will be re-enabled when rx65n_ifup() is called.
   */

  up_disable_irq(RX65N_ETH_IRQ);

  /* Schedule to perform the TX timeout processing on the worker thread,
   * perhaps canceling any pending IRQ processing.
   */

  work_queue(ETHWORK, &priv->irqwork, rx65n_txtimeout_work, priv, 0);
}

/****************************************************************************
 * Function: rx65n_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the Ethernet interface when an IP address is
 *   provided
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   Zero is returned on success; a negated errno value is returned on any
 *   failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int rx65n_ifup(struct net_driver_s *dev)
{
  FAR struct rx65n_ethmac_s *priv = (FAR struct rx65n_ethmac_s *)dev->
                                     d_private;
  int ret;

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

  /* Configure the Ethernet interface for DMA operation. */

  ret = rx65n_ethconfig(priv);
  if (ret < 0)
    {
      return ret;
    }

  priv->ifup = true;

  /* Enable the Ethernet interrupt */

  up_enable_irq(RX65N_ETH_IRQ);

  priv->prevlinkstatus = ETHER_LINKUP;
  rx65n_checksetup();
  return OK;
}

/****************************************************************************
 * Function: rx65n_ifdown
 *
 * Description:
 *   NuttX Callback: Stop the interface.
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   Returns zero on success; a negated errno value is returned on any
 *   failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int rx65n_ifdown(struct net_driver_s *dev)
{
  FAR struct rx65n_ethmac_s *priv = (FAR struct rx65n_ethmac_s *)dev->
                                                d_private;
  irqstate_t flags;
  int ret = OK;
  ninfo("Taking the network down\n");
  flags = enter_critical_section();

  /* Disable the Ethernet interrupt */

  up_disable_irq(RX65N_ETH_IRQ);

  /* Cancel the TX timeout timers */

  rx65n_cmtw0_stop(rx65n_cmtw0_timeout);

  /* Put the EMAC in its reset, non-operational state.
   * This should be a known configuration that will guarantee
   * the rx65n_ifup() always successfully brings the interface back up.
   */

  ret = rx65n_ethreset(priv);
  if (ret < 0)
    {
      nerr("ERROR: rx65n_ethreset failed (timeout), "
           "still assuming it's going down.\n");
    }

  /* Mark the device "down" */

  priv->ifup = false;

  priv->prevlinkstatus = ETHER_LINKDOWN;

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Function: rx65n_txavail_work
 *
 * Description:
 *   Perform an out-of-cycle poll on the worker thread.
 *
 * Input Parameters:
 *   arg  - Reference to the NuttX driver state structure (cast to void*)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called on the higher priority worker thread.
 *
 ****************************************************************************/

static void rx65n_txavail_work(FAR void *arg)
{
  FAR struct rx65n_ethmac_s *priv = (FAR struct rx65n_ethmac_s *)arg;

  ninfo("ifup: %d\n", priv->ifup);

  /* Ignore the notification if the interface is not yet up */

  net_lock();
  if (priv->ifup)
    {
      /* Poll the network for new XMIT data */

      rx65n_dopoll(priv);
    }

  net_unlock();
}

/****************************************************************************
 * Function: rx65n_txavail
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

static int rx65n_txavail(struct net_driver_s *dev)
{
  FAR struct rx65n_ethmac_s *priv = (FAR struct rx65n_ethmac_s *)dev->
                                               d_private;

  /* Is our single work structure available?
   * It may not be if there are
   * pending interrupt actions and we will have to ignore the Tx
   * availability action.
   */

  if (work_available(&priv->pollwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      work_queue(ETHWORK, &priv->pollwork, rx65n_txavail_work, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Function: rx65n_addmac
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

#if defined(CONFIG_NET_MCASTGROUP) || defined(CONFIG_NET_ICMPv6)
static int rx65n_addmac(struct net_driver_s *dev, FAR const uint8_t *mac)
{
  ninfo("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  /* RX65N do not support add on mac multi cast feature because related
   * hash table mac multi cast register is not supported in RX65N.
   */

  return OK;
}
#endif /* CONFIG_NET_MCASTGROUP || CONFIG_NET_ICMPv6 */

/****************************************************************************
 * Function: rx65n_rmmac
 *
 * Description:
 *   NuttX Callback: Remove the specified MAC address from the
 *   hardware multicast
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

#ifdef CONFIG_NET_MCASTGROUP
static int rx65n_rmmac(struct net_driver_s *dev, FAR const uint8_t *mac)
{
  ninfo("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  /* RX65N do not support add on mac multi cast feature because related
   * hash table multi cast mac register not available. This function is
   * not need to implement.
   */

  return OK;
}
#endif

/****************************************************************************
 * Function: rx65n_txdescinit
 *
 * Description:
 *   Initializes the DMA TX descriptors in chain mode.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void rx65n_txdescinit(FAR struct rx65n_ethmac_s *priv)
{
  struct eth_txdesc_s *txdesc;
  int i;

  /* priv->txhead will point to the first, available TX descriptor in
   * the chain.
   * Set the priv->txhead pointer to the first descriptor in the table.
   */

  priv->txhead = priv->txtable;

  /* priv->txtail will point to the first segment
   * of the oldest pending "in-flight" TX transfer.
   * NULL means that there are no active TX transfers.
   */

  priv->txtail   = NULL;
  priv->inflight = 0;

  /* Initialize each TX descriptor */

  for (i = 0; i < CONFIG_RX65N_ETH_NTXDESC; i++)
    {
      txdesc = &priv->txtable[i];

      /* Set Second Address Chained bit */

      txdesc->tdes0 = 0x00000000; /* All bits are cleared initially */

      /* Clear Buffer1 address pointer
       * (buffers will be assigned as they are used)
       */

      txdesc->tdes1 = (uint32_t)(1 << 16);
      txdesc->tdes2 = 0;

      /* Initialize the next descriptor with the Next Descriptor Polling
       * Enable
       */

      if (i < (CONFIG_RX65N_ETH_NTXDESC - 1))
        {
          /* Set next descriptor address register with
           * next descriptor base address
           */

          txdesc->tdes3 = (uint32_t)&priv->txtable[i + 1];
        }
      else
        {
          /* For last descriptor, set next descriptor
           * address register equal to the first descriptor base address
           */

            txdesc->tdes0 = TDLE; /* When this bit is 1, it indicates that this descriptor is the last descriptor of the descriptor list. */

            txdesc->tdes3 = (uint32_t)priv->txtable;
        }
    }

  /* Set Transmit Descriptor List Address Register */

  rx65n_putreg((uint32_t)priv->txtable, RX65N_ETHD_TDLAR);
}

/****************************************************************************
 * Function: rx65n_rxdescinit
 *
 * Description:
 *   Initializes the DMA RX descriptors in chain mode.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void rx65n_rxdescinit(FAR struct rx65n_ethmac_s *priv)
{
  struct eth_rxdesc_s *rxdesc;
  int i;

  /* priv->rxhead will point to the first,
   * RX descriptor in the chain.
   * This will be where we receive the first incomplete frame.
   */

  priv->rxhead = priv->rxtable;

  /* If we accumulate the frame in segments, priv->rxcurr points to the
   * RX descriptor of the first segment in the current TX frame.
   */

  priv->rxcurr   = NULL;
  priv->segments = 0;

  /* Initialize each RX descriptor */

  for (i = 0; i < CONFIG_RX65N_ETH_NRXDESC; i++)
    {
      rxdesc = &priv->rxtable[i];

      /* Set Own bit of the RX descriptor rdes0 */

      rxdesc->rdes0 = RACT;  /* Indicates that Descriptor is valid */

      /* Set Buffer1 size RX desc receive interrupt */

      rxdesc->rdes1 = (uint32_t)(CONFIG_RX65N_ETH_BUFSIZE << 16);

      /* Set Buffer1 address pointer */

      rxdesc->rdes2 = (uint32_t)&priv->rxbuffer[i*CONFIG_RX65N_ETH_BUFSIZE];

      /* Initialize the next descriptor with the Next Descriptor Polling
       * Enable
       */

      if (i < (CONFIG_RX65N_ETH_NRXDESC - 1))
        {
          /* Set next descriptor address register with next descriptor base
           * address
           */

          rxdesc->rdes3 = (uint32_t)&priv->rxtable[i + 1];
        }
      else
        {
          /* For last descriptor, set next descriptor address register equal
           * to the first descriptor base address
           */

          rxdesc->rdes0 |= RDLE;  /* Indicates that Descriptor is last descriptor valid */

          rxdesc->rdes3 = (uint32_t)priv->rxtable;
        }
    }

  /* Set Receive Descriptor List Address Register */

  rx65n_putreg((uint32_t)priv->rxtable, RX65N_ETHD_RDLAR);
}

/****************************************************************************
 * Function: rx65n_ioctl
 *
 * Description:
 *  Executes the SIOCxMIIxxx command and responds using the request struct
 *  that must be provided as its 2nd parameter.
 *
 *  When called with SIOCGMIIPHY it will get the PHY address for the device
 *  and write it to the req->phy_id field of the request struct.
 *
 *  When called with SIOCGMIIREG it will read a register of the PHY that is
 *  specified using the req->reg_no struct field and then write its output
 *  to the req->val_out field.
 *
 *  When called with SIOCSMIIREG it will write to a register of the PHY that
 *  is specified using the req->reg_no struct field and use req->val_in as
 *  its input.
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
static int rx65n_ioctl(struct net_driver_s *dev, int cmd, unsigned long arg)
{
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

              rx65n_phyintenable(true);
            }
        }
        break;
#endif

      case SIOCGMIIPHY: /* Get MII PHY address */
        {
          struct mii_ioctl_data_s *req =
                 (struct mii_ioctl_data_s *)((uintptr_t)arg);
          req->phy_id = CONFIG_RX65N_EMAC0_PHYADDR;
          ret = OK;
        }
        break;

      case SIOCGMIIREG: /* Get register from MII PHY */
        {
          struct mii_ioctl_data_s *req =
                 (struct mii_ioctl_data_s *)((uintptr_t)arg);
          ret = rx65n_phyread(req->phy_id, req->reg_num, &req->val_out);
        }
        break;

      case SIOCSMIIREG: /* Set register in MII PHY */
        {
          struct mii_ioctl_data_s *req =
                 (struct mii_ioctl_data_s *)((uintptr_t)arg);
          ret = rx65n_phywrite(req->phy_id, req->reg_num, req->val_in);
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
 * Function: rx65n_phyintenable
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
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_NETDEV_PHY_IOCTL) && defined(CONFIG_ARCH_PHY_INTERRUPT)
static void rx65n_phyintenable(bool enable)
{
  uint32_t regval;
  regval  = rx65n_getreg(RX65N_ETH_ECSR);
  regval |= ((ETH_ECSR_LCHNG)); /* Write 1 to clear flag */
  rx65n_putreg(regval, RX65N_ETH_ECSR);

  if (enable)
    {
    }
  else
    {
    }
}
#endif

/****************************************************************************
 * Name: arch_phy_irq
 *
 * Description:
 *   This function may be called to register an interrupt handler that will
 *   be called when a PHY interrupt occurs.  This function both attaches
 *   the interrupt handler and enables the interrupt if 'handler' is non-
 *   NULL.  If handler is NULL, then the interrupt is detached and disabled
 *   instead.
 *
 *   The PHY interrupt is always disabled upon return.  The caller must
 *   call back through the enable function point to control the state of
 *   the interrupt.
 *
 *   This interrupt may or may not be available on a given platform depending
 *   on how the network hardware architecture is implemented.  In a typical
 *   case, the PHY interrupt is provided to board-level logic as a GPIO
 *   interrupt (in which case this is a board-specific interface and really
 *   should be called board_phy_irq()); In other cases, the PHY interrupt
 *   may be cause by the chip's MAC logic (in which case arch_phy_irq()) is
 *   an appropriate name.  Other other boards, there may be no PHY interrupts
 *   available at all.  If client attachable PHY interrupts are available
 *   from the board or from the chip, then CONFIG_ARCH_PHY_INTERRUPT should
 *   be defined to indicate that fact.
 *
 *   Typical usage:
 *   a. OS service logic (not application logic*) attaches to the PHY
 *      PHY interrupt and enables the PHY interrupt.
 *   b. When the PHY interrupt occurs:  (1) the interrupt should be
 *      disabled and () work should be scheduled on the worker thread (or
 *      perhaps a dedicated application thread).
 *   c. That worker thread should use the SIOCGMIIPHY, SIOCGMIIREG,
 *      and SIOCSMIIREG ioctl calls** to communicate with the PHY,
 *      determine what network event took place (Link Up/Down?), and
 *      take the appropriate actions.
 *   d. It should then interact the PHY to clear any pending
 *      interrupts, then re-enable the PHY interrupt.
 *
 *    * This is an OS internal interface and should not be used from
 *      application space.  Rather applications should use the SIOCMIISIG
 *      ioctl to receive a signal when a PHY event occurs.
 *   ** This interrupt is really of no use if the Ethernet MAC driver
 *      does not support these ioctl calls.
 *
 * Input Parameters:
 *   intf    - Identifies the network interface.  For example "eth0".  Only
 *             useful on platforms that support multiple Ethernet interfaces
 *             and, hence, multiple PHYs and PHY interrupts.
 *   handler - The client interrupt handler to be invoked when the PHY
 *             asserts an interrupt.  Must reside in OS space, but can
 *             signal tasks in user space.  A value of NULL can be passed
 *             in order to detach and disable the PHY interrupt.
 *   arg     - The argument that will accompany the interrupt
 *   enable  - A function pointer that be unused to enable or disable the
 *             PHY interrupt.
 *
 * Returned Value:
 *   Zero (OK) returned on success; a negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

#if defined(CONFIG_ARCH_PHY_INTERRUPT)
int arch_phy_irq(FAR const char *intf, xcpt_t handler, void *arg,
                 phy_enable_t *enable)
{
  /* Using  ET0_LINKSTA for PHY interrupt line which is connected to ETHERC.
   * Interrupt will generate on common line EINT0 which is
   * used for all other interrupt as well.
   */

  irqstate_t flags;
  phy_enable_t enabler;

  DEBUGASSERT(intf);

  ninfo("%s: handler=%p\n", intf, handler);
  phyinfo("EMAC0: devname=%s\n", RX65N_EMAC0_DEVNAME);

  if (strcmp(intf, RX65N_EMAC0_DEVNAME) == 0)
    {
      phyinfo("Select EMAC0\n");
    }
  else
    {
      nerr("ERROR: Unsupported interface: %s\n", intf);
      return -ENODEV;
    }

  flags = enter_critical_section();
  rx65n_phyintenable(false);

  /* Configure the interrupt */

  if (handler)
    {
      phylinknotification.phandler = handler;
      phylinknotification.penable = enable;
      phylinknotification.pclient = arg;
      enabler  = rx65n_phyintenable;
    }
  else
    {
      phylinknotification.phandler = NULL;
      phylinknotification.penable = NULL;
      phylinknotification.pclient = NULL;
      enabler = NULL;
    }

  /* Return the enabling function pointer */

  if (enable)
    {
      *enable = enabler;
    }

  /* Return the old handler (so that it can be restored) */

  leave_critical_section(flags);
  return OK;
}
#endif

/****************************************************************************
 * Function Name: phy_start_autonegotiate
 * Description  : Starts auto-negotiate
 * Arguments    : pause -
 *                    Using state of pause frames
 * Return Value : none
 ****************************************************************************/

void phy_start_autonegotiate (uint8_t pause)
{
  volatile uint16_t regval = 0;

  /* Set local ability */

  /* When pause frame is not used */

  if (ETHER_FLAG_OFF == pause)
    {
      regval = ((((PHY_AN_ADVERTISEMENT_100F |
                   PHY_AN_ADVERTISEMENT_100H) |
                   PHY_AN_ADVERTISEMENT_10F) |
                   PHY_AN_ADVERTISEMENT_10H) |
                   PHY_AN_ADVERTISEMENT_SELECTOR);
    }

  /* When pause frame is used */

  else
    {
      regval = ((((((PHY_AN_ADVERTISEMENT_ASM_DIR |
                     PHY_AN_ADVERTISEMENT_PAUSE) |
                     PHY_AN_ADVERTISEMENT_100F) |
                     PHY_AN_ADVERTISEMENT_100H) |
                     PHY_AN_ADVERTISEMENT_10F) |
                     PHY_AN_ADVERTISEMENT_10H) |
                     PHY_AN_ADVERTISEMENT_SELECTOR);
    }

  /* Configure what the PHY and the Ethernet controller on this board
   * supports
   */

  rx65n_phywrite(CONFIG_RX65N_EMAC0_PHYADDR,
                           PHY_REG_AN_ADVERTISEMENT, regval);
  rx65n_phywrite(CONFIG_RX65N_EMAC0_PHYADDR, PHY_REG_CONTROL,
                      (PHY_CONTROL_AN_ENABLE | PHY_CONTROL_AN_RESTART));
}

/****************************************************************************
 * Function Name: rx65n_power_on_control
 * Description  : Powers on the channel if the ETHEC channel
 *                used and the PHY access channel are different, or if the
 *                PHY access channel is powered off.
 * Arguments    : none
 * Return Value : none
 ****************************************************************************/

void rx65n_power_on_control(void)
{
  uint8_t regval = 0;

  /* Enable writing to registers related to operating modes, LPC, CGC and
   * software reset
   */

  SYSTEM.PRCR.WORD = 0xa50bu;

  /* Enable writing to MSTP registers. */

  regval  = getreg8(RX65N_MPC_PWPR);
  regval &= (~(1 << 7)); /* Clearing the PWPR register B0WI bit */
  putreg8(regval, RX65N_MPC_PWPR);
  regval  = getreg8(RX65N_MPC_PWPR);
  regval |= (1 << 6); /* Enabling the PWPR registers PFSWE bit */
  putreg8(regval, RX65N_MPC_PWPR);

  /* Enable selected ETHERC/EDMAC Channel. */

  regval  = rx65n_getreg(RX65N_MSTP_CRB);
  regval &= (~(1 << 15)); /* Clearing the MSTPB bit in MSTPCRB to release from module stop state */
  rx65n_putreg(regval, RX65N_MSTP_CRB);

  /* Disable writing to MSTP registers. */

  regval  = getreg8(RX65N_MPC_PWPR);
  regval &= (~(1 << 7)); /* Clearing the PWPR register B0WI bit */
  putreg8(regval, RX65N_MPC_PWPR);
  regval  = getreg8(RX65N_MPC_PWPR);
  regval &= (~(1 << 6)); /* Clear the PWPR registers PFSWE bit */
  putreg8(regval, RX65N_MPC_PWPR);
  regval  = getreg8(RX65N_MPC_PWPR);
  regval |= (1 << 7); /* Enable the PWPR register B0WI bit */
  putreg8(regval, RX65N_MPC_PWPR);

  /* Enable protection */

  SYSTEM.PRCR.WORD = 0xa500u;
}

/****************************************************************************
 * Function Name: rx65n_ether_set_phy_mode
 * Description  : Set port connect for Mode selection
 * Arguments    : mode -
 *                    phy mode
 * Return Value : none
 ****************************************************************************/

void rx65n_ether_set_phy_mode(uint8_t mode)
{
  if (PHY_MII_SET_MODE == mode)
    {
      /* MII */

      /* PFENET -> PHYMODE0 is Enabled Ethernet Channel0 to use MII Mode; */

      putreg8(ETH_PFENET_MII_MODE, RX65N_MPC_PFENET);
    }
  else if (PHY_RMII_SET_MODE == mode)
    {
      /* RMII */

      /* PFENET -> PHYMODE0 is Enabled Ethernet Channel0 to use MII Mode; */

      putreg8(ETH_PFENET_RMII_MODE, RX65N_MPC_PFENET);
    }
  else
    {
      /* By Default MII will be selected */

      /* PFENET -> PHYMODE0 is Enabled Ethernet Channel0 to use MII Mode; */

      putreg8(ETH_PFENET_MII_MODE, RX65N_MPC_PFENET);
    }
}

/****************************************************************************
 * Function Name: phy_reg_read
 * Description  : Reads PHY register through MII interface
 * Arguments    : pdata -
 *                    pointer to store the data read
 * Return Value : none
 ****************************************************************************/

static void phy_reg_read (uint16_t *pdata)
{
  int32_t databitcnt = 0;
  int32_t j;
  uint16_t reg_data;

  /* The processing of DATA (data) about reading of the frame format
   * of MII Management Interface which is
   * provided by "Table 22-12" of "22.2.4.5" of
   * "IEEE 802.3-2008_section2".
   */

  reg_data = 0;
  databitcnt = 16; /* Number of bit to read */

  while (databitcnt > 0) /* reading 1 bit per loop */
    {
      for (j = ETHER_CFG_PHY_MII_WAIT; j > 0; j--)
        {
          /* Reset All Flags of PIR */

          rx65n_putreg(ETH_PIR_RESET_ALL, RX65N_ETH_PIR);
        }

      for (j = ETHER_CFG_PHY_MII_WAIT; j > 0; j--)
        {
          /* Setting MDC of PIR */

          rx65n_putreg(ETH_PIR_SET_MDC, RX65N_ETH_PIR);
        }

      reg_data <<= 1;

      /* MDI read  */

      reg_data |= (uint16_t) (((rx65n_getreg(RX65N_ETH_PIR))
                                                  & ETH_PIR_MDI) >> 3);

      for (j = ETHER_CFG_PHY_MII_WAIT; j > 0; j--)
        {
          /* Setting MDC of PIR */

          rx65n_putreg(ETH_PIR_SET_MDC, RX65N_ETH_PIR);
        }

      for (j = ETHER_CFG_PHY_MII_WAIT; j > 0; j--)
        {
          rx65n_putreg(ETH_PIR_RESET_ALL, RX65N_ETH_PIR); /* Reset All Flags of PIR */
        }

      databitcnt--;
    }

  (*pdata) = reg_data;
}

/****************************************************************************
 * Function Name: rx65n_phyread
 * Description  : Reads a PHY register
 * Arguments    : phydevaddr -
 *                    Phy address
 *                reg_addr -
 *                    address of the PHY register
 * Return Value : read value
 ****************************************************************************/

static uint16_t rx65n_phyread (uint8_t phydevaddr, uint16_t reg_addr,
                               uint16_t *value)
{
  uint16_t data;

  /* The value is read from the PHY register by the frame format
   * of MII Management Interface provided
   * for by Table 22-12 of 22.2.4.5 of
   * IEEE 802.3-2008_section2.
   */

  rx65n_phy_preamble();
  rx65n_phy_reg_set(phydevaddr, reg_addr, PHY_MII_READ);
  rx65n_phy_trans_zto0();
  phy_reg_read(&data);
  rx65n_phy_trans_zto0();
  *value = data;
  return OK;
}

/****************************************************************************
 * Function Name: rx65n_ether_enable_icu
 * Description  :
 * Arguments    :
 * Return Value : none
 ****************************************************************************/

void rx65n_ether_enable_icu(void)
{
  uint32_t ipl;

  /* Enabling EDMAC0 interrupt request using AL1 group interrupt */

  ICU.GENAL1.BIT.EN4 = 1;

  /* Priority to this interrupt should be value 2 */

  ipl = ETHER_CFG_AL1_INT_PRIORITY;

  /* Disable group interrupts */

  IEN(ICU, GROUPAL1) = 0;

  /* Clear interrupt flag */

  IR(ICU, GROUPAL1)  = 0;

  /* Set priority level */

  IPR(ICU, GROUPAL1) = (uint8_t)
                       (ipl > IPR(ICU, GROUPAL1) ? ipl : IPR(ICU, GROUPAL1));

  /* Enable group BL0 interrupt */

  IEN(ICU, GROUPAL1) = 1;
}

/****************************************************************************
 * Function Name: rx65n_phy_trans_zto0
 * Description  : Performs bus release so that PHY can drive data
 *              : for read operation
 * Arguments    : ether_channel -
 *                    Ethernet channel number
 * Return Value : none
 ****************************************************************************/

static void rx65n_phy_trans_zto0 ()
{
  int32_t j;

  /* The processing of TA (turnaround) about reading of the frame format of
   * MII Management Interface which is
   * provided by "Table 22-12" of "22.2.4.5" of "IEEE 802.3-2008_section2".
   */

  for (j = ETHER_CFG_PHY_MII_WAIT; j > 0; j--)
    {
          /* Resetting All flags of PIR */

      rx65n_putreg(ETH_PIR_RESET_ALL, RX65N_ETH_PIR);
    }

  for (j = ETHER_CFG_PHY_MII_WAIT; j > 0; j--)
    {
      rx65n_putreg(ETH_PIR_SET_MDC, RX65N_ETH_PIR); /* Setting MDC of PIR */
    }

  for (j = ETHER_CFG_PHY_MII_WAIT; j > 0; j--)
    {
      rx65n_putreg(ETH_PIR_SET_MDC, RX65N_ETH_PIR); /* Setting MDC of PIR */
    }

  for (j = ETHER_CFG_PHY_MII_WAIT; j > 0; j--)
    {
      /* Resetting All flags of PIR */

      rx65n_putreg(ETH_PIR_RESET_ALL, RX65N_ETH_PIR);
    }
}

/****************************************************************************
 * Function Name: rx65n_phy_trans_1to0
 * Description  : Switches data bus so MII interface can drive data
 *              : for write operation
 * Arguments    : ether_channel -
 *                    Ethernet channel number
 * Return Value : none
 ****************************************************************************/

static void rx65n_phy_trans_1to0 ()
{
  /* The processing of TA (turnaround) about writing of the frame format
   * of MII Management Interface which is
   * provided by "Table 22-12" of "22.2.4.5" of "IEEE 802.3-2008_section2".
   */

  rx65n_phy_mii_write1();
  rx65n_phy_mii_write0();
}

/****************************************************************************
 * Function Name: rx65n_phy_reg_set
 * Description  : Sets a PHY device to read or write mode
 * Arguments    : phydevaddr -
 *                    Phy address
 *                reg_addr -
 *                    address of the PHY register
 *                option -
 *                    mode
 * Return Value : none
 ****************************************************************************/

static void rx65n_phy_reg_set (uint8_t phydevaddr, uint16_t reg_addr,
                               int32_t option)
{
  int32_t bitcnt = 0;
  uint16_t data;

  /* The processing of ST (start of frame),
   * OP (operation code), PHYAD (PHY Address), and
   * REGAD (Register Address)  about the frame format of
   * MII Management Interface which is
   * provided by "Table 22-12" of
   * "22.2.4.5" of "IEEE 802.3-2008_section2".
   */

  data = 0;
  data = (PHY_MII_ST << 14); /* ST code    */
  if (PHY_MII_READ == option)
    {
      data |= (PHY_MII_READ << 12); /* OP code(RD)  */
    }
  else
    {
      data |= (PHY_MII_WRITE << 12); /* OP code(WR)  */
    }

  data |= (uint16_t) (phydevaddr << 7); /* PHY Address configured 0x1e  in decimal 30 */
  data |= (reg_addr << 2);
  bitcnt = 14;  /* These number of bits to send for ST, OP, PHYAD and REGAD */
  while (bitcnt > 0)
    {
      if (0 == (data & 0x8000))
        {
          rx65n_phy_mii_write0();
        }
      else
        {
          rx65n_phy_mii_write1();
        }

      data <<= 1;
      bitcnt--;
    }
}

/****************************************************************************
 * Function Name: rx65n_phy_reg_write
 * Description  : Writes to PHY register through MII interface
 * Arguments    :  data -
 *                value to write
 * Return Value : none
 ****************************************************************************/

static void rx65n_phy_reg_write (uint16_t data)
{
  int32_t databitcnt = 0;

  /* The processing of DATA (data) about writing of the frame format
   * of MII Management Interface which is
   * provided by "Table 22-12" of "22.2.4.5"
   * of "IEEE 802.3-2008_section2".
   */

  databitcnt = 16;       /* Number of bit to write */
  while (databitcnt > 0) /* writing 1 bit per loop */
    {
      if (0 == (data & 0x8000))
        {
          rx65n_phy_mii_write0();
        }
      else
        {
          rx65n_phy_mii_write1();
        }

      databitcnt--;
      data <<= 1;
    }
}

/****************************************************************************
 * Function Name: rx65n_phy_mii_write1
 * Description  : Outputs 1 to the MII interface
 * Arguments    : none
 * Return Value : none
 ****************************************************************************/

static void rx65n_phy_mii_write1 ()
{
  int32_t j;

  /* The processing of one bit about frame format
   * of MII Management Interface which is
   * provided by "Table 22-12" of "22.2.4.5"
   * of "IEEE 802.3-2008_section2".
   * The data that 1 is output.
   */

  for (j = ETHER_CFG_PHY_MII_WAIT; j > 0; j--)
    {
      /* Setting MDO and MMD and by default MDI */

       rx65n_putreg(ETH_PIR_SET_MDO_MMD, RX65N_ETH_PIR);
    }

  for (j = ETHER_CFG_PHY_MII_WAIT; j > 0; j--)
    {
      /* Setting MDO, MMD and MDC and by default MDI */

      rx65n_putreg(ETH_PIR_SET_MDO_MMD_MDC, RX65N_ETH_PIR);
    }

  for (j = ETHER_CFG_PHY_MII_WAIT; j > 0; j--)
    {
      /* Setting MDO, MMD and MDC and by default MDI */

      rx65n_putreg(ETH_PIR_SET_MDO_MMD_MDC, RX65N_ETH_PIR);
    }

  for (j = ETHER_CFG_PHY_MII_WAIT; j > 0; j--)
    {
      /* Setting MDO and MMD and by default MDI */

      rx65n_putreg(ETH_PIR_SET_MDO_MMD, RX65N_ETH_PIR);
    }
}

/****************************************************************************
 * Function Name: rx65n_phy_mii_write0
 * Description  : Outputs 0 to the MII interface
 * Arguments    : none
 * Return Value : none
 ****************************************************************************/

static void rx65n_phy_mii_write0(void)
{
  int32_t j;

  /* The processing of one bit about frame format
   * of MII Management Interface which is
   * provided by "Table 22-12" of "22.2.4.5" of "IEEE 802.3-2008_section2".
   * The data that 0 is output.
   */

  for (j = ETHER_CFG_PHY_MII_WAIT; j > 0; j--)
    {
      /* Setting MMD and by default MDI */

      rx65n_putreg(ETH_PIR_SET_MMD, RX65N_ETH_PIR);
    }

  for (j = ETHER_CFG_PHY_MII_WAIT; j > 0; j--)
    {
      /* Setting MDC and MMD and by default MDI */

      rx65n_putreg(ETH_PIR_SET_MMD_MDC, RX65N_ETH_PIR);
    }

  for (j = ETHER_CFG_PHY_MII_WAIT; j > 0; j--)
    {
      /* Setting MDC and MMD and by default MDI */

      rx65n_putreg(ETH_PIR_SET_MMD_MDC, RX65N_ETH_PIR);
    }

  for (j = ETHER_CFG_PHY_MII_WAIT; j > 0; j--)
    {
      /* Setting MMD and by default MDI */

      rx65n_putreg(ETH_PIR_SET_MMD, RX65N_ETH_PIR);
    }
}

/****************************************************************************
 * Function Name: rx65n_phy_preamble
 * Description  : As preliminary preparation for access
 *                to the PHY module register,
 *                "1" is output via the MII management interface.
 * Arguments    : none
 * Return Value : none
 ****************************************************************************/

static void rx65n_phy_preamble(void)
{
  int16_t preamble_bits = 0;

  /* The processing of PRE (preamble) about the frame format
   * of MII Management Interface which is
   * provided by "Table 22-12" of "22.2.4.5"
   * of "IEEE 802.3-2008_section2".
   */

  /* Send 32 consecutive 1’s as per 34.3.4.1
   * MII/RMII Management Frame Format
   */

  preamble_bits = 32;
  while (preamble_bits > 0)
    {
      rx65n_phy_mii_write1();
      preamble_bits--;
    }
}

/****************************************************************************
 * Function Name: rx65n_phywrite
 * Description  : Writes to a PHY register
 * Arguments    : phydevaddr -
 *                    Phy device address
 *                reg_addr -
 *                    address of the PHY register
 *                data -
 *                    value
 * Return Value : none
 ****************************************************************************/

static int rx65n_phywrite (uint8_t phydevaddr, uint16_t reg_addr,
                           uint16_t data)
{
  /* The value is read from the PHY register by the frame format
   * of MII Management Interface provided
   * for by Table 22-12 of 22.2.4.5 of IEEE 802.3-2008_section2.
   */

  rx65n_phy_preamble();
  rx65n_phy_reg_set(phydevaddr, reg_addr, PHY_MII_WRITE);
  rx65n_phy_trans_1to0();
  rx65n_phy_reg_write(data);
  rx65n_phy_trans_zto0();
  return OK;
}

/****************************************************************************
 * Function Name: rx65n_ether_interrupt_init
 * Description  : Writes to a ETHERC/EDMAC register
 * Arguments    : none
 * Return Value : none
 ****************************************************************************/

void rx65n_ether_interrupt_init(void)
{
  uint32_t regval;

  /* Start interrupt control unit for ethernet interrupt */

  rx65n_ether_enable_icu();

  /* Clear all ETHERC status BFR, PSRTO, LCHNG, MPD, ICD */

  rx65n_putreg(ETH_ECSR_CLR, RX65N_ETH_ECSR);

  /* Clear all EDMAC status bits */

  rx65n_putreg(ETHD_EESR_EDMAC, RX65N_ETHD_EESR);

  /* Notification of ET0_LINKSTA signal change interrupt is disabled. */

  regval  = rx65n_getreg(RX65N_ETH_ECSIPR);
  regval &= (~(ETH_ECSIPR_LCHNGIP));
  rx65n_putreg(regval, RX65N_ETH_ECSIPR);

  /* ETHERC status interrupt request is disabled */

  regval  = rx65n_getreg(RX65N_ETHD_EESIPR);
  regval &= (~(ETHD_EESIPR_ECIIP));
  rx65n_putreg(regval, RX65N_ETHD_EESIPR);
}

/****************************************************************************
 * Function: rx65n_phyinit
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
 * Assumptions:
 *
 ****************************************************************************/

static int rx65n_phyinit(FAR struct rx65n_ethmac_s *priv)
{
  uint32_t count;
  uint16_t reg;
  int ret;
  count = 0;

#ifdef CONFIG_RX65N_EMAC0_AUTONEG
  /*  Software Reset the PHY */

    ret = rx65n_phywrite(CONFIG_RX65N_EMAC0_PHYADDR,
                         PHY_REG_CONTROL, PHY_CONTROL_RESET);

  /* Reset completion waiting */

  do
    {
      ret = rx65n_phyread(CONFIG_RX65N_EMAC0_PHYADDR,
                          PHY_REG_CONTROL, &reg);
      count++;
    }
  while ((reg & PHY_CONTROL_RESET) && (count < ETHER_CFG_PHY_DELAY_RESET));

  if (count > ETHER_CFG_PHY_DELAY_RESET)
    {
      ret = -ETIMEDOUT;
      goto error_with_reset_timeout;
    }

  priv->mbps100 = 0;
  priv->fduplex = 0;

  /* Initialize the PHY Auto negotiation with frame pause disable */

  phy_start_autonegotiate (PAUSE_FRAME_DISABLE);

  /* Interrupt initialization */

  rx65n_ether_interrupt_init();

  /* To check the Link status in PHY registers read Auto-negotiation
   * Advertisement register
   */

  ret = rx65n_phyread(CONFIG_RX65N_EMAC0_PHYADDR, PHY_REG_STATUS, &reg);

  count = 0;
  do
    {
      /* Reading PHY status register after negotiation */

      ret = rx65n_phyread(CONFIG_RX65N_EMAC0_PHYADDR, PHY_REG_STATUS, &reg);
      count++;
    }
  while ((!((reg & (PHY_VALID_LINK | PHY_AUTO_NEG_DONE))
                         || (count > ETHER_PHY_STATUS_CHECK_DELAY))));

  if (count > ETHER_PHY_STATUS_CHECK_DELAY)
    {
      ret = -ETIMEDOUT;
      goto error_with_auto_neg_timeout;
    }

#ifdef CONFIG_RX65N_EMAC0_PHYSR_ALTCONFIG
  ret = rx65n_phyread(CONFIG_RX65N_EMAC0_PHYADDR, PHY_STS_REG, &reg);
  switch (reg & CONFIG_RX65N_EMAC0_PHYSR_ALTMODE)
    {
      default:
      case CONFIG_RX65N_EMAC0_PHYSR_10HD:
        priv->fduplex = 0;
        priv->mbps100 = 0;
        break;

      case CONFIG_RX65N_EMAC0_PHYSR_100HD:
        priv->fduplex = 0;
        priv->mbps100 = 1;
        break;

      case CONFIG_RX65N_EMAC0_PHYSR_10FD:
        priv->fduplex = 1;
        priv->mbps100 = 0;
        break;

      case CONFIG_RX65N_EMAC0_PHYSR_100FD:
        priv->fduplex = 1;
        priv->mbps100 = 1;
        break;
    }

#endif
#endif
  error_with_reset_timeout:
  error_with_auto_neg_timeout:
  return ret;
}

/****************************************************************************
 * Function: rx65n_ethreset
 *
 * Description:
 *  Reset the Ethernet block.
 *
 * Input Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   Zero on success, or a negated errno value on any failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int rx65n_ethreset(FAR struct rx65n_ethmac_s *priv)
{
  uint32_t regval;
  uint32_t retries;

  /* Notification of ET0_LINKSTA signal change interrupt is disabled. */

  regval  = rx65n_getreg(RX65N_ETH_ECSIPR);
  regval &= (~(ETH_ECSIPR_LCHNGIP));
  rx65n_putreg(regval, RX65N_ETH_ECSIPR);

  regval  = rx65n_getreg(RX65N_ETHD_EESIPR);
  regval &= (~(ETHD_EESIPR_ECIIP));
  rx65n_putreg(regval, RX65N_ETHD_EESIPR);

  /* Disable TE and RE in ECMR Register */

  /* rx65n_putreg(ETH_ECMR_CLR, RX65N_ETH_ECMR); */

  regval  = rx65n_getreg(RX65N_ETH_ECMR);
  regval &= (~(ETH_ECMR_TE | ETH_ECMR_RE));
  rx65n_putreg(regval, RX65N_ETH_ECMR);

  /* Perform a software reset by setting the SWR bit in the EDMR register.
   * When 1 is written, the EDMAC and ETHERC are reset
   */

  /* Registers TDLAR, RDLAR, RMFCR TFUCR and RFOCR are not reset */

  regval  = rx65n_getreg(RX65N_ETHD_EDMR);
  regval |= ETHD_EDMR_SWR;
  rx65n_putreg(regval, RX65N_ETHD_EDMR);

  /* Wait for software reset to complete. The SR bit is cleared automatically
   * after the reset operation has completed in all of the core clock
   * domains.
   */

  retries = 10;
  while (((rx65n_getreg(RX65N_ETHD_EDMR) & ETHD_EDMR_SWR) != 0) &&
         retries > 0)
    {
      retries--;
      up_mdelay(10);
    }

  if (retries == 0)
    {
      return -ETIMEDOUT;
    }

  /* Notification of ET0_LINKSTA signal change interrupt is disabled. */

  regval  = rx65n_getreg(RX65N_ETH_ECSIPR);
  regval |= (ETH_ECSIPR_LCHNGIP);
  rx65n_putreg(regval, RX65N_ETH_ECSIPR);

  regval  = rx65n_getreg(RX65N_ETHD_EESIPR);
  regval |= (ETHD_EESIPR_ECIIP);
  rx65n_putreg(regval, RX65N_ETHD_EESIPR);

  return 0;
}

/****************************************************************************
 * Function: rx65n_macconfig
 *
 * Description:
 *  Configure the Ethernet MAC for DMA operation.
 *  Software reset has to be executed, and ETHERC and EDMAC are configured.
 * Input Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int rx65n_macconfig(FAR struct rx65n_ethmac_s *priv)
{
  uint32_t regval;
  uint32_t retries;

  /* Perform a software reset by setting the SWR bit in the EDMR register.
   * When 1 is written, the EDMAC and ETHERC are reset
   */

  /* Registers TDLAR, RDLAR, RMFCR TFUCR and RFOCR are not reset */

  regval  = rx65n_getreg(RX65N_ETHD_EDMR);
  regval |= ETHD_EDMR_SWR;
  rx65n_putreg(regval, RX65N_ETHD_EDMR);

  retries = 10;
  while (((rx65n_getreg(RX65N_ETHD_EDMR) & ETHD_EDMR_SWR) != 0) &&
         retries > 0)
    {
      retries--;
      up_mdelay(10);
    }

  /* Notification of ET0_LINKSTA signal change interrupt is disabled. */

  regval  = rx65n_getreg(RX65N_ETH_ECSIPR);
  regval |= (ETH_ECSIPR_LCHNGIP);
  rx65n_putreg(regval, RX65N_ETH_ECSIPR);

  regval  = rx65n_getreg(RX65N_ETHD_EESIPR);
  regval |= (ETHD_EESIPR_ECIIP);
  rx65n_putreg(regval, RX65N_ETHD_EESIPR);

  rx65n_macaddress(priv);

  return OK;
}

/****************************************************************************
 * Function: rx65n_macaddress
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

static void rx65n_macaddress(FAR struct rx65n_ethmac_s *priv)
{
  FAR struct net_driver_s *dev = &priv->dev;
  uint32_t regval;
  regval = 0;

  ninfo("%s MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
        dev->d_ifname,
        dev->d_mac.ether.ether_addr_octet[0],
                     dev->d_mac.ether.ether_addr_octet[1],
        dev->d_mac.ether.ether_addr_octet[2],
                     dev->d_mac.ether.ether_addr_octet[3],
        dev->d_mac.ether.ether_addr_octet[4],
                    dev->d_mac.ether.ether_addr_octet[5]);

  /* Set the MAC address high register */

  regval = (((((uint32_t) dev->d_mac.ether.ether_addr_octet[0] << 24) |
              ((uint32_t) dev->d_mac.ether.ether_addr_octet[1] << 16))
           | ((uint32_t) dev->d_mac.ether.ether_addr_octet[2] << 8)) |
                   (uint32_t) dev->d_mac.ether.ether_addr_octet[3]);
  rx65n_putreg(regval, RX65N_ETH_MAHR);

  /* Set the MAC address low register */

  regval = 0;
  regval = (((uint32_t) dev->d_mac.ether.ether_addr_octet[4] << 8) |
             (uint32_t)dev->d_mac.ether.ether_addr_octet[5]);
  rx65n_putreg(regval, RX65N_ETH_MALR);
}

/****************************************************************************
 * Function: rx65n_ethconfig
 *
 * Description:
 *  Configure the Ethernet interface for operation.
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

static int rx65n_ethconfig(FAR struct rx65n_ethmac_s *priv)
{
  int ret;
  uint32_t regval;

  /* Reset the Ethernet block */

  ninfo("Reset the Ethernet block\n");
  ret = rx65n_ethreset(priv);
  if (ret < 0)
    {
      nerr("ERROR: Reset of Ethernet block failed\n");
      return ret;
    }

  /* Initialize the PHY selection Set port connect
   * Currently we are using MII
   */

  rx65n_ether_set_phy_mode(PHY_SET_MODE_REG);

  /* ETHERC/EDMAC Power on */

  rx65n_power_on_control();

  /* Initialize the PHY */

  ninfo("Initialize the PHY\n");
  ret = rx65n_phyinit(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Initialize the MAC and Ethernet DMA */

  ninfo("Initialize the MAC and DMA\n");
  ret = rx65n_macconfig(priv);
  if (ret < 0)
    {
      return ret;
    }

#ifndef CONFIG_ENDIAN_BIG

  /* Set little endian mode */

  regval  = rx65n_getreg(RX65N_ETHD_EDMR);
  regval |= ETHD_EDMR_DE;
  rx65n_putreg(regval, RX65N_ETHD_EDMR);
#endif

  /* Initialize the free buffer list */

  rx65n_initbuffer(priv);

  /* Initialize TX Descriptors list: Chain Mode */

  rx65n_txdescinit(priv);

  /* Initialize RX Descriptors list: Chain Mode  */

  rx65n_rxdescinit(priv);

  /* Clear all ETHERC status BFR, PSRTO, LCHNG, MPD, ICD */

  rx65n_putreg(ETH_ECSR_CLR, RX65N_ETH_ECSR);

  /* Clear all EDMAC status bits */

  rx65n_putreg(ETHD_EESR_EDMAC, RX65N_ETHD_EESR);

  /* Interrupt configuration */

  /* Notification of ET0_LINKSTA signal change interrupt */

  regval  = rx65n_getreg(RX65N_ETH_ECSR);
  regval |= ((ETH_ECSR_LCHNG)); /* Write 1 to clear flag */
  rx65n_putreg(regval, RX65N_ETH_ECSR);
  regval  = rx65n_getreg(RX65N_ETH_ECSIPR);
  regval |= ((ETH_ECSIPR_LCHNGIP));
  rx65n_putreg(regval, RX65N_ETH_ECSIPR);

  /* ETHERC/EDMAC enabling Status Interrupt */

  regval  = rx65n_getreg(RX65N_ETHD_EESIPR);
  regval |= ((ETHD_EESIPR_ECIIP));
  rx65n_putreg(regval, RX65N_ETHD_EESIPR);

  /* Ethernet length 1514bytes + CRC and */

  rx65n_putreg(ETH_RFLR_RFL, RX65N_ETH_RFLR);

  /* Intergap is 96-bit time */

  rx65n_putreg(ETH_IPGR_IPG_INITIAL, RX65N_ETH_IPGR);

  /* Continuous reception number of Broadcast frame */

  regval  = rx65n_getreg(RX65N_ETH_BCFRR);
  regval |= (ETH_BCFRR_BCF);
  rx65n_putreg(regval, RX65N_ETH_BCFRR);

  regval  = rx65n_getreg(RX65N_ETHD_TRSCER);
  if (ETHER_MC_FILTER_ON == priv->mc_filter_flag)
    {
  /* Reflect the EESR.RMAF bit status in the RD0.RFS bit
   * in the receive descriptor
   */

      regval &= (~((ETHD_TRSCER_RMAFCE) | (ETHD_TRSCER_RRFCE)));
      rx65n_putreg(regval, RX65N_ETHD_TRSCER);
    }
  else
    {
  /* Don't reflect the EESR.RMAF bit status in the RD0.RFS
   * bit in the receive descriptor
   */

      regval &= (~(ETHD_TRSCER_RRFCE));
      regval |= (ETHD_TRSCER_RMAFCE);
      rx65n_putreg(regval, RX65N_ETHD_TRSCER);
    }

  /* Threshold of Tx_FIFO */

  rx65n_putreg(ETHD_TFTR_TFT, RX65N_ETHD_TFTR); /* Store and forward mode */

  /* transmit fifo is 1968 bytes & receive fifo is 2048 bytes */

  regval  = rx65n_getreg(RX65N_ETHD_FDR);
  regval |= ((ETHD_FDR_RFD) | (ETHD_FDR_TFD));
  rx65n_putreg(regval, RX65N_ETHD_FDR);

  /* Configure receiving method */

  /* b0      RNR - Receive Request Bit Reset - Continuous reception
   * of multiple frames is possible.
   * b31:b1  Reserved set to 0
   */

  regval  = rx65n_getreg(RX65N_ETHD_RMCR);
  regval |= (ETHD_RMCR_RNR);
  rx65n_putreg(regval, RX65N_ETHD_RMCR);

  /* Transmit Interrupt Setting */

  regval  = rx65n_getreg(RX65N_ETHD_TRIMD);
  regval |= ((ETHD_TRIMD_TIS) | (ETHD_TRIMD_TIM));
  rx65n_putreg(regval, RX65N_ETHD_TRIMD);

  /* ETHERC/EDMAC enabling Interrupt */

  regval  = rx65n_getreg(RX65N_ETHD_EESIPR);
  regval |= ((ETHD_EESIPR_TCIP) | (ETHD_EESIPR_FRIP) | (ETHD_EESIPR_ECIIP));
  rx65n_putreg(regval, RX65N_ETHD_EESIPR);

  regval  = rx65n_getreg(RX65N_ETH_ECSIPR);
  regval |= ((ETH_ECSIPR_LCHNGIP));
  rx65n_putreg(regval, RX65N_ETH_ECSIPR);

  /* Enabling TE and RE and set DM mode  */

  regval  = rx65n_getreg(RX65N_ETH_ECMR);
  regval  |= ETH_ECMR_RE;
  regval  |= ETH_ECMR_TE;
  if (priv->fduplex)
    {
      regval  |= ETH_ECMR_DM;
    }
  else
    {
      regval  &= (~ETH_ECMR_DM);
    }

  if (priv->mbps100)
    {
      regval  |= ETH_ECMR_RTM;
    }
  else
    {
      regval  &= (~ETH_ECMR_RTM);
    }

  rx65n_putreg(regval, RX65N_ETH_ECMR);

  /* Start DMA reception */

  regval  = rx65n_getreg(RX65N_ETHD_EDRRR);
  regval |= (ETHD_EDRRR_RR);
  rx65n_putreg(regval, RX65N_ETHD_EDRRR);

  return ret;
}

/****************************************************************************
 * Function: rx65n_ethinitialize
 *
 * Description:
 *   Initialize the EMAC driver.
 *
 * Input Parameters:
 *   intf - If multiple EMAC peripherals are supported, this identifies the
 *     the EMAC peripheral being initialized.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *   Called very early in the initialization sequence.
 *
 ****************************************************************************/

int rx65n_ethinitialize(int intf)
{
  struct rx65n_ethmac_s *priv;
  int ret;
  uint32_t reg32;

  /* Initialize hardware mac address */

  uint8_t mac[6];
  mac[0] = RX65N_MAC_ADDRL & 0xff;
  mac[1] = (RX65N_MAC_ADDRL & 0xff00) >> 8;
  mac[2] = (RX65N_MAC_ADDRL & 0xff0000) >> 16;
  mac[3] = (RX65N_MAC_ADDRL & 0xff000000) >> 24;
  mac[4] = RX65N_MAC_ADDRH & 0xff;
  mac[5] = (RX65N_MAC_ADDRH & 0xff00) >> 8;

  ninfo("intf: %d\n", intf);

  /* Get the interface structure associated with this interface number. */

  DEBUGASSERT(intf < RX65N_NETHERNET);

  priv = &g_rx65nethmac[intf];

  /* Initialize the driver structure */

  memset(priv, 0, sizeof(struct rx65n_ethmac_s));

  /* I/F up (new IP address) callback */

  priv->dev.d_ifup    = rx65n_ifup;

  /* I/F down callback */

  priv->dev.d_ifdown  = rx65n_ifdown;
  priv->dev.d_txavail = rx65n_txavail;  /* New TX data callback */
#ifdef CONFIG_NET_MCASTGROUP
  priv->dev.d_addmac  = rx65n_addmac;   /* Add multicast MAC address */
  priv->dev.d_rmmac   = rx65n_rmmac;    /* Remove multicast MAC address */
#endif
#ifdef CONFIG_NETDEV_IOCTL
  priv->dev.d_ioctl   = rx65n_ioctl;    /* Support PHY ioctl() calls */
#endif

  /* Used to recover private state from dev */

  priv->dev.d_private = g_rx65nethmac;

  /* Multi cast flag */

  priv->mc_filter_flag = ETHER_MC_FILTER_OFF;

  /* hw mac address */

  priv->dev.d_mac.ether.ether_addr_octet[0] =  mac[0];
  priv->dev.d_mac.ether.ether_addr_octet[1] =  mac[1];
  priv->dev.d_mac.ether.ether_addr_octet[2] =  mac[2];
  priv->dev.d_mac.ether.ether_addr_octet[3] =  mac[3];
  priv->dev.d_mac.ether.ether_addr_octet[4] =  mac[4];
  priv->dev.d_mac.ether.ether_addr_octet[5] =  mac[5];

  /* Enable write to System registers */

  putreg16(RX65N_PRCR_VALUE, RX65N_PRCR_ADDR);

  /* Start CMT module */

  reg32 = getreg32(RX65N_MSTPCRA_ADDR);

  /* Release CMTW unit1 from module stop state(for CMTW1) */

  reg32 &= (~RX65N_CMTW_UNIT1_MSTPCRA_STOP);

  /* Release CMTW unit1 from module stop state(for CMTW0) */

  reg32 &= (~RX65N_CMTW_UNIT0_MSTPCRA_STOP);
  putreg32(reg32, RX65N_MSTPCRA_ADDR);

  /* Create timer for timing polling for and timing of transmissions */

  rx65n_cmtw0_create(RX65N_CMTW0_COUNT_VALUE_FOR_TXPOLL ,
                     RX65N_CMTW0_COUNT_VALUE_FOR_TXTIMEOUT);

  /* Attach the IRQ to the driver */

  if (irq_attach(RX65N_ETH_IRQ, rx65n_interrupt, NULL))
    {
      /* We could not attach the ISR to the interrupt */

      return -EAGAIN;
    }

  /* Put the interface in the down state. */

  ret = rx65n_ifdown(&priv->dev);
  if (ret < 0)
    {
      nerr("ERROR: Initialization of Ethernet block failed: %d\n", ret);
      return ret;
    }

  /* Register the device with the OS so that socket IOCTLs can be performed */

  netdev_register(&priv->dev, NET_LL_ETHERNET);
  return OK;
}

/****************************************************************************
 * Function: up_netinitialize
 *
 * Description:
 *   This is the "standard" network initialization logic called from the
 *   low-level initialization logic in up_initialize.c.  If RX65N_NETHERNET
 *   greater than one, then board specific logic will have to supply a
 *   version of up_netinitialize() that calls rx65n_ethinitialize() with
 *   the appropriate interface number.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *
 ****************************************************************************/

#if defined (CONFIG_NET) && !defined(CONFIG_NETDEV_LATEINIT)
void up_netinitialize(void)
{
  int ret;

#if defined(CONFIG_RX65N_EMAC0)

  /* Initialize the EMAC0 driver */

  ret = rx65n_ethinitialize(EMAC0_INTF);
  if (ret < 0)
    {
      nerr("ERROR: up_emac_initialize(EMAC0) failed: %d\n", ret);
    }
#endif
}
#endif
#endif /* RX65N_NETHERNET > 0 */
#endif /* CONFIG_NET && CONFIG_RX65N_EMAC */
