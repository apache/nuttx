/****************************************************************************
 * arch/arm/src/samv7/sam_emac.c
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

#if defined(CONFIG_DEBUG_FEATURES) && defined(CONFIG_SAMV7_EMAC_DEBUG)
  /* Force debug output (from this file only) */

#  undef  CONFIG_DEBUG_NET
#  define CONFIG_DEBUG_NET 1
#endif

#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <queue.h>
#include <errno.h>

#include <arpa/inet.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/wdog.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/net/mii.h>
#include <nuttx/net/arp.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/phy.h>

#ifdef CONFIG_NET_PKT
#  include <nuttx/net/pkt.h>
#endif

#include <arch/samv7/chip.h>

#include "arm_internal.h"
#include "hardware/sam_pinmap.h"
#include "hardware/sam_chipid.h"
#include "sam_gpio.h"
#include "sam_periphclks.h"
#include "sam_ethernet.h"

#include <arch/board/board.h>

#if defined(CONFIG_NET) && defined(CONFIG_SAMV7_EMAC)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* If processing is not done at the interrupt level, then work queue support
 * is required.
 */

#if !defined(CONFIG_SCHED_WORKQUEUE)
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

/* EMAC0 Configuration ******************************************************/

#ifdef CONFIG_SAMV7_EMAC0
  /* Number of buffers for RX */

#  ifndef CONFIG_SAMV7_EMAC0_NRXBUFFERS
#    define CONFIG_SAMV7_EMAC0_NRXBUFFERS  16
#  endif

#  if CONFIG_SAMV7_EMAC0_NRXBUFFERS <= 1
#    error CONFIG_SAMV7_EMAC0_NRXBUFFERS invalid
#  endif

  /* Number of buffers for TX */

#  ifndef CONFIG_SAMV7_EMAC0_NTXBUFFERS
#    define CONFIG_SAMV7_EMAC0_NTXBUFFERS  8
#  endif

#  if CONFIG_SAMV7_EMAC0_NTXBUFFERS <= 1
#    error CONFIG_SAMV7_EMAC0_NTXBUFFERS invalid
#  endif

#  ifndef CONFIG_SAMV7_EMAC0_PHYADDR
#    error "CONFIG_SAMV7_EMAC0_PHYADDR must be defined in the NuttX configuration"
#  endif

#  if !defined(CONFIG_SAMV7_EMAC0_MII) && !defined(CONFIG_SAMV7_EMAC0_RMII)
#    warning "Neither CONFIG_SAMV7_EMAC0_MII nor CONFIG_SAMV7_EMAC0_RMII defined"
#  endif

#  if defined(CONFIG_SAMV7_EMAC0_MII) && defined(CONFIG_SAMV7_EMAC0_RMII)
#    error "Both CONFIG_SAMV7_EMAC0_MII and CONFIG_SAMV7_EMAC0_RMII defined"
#  endif

#  ifndef CONFIG_SAMV7_EMAC0_PHYSR
#    error "CONFIG_SAMV7_EMAC0_PHYSR must be defined in the NuttX configuration"
#  endif

#  ifdef CONFIG_SAMV7_EMAC0_AUTONEG
#    ifdef CONFIG_SAMV7_EMAC0_PHYSR_ALTCONFIG
#      ifndef CONFIG_SAMV7_EMAC0_PHYSR_ALTMODE
#        error "CONFIG_SAMV7_EMAC0_PHYSR_ALTMODE must be defined in the NuttX configuration"
#      endif
#      ifndef CONFIG_SAMV7_EMAC0_PHYSR_10HD
#        error "CONFIG_SAMV7_EMAC0_PHYSR_10HD must be defined in the NuttX configuration"
#      endif
#      ifndef CONFIG_SAMV7_EMAC0_PHYSR_100HD
#        error "CONFIG_SAMV7_EMAC0_PHYSR_100HD must be defined in the NuttX configuration"
#      endif
#      ifndef CONFIG_SAMV7_EMAC0_PHYSR_10FD
#        error "CONFIG_SAMV7_EMAC0_PHYSR_10FD must be defined in the NuttX configuration"
#      endif
#      ifndef CONFIG_SAMV7_EMAC0_PHYSR_100FD
#        error "CONFIG_SAMV7_EMAC0_PHYSR_100FD must be defined in the NuttX configuration"
#      endif
#    else
#      ifndef CONFIG_SAMV7_EMAC0_PHYSR_SPEED
#        error "CONFIG_SAMV7_EMAC0_PHYSR_SPEED must be defined in the NuttX configuration"
#      endif
#      ifndef CONFIG_SAMV7_EMAC0_PHYSR_100MBPS
#        error "CONFIG_SAMV7_EMAC0_PHYSR_100MBPS must be defined in the NuttX configuration"
#      endif
#      ifndef CONFIG_SAMV7_EMAC0_PHYSR_MODE
#        error "CONFIG_SAMV7_EMAC0_PHYSR_MODE must be defined in the NuttX configuration"
#      endif
#      ifndef CONFIG_SAMV7_EMAC0_PHYSR_FULLDUPLEX
#        error "CONFIG_SAMV7_EMAC0_PHYSR_FULLDUPLEX must be defined in the NuttX configuration"
#      endif
#    endif
#  endif

  /* PHY definitions */

#  if defined(SAMV7_EMAC0_PHY_DM9161)
#    define EMAC0_MII_OUI_MSB    0x0181
#    define EMAC0_MII_OUI_LSB    0x2e
#  elif defined(SAMV7_EMAC0_PHY_LAN8700)
#    define EMAC0_MII_OUI_MSB    0x0007
#    define EMAC0_MII_OUI_LSB    0x30
#  elif defined(SAMV7_EMAC0_PHY_KSZ8051)
#    define EMAC0_MII_OUI_MSB    0x0022
#    define EMAC0_MII_OUI_LSB    0x05
#  elif defined(SAMV7_EMAC0_PHY_KSZ8061)
#    define EMAC0_MII_OUI_MSB    0x0022
#    define EMAC0_MII_OUI_LSB    0x05
#  elif defined(SAMV7_EMAC0_PHY_KSZ8081)
#    define EMAC0_MII_OUI_MSB    0x0022
#    define EMAC0_MII_OUI_LSB    0x05
#  else
#    error EMAC PHY unrecognized
#  endif
#endif /* CONFIG_SAMV7_EMAC0 */

/* EMAC1 Configuration ******************************************************/

#ifdef CONFIG_SAMV7_EMAC1
  /* Number of buffers for RX */

#  ifndef CONFIG_SAMV7_EMAC1_NRXBUFFERS
#    define CONFIG_SAMV7_EMAC1_NRXBUFFERS  16
#  endif

#  if CONFIG_SAMV7_EMAC1_NRXBUFFERS <= 1
#    error CONFIG_SAMV7_EMAC1_NRXBUFFERS invalid
#  endif

  /* Number of buffers for TX */

#  ifndef CONFIG_SAMV7_EMAC1_NTXBUFFERS
#    define CONFIG_SAMV7_EMAC1_NTXBUFFERS  8
#  endif

#  if CONFIG_SAMV7_EMAC1_NTXBUFFERS <= 1
#    error CONFIG_SAMV7_EMAC1_NTXBUFFERS invalid
#  endif

#  ifndef CONFIG_SAMV7_EMAC1_PHYADDR
#    error "CONFIG_SAMV7_EMAC1_PHYADDR must be defined in the NuttX configuration"
#  endif

#  if !defined(CONFIG_SAMV7_EMAC1_MII) && !defined(CONFIG_SAMV7_EMAC1_RMII)
#    warning "Neither CONFIG_SAMV7_EMAC1_MII nor CONFIG_SAMV7_EMAC1_RMII defined"
#  endif

#  if defined(CONFIG_SAMV7_EMAC1_MII) && defined(CONFIG_SAMV7_EMAC1_RMII)
#    error "Both CONFIG_SAMV7_EMAC1_MII and CONFIG_SAMV7_EMAC1_RMII defined"
#  endif

#  ifndef CONFIG_SAMV7_EMAC1_PHYSR
#    error "CONFIG_SAMV7_EMAC1_PHYSR must be defined in the NuttX configuration"
#  endif

#  ifdef CONFIG_SAMV7_EMAC1_AUTONEG
#    ifdef CONFIG_SAMV7_EMAC1_PHYSR_ALTCONFIG
#      ifndef CONFIG_SAMV7_EMAC1_PHYSR_ALTMODE
#        error "CONFIG_SAMV7_EMAC1_PHYSR_ALTMODE must be defined in the NuttX configuration"
#      endif
#      ifndef CONFIG_SAMV7_EMAC1_PHYSR_10HD
#        error "CONFIG_SAMV7_EMAC1_PHYSR_10HD must be defined in the NuttX configuration"
#      endif
#      ifndef CONFIG_SAMV7_EMAC1_PHYSR_100HD
#        error "CONFIG_SAMV7_EMAC1_PHYSR_100HD must be defined in the NuttX configuration"
#      endif
#      ifndef CONFIG_SAMV7_EMAC1_PHYSR_10FD
#        error "CONFIG_SAMV7_EMAC1_PHYSR_10FD must be defined in the NuttX configuration"
#      endif
#      ifndef CONFIG_SAMV7_EMAC1_PHYSR_100FD
#        error "CONFIG_SAMV7_EMAC1_PHYSR_100FD must be defined in the NuttX configuration"
#      endif
#    else
#      ifndef CONFIG_SAMV7_EMAC1_PHYSR_SPEED
#        error "CONFIG_SAMV7_EMAC1_PHYSR_SPEED must be defined in the NuttX configuration"
#      endif
#      ifndef CONFIG_SAMV7_EMAC1_PHYSR_100MBPS
#        error "CONFIG_SAMV7_EMAC1_PHYSR_100MBPS must be defined in the NuttX configuration"
#      endif
#      ifndef CONFIG_SAMV7_EMAC1_PHYSR_MODE
#        error "CONFIG_SAMV7_EMAC1_PHYSR_MODE must be defined in the NuttX configuration"
#      endif
#      ifndef CONFIG_SAMV7_EMAC1_PHYSR_FULLDUPLEX
#        error "CONFIG_SAMV7_EMAC1_PHYSR_FULLDUPLEX must be defined in the NuttX configuration"
#      endif
#    endif
#  endif

  /* PHY definitions */

#  if defined(SAMV7_EMAC1_PHY_DM9161)
#    define EMAC1_MII_OUI_MSB    0x0181
#    define EMAC1_MII_OUI_LSB    0x2e
#  elif defined(SAMV7_EMAC1_PHY_LAN8700)
#    define EMAC1_MII_OUI_MSB    0x0007
#    define EMAC1_MII_OUI_LSB    0x30
#  elif defined(SAMV7_EMAC1_PHY_KSZ8051)
#    define EMAC1_MII_OUI_MSB    0x0022
#    define EMAC1_MII_OUI_LSB    0x05
#  elif defined(SAMV7_EMAC1_PHY_KSZ8061)
#    define EMAC1_MII_OUI_MSB    0x0022
#    define EMAC1_MII_OUI_LSB    0x05
#  elif defined(SAMV7_EMAC1_PHY_KSZ8081)
#    define EMAC1_MII_OUI_MSB    0x0022
#    define EMAC1_MII_OUI_LSB    0x05
#  else
#    error EMAC PHY unrecognized
#  endif
#endif /* CONFIG_SAMV7_EMAC1 */

/* Common Configuration *****************************************************/

#undef CONFIG_SAMV7_EMAC_NBC

/* Extremely detailed register debug that you would normally never want
 * enabled.
 */

#ifdef CONFIG_NET_DUMPPACKET
#  define sam_dumppacket(m,a,n) lib_dumpbuffer(m,a,n)
#else
#  define sam_dumppacket(m,a,n)
#endif

#ifndef CONFIG_NET_INFO
#  undef CONFIG_SAMV7_EMAC_REGDEBUG
#endif

/* EMAC buffer sizes, number of buffers, and number of descriptors **********/

/* Queue identifiers/indices */

#define EMAC_QUEUE_0        0
#define EMAC_QUEUE_1        1
#define EMAC_QUEUE_2        2

/* After chip Revision A, the SAMv7 family increased from 3 to 6 queues. */

#define EMAC_QUEUE_3        3
#define EMAC_QUEUE_4        4
#define EMAC_QUEUE_5        5
#define EMAC_NQUEUES        (g_emac_nqueues)

#define EMAC_NQUEUES_REVA   3
#define EMAC_NQUEUES_REVB   6
#define EMAC_NQUEUES_MAX    6

/* Interrupt settings */

#define EMAC_RX_INTS        (EMAC_INT_RCOMP | EMAC_INT_RXUBR | EMAC_INT_ROVR)
#define EMAC_TXERR_INTS     (EMAC_INT_TUR | EMAC_INT_RLEX | EMAC_INT_TFC | \
                             EMAC_INT_HRESP)
#define EMAC_TX_INTS        (EMAC_TXERR_INTS | EMAC_INT_TCOMP)

/* Buffer Alignment.
 *
 * The EMAC peripheral requires that descriptors and buffers be aligned
 * the 8-byte (2 word boundaries).  However, if the data cache is enabled
 * the a higher level of alignment is required.  That is because the data
 * will need to be invalidated and that cache invalidation will occur in
 * multiples of full cache lines.
 *
 * In addition, padding may be required at the ends of the descriptors and
 * buffers to protect data after the end of from invalidation.
 */

#ifdef CONFIG_ARMV7M_DCACHE
/* Align to the cache line size which we assume is >= 8 */

#  define EMAC_ALIGN        ARMV7M_DCACHE_LINESIZE
#  define EMAC_ALIGN_MASK   (EMAC_ALIGN-1)
#  define EMAC_ALIGN_UP(n)  (((n) + EMAC_ALIGN_MASK) & ~EMAC_ALIGN_MASK)

#  define EMAC0_RX_DPADSIZE (EMAC0_RX_DESCSIZE & EMAC_ALIGN_MASK)
#  define EMAC0_TX_DPADSIZE (EMAC0_TX_DESCSIZE & EMAC_ALIGN_MASK)
#  define EMAC1_RX_DPADSIZE (EMAC1_RX_DESCSIZE & EMAC_ALIGN_MASK)
#  define EMAC1_TX_DPADSIZE (EMAC1_TX_DESCSIZE & EMAC_ALIGN_MASK)

#else
/* Use the minimum alignment requirement */

#  define EMAC_ALIGN        8
#  define EMAC_ALIGN_MASK   7
#  define EMAC_ALIGN_UP(n)  (((n) + 7) & ~7)

#  define EMAC0_RX_DPADSIZE 0
#  define EMAC0_TX_DPADSIZE 0
#  define EMAC1_RX_DPADSIZE 0
#  define EMAC1_TX_DPADSIZE 0

#endif

/* Buffer sizes.
 *
 * RX buffer size if fixed at 128 bytes since fragmented incoming packets
 * are handled.
 */

#define EMAC_RX_UNITSIZE    EMAC_ALIGN_UP(128)
#define EMAC_TX_UNITSIZE    EMAC_ALIGN_UP(CONFIG_NET_ETH_PKTSIZE)
#define DUMMY_BUFSIZE       EMAC_ALIGN_UP(128)
#define DUMMY_NBUFFERS      2

#define EMAC0_RX_DESCSIZE   (CONFIG_SAMV7_EMAC0_NRXBUFFERS * sizeof(struct emac_rxdesc_s))
#define EMAC0_TX_DESCSIZE   (CONFIG_SAMV7_EMAC0_NTXBUFFERS * sizeof(struct emac_txdesc_s))
#define EMAC0_RX_BUFSIZE    (CONFIG_SAMV7_EMAC0_NRXBUFFERS * EMAC_RX_UNITSIZE)
#define EMAC0_TX_BUFSIZE    (CONFIG_SAMV7_EMAC0_NTXBUFFERS * EMAC_TX_UNITSIZE)

#define EMAC1_RX_DESCSIZE   (CONFIG_SAMV7_EMAC1_NRXBUFFERS * sizeof(struct emac_rxdesc_s))
#define EMAC1_TX_DESCSIZE   (CONFIG_SAMV7_EMAC1_NTXBUFFERS * sizeof(struct emac_txdesc_s))
#define EMAC1_RX_BUFSIZE    (CONFIG_SAMV7_EMAC1_NRXBUFFERS * EMAC_RX_UNITSIZE)
#define EMAC1_TX_BUFSIZE    (CONFIG_SAMV7_EMAC1_NTXBUFFERS * EMAC_TX_UNITSIZE)

/* Timing *******************************************************************/

/* TX timeout = 1 minute */

#define SAM_TXTIMEOUT       (60*CLK_TCK)

/* PHY read/write delays in loop counts */

#define PHY_RETRY_MAX       1000000

/* Helpers ******************************************************************/

/* This is a helper pointer for accessing the contents of the EMAC
 * header
 */

#define BUF ((struct eth_hdr_s *)priv->dev.d_buf)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure defines the constant an configured attributes of an EMAC */

struct sam_emacattr_s
{
  /* Basic hardware information */

  uint32_t             base;         /* EMAC Register base address */
  uint8_t              emac;         /* EMACn, n=0 or 1 */
  uint8_t              irq;          /* EMAC interrupt number */

  /* PHY Configuration */

  uint8_t              phyaddr;      /* PHY address */
  uint8_t              physr;        /* PHY status register address */
  uint16_t             msoui;        /* MS 16 bits of the 18-bit OUI */
  uint8_t              lsoui;        /* LS 2 bits of the 18-bit OUI */
  bool                 rmii;         /* True: RMII vs. False: MII */
  bool                 clause45;     /* True: Clause 45 behavior */
  bool                 autoneg;      /* True: Autonegotiate rate and *plex */
  bool                 sralt;        /* True: Alternate PHYSR bit access */

  union
  {
  /* "Standard" form:  Individual bits determine speed and half/full
   * duplex.
   */

    struct
    {
      uint16_t stdmask;             /* Mask for speed and *plex mode bits */
      uint16_t speed100;            /* 100Base_t bit */
      uint16_t fduplex;             /* Full duplex bit */
    } std;

  /* Alternative form:  Speed and duplex are encoded in a single,
   * multi-bit field.
   */

    struct
    {
      uint16_t altmask;             /* Mask speed for mode bits */
      uint16_t hdx10;               /* 10Base_T Half Duplex bit pattern */
      uint16_t hdx100;              /* 100Base_T Half Duplex bit pattern */
      uint16_t fdx10;               /* 10Base_T Half Duplex bit pattern */
      uint16_t fdx100;              /* 100Base_T Half Duplex bit pattern */
    } alt;
  } u;

  /* Buffer and descriptor configuration */

  uint8_t              ntxbuffers;   /* Number of TX buffers */
  uint8_t              nrxbuffers;   /* Number of RX buffers */

#ifdef CONFIG_SAMV7_EMAC_PREALLOCATE
  /* Attributes and addresses of preallocated buffers */

  struct emac_txdesc_s *tx0desc;     /* Preallocated TX descriptor list */
  struct emac_rxdesc_s *rx0desc;     /* Preallocated RX descriptor list */
  uint8_t              *tx0buffer;   /* Preallocated TX buffers */
  uint8_t              *rx0buffer;   /* Preallocated RX buffers */

  struct emac_txdesc_s *tx1desc;     /* Preallocated TX dummy descriptor list */
  struct emac_rxdesc_s *rx1desc;     /* Preallocated RX dummy descriptor list */
  uint8_t              *tx1buffer;   /* Preallocated TX dummy buffers */
  uint8_t              *rx1buffer;   /* Preallocated RX dummy buffers */

#endif
};

/* This structure describes one transfer queue */

struct sam_queue_s
{
  struct emac_rxdesc_s *rxdesc;      /* Allocated RX descriptors */
  struct emac_txdesc_s *txdesc;      /* Allocated TX descriptors */
  uint8_t              *rxbuffer;    /* Allocated RX buffers */
  uint8_t              *txbuffer;    /* Allocated TX buffers */
  uint16_t              rxbufsize;   /* Size of one RX buffer */
  uint16_t              txbufsize;   /* Size of one TX buffer */
  uint8_t               nrxbuffers;  /* Number of RX buffers/TDs  */
  uint8_t               rxndx;       /* Current RX TD index */
  uint8_t               ntxbuffers;  /* Number of TX buffers/TDs */
  uint16_t              txhead;      /* Buffer head pointer */
  uint16_t              txtail;      /* Buffer tail pointer */
};

/* The sam_emac_s encapsulates all state information for EMAC peripheral */

struct sam_emac_s
{
  uint8_t               ifup    : 1; /* true:ifup false:ifdown */
  struct wdog_s         txtimeout;   /* TX timeout timer */
  struct work_s         irqwork;     /* For deferring work to the work queue */
  struct work_s         pollwork;    /* For deferring work to the work queue */

  /* This holds the information visible to the NuttX network */

  struct net_driver_s   dev;         /* Interface understood by the network */

  /* Constant and configured attributes of the EMAC */

  const struct sam_emacattr_s *attr;

  /* Used to track transmit and receive descriptors */

  uint8_t               phyaddr;     /* PHY address (pre-defined by pins on reset) */
#if defined(CONFIG_NETDEV_PHY_IOCTL) && defined(CONFIG_ARCH_PHY_INTERRUPT)
  uint8_t               phytype;     /* See SAMV7_EMAC0/1_PHY_TYPE definitions */
#endif

  /* Transfer queues */

  struct sam_queue_s    xfrq[EMAC_NQUEUES_MAX];

  /* Debug stuff */

#ifdef CONFIG_SAMV7_EMAC_REGDEBUG
  bool                  wrlast;     /* Last was a write */
  uintptr_t             addrlast;   /* Last address */
  uint32_t              vallast;    /* Last value */
  int                   ntimes;     /* Number of times */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register operations ******************************************************/

#ifdef CONFIG_SAMV7_EMAC_REGDEBUG
static bool sam_checkreg(struct sam_emac_s *priv, bool wr,
                         uint32_t regval, uintptr_t address);
#endif

static uint32_t sam_getreg(struct sam_emac_s *priv, uint16_t offset);
static void sam_putreg(struct sam_emac_s *priv,
                       uint16_t offset, uint32_t val);

/* Buffer management */

static uint16_t sam_txinuse(struct sam_emac_s *priv, int qid);
static uint16_t sam_txfree(struct sam_emac_s *priv, int qid);
static int  sam_buffer_allocate(struct sam_emac_s *priv);
static void sam_buffer_free(struct sam_emac_s *priv);

/* Common TX logic */

static int  sam_transmit(struct sam_emac_s *priv, int qid);
static int  sam_txpoll(struct net_driver_s *dev);
static void sam_dopoll(struct sam_emac_s *priv, int qid);

/* Interrupt handling */

static int  sam_recvframe(struct sam_emac_s *priv, int qid);
static void sam_receive(struct sam_emac_s *priv, int qid);
static void sam_txdone(struct sam_emac_s *priv, int qid);
static void sam_txerr_interrupt(struct sam_emac_s *priv, int qid);

static void sam_interrupt_work(void *arg);
static int  sam_emac_interrupt(int irq, void *context, void *arg);

/* Watchdog timer expirations */

static void sam_txtimeout_work(void *arg);
static void sam_txtimeout_expiry(wdparm_t arg);

/* NuttX callback functions */

static int  sam_ifup(struct net_driver_s *dev);
static int  sam_ifdown(struct net_driver_s *dev);

static void sam_txavail_work(void *arg);
static int  sam_txavail(struct net_driver_s *dev);

#if defined(CONFIG_NET_MCASTGROUP) || defined(CONFIG_NET_ICMPv6)
static unsigned int sam_hashindx(const uint8_t *mac);
static int  sam_addmac(struct net_driver_s *dev, const uint8_t *mac);
#endif
#ifdef CONFIG_NET_MCASTGROUP
static int  sam_rmmac(struct net_driver_s *dev, const uint8_t *mac);
#endif

#ifdef CONFIG_NETDEV_IOCTL
static int  sam_ioctl(struct net_driver_s *dev, int cmd, unsigned long arg);
#endif

/* PHY Initialization */

#if defined(CONFIG_DEBUG_NET) && defined(CONFIG_DEBUG_INFO)
static void sam_phydump(struct sam_emac_s *priv);
#else
#  define sam_phydump(priv)
#endif

#if 0 /* Not used */
static bool sam_is10hdx(struct sam_emac_s *priv, uint16_t physr);
#endif
static bool sam_is100hdx(struct sam_emac_s *priv, uint16_t physr);
static bool sam_is10fdx(struct sam_emac_s *priv, uint16_t physr);
static bool sam_is100fdx(struct sam_emac_s *priv, uint16_t physr);

#if defined(CONFIG_NETDEV_PHY_IOCTL) && defined(CONFIG_ARCH_PHY_INTERRUPT)
static int  sam_phyintenable(struct sam_emac_s *priv);
#endif
static int  sam_phywait(struct sam_emac_s *priv);
static int  sam_phyreset(struct sam_emac_s *priv);
static int  sam_phyfind(struct sam_emac_s *priv, uint8_t *phyaddr);
static int  sam_phyread(struct sam_emac_s *priv, uint8_t phyaddr,
                        uint8_t regaddr, uint16_t *phyval);
static int  sam_phywrite(struct sam_emac_s *priv, uint8_t phyaddr,
                         uint8_t regaddr, uint16_t phyval);
static int  sam_autonegotiate(struct sam_emac_s *priv);
static bool sam_linkup(struct sam_emac_s *priv);
static int  sam_phyinit(struct sam_emac_s *priv);

/* EMAC Initialization */

static void sam_txreset(struct sam_emac_s *priv, int qid);
static void sam_rxreset(struct sam_emac_s *priv, int qid);
static void sam_emac_enableclk(struct sam_emac_s *priv);
#ifndef CONFIG_NETDEV_PHY_IOCTL
static void sam_emac_disableclk(struct sam_emac_s *priv);
#endif
static void sam_emac_reset(struct sam_emac_s *priv);
static void sam_macaddress(struct sam_emac_s *priv);
#ifdef CONFIG_NET_ICMPv6
static void sam_ipv6multicast(struct sam_emac_s *priv);
#endif

static int  sam_queue0_configure(struct sam_emac_s *priv);
static int  sam_queue_configure(struct sam_emac_s *priv, int qid);
static int  sam_emac_configure(struct sam_emac_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_SAMV7_EMAC_PREALLOCATE
/* Preallocated data */

#ifdef CONFIG_SAMV7_EMAC0
/* EMAC0 TX descriptors list */

static struct emac_txdesc_s g_emac0_tx0desc[CONFIG_SAMV7_EMAC0_NTXBUFFERS]
              aligned_data(EMAC_ALIGN);

#if EMAC0_TX_DPADSIZE > 0
static uint8_t g_emac0_txdpad[EMAC0_TX_DPADSIZE] __atrribute__((used));
#endif

static struct emac_txdesc_s g_emac0_tx1desc[DUMMY_NBUFFERS]
              aligned_data(EMAC_ALIGN);

/* EMAC0 RX descriptors list */

static struct emac_rxdesc_s g_emac0_rx0desc[CONFIG_SAMV7_EMAC0_NRXBUFFERS]
              aligned_data(EMAC_ALIGN);

#if EMAC0_RX_DPADSIZE > 0
static uint8_t g_emac0_rxdpad[EMAC0_RX_DPADSIZE] __atrribute__((used));
#endif

static struct emac_rxdesc_s g_emac0_rx1desc[DUMMY_NBUFFERS]
              aligned_data(EMAC_ALIGN);

/* EMAC0 Transmit Buffers
 *
 * Section 3.6 of AMBA 2.0 spec states that burst should not cross 1K
 * Boundaries. Receive buffer manager writes are burst of 2 words => 3
 * lsb bits of the address shall be set to 0
 */

static uint8_t g_emac0_tx0buffer[EMAC0_TX_BUFSIZE]
               aligned_data(EMAC_ALIGN);

static uint8_t g_emac0_tx1buffer[DUMMY_NBUFFERS * DUMMY_BUFSIZE]
               aligned_data(EMAC_ALIGN);

/* EMAC0 Receive Buffers */

static uint8_t g_emac0_rx0buffer[EMAC0_RX_BUFSIZE]
               aligned_data(EMAC_ALIGN);

#endif

#ifdef CONFIG_SAMV7_EMAC1
/* EMAC1 TX descriptors list */

static struct emac_txdesc_s g_emac1_tx1desc[CONFIG_SAMV7_EMAC1_NTXBUFFERS]
              aligned_data(EMAC_ALIGN);

#if EMAC1_TX_DPADSIZE > 0
static uint8_t g_emac1_txdpad[EMAC1_TX_DPADSIZE] __atrribute__((used));
#endif

static struct emac_txdesc_s g_emac1_tx1desc[DUMMY_NBUFFERS]
              aligned_data(EMAC_ALIGN);

/* EMAC1 RX descriptors list */

static struct emac_rxdesc_s g_emac1_rx1desc[CONFIG_SAMV7_EMAC1_NRXBUFFERS]
              aligned_data(EMAC_ALIGN);

#if EMAC1_RX_DPADSIZE > 0
static uint8_t g_emac1_rxdpad[EMAC1_RX_DPADSIZE] __atrribute__((used));
#endif

static struct emac_rxdesc_s g_emac1_rx1desc[DUMMY_NBUFFERS]
              aligned_data(EMAC_ALIGN);

/* EMAC1 Transmit Buffers
 *
 * Section 3.6 of AMBA 2.0 spec states that burst should not cross 1K
 * Boundaries. Receive buffer manager writes are burst of 2 words => 3
 * lsb bits of the address shall be set to 0
 */

static uint8_t g_emac1_tx1buffer[EMAC1_TX_BUFSIZE]
               aligned_data(EMAC_ALIGN);

static uint8_t g_emac1_tx1buffer[DUMMY_NBUFFERS * DUMMY_BUFSIZE]
               aligned_data(EMAC_ALIGN);

/* EMAC1 Receive Buffers */

static uint8_t g_emac1_rxbuffer[EMAC1_RX_BUFSIZE]
               aligned_data(EMAC_ALIGN);

static uint8_t g_emac1_rx1buffer[DUMMY_NBUFFERS * DUMMY_BUFSIZE]
               aligned_data(EMAC_ALIGN);

#endif
#endif

/* The driver state singletons */

#ifdef CONFIG_SAMV7_EMAC0
static const struct sam_emacattr_s g_emac0_attr =
{
  /* Basic hardware information */

  .base         = SAM_EMAC0_BASE,
  .emac         = EMAC0_INTF,
  .irq          = SAM_IRQ_EMAC0,

  /* PHY Configuration */

  .phyaddr      = CONFIG_SAMV7_EMAC0_PHYADDR,
  .physr        = CONFIG_SAMV7_EMAC0_PHYSR,
  .msoui        = EMAC0_MII_OUI_MSB,
  .lsoui        = EMAC0_MII_OUI_LSB,
#ifdef CONFIG_SAMV7_EMAC0_RMII
  .rmii         = true,
#endif
#ifdef CONFIG_SAMV7_EMAC0_CLAUSE45
  .clause45     = true,
#endif
#ifdef CONFIG_SAMV7_EMAC0_AUTONEG
  .autoneg      = true,
#endif
#ifdef CONFIG_SAMV7_EMAC0_PHYSR_ALTCONFIG
  .sralt        = true,
#endif

  .u            =
  {
#ifdef CONFIG_SAMV7_EMAC0_PHYSR_ALTCONFIG
    .alt        =
    {
      .altmask  = CONFIG_SAMV7_EMAC0_PHYSR_ALTMODE,
      .hdx10    = CONFIG_SAMV7_EMAC0_PHYSR_10HD,
      .hdx100   = CONFIG_SAMV7_EMAC0_PHYSR_100HD,
      .fdx10    = CONFIG_SAMV7_EMAC0_PHYSR_10FD,
      .fdx100   = CONFIG_SAMV7_EMAC0_PHYSR_100FD,
    },
#else
    .std        =
    {
      .stdmask  = CONFIG_SAMV7_EMAC0_PHYSR_SPEED |
                  CONFIG_SAMV7_EMAC0_PHYSR_MODE,
      .speed100 = CONFIG_SAMV7_EMAC0_PHYSR_100MBPS,
      .fduplex  = CONFIG_SAMV7_EMAC0_PHYSR_FULLDUPLEX,
    },
#endif
  },

  /* Buffer and descriptor configuration */

  .ntxbuffers   = CONFIG_SAMV7_EMAC0_NTXBUFFERS,
  .nrxbuffers   = CONFIG_SAMV7_EMAC0_NRXBUFFERS,

#ifdef CONFIG_SAMV7_EMAC_PREALLOCATE
  /* Addresses of preallocated buffers */

  .tx0desc      = g_emac0_tx0desc,
  .rx0desc      = g_emac0_rx0desc,
  .tx0buffer    = g_emac0_tx0buffer,
  .rx0buffer    = g_emac0_rx0buffer,
#endif
};

/* A single packet buffer is used
 *
 * REVISIT:  It might be possible to use this option to send and receive
 * messages directly into the DMA buffers, saving a copy.  There might be
 * complications on the receiving side, however, where buffers may wrap
 * and where the size of the received frame will typically be smaller than
 * a full packet.
 */

static uint8_t g_pktbuf0[MAX_NETDEV_PKTSIZE + CONFIG_NET_GUARDSIZE];

/* EMAC0 peripheral state */

static struct sam_emac_s g_emac0;
#endif

#ifdef CONFIG_SAMV7_EMAC1
static const struct sam_emacattr_s g_emac1_attr =
{
  /* Basic hardware information */

  .base         = SAM_EMAC1_BASE,
  .emac         = EMAC1_INTF,
  .irq          = SAM_IRQ_EMAC1,

  /* PHY Configuration */

  .phyaddr      = CONFIG_SAMV7_EMAC1_PHYADDR,
  .physr        = CONFIG_SAMV7_EMAC1_PHYSR,
  .msoui        = EMAC1_MII_OUI_MSB,
  .lsoui        = EMAC1_MII_OUI_LSB,
#ifdef CONFIG_SAMV7_EMAC1_RMII
  .rmii         = true,
#endif
#ifdef CONFIG_SAMV7_EMAC1_CLAUSE45
  .clause45     = true,
#endif
#ifdef CONFIG_SAMV7_EMAC1_AUTONEG
  .autoneg      = true,
#endif
#ifdef CONFIG_SAMV7_EMAC1_PHYSR_ALTCONFIG
  .sralt        = true,
#endif

  .u            =
  {
#ifdef CONFIG_SAMV7_EMAC1_PHYSR_ALTCONFIG
    .alt        =
    {
      .altmask  = CONFIG_SAMV7_EMAC1_PHYSR_ALTMODE,
      .hdx10    = CONFIG_SAMV7_EMAC1_PHYSR_10HD,
      .hdx100   = CONFIG_SAMV7_EMAC1_PHYSR_100HD,
      .fdx10    = CONFIG_SAMV7_EMAC1_PHYSR_10FD,
      .fdx100   = CONFIG_SAMV7_EMAC1_PHYSR_100FD,
    },
#else
    .std        =
    {
      .stdmask  = CONFIG_SAMV7_EMAC1_PHYSR_SPEED |
                  CONFIG_SAMV7_EMAC1_PHYSR_MODE,
      .speed100 = CONFIG_SAMV7_EMAC1_PHYSR_100MBPS,
      .fduplex  = CONFIG_SAMV7_EMAC1_PHYSR_FULLDUPLEX,
    },
#endif
  },

  /* Buffer and descriptor configuration */

  .ntxbuffers   = CONFIG_SAMV7_EMAC1_NTXBUFFERS,
  .nrxbuffers   = CONFIG_SAMV7_EMAC1_NRXBUFFERS,

#ifdef CONFIG_SAMV7_EMAC_PREALLOCATE
  /* Attributes and addresses of preallocated buffers */

  .txdesc       = g_emac1_tx0desc,
  .rxdesc       = g_emac1_rx0desc,
  .txbuffer     = g_emac1_tx0buffer,
  .rxbuffer     = g_emac1_rxbuffer,
#endif
};

/* A single packet buffer is used
 *
 * REVISIT:  It might be possible to use this option to send and receive
 * messages directly into the DMA buffers, saving a copy.  There might be
 * complications on the receiving side, however, where buffers may wrap
 * and where the size of the received frame will typically be smaller than
 * a full packet.
 */

static uint8_t g_pktbuf1[MAX_NETDEV_PKTSIZE + CONFIG_NET_GUARDSIZE];

/* EMAC1 peripheral state */

static struct sam_emac_s g_emac1;

#endif /* CONFIG_SAMV7_EMAC1 */

/* The SAMv7 may support from 3 to 6 queue, depending upon the chip
 * revision.  NOTE that this is a global setting and applies to both
 * EMAC peripherals.
 */

static uint8_t g_emac_nqueues = EMAC_NQUEUES_REVA; /* Assume Rev A */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_checkreg
 *
 * Description:
 *   Check if the current register access is a duplicate of the preceding.
 *
 * Input Parameters:
 *   regval  - The value to be written
 *   address - The address of the register to write to
 *
 * Returned Value:
 *   true:  This is the first register access of this type.
 *   flase: This is the same as the preceding register access.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMV7_EMAC_REGDEBUG
static bool sam_checkreg(struct sam_emac_s *priv, bool wr, uint32_t regval,
                         uintptr_t regaddr)
{
  if (wr      == priv->wrlast &&   /* Same kind of access? */
      regval  == priv->vallast &&  /* Same value? */
      regaddr == priv->addrlast)   /* Same address? */
    {
      /* Yes, then just keep a count of the number of times we did this. */

      priv->ntimes++;
      return false;
    }
  else
    {
      /* Did we do the previous operation more than once? */

      if (priv->ntimes > 0)
        {
          /* Yes... show how many times we did it */

          ninfo("...[Repeats %d times]...\n", priv->ntimes);
        }

      /* Save information about the new access */

      priv->wrlast   = wr;
      priv->vallast  = regval;
      priv->addrlast = regaddr;
      priv->ntimes   = 0;
    }

  /* Return true if this is the first time that we have done this operation */

  return true;
}
#endif

/****************************************************************************
 * Name: sam_getreg
 *
 * Description:
 *  Read a 32-bit EMAC register using an offset from the EMAC base address
 *
 ****************************************************************************/

static uint32_t sam_getreg(struct sam_emac_s *priv, uint16_t offset)
{
  uintptr_t regaddr = priv->attr->base + (uintptr_t)offset;
  uint32_t regval = getreg32(regaddr);

#ifdef CONFIG_SAMV7_EMAC_REGDEBUG
  if (sam_checkreg(priv, false, regval, regaddr))
    {
      ninfo("%08x->%08x\n", regaddr, regval);
    }
#endif

  return regval;
}

/****************************************************************************
 * Name: sam_putreg
 *
 * Description:
 *  Write to a 32-bit EMAC register using an offset from the EMAC base
 *  address
 *
 ****************************************************************************/

static void sam_putreg(struct sam_emac_s *priv,
                       uint16_t offset, uint32_t regval)
{
  uintptr_t regaddr = priv->attr->base + (uintptr_t)offset;

#ifdef CONFIG_SAMV7_EMAC_REGDEBUG
  if (sam_checkreg(priv, true, regval, regaddr))
    {
      ninfo("%08x<-%08x\n", regaddr, regval);
    }
#endif

  putreg32(regval, regaddr);
}

/****************************************************************************
 * Function: sam_txinuse
 *
 * Description:
 *   Return the number of TX buffers in-use
 *
 * Input Parameters:
 *   priv - The EMAC driver state
 *   quid - The transfer queue to examine
 *
 * Returned Value:
 *   The number of TX buffers in-use
 *
 ****************************************************************************/

static uint16_t sam_txinuse(struct sam_emac_s *priv, int qid)
{
  struct sam_queue_s *xfrq = &priv->xfrq[qid];

  uint32_t txhead32 = (uint32_t)xfrq->txhead;
  if ((uint32_t)xfrq->txtail > txhead32)
    {
      txhead32 += xfrq->ntxbuffers;
    }

  return (uint16_t)(txhead32 - (uint32_t)xfrq->txtail);
}

/****************************************************************************
 * Function: sam_txfree
 *
 * Description:
 *   Return the number of TX buffers available
 *
 * Input Parameters:
 *   priv - The EMAC driver state
 *   qid  - The transfer queue to examine
 *
 * Returned Value:
 *   The number of TX buffers available
 *
 ****************************************************************************/

static uint16_t sam_txfree(struct sam_emac_s *priv, int qid)
{
  /* The number available is equal to the total number of buffers, minus the
   * number of buffers in use.  Notice that that actual number of buffers is
   * the configured size minus 1.
   */

  return (priv->xfrq[qid].ntxbuffers - 1) - sam_txinuse(priv, qid);
}

/****************************************************************************
 * Function: sam_buffer_allocate
 *
 * Description:
 *   Allocate aligned TX and RX descriptors and buffers.  For the case of
 *   pre-allocated structures, the function degenerates to a few assignments.
 *
 * Input Parameters:
 *   priv - The EMAC driver state
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *   Called very early in the initialization sequence.
 *
 ****************************************************************************/

static int sam_buffer_allocate(struct sam_emac_s *priv)
{
#ifdef CONFIG_SAMV7_EMAC_PREALLOCATE
  struct sam_queue_s *xfrq;
  int qid;

  /* Use pre-allocated buffers */

  priv->xfrq[0].txdesc         = priv->attr->tx0desc;
  priv->xfrq[0].ntxbuffers     = priv->attr->ntxbuffers;
  priv->xfrq[0].rxdesc         = priv->attr->rx0desc;
  priv->xfrq[0].nrxbuffers     = priv->attr->nrxbuffers;

  priv->xfrq[0].txbuffer       = priv->attr->tx0buffer;
  priv->xfrq[0].txbufsize      = EMAC_TX_UNITSIZE;
  priv->xfrq[0].rxbuffer       = priv->attr->rx0buffer;
  priv->xfrq[0].rxbufsize      = EMAC_RX_UNITSIZE;

  /* Priority queues */

  for (qid = 1; qid < EMAC_NQUEUES; qid++)
    {
      xfrq             = &priv->xfrq[qid];

      xfrq->txdesc     = priv->attr->tx1desc;
      xfrq->ntxbuffers = DUMMY_NBUFFERS;
      xfrq->rxdesc     = priv->attr->rx1desc;
      xfrq->nrxbuffers = DUMMY_NBUFFERS;

      xfrq->txbuffer   = priv->attr->tx1buffer;
      xfrq->txbufsize  = DUMMY_BUFSIZE;
      xfrq->rxbuffer   = priv->attr->rx1buffer;
      xfrq->rxbufsize  = DUMMY_BUFSIZE;
    }

#else
  struct sam_queue_s *xfrq;
  size_t allocsize;
  int qid;

  /* Allocate Queue 0 buffers */

  allocsize = EMAC_ALIGN_UP(priv->attr->ntxbuffers *
                            sizeof(struct emac_txdesc_s));
  priv->xfrq[0].txdesc = kmm_memalign(EMAC_ALIGN, allocsize);
  if (!priv->xfrq[0].txdesc)
    {
      nerr("ERROR: Failed to allocate TX descriptors\n");
      return -ENOMEM;
    }

  memset(priv->xfrq[0].txdesc, 0, allocsize);
  priv->xfrq[0].ntxbuffers = priv->attr->ntxbuffers;

  allocsize = EMAC_ALIGN_UP(priv->attr->nrxbuffers *
                            sizeof(struct emac_rxdesc_s));
  priv->xfrq[0].rxdesc = kmm_memalign(EMAC_ALIGN, allocsize);
  if (!priv->xfrq[0].rxdesc)
    {
      nerr("ERROR: Failed to allocate RX descriptors\n");
      sam_buffer_free(priv);
      return -ENOMEM;
    }

  memset(priv->xfrq[0].rxdesc, 0, allocsize);
  priv->xfrq[0].nrxbuffers = priv->attr->nrxbuffers;

  allocsize = priv->attr->ntxbuffers * EMAC_TX_UNITSIZE;
  priv->xfrq[0].txbuffer = (uint8_t *)kmm_memalign(EMAC_ALIGN, allocsize);
  if (!priv->xfrq[0].txbuffer)
    {
      nerr("ERROR: Failed to allocate TX buffer\n");
      sam_buffer_free(priv);
      return -ENOMEM;
    }

  priv->xfrq[0].txbufsize = EMAC_TX_UNITSIZE;

  allocsize = priv->attr->nrxbuffers * EMAC_RX_UNITSIZE;
  priv->xfrq[0].rxbuffer = (uint8_t *)kmm_memalign(EMAC_ALIGN, allocsize);
  if (!priv->xfrq[0].rxbuffer)
    {
      nerr("ERROR: Failed to allocate RX buffer\n");
      sam_buffer_free(priv);
      return -ENOMEM;
    }

  priv->xfrq[0].rxbufsize = EMAC_RX_UNITSIZE;

  /* Allocate Queue 1 buffers */

  allocsize = EMAC_ALIGN_UP(DUMMY_NBUFFERS * sizeof(struct emac_txdesc_s));
  priv->xfrq[1].txdesc = kmm_memalign(EMAC_ALIGN, allocsize);
  if (!priv->xfrq[1].txdesc)
    {
      nerr("ERROR: Failed to allocate TX descriptors\n");
      return -ENOMEM;
    }

  memset(priv->xfrq[1].txdesc, 0, allocsize);
  priv->xfrq[1].ntxbuffers = DUMMY_NBUFFERS;

  allocsize = EMAC_ALIGN_UP(DUMMY_NBUFFERS * sizeof(struct emac_rxdesc_s));
  priv->xfrq[1].rxdesc = kmm_memalign(EMAC_ALIGN, allocsize);
  if (!priv->xfrq[1].rxdesc)
    {
      nerr("ERROR: Failed to allocate RX descriptors\n");
      sam_buffer_free(priv);
      return -ENOMEM;
    }

  memset(priv->xfrq[1].rxdesc, 0, allocsize);
  priv->xfrq[1].nrxbuffers = DUMMY_NBUFFERS;

  allocsize = DUMMY_NBUFFERS * DUMMY_BUFSIZE;
  priv->xfrq[1].txbuffer = (uint8_t *)kmm_memalign(EMAC_ALIGN, allocsize);
  if (!priv->xfrq[1].txbuffer)
    {
      nerr("ERROR: Failed to allocate TX buffer\n");
      sam_buffer_free(priv);
      return -ENOMEM;
    }

  priv->xfrq[1].txbufsize = DUMMY_BUFSIZE;

  allocsize = DUMMY_NBUFFERS * DUMMY_BUFSIZE;
  priv->xfrq[1].rxbuffer = (uint8_t *)kmm_memalign(EMAC_ALIGN, allocsize);
  if (!priv->xfrq[1].rxbuffer)
    {
      nerr("ERROR: Failed to allocate RX buffer\n");
      sam_buffer_free(priv);
      return -ENOMEM;
    }

  priv->xfrq[1].rxbufsize = DUMMY_BUFSIZE;

  /* Clone the Priority Queue 1 allocations to the other priority queues */

  for (qid = 2; qid < EMAC_NQUEUES; qid++)
    {
      xfrq             = &priv->xfrq[qid];

      xfrq->txdesc     = priv->xfrq[1].txdesc;
      xfrq->rxdesc     = priv->xfrq[1].rxdesc;

      xfrq->txbuffer   = priv->xfrq[1].txbuffer;
      xfrq->rxbuffer   = priv->xfrq[1].rxbuffer;

      xfrq->ntxbuffers = DUMMY_NBUFFERS;
      xfrq->nrxbuffers = DUMMY_NBUFFERS;

      xfrq->txbufsize  = DUMMY_BUFSIZE;
      xfrq->rxbufsize  = DUMMY_BUFSIZE;
    }

#endif

  /* Verify Alignment */

  DEBUGASSERT(((uintptr_t)priv->xfrq[0].rxdesc   & EMAC_ALIGN_MASK) == 0 &&
              ((uintptr_t)priv->xfrq[0].rxbuffer & EMAC_ALIGN_MASK) == 0 &&
              ((uintptr_t)priv->xfrq[0].txdesc   & EMAC_ALIGN_MASK) == 0 &&
              ((uintptr_t)priv->xfrq[0].txbuffer & EMAC_ALIGN_MASK) == 0);
  DEBUGASSERT(((uintptr_t)priv->xfrq[1].rxdesc   & EMAC_ALIGN_MASK) == 0 &&
              ((uintptr_t)priv->xfrq[1].rxbuffer & EMAC_ALIGN_MASK) == 0 &&
              ((uintptr_t)priv->xfrq[1].txdesc   & EMAC_ALIGN_MASK) == 0 &&
              ((uintptr_t)priv->xfrq[1].txbuffer & EMAC_ALIGN_MASK) == 0);

  return OK;
}

/****************************************************************************
 * Function: sam_buffer_free
 *
 * Description:
 *   Free aligned TX and RX descriptors and buffers.  For the case of
 *   pre-allocated structures, the function does nothing.
 *
 * Input Parameters:
 *   priv - The EMAC driver state
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sam_buffer_free(struct sam_emac_s *priv)
{
#ifndef CONFIG_SAMV7_EMAC_PREALLOCATE
  struct sam_queue_s *xfrq;
  int qid;

  /* Free allocated buffers */

  for (qid = 0; qid < EMAC_NQUEUES; qid++)
    {
      xfrq = &priv->xfrq[qid];
      if (qid < 2)
        {
          if (xfrq->txdesc)
            {
              kmm_free(xfrq->txdesc);
              xfrq->txdesc = NULL;
            }

          if (xfrq->rxdesc)
            {
              kmm_free(xfrq->rxdesc);
              xfrq->rxdesc = NULL;
            }

          if (xfrq->txbuffer)
            {
              kmm_free(xfrq->txbuffer);
              xfrq->txbuffer = NULL;
            }

          if (xfrq->rxbuffer)
            {
              kmm_free(xfrq->rxbuffer);
              xfrq->rxbuffer = NULL;
            }
        }

      xfrq->txdesc = NULL;
      xfrq->rxdesc = NULL;
      xfrq->txbuffer = NULL;
      xfrq->rxbuffer = NULL;
    }
#endif
}

/****************************************************************************
 * Function: sam_transmit
 *
 * Description:
 *   Start hardware transmission.  Called either from the txdone interrupt
 *   handling or from watchdog based polling.
 *
 *   REVISIT:  This implementation does not support scatter-gather DMA.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *   qid   - The queue to send the frame
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

static int sam_transmit(struct sam_emac_s *priv, int qid)
{
  struct net_driver_s *dev = &priv->dev;
  volatile struct emac_txdesc_s *txdesc;
  struct sam_queue_s *xfrq;
  uint32_t regval;
  uint32_t status;
  uint16_t txhead;

  NETDEV_TXPACKETS(&priv->dev);

  /* Check parameter */

  if (dev->d_len > EMAC_TX_UNITSIZE)
    {
      nerr("ERROR: Packet too big: %d\n", dev->d_len);
      return -EINVAL;
    }

  /* Pointer to the current TX descriptor */

  xfrq   = &priv->xfrq[qid];
  txhead = xfrq->txhead;
  txdesc = &xfrq->txdesc[txhead];

  ninfo("d_len: %d txhead[%d]: %d\n",  dev->d_len, qid, xfrq->txhead);
  sam_dumppacket("Transmit packet", dev->d_buf, dev->d_len);

  /* If no free TX descriptor, buffer can't be sent */

  if (sam_txfree(priv, qid) < 1)
    {
      nerr("ERROR: No free TX descriptors\n");
      return -EBUSY;
    }

  /* Setup/Copy data to transmission buffer */

  if (dev->d_len > 0)
    {
      /* Copy the data into the driver managed the ring buffer.  If we
       * wanted to support zero copy transfers, we would need to make sure
       * that dev->d_buf and txdesc->addr refer to the same memory.
       */

      memcpy((void *)txdesc->addr, dev->d_buf, dev->d_len);
      up_clean_dcache((uint32_t)txdesc->addr,
                      (uint32_t)txdesc->addr + dev->d_len);
    }

  /* Update TX descriptor status (with USED=0). */

  status = dev->d_len | EMACTXD_STA_LAST;
  if (txhead == xfrq->ntxbuffers - 1)
    {
      status |= EMACTXD_STA_WRAP;
    }

  /* Update the descriptor status and flush the updated value to RAM */

  txdesc->status = status;
  up_clean_dcache((uint32_t)txdesc,
                  (uint32_t)txdesc + sizeof(struct emac_txdesc_s));

  /* Increment the head index */

  if (++txhead >= xfrq->ntxbuffers)
    {
      txhead = 0;
    }

  xfrq->txhead = txhead;

  /* Now start transmission (if it is not already done) */

  regval  = sam_getreg(priv, SAM_EMAC_NCR_OFFSET);
  regval |= EMAC_NCR_TSTART;
  sam_putreg(priv, SAM_EMAC_NCR_OFFSET, regval);

  /* REVISIT: Sometimes TSTART is missed?  In this case, the symptom is
   * that the packet is not sent until the next transfer when TXSTART
   * is set again.
   */

  sam_putreg(priv, SAM_EMAC_NCR_OFFSET, regval);

  /* Setup the TX timeout watchdog (perhaps restarting the timer) */

  wd_start(&priv->txtimeout, SAM_TXTIMEOUT,
           sam_txtimeout_expiry, (wdparm_t)priv);

  /* Set d_len to zero meaning that the d_buf[] packet buffer is again
   * available.
   */

  dev->d_len = 0;

  /* If we have no more available TX descriptors, then we must disable the
   * RCOMP interrupt to stop further RX processing.  Why?  Because EACH RX
   * packet that is dispatched is also an opportunity to replay with a TX
   * packet.  So, if we cannot handle an RX packet reply, then we disable
   * all RX packet processing.
   */

  if (sam_txfree(priv, qid) < 1)
    {
      ninfo("Disabling RX interrupts\n");
      sam_putreg(priv, SAM_EMAC_IDR_OFFSET, EMAC_INT_RCOMP);
    }

  return OK;
}

/****************************************************************************
 * Function: sam_txpoll
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

static int sam_txpoll(struct net_driver_s *dev)
{
  struct sam_emac_s *priv = (struct sam_emac_s *)dev->d_private;

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

          sam_transmit(priv, EMAC_QUEUE_0);

          /* Check if there are any free TX descriptors.  We cannot perform
           * the TX poll if we do not have buffering for another packet.
           */

          if (sam_txfree(priv, EMAC_QUEUE_0) == 0)
            {
              /* We have to terminate the poll if we have no more descriptors
               * available for another transfer.
               */

              return -EBUSY;
            }
        }
    }

  /* If zero is returned, the polling will continue until all connections
   * have been examined.
   */

  return 0;
}

/****************************************************************************
 * Function: sam_dopoll
 *
 * Description:
 *   The function is called in order to perform an out-of-sequence TX poll.
 *   This is done:
 *
 *   1. After completion of a transmission (sam_txdone),
 *   2. When new TX data is available (sam_txavail_process),
 *   3. For certain TX errors (sam_txerr_interrupt), and
 *   4. After a TX timeout to restart the sending process
 *      (sam_txtimeout_process).
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *   qid  - The transfer queue to send packets on
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static void sam_dopoll(struct sam_emac_s *priv, int qid)
{
  struct net_driver_s *dev = &priv->dev;

  /* Check if there are any free TX descriptors.  We cannot perform the
   * TX poll if we do not have buffering for another packet.
   */

  if (sam_txfree(priv, qid) > 0)
    {
      /* If we have the descriptor,
       * then poll the network for new XMIT data.
       */

      devif_poll(dev, sam_txpoll);
    }
}

/****************************************************************************
 * Function: sam_recvframe
 *
 * Description:
 *   The function is called when a frame is received. It scans the RX
 *   descriptors of the received frame and assembles the full packet/
 *
 *   NOTE: This function will silently discard any packets containing errors.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *   qid   - The transfer queue to receive the frame
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

static int sam_recvframe(struct sam_emac_s *priv, int qid)
{
  volatile struct emac_rxdesc_s *rxdesc;
  struct sam_queue_s *xfrq;
  struct net_driver_s *dev;
  const uint8_t *src;
  uint8_t  *dest;
  uint32_t rxndx;
  uint32_t pktlen;
  uint16_t copylen;
  bool isframe;

  /* Process received RX descriptor.  The ownership bit is set by the EMAC
   * once it has successfully written a frame to memory.
   */

  dev        = &priv->dev;
  dev->d_len = 0;

  dest       = dev->d_buf;
  pktlen     = 0;

  xfrq       = &priv->xfrq[qid];
  rxndx      = xfrq->rxndx;
  rxdesc     = &xfrq->rxdesc[rxndx];
  isframe    = false;

  /* Invalidate the RX descriptor to force re-fetching from RAM. */

  up_invalidate_dcache((uintptr_t)rxdesc,
                       (uintptr_t)rxdesc + sizeof(struct emac_rxdesc_s));

  ninfo("Entry rxndx[%d]: %" PRId32 "\n", qid, rxndx);

  while ((rxdesc->addr & EMACRXD_ADDR_OWNER) != 0)
    {
      NETDEV_RXFRAGMENTS(&priv->dev);

      /* The start of frame bit indicates the beginning of a frame.  Discard
       * any previous fragments.
       */

      if ((rxdesc->status & EMACRXD_STA_SOF) != 0)
        {
          /* Skip previous fragments.  Loop incrementing the index to the
           * start fragment until it is equal to the index with the SOF mark
           */

          while (rxndx != xfrq->rxndx)
            {
              /* Give ownership back to the EMAC */

              rxdesc = &xfrq->rxdesc[xfrq->rxndx];
              rxdesc->addr &= ~(EMACRXD_ADDR_OWNER);

              /* Flush the modified RX descriptor to RAM */

              up_clean_dcache((uintptr_t)rxdesc,
                              (uintptr_t)rxdesc +
                              sizeof(struct emac_rxdesc_s));

              /* Increment the RX index to the start fragment */

              if (++xfrq->rxndx >= xfrq->nrxbuffers)
                {
                  xfrq->rxndx = 0;
                }
            }

          /* Reset the packet data pointer and packet length */

          dest   = dev->d_buf;
          pktlen = 0;

          /* Start to gather buffers into the packet buffer */

          isframe = true;
        }

      /* Increment the working index. rxndx points to the next fragment
       * in this frame (or, if the LAST bit was set, to the first fragment
       * of the next frame).
       */

      if (++rxndx >= xfrq->nrxbuffers)
        {
          rxndx = 0;
        }

      /* Copy data into the packet buffer */

      if (isframe)
        {
          if (rxndx == xfrq->rxndx)
            {
              nerr("ERROR: No EOF (Invalid or buffers too small)\n");
              do
                {
                  /* Give ownership back to the EMAC */

                  rxdesc = &xfrq->rxdesc[xfrq->rxndx];
                  rxdesc->addr &= ~(EMACRXD_ADDR_OWNER);

                  /* Flush the modified RX descriptor to RAM */

                  up_clean_dcache((uintptr_t)rxdesc,
                                  (uintptr_t)rxdesc +
                                  sizeof(struct emac_rxdesc_s));

                  /* Increment the RX index */

                  if (++xfrq->rxndx >= xfrq->nrxbuffers)
                    {
                      xfrq->rxndx = 0;
                    }
                }
              while (rxndx != xfrq->rxndx);

              NETDEV_RXERRORS(&priv->dev);
              return -EIO;
            }

          /* Get the number of bytes to copy from the buffer */

          copylen = EMAC_RX_UNITSIZE;
          if ((pktlen + copylen) > CONFIG_NET_ETH_PKTSIZE)
            {
              copylen = CONFIG_NET_ETH_PKTSIZE - pktlen;
            }

          /* Get the data source.  Invalidate the source memory region to
           * force reload from RAM.
           */

          src = (const uint8_t *)(rxdesc->addr & EMACRXD_ADDR_MASK);
          up_invalidate_dcache((uintptr_t)src, (uintptr_t)src + copylen);

          /* Copy the data from the driver managed the ring buffer.  If we
           * wanted to support zero copy transfers, we would need to make
           * sure that dev->d_buf and rxdesc->addr refer to the same memory.
           */

          memcpy(dest, src, copylen);
          dest   += copylen;
          pktlen += copylen;

          /* If the end of frame has been received, return the data */

          if ((rxdesc->status & EMACRXD_STA_EOF) != 0)
            {
              /* Frame size from the EMAC */

              dev->d_len = (rxdesc->status & EMACRXD_STA_FRLEN_MASK);
              ninfo("packet %d-%" PRId32 " (%d)\n",
                    xfrq->rxndx, rxndx, dev->d_len);

              /* All data have been copied in the application frame buffer,
               * release the RX descriptor(s).  Loop until all descriptors
               * have been released up to the start of the next frame.
               */

              while (xfrq->rxndx != rxndx)
                {
                  /* Give ownership back to the EMAC */

                  rxdesc = &xfrq->rxdesc[xfrq->rxndx];
                  rxdesc->addr &= ~(EMACRXD_ADDR_OWNER);

                  /* Flush the modified RX descriptor to RAM */

                  up_clean_dcache((uintptr_t)rxdesc,
                                  (uintptr_t)rxdesc +
                                  sizeof(struct emac_rxdesc_s));

                  /* Increment the RX index of the descriptor that was just
                   * released.
                   */

                  if (++xfrq->rxndx >= xfrq->nrxbuffers)
                    {
                      xfrq->rxndx = 0;
                    }
                }

              /* Check if the device packet buffer was large enough to accept
               * all of the data.
               */

              ninfo("rxndx: %d d_len: %d\n",
                    xfrq->rxndx, dev->d_len);
              if (pktlen < dev->d_len)
                {
                  nerr("ERROR: Buffer size %d; frame size %" PRId32 "\n",
                       dev->d_len, pktlen);
                  NETDEV_RXERRORS(&priv->dev);
                  return -E2BIG;
                }

              return OK;
            }
        }

      /* We found a descriptor that we own, but we have not encountered the
       * SOF yet... discard this fragment and keep looking starting at the
       * next fragment.
       */

      else
        {
          /* Give ownership back to the EMAC */

          rxdesc->addr &= ~(EMACRXD_ADDR_OWNER);

          /* Flush the modified RX descriptor to RAM */

          up_clean_dcache((uintptr_t)rxdesc,
                          (uintptr_t)rxdesc +
                          sizeof(struct emac_rxdesc_s));

          /* rxndx already points to the next fragment to be examined.
           * Use it to update the candidate Start-of-Frame.
           */

          xfrq->rxndx = rxndx;
        }

      /* Set-up to process the next fragment.  Get the RX descriptor
       * associated with the next fragment.
       */

      rxdesc = &xfrq->rxdesc[rxndx];

      /* Invalidate the RX descriptor to force re-fetching from RAM */

     up_invalidate_dcache((uintptr_t)rxdesc,
                          (uintptr_t)rxdesc + sizeof(struct emac_rxdesc_s));
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
      xfrq->rxndx = rxndx;
    }

  ninfo("Exit rxndx[%d]: %d\n", qid, xfrq->rxndx);
  return -EAGAIN;
}

/****************************************************************************
 * Function: sam_receive
 *
 * Description:
 *   An interrupt was received indicating the availability of one or more
 *   new RX packets in FIFO memory.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *   qid   - The transfer queue on which the packet was received
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static void sam_receive(struct sam_emac_s *priv, int qid)
{
  struct net_driver_s *dev = &priv->dev;

  /* Loop while while sam_recvframe() successfully retrieves valid
   * EMAC frames.
   */

  while (sam_recvframe(priv, qid) == OK)
    {
      sam_dumppacket("Received packet", dev->d_buf, dev->d_len);
      NETDEV_RXPACKETS(&priv->dev);

      /* Check if the packet is a valid size for the network buffer
       * configuration (this should not happen)
       */

      if (dev->d_len > CONFIG_NET_ETH_PKTSIZE)
        {
          nwarn("WARNING: Dropped, Too big: %d\n", dev->d_len);
          NETDEV_RXERRORS(&priv->dev);
          continue;
        }

#ifdef CONFIG_NET_PKT
      /* When packet sockets are enabled, feed the frame into the tap */

     pkt_input(&priv->dev);
#endif

      /* We only accept IP packets of the configured type and ARP packets */

#ifdef CONFIG_NET_IPv4
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

              sam_transmit(priv, qid);
            }
        }
      else
#endif
#ifdef CONFIG_NET_IPv6
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

              sam_transmit(priv, qid);
            }
        }
      else
#endif
#ifdef CONFIG_NET_ARP
      if (BUF->type == HTONS(ETHTYPE_ARP))
        {
          ninfo("ARP frame\n");
          NETDEV_RXARP(&priv->dev);

          /* Handle ARP packet */

          arp_arpin(&priv->dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, d_len field will set to a value > 0.
           */

          if (priv->dev.d_len > 0)
            {
              sam_transmit(priv, qid);
            }
        }
      else
#endif
        {
          nwarn("WARNING: Dropped, Unknown type: %04x\n", BUF->type);
          NETDEV_RXDROPPED(&priv->dev);
        }
    }
}

/****************************************************************************
 * Function: sam_txdone
 *
 * Description:
 *   An interrupt was received indicating that one or more frames have
 *   completed transmission.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *   qid   - The transfer queue on which the packet was sent
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by the watchdog logic.
 *
 ****************************************************************************/

static void sam_txdone(struct sam_emac_s *priv, int qid)
{
  struct emac_txdesc_s *txdesc;
  struct sam_queue_s *xfrq;
  uint16_t tail;

  /* Are there any outstanding transmissions?  Loop until either (1) all of
   * the TX descriptors have been examined, or (2) until we encounter the
   * first descriptor that is still in use by the hardware.
   */

  xfrq = &priv->xfrq[qid];
  tail = xfrq->txtail;

  while (tail != xfrq->txhead)
    {
      /* Yes.. check the next buffer at the tail of the list */

      txdesc = &xfrq->txdesc[tail];
      up_invalidate_dcache((uintptr_t)txdesc,
                           (uintptr_t)txdesc + sizeof(struct emac_txdesc_s));

      /* Break out of the loop if frame has not yet been sent.  On TX
       * completion, the GMAC sets the USED bit only into the very first
       * buffer descriptor of the sent frame.  Otherwise it updates this
       * descriptor with status error bits.
       */

      if ((txdesc->status & EMACTXD_STA_USED) == 0)
        {
          /* The descriptor is still in use.  Break out of the loop now. */

          break;
        }

      NETDEV_TXDONE(&priv->dev);

      /* Process all buffers of the current transmitted frame */

      while (tail != xfrq->txhead &&
             (txdesc->status & EMACTXD_STA_LAST) == 0)
        {
          /* Increment the tail index */

          if (++tail >= xfrq->ntxbuffers)
            {
              tail = 0;
            }

          /* Get the next TX descriptor */

          txdesc = &xfrq->txdesc[tail];
          up_invalidate_dcache((uintptr_t)txdesc,
            (uintptr_t)txdesc + sizeof(struct emac_txdesc_s));
        }

      /* Go to first buffer of the next frame */

      if (tail != xfrq->txhead &&
          ++tail >= xfrq->ntxbuffers)
        {
          tail = 0;
        }

      /* At least one TX descriptor is available.  Re-enable RX interrupts.
       * RX interrupts may previously have been disabled when we ran out of
       * TX descriptors (see comments in sam_transmit()).
       */

      sam_putreg(priv, SAM_EMAC_IER_OFFSET, EMAC_RX_INTS);
    }

  /* Save the new tail index */

  xfrq->txtail = tail;

  /* Then poll the network for new XMIT data */

  sam_dopoll(priv, qid);
}

/****************************************************************************
 * Function: sam_txerr_interrupt
 *
 * Description:
 *   TX error interrupt processing.
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *   quid - Index of the transfer queue that generated the interrupt
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Ethernet interrupts are disabled
 *
 ****************************************************************************/

static void sam_txerr_interrupt(struct sam_emac_s *priv, int qid)
{
  struct emac_txdesc_s *txdesc;
  struct sam_queue_s *xfrq;
  uint32_t regval;
  uint16_t tail;

  NETDEV_TXERRORS(&priv->dev);

  /* Clear TXEN bit into the Network Configuration Register.  This is a
   * workaround to recover from TX lockups that occurred on the sama5d3 gmac
   * (r1p24f2) when using  scatter-gather.   This issue has never been
   * seen on sama5d4 gmac (r1p31).
   */

  regval  = sam_getreg(priv, SAM_EMAC_NCR_OFFSET);
  regval &= ~EMAC_NCR_TXEN;
  sam_putreg(priv, SAM_EMAC_NCR_OFFSET, regval);

  /* The following step should be optional since this function is called
   * directly by the IRQ handler. Indeed, according to Cadence
   * documentation, the transmission is halted on errors such as
   * too many retries or transmit under run.  However it would becom
   * mandatory if the call of this function were scheduled as a task by
   * the IRQ handler (this is how Linux driver works). Then this function
   * might compete with GMACD_Send().
   *
   * Setting bit 10, tx_halt, of the Network Control Register is not enough:
   * We should wait for bit 3, tx_go, of the Transmit Status Register to
   * be cleared at transmit completion if a frame is being transmitted.
   */

  regval |= EMAC_NCR_THALT;
  sam_putreg(priv, SAM_EMAC_NCR_OFFSET, regval);

  while ((sam_getreg(priv, SAM_EMAC_TSR_OFFSET) & EMAC_TSR_TXGO) != 0);

  /* Treat frames in TX queue including the ones that caused the error. */

  xfrq = &priv->xfrq[qid];
  tail = xfrq->txtail;

  while (tail != xfrq->txhead)
    {
      txdesc = &xfrq->txdesc[tail];

      /* Make H/W updates to the TX descriptor visible to the CPU. */

      up_invalidate_dcache((uintptr_t)txdesc,
                           (uintptr_t)txdesc + sizeof(struct emac_txdesc_s));

      /* Go to the last buffer descriptor of the frame */

      while (tail != xfrq->txhead &&
             (txdesc->status & EMACTXD_STA_LAST) == 0)
        {
          /* Increment the tail index */

          if (++tail >= xfrq->ntxbuffers)
            {
              tail = 0;
            }

          /* Get the next TX descriptor */

          txdesc = &xfrq->txdesc[tail];
          up_invalidate_dcache((uintptr_t)txdesc,
            (uintptr_t)txdesc + sizeof(struct emac_txdesc_s));
        }

      /* Go to first buffer of the next frame */

      if (tail != xfrq->txhead &&
          ++tail >= xfrq->ntxbuffers)
        {
          tail = 0;
        }
    }

  /* Save the new tail index */

  xfrq->txtail = tail;

  /* Reset TX queue */

  sam_txreset(priv, qid);

  /* Clear status */

  regval = sam_getreg(priv, SAM_EMAC_TSR_OFFSET);
  sam_putreg(priv, SAM_EMAC_TSR_OFFSET, regval);

  /* Now we are ready to start transmission again */

  regval  = sam_getreg(priv, SAM_EMAC_NCR_OFFSET);
  regval &= ~EMAC_NCR_TXEN;
  sam_putreg(priv, SAM_EMAC_NCR_OFFSET, regval);

  /* At least one TX descriptor is available.  Re-enable RX interrupts.
   * RX interrupts may previously have been disabled when we ran out of
   * TX descriptors (see comments in sam_transmit()).
   */

  sam_putreg(priv, SAM_EMAC_IER_OFFSET, EMAC_RX_INTS);

  /* Then poll the network for new XMIT data */

  sam_dopoll(priv, qid);
}

/****************************************************************************
 * Function: sam_interrupt_work
 *
 * Description:
 *   Perform interrupt related work from the worker thread.
 *
 * Input Parameters:
 *   arg - The argument passed when work_queue() was called.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Ethernet interrupts are disabled
 *
 ****************************************************************************/

static void sam_interrupt_work(void *arg)
{
  struct sam_emac_s *priv = (struct sam_emac_s *)arg;
  uint32_t isr;
  uint32_t rsr;
  uint32_t tsr;
  uint32_t imr;
  uint32_t regval;
  uint32_t pending;
  uint32_t clrbits;
  int qid = EMAC_QUEUE_0;  /* REVISIT: Currently services on EMAC_QUEUE_0 */

  /* Process pending Ethernet interrupts */

  net_lock();

  /* Read the interrupt status, RX status, and TX status registers.
   * NOTE that the interrupt status register is cleared by this read.
   */

  isr = sam_getreg(priv, SAM_EMAC_ISR_OFFSET);
  rsr = sam_getreg(priv, SAM_EMAC_RSR_OFFSET);
  tsr = sam_getreg(priv, SAM_EMAC_TSR_OFFSET);

  /* Make the pending interrupt status (probably not necessary) */

  imr = sam_getreg(priv, SAM_EMAC_IMR_OFFSET);
  pending = isr & ~(imr | EMAC_INT_UNUSED);

  ninfo("isr: %08" PRIx32 " pending: %08" PRIx32 "\n", isr, pending);

  /* Check for the receipt of an RX packet.
   *
   * RXCOMP indicates that a packet has been received and stored in memory.
   *   The RXCOMP bit is cleared when the interrupt status register was read.
   * RSR:REC indicates that one or more frames have been received and placed
   *   in memory. This indication is cleared by writing a one to this bit.
   */

  if ((pending & EMAC_RX_INTS) != 0 || (rsr & EMAC_RSR_REC) != 0)
    {
      clrbits = EMAC_RSR_REC;

      /* Check for Receive Overrun.
       *
       * RSR:RXOVR will be set if the RX FIFO is not able to store the
       *   receive frame due to a FIFO overflow, or if the receive status
       *   was not taken at the end of the frame. This bit is also set in
       *   DMA packet buffer mode if the packet buffer overflows. For DMA
       *   operation, the buffer will be recovered if an overrun occurs. This
       *   bit is cleared when set to 1.
       */

      if ((rsr & EMAC_RSR_RXOVR) != 0)
        {
          nerr("ERROR: Receiver overrun RSR: %08" PRIx32 "\n", rsr);
          clrbits |= EMAC_RSR_RXOVR;
        }

      /* Check for buffer not available (BNA)
       *
       * RSR:BNA means that an attempt was made to get a new buffer and the
       *   pointer indicated that it was owned by the processor. The DMA will
       *   reread the pointer each time an end of frame is received until a
       *   valid pointer is found. This bit is set following each descriptor
       *   read attempt that fails, even if consecutive pointers are
       *   unsuccessful and software has in the mean time cleared the status
       *   flag. Cleared by writing a one to this bit.
       */

      if ((rsr & EMAC_RSR_BNA) != 0)
        {
          nerr("ERROR: Buffer not available RSR: %08" PRIx32 "\n", rsr);
          clrbits |= EMAC_RSR_BNA;
        }

      /* Clear status */

      sam_putreg(priv, SAM_EMAC_RSR_OFFSET, clrbits);

      /* Handle the received packet */

      sam_receive(priv, qid);
    }

  /* Check for TX errors */

  if ((pending & EMAC_TXERR_INTS) != 0)
    {
      sam_txerr_interrupt(priv, qid);
    }

  /* Check for the completion of a transmission.  This should be done before
   * checking for received data (because receiving can cause another
   * transmission before we had a chance to handle the last one).
   *
   * ISR:TCOMP is set when a frame has been transmitted. Cleared on read.
   * TSR:TXCOMP is set when a frame has been transmitted. Cleared by writing
   *   a one to this bit.
   */

  if ((pending & EMAC_INT_TCOMP) != 0 || (tsr & EMAC_TSR_TXCOMP) != 0)
    {
      /* A frame has been transmitted */

      /* Check for Retry Limit Exceeded (RLE) */

      if ((tsr & EMAC_TSR_RLE) != 0)
        {
          /* Status RLE & Number of discarded buffers */

          clrbits = EMAC_TSR_RLE | sam_txinuse(priv, qid);
          sam_txreset(priv, qid);

          nerr("ERROR: Retry Limit Exceeded TSR: %08" PRIx32 "\n", tsr);

          regval  = sam_getreg(priv, SAM_EMAC_NCR_OFFSET);
          regval |= EMAC_NCR_TXEN;
          sam_putreg(priv, SAM_EMAC_NCR_OFFSET, regval);
        }

      /* Check Collision Occurred (COL) */

      if ((tsr & EMAC_TSR_COL) != 0)
        {
          nerr("ERROR: Collision occurred TSR: %08" PRIx32 "\n", tsr);
          NETDEV_TXERRORS(&priv->dev);
        }

      /* Check Transmit Frame Corruption due to AHB error (TFC) */

      if ((tsr & EMAC_TSR_TFC) != 0)
        {
          nerr("ERROR: Transmit Frame Corruption "
               "due to AHB error: %08" PRIx32 "\n",
               tsr);
          NETDEV_TXERRORS(&priv->dev);
        }

      /* Clear status */

      sam_putreg(priv, SAM_EMAC_TSR_OFFSET, tsr);

      /* And handle the TX done event */

      sam_txdone(priv, qid);
    }

#ifdef CONFIG_DEBUG_NET
  /* Check for response not OK */

  if ((pending & EMAC_INT_HRESP) != 0)
    {
      nerr("ERROR: Hresp not OK\n");
    }

  /* Check for PAUSE Frame received (PFRE).
   *
   * ISR:PFRE indicates that a pause frame has been received with non-zero
   * pause quantum.  Cleared on a read.
   */

  if ((pending & EMAC_INT_PFNZ) != 0)
    {
      ninfo("Pause frame received\n");
    }

  /* Check for Pause Time Zero (PTZ)
   *
   * ISR:PTZ is set Pause Time Zero
   */

  if ((pending & EMAC_INT_PTZ) != 0)
    {
      ninfo("Pause TO!\n");
    }
#endif

  net_unlock();

  /* Re-enable Ethernet interrupts */

  up_enable_irq(priv->attr->irq);
}

/****************************************************************************
 * Function: sam_emac_interrupt
 *
 * Description:
 *   Common hardware interrupt handler.
 *
 * Input Parameters:
 *   priv - Reference to the EMAC private state structure
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *
 ****************************************************************************/

static int sam_emac_interrupt(int irq, void *context, void *arg)
{
  struct sam_emac_s *priv = (struct sam_emac_s *)arg;
  uint32_t tsr;

  DEBUGASSERT(priv != NULL);

  /* Disable further Ethernet interrupts.  Because Ethernet interrupts are
   * also disabled if the TX timeout event occurs, there can be no race
   * condition here.
   */

  up_disable_irq(priv->attr->irq);

  /* Check for the completion of a transmission.  Careful:
   *
   * ISR:TCOMP is set when a frame has been transmitted. Cleared on read (so
   *   we cannot read it here).
   * TSR:TXCOMP is set when a frame has been transmitted. Cleared by writing
   *   a one to this bit.
   */

  tsr = sam_getreg(priv, SAM_EMAC_TSR_OFFSET);
  if ((tsr & EMAC_TSR_TXCOMP) != 0)
    {
      /* If a TX transfer just completed, then cancel the TX timeout so
       * there will be do race condition between any subsequent timeout
       * expiration and the deferred interrupt processing.
       */

      wd_cancel(&priv->txtimeout);
    }

  /* Schedule to perform the interrupt processing on the worker thread. */

  work_queue(ETHWORK, &priv->irqwork, sam_interrupt_work, priv, 0);
  return OK;
}

/****************************************************************************
 * Function: sam_txtimeout_work
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

static void sam_txtimeout_work(void *arg)
{
  struct sam_emac_s *priv = (struct sam_emac_s *)arg;

  nerr("ERROR: Timeout!\n");

  net_lock();
  NETDEV_TXTIMEOUTS(&priv->dev);

  /* Reset the hardware.  Just take the interface down, then back up again. */

  sam_ifdown(&priv->dev);
  sam_ifup(&priv->dev);

  /* Then poll the network for new XMIT data */

  sam_dopoll(priv, EMAC_QUEUE_0);
  net_unlock();
}

/****************************************************************************
 * Function: sam_txtimeout_expiry
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

static void sam_txtimeout_expiry(wdparm_t arg)
{
  struct sam_emac_s *priv = (struct sam_emac_s *)arg;

  /* Disable further Ethernet interrupts.  This will prevent some race
   * conditions with interrupt work.  There is still a potential race
   * condition with interrupt work that is already queued and in progress.
   */

  up_disable_irq(priv->attr->irq);

  /* Schedule to perform the TX timeout processing on the worker thread. */

  work_queue(ETHWORK, &priv->irqwork, sam_txtimeout_work, priv, 0);
}

/****************************************************************************
 * Function: sam_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the EMAC interface when an IP address is
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

static int sam_ifup(struct net_driver_s *dev)
{
  struct sam_emac_s *priv = (struct sam_emac_s *)dev->d_private;
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

  /* Configure the EMAC interface for normal operation. */

  ninfo("Initialize the EMAC\n");
  sam_emac_configure(priv);
  sam_queue_configure(priv, EMAC_QUEUE_1);
  sam_queue_configure(priv, EMAC_QUEUE_2);

  if (g_emac_nqueues > 3)
    {
      sam_queue_configure(priv, EMAC_QUEUE_3);
      sam_queue_configure(priv, EMAC_QUEUE_4);
      sam_queue_configure(priv, EMAC_QUEUE_5);
    }

  sam_queue0_configure(priv);

  /* Set the MAC address (should have been configured while we were down) */

  sam_macaddress(priv);

#ifdef CONFIG_NET_ICMPv6
  /* Set up IPv6 multicast address filtering */

  sam_ipv6multicast(priv);
#endif

  /* Initialize for PHY access */

  ret = sam_phyinit(priv);
  if (ret < 0)
    {
      nerr("ERROR: sam_phyinit failed: %d\n", ret);
      return ret;
    }

  /* Auto Negotiate, working in RMII mode */

  ret = sam_autonegotiate(priv);
  if (ret < 0)
    {
      nerr("ERROR: sam_autonegotiate failed: %d\n", ret);
      return ret;
    }

  while (sam_linkup(priv) == 0);
  ninfo("Link detected\n");

  /* Enable normal MAC operation */

  ninfo("Enable normal operation\n");

  /* Enable the EMAC interrupt */

  priv->ifup = true;
  up_enable_irq(priv->attr->irq);
  return OK;
}

/****************************************************************************
 * Function: sam_ifdown
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

static int sam_ifdown(struct net_driver_s *dev)
{
  struct sam_emac_s *priv = (struct sam_emac_s *)dev->d_private;
  irqstate_t flags;

  ninfo("Taking the network down\n");

  /* Disable the EMAC interrupt */

  flags = enter_critical_section();
  up_disable_irq(priv->attr->irq);

  /* Cancel the TX timeout timers */

  wd_cancel(&priv->txtimeout);

  /* Put the EMAC in its reset, non-operational state.  This should be
   * a known configuration that will guarantee the sam_ifup() always
   * successfully brings the interface back up.
   */

  sam_emac_reset(priv);

  /* Mark the device "down" */

  priv->ifup = false;
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Function: sam_txavail_work
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

static void sam_txavail_work(void *arg)
{
  struct sam_emac_s *priv = (struct sam_emac_s *)arg;

  ninfo("ifup: %d\n", priv->ifup);

  /* Ignore the notification if the interface is not yet up */

  net_lock();
  if (priv->ifup)
    {
      /* Poll the network for new XMIT data */

      sam_dopoll(priv, EMAC_QUEUE_0);
    }

  net_unlock();
}

/****************************************************************************
 * Function: sam_txavail
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

static int sam_txavail(struct net_driver_s *dev)
{
  struct sam_emac_s *priv = (struct sam_emac_s *)dev->d_private;

  /* Is our single work structure available?  It may not be if there are
   * pending interrupt actions and we will have to ignore the Tx
   * availability action.
   */

  if (work_available(&priv->pollwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      work_queue(ETHWORK, &priv->pollwork, sam_txavail_work, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: sam_hashindx
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
static unsigned int sam_hashindx(const uint8_t *mac)
{
  unsigned int ndx;

  /* Isolate: mac[0]
   *           ... 05 04 03 02 01 00]
   */

  ndx = mac[0];

  /* Isolate: mac[1]           mac[0]
   *          ...11 10 09 08] [07 06 ...
   *
   * Accumulate: 05 04 03 02 01 00
   *        XOR: 11 10 09 08 07 06
   */

  ndx ^= (mac[1] << 2) | (mac[0] >> 6);

  /* Isolate: mac[2]      mac[1]
   *          ... 17 16] [15 14 13 12 ...
   *
   * Accumulate: 05 04 03 02 01 00
   *        XOR: 11 10 09 08 07 06
   *        XOR: 17 16 15 14 13 12
   */

  ndx ^= (mac[2] << 4) | (mac[1] >> 4);

  /* Isolate:  mac[2]
   *          [23 22 21 20 19 18 ...
   *
   * Accumulate: 05 04 03 02 01 00
   *        XOR: 11 10 09 08 07 06
   *        XOR: 17 16 15 14 13 12
   *        XOR: 23 22 21 20 19 18
   */

  ndx ^= (mac[2] >> 2);

  /* Isolate:     mac[3]
   *          ... 29 28 27 26 25 24]
   *
   * Accumulate: 05 04 03 02 01 00
   *        XOR: 11 10 09 08 07 06
   *        XOR: 17 16 15 14 13 12
   *        XOR: 23 22 21 20 19 18
   *        XOR: 29 28 27 26 25 24
   */

  ndx ^= mac[3];

  /* Isolate:     mac[4]        mac[3]
   *          ... 35 34 33 32] [31 30 ...
   *
   * Accumulate: 05 04 03 02 01 00
   *        XOR: 11 10 09 08 07 06
   *        XOR: 17 16 15 14 13 12
   *        XOR: 23 22 21 20 19 18
   *        XOR: 29 28 27 26 25 24
   *        XOR: 35 34 33 32 31 30
   */

  ndx ^= (mac[4] << 2) | (mac[3] >> 6);

  /* Isolate:     mac[5]  mac[4]
   *          ... 41 40] [39 38 37 36 ...
   *
   * Accumulate: 05 04 03 02 01 00
   *        XOR: 11 10 09 08 07 06
   *        XOR: 17 16 15 14 13 12
   *        XOR: 23 22 21 20 19 18
   *        XOR: 29 28 27 26 25 24
   *        XOR: 35 34 33 32 31 30
   *        XOR: 41 40 39 38 37 36
   */

  ndx ^= (mac[5] << 4) | (mac[4] >> 4);

  /* Isolate:  mac[5]
   *          [47 46 45 44 43 42 ...
   *
   * Accumulate: 05 04 03 02 01 00
   *        XOR: 11 10 09 08 07 06
   *        XOR: 17 16 15 14 13 12
   *        XOR: 23 22 21 20 19 18
   *        XOR: 29 28 27 26 25 24
   *        XOR: 35 34 33 32 31 30
   *        XOR: 41 40 39 38 37 36
   *        XOR: 47 46 45 44 43 42
   */

  ndx ^= (mac[5] >> 2);

  /* Mask out the garbage bits and return the 6-bit index */

  return ndx & 0x3f;
}
#endif /* CONFIG_NET_MCASTGROUP || CONFIG_NET_ICMPv6 */

/****************************************************************************
 * Function: sam_addmac
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
static int sam_addmac(struct net_driver_s *dev, const uint8_t *mac)
{
  struct sam_emac_s *priv = (struct sam_emac_s *)dev->d_private;
  uint32_t regval;
  unsigned int regoffset;
  unsigned int ndx;
  unsigned int bit;

  ninfo("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  /* Calculate the 6-bit has table index */

  ndx = sam_hashindx(mac);

  /* Add the multicast address to the hardware multicast hash table */

  if (ndx >= 32)
    {
      regoffset = SAM_EMAC_HRT_OFFSET;  /* Hash Register Top [63:32] Register */
      bit       = 1 << (ndx - 32);      /* Bit 0-31 */
    }
  else
    {
      regoffset = SAM_EMAC_HRB_OFFSET;  /* Hash Register Bottom [31:0] Register */
      bit       = 1 << ndx;             /* Bit 0-31 */
    }

  regval = sam_getreg(priv, regoffset);
  regval |= bit;
  sam_putreg(priv, regoffset, regval);

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

  regval  = sam_getreg(priv, SAM_EMAC_NCFGR_OFFSET);
  regval &= ~EMAC_NCFGR_UNIHEN;  /* Disable unicast matching */
  regval |= EMAC_NCFGR_MTIHEN;   /* Enable multicast matching */
  sam_putreg(priv, SAM_EMAC_NCFGR_OFFSET, regval);

  return OK;
}
#endif

/****************************************************************************
 * Function: sam_rmmac
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
static int sam_rmmac(struct net_driver_s *dev, const uint8_t *mac)
{
  struct sam_emac_s *priv = (struct sam_emac_s *)dev->d_private;
  uint32_t regval;
  unsigned int regoffset1;
  unsigned int regoffset2;
  unsigned int ndx;
  unsigned int bit;

  ninfo("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  /* Calculate the 6-bit has table index */

  ndx = sam_hashindx(mac);

  /* Remove the multicast address to the hardware multicast hast table */

  if (ndx >= 32)
    {
      regoffset1 = SAM_EMAC_HRT_OFFSET;  /* Hash Register Top [63:32] Register */
      regoffset2 = SAM_EMAC_HRB_OFFSET;  /* Hash Register Bottom [31:0] Register */
      bit        = 1 << (ndx - 32);      /* Bit 0-31 */
    }
  else
    {
      regoffset1 = SAM_EMAC_HRB_OFFSET;  /* Hash Register Bottom [31:0] Register */
      regoffset2 = SAM_EMAC_HRT_OFFSET;  /* Hash Register Top [63:32] Register */
      bit        = 1 << ndx;             /* Bit 0-31 */
    }

  regval  = sam_getreg(priv, regoffset1);
  regval &= ~bit;
  sam_putreg(priv, regoffset1, regval);

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

  if (regval == 0 && sam_getreg(priv, regoffset2) == 0)
    {
      /* Yes.. disable all address matching */

      regval  = sam_getreg(priv, SAM_EMAC_NCFGR_OFFSET);
      regval &= ~(EMAC_NCFGR_UNIHEN | EMAC_NCFGR_MTIHEN);
      sam_putreg(priv, SAM_EMAC_NCFGR_OFFSET, regval);
    }

  return OK;
}
#endif

/****************************************************************************
 * Function: sam_ioctl
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
 *    that is specified using the req->reg_no struct field and use req->
 *    val_in as its input.
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
static int sam_ioctl(struct net_driver_s *dev, int cmd, unsigned long arg)
{
#ifdef CONFIG_NETDEV_PHY_IOCTL
  struct sam_emac_s *priv = (struct sam_emac_s *)dev->d_private;
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

              ret = sam_phyintenable(priv);
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
          uint32_t regval;

          /* Enable management port */

          regval = sam_getreg(priv, SAM_EMAC_NCR_OFFSET);
          sam_putreg(priv, SAM_EMAC_NCR_OFFSET, regval | EMAC_NCR_MPE);

          /* Read from the requested register */

          ret = sam_phyread(priv, req->phy_id, req->reg_num, &req->val_out);

          /* Disable management port (probably) */

          sam_putreg(priv, SAM_EMAC_NCR_OFFSET, regval);
        }
        break;

      case SIOCSMIIREG: /* Set register in MII PHY */
        {
          struct mii_ioctl_data_s *req =
        (struct mii_ioctl_data_s *)((uintptr_t)arg);
          uint32_t regval;

          /* Enable management port */

          regval = sam_getreg(priv, SAM_EMAC_NCR_OFFSET);
          sam_putreg(priv, SAM_EMAC_NCR_OFFSET, regval | EMAC_NCR_MPE);

          /* Write to the requested register */

          ret = sam_phywrite(priv, req->phy_id, req->reg_num, req->val_in);

          /* Disable management port (probably) */

          sam_putreg(priv, SAM_EMAC_NCR_OFFSET, regval);
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
 * Function: sam_phydump
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
static void sam_phydump(struct sam_emac_s *priv)
{
  uint32_t regval;
  uint16_t phyval;

  /* Enable management port */

  regval  = sam_getreg(priv, SAM_EMAC_NCR_OFFSET);
  regval |= EMAC_NCR_MPE;
  sam_putreg(priv, SAM_EMAC_NCR_OFFSET, regval);

  ninfo("%s Registers (Address %02x)\n",
        priv->attr->rmii ? "RMII" : "MII", priv->phyaddr);

  sam_phyread(priv, priv->phyaddr, MII_MCR, &phyval);
  ninfo("  MCR:       %04x\n", phyval);
  sam_phyread(priv, priv->phyaddr, MII_MSR, &phyval);
  ninfo("  MSR:       %04x\n", phyval);
  sam_phyread(priv, priv->phyaddr, MII_ADVERTISE, &phyval);
  ninfo("  ADVERTISE: %04x\n", phyval);
  sam_phyread(priv, priv->phyaddr, MII_LPA, &phyval);
  ninfo("  LPR:       %04x\n", phyval);
  sam_phyread(priv, priv->phyaddr, priv->attr->physr, &phyval);
  ninfo("  PHYSR:     %04x\n", phyval);

  /* Disable management port */

  regval  = sam_getreg(priv, SAM_EMAC_NCR_OFFSET);
  regval &= ~EMAC_NCR_MPE;
  sam_putreg(priv, SAM_EMAC_NCR_OFFSET, regval);
}
#endif

/****************************************************************************
 * Function: sam_is*
 *
 * Description:
 *   Helpers to simplify decoding PHY status register bits
 *
 * Input Parameters:
 *   physr - The value of the PHY status register
 *
 * Returned Value:
 *   True: The PHY configuration is selected; False; some other PHY
 *   configuration is selected.
 *
 ****************************************************************************/

#if 0 /* Not used */
static bool sam_is10hdx(struct sam_emac_s *priv, uint16_t physr)
{
  uint16_t mask;
  uint16_t match;

  if (priv->attr->sralt)
    {
      mask  = priv->attr->u.alt.altmask;
      match = priv->attr->u.alt.hdx10;
    }
  else
    {
      mask  = priv->attr->u.std.stdmask;
      match = 0;
    }

  return (physr & mask) == match;
}
#endif

static bool sam_is100hdx(struct sam_emac_s *priv, uint16_t physr)
{
  uint16_t mask;
  uint16_t match;

  if (priv->attr->sralt)
    {
      mask  = priv->attr->u.alt.altmask;
      match = priv->attr->u.alt.hdx100;
    }
  else
    {
      mask  = priv->attr->u.std.stdmask;
      match = priv->attr->u.std.speed100;
    }

  return (physr & mask) == match;
}

static bool sam_is10fdx(struct sam_emac_s *priv, uint16_t physr)
{
  uint16_t mask;
  uint16_t match;

  if (priv->attr->sralt)
    {
      mask  = priv->attr->u.alt.altmask;
      match = priv->attr->u.alt.fdx10;
    }
  else
    {
      mask  = priv->attr->u.std.stdmask;
      match = priv->attr->u.std.fduplex;
    }

  return (physr & mask) == match;
}

static bool sam_is100fdx(struct sam_emac_s *priv, uint16_t physr)
{
  uint16_t mask;
  uint16_t match;

  if (priv->attr->sralt)
    {
      mask  = priv->attr->u.alt.altmask;
      match = priv->attr->u.alt.fdx100;
    }
  else
    {
      mask  = priv->attr->u.std.stdmask;
      match = (priv->attr->u.std.fduplex | priv->attr->u.std.speed100);
    }

  return (physr & mask) == match;
}

/****************************************************************************
 * Function: sam_phyintenable
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
static int sam_phyintenable(struct sam_emac_s *priv)
{
#if defined(SAMV7_EMAC0_PHY_KSZ8051) || defined(SAMV7_EMAC0_PHY_KSZ8061) || \
    defined(SAMV7_EMAC0_PHY_KSZ8081) || \
    defined(SAMV7_EMAC1_PHY_KSZ8051) || defined(SAMV7_EMAC1_PHY_KSZ8061) || \
    defined(SAMV7_EMAC1_PHY_KSZ8081)
  uint32_t regval;
  uint16_t phyval;
  int ret;

  /* Does this MAC support a KSZ80X1 PHY? */

  if (priv->phytype == SAMV7_PHY_KSZ8051 ||
      priv->phytype == SAMV7_PHY_KSZ8061 ||
      priv->phytype == SAMV7_PHY_KSZ8081)
    {
      /* Enable management port */

      regval = sam_getreg(priv, SAM_EMAC_NCR_OFFSET);
      sam_putreg(priv, SAM_EMAC_NCR_OFFSET, regval | EMAC_NCR_MPE);

      /* Read the interrupt status register in order to clear any pending
       * interrupts
       */

      ret = sam_phyread(priv, priv->phyaddr, MII_KSZ8081_INT, &phyval);
      if (ret == OK)
        {
          /* Enable link up/down interrupts */

          ret = sam_phywrite(priv, priv->phyaddr, MII_KSZ8081_INT,
                            (MII_KSZ80X1_INT_LDEN | MII_KSZ80X1_INT_LUEN));
        }

      /* Disable management port (probably) */

      sam_putreg(priv, SAM_EMAC_NCR_OFFSET, regval);
    }
  else
#endif
    {
      nerr("ERROR: Unsupported PHY type: %d\n", priv->phytype);
      ret = -ENOSYS;
    }

  return ret;
}
#endif

/****************************************************************************
 * Function: sam_phywait
 *
 * Description:
 *   Wait for the PHY to become IDLE
 *
 * Input Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno (-ETIMEDOUT) on failure.
 *
 ****************************************************************************/

static int sam_phywait(struct sam_emac_s *priv)
{
  volatile unsigned int retries;

  /* Loop for the configured number of attempts */

  for (retries = 0; retries < PHY_RETRY_MAX; retries++)
    {
      /* Is the PHY IDLE */

      if ((sam_getreg(priv, SAM_EMAC_NSR_OFFSET) & EMAC_NSR_IDLE) != 0)
        {
          return OK;
        }
    }

  return -ETIMEDOUT;
}

/****************************************************************************
 * Function: sam_phyreset
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

static int sam_phyreset(struct sam_emac_s *priv)
{
  uint32_t regval;
  uint16_t mcr;
  int timeout;
  int ret;

  ninfo(" sam_phyreset\n");

  /* Enable management port */

  regval  = sam_getreg(priv, SAM_EMAC_NCR_OFFSET);
  regval |= EMAC_NCR_MPE;
  sam_putreg(priv, SAM_EMAC_NCR_OFFSET, regval);

  /* Reset the PHY */

  ret = sam_phywrite(priv, priv->phyaddr, MII_MCR, MII_MCR_RESET);
  if (ret < 0)
    {
      nerr("ERROR: sam_phywrite failed: %d\n", ret);
    }

  /* Wait for the PHY reset to complete */

  ret = -ETIMEDOUT;
  for (timeout = 0; timeout < 10; timeout++)
    {
      mcr = MII_MCR_RESET;
      int result = sam_phyread(priv, priv->phyaddr, MII_MCR, &mcr);
      if (result < 0)
        {
          nerr("ERROR: Failed to read the MCR register: %d\n", ret);
          ret = result;
        }
      else if ((mcr & MII_MCR_RESET) == 0)
        {
          ret = OK;
          break;
        }
    }

  /* Disable management port */

  regval  = sam_getreg(priv, SAM_EMAC_NCR_OFFSET);
  regval &= ~EMAC_NCR_MPE;
  sam_putreg(priv, SAM_EMAC_NCR_OFFSET, regval);
  return ret;
}

/****************************************************************************
 * Function: sam_phyfind
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

static int sam_phyfind(struct sam_emac_s *priv, uint8_t *phyaddr)
{
  uint32_t regval;
  uint16_t phyval;
  uint8_t candidate;
  unsigned int offset;
  int ret = -ESRCH;

  ninfo("Find a valid PHY address\n");

  /* Enable management port */

  regval  = sam_getreg(priv, SAM_EMAC_NCR_OFFSET);
  regval |= EMAC_NCR_MPE;
  sam_putreg(priv, SAM_EMAC_NCR_OFFSET, regval);

  candidate = *phyaddr;

  /* Check current candidate address */

  ret = sam_phyread(priv, candidate, MII_PHYID1, &phyval);
  if (ret == OK && phyval == priv->attr->msoui)
    {
      *phyaddr = candidate;
      ret = OK;
    }

  /* The current address does not work... try another */

  else
    {
      nerr("ERROR: sam_phyread failed for PHY address %02x: %d\n",
           candidate, ret);

      for (offset = 0; offset < 32; offset++)
        {
          /* Get the next candidate PHY address */

          candidate = (candidate + 1) & 0x1f;

          /* Try reading the PHY ID from the candidate PHY address */

          ret = sam_phyread(priv, candidate, MII_PHYID1, &phyval);
          if (ret == OK && phyval == priv->attr->msoui)
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
      sam_phyread(priv, candidate, priv->attr->physr, &phyval);
      ninfo("  PHYSR:  %04x PHY addr: %d\n", phyval, candidate);
    }

  /* Disable management port */

  regval  = sam_getreg(priv, SAM_EMAC_NCR_OFFSET);
  regval &= ~EMAC_NCR_MPE;
  sam_putreg(priv, SAM_EMAC_NCR_OFFSET, regval);
  return ret;
}

/****************************************************************************
 * Function: sam_phyread
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

static int sam_phyread(struct sam_emac_s *priv, uint8_t phyaddr,
                       uint8_t regaddr, uint16_t *phyval)
{
  uint32_t regval;
  int ret;

  /* Make sure that the PHY is idle */

  ret = sam_phywait(priv);
  if (ret < 0)
    {
      nerr("ERROR: sam_phywait failed: %d\n", ret);
      return ret;
    }

  /* Write the PHY Maintenance register */

  regval = EMAC_MAN_DATA(0)       | EMAC_MAN_WTN  | EMAC_MAN_REGA(regaddr) |
           EMAC_MAN_PHYA(phyaddr) | EMAC_MAN_READ | EMAC_MAN_WZO;

  if (!priv->attr->clause45)
    {
      /* CLTTO must be set for Clause 22 operation. To read clause 45 PHYs,
       * bit 30 should be written with a 0 rather than a 1.
       */

      regval |= EMAC_MAN_CLTTO;
    }

  sam_putreg(priv, SAM_EMAC_MAN_OFFSET, regval);

  /* Wait until the PHY is again idle */

  ret = sam_phywait(priv);
  if (ret < 0)
    {
      nerr("ERROR: sam_phywait failed: %d\n", ret);
      return ret;
    }

  /* Return data */

  *phyval = sam_getreg(priv, SAM_EMAC_MAN_OFFSET) & EMAC_MAN_DATA_MASK;
  return OK;
}

/****************************************************************************
 * Function: sam_phywrite
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

static int sam_phywrite(struct sam_emac_s *priv, uint8_t phyaddr,
                        uint8_t regaddr, uint16_t phyval)
{
  uint32_t regval;
  int ret;

  /* Make sure that the PHY is idle */

  ret = sam_phywait(priv);
  if (ret < 0)
    {
      nerr("ERROR: sam_phywait failed: %d\n", ret);
      return ret;
    }

  /* Write the PHY Maintenance register */

  regval = EMAC_MAN_DATA(phyval)  | EMAC_MAN_WTN   | EMAC_MAN_REGA(regaddr) |
           EMAC_MAN_PHYA(phyaddr) | EMAC_MAN_WRITE | EMAC_MAN_WZO;

  if (!priv->attr->clause45)
    {
      /* CLTTO must be set for Clause 22 operation. To read clause 45 PHYs,
       * bit 30 should be written with a 0 rather than a 1.
       */

      regval |= EMAC_MAN_CLTTO;
    }

  sam_putreg(priv, SAM_EMAC_MAN_OFFSET, regval);

  /* Wait until the PHY is again IDLE */

  ret = sam_phywait(priv);
  if (ret < 0)
    {
      nerr("ERROR: sam_phywait failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Function: sam_autonegotiate
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

static int sam_autonegotiate(struct sam_emac_s *priv)
{
  uint32_t regval;
  uint32_t ncr;
  uint16_t phyid1;
  uint16_t phyid2;
  uint16_t mcr;
  uint16_t msr;
  uint16_t advertise;
  uint16_t lpa;
  int timeout;
  int ret;

  /* Enable management port */

  regval  = sam_getreg(priv, SAM_EMAC_NCR_OFFSET);
  regval |= EMAC_NCR_MPE;
  sam_putreg(priv, SAM_EMAC_NCR_OFFSET, regval);

  /* Verify that we can read the PHYID register */

  ret = sam_phyread(priv, priv->phyaddr, MII_PHYID1, &phyid1);
  if (ret < 0)
    {
      nerr("ERROR: Failed to read PHYID1\n");
      goto errout;
    }

  ninfo("PHYID1: %04x PHY address: %02x\n", phyid1, priv->phyaddr);

  ret = sam_phyread(priv, priv->phyaddr, MII_PHYID2, &phyid2);
  if (ret < 0)
    {
      nerr("ERROR: Failed to read PHYID2\n");
      goto errout;
    }

  ninfo("PHYID2: %04x PHY address: %02x\n", phyid2, priv->phyaddr);

  if (phyid1 == priv->attr->msoui &&
     ((phyid2 & MII_PHYID2_OUI_MASK) >> MII_PHYID2_OUI_SHIFT) ==
      (uint16_t)priv->attr->lsoui)
    {
      ninfo("  Vendor Model Number:   %04x\n",
           (phyid2 & MII_PHYID2_MODEL_MASK) >> MII_PHYID2_MODEL_SHIFT);
      ninfo("  Model Revision Number: %04x\n",
           (phyid2 & MII_PHYID2_REV_MASK) >> MII_PHYID2_REV_SHIFT);
    }
  else
    {
      nerr("ERROR: PHY not recognized\n");
    }

  /* Setup control register */

  ret = sam_phyread(priv, priv->phyaddr, MII_MCR, &mcr);
  if (ret < 0)
    {
      nerr("ERROR: Failed to read MCR\n");
      goto errout;
    }

  mcr &= ~MII_MCR_ANENABLE;  /* Remove autonegotiation enable */
  mcr &= ~(MII_MCR_LOOPBACK | MII_MCR_PDOWN);
  mcr |= MII_MCR_ISOLATE;    /* Electrically isolate PHY */

  ret = sam_phywrite(priv, priv->phyaddr, MII_MCR, mcr);
  if (ret < 0)
    {
      nerr("ERROR: Failed to write MCR\n");
      goto errout;
    }

  /* Set the Auto_negotiation Advertisement Register MII advertising for
   * Next page 100BaseTxFD and HD, 10BaseTFD and HD, IEEE 802.3
   */

  advertise = MII_ADVERTISE_100BASETXFULL | MII_ADVERTISE_100BASETXHALF |
              MII_ADVERTISE_10BASETXFULL | MII_ADVERTISE_10BASETXHALF |
              MII_ADVERTISE_8023;

  ret = sam_phywrite(priv, priv->phyaddr, MII_ADVERTISE, advertise);
  if (ret < 0)
    {
      nerr("ERROR: Failed to write ANAR\n");
      goto errout;
    }

  /* Read and modify control register */

  ret = sam_phyread(priv, priv->phyaddr, MII_MCR, &mcr);
  if (ret < 0)
    {
      nerr("ERROR: Failed to read MCR\n");
      goto errout;
    }

  mcr |= (MII_MCR_SPEED100 | MII_MCR_ANENABLE | MII_MCR_FULLDPLX);
  ret = sam_phywrite(priv, priv->phyaddr, MII_MCR, mcr);
  if (ret < 0)
    {
      nerr("ERROR: Failed to write MCR\n");
      goto errout;
    }

  /* Restart Auto_negotiation */

  mcr |=  MII_MCR_ANRESTART;
  mcr &= ~MII_MCR_ISOLATE;

  ret = sam_phywrite(priv, priv->phyaddr, MII_MCR, mcr);
  if (ret < 0)
    {
      nerr("ERROR: Failed to write MCR\n");
      goto errout;
    }

  ninfo("  MCR: %04x\n", mcr);

  /* Check AutoNegotiate complete */

  timeout = 0;
  for (; ; )
    {
      ret = sam_phyread(priv, priv->phyaddr, MII_MSR, &msr);
      if (ret < 0)
        {
          nerr("ERROR: Failed to read MSR\n");
          goto errout;
        }

      /* Completed successfully? */

      if ((msr & MII_MSR_ANEGCOMPLETE) != 0)
        {
          /* Yes.. break out of the loop */

          ninfo("AutoNegotiate complete\n");
          break;
        }

      /* No.. check for a timeout */

      if (++timeout >= PHY_RETRY_MAX)
        {
          nerr("ERROR: TimeOut\n");
          sam_phydump(priv);
          ret = -ETIMEDOUT;
          goto errout;
        }
    }

  /* Get the AutoNeg Link partner base page */

  ret = sam_phyread(priv, priv->phyaddr, MII_LPA, &lpa);
  if (ret < 0)
    {
      nerr("ERROR: Failed to read ANLPAR\n");
      goto errout;
    }

  /* Setup the EMAC link speed */

  regval  = sam_getreg(priv, SAM_EMAC_NCFGR_OFFSET);
  regval &= ~(EMAC_NCFGR_SPD | EMAC_NCFGR_FD);

  if (((advertise & lpa) & MII_ADVERTISE_100BASETXFULL) != 0)
    {
      /* Set MII for 100BaseTX and Full Duplex */

      regval |= (EMAC_NCFGR_SPD | EMAC_NCFGR_FD);
    }
  else if (((advertise & lpa) & MII_ADVERTISE_10BASETXFULL) != 0)
    {
      /* Set MII for 10BaseT and Full Duplex */

      regval |= EMAC_NCFGR_FD;
    }
  else if (((advertise & lpa) & MII_ADVERTISE_100BASETXHALF) != 0)
    {
      /* Set MII for 100BaseTX and half Duplex */

      regval |= EMAC_NCFGR_SPD;
    }
#if 0
  else if (((advertise & lpa) & MII_ADVERTISE_10BASETXHALF) != 0)
    {
      /* set MII for 10BaseT and half Duplex */
    }
#endif

  sam_putreg(priv, SAM_EMAC_NCFGR_OFFSET, regval);

  /* Select RMII/MII */

  ncr = sam_getreg(priv, SAM_EMAC_NCR_OFFSET);
  sam_putreg(priv, SAM_EMAC_NCR_OFFSET,
             ncr & ~(EMAC_NCR_TXEN | EMAC_NCR_RXEN));

  regval = sam_getreg(priv, SAM_EMAC_UR_OFFSET);
  regval &= ~EMAC_UR_RMII;  /* Assume RMII */

  if (!priv->attr->rmii)
    {
      /* Not RMII, select MII */

      regval |= EMAC_UR_RMII;
    }

  sam_putreg(priv, SAM_EMAC_UR_OFFSET, regval);
  sam_putreg(priv, SAM_EMAC_NCR_OFFSET, ncr);

errout:

  /* Disable management port */

  regval  = sam_getreg(priv, SAM_EMAC_NCR_OFFSET);
  regval &= ~EMAC_NCR_MPE;
  sam_putreg(priv, SAM_EMAC_NCR_OFFSET, regval);
  return ret;
}

/****************************************************************************
 * Function: sam_linkup
 *
 * Description:
 *   Check if the link is up
 *
 * Input Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   true: The link is up
 *
 ****************************************************************************/

static bool sam_linkup(struct sam_emac_s *priv)
{
  uint32_t regval;
  uint16_t msr;
  uint16_t physr;
  bool linkup = false;
  int ret;

  /* Enable management port */

  regval  = sam_getreg(priv, SAM_EMAC_NCR_OFFSET);
  regval |= EMAC_NCR_MPE;
  sam_putreg(priv, SAM_EMAC_NCR_OFFSET, regval);

  ret = sam_phyread(priv, priv->phyaddr, MII_MSR, &msr);
  if (ret < 0)
    {
      nerr("ERROR: Failed to read MSR: %d\n", ret);
      goto errout;
    }

  if ((msr & MII_MSR_LINKSTATUS) == 0)
    {
      nerr("ERROR: MSR LinkStatus: %04x\n", msr);
      goto errout;
    }

  /* Re-configure Link speed */

  ret = sam_phyread(priv, priv->phyaddr, priv->attr->physr, &physr);
  if (ret < 0)
    {
      nerr("ERROR: Failed to read PHYSR: %d\n", ret);
      goto errout;
    }

  regval = sam_getreg(priv, SAM_EMAC_NCFGR_OFFSET);
  regval &= ~(EMAC_NCFGR_SPD | EMAC_NCFGR_FD);

  if ((msr & MII_MSR_100BASETXFULL) != 0 && sam_is100fdx(priv, physr))
    {
      /* Set EMAC for 100BaseTX and Full Duplex */

      regval |= (EMAC_NCFGR_SPD | EMAC_NCFGR_FD);
    }
  else if ((msr & MII_MSR_10BASETXFULL) != 0  && sam_is10fdx(priv, physr))
    {
      /* Set MII for 10BaseT and Full Duplex */

      regval |= EMAC_NCFGR_FD;
    }

  else if ((msr & MII_MSR_100BASETXHALF) != 0  && sam_is100hdx(priv, physr))
    {
      /* Set MII for 100BaseTX and Half Duplex */

      regval |= EMAC_NCFGR_SPD;
    }

#if 0
  else if ((msr & MII_MSR_10BASETXHALF) != 0  && sam_is10hdx(priv, physr))
    {
      /* Set MII for 10BaseT and Half Duplex */
    }
#endif

  sam_putreg(priv, SAM_EMAC_NCFGR_OFFSET, regval);

  /* Start the EMAC transfers */

  ninfo("Link is up\n");
  linkup = true;

errout:

  /* Disable management port */

  regval  = sam_getreg(priv, SAM_EMAC_NCR_OFFSET);
  regval &= ~EMAC_NCR_MPE;
  sam_putreg(priv, SAM_EMAC_NCR_OFFSET, regval);

  return linkup;
}

/****************************************************************************
 * Function: sam_phyinit
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

static int sam_phyinit(struct sam_emac_s *priv)
{
  uint32_t regval;
  uint32_t mck;
  int ret;

  /* Configure PHY clocking */

  regval = sam_getreg(priv, SAM_EMAC_NCFGR_OFFSET);
  regval &= ~EMAC_NCFGR_CLK_MASK;

  mck = BOARD_MCK_FREQUENCY;
  if (mck > (240 * 1000 * 1000))
    {
      nerr("ERROR: Cannot realize PHY clock\n");
      return -EINVAL;
    }
  else if (mck > (160 * 1000 * 1000))
    {
      regval |= EMAC_NCFGR_CLK_DIV96; /* MCK divided by 64 (MCK up to 240 MHz) */
    }
  else if (mck > (120 * 1000 * 1000))
    {
      regval |= EMAC_NCFGR_CLK_DIV64; /* MCK divided by 64 (MCK up to 160 MHz) */
    }
  else if (mck > (80 * 1000 * 1000))
    {
      regval |= EMAC_NCFGR_CLK_DIV48; /* MCK divided by 64 (MCK up to 120 MHz) */
    }
  else if (mck > (40 * 1000 * 1000))
    {
      regval |= EMAC_NCFGR_CLK_DIV32; /* MCK divided by 32 (MCK up to 80 MHz) */
    }
  else if (mck > (20 * 1000 * 1000))
    {
      regval |= EMAC_NCFGR_CLK_DIV16; /* MCK divided by 16 (MCK up to 40 MHz) */
    }
  else
    {
      regval |= EMAC_NCFGR_CLK_DIV8;  /* MCK divided by 8 (MCK up to 20 MHz) */
    }

  sam_putreg(priv, SAM_EMAC_NCFGR_OFFSET, regval);

  /* Check the PHY Address */

  priv->phyaddr = priv->attr->phyaddr;
  ret = sam_phyfind(priv, &priv->phyaddr);
  if (ret < 0)
    {
      nerr("ERROR: sam_phyfind failed: %d\n", ret);
      return ret;
    }

  if (priv->phyaddr != priv->attr->phyaddr)
    {
      sam_phyreset(priv);
    }

  return OK;
}

/****************************************************************************
 * Function: sam_ethgpioconfig
 *
 * Description:
 *   Configure PIOs for the EMAC0/1 RMII/MII interface.
 *
 *   Signal   Name Function                     MII      RMII
 *   -------- --------------------------------- -------- -----------
 *   TXCK     Transmit Clock or Reference Clock TXCK     REFCK
 *   TXEN     Transmit Enable                   TXEN     TXEN
 *   TX[3..0] Transmit Data                     TXD[3:0] TXD[1:0]
 *   TXER     Transmit Coding Error             TXER     Not Used
 *   RXCK     Receive Clock                     RXCK     Not Used
 *   RXDV     Receive Data Valid                RXDV     CRSDV
 *   RX[3..0] Receive Data                      RXD[3:0] RXD[1:0]
 *   RXER     Receive Error                     RXER     RXER
 *   CRS      Carrier Sense and Data Valid      CRS      Not Used
 *   COL      Collision Detect                  COL      Not Used
 *   MDC      Management Data Clock             MDC      MDC
 *   MDIO     Management Data Input/Output      MDIO     MDIO
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

static inline void sam_ethgpioconfig(struct sam_emac_s *priv)
{
#if defined(CONFIG_SAMV7_EMAC0)
  /* Configure EMAC0 PIO pins */

  if (priv->attr->emac == EMAC0_INTF)
    {
      /* Configure PIO pins common to RMII and MII mode */

       sam_configgpio(GPIO_EMAC0_TXCK);    /* Transmit Clock (or Reference Clock) */
       sam_configgpio(GPIO_EMAC0_TXEN);    /* Transmit Enable */
       sam_configgpio(GPIO_EMAC0_TX0);     /* Transmit data TXD0 */
       sam_configgpio(GPIO_EMAC0_TX1);     /* Transmit data TXD1 */
       sam_configgpio(GPIO_EMAC0_RXDV);    /* Receive Data Valid */
       sam_configgpio(GPIO_EMAC0_RX0);     /* Receive data RXD0 */
       sam_configgpio(GPIO_EMAC0_RX1);     /* Receive data RXD0 */
       sam_configgpio(GPIO_EMAC0_RXER);    /* Receive Error */
       sam_configgpio(GPIO_EMAC0_MDC);     /* Management Data Clock */
       sam_configgpio(GPIO_EMAC0_MDIO);    /* Management Data Input/Output */

      /* Configure additional PIO pins to support EMAC in MII mode */

      if (!priv->attr->rmii)
        {
          sam_configgpio(GPIO_EMAC0_TX2);  /* Transmit data TXD2 */
          sam_configgpio(GPIO_EMAC0_TX3);  /* Transmit data TXD3 */
          sam_configgpio(GPIO_EMAC0_TXER); /* Transmit Coding Error */
          sam_configgpio(GPIO_EMAC0_RXCK); /* Receive Clock */
          sam_configgpio(GPIO_EMAC0_RX2);  /* Receive data RXD2 */
          sam_configgpio(GPIO_EMAC0_RX3);  /* Receive data RXD3 */
          sam_configgpio(GPIO_EMAC0_CRS);  /* Carrier Sense and Data Valid */
          sam_configgpio(GPIO_EMAC0_COL);  /* Collision Detect */
        }
    }
  else
#endif

#if defined(CONFIG_SAMV7_EMAC1)
  /* Configure EMAC0 PIO pins */

  if (priv->attr->emac == EMAC1_INTF)
    {
      /* Configure PIO pins common to RMII and MII mode */

       sam_configgpio(GPIO_EMAC1_TXCK);    /* Transmit Clock (or Reference Clock) */
       sam_configgpio(GPIO_EMAC1_TXEN);    /* Transmit Enable */
       sam_configgpio(GPIO_EMAC1_TX0);     /* Transmit data TXD0 */
       sam_configgpio(GPIO_EMAC1_TX1);     /* Transmit data TXD1 */
       sam_configgpio(GPIO_EMAC1_RXDV);    /* Receive Data Valid */
       sam_configgpio(GPIO_EMAC1_RX0);     /* Receive data RXD0 */
       sam_configgpio(GPIO_EMAC1_RX1);     /* Receive data RXD0 */
       sam_configgpio(GPIO_EMAC1_RXER);    /* Receive Error */
       sam_configgpio(GPIO_EMAC1_MDC);     /* Management Data Clock */
       sam_configgpio(GPIO_EMAC1_MDIO);    /* Management Data Input/Output */

      /* Configure additional PIO pins to support EMAC in MII mode */

      if (!priv->attr->rmii)
        {
          sam_configgpio(GPIO_EMAC1_TX2);  /* Transmit data TXD2 */
          sam_configgpio(GPIO_EMAC1_TX3);  /* Transmit data TXD3 */
          sam_configgpio(GPIO_EMAC1_TXER); /* Transmit Coding Error */
          sam_configgpio(GPIO_EMAC1_RXCK); /* Receive Clock */
          sam_configgpio(GPIO_EMAC1_RX2);  /* Receive data RXD2 */
          sam_configgpio(GPIO_EMAC1_RX3);  /* Receive data RXD3 */
          sam_configgpio(GPIO_EMAC1_CRS);  /* Carrier Sense and Data Valid */
          sam_configgpio(GPIO_EMAC1_COL);  /* Collision Detect */
        }
    }
  else
#endif
    {
      nerr("ERROR: emac=%d\n", priv->attr->emac);
    }
}

/****************************************************************************
 * Function: sam_txreset
 *
 * Description:
 *  Reset the transmit logic
 *
 * Input Parameters:
 *   priv - A reference to the private driver state structure
 *   qid  - Identifies the queue to be reset
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *
 ****************************************************************************/

static void sam_txreset(struct sam_emac_s *priv, int qid)
{
  struct emac_txdesc_s *txdesc;
  struct sam_queue_s *xfrq;
  uint8_t *txbuffer;
  uintptr_t bufaddr;
  uintptr_t regaddr;
  uint32_t regval;
  int ndx;

  /* Get some convenience pointers */

  xfrq     = &priv->xfrq[qid];
  txdesc   = xfrq->txdesc;
  txbuffer = xfrq->txbuffer;

  /* Disable TX */

  regval = sam_getreg(priv, SAM_EMAC_NCR_OFFSET);
  regval &= ~EMAC_NCR_TXEN;
  sam_putreg(priv, SAM_EMAC_NCR_OFFSET, regval);

  /* Configure the TX descriptors. */

  xfrq->txhead = 0;
  xfrq->txtail = 0;

  for (ndx = 0; ndx < xfrq->ntxbuffers; ndx++)
    {
      bufaddr = (uintptr_t)&txbuffer[ndx * xfrq->txbufsize];

      /* Set the buffer address and mark the descriptor
       * as in used by firmware
       */

      txdesc[ndx].addr   = bufaddr;
      txdesc[ndx].status = EMACTXD_STA_USED;
    }

  /* Mark the final descriptor in the list */

  txdesc[xfrq->ntxbuffers - 1].status =
    EMACTXD_STA_USED | EMACTXD_STA_WRAP;

  /* Flush the entire TX descriptor table to RAM */

  up_clean_dcache((uintptr_t)txdesc,
                  (uintptr_t)txdesc +
                  xfrq->ntxbuffers * sizeof(struct emac_txdesc_s));

  /* Set the Transmit Buffer Queue Pointer Register */

  regaddr = qid ? SAM_EMAC_ISRPQ_TBQBAPQ_OFFSET(qid) : SAM_EMAC_TBQB_OFFSET;
  sam_putreg(priv, regaddr, (uint32_t)txdesc);
}

/****************************************************************************
 * Function: sam_rxreset
 *
 * Description:
 *  Reset the receive logic
 *
 * Input Parameters:
 *   priv - A reference to the private driver state structure
 *   qid  - The transfer queue to be reset
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *
 ****************************************************************************/

static void sam_rxreset(struct sam_emac_s *priv, int qid)
{
  struct emac_rxdesc_s *rxdesc;
  struct sam_queue_s *xfrq;
  uint8_t *rxbuffer;
  uintptr_t bufaddr;
  uintptr_t regaddr;
  uint32_t regval;
  int ndx;

  /* Get some convenience pointers */

  xfrq     = &priv->xfrq[qid];
  rxdesc   = xfrq->rxdesc;
  rxbuffer = xfrq->rxbuffer;

  /* Disable RX */

  regval  = sam_getreg(priv, SAM_EMAC_NCR_OFFSET);
  regval &= ~EMAC_NCR_RXEN;
  sam_putreg(priv, SAM_EMAC_NCR_OFFSET, regval);

  /* Configure the RX descriptors. */

  xfrq->rxndx = 0;
  for (ndx = 0; ndx < xfrq->nrxbuffers; ndx++)
    {
      bufaddr = (uintptr_t)&rxbuffer[ndx * xfrq->rxbufsize];
      DEBUGASSERT((bufaddr & ~EMACRXD_ADDR_MASK) == 0);

      /* Set the buffer address and remove EMACRXD_ADDR_OWNER and
       * EMACRXD_ADDR_WRAP.
       */

      rxdesc[ndx].addr   = bufaddr;
      rxdesc[ndx].status = 0;
    }

  /* Mark the final descriptor in the list */

  rxdesc[xfrq->nrxbuffers - 1].addr |= EMACRXD_ADDR_WRAP;

  /* Flush the entire RX descriptor table to RAM */

  up_clean_dcache((uintptr_t)rxdesc,
                  (uintptr_t)rxdesc +
                  xfrq->nrxbuffers * sizeof(struct emac_rxdesc_s));

  /* Set the Receive Buffer Queue Pointer Register */

  regaddr = qid ? SAM_EMAC_ISRPQ_RBQBAPQ_OFFSET(qid) : SAM_EMAC_RBQB_OFFSET;
  sam_putreg(priv, regaddr, (uint32_t)rxdesc);
}

/****************************************************************************
 * Function: sam_emac_enableclk
 *
 * Description:
 *  Enable clocking to the EMAC block
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

static void sam_emac_enableclk(struct sam_emac_s *priv)
{
#if defined(CONFIG_SAMV7_EMAC0) && defined(CONFIG_SAMV7_EMAC1)
  /* Both EMAC blocks are selected, which are we enabling? */

  if (priv->attr->emac == EMAC0_INTF)
    {
      sam_emac0_enableclk();
    }
  else
    {
      sam_emac1_enableclk();
    }

#elif defined(CONFIG_SAMV7_EMAC0)
  /* Only EMAC0 is selected */

  sam_emac0_enableclk();

#else
  /* Only EMAC1 is selected */

  sam_emac1_enableclk();
#endif
}

/****************************************************************************
 * Function: sam_emac_disableclk
 *
 * Description:
 *  Disable clocking to the EMAC block
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

#ifndef CONFIG_NETDEV_PHY_IOCTL
static void sam_emac_disableclk(struct sam_emac_s *priv)
{
#if defined(CONFIG_SAMV7_EMAC0) && defined(CONFIG_SAMV7_EMAC1)
  /* Both EMAC blocks are selected, which are we disabling? */

  if (priv->attr->emac == EMAC0_INTF)
    {
      sam_emac0_disableclk();
    }
  else
    {
      sam_emac1_disableclk();
    }

#elif defined(CONFIG_SAMV7_EMAC0)
  /* Only EMAC0 is selected */

  sam_emac0_disableclk();

#else
  /* Only EMAC1 is selected */

  sam_emac1_disableclk();
#endif
}
#endif

/****************************************************************************
 * Function: sam_emac_reset
 *
 * Description:
 *  Reset the EMAC block.
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

static void sam_emac_reset(struct sam_emac_s *priv)
{
#ifdef CONFIG_NETDEV_PHY_IOCTL
  uint32_t regval;

  /* We are supporting PHY IOCTLs, then do not reset the MAC.  If we do,
   * then we cannot communicate with the PHY.  So, instead, just disable
   * interrupts, cancel timers, and disable TX and RX.
   */

  sam_putreg(priv, SAM_EMAC_IDR_OFFSET, EMAC_INT_ALL);

  /* Reset RX and TX logic */

  sam_rxreset(priv, EMAC_QUEUE_0);
  sam_rxreset(priv, EMAC_QUEUE_1);
  sam_rxreset(priv, EMAC_QUEUE_2);

  if (g_emac_nqueues > 3)
    {
      sam_rxreset(priv, EMAC_QUEUE_3);
      sam_rxreset(priv, EMAC_QUEUE_4);
      sam_rxreset(priv, EMAC_QUEUE_5);
    }

  sam_txreset(priv, EMAC_QUEUE_0);
  sam_txreset(priv, EMAC_QUEUE_1);
  sam_txreset(priv, EMAC_QUEUE_2);

  if (g_emac_nqueues > 3)
    {
      sam_txreset(priv, EMAC_QUEUE_3);
      sam_txreset(priv, EMAC_QUEUE_4);
      sam_txreset(priv, EMAC_QUEUE_5);
    }

  /* Disable Rx and Tx, plus the statistics registers. */

  regval  = sam_getreg(priv, SAM_EMAC_NCR_OFFSET);
  regval &= ~(EMAC_NCR_RXEN | EMAC_NCR_TXEN | EMAC_NCR_WESTAT);
  sam_putreg(priv, SAM_EMAC_NCR_OFFSET, regval);

#else
  /* Disable all EMAC interrupts */

  sam_putreg(priv, SAM_EMAC_IDR_OFFSET, EMAC_INT_ALL);

  /* Reset RX and TX logic */

  sam_rxreset(priv, EMAC_QUEUE_0);
  sam_rxreset(priv, EMAC_QUEUE_1);
  sam_rxreset(priv, EMAC_QUEUE_2);

  if (g_emac_nqueues > 3)
    {
      sam_rxreset(priv, EMAC_QUEUE_3);
      sam_rxreset(priv, EMAC_QUEUE_4);
      sam_rxreset(priv, EMAC_QUEUE_5);
    }

  sam_txreset(priv, EMAC_QUEUE_0);
  sam_txreset(priv, EMAC_QUEUE_1);
  sam_txreset(priv, EMAC_QUEUE_2);

  if (g_emac_nqueues > 3)
    {
      sam_txreset(priv, EMAC_QUEUE_3);
      sam_txreset(priv, EMAC_QUEUE_4);
      sam_txreset(priv, EMAC_QUEUE_5);
    }

  /* Make sure that RX and TX are disabled; clear statistics registers */

  sam_putreg(priv, SAM_EMAC_NCR_OFFSET, EMAC_NCR_CLRSTAT);

  /* Disable clocking to the EMAC peripheral */

  sam_emac_disableclk(priv);

#endif
}

/****************************************************************************
 * Function: sam_macaddress
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

static void sam_macaddress(struct sam_emac_s *priv)
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
  sam_putreg(priv, SAM_EMAC_SAB1_OFFSET, regval);

  regval = (uint32_t)dev->d_mac.ether.ether_addr_octet[4] |
           (uint32_t)dev->d_mac.ether.ether_addr_octet[5] << 8;
  sam_putreg(priv, SAM_EMAC_SAT1_OFFSET, regval);
}

/****************************************************************************
 * Function: sam_ipv6multicast
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
static void sam_ipv6multicast(struct sam_emac_s *priv)
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

  sam_addmac(dev, mac);

#ifdef CONFIG_NET_ICMPv6_AUTOCONF
  /* Add the IPv6 all link-local nodes Ethernet address.  This is the
   * address that we expect to receive ICMPv6 Router Advertisement
   * packets.
   */

  sam_addmac(dev, g_ipv6_ethallnodes.ether_addr_octet);

#endif /* CONFIG_NET_ICMPv6_AUTOCONF */
#ifdef CONFIG_NET_ICMPv6_ROUTER
  /* Add the IPv6 all link-local routers Ethernet address.  This is the
   * address that we expect to receive ICMPv6 Router Solicitation
   * packets.
   */

  sam_addmac(dev, g_ipv6_ethallrouters.ether_addr_octet);

#endif /* CONFIG_NET_ICMPv6_ROUTER */
}
#endif /* CONFIG_NET_ICMPv6 */

/****************************************************************************
 * Function: sam_queue0_configure
 *
 * Description:
 *  Put transfer queue 0 in the operational state
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

static int sam_queue0_configure(struct sam_emac_s *priv)
{
  uint32_t regval;

  /* Set up the DMA configuration register
   *
   * EMAC_DCFGR_FBLDO_INCR4 - Attempt to use INCR8 AHB bursts
   *                        - No endian swap mode
   * EMAC_DCFGR_RXBMS_FULL  - 4 Kbytes Memory Size
   * EMAC_DCFGR_TXPBMS      - Full configured address space (4Kbytes)
   *                        - No Checksum generation offload enable
   * EMAC_DCFGR_DRBS        - Set configured receive buffer size
   *                          (units of 64 bytes)
   */

  regval = EMAC_DCFGR_FBLDO_INCR4 | EMAC_DCFGR_RXBMS_FULL | /* EMAC_DCFGR_TXPBMS | */
           EMAC_DCFGR_DRBS(priv->xfrq[0].rxbufsize >> 6);
  sam_putreg(priv, SAM_EMAC_DCFGR_OFFSET, regval);

  /* Reset RX and TX */

  sam_rxreset(priv, EMAC_QUEUE_0);
  sam_txreset(priv, EMAC_QUEUE_0);

  /* Enable Rx and Tx, plus the statistics registers. */

  regval  = sam_getreg(priv, SAM_EMAC_NCR_OFFSET);
  regval |= EMAC_NCR_RXEN | EMAC_NCR_TXEN | EMAC_NCR_WESTAT;
  sam_putreg(priv, SAM_EMAC_NCR_OFFSET, regval);

  /* Setup the interrupts for TX events, RX events, and error events */

  regval = EMAC_RX_INTS | EMAC_TX_INTS;
  sam_putreg(priv, SAM_EMAC_IER_OFFSET, regval);
  return OK;
}

/****************************************************************************
 * Function: sam_queue_configure
 *
 * Description:
 *  Put transfer queue n, n=1..(EMAC_NQUEUES-1), in the operational state
 *
 * Input Parameters:
 *   priv - A reference to the private driver state structure
 *   qid  - Identifies the transfer queue to be configured
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int sam_queue_configure(struct sam_emac_s *priv, int qid)
{
  uint32_t regval;

  /* Set the receive buffer size */

  regval = (uint32_t)priv->xfrq[qid].rxbufsize >> 6;
  sam_putreg(priv, SAM_EMAC_ISRPQ_RBSRPQ_OFFSET(qid), regval);

  /* Reset RX and TX */

  sam_rxreset(priv, qid);
  sam_txreset(priv, qid);

  /* Setup interrupts for RX/TX completion events */

  regval = EMAC_RX_INTS | EMAC_TX_INTS;
  sam_putreg(priv, SAM_EMAC_ISRPQ_IERPQ_OFFSET(qid), regval);
  return OK;
}

/****************************************************************************
 * Function: sam_emac_configure
 *
 * Description:
 *  Configure the EMAC interface for normal operation.
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

static int sam_emac_configure(struct sam_emac_s *priv)
{
  uint32_t regval;

  ninfo("Entry\n");

  /* Enable clocking to the EMAC peripheral */

  sam_emac_enableclk(priv);

  /* Disable TX, RX, clear statistics.  Disable all interrupts. */

  sam_putreg(priv, SAM_EMAC_NCR_OFFSET, EMAC_NCR_CLRSTAT);
  sam_putreg(priv, SAM_EMAC_IDR_OFFSET, EMAC_INT_ALL);
  sam_putreg(priv, SAM_EMAC_ISRPQ_IDRPQ_OFFSET(1), EMAC_INTPQ_ALL);
  sam_putreg(priv, SAM_EMAC_ISRPQ_IDRPQ_OFFSET(2), EMAC_INTPQ_ALL);

  /* Clear all status bits in the receive status register. */

  regval = (EMAC_RSR_RXOVR | EMAC_RSR_REC | EMAC_RSR_BNA | EMAC_RSR_HNO);
  sam_putreg(priv, SAM_EMAC_RSR_OFFSET, regval);

  /* Clear all status bits in the transmit status register */

  regval = (EMAC_TSR_UBR | EMAC_TSR_COL | EMAC_TSR_RLE | EMAC_TSR_TXGO |
            EMAC_TSR_TFC | EMAC_TSR_TXCOMP | EMAC_TSR_HRESP);
  sam_putreg(priv, SAM_EMAC_TSR_OFFSET, regval);

  /* Clear any pending interrupts */

  sam_getreg(priv, SAM_EMAC_ISR_OFFSET);
  sam_getreg(priv, SAM_EMAC_ISRPQ_ISRPQ_OFFSET(1));
  sam_getreg(priv, SAM_EMAC_ISRPQ_ISRPQ_OFFSET(2));

  /* Enable/disable the copy of data into the buffers, ignore broadcasts.
   * Don't copy FCS.
   */

  regval = EMAC_NCFGR_FD | EMAC_NCFGR_DBW_ZERO | EMAC_NCFGR_CLK_DIV64 |
           EMAC_NCFGR_MAXFS | EMAC_NCFGR_PEN | EMAC_NCFGR_RFCS;

#ifdef CONFIG_NET_PROMISCUOUS
  regval |=  EMAC_NCFGR_CAF;
#endif

#ifdef CONFIG_SAMV7_EMAC_NBC
  regval |=  EMAC_NCFGR_NBC;
#endif

  sam_putreg(priv, SAM_EMAC_NCFGR_OFFSET, regval);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: sam_emac_initialize
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

int sam_emac_initialize(int intf)
{
  struct sam_emac_s *priv;
  const struct sam_emacattr_s *attr;
  uint32_t regval;
  uint8_t *pktbuf;
#if defined(CONFIG_NETDEV_PHY_IOCTL) && defined(CONFIG_ARCH_PHY_INTERRUPT)
  uint8_t phytype;
#endif
  int ret;

  /* Determine if the chip has 3 or 6 queues.  This logic is for the
   * V71 only -- if you are using a different chip in the family,
   * the version number at which to switch from 3 to 6 queues may
   * be different.  Version 1 (Rev B) and higher have 6 queues.
   *
   * If both emacs are enabled, this code will be run twice, which
   * should not be a problem as the result will be the same each time
   * it is run.
   */

  regval = getreg32(SAM_CHIPID_CIDR);
  if (((regval & CHIPID_CIDR_VERSION_MASK) >> CHIPID_CIDR_VERSION_SHIFT) > 0)
    {
      g_emac_nqueues = EMAC_NQUEUES_REVB;  /* Change to Rev. B with 6 queues */
    }

#if defined(CONFIG_SAMV7_EMAC0)
  if (intf == EMAC0_INTF)
    {
      priv    = &g_emac0;
      attr    = &g_emac0_attr;
      pktbuf  = g_pktbuf0;

#if defined(CONFIG_NETDEV_PHY_IOCTL) && defined(CONFIG_ARCH_PHY_INTERRUPT)
      phytype = SAMV7_EMAC0_PHY_TYPE;
#endif
    }
  else
#endif
#if defined(CONFIG_SAMV7_EMAC1)
  if (intf == EMAC1_INTF)
    {
      priv    = &g_emac1;
      attr    = &g_emac1_attr;
      pktbuf  = g_pktbuf1;

#if defined(CONFIG_NETDEV_PHY_IOCTL) && defined(CONFIG_ARCH_PHY_INTERRUPT)
      phytype = SAMV7_EMAC1_PHY_TYPE;
#endif
    }
  else
#endif
    {
      nerr("ERROR:  Interface %d not supported\n", intf);
      return -EINVAL;
    }

  /* Initialize the driver structure */

  memset(priv, 0, sizeof(struct sam_emac_s));
  priv->attr          = attr;           /* Save the constant attributes */
  priv->dev.d_buf     = pktbuf;         /* Single packet buffer */
  priv->dev.d_ifup    = sam_ifup;       /* I/F up (new IP address) callback */
  priv->dev.d_ifdown  = sam_ifdown;     /* I/F down callback */
  priv->dev.d_txavail = sam_txavail;    /* New TX data callback */
#ifdef CONFIG_NET_MCASTGROUP
  priv->dev.d_addmac  = sam_addmac;     /* Add multicast MAC address */
  priv->dev.d_rmmac   = sam_rmmac;      /* Remove multicast MAC address */
#endif
#ifdef CONFIG_NETDEV_IOCTL
  priv->dev.d_ioctl   = sam_ioctl;      /* Support PHY ioctl() calls */
#if defined(CONFIG_NETDEV_PHY_IOCTL) && defined(CONFIG_ARCH_PHY_INTERRUPT)
  priv->phytype       = phytype;        /* Type of PHY on port */
#endif
#endif

  priv->dev.d_private = priv;           /* Used to recover private state from dev */

  /* Configure PIO pins to support EMAC */

  sam_ethgpioconfig(priv);

  /* Allocate buffers */

  ret = sam_buffer_allocate(priv);
  if (ret < 0)
    {
      nerr("ERROR: sam_buffer_allocate failed: %d\n", ret);
      return ret;
    }

  /* Attach the IRQ to the driver.  It will not be enabled at the AIC until
   * the interface is in the 'up' state.
   */

  ret = irq_attach(priv->attr->irq, sam_emac_interrupt, priv);
  if (ret < 0)
    {
      nerr("ERROR: Failed to attach the handler to the IRQ%d\n",
           priv->attr->irq);
      goto errout_with_buffers;
    }

  /* Enable clocking to the EMAC peripheral (just for sam_ifdown()) */

  sam_emac_enableclk(priv);

  /* Put the interface in the down state (disabling clocking again). */

  ret = sam_ifdown(&priv->dev);
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
  sam_buffer_free(priv);
  return ret;
}

/****************************************************************************
 * Function: sam_emac_setmacaddr
 *
 * Description:
 *   There are two ways that the Ethernet MAC address can be set:
 *
 *   1) Application level code can set the Ethernet MAC address using a
 *      netdev ioctl.  The application level code have gotten the MAC
 *      address from some configuration parameter or by accessing some
 *      non-volatile storage containing the address.  This is the
 *      "canonically correct" way to set the MAC address.
 *   2) Alternatively, the board logic may support some other less obvious
 *      non-volatile storage and the board-level boot-up code may access
 *      this and use this interface to set the Ethernet MAC address more
 *      directly.  This is mostly a kludge for the case where you just don't
 *      want to expose a application level storage interface.
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

int sam_emac_setmacaddr(int intf, uint8_t mac[6])
{
  struct sam_emac_s *priv;
  struct net_driver_s *dev;

  /* Get the driver state structure */

#if defined(CONFIG_SAMV7_EMAC0)
  if (intf == EMAC0_INTF)
    {
      priv = &g_emac0;
    }
  else
#endif
#if defined(CONFIG_SAMV7_EMAC1)
  if (intf == EMAC1_INTF)
    {
      priv = &g_emac1;
    }
  else
#endif
    {
      nerr("ERROR:  Interface %d not supported\n", intf);
      return -EINVAL;
    }

  /* Copy the MAC address into the device structure */

  dev = &priv->dev;
  memcpy(dev->d_mac.ether.ether_addr_octet, mac, 6);

  ninfo("%s MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
        dev->d_ifname,
        dev->d_mac.ether.ether_addr_octet[0],
        dev->d_mac.ether.ether_addr_octet[1],
        dev->d_mac.ether.ether_addr_octet[2],
        dev->d_mac.ether.ether_addr_octet[3],
        dev->d_mac.ether.ether_addr_octet[4],
        dev->d_mac.ether.ether_addr_octet[5]);

  return OK;
}

#endif /* CONFIG_NET && CONFIG_SAMV7_EMAC */
