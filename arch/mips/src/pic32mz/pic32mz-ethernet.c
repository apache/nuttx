/****************************************************************************
 * arch/mips/src/pic32mz/pic32mz_ethernet.c
 *
 *   Copyright (C) 2015-2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * This driver derives from the PIC32MZ Ethernet Driver
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
#if defined(CONFIG_NET) && defined(CONFIG_PIC32MZ_ETHERNET)

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>
#include <debug.h>
#include <errno.h>
#include <assert.h>

#include <arpa/inet.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>
#include <nuttx/net/mii.h>
#include <nuttx/net/netconfig.h>
#include <nuttx/net/arp.h>
#include <nuttx/net/netdev.h>

#ifdef CONFIG_NET_PKT
#  include <nuttx/net/pkt.h>
#endif

#include <arch/irq.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "up_internal.h"

#include "pic32mz-config.h"
#include "hardware/pic32mz-ethernet.h"

/* Does this chip have and Ethernet controller? */

#if CHIP_NETHERNET > 0

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

/* CONFIG_PIC32MZ_NINTERFACES determines the number of physical interfaces
 * that will be supported -- unless it is more than actually supported by the
 * hardware!
 */

#if !defined(CONFIG_PIC32MZ_NINTERFACES) || CONFIG_PIC32MZ_NINTERFACES > CHIP_NETHERNET
#  undef CONFIG_PIC32MZ_NINTERFACES
#  define CONFIG_PIC32MZ_NINTERFACES CHIP_NETHERNET
#endif

/* The logic here has a few hooks for support for multiple interfaces, but
 * that capability is not yet in place (and I won't worry about it until I
 * get the first multi-interface PIC32MZ).
 */

#if CONFIG_PIC32MZ_NINTERFACES > 1
#  warning "Only a single ethernet controller is supported"
#  undef CONFIG_PIC32MZ_NINTERFACES
#  define CONFIG_PIC32MZ_NINTERFACES 1
#endif

/* If IGMP is enabled, then accept multi-cast frames. */

#if defined(CONFIG_NET_MCASTGROUP) && !defined(CONFIG_PIC32MZ_MULTICAST)
#  define CONFIG_PIC32MZ_MULTICAST 1
#endif

/* Use defaults if the number of discriptors is not provided */

#ifndef CONFIG_PIC32MZ_ETH_NTXDESC
#  define CONFIG_PIC32MZ_ETH_NTXDESC 2
#endif

#if CONFIG_PIC32MZ_ETH_NTXDESC > 255
#  error "The number of TX descriptors exceeds the range of a uint8_t index"
#endif

#ifndef CONFIG_PIC32MZ_ETH_NRXDESC
#  define CONFIG_PIC32MZ_ETH_NRXDESC 4
#endif

/* Align descriptors' size to D-Cache line size */

#define DMA_BUFFER_MASK   (PIC32MZ_DCACHE_LINESIZE - 1)
#define DMA_ALIGN_UP(n)   (((n) + DMA_BUFFER_MASK) & ~DMA_BUFFER_MASK)
#define DMA_ALIGN_DOWN(n) ((n) & ~DMA_BUFFER_MASK)

/* A circular linked list of descriptors is used in this implementation.
 * This means the size of a descriptor is 20 bytes. If however a linear
 * implementation is used, the descriptor size will be 16 bytes and no
 * padding is required.
 */

#define DESC_SIZE      20
#define DESC_PADSIZE   DMA_ALIGN_UP(DESC_SIZE)

/* Make sure that the size of each buffer is a multiple of 4 bytes.  This
 * will force alignment of all buffers to 4-byte boundaries (this is needed
 * by the queuing logic which will cast each buffer address to a pointer
 * type).
 * Furthermore, the buffers should be aligned to D-Cache line size,
 * so that there are no spill-over effects at the borders of the
 * cleaned line when D-Cache is enabled.
 *
 * D-Cache line size is 16 bytes, then we will fulfill both of the conditions
 * above by aligning the buffer to D-Cache size.
 */

#define PIC32MZ_ALIGNED_BUFSIZE DMA_ALIGN_UP(CONFIG_NET_ETH_PKTSIZE)

/* The number of buffers will, then, be one for each descriptor plus one extra */

#define PIC32MZ_NBUFFERS (CONFIG_PIC32MZ_ETH_NRXDESC + CONFIG_PIC32MZ_ETH_NTXDESC + 1)

/* Debug Configuration ******************************************************/

/* Register/Descriptor debug -- can only happen if CONFIG_DEBUG_FEATURES is
 * selected. This will probably generate much more output than you care
 * to see.
 */

#ifndef CONFIG_DEBUG_FEATURES
#  undef CONFIG_NET_REGDEBUG
#  undef CONFIG_NET_DESCDEBUG
#endif

/* CONFIG_NET_DUMPPACKET will dump the contents of each packet to the
 * console.
 */

#ifndef CONFIG_DEBUG_FEATURES
#  undef  CONFIG_NET_DUMPPACKET
#endif

#ifdef CONFIG_NET_DUMPPACKET
#  define pic32mz_dumppacket(m,a,n) lib_dumpbuffer(m,a,n)
#else
#  define pic32mz_dumppacket(m,a,n)
#endif

/* Timing *******************************************************************/

/* TX poll deley = 1 seconds. CLK_TCK is the number of clock ticks per second */

#define PIC32MZ_WDDELAY        (1*CLK_TCK)

/* TX timeout = 1 minute */

#define PIC32MZ_TXTIMEOUT      (60*CLK_TCK)

/* PHY read/write delays in loop counts */

#define PIC32MZ_MIITIMEOUT     (666666)

/* Ethernet MII clocking.
 *
 * The clock divider used to create the MII Management Clock (MDC).  The MIIM
 * module uses the SYSCLK as an input clock.  According to the IEEE 802.3
 * Specification this should be no faster than 2.5 MHz. However, some PHYs
 * support clock rates up to 12.5 MHz.
 *
 * The board.h file provides the "ideal" divisor as BOARD_EMAC_MIIM_DIV.  We
 * pick the closest, actual divisor greater than or equal to this.
 */

#ifndef BOARD_EMAC_MIIM_DIV
#  error "MIIM divider is not defined"
#elif BOARD_EMAC_MIIM_DIV <= 4
#  define EMAC1_MCFG_CLKSEL_DIV EMAC1_MCFG_CLKSEL_DIV4
#elif BOARD_EMAC_MIIM_DIV <= 6
#  define EMAC1_MCFG_CLKSEL_DIV EMAC1_MCFG_CLKSEL_DIV6
#elif BOARD_EMAC_MIIM_DIV <= 8
#  define EMAC1_MCFG_CLKSEL_DIV EMAC1_MCFG_CLKSEL_DIV8
#elif BOARD_EMAC_MIIM_DIV <= 10
#  define EMAC1_MCFG_CLKSEL_DIV EMAC1_MCFG_CLKSEL_DIV10
#elif BOARD_EMAC_MIIM_DIV <= 14
#  define EMAC1_MCFG_CLKSEL_DIV EMAC1_MCFG_CLKSEL_DIV14
#elif BOARD_EMAC_MIIM_DIV <= 20
#  define EMAC1_MCFG_CLKSEL_DIV EMAC1_MCFG_CLKSEL_DIV20
#elif BOARD_EMAC_MIIM_DIV <= 28
#  define EMAC1_MCFG_CLKSEL_DIV EMAC1_MCFG_CLKSEL_DIV28
#elif BOARD_EMAC_MIIM_DIV <= 40
#  define EMAC1_MCFG_CLKSEL_DIV EMAC1_MCFG_CLKSEL_DIV40
#elif BOARD_EMAC_MIIM_DIV <= 48
#  define EMAC1_MCFG_CLKSEL_DIV EMAC1_MCFG_CLKSEL_DIV48
#elif BOARD_EMAC_MIIM_DIV <= 50
#  define EMAC1_MCFG_CLKSEL_DIV EMAC1_MCFG_CLKSEL_DIV50
#else
#  error "MIIM divider cannot be realized"
#endif

/* Interrupts ***************************************************************/

#define ETH_RXINTS (ETH_INT_RXOVFLW | ETH_INT_RXBUFNA |\
                    ETH_INT_RXDONE | ETH_INT_RXBUSE)
#define ETH_TXINTS (ETH_INT_TXABORT | ETH_INT_TXDONE | ETH_INT_TXBUSE)

/* Misc. Helpers ************************************************************/

/* This is a helper pointer for accessing the contents of the Ethernet header */

#define BUF ((struct eth_hdr_s *)priv->pd_dev.d_buf)

/* PHYs *********************************************************************/

/* Select PHY-specific values.  Add more PHYs as needed. */

#if defined(CONFIG_ETH0_PHY_KS8721)
#  define PIC32MZ_PHYNAME      "KS8721"
#  define PIC32MZ_PHYID1       MII_PHYID1_KS8721
#  define PIC32MZ_PHYID2       MII_PHYID2_KS8721
#  define PIC32MZ_HAVE_PHY     1
#elif defined(CONFIG_ETH0_PHY_DP83848C)
#  define PIC32MZ_PHYNAME      "DP83848C"
#  define PIC32MZ_PHYID1       MII_PHYID1_DP83848C
#  define PIC32MZ_PHYID2       MII_PHYID2_DP83848C
#  define PIC32MZ_HAVE_PHY     1
#elif defined(CONFIG_ETH0_PHY_LAN8720)
#  define PIC32MZ_PHYNAME      "LAN8720"
#  define PIC32MZ_PHYID1       MII_PHYID1_LAN8720
#  define PIC32MZ_PHYID2       MII_PHYID2_LAN8720
#  define PIC32MZ_HAVE_PHY     1
#elif defined(CONFIG_ETH0_PHY_LAN8740)
#  define PIC32MZ_PHYNAME      "LAN8740"
#  define PIC32MZ_PHYID1       MII_PHYID1_LAN8740
#  define PIC32MZ_PHYID2       MII_PHYID2_LAN8740
#  define PIC32MZ_HAVE_PHY     1
#elif defined(CONFIG_ETH0_PHY_LAN8740A)
#  define PIC32MZ_PHYNAME      "LAN8740A"
#  define PIC32MZ_PHYID1       MII_PHYID1_LAN8740A
#  define PIC32MZ_PHYID2       MII_PHYID2_LAN8740A
#  define PIC32MZ_HAVE_PHY     1
#else
#  warning "No PHY specified!"
#  undef PIC32MZ_HAVE_PHY
#endif

/* These definitions are used to remember the speed/duplex settings */

#define PIC32MZ_SPEED_MASK     0x01
#define PIC32MZ_SPEED_100      0x01
#define PIC32MZ_SPEED_10       0x00

#define PIC32MZ_DUPLEX_MASK    0x02
#define PIC32MZ_DUPLEX_FULL    0x02
#define PIC32MZ_DUPLEX_HALF    0x00

#define PIC32MZ_10BASET_HD     (PIC32MZ_SPEED_10  | PIC32MZ_DUPLEX_HALF)
#define PIC32MZ_10BASET_FD     (PIC32MZ_SPEED_10  | PIC32MZ_DUPLEX_FULL)
#define PIC32MZ_100BASET_HD    (PIC32MZ_SPEED_100 | PIC32MZ_DUPLEX_HALF)
#define PIC32MZ_100BASET_FD    (PIC32MZ_SPEED_100 | PIC32MZ_DUPLEX_FULL)

#ifdef CONFIG_PIC32MZ_PHY_SPEED100
#  ifdef CONFIG_PIC32MZ_PHY_FDUPLEX
#    define PIC32MZ_MODE_DEFLT PIC32MZ_100BASET_FD
#  else
#    define PIC32MZ_MODE_DEFLT PIC32MZ_100BASET_HD
#  endif
#else
#  ifdef CONFIG_PIC32MZ_PHY_FDUPLEX
#    define PIC32MZ_MODE_DEFLT PIC32MZ_10BASET_FD
#  else
#    define PIC32MZ_MODE_DEFLT PIC32MZ_10BASET_HD
#  endif
#endif

/* Misc Helper Macros *******************************************************/

#define PHYS_ADDR(va) ((uint32_t)(va) & 0x1fffffff)
#define VIRT_ADDR(pa) (KSEG1_BASE | (uint32_t)(pa))

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This union type forces the allocated size of RX descriptors to be the
 * padded to a exact multiple of the PIC32MZ D-Cache line size.
 */

union pic32mz_txdesc_u
{
  uint8_t pad[DESC_PADSIZE];
  struct pic32mz_txdesc_s txdesc;
};

union pic32mz_rxdesc_u
{
  uint8_t pad[DESC_PADSIZE];
  struct pic32mz_rxdesc_s rxdesc;
};

/* The pic32mz_driver_s encapsulates all state information for a single
 * single interface
 */

struct pic32mz_driver_s
{
  /* The following fields would only be necessary on chips that support
   * multiple Ethernet controllers.
   */

#if CONFIG_PIC32MZ_NINTERFACES > 1
  uint32_t   pd_base;           /* Ethernet controller base address */
  int        pd_irq;            /* Ethernet controller IRQ vector number */
  int        pd_irqsrc;         /* Ethernet controller IRQ source number */
#endif

  bool       pd_ifup;           /* true:ifup false:ifdown */
  bool       pd_txpending;      /* There is a pending Tx in pd_dev */
  bool       pd_polling;        /* Avoid concurrent attempts to poll */
  uint8_t    pd_mode;           /* Speed/duplex */
#ifdef PIC32MZ_HAVE_PHY
  uint8_t    pd_phyaddr;        /* PHY device address */
#endif
  uint8_t    pd_txnext;         /* Index to the next Tx descriptor */
  uint32_t   pd_inten;          /* Shadow copy of INTEN register */
  WDOG_ID    pd_txpoll;         /* TX poll timer */
  WDOG_ID    pd_txtimeout;      /* TX timeout timer */
  struct work_s pd_irqwork;     /* For deferring interrupt work to the work queue */
  struct work_s pd_pollwork;    /* For deferring poll work to the work queue */

  sq_queue_t pd_freebuffers;    /* The free buffer list */

  /* This holds the information visible to the NuttX network */

  struct net_driver_s pd_dev;  /* Interface understood by the network */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Descriptors and packet buffers */

union pic32mz_rxdesc_u g_rxdesc[CONFIG_PIC32MZ_ETH_NRXDESC]
  __attribute__((aligned(PIC32MZ_DCACHE_LINESIZE)));

union pic32mz_txdesc_u g_txdesc[CONFIG_PIC32MZ_ETH_NTXDESC]
  __attribute__((aligned(PIC32MZ_DCACHE_LINESIZE)));

uint8_t g_buffers[PIC32MZ_NBUFFERS * PIC32MZ_ALIGNED_BUFSIZE]
  __attribute__((aligned(PIC32MZ_DCACHE_LINESIZE)));

/* Array of ethernet driver status structures */

static struct pic32mz_driver_s g_ethdrvr[CONFIG_PIC32MZ_NINTERFACES];

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register operations */

#ifdef CONFIG_NET_REGDEBUG
static void pic32mz_printreg(uint32_t addr, uint32_t val, bool iswrite);
static void pic32mz_checkreg(uint32_t addr, uint32_t val, bool iswrite);
static uint32_t pic32mz_getreg(uint32_t addr);
static void pic32mz_putreg(uint32_t val, uint32_t addr);
#else
#  define pic32mz_getreg(addr)     getreg32(addr)
#  define pic32mz_putreg(val,addr) putreg32(val,addr)
#endif

/* Buffer and descriptor management */

#ifdef CONFIG_NET_DESCDEBUG
static void pic32mz_dumptxdesc(struct pic32mz_txdesc_s *txdesc,
                               const char *msg);
static void pic32mz_dumprxdesc(struct pic32mz_rxdesc_s *rxdesc,
                               const char *msg);
#else
#  define pic32mz_dumptxdesc(txdesc,msg)
#  define pic32mz_dumprxdesc(rxdesc,msg)
#endif

static inline void pic32mz_bufferinit(struct pic32mz_driver_s *priv);
static uint8_t *pic32mz_allocbuffer(struct pic32mz_driver_s *priv);
static void pic32mz_freebuffer(struct pic32mz_driver_s *priv,
                               uint8_t *buffer);

static inline void pic32mz_txdescinit(struct pic32mz_driver_s *priv);
static inline void pic32mz_rxdescinit(struct pic32mz_driver_s *priv);
static inline struct pic32mz_txdesc_s *pic32mz_txdesc(
  struct pic32mz_driver_s *priv);
static inline void pic32mz_txnext(struct pic32mz_driver_s *priv);
static inline void pic32mz_rxreturn(struct pic32mz_rxdesc_s *rxdesc);
static struct pic32mz_rxdesc_s *pic32mz_rxdesc(
  struct pic32mz_driver_s *priv);

/* Common TX logic */

static int  pic32mz_transmit(struct pic32mz_driver_s *priv);
static int  pic32mz_txpoll(struct net_driver_s *dev);
static void pic32mz_poll(struct pic32mz_driver_s *priv);
static void pic32mz_timerpoll(struct pic32mz_driver_s *priv);

/* Interrupt handling */

static void pic32mz_response(struct pic32mz_driver_s *priv);
static void pic32mz_rxdone(struct pic32mz_driver_s *priv);
static void pic32mz_txdone(struct pic32mz_driver_s *priv);

static void pic32mz_interrupt_work(void *arg);
static int  pic32mz_interrupt(int irq, void *context, FAR void *arg);

/* Watchdog timer expirations */

static void pic32mz_txtimeout_work(void *arg);
static void pic32mz_txtimeout_expiry(int argc, uint32_t arg, ...);

static void pic32mz_poll_work(void *arg);
static void pic32mz_poll_expiry(int argc, uint32_t arg, ...);

/* NuttX callback functions */

static int pic32mz_ifup(struct net_driver_s *dev);
static int pic32mz_ifdown(struct net_driver_s *dev);

static void pic32mz_txavail_work(void *arg);
static int pic32mz_txavail(struct net_driver_s *dev);

#ifdef CONFIG_NET_MCASTGROUP
static int pic32mz_addmac(struct net_driver_s *dev, const uint8_t *mac);
static int pic32mz_rmmac(struct net_driver_s *dev, const uint8_t *mac);
#endif

/* PHY initialization functions */

#ifdef PIC32MZ_HAVE_PHY
#  ifdef CONFIG_NET_REGDEBUG
static void pic32mz_showmii(uint8_t phyaddr, const char *msg);
#  else
#    define pic32mz_showmii(phyaddr,msg)
#  endif

static void pic32mz_phybusywait(void);
static void pic32mz_phywrite(uint8_t phyaddr, uint8_t regaddr,
                             uint16_t phydata);
static uint16_t pic32mz_phyread(uint8_t phyaddr, uint8_t regaddr);
static inline int pic32mz_phyreset(uint8_t phyaddr);
#  ifdef CONFIG_PIC32MZ_PHY_AUTONEG
static inline int pic32mz_phyautoneg(uint8_t phyaddr);
#  endif
static int pic32mz_phymode(uint8_t phyaddr, uint8_t mode);
static inline int pic32mz_phyinit(struct pic32mz_driver_s *priv);
#else
#  define pic32mz_phyinit(priv)
#endif

/* EMAC Initialization functions */

static void pic32mz_macmode(uint8_t mode);
static void pic32mz_ethreset(struct pic32mz_driver_s *priv);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mz_printreg
 *
 * Description:
 *   Print the contents of an PIC32MZ register operation
 *
 ****************************************************************************/

#ifdef CONFIG_NET_REGDEBUG
static void pic32mz_printreg(uint32_t addr, uint32_t val, bool iswrite)
{
  ninfo("%08x%s%08x\n", addr, iswrite ? "<-" : "->", val);
}
#endif

/****************************************************************************
 * Name: pic32mz_checkreg
 *
 * Description:
 *   Get the contents of an PIC32MZ register
 *
 ****************************************************************************/

#ifdef CONFIG_NET_REGDEBUG
static void pic32mz_checkreg(uint32_t addr, uint32_t val, bool iswrite)
{
  static uint32_t prevaddr = 0;
  static uint32_t preval = 0;
  static uint32_t count = 0;
  static bool     prevwrite = false;

  /* Is this the same value that we read from/wrote to the same register
   * last time? Are we polling the register?  If so, suppress the output.
   */

  if (addr == prevaddr && val == preval && prevwrite == iswrite)
    {
      /* Yes.. Just increment the count */

      count++;
    }
  else
    {
      /* No this is a new address or value or operation. Were there any
       * duplicate accesses before this one?
       */

      if (count > 0)
        {
          /* Yes.. Just one? */

          if (count == 1)
            {
              /* Yes.. Just one */

              pic32mz_printreg(prevaddr, preval, prevwrite);
            }
          else
            {
              /* No.. More than one. */

              ninfo("[repeats %d more times]\n", count);
            }
        }

      /* Save the new address, value, count, and operation for next time */

      prevaddr  = addr;
      preval    = val;
      count     = 0;
      prevwrite = iswrite;

      /* Show the new register access */

      pic32mz_printreg(addr, val, iswrite);
    }
}
#endif

/****************************************************************************
 * Name: pic32mz_getreg
 *
 * Description:
 *   Get the contents of an PIC32MZ register
 *
 ****************************************************************************/

#ifdef CONFIG_NET_REGDEBUG
static uint32_t pic32mz_getreg(uint32_t addr)
{
  /* Read the value from the register */

  uint32_t val = getreg32(addr);

  /* Check if we need to print this value */

  pic32mz_checkreg(addr, val, false);
  return val;
}
#endif

/****************************************************************************
 * Name: pic32mz_putreg
 *
 * Description:
 *   Set the contents of an PIC32MZ register to a value
 *
 ****************************************************************************/

#ifdef CONFIG_NET_REGDEBUG
static void pic32mz_putreg(uint32_t val, uint32_t addr)
{
  /* Check if we need to print this value */

  pic32mz_checkreg(addr, val, true);

  /* Write the value */

  putreg32(val, addr);
}
#endif

/****************************************************************************
 * Function: pic32mz_dumptxdesc
 *
 * Description:
 *   Dump the contents of the specified TX descriptor
 *
 * Input Parameters:
 *   txdesc - Pointer to the TX descriptor to dump
 *   msg    - Annotation for the TX descriptor
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_NET_DESCDEBUG
static void pic32mz_dumptxdesc(struct pic32mz_txdesc_s *txdesc,
                               const char *msg)
{
  ninfo("TX Descriptor [%p]: %s\n", txdesc, msg);
  ninfo("   status: %08x\n", txdesc->status);
  ninfo("  address: %08x [%08x]\n",
        txdesc->address, VIRT_ADDR(txdesc->address));
  ninfo("     tsv1: %08x\n", txdesc->tsv1);
  ninfo("     tsv2: %08x\n", txdesc->tsv2);
  ninfo("   nexted: %08x [%08x]\n",
        txdesc->nexted, VIRT_ADDR(txdesc->nexted));
}
#endif

/****************************************************************************
 * Function: pic32mz_dumprxdesc
 *
 * Description:
 *   Dump the contents of the specified RX descriptor
 *
 * Input Parameters:
 *   txdesc - Pointer to the RX descriptor to dump
 *   msg    - Annotation for the RX descriptor
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_NET_DESCDEBUG
static void pic32mz_dumprxdesc(struct pic32mz_rxdesc_s *rxdesc,
                               const char *msg)
{
  ninfo("RX Descriptor [%p]: %s\n", rxdesc, msg);
  ninfo("   status: %08x\n", rxdesc->status);
  ninfo("  address: %08x [%08x]\n",
        rxdesc->address, VIRT_ADDR(rxdesc->address));
  ninfo("     rsv1: %08x\n", rxdesc->rsv1);
  ninfo("     rsv2: %08x\n", rxdesc->rsv2);
  ninfo("   nexted: %08x [%08x]\n",
        rxdesc->nexted, VIRT_ADDR(rxdesc->nexted));
}
#endif

/****************************************************************************
 * Function: pic32mz_bufferinit
 *
 * Description:
 *   Initialize the buffers by placing them all in a free list
 *
 * Input Parameters:
 *   priv - Pointer to EMAC device driver structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void pic32mz_bufferinit(struct pic32mz_driver_s *priv)
{
  uint8_t *buffer;
  int i;

  for (i = 0, buffer = g_buffers; i < PIC32MZ_NBUFFERS; i++)
    {
      /* Add the buffer to the end of the list of free buffers */

      sq_addlast((sq_entry_t *)buffer, &priv->pd_freebuffers);

      /* Get the address of the next buffer */

      buffer += PIC32MZ_ALIGNED_BUFSIZE;
    }
}

/****************************************************************************
 * Function: pic32mz_allocbuffer
 *
 * Description:
 *   Allocate one buffer by removing it from the free list
 *
 * Input Parameters:
 *   priv - Pointer to EMAC device driver structure
 *
 * Returned Value:
 *   Pointer to the allocated buffer (or NULL on failure)
 *
 ****************************************************************************/

static uint8_t *pic32mz_allocbuffer(struct pic32mz_driver_s *priv)
{
  /* Return the next free buffer from the head of the free buffer list */

  return (uint8_t *)sq_remfirst(&priv->pd_freebuffers);
}

/****************************************************************************
 * Function: pic32mz_freebuffer
 *
 * Description:
 *   Free one buffer by returning it to the free list
 *
 * Input Parameters:
 *   priv - Pointer to EMAC device driver structure
 *
 * Returned Value:
 *   Pointer to the allocated buffer (or NULL on failure)
 *
 ****************************************************************************/

static void pic32mz_freebuffer(struct pic32mz_driver_s *priv,
                               uint8_t *buffer)
{
  /* Add the buffer to the end of the free buffer list */

  sq_addlast((sq_entry_t *)buffer, &priv->pd_freebuffers);
}

/****************************************************************************
 * Function: pic32mz_txdescinit
 *
 * Description:
 *   Initialize the EMAC Tx descriptor table
 *
 * Input Parameters:
 *   priv - Pointer to EMAC device driver structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static inline void pic32mz_txdescinit(struct pic32mz_driver_s *priv)
{
  struct pic32mz_txdesc_s *txdesc;
  int i;

  /* Assign a buffer to each TX descriptor.  For now, just mark each TX
   * descriptor as owned by softare and not linked.
   */

  for (i = 0; i < CONFIG_PIC32MZ_ETH_NTXDESC; i++)
    {
      /* Point to the next entry */

      txdesc          = &g_txdesc[i].txdesc;

      /* Initialize the buffer.  It is idle, owned by software and has
       * no buffer assigned to it.
       */

      txdesc->status  = TXDESC_STATUS_SOWN | TXDESC_STATUS_NPV;
      txdesc->address = 0;
      txdesc->tsv1    = 0;
      txdesc->tsv2    = 0;

      /* Set the NEXTED pointer.  If this is the last descriptor in the
       * list, then set the NEXTED pointer back to the first entry,
       * creating a ring.
       */

      if (i == (CONFIG_PIC32MZ_ETH_NRXDESC - 1))
        {
          txdesc->nexted = PHYS_ADDR(g_txdesc);
        }
      else
        {
          txdesc->nexted = PHYS_ADDR(&g_txdesc[i + 1]);
        }

      pic32mz_dumptxdesc(txdesc, "Initial");
    }

  /* Flush all of the initialized TX descriptors to physical memory */

  up_flush_dcache((uintptr_t)g_txdesc,
                  (uintptr_t)g_txdesc +
                  CONFIG_PIC32MZ_ETH_NTXDESC *
                  sizeof(union pic32mz_txdesc_u));

  /* Position the Tx index to the first descriptor in the ring */

  priv->pd_txnext = 0;

  /* Update the ETHTXST register with the physical address of the head of
   * the TX descriptors list.
   */

  pic32mz_putreg(PHYS_ADDR(g_txdesc), PIC32MZ_ETH_TXST);
}

/****************************************************************************
 * Function: pic32mz_rxdescinit
 *
 * Description:
 *   Initialize the EMAC Rx descriptor table
 *
 * Input Parameters:
 *   priv - Pointer to EMAC device driver structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static inline void pic32mz_rxdescinit(struct pic32mz_driver_s *priv)
{
  struct pic32mz_rxdesc_s *rxdesc;
  int i;

  /* Prepare a list of RX descriptors populated with valid buffers for
   * messages to be received. Properly update the NPV, EOWN = 1 and
   * DATA_BUFFER_ADDRESS fields in the RX descriptors. The
   * DATA_BUFFER_ADDRESS should contain the physical address of the
   * corresponding RX buffer.
   */

  for (i = 0; i < CONFIG_PIC32MZ_ETH_NRXDESC; i++)
    {
      /* Point to the next entry */

      rxdesc = &g_rxdesc[i].rxdesc;

      /* Initialize the descriptor.  Assign it a buffer and make it ready
       * for reception.
       */

      rxdesc->rsv1    = 0;
      rxdesc->rsv2    = 0;
      rxdesc->address = PHYS_ADDR(pic32mz_allocbuffer(priv));
      rxdesc->status  = RXDESC_STATUS_EOWN | RXDESC_STATUS_NPV;

      /* Set the NEXTED pointer.  If this is the last descriptor in the
       * list, then set the NEXTED pointer back to the first entry,
       * creating a ring.
       */

      if (i == (CONFIG_PIC32MZ_ETH_NRXDESC - 1))
        {
          rxdesc->nexted = PHYS_ADDR(g_rxdesc);
        }
      else
        {
          rxdesc->nexted = PHYS_ADDR(&g_rxdesc[i + 1]);
        }

      pic32mz_dumprxdesc(rxdesc, "Initial");
    }

  /* Flush all of the initialized RX descriptors to physical memory */

  up_flush_dcache((uintptr_t)g_rxdesc,
                  (uintptr_t)g_rxdesc +
                  CONFIG_PIC32MZ_ETH_NRXDESC *
                  sizeof(union pic32mz_rxdesc_u));

  /* Update the ETHRXST register with the physical address of the head of the
   * RX descriptors list.
   */

  pic32mz_putreg(PHYS_ADDR(g_rxdesc), PIC32MZ_ETH_RXST);
}

/****************************************************************************
 * Function: pic32mz_txdesc
 *
 * Description:
 *   Check if the next Tx descriptor is available.
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   A pointer to the next available Tx descriptor on success; NULL if the
 *   next Tx dscriptor is not available.
 *
 ****************************************************************************/

static inline struct pic32mz_txdesc_s *
  pic32mz_txdesc(struct pic32mz_driver_s *priv)
{
  struct pic32mz_txdesc_s *txdesc;

  /* Get a reference to the next Tx descriptor in the ring */

  txdesc = &g_txdesc[priv->pd_txnext].txdesc;

  /* Force re-reading of the TX descriptor from physical memory.
   * The descriptor may have been owned by the Ethernet controller.
   * Cache data would be stale in this case.
   */

  up_invalidate_dcache((uintptr_t)txdesc,
                       (uintptr_t)txdesc + sizeof(union pic32mz_txdesc_u));

  /* Check if the EOWN bit is cleared. If it is, this descriptor is now under
   * software control and the message has been transmitted.
   *
   * Also check that the buffer address is NULL.  There is a race condition
   * in that the hardware may have completed the transfer, but there may
   * still be a valid buffer attached to the Tx descriptor because we have
   * not yet processed the Tx done condition.  We will know that the Tx
   * done condition has been processed when the buffer has been freed and
   * reset to zero.
   */

  if ((txdesc->status & TXDESC_STATUS_EOWN) == 0 && txdesc->address == 0)
    {
      /* Yes.. return a pointer to the descriptor */

      return txdesc;
    }

  /* The next Tx descriptor is still owned by the Ethernet controller.. the
   * Tx ring if full and cannot be used now.  Return NULL.
   */

  return NULL;
}

/****************************************************************************
 * Function: pic32mz_txnext
 *
 * Description:
 *   After the next Tx descriptor has been given to the hardware, update the
 *   index to the next Tx descriptor in the ring.
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void pic32mz_txnext(struct pic32mz_driver_s *priv)
{
  /* Increment the index to the next Tx descriptor in the ring */

  int txnext = priv->pd_txnext + 1;

  /* If the new index would go beyond the end of the allocated descriptors
   * for the Tx ring, then reset to first descriptor.
   */

  if (txnext >= CONFIG_PIC32MZ_ETH_NTXDESC)
    {
      txnext = 0;
    }

  /* Save the index to the next Tx descriptor */

  priv->pd_txnext = txnext;
}

/****************************************************************************
 * Function: pic32mz_rxreturn
 *
 * Description:
 *   Return an RX descriptor to the hardware.
 *
 * Input Parameters:
 *   rxdesc - Reference to the RX descriptor to be returned
 *
 * Returned Value:
 *    None
 *
 * Assumptions:
 *
 ****************************************************************************/

static inline void pic32mz_rxreturn(struct pic32mz_rxdesc_s *rxdesc)
{
  rxdesc->rsv1   = 0;
  rxdesc->rsv2   = 0;
  rxdesc->status = RXDESC_STATUS_EOWN | RXDESC_STATUS_NPV;
  pic32mz_dumprxdesc(rxdesc, "Returned to hardware");

  /* Flush the content of the modified TX descriptor into physical memory. */

  up_flush_dcache((uintptr_t)rxdesc,
                  (uintptr_t)rxdesc + sizeof(union pic32mz_rxdesc_u));
}

/****************************************************************************
 * Function: pic32mz_rxdesc
 *
 * Description:
 *   Check if a RX descriptor is owned by the software.
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   A pointer to the RX descriptor on success; NULL on failure
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/

static struct pic32mz_rxdesc_s *pic32mz_rxdesc(struct pic32mz_driver_s *priv)
{
  struct pic32mz_rxdesc_s *rxdesc;
  int i;

  /* Inspect the list of RX descriptors to see if the EOWN bit is cleared.
   * If it is, this descriptor is now under software control and a message
   * was received. Use SOP and EOP to extract the message, use BYTE_COUNT,
   * RXF_RSV, RSV and PKT_CHECKSUM to get the message characteristics.
   */

  for (i = 0; i < CONFIG_PIC32MZ_ETH_NRXDESC; i++)
    {
      /* Check if software owns this descriptor */

      rxdesc = &g_rxdesc[i].rxdesc;

      /* Forces the first RX descriptor to be re-read from physical memory */

      up_invalidate_dcache((uintptr_t)rxdesc,
                           (uintptr_t)rxdesc +
                            sizeof(union pic32mz_rxdesc_u));

      if ((rxdesc->status & RXDESC_STATUS_EOWN) == 0)
        {
          /* Yes.. return a pointer to the descriptor */

          return rxdesc;
        }
    }

  /* All descriptors are owned by the Ethernet controller.. return NULL */

  return NULL;
}

/****************************************************************************
 * Function: pic32mz_transmit
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

static int pic32mz_transmit(struct pic32mz_driver_s *priv)
{
  struct pic32mz_txdesc_s *txdesc;
  uint32_t status;

  /* Verify that the hardware is ready to send another packet.  If we get
   * here, then we are committed to sending a packet; Higher level logic
   * must have assured that there is no transmission in progress.
   */

  DEBUGASSERT(priv->pd_dev.d_buf != NULL &&
              priv->pd_dev.d_len <= CONFIG_NET_ETH_PKTSIZE);

  /* Increment statistics and dump the packet (if so configured) */

  NETDEV_TXPACKETS(&priv->pd_dev);
  pic32mz_dumppacket("Transmit packet",
                      priv->pd_dev.d_buf, priv->pd_dev.d_len);

  /* Flush the content of the TX buffer into physical memory */

  up_flush_dcache((uintptr_t)priv->pd_dev.d_buf,
                  (uintptr_t)priv->pd_dev.d_buf + PIC32MZ_ALIGNED_BUFSIZE);

  /* In order to transmit a message:
   *
   * The SOP, EOP, DATA_BUFFER_ADDRESS and BYTE_COUNT will be updated when a
   * particular message has to be transmitted. The DATA_BUFFER_ADDRESS will
   * contain the physical address of the message, the BYTE_COUNT message
   * size. SOP and EOP are set depending on how many packets are needed to
   * transmit the message.
   */

  /* Find the next available TX descriptor.  We are guaranteed that is will
   * not fail by upstream logic that assures that a TX packet is available
   * before polling the network.
   */

  txdesc = pic32mz_txdesc(priv);
  DEBUGASSERT(txdesc != NULL);
  pic32mz_dumptxdesc(txdesc, "Before transmit setup");

  /* Remove the transmit buffer from the device structure and assign it to
   * the TX descriptor.
   */

  txdesc->address    = PHYS_ADDR(priv->pd_dev.d_buf);
  priv->pd_dev.d_buf = NULL;

  /* Set the BYTE_COUNT for in the TX descriptor with the number of bytes
   * contained in the buffer.
   */

  status = ((uint32_t)priv->pd_dev.d_len << TXDESC_STATUS_BYTECOUNT_SHIFT);
  priv->pd_dev.d_len = 0;

  /* Set EOWN = 1 to indicate that the packet belongs to Ethernet and set
   * both SOP and EOP to indicate that the packet both begins and ends with
   * this frame.
   */

  status        |= (TXDESC_STATUS_EOWN | TXDESC_STATUS_NPV |
                    TXDESC_STATUS_EOP | TXDESC_STATUS_SOP);
  txdesc->status = status;
  pic32mz_dumptxdesc(txdesc, "After transmit setup");

  /* Flush the content of the modified TX descriptor into physical memory */

  up_flush_dcache((uintptr_t)txdesc,
                  (uintptr_t)txdesc + sizeof(union pic32mz_txdesc_u));

  /* Update the index to the next descriptor to use in the Tx ring */

  pic32mz_txnext(priv);

  /* Enable the transmission of the message by setting the TXRTS bit (ETHCON1:9). */

  pic32mz_putreg(ETH_CON1_TXRTS | ETH_CON1_ON, PIC32MZ_ETH_CON1SET);

  /* Enable Tx interrupts */

  priv->pd_inten |= ETH_TXINTS;
  pic32mz_putreg(priv->pd_inten, PIC32MZ_ETH_IEN);

  /* Setup the TX timeout watchdog (perhaps restarting the timer) */

  wd_start(priv->pd_txtimeout, PIC32MZ_TXTIMEOUT,
           pic32mz_txtimeout_expiry, 1, (uint32_t)priv);

  return OK;
}

/****************************************************************************
 * Function: pic32mz_txpoll
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

static int pic32mz_txpoll(struct net_driver_s *dev)
{
  struct pic32mz_driver_s *priv = (struct pic32mz_driver_s *)dev->d_private;
  int ret = OK;

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  if (priv->pd_dev.d_len > 0)
    {
      /* Look up the destination MAC address and add it to the Ethernet
       * header.
       */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
      if (IFF_IS_IPv4(priv->pd_dev.d_flags))
#endif
        {
          arp_out(&priv->pd_dev);
        }
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
      else
#endif
        {
          neighbor_out(&priv->pd_dev);
        }
#endif /* CONFIG_NET_IPv6 */

      if (!devif_loopback(&priv->pd_dev))
        {
          /* Send this packet.  In this context, we know that there is space
           * for at least one more packet in the descriptor list.
           */

          pic32mz_transmit(priv);

          /* Check if the next TX descriptor is available. If not, return a
           * non-zero value to terminate the poll.
           */

          if (pic32mz_txdesc(priv) == NULL)
            {
              /* There are no more TX descriptors/buffers available..
               * stop the poll
               */

              return -EAGAIN;
            }

          /* Get the next Tx buffer needed in order to continue the poll */

          priv->pd_dev.d_buf = pic32mz_allocbuffer(priv);
          if (priv->pd_dev.d_buf == NULL)
            {
              /* We have no more buffers available for the next Tx..
               * stop the poll
               */

              return -ENOMEM;
            }
        }
    }

  /* If zero is returned, the polling will continue until all connections
   * have been examined.
   */

  return ret;
}

/****************************************************************************
 * Function: pic32mz_poll
 *
 * Description:
 *   Perform the network poll.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void pic32mz_poll(struct pic32mz_driver_s *priv)
{
  /* Is there already a poll in progress.  This happens, for example, when
   * debugging output is enabled.  Interrupts may be re-enabled while debug
   * output is performed and a timer expiration could attempt a concurrent
   * poll.
   */

  if (!priv->pd_polling)
    {
      /* Assign a buffer for the poll */

      DEBUGASSERT(priv->pd_dev.d_buf == NULL);
      priv->pd_dev.d_buf = pic32mz_allocbuffer(priv);
      if (priv->pd_dev.d_buf != NULL)
        {
          /* And perform the poll */

          priv->pd_polling = true;
          devif_poll(&priv->pd_dev, pic32mz_txpoll);

          /* Free any buffer left attached after the poll */

          if (priv->pd_dev.d_buf != NULL)
            {
              pic32mz_freebuffer(priv, priv->pd_dev.d_buf);
              priv->pd_dev.d_buf = NULL;
            }

          priv->pd_polling = false;
        }
    }
}

/****************************************************************************
 * Function: pic32mz_timerpoll
 *
 * Description:
 *   Perform the network timer poll.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void pic32mz_timerpoll(struct pic32mz_driver_s *priv)
{
  /* Is there already a poll in progress.  This happens, for example, when
   * debugging output is enabled.  Interrupts may be re-enabled while debug
   * output is performed and a timer expiration could attempt a concurrent
   * poll.
   */

  if (!priv->pd_polling)
    {
      DEBUGASSERT(priv->pd_dev.d_buf == NULL);
      priv->pd_dev.d_buf = pic32mz_allocbuffer(priv);
      if (priv->pd_dev.d_buf != NULL)
        {
          /* And perform the poll */

          priv->pd_polling = true;
          devif_timer(&priv->pd_dev, PIC32MZ_WDDELAY, pic32mz_txpoll);

          /* Free any buffer left attached after the poll */

          if (priv->pd_dev.d_buf != NULL)
            {
              pic32mz_freebuffer(priv, priv->pd_dev.d_buf);
              priv->pd_dev.d_buf = NULL;
            }

          priv->pd_polling = false;
        }
    }
}

/****************************************************************************
 * Function: pic32mz_response
 *
 * Description:
 *   While processing an RxDone event, higher logic decides to send a packet,
 *   possibly a response to the incoming packet (but probably not in reality)
 *   However, since the Rx and Tx operations are decoupled, there is no
 *   guarantee that there will be a Tx descriptor available at that time.
 *   This function will perform that check and, if no Tx descriptor is
 *   available, this function will (1) stop incoming Rx processing (bad), and
 *   (2) hold the outgoing packet in a pending state until the next Tx
 *   interrupt occurs.
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

static void pic32mz_response(struct pic32mz_driver_s *priv)
{
  struct pic32mz_txdesc_s *txdesc;

  /* Check if the next TX descriptor is available. */

  txdesc = pic32mz_txdesc(priv);
  if (txdesc != NULL)
    {
      /* Yes.. queue the packet now. */

      pic32mz_transmit(priv);
    }
  else
    {
      /* No.. mark the Tx as pending and halt further Rx interrupts */

      DEBUGASSERT((priv->pd_inten & ETH_INT_TXDONE) != 0);

      priv->pd_txpending = true;
      priv->pd_inten    &= ~ETH_RXINTS;
      pic32mz_putreg(priv->pd_inten, PIC32MZ_ETH_IEN);
    }
}

/****************************************************************************
 * Function: pic32mz_rxdone
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

static void pic32mz_rxdone(struct pic32mz_driver_s *priv)
{
  struct pic32mz_rxdesc_s *rxdesc;

  /* Loop while there are incoming packets to be processed, that is, while
   * the producer index is not equal to the consumer index.
   */

  for (; ; )
    {
      /* Check if any RX descriptor has the EOWN bit cleared meaning that the
       * this descriptor is now under software control and a message was
       * received.
       */

      rxdesc = pic32mz_rxdesc(priv);
      if (rxdesc == NULL)
        {
          /* All RX descriptors are owned by the Ethernet controller... we
           * are finished here.
           */

          return;
        }

      pic32mz_dumprxdesc(rxdesc, "RX Complete");

      /* Get the packet length */

      priv->pd_dev.d_len = (rxdesc->rsv2 & RXDESC_RSV2_BYTECOUNT_MASK) >>
                            RXDESC_RSV2_BYTECOUNT_SHIFT;

      /* Update statistics */

      NETDEV_RXPACKETS(&priv->pd_dev);

      /* Check for errors */

      if ((rxdesc->rsv2 & RXDESC_RSV2_OK) == 0)
        {
          nwarn("WARNING. rsv1: %08x rsv2: %08x\n",
                rxdesc->rsv1, rxdesc->rsv2);
          NETDEV_RXERRORS(&priv->pd_dev);
          pic32mz_rxreturn(rxdesc);
        }

      /* If the packet length is greater then the buffer, then we cannot
       * accept the packet.  Also, since the DMA packet buffers are set up to
       * be the same size as our max packet size, any fragments also
       * imply that the packet is too big.
       */

      else if (priv->pd_dev.d_len > CONFIG_NET_ETH_PKTSIZE)
        {
          nwarn("WARNING: Too big. packet length: %d rxdesc: %08x\n",
                priv->pd_dev.d_len, rxdesc->status);
          NETDEV_RXERRORS(&priv->pd_dev);
          pic32mz_rxreturn(rxdesc);
        }

      /* We don't have any logic here for reassembling packets from fragments. */

      else if ((rxdesc->status & (RXDESC_STATUS_EOP | RXDESC_STATUS_SOP)) !=
               (RXDESC_STATUS_EOP | RXDESC_STATUS_SOP))
        {
          nwarn("WARNING: Fragment. packet length: %d rxdesc: %08x\n",
                priv->pd_dev.d_len, rxdesc->status);
          NETDEV_RXFRAGMENTS(&priv->pd_dev);
          pic32mz_rxreturn(rxdesc);
        }
      else
        {
          uint8_t *rxbuffer;

          /* Get the Rx buffer address from the Rx descriptor */

          priv->pd_dev.d_buf = (uint8_t *)VIRT_ADDR(rxdesc->address);

          /* Force the completed RX DMA buffer to be re-read from
           * physical memory.
           */

          up_invalidate_dcache((uintptr_t)priv->pd_dev.d_buf,
                               (uintptr_t)priv->pd_dev.d_buf +
                               PIC32MZ_ALIGNED_BUFSIZE);

          DEBUGASSERT(priv->pd_dev.d_buf != NULL);

          /* Replace the buffer in the RX descriptor with a new one */

          rxbuffer = pic32mz_allocbuffer(priv);
          DEBUGASSERT(rxbuffer != NULL);
          rxdesc->address = PHYS_ADDR(rxbuffer);

          /* And give the RX descriptor back to the hardware */

          pic32mz_rxreturn(rxdesc);
          pic32mz_dumppacket("Received packet",
                             priv->pd_dev.d_buf, priv->pd_dev.d_len);

#ifdef CONFIG_NET_PKT
          /* When packet sockets are enabled, feed the frame into the packet
           * tap.
           */

          pkt_input(&priv->pd_dev);
#endif

          /* We only accept IP packets of the configured type and ARP packets */

#ifdef CONFIG_NET_IPv4
          if (BUF->type == HTONS(ETHTYPE_IP))
            {
              ninfo("IPv4 frame\n");
              NETDEV_RXIPV4(&priv->pd_dev);

              /* Handle ARP on input then give the IPv4 packet to the network
               * layer
               */

              arp_ipin(&priv->pd_dev);
              ipv4_input(&priv->pd_dev);

              /* If the above function invocation resulted in data that
               * should be sent out on the network, the field d_len will
               * set to a value > 0.
               */

              if (priv->pd_dev.d_len > 0)
                {
                  /* Update the Ethernet header with the correct MAC address */

#ifdef CONFIG_NET_IPv6
                  if (IFF_IS_IPv4(priv->pd_dev.d_flags))
#endif
                    {
                      arp_out(&priv->pd_dev);
                    }
#ifdef CONFIG_NET_IPv6
                  else
                    {
                      neighbor_out(&priv->pd_dev);
                    }
#endif

                  /* And send the packet */

                  pic32mz_response(priv);
                }
            }
          else
#endif
#ifdef CONFIG_NET_IPv6
          if (BUF->type == HTONS(ETHTYPE_IP6))
            {
              ninfo("IPv6 frame\n");
              NETDEV_RXIPV6(&priv->pd_dev);

              /* Give the IPv6 packet to the network layer */

              ipv6_input(&priv->pd_dev);

              /* If the above function invocation resulted in data that
               * should be sent out on the network, the field d_len will
               * set to a value > 0.
               */

              if (priv->pd_dev.d_len > 0)
                {
                  /* Update the Ethernet header with the correct MAC address */

#ifdef CONFIG_NET_IPv4
                  if (IFF_IS_IPv4(priv->pd_dev.d_flags))
                    {
                      arp_out(&priv->pd_dev);
                    }
                  else
#endif
#ifdef CONFIG_NET_IPv6
                    {
                      neighbor_out(&priv->pd_dev);
                    }
#endif

                  /* And send the packet */

                  pic32mz_response(priv);
                }
            }
          else
#endif
#ifdef CONFIG_NET_ARP
          if (BUF->type == htons(ETHTYPE_ARP))
            {
              /* Handle the incoming ARP packet */

              NETDEV_RXARP(&priv->pd_dev);
              arp_arpin(&priv->pd_dev);

              /* If the above function invocation resulted in data that
               * should be sent out on the network, the field  d_len will
               * set to a value > 0.
               */

              if (priv->pd_dev.d_len > 0)
                {
                  pic32mz_response(priv);
                }
            }
          else
#endif
            {
              /* Unrecognized... drop it. */

              nwarn("WARNING: Unrecognized packet type dropped: %04x\n",
                    ntohs(BUF->type));
              NETDEV_RXDROPPED(&priv->pd_dev);
            }

          /* Discard any buffers still attached to the device structure */

          priv->pd_dev.d_len = 0;
          if (priv->pd_dev.d_buf)
            {
              pic32mz_freebuffer(priv, priv->pd_dev.d_buf);
              priv->pd_dev.d_buf = NULL;
            }
        }
    }
}

/****************************************************************************
 * Function: pic32mz_txdone
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
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static void pic32mz_txdone(struct pic32mz_driver_s *priv)
{
  struct pic32mz_txdesc_s *txdesc;
  int i;

  /* Cancel the pending Tx timeout */

  wd_cancel(priv->pd_txtimeout);

  /* Disable further Tx interrupts.  Tx interrupts may be re-enabled again
   * depending upon the result of the poll.
   */

  priv->pd_inten &= ~ETH_TXINTS;
  pic32mz_putreg(priv->pd_inten, PIC32MZ_ETH_IEN);

  /* Verify that the hardware is ready to send another packet.  Since a Tx
   * just completed, this must be the case.
   */

  DEBUGASSERT(pic32mz_txdesc(priv) != NULL);

  /* Inspect the list of TX descriptors to see if the EOWN bit is cleared.
   * If it is, this descriptor is now under software control and the message
   * was transmitted. Use TSV to check for the transmission result.
   */

  for (i = 0; i < CONFIG_PIC32MZ_ETH_NTXDESC; i++)
    {
      txdesc = &g_txdesc[i].txdesc;

      /* Force re-reading of the TX descriptor from physical memory.
       * The descriptor may have been owned by the Ethernet controller.
       * Cache data would be stale in this case.
       */

      up_invalidate_dcache((uintptr_t)txdesc,
                           (uintptr_t)txdesc +
                           sizeof(union pic32mz_txdesc_u));

      /* Check if software owns this descriptor */

      if ((txdesc->status & TXDESC_STATUS_EOWN) == 0)
        {
          /* Yes.. Check if there is a buffer attached? */

          if (txdesc->address != 0)
            {
              pic32mz_dumptxdesc(txdesc, "Freeing TX buffer");

              /* Free the TX buffer */

              pic32mz_freebuffer(priv,
                                 (uint8_t *)VIRT_ADDR(txdesc->address));
              txdesc->address = 0;

              /* Reset status */

              txdesc->tsv1    = 0;
              txdesc->tsv2    = 0;
              txdesc->status  = TXDESC_STATUS_SOWN | TXDESC_STATUS_NPV;

              pic32mz_dumptxdesc(txdesc, "TX buffer freed");

              /* Flush the content of the modified TX descriptor. */

              up_flush_dcache((uintptr_t)txdesc,
                              (uintptr_t)txdesc +
                              sizeof(union pic32mz_txdesc_u));
            }
        }
    }

  /* Check if there is a pending Tx transfer that was deferred by Rx handling
   * because there were no available Tx descriptors.  If so, process that
   * pending Tx now.
   */

  if (priv->pd_txpending)
    {
      /* Clear the pending condition, send the packet, and restore Rx interrupts */

      priv->pd_txpending = false;

      pic32mz_transmit(priv);

      priv->pd_inten    |= ETH_RXINTS;
      pic32mz_putreg(priv->pd_inten, PIC32MZ_ETH_IEN);
    }

  /* Otherwise poll the network for new XMIT data */

  else
    {
      /* Perform the network poll */

      pic32mz_poll(priv);
    }
}

/****************************************************************************
 * Function: pic32mz_interrupt_work
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

static void pic32mz_interrupt_work(void *arg)
{
  struct pic32mz_driver_s *priv = (struct pic32mz_driver_s *)arg;
  uint32_t status;

  /* Process pending Ethernet interrupts */

  net_lock();

  /* Get the interrupt status (zero means no interrupts pending). */

  status = pic32mz_getreg(PIC32MZ_ETH_IRQ);
  if (status != 0)
    {
      /* Clear all pending interrupts */

      pic32mz_putreg(status, PIC32MZ_ETH_IRQCLR);

      /* Handle each pending interrupt **************************************/

      /* Receive Errors *****************************************************/

      /* RXOVFLW: Receive FIFO Over Flow Error.  RXOVFLW is set by the RXBM
       * Logic for an RX FIFO Overflow condition. It is cleared by either a
       * Reset or CPU write of a '1' to the CLR register.
       */

      if ((status & ETH_INT_RXOVFLW) != 0)
        {
          nerr("ERROR: RX Overrun. status: %08x\n", status);
          NETDEV_RXERRORS(&priv->pd_dev);
        }

      /* RXBUFNA: Receive Buffer Not Available Interrupt.  This bit is set by
       * a RX Buffer Descriptor Overrun condition. It is cleared by either a
       * Reset or a CPU write of a '1' to the CLR register.
       */

      if ((status & ETH_INT_RXBUFNA) != 0)
        {
          nerr("ERROR: RX buffer descriptor overrun. status: %08x\n",
                status);
          NETDEV_RXERRORS(&priv->pd_dev);
        }

      /* RXBUSE: Receive BVCI Bus Error Interrupt.  This bit is set when the
       * RX DMA encounters a BVCI Bus error during a memory access. It is
       * cleared by either a Reset or CPU write of a '1' to the CLR register.
       */

      if ((status & ETH_INT_RXBUSE) != 0)
        {
          nerr("ERROR: RX BVCI bus error. status: %08x\n", status);
          NETDEV_RXERRORS(&priv->pd_dev);
        }

      /* Receive Normal Events **********************************************/

      /* RXACT: Receive Activity Interrupt.  This bit is set whenever RX
       * packet data is stored in the RXBM FIFO. It is cleared by either
       * a Reset or CPU write of a '1' to the CLR register.
       */

      /* PKTPEND: Packet Pending Interrupt.  This bit is set when the BUFCNT
       * counter has a value other than '0'. It is cleared by either a Reset
       * or by writing the BUFCDEC bit to decrement the BUFCNT counter.
       * Writing a '0' or a '1' has no effect.
       */

      /* RXDONE: Receive Done Interrupt.  This bit is set whenever an RX
       * packet is successfully received. It is cleared by either a Reset
       * or CPU write of a '1' to the CLR register.
       */

      if ((status & ETH_INT_RXDONE) != 0)
        {
          /* We have received at least one new incoming packet. */

          pic32mz_rxdone(priv);
        }

      /* Transmit Errors ****************************************************/

      /* TXABORT: Transmit Abort Condition Interrupt.  This bit is set when
       * the MAC aborts the transmission of a TX packet for one of the
       * following reasons:
       * - Jumbo TX packet abort
       * - Underrun abort
       * - Excessive defer abort
       * - Late collision abort
       * - Excessive collisions abort
       * This bit is cleared by either a Reset or CPU write of a '1' to the
       * CLR register.
       */

      if ((status & ETH_INT_TXABORT) != 0)
        {
          nerr("ERROR: TX abort. status: %08x\n", status);
          NETDEV_TXERRORS(&priv->pd_dev);
        }

      /* TXBUSE: Transmit BVCI Bus Error Interrupt. This bit is set when the
       * TX DMA encounters a BVCI Bus error during a memory access. It is
       * cleared by either a Reset or CPU write of a '1' to the CLR register.
       */

      if ((status & ETH_INT_TXBUSE) != 0)
        {
          nerr("ERROR: TX BVCI bus error. status: %08x\n", status);
          NETDEV_TXERRORS(&priv->pd_dev);
        }

      /* TXDONE: Transmit Done Interrupt.  This bit is set when the currently
       * transmitted TX packet completes transmission, and the Transmit
       * Status Vector is loaded into the first descriptor used for the
       * packet. It is cleared by either a Reset or CPU write of a '1' to
       * the CLR register.
       */

      if ((status & ETH_INT_TXDONE) != 0)
        {
          NETDEV_TXDONE(&priv->pd_dev);

          /* A packet transmission just completed */

          pic32mz_txdone(priv);
        }

      /* Watermark Events ***************************************************/

      /* EWMARK: Empty Watermark Interrupt.  This bit is set when the RX
       * Descriptor Buffer Count is less than or equal to the value in the
       * RXEWM bit (ETHRXWM:0-7) value. It is cleared by BUFCNT bit
       * (ETHSTAT:16-23) being incremented by hardware. Writing a '0' or
       * a '1' has no effect.
       */

      /* FWMARK: Full Watermark Interrupt.  This bit is set when the RX
       * escriptor Buffer Count is greater than or equal to the value in the
       * RXFWM bit (ETHRXWM:16-23) field. It is cleared by writing the
       * BUFCDEC (ETHCON1:0) bit to decrement the BUFCNT counter. Writing a
       * '0' or a '1' has no effect.
       */
    }

  /* Clear the pending interrupt */

#if CONFIG_PIC32MZ_NINTERFACES > 1
  up_clrpend_irq(priv->pd_irqsrc);
#else
  up_clrpend_irq(PIC32MZ_IRQ_ETH);
#endif
  net_unlock();

  /* Re-enable Ethernet interrupts */

#if CONFIG_PIC32MZ_NINTERFACES > 1
  up_enable_irq(priv->pd_irqsrc);
#else
  up_enable_irq(PIC32MZ_IRQ_ETH);
#endif
}

/****************************************************************************
 * Function: pic32mz_interrupt
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

static int pic32mz_interrupt(int irq, void *context, FAR void *arg)
{
  struct pic32mz_driver_s *priv;
  uint32_t status;

#if CONFIG_PIC32MZ_NINTERFACES > 1
# error "A mechanism to associate an interface with an IRQ is needed"
#else
  priv = &g_ethdrvr[0];
#endif

  /* Disable further Ethernet interrupts.  Because Ethernet interrupts are
   * also disabled if the TX timeout event occurs, there can be no race
   * condition here.
   */

#if CONFIG_PIC32MZ_NINTERFACES > 1
  up_disable_irq(priv->pd_irqsrc);
#else
  up_disable_irq(PIC32MZ_IRQ_ETH);
#endif

  /* Get the interrupt status (zero means no interrupts pending). */

  status = pic32mz_getreg(PIC32MZ_ETH_IRQ);

  /* Determine if a TX transfer just completed */

  if ((status & ETH_INT_TXDONE) != 0)
    {
      /* If a TX transfer just completed, then cancel the TX timeout so
       * there will be no race condition between any subsequent timeout
       * expiration and the deferred interrupt processing.
       */

       wd_cancel(priv->pd_txtimeout);
    }

  /* Schedule to perform the interrupt processing on the worker thread. */

  work_queue(ETHWORK, &priv->pd_irqwork, pic32mz_interrupt_work, priv, 0);
  return OK;
}

/****************************************************************************
 * Function: pic32mz_txtimeout_work
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

static void pic32mz_txtimeout_work(void *arg)
{
  struct pic32mz_driver_s *priv = (struct pic32mz_driver_s *)arg;

  /* Increment statistics and dump debug info */

  net_lock();
  NETDEV_TXTIMEOUTS(&priv->pd_dev);
  if (priv->pd_ifup)
    {
      /* Then reset the hardware. ifup() will reset the interface, then bring
       * it back up.
       */

      pic32mz_ifup(&priv->pd_dev);

      /* Then poll the network for new XMIT data (We are guaranteed to have
       * a free buffer here).
       */

      pic32mz_poll(priv);
    }

  net_unlock();
}

/****************************************************************************
 * Function: pic32mz_txtimeout_expiry
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

static void pic32mz_txtimeout_expiry(int argc, wdparm_t arg, ...)
{
  struct pic32mz_driver_s *priv = (struct pic32mz_driver_s *)arg;

  /* Disable further Ethernet interrupts.  This will prevent some race
   * conditions with interrupt work.  There is still a potential race
   * condition with interrupt work that is already queued and in progress.
   */

#if CONFIG_PIC32MZ_NINTERFACES > 1
  up_disable_irq(priv->pd_irqsrc);
#else
  up_disable_irq(PIC32MZ_IRQ_ETH);
#endif

  /* Schedule to perform the TX timeout processing on the worker thread. */

  work_queue(ETHWORK, &priv->pd_irqwork, pic32mz_txtimeout_work, priv, 0);
}

/****************************************************************************
 * Function: pic32mz_poll_work
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

static void pic32mz_poll_work(void *arg)
{
  struct pic32mz_driver_s *priv = (struct pic32mz_driver_s *)arg;

  /* Check if the next Tx descriptor is available.  We cannot perform the Tx
   * poll if we are unable to accept another packet for transmission.
   */

  net_lock();
  if (pic32mz_txdesc(priv) != NULL)
    {
      /* If so, update TCP timing states and poll the network for new XMIT
       * data. Hmmm.. might be bug here.  Does this mean if there is a
       * transmit in progress, we will missing TCP time state updates?
       */

      pic32mz_timerpoll(priv);
    }

  /* Setup the watchdog poll timer again */

  wd_start(priv->pd_txpoll, PIC32MZ_WDDELAY, pic32mz_poll_expiry,
           1, priv);
  net_unlock();
}

/****************************************************************************
 * Function: pic32mz_poll_expiry
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

static void pic32mz_poll_expiry(int argc, wdparm_t arg, ...)
{
  struct pic32mz_driver_s *priv = (struct pic32mz_driver_s *)arg;

  /* Schedule to perform the interrupt processing on the worker thread. */

  work_queue(ETHWORK, &priv->pd_pollwork, pic32mz_poll_work, priv, 0);
}

/****************************************************************************
 * Function: pic32mz_ifup
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

static int pic32mz_ifup(struct net_driver_s *dev)
{
  struct pic32mz_driver_s *priv = (struct pic32mz_driver_s *)dev->d_private;
  uint32_t regval;
  int ret;

  ninfo("Bringing up: %d.%d.%d.%d\n",
        dev->d_ipaddr & 0xff, (dev->d_ipaddr >> 8) & 0xff,
        (dev->d_ipaddr >> 16) & 0xff, dev->d_ipaddr >> 24);

  /* Reset the Ethernet controller (again) */

  pic32mz_ethreset(priv);

  /* MAC Initialization *****************************************************/

  /* Configuration:
   * - Use the configuration fuse setting FETHIO bit (DEVCFG3:25) to detect
   *   the alternate/default I/O configuration
   * - Use the configuration fuse setting FMIIEN (DEVCFG3:24) to detect the
   *   MII/RMII operation mode.
   */

  /* Pin Configuration:
   *
   * No GPIO pin configuration is required.  Enabling the Ethernet Controller
   * will configure the IO pin direction as defined by the Ethernet
   * Controller control bits. The port TRIS and LATCH registers will be
   * overridden.
   *
   * I/O Pin    MII     RMII   Pin  Description
   *   Name   Required Required Type
   * EMDC     Yes      Yes      O    Ethernet MII Management Clock
   * EMDIO    Yes      Yes      I/O  Ethernet MII Management IO
   * ETXCLK   Yes      No       I    Ethernet MII TX Clock
   * ETXEN    Yes      Yes      O    Ethernet Transmit Enable
   * ETXD0    Yes      Yes      O    Ethernet Data Transmit 0
   * ETXD1    Yes      Yes      O    Ethernet Data Transmit 1
   * ETXD2    Yes      No       O    Ethernet Data Transmit 2
   * ETXD3    Yes      No       O    Ethernet Data Transmit 3
   * ETXERR   Yes      No       O    Ethernet Transmit Error
   * ERXCLK   Yes      No       I    Ethernet MII RX Clock
   * EREF_CLK No       Yes      I    Ethernet RMII Ref Clock
   * ERXDV    Yes      No       I    Ethernet MII Receive Data Valid
   * ECRS_DV  No       Yes      I    Ethernet RMII Carrier Sense/Receive
   *                                 Data Valid
   * ERXD0    Yes      Yes      I    Ethernet Data Receive 0
   * ERXD1    Yes      Yes      I    Ethernet Data Receive 1
   * ERXD2    Yes      No       I    Ethernet Data Receive 2
   * ERXD3    Yes      No       I    Ethernet Data Receive 3
   * ERXERR   Yes      Yes      I    Ethernet Receive Error
   * ECRS     Yes      No       I    Ethernet Carrier Sense
   * ECOL     Yes      No       I    Ethernet Collision Detected
   *
   * All that is required is to assure that the pins are initialized as
   * digital (normally only those pins that have shared analog functionality
   * need to be configured).
   */

  /* Initialize the MIIM interface
   *
   * If the RMII operation is selected, reset the RMII module by using the
   * RESETRMII (EMAC1SUPP:11) bit and set the proper speed in the SPEEDRMII
   * bit (EMAC1SUPP:8) bit.
   */

#if CONFIG_PIC32MZ_FMIIEN == 0
  pic32mz_putreg(EMAC1_SUPP_RESETRMII, PIC32MZ_EMAC1_SUPPSET);
  pic32mz_putreg((EMAC1_SUPP_RESETRMII | EMAC1_SUPP_SPEEDRMII),
                 PIC32MZ_EMAC1_SUPPCLR);
#endif

  /* Issue an MIIM block reset, by setting the RESETMGMT (EMAC1MCFG:15) bit,
   * and then clear the reset bit.
   */

  regval = pic32mz_getreg(PIC32MZ_EMAC1_MCFG);
  pic32mz_putreg(EMAC1_MCFG_MGMTRST, PIC32MZ_EMAC1_MCFGSET);

  regval &= ~EMAC1_MCFG_MGMTRST;
  pic32mz_putreg(regval, PIC32MZ_EMAC1_MCFG);

  /* Select a proper divider in the CLKSEL bit (EMAC1CFG:2-5) for the MIIM
   * PHY communication based on the system running clock frequency and the
   * external PHY supported clock.
   *
   * MII configuration: host clocked divider per board.h, no suppress
   * preamble, no scan increment.
   */

  regval &= ~(EMAC1_MCFG_CLKSEL_MASK | EMAC1_MCFG_NOPRE |
              EMAC1_MCFG_SCANINC);
  regval |= EMAC1_MCFG_CLKSEL_DIV;
  pic32mz_putreg(regval, PIC32MZ_EMAC1_MCFG);

  /* PHY Initialization *****************************************************/

  /* Initialize the PHY and wait for the link to be established */

  ret = pic32mz_phyinit(priv);
  if (ret != 0)
    {
      nerr("ERROR: pic32mz_phyinit failed: %d\n", ret);
      return ret;
    }

  /* MAC Configuration ******************************************************/

  /* Set other misc configuration-related registers to default values */

  pic32mz_putreg(0, PIC32MZ_EMAC1_CFG2);
  pic32mz_putreg(0, PIC32MZ_EMAC1_TEST);

  /* Having available the Duplex and Speed settings, configure the MAC
   * accordingly, using the following steps:
   *
   * Enable the RXENABLE bit (EMAC1CFG1:0), selecting both the TXPAUSE and
   * RXPAUSE bit (EMAC1CFG1:2-3) (the PIC32 MAC supports both).
   */

  pic32mz_putreg(EMAC1_CFG1_RXEN | EMAC1_CFG1_RXPAUSE | EMAC1_CFG1_TXPAUSE,
                 PIC32MZ_EMAC1_CFG1SET);

  /* Select the desired auto-padding and CRC capabilities, and the enabling
   * of the huge frames and the Duplex type in the EMAC1CFG2 register.
   * (This was done in the PHY initialization logic).
   */

  /* Program EMAC1IPGT with the back-to-back inter-packet gap */

  /* Use EMAC1IPGR for setting the non back-to-back inter-packet gap */

  pic32mz_putreg(((12 << EMAC1_IPGR_GAP1_SHIFT) |
                  (12 << EMAC1_IPGR_GAP2_SHIFT)),
                 PIC32MZ_EMAC1_IPGR);

  /* Set the collision window and the maximum number of retransmissions in
   * EMAC1CLRT.
   */

  pic32mz_putreg(((15 << EMAC1_CLRT_RETX_SHIFT) |
                  (55 << EMAC1_CLRT_CWINDOW_SHIFT)),
                 PIC32MZ_EMAC1_CLRT);

  /* Set the maximum frame length in EMAC1MAXF.  "This field resets to
   * 0x05EE, which represents a maximum receive frame of 1518 octets. An
   * untagged maximum size Ethernet frame is 1518 octets. A tagged frame adds
   * four octets for a total of 1522 octets. If a shorter/longer maximum
   * length restriction is desired, program this 16-bit field.
   */

  pic32mz_putreg(CONFIG_NET_ETH_PKTSIZE, PIC32MZ_EMAC1_MAXF);

  /* Configure the MAC station address in the EMAC1SA0, EMAC1SA1 and
   * EMAC1SA2 registers (these registers are loaded at reset from the
   * factory preprogrammed station address).
   */

#if 0
  regval = (uint32_t)priv->pd_dev.d_mac.ether.ether_addr_octet[5] << 8 |
           (uint32_t)priv->pd_dev.d_mac.ether.ether_addr_octet[4];
  pic32mz_putreg(regval, PIC32MZ_EMAC1_SA0);

  regval = (uint32_t)priv->pd_dev.d_mac.ether.ether_addr_octet[3] << 8 |
           (uint32_t)priv->pd_dev.d_mac.ether.ether_addr_octet[2];
  pic32mz_putreg(regval, PIC32MZ_EMAC1_SA1);

  regval = (uint32_t)priv->pd_dev.d_mac.ether.ether_addr_octet[1] << 8 |
           (uint32_t)priv->pd_dev.d_mac.ether.ether_addr_octet[0];
  pic32mz_putreg(regval, PIC32MZ_EMAC1_SA2);
#else
  regval = pic32mz_getreg(PIC32MZ_EMAC1_SA0);
  priv->pd_dev.d_mac.ether.ether_addr_octet[4] = (uint32_t)(regval & 0xff);
  priv->pd_dev.d_mac.ether.ether_addr_octet[5] = (uint32_t)((regval >> 8) &
                                                             0xff);

  regval = pic32mz_getreg(PIC32MZ_EMAC1_SA1);
  priv->pd_dev.d_mac.ether.ether_addr_octet[2] = (uint32_t)(regval & 0xff);
  priv->pd_dev.d_mac.ether.ether_addr_octet[3] = (uint32_t)((regval >> 8) &
                                                             0xff);

  regval = pic32mz_getreg(PIC32MZ_EMAC1_SA2);
  priv->pd_dev.d_mac.ether.ether_addr_octet[0] = (uint32_t)(regval & 0xff);
  priv->pd_dev.d_mac.ether.ether_addr_octet[1] = (uint32_t)((regval >> 8) &
                                                             0xff);

  ninfo("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
        dev->d_mac.ether.ether_addr_octet[0],
        dev->d_mac.ether.ether_addr_octet[1],
        dev->d_mac.ether.ether_addr_octet[2],
        dev->d_mac.ether.ether_addr_octet[3],
        dev->d_mac.ether.ether_addr_octet[4],
        dev->d_mac.ether.ether_addr_octet[5]);
#endif

  /* Continue Ethernet Controller Initialization ****************************/

  /* If planning to turn on the flow control, update the PTV value
   * (ETHCON1:16-31).
   */

  /* If using the auto-flow control, set the full and empty watermarks: RXFWM
   * and RXEWM (ETHRXWM:16-23 and ETHRXWM:0-7).
   */

  /* If needed, enable the auto-flow control by setting AUTOFC (ETHCON1:7). */

  /* Set the RX filters by updating the ETHHT0, ETHHT1, ETHPMM0, ETHPMM1,
   * ETHPMCS and ETHRXFC registers.
   *
   * Set up RX filter and configure to accept broadcast addresses and
   * multicast addresses (if so configured).  NOTE: There is a selection
   * CONFIG_NET_BROADCAST, but this enables receipt of UDP broadcast packets
   * inside of the stack.
   */

  regval  = ETH_RXFC_BCEN | ETH_RXFC_UCEN | ETH_RXFC_PMMODE_DISABLED;
#ifdef CONFIG_PIC32MZ_MULTICAST
  regval |= ETH_RXFC_MCEN;
#endif
  pic32mz_putreg(regval, PIC32MZ_ETH_RXFC);

  /* Set the size of the RX buffers in the RXBUFSZ bit (ETHCON2:4-10) (all
   * receive descriptors use the same buffer size). Keep in mind that using
   * packets that are too small leads to packet fragmentation and has a
   * noticeable impact on the performance.
   */

  pic32mz_putreg(ETH_CON2_RXBUFSZ(CONFIG_NET_ETH_PKTSIZE), PIC32MZ_ETH_CON2);

  /* Reset state variables */

  priv->pd_polling   = false;
  priv->pd_txpending = false;

  /* Initialize the buffer list */

  pic32mz_bufferinit(priv);

  /* Initialize the TX descriptor list */

  pic32mz_txdescinit(priv);

  /* Initialize the RX descriptor list */

  pic32mz_rxdescinit(priv);

  /* Enable the Ethernet Controller by setting the ON bit (ETHCON1:15).
   * Enable the receiving of messages by setting the RXEN bit (ETHCON1:8).
   */

  pic32mz_putreg(ETH_CON1_RXEN | ETH_CON1_ON, PIC32MZ_ETH_CON1SET);

  /* Initialize Ethernet interface for the PHY setup */

  pic32mz_macmode(priv->pd_mode);

  /* Configure to pass all received frames */

  regval  = pic32mz_getreg(PIC32MZ_EMAC1_CFG1);
  regval |= EMAC1_CFG1_PASSALL;
  pic32mz_putreg(regval, PIC32MZ_EMAC1_CFG1);

  /* Clear any pending interrupts (shouldn't be any) */

  pic32mz_putreg(0xffffffff, PIC32MZ_ETH_IRQCLR);

  /* Configure interrupts.  The Ethernet interrupt was attached during
   * one-time initialization, so we only need to set the interrupt priority,
   * configure interrupts, and enable them.
   */

  /* If the user provided an interrupt priority, then set the interrupt
   * to that priority.
   */

#if defined(CONFIG_PIC32MZ_ETH_PRIORITY) && defined(CONFIG_ARCH_IRQPRIO)
#if CONFIG_PIC32MZ_NINTERFACES > 1
  up_prioritize_irq(priv->pd_irq, CONFIG_PIC32MZ_ETH_PRIORITY);
#else
  up_prioritize_irq(PIC32MZ_IRQ_ETH, CONFIG_PIC32MZ_ETH_PRIORITY);
#endif
#endif

  /* Otherwise, enable all Rx interrupts.  Tx interrupts, SOFTINT and WoL are
   * excluded.  Tx interrupts will not be enabled until there is data to be
   * sent.
   */

  priv->pd_inten = ETH_RXINTS;
  pic32mz_putreg(ETH_RXINTS, PIC32MZ_ETH_IENSET);

  /* Set and activate a timer process */

  wd_start(priv->pd_txpoll, PIC32MZ_WDDELAY, pic32mz_poll_expiry, 1,
           (uint32_t)priv);

  /* Finally, enable the Ethernet interrupt at the interrupt controller */

  priv->pd_ifup = true;

#if CONFIG_PIC32MZ_NINTERFACES > 1
  up_enable_irq(priv->pd_irqsrc);
#else
  up_enable_irq(PIC32MZ_IRQ_ETH);
#endif

  return OK;
}

/****************************************************************************
 * Function: pic32mz_ifdown
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

static int pic32mz_ifdown(struct net_driver_s *dev)
{
  struct pic32mz_driver_s *priv = (struct pic32mz_driver_s *)dev->d_private;
  irqstate_t flags;

  /* Disable the Ethernet interrupt */

  flags = enter_critical_section();
#if CONFIG_PIC32MZ_NINTERFACES > 1
  up_disable_irq(priv->pd_irqsrc);
#else
  up_disable_irq(PIC32MZ_IRQ_ETH);
#endif

  /* Cancel the TX poll timer and TX timeout timers */

  wd_cancel(priv->pd_txpoll);
  wd_cancel(priv->pd_txtimeout);

  /* Reset the device and mark it as down. */

  pic32mz_ethreset(priv);
  priv->pd_ifup = false;
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Function: pic32mz_txavail_work
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

static void pic32mz_txavail_work(void *arg)
{
  struct pic32mz_driver_s *priv = (struct pic32mz_driver_s *)arg;

  /* Ignore the notification if the interface is not yet up */

  net_lock();
  if (priv->pd_ifup)
    {
      /* Check if the next Tx descriptor is available. */

      if (pic32mz_txdesc(priv) != NULL)
        {
          /* If so, then poll the network for new XMIT data.
           * First allocate a buffer to perform the poll
           */

          pic32mz_poll(priv);
        }
    }

  net_unlock();
}

/****************************************************************************
 * Function: pic32mz_txavail
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

static int pic32mz_txavail(struct net_driver_s *dev)
{
  struct pic32mz_driver_s *priv = (struct pic32mz_driver_s *)dev->d_private;

  /* Is our single work structure available?  It may not be if there are
   * pending interrupt actions and we will have to ignore the Tx
   * availability action.
   */

  if (work_available(&priv->pd_pollwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      work_queue(ETHWORK, &priv->pd_pollwork, pic32mz_txavail_work, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Function: pic32mz_addmac
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
static int pic32mz_addmac(struct net_driver_s *dev, const uint8_t *mac)
{
  struct pic32mz_driver_s *priv = (struct pic32mz_driver_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

#warning "Not implemented"
  return OK;
}
#endif

/****************************************************************************
 * Function: pic32mz_rmmac
 *
 * Description:
 *   NuttX Callback: Remove the specified MAC address from the hardware
 *   multicast address filtering.
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
static int pic32mz_rmmac(struct net_driver_s *dev, const uint8_t *mac)
{
  struct pic32mz_driver_s *priv = (struct pic32mz_driver_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

#warning "Not implemented"
  return OK;
}
#endif

/****************************************************************************
 * Name: pic32mz_showmii
 *
 * Description:
 *   Dump PHY MII registers
 *
 * Input Parameters:
 *   phyaddr - The device address where the PHY was discovered
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#if defined(CONFIG_NET_REGDEBUG) && defined(PIC32MZ_HAVE_PHY)
static void pic32mz_showmii(uint8_t phyaddr, const char *msg)
{
  ninfo("PHY " PIC32MZ_PHYNAME ": %s\n", msg);
  ninfo("  MCR:       %04x\n", pic32mz_phyread(phyaddr, MII_MCR));
  ninfo("  MSR:       %04x\n", pic32mz_phyread(phyaddr, MII_MSR));
  ninfo("  ADVERTISE: %04x\n", pic32mz_phyread(phyaddr, MII_ADVERTISE));
  ninfo("  LPA:       %04x\n", pic32mz_phyread(phyaddr, MII_LPA));
  ninfo("  EXPANSION: %04x\n", pic32mz_phyread(phyaddr, MII_EXPANSION));
#ifdef CONFIG_ETH0_PHY_KS8721
  ninfo("  10BTCR:    %04x\n", pic32mz_phyread(phyaddr, MII_KS8721_10BTCR));
#endif
}
#endif

/****************************************************************************
 * Function: pic32mz_phybusywait
 *
 * Description:
 *   Wait until the PHY is no longer busy
 *
 * Input Parameters:
 *  None
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void pic32mz_phybusywait(void)
{
  while ((pic32mz_getreg(PIC32MZ_EMAC1_MIND) & EMAC1_MIND_MIIMBUSY) != 0);
}

/****************************************************************************
 * Function: pic32mz_phywrite
 *
 * Description:
 *   Write a value to an MII PHY register
 *
 * Input Parameters:
 *   phyaddr - The device address where the PHY was discovered
 *   regaddr - The address of the PHY register to be written
 *   phydata - The data to write to the PHY register
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef PIC32MZ_HAVE_PHY
static void pic32mz_phywrite(uint8_t phyaddr, uint8_t regaddr,
                             uint16_t phydata)
{
  uint32_t regval;

  /* Make sure that the PHY is not still busy from the last command */

  pic32mz_phybusywait();

  /* Set PHY address and PHY register address */

  regval = ((uint32_t)phyaddr << EMAC1_MADR_PHYADDR_SHIFT) |
           ((uint32_t)regaddr << EMAC1_MADR_REGADDR_SHIFT);
  pic32mz_putreg(regval, PIC32MZ_EMAC1_MADR);

  /* Write the register data to the PHY */

  pic32mz_putreg((uint32_t)phydata, PIC32MZ_EMAC1_MWTD);

  /* Sixteen clock cycles until busy is set from the write operation */

  __asm__ __volatile__
    (
      "nop; nop; nop; nop;"
      "nop; nop; nop; nop;"
      "nop; nop; nop; nop;"
      "nop; nop; nop; nop;"
    );
}
#endif

/****************************************************************************
 * Function: pic32mz_phyread
 *
 * Description:
 *   Read a value from an MII PHY register
 *
 * Input Parameters:
 *   phyaddr - The device address where the PHY was discovered
 *   regaddr - The address of the PHY register to be written
 *
 * Returned Value:
 *   Data read from the PHY register
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef PIC32MZ_HAVE_PHY
static uint16_t pic32mz_phyread(uint8_t phyaddr, uint8_t regaddr)
{
  uint32_t regval;

  /* Make sure that the PHY is not still busy from the last command */

  pic32mz_phybusywait();

  /* Set PHY address and PHY register address */

  regval = ((uint32_t)phyaddr << EMAC1_MADR_PHYADDR_SHIFT) |
           ((uint32_t)regaddr << EMAC1_MADR_REGADDR_SHIFT);
  pic32mz_putreg(regval, PIC32MZ_EMAC1_MADR);

  /* Set up to read */

  pic32mz_putreg(EMAC1_MCMD_READ, PIC32MZ_EMAC1_MCMDSET);

  /* Sixteen clock cycles until busy is set from the write operation */

  __asm__ __volatile__
    (
      "nop; nop; nop; nop;"
      "nop; nop; nop; nop;"
      "nop; nop; nop; nop;"
      "nop; nop; nop; nop;"
    );

  /* Wait for the PHY command to complete */

  pic32mz_phybusywait();

  /* Clear the read bit */

  pic32mz_putreg(EMAC1_MCMD_READ, PIC32MZ_EMAC1_MCMDCLR);

  /* Return the PHY register data */

  return (uint16_t)(pic32mz_getreg(PIC32MZ_EMAC1_MRDD) & EMAC1_MRDD_MASK);
}
#endif

/****************************************************************************
 * Function: pic32mz_phyreset
 *
 * Description:
 *   Reset the PHY
 *
 * Input Parameters:
 *   phyaddr - The device address where the PHY was discovered
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef PIC32MZ_HAVE_PHY
static inline int pic32mz_phyreset(uint8_t phyaddr)
{
  int32_t  timeout;
  uint16_t phyreg;

  /* Reset the PHY.  Needs a minimal 50uS delay after reset. */

  pic32mz_phywrite(phyaddr, MII_MCR, MII_MCR_RESET);

  /* Wait for a minimum of 50uS no matter what */

  up_udelay(50);

  /* The MCR reset bit is self-clearing.  Wait for it to be clear indicating
   * that the reset is complete.
   */

  for (timeout = PIC32MZ_MIITIMEOUT; timeout > 0; timeout--)
    {
      phyreg = pic32mz_phyread(phyaddr, MII_MCR);
      if ((phyreg & MII_MCR_RESET) == 0)
        {
          return OK;
        }
    }

  nerr("ERROR: Reset failed. MCR: %04x\n", phyreg);
  return -ETIMEDOUT;
}
#endif

/****************************************************************************
 * Function: pic32mz_phyautoneg
 *
 * Description:
 *   Enable auto-negotiation.
 *
 * Input Parameters:
 *   phyaddr - The device address where the PHY was discovered
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The adverisement register has already been configured.
 *
 ****************************************************************************/

#if defined(PIC32MZ_HAVE_PHY) && defined(CONFIG_PIC32MZ_PHY_AUTONEG)
static inline int pic32mz_phyautoneg(uint8_t phyaddr)
{
  int32_t timeout;
  uint16_t phyreg;

  /* Start auto-negotiation */

  pic32mz_phywrite(phyaddr, MII_MCR, MII_MCR_ANENABLE | MII_MCR_ANRESTART);

  /* Wait for autonegotiation to complete */

  for (timeout = PIC32MZ_MIITIMEOUT; timeout > 0; timeout--)
    {
      /* Check if auto-negotiation has completed */

      phyreg = pic32mz_phyread(phyaddr, MII_MSR);
      if ((phyreg & MII_MSR_ANEGCOMPLETE) != 0)
        {
          /* Yes.. return success */

          return OK;
        }
    }

  nerr("ERROR: Auto-negotiation failed. MSR: %04x\n", phyreg);
  return -ETIMEDOUT;
}
#endif

/****************************************************************************
 * Function: pic32mz_phymode
 *
 * Description:
 *   Set the PHY to operate at a selected speed/duplex mode.
 *
 * Input Parameters:
 *   phyaddr - The device address where the PHY was discovered
 *   mode - speed/duplex mode
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef PIC32MZ_HAVE_PHY
static int pic32mz_phymode(uint8_t phyaddr, uint8_t mode)
{
  int32_t timeout;
  uint16_t phyreg;

  /* Disable auto-negotiation and set fixed Speed and Duplex settings:
   *
   *   MII_MCR_UNIDIR      0=Disable unidirectional enable
   *   MII_MCR_SPEED1000   0=Reserved on 10/100
   *   MII_MCR_CTST        0=Disable collision test
   *   MII_MCR_FULLDPLX    ?=Full duplex
   *   MII_MCR_ANRESTART   0=Don't restart auto negotiation
   *   MII_MCR_ISOLATE     0=Don't electronically isolate PHY from MII
   *   MII_MCR_PDOWN       0=Don't powerdown the PHY
   *   MII_MCR_ANENABLE    0=Disable auto negotiation
   *   MII_MCR_SPEED100    ?=Select 100Mbps
   *   MII_MCR_LOOPBACK    0=Disable loopback mode
   *   MII_MCR_RESET       0=No PHY reset
   */

  phyreg = 0;
  if ((mode & PIC32MZ_SPEED_MASK) ==  PIC32MZ_SPEED_100)
    {
      phyreg = MII_MCR_SPEED100;
    }

  if ((mode & PIC32MZ_DUPLEX_MASK) == PIC32MZ_DUPLEX_FULL)
    {
      phyreg |= MII_MCR_FULLDPLX;
    }

  pic32mz_phywrite(phyaddr, MII_MCR, phyreg);

  /* Then wait for the link to be established */

  for (timeout = PIC32MZ_MIITIMEOUT; timeout > 0; timeout--)
    {
#ifdef CONFIG_ETH0_PHY_DP83848C
      phyreg = pic32mz_phyread(phyaddr, MII_DP83848C_STS);
      if ((phyreg & 0x0001) != 0)
        {
          /* Yes.. return success */

          return OK;
        }
#else
      phyreg = pic32mz_phyread(phyaddr, MII_MSR);
      if ((phyreg & MII_MSR_LINKSTATUS) != 0)
        {
          /* Yes.. return success */

          return OK;
        }
#endif
    }

  nerr("ERROR: Link failed. MSR: %04x\n", phyreg);
  return -ETIMEDOUT;
}
#endif

/****************************************************************************
 * Function: pic32mz_phyinit
 *
 * Description:
 *   Initialize the PHY
 *
 * Input Parameters:
 *   priv - Pointer to EMAC device driver structure
 *
 * Returned Value:
 *   None directly.  As a side-effect, it will initialize priv->pd_phyaddr
 *   and priv->pd_phymode.
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef PIC32MZ_HAVE_PHY
static inline int pic32mz_phyinit(struct pic32mz_driver_s *priv)
{
  unsigned int phyaddr;
  uint16_t phyreg;
  uint32_t regval;
  int ret;

#if CONFIG_PIC32MZ_FMIIEN == 0
  /* Set the RMII operation mode. This usually requires access to a vendor
   * specific control register.
   */

#ifdef CONFIG_ETH0_PHY_DP83848C
  /* The RMII/MII of operation can be selected by strap options or register
   * control (using the RBR register). For RMII mode, it is required to use
   * the strap option, since it requires a 50 MHz clock instead of the normal
   * 25 Mhz.
   */

#endif

#else
  /* Set the MII/ operation mode. This usually requires access to a vendor-
   * specific control register.
   */

#ifdef CONFIG_ETH0_PHY_DP83848C
#  warning "Missing logic"
#endif

#endif

  /* Find PHY Address.  Because the controller has a pull-up and the
   * PHY has pull-down resistors on RXD lines some times the PHY
   * latches different at different addresses.
   */

  for (phyaddr = 0; phyaddr < 32; phyaddr++)
    {
      /* Clear any ongoing PHY command bits */

      pic32mz_putreg(0, PIC32MZ_EMAC1_MCMD);

      /* Reset the PHY (use Control Register 0). */

      ret = pic32mz_phyreset(phyaddr);
      if (ret < 0)
        {
          nerr("ERROR: Failed to reset PHY at address %d\n", phyaddr);
          continue;
        }

      /* Set the normal, swapped or auto (preferred) MDIX. This usually
       * requires access to a vendor-specific control register.
       */

      /* Check if we can see the selected device ID at this
       * PHY address.
       */

      phyreg = (unsigned int)pic32mz_phyread(phyaddr, MII_PHYID1);
      ninfo("Addr: %d PHY ID1: %04x EXPECT: %04x\n",
            phyaddr, phyreg, PIC32MZ_PHYID1);

      if (phyreg == PIC32MZ_PHYID1)
        {
          phyreg = pic32mz_phyread(phyaddr, MII_PHYID2);
          ninfo("Addr: %d PHY ID2: %04x EXPECT: %04x\n",
                phyaddr, phyreg, PIC32MZ_PHYID2);

          if (phyreg == PIC32MZ_PHYID2)
            {
              break;
            }
        }
    }

  /* Check if the PHY device address was found */

  if (phyaddr > 31)
    {
      /* Failed to find PHY at any location */

      nerr("ERROR: No PHY detected\n");
      return -ENODEV;
    }

  ninfo("phyaddr: %d\n", phyaddr);

  /* Save the discovered PHY device address */

  priv->pd_phyaddr = phyaddr;

  /* Reset the PHY */

  ret = pic32mz_phyreset(phyaddr);
  if (ret < 0)
    {
      return ret;
    }

  pic32mz_showmii(phyaddr, "After reset");

  /* Set the MII/RMII operation mode. This usually requires access to a
   * vendor-specific control register.
   */

  /* Set the normal, swapped or auto (preferred) MDIX. This usually requires
   * access to a vendor-specific control register.
   */

  /* Check the PHY capabilities by investigating the Status Register 1. */

  /* Check for preamble suppression support */

  phyreg = pic32mz_phyread(phyaddr, MII_MSR);
  if ((phyreg & MII_MSR_MFRAMESUPPRESS) != 0)
    {
      /* The PHY supports preamble suppression */

      regval  = pic32mz_getreg(PIC32MZ_EMAC1_MCFG);
      regval |= EMAC1_MCFG_NOPRE;
      pic32mz_putreg(regval, PIC32MZ_EMAC1_MCFG);
    }

  /* Are we configured to do auto-negotiation?
   *
   * Preferably the auto-negotiation should be selected if the PHY supports
   * it. Expose the supported capabilities: Half/Full Duplex, 10BaseT/100Base
   * TX, etc. (Extended Register 4). Start the negotiation (Control Register
   * 0) and wait for the negotiation complete and get the link partner
   * capabilities (Extended Register 5) and negotiation result (vendor-
   * specific register).
   */

#ifdef CONFIG_PIC32MZ_PHY_AUTONEG
  /* Setup the Auto-negotiation advertisement: 100 or 10, and HD or FD */

  pic32mz_phywrite(phyaddr, MII_ADVERTISE,
                 (MII_ADVERTISE_100BASETXFULL | MII_ADVERTISE_100BASETXHALF |
                  MII_ADVERTISE_10BASETXFULL  | MII_ADVERTISE_10BASETXHALF  |
                  MII_ADVERTISE_CSMA));

  /* Then perform the auto-negotiation */

  ret = pic32mz_phyautoneg(phyaddr);
  if (ret < 0)
    {
      return ret;
    }
#else
  /* Set up the fixed PHY configuration
   *
   * If auto-negotiation is not supported/selected, update the PHY Duplex and
   * Speed settings directly (use Control Register 0 and possibly some
   * vendor-specific registers).
   */

  ret = pic32mz_phymode(phyaddr, PIC32MZ_MODE_DEFLT);
  if (ret < 0)
    {
      return ret;
    }
#endif

  /* The link is established */

  pic32mz_showmii(phyaddr, "After link established");

  /* Check configuration */

#if defined(CONFIG_ETH0_PHY_KS8721)
  phyreg = pic32mz_phyread(phyaddr, MII_KS8721_10BTCR);

  switch (phyreg & KS8721_10BTCR_MODE_MASK)
    {
      case KS8721_10BTCR_MODE_10BTHD:  /* 10BASE-T half duplex */
        priv->pd_mode = PIC32MZ_10BASET_HD;
        break;
      case KS8721_10BTCR_MODE_100BTHD: /* 100BASE-T half duplex */
        priv->pd_mode = PIC32MZ_100BASET_HD;
        break;
      case KS8721_10BTCR_MODE_10BTFD: /* 10BASE-T full duplex */
        priv->pd_mode = PIC32MZ_10BASET_FD;
        break;
      case KS8721_10BTCR_MODE_100BTFD: /* 100BASE-T full duplex */
        priv->pd_mode = PIC32MZ_100BASET_FD;
        break;
      default:
        nerr("ERROR: Unrecognized mode: %04x\n", phyreg);
        return -ENODEV;
    }
#elif defined(CONFIG_ETH0_PHY_DP83848C)
  phyreg = pic32mz_phyread(phyaddr, MII_DP83848C_STS);

  /* Configure for full/half duplex mode and speed */

  switch (phyreg & 0x0006)
    {
      case 0x0000:
        priv->pd_mode = PIC32MZ_100BASET_HD;
        break;
      case 0x0002:
        priv->pd_mode = PIC32MZ_10BASET_HD;
        break;
      case 0x0004:
        priv->pd_mode = PIC32MZ_100BASET_FD;
        break;
      case 0x0006:
        priv->pd_mode = PIC32MZ_10BASET_FD;
        break;
      default:
        nerr("ERROR: Unrecognized mode: %04x\n", phyreg);
        return -ENODEV;
    }
#elif defined(CONFIG_ETH0_PHY_LAN8720) || defined(CONFIG_ETH0_PHY_LAN8740) ||\
      defined(CONFIG_ETH0_PHY_LAN8740A)
  {
    uint16_t advertise;
    uint16_t lpa;

    up_udelay(500);
    advertise = pic32mz_phyread(phyaddr, MII_ADVERTISE);
    lpa       = pic32mz_phyread(phyaddr, MII_LPA);

    /* Check for 100BASETX full duplex */

    if ((advertise & MII_ADVERTISE_100BASETXFULL) != 0 &&
        (lpa & MII_LPA_100BASETXFULL) != 0)
      {
        priv->pd_mode = PIC32MZ_100BASET_FD;
      }

    /* Check for 100BASETX half duplex */

    else if ((advertise & MII_ADVERTISE_100BASETXHALF) != 0 &&
             (lpa & MII_LPA_100BASETXHALF) != 0)
      {
        priv->pd_mode = PIC32MZ_100BASET_HD;
      }

    /* Check for 10BASETX full duplex */

    else if ((advertise & MII_ADVERTISE_10BASETXFULL) != 0 &&
             (lpa & MII_LPA_10BASETXFULL) != 0)
      {
        priv->pd_mode = PIC32MZ_10BASET_FD;
      }

    /* Check for 10BASETX half duplex */

    else if ((advertise & MII_ADVERTISE_10BASETXHALF) != 0 &&
             (lpa & MII_LPA_10BASETXHALF) != 0)
      {
        priv->pd_mode = PIC32MZ_10BASET_HD;
      }
    else
      {
        nerr("ERROR: Unrecognized mode: %04x\n", phyreg);
        return -ENODEV;
      }
  }
#else
#  warning "PHY Unknown: speed and duplex are bogus"
#endif

  ninfo("%dBase-T %s duplex\n",
        (priv->pd_mode & PIC32MZ_SPEED_MASK) ==
          PIC32MZ_SPEED_100 ? 100 : 10,
        (priv->pd_mode & PIC32MZ_DUPLEX_MASK) ==
          PIC32MZ_DUPLEX_FULL ?"full" : "half");

  /* Disable auto-configuration.  Set the fixed speed/duplex mode.
   * (probably more than little redundant).
   */

  ret = pic32mz_phymode(phyaddr, priv->pd_mode);
  pic32mz_showmii(phyaddr, "After final configuration");
  return ret;
}
#else
static inline int pic32mz_phyinit(struct pic32mz_driver_s *priv)
{
  priv->pd_mode = PIC32MZ_MODE_DEFLT;
  return OK;
}
#endif

/****************************************************************************
 * Function: pic32mz_macmode
 *
 * Description:
 *   Set the MAC to operate at a selected speed/duplex mode.
 *
 * Input Parameters:
 *   mode - speed/duplex mode
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef PIC32MZ_HAVE_PHY
static void pic32mz_macmode(uint8_t mode)
{
  /* Set up for full or half duplex operation */

  if ((mode & PIC32MZ_DUPLEX_MASK) == PIC32MZ_DUPLEX_FULL)
    {
      /* Set the back-to-back inter-packet gap */

      pic32mz_putreg(21, PIC32MZ_EMAC1_IPGT);

      /* Set MAC to operate in full duplex mode with CRC and Pad enabled */

      pic32mz_putreg((EMAC1_CFG2_FULLDPLX | EMAC1_CFG2_CRCEN |
                      EMAC1_CFG2_PADCRCEN), PIC32MZ_EMAC1_CFG2SET);
    }
  else
    {
      /* Set the back-to-back inter-packet gap */

      pic32mz_putreg(18, PIC32MZ_EMAC1_IPGT);

      /* Set MAC to operate in half duplex mode with CRC and Pad enabled */

      pic32mz_putreg(EMAC1_CFG2_FULLDPLX, PIC32MZ_EMAC1_CFG2CLR);
      pic32mz_putreg((EMAC1_CFG2_CRCEN | EMAC1_CFG2_PADCRCEN),
                     PIC32MZ_EMAC1_CFG2SET);
    }

  /* Set the RMII MAC speed. */

#if CONFIG_PIC32MZ_FMIIEN == 0
  if ((mode & PIC32MZ_SPEED_MASK) == PIC32MZ_SPEED_100)
    {
      pic32mz_putreg(EMAC1_SUPP_SPEEDRMII, PIC32MZ_EMAC1_SUPPSET);
    }
  else
    {
      pic32mz_putreg(EMAC1_SUPP_SPEEDRMII, PIC32MZ_EMAC1_SUPPCLR);
    }
#endif
}
#endif

/****************************************************************************
 * Function: pic32mz_ethreset
 *
 * Description:
 *   Configure and reset the Ethernet module, leaving it in a disabled state.
 *
 * Input Parameters:
 *   priv   - Reference to the driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *
 ****************************************************************************/

static void pic32mz_ethreset(struct pic32mz_driver_s *priv)
{
  irqstate_t flags;

  /* Reset the MAC */

  flags = enter_critical_section();

  /* Ethernet Controller Initialization *************************************/

  /* Disable Ethernet interrupts in the EVIC */

#if CONFIG_PIC32MZ_NINTERFACES > 1
  up_disable_irq(priv->pd_irqsrc);
#else
  up_disable_irq(PIC32MZ_IRQ_ETH);
#endif

  /* Turn the Ethernet Controller off: Clear the ON, RXEN and TXRTS bits */

  pic32mz_putreg(ETH_CON1_RXEN | ETH_CON1_TXRTS | ETH_CON1_ON,
                 PIC32MZ_ETH_CON1CLR);

  /* Wait activity abort by polling the ETHBUSY bit */

  while ((pic32mz_getreg(PIC32MZ_ETH_STAT) & ETH_STAT_ETHBUSY) != 0)
    continue;

  /* Turn the Ethernet controller on. */

  pic32mz_putreg(ETH_CON1_ON, PIC32MZ_ETH_CON1SET);

  /* Clear the Ethernet STAT BUFCNT */

  while ((pic32mz_getreg(PIC32MZ_ETH_STAT) & ETH_STAT_BUFCNT_MASK) != 0)
    {
      pic32mz_putreg(ETH_CON1_BUFCDEC, PIC32MZ_ETH_CON1SET);
    }

  /* Clear the Ethernet Interrupt Flag (ETHIF) bit in the Interrupts module */

#if CONFIG_PIC32MZ_NINTERFACES > 1
  up_pending_irq(priv->pd_irqsrc);
#else
  up_pending_irq(PIC32MZ_IRQ_ETH);
#endif

  /* Disable any Ethernet Controller interrupt generation by clearing the IEN
   * register.
   */

  pic32mz_putreg(ETH_INT_ALLINTS, PIC32MZ_ETH_IENCLR);

  /* Clear the TX and RX start addresses by using ETHTXSTCLR and ETHRXSTCLR */

  pic32mz_putreg(0xffffffff, PIC32MZ_ETH_TXSTCLR);
  pic32mz_putreg(0xffffffff, PIC32MZ_ETH_RXSTCLR);

  /* MAC Initialization *****************************************************/

  /* Put the MAC into the reset state */

  pic32mz_putreg((EMAC1_CFG1_TXRST  | EMAC1_CFG1_MCSTXRST |
                  EMAC1_CFG1_RXRST  | EMAC1_CFG1_MCSRXRST |
                  EMAC1_CFG1_SIMRST | EMAC1_CFG1_SOFTRST),
                  PIC32MZ_EMAC1_CFG1);

  /* Take the MAC out of the reset state */

  up_udelay(50);
  pic32mz_putreg(0, PIC32MZ_EMAC1_CFG1);
  leave_critical_section(flags);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: pic32mz_ethinitialize
 *
 * Description:
 *   Initialize one Ethernet controller and driver structure.
 *
 * Input Parameters:
 *   intf - Selects the interface to be initialized.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

#if CONFIG_PIC32MZ_NINTERFACES > 1 || defined(CONFIG_NETDEV_LATEINIT)
int pic32mz_ethinitialize(int intf)
#else
static inline int pic32mz_ethinitialize(int intf)
#endif
{
  struct pic32mz_driver_s *priv;
  int ret;

  DEBUGASSERT(intf < CONFIG_PIC32MZ_NINTERFACES);
  priv = &g_ethdrvr[intf];

  /* Initialize the driver structure */

  memset(priv, 0, sizeof(struct pic32mz_driver_s));
  priv->pd_dev.d_ifup    = pic32mz_ifup;    /* I/F down callback */
  priv->pd_dev.d_ifdown  = pic32mz_ifdown;  /* I/F up (new IP address) callback */
  priv->pd_dev.d_txavail = pic32mz_txavail; /* New TX data callback */
#ifdef CONFIG_NET_MCASTGROUP
  priv->pd_dev.d_addmac  = pic32mz_addmac;  /* Add multicast MAC address */
  priv->pd_dev.d_rmmac   = pic32mz_rmmac;   /* Remove multicast MAC address */
#endif
  priv->pd_dev.d_private = (void *)priv;    /* Used to recover private state from dev */

#if CONFIG_PIC32MZ_NINTERFACES > 1
#  error "A mechanism to associate base address an IRQ with an interface is needed"
  priv->pd_base          = ;            /* Ethernet controller base address */
  priv->pd_irq           = ;            /* Ethernet controller IRQ vector number */
  priv->pd_irqsrc        = ;            /* Ethernet controller IRQ source number */
#endif

  /* Create a watchdog for timing polling for and timing of transmissions */

  priv->pd_txpoll        = wd_create();   /* Create periodic poll timer */
  priv->pd_txtimeout     = wd_create();   /* Create TX timeout timer */

  /* Configure Ethernet peripheral pin selections */

  /* Controlled by DEVCFG FMIIEN and FETHIO settings */

  /* Reset the Ethernet controller and leave in the ifdown state.  The
   * Ethernet controller will be properly re-initialized each time
   * pic32mz_ifup() is called.
   */

  pic32mz_ifdown(&priv->pd_dev);

  /* Attach the IRQ to the driver */

#if CONFIG_PIC32MZ_NINTERFACES > 1
  ret = irq_attach(priv->pd_irq, pic32mz_interrupt, NULL);
#else
  ret = irq_attach(PIC32MZ_IRQ_ETH, pic32mz_interrupt, NULL);
#endif
  if (ret != 0)
    {
      /* We could not attach the ISR to the interrupt */

      return -EAGAIN;
    }

  /* Register the device with the OS so that socket IOCTLs can be performed */

  netdev_register(&priv->pd_dev, NET_LL_ETHERNET);
  return OK;
}

/****************************************************************************
 * Name: up_netinitialize
 *
 * Description:
 *   Initialize the first network interface.  If there are more than one
 *   interface in the chip, then board-specific logic will have to provide
 *   this function to determine which, if any, Ethernet controllers should
 *   be initialized.
 *
 ****************************************************************************/

#if CONFIG_PIC32MZ_NINTERFACES == 1 && !defined(CONFIG_NETDEV_LATEINIT)
void up_netinitialize(void)
{
  pic32mz_ethinitialize(0);
}
#endif
#endif /* CHIP_NETHERNET > 0 */
#endif /* CONFIG_NET && CONFIG_PIC32MZ_ETHERNET */
