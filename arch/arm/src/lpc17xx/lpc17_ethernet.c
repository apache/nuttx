/****************************************************************************
 * arch/arm/src/lpc17xx/lpc17_ethernet.c
 *
 *   Copyright (C) 2010-2015, 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
#if defined(CONFIG_NET) && defined(CONFIG_LPC17_ETHERNET)

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include <arpa/inet.h>

#include <nuttx/wdog.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/net/mii.h>
#include <nuttx/net/netconfig.h>
#include <nuttx/net/arp.h>
#include <nuttx/net/netdev.h>

#ifdef CONFIG_NET_PKT
#  include <nuttx/net/pkt.h>
#endif

#include "up_arch.h"
#include "chip.h"
#include "chip/lpc17_syscon.h"
#include "lpc17_gpio.h"
#include "lpc17_ethernet.h"
#include "lpc17_emacram.h"
#include "lpc17_clrpend.h"

#include <arch/board/board.h>

/* Does this chip have and Ethernet controller? */

#if LPC17_NETHCONTROLLERS > 0

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* If processing is not done at the interrupt level, then work queue support
 * is required.
 */

#if !defined(CONFIG_SCHED_WORKQUEUE)
#  error Work queue support is required
#else

  /* Select work queue */

#  if defined(CONFIG_LPC17_ETHERNET_HPWORK)
#    define ETHWORK HPWORK
#  elif defined(CONFIG_LPC17_ETHERNET_LPWORK)
#    define ETHWORK LPWORK
#  else
#    error Neither CONFIG_LPC17_ETHERNET_HPWORK nor CONFIG_LPC17_ETHERNET_LPWORK defined
#  endif
#endif

/* CONFIG_LPC17_NINTERFACES determines the number of physical interfaces
 * that will be supported -- unless it is more than actually supported by the
 * hardware!
 */

#if !defined(CONFIG_LPC17_NINTERFACES) || CONFIG_LPC17_NINTERFACES > LPC17_NETHCONTROLLERS
#  undef CONFIG_LPC17_NINTERFACES
#  define CONFIG_LPC17_NINTERFACES LPC17_NETHCONTROLLERS
#endif

/* The logic here has a few hooks for support for multiple interfaces, but
 * that capability is not yet in place (and I won't worry about it until I get
 * the first multi-interface LPC17xx).
 */

#if CONFIG_LPC17_NINTERFACES > 1
#  warning "Only a single ethernet controller is supported"
#  undef CONFIG_LPC17_NINTERFACES
#  define CONFIG_LPC17_NINTERFACES 1
#endif

/* If IGMP is enabled, then accept multi-cast frames. */

#if defined(CONFIG_NET_IGMP) && !defined(CONFIG_LPC17_MULTICAST)
#  define CONFIG_LPC17_MULTICAST 1
#endif

/* If the user did not specify a priority for Ethernet interrupts, set the
 * interrupt priority to the default.
 */

#ifndef CONFIG_LPC17_ETH_PRIORITY
#  define CONFIG_LPC17_ETH_PRIORITY NVIC_SYSH_PRIORITY_DEFAULT
#endif

#define PKTBUF_SIZE (MAX_NETDEV_PKTSIZE + CONFIG_NET_GUARDSIZE)

/* Debug Configuration *****************************************************/
/* Register debug -- can only happen of CONFIG_DEBUG_NET_INFO is selected */

#ifndef CONFIG_DEBUG_NET_INFO
#  undef  CONFIG_LPC17_NET_REGDEBUG
#endif

/* CONFIG_NET_DUMPPACKET will dump the contents of each packet to the
 * console.
 */

#ifndef CONFIG_DEBUG_NET_INFO
#  undef  CONFIG_NET_DUMPPACKET
#endif

#ifdef CONFIG_NET_DUMPPACKET
#  define lpc17_dumppacket(m,a,n) lib_dumpbuffer(m,a,n)
#else
#  define lpc17_dumppacket(m,a,n)
#endif

/* Timing *******************************************************************/

/* TX poll deley = 1 seconds. CLK_TCK is the number of clock ticks per second */

#define LPC17_WDDELAY        (1*CLK_TCK)

/* TX timeout = 1 minute */

#define LPC17_TXTIMEOUT      (60*CLK_TCK)

/* Interrupts ***************************************************************/

#define ETH_RXINTS           (ETH_INT_RXOVR | ETH_INT_RXERR | \
                              ETH_INT_RXFIN | ETH_INT_RXDONE)
#define ETH_TXINTS           (ETH_INT_TXUNR | ETH_INT_TXERR | \
                              ETH_INT_TXFIN | ETH_INT_TXDONE)

/* Misc. Helpers ***********************************************************/

/* This is a helper pointer for accessing the contents of the Ethernet header */

#define BUF ((struct eth_hdr_s *)priv->lp_dev.d_buf)

/* This is the number of ethernet GPIO pins that must be configured */

#define GPIO_NENET_PINS      10

/* PHYs *********************************************************************/
/* Select PHY-specific values.  Add more PHYs as needed. */

#if defined(CONFIG_ETH0_PHY_KS8721)
#  define LPC17_PHYNAME      "KS8721"
#  define LPC17_PHYID1       MII_PHYID1_KS8721
#  define LPC17_PHYID2       MII_PHYID2_KS8721
#  define LPC17_HAVE_PHY     1
#elif defined(CONFIG_ETH0_PHY_KSZ8041)
#  define LPC17_PHYNAME      "KSZ8041"
#  define LPC17_PHYID1       MII_PHYID1_KSZ8041
#  define LPC17_PHYID2       MII_PHYID2_KSZ8041
#  define LPC17_HAVE_PHY     1
#elif defined(CONFIG_ETH0_PHY_DP83848C)
#  define LPC17_PHYNAME      "DP83848C"
#  define LPC17_PHYID1       MII_PHYID1_DP83848C
#  define LPC17_PHYID2       MII_PHYID2_DP83848C
#  define LPC17_HAVE_PHY     1
#elif defined(CONFIG_ETH0_PHY_LAN8720)
#  define LPC17_PHYNAME      "LAN8720"
#  define LPC17_PHYID1       MII_PHYID1_LAN8720
#  define LPC17_PHYID2       MII_PHYID2_LAN8720
#  define LPC17_HAVE_PHY     1
#else
#  warning "No PHY specified!"
#  undef LPC17_HAVE_PHY
#endif

#define MII_BIG_TIMEOUT      666666

/* These definitions are used to remember the speed/duplex settings */

#define LPC17_SPEED_MASK     0x01
#define LPC17_SPEED_100      0x01
#define LPC17_SPEED_10       0x00

#define LPC17_DUPLEX_MASK    0x02
#define LPC17_DUPLEX_FULL    0x02
#define LPC17_DUPLEX_HALF    0x00

#define LPC17_10BASET_HD     (LPC17_SPEED_10  | LPC17_DUPLEX_HALF)
#define LPC17_10BASET_FD     (LPC17_SPEED_10  | LPC17_DUPLEX_FULL)
#define LPC17_100BASET_HD    (LPC17_SPEED_100 | LPC17_DUPLEX_HALF)
#define LPC17_100BASET_FD    (LPC17_SPEED_100 | LPC17_DUPLEX_FULL)

#ifdef CONFIG_LPC17_PHY_SPEED100
#  ifdef CONFIG_LPC17_PHY_FDUPLEX
#    define LPC17_MODE_DEFLT LPC17_100BASET_FD
#  else
#    define LPC17_MODE_DEFLT LPC17_100BASET_HD
#  endif
#else
#  ifdef CONFIG_LPC17_PHY_FDUPLEX
#    define LPC17_MODE_DEFLT LPC17_10BASET_FD
#  else
#    define LPC17_MODE_DEFLT LPC17_10BASET_HD
#  endif
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The lpc17_driver_s encapsulates all state information for a single hardware
 * interface
 */

struct lpc17_driver_s
{
  /* The following fields would only be necessary on chips that support
   * multiple Ethernet controllers.
   */

#if CONFIG_LPC17_NINTERFACES > 1
  uint32_t lp_base;             /* Ethernet controller base address */
  int      lp_irq;              /* Ethernet controller IRQ */
#endif

  bool     lp_ifup;             /* true:ifup false:ifdown */
  bool     lp_mode;             /* speed/duplex */
  bool     lp_txpending;        /* There is a pending Tx in lp_dev */
#ifdef LPC17_HAVE_PHY
  uint8_t  lp_phyaddr;          /* PHY device address */
#endif
  uint32_t lp_inten;            /* Shadow copy of INTEN register */
  WDOG_ID  lp_txpoll;           /* TX poll timer */
  WDOG_ID  lp_txtimeout;        /* TX timeout timer */

  struct work_s lp_txwork;      /* TX work continuation */
  struct work_s lp_rxwork;      /* RX work continuation */
  struct work_s lp_pollwork;    /* Poll work continuation */
  uint32_t status;

  /* This holds the information visible to the NuttX networking layer */

  struct net_driver_s lp_dev;  /* Interface understood by the network layer */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* A single packet buffer per interface is used */

static uint8_t g_pktbuf[PKTBUF_SIZE * CONFIG_LPC17_NINTERFACES];

/* Array of ethernet driver status structures */

static struct lpc17_driver_s g_ethdrvr[CONFIG_LPC17_NINTERFACES];

/* ENET pins are on P1[0,1,4,6,8,9,10,14,15] + MDC on P1[16] or P2[8] and
 * MDIO on P1[17] or P2[9].  The board.h file will define GPIO_ENET_MDC and
 * PGIO_ENET_MDIO to selec which pin setting to use.
 *
 * On older Rev '-' devices, P1[6] ENET-TX_CLK would also have be to configured.
 */

static const uint16_t g_enetpins[GPIO_NENET_PINS] =
{
  GPIO_ENET_TXD0, GPIO_ENET_TXD1, GPIO_ENET_TXEN,   GPIO_ENET_CRS, GPIO_ENET_RXD0,
  GPIO_ENET_RXD1, GPIO_ENET_RXER, GPIO_ENET_REFCLK, GPIO_ENET_MDC, GPIO_ENET_MDIO
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register operations */

#ifdef CONFIG_LPC17_NET_REGDEBUG
static void lpc17_printreg(uint32_t addr, uint32_t val, bool iswrite);
static void lpc17_checkreg(uint32_t addr, uint32_t val, bool iswrite);
static uint32_t lpc17_getreg(uint32_t addr);
static void lpc17_putreg(uint32_t val, uint32_t addr);
#else
# define lpc17_getreg(addr)     getreg32(addr)
# define lpc17_putreg(val,addr) putreg32(val,addr)
#endif

/* Common TX logic */

static int  lpc17_txdesc(struct lpc17_driver_s *priv);
static int  lpc17_transmit(struct lpc17_driver_s *priv);
static int  lpc17_txpoll(struct net_driver_s *dev);

/* Interrupt handling */

static void lpc17_response(struct lpc17_driver_s *priv);

static void lpc17_txdone_work(FAR void *arg);
static void lpc17_rxdone_work(FAR void *arg);
static int  lpc17_interrupt(int irq, void *context, FAR void *arg);

/* Watchdog timer expirations */

static void lpc17_txtimeout_work(FAR void *arg);
static void lpc17_txtimeout_expiry(int argc, uint32_t arg, ...);

static void lpc17_poll_work(FAR void *arg);
static void lpc17_poll_expiry(int argc, uint32_t arg, ...);

/* NuttX callback functions */

#ifdef CONFIG_NET_ICMPv6
static void lpc17_ipv6multicast(FAR struct lpc17_driver_s *priv);
#endif
static int lpc17_ifup(struct net_driver_s *dev);
static int lpc17_ifdown(struct net_driver_s *dev);

static void lpc17_txavail_work(FAR void *arg);
static int lpc17_txavail(struct net_driver_s *dev);

#if defined(CONFIG_NET_IGMP) || defined(CONFIG_NET_ICMPv6)
static uint32_t lpc17_calcethcrc(const uint8_t *data, size_t length);
static int lpc17_addmac(struct net_driver_s *dev, const uint8_t *mac);
#endif
#ifdef CONFIG_NET_IGMP
static int lpc17_rmmac(struct net_driver_s *dev, const uint8_t *mac);
#endif

/* Initialization functions */

#if defined(CONFIG_LPC17_NET_REGDEBUG) && defined(CONFIG_DEBUG_GPIO_INFO)
static void lpc17_showpins(void);
#else
#  define lpc17_showpins()
#endif

/* PHY initialization functions */

#ifdef LPC17_HAVE_PHY
#  ifdef CONFIG_LPC17_NET_REGDEBUG
static void lpc17_showmii(uint8_t phyaddr, const char *msg);
#  else
#    define lpc17_showmii(phyaddr,msg)
#  endif

static void lpc17_phywrite(uint8_t phyaddr, uint8_t regaddr,
                           uint16_t phydata);
static uint16_t lpc17_phyread(uint8_t phyaddr, uint8_t regaddr);
static inline int lpc17_phyreset(uint8_t phyaddr);
#  ifdef CONFIG_LPC17_PHY_AUTONEG
static inline int lpc17_phyautoneg(uint8_t phyaddr);
#  endif
static int lpc17_phymode(uint8_t phyaddr, uint8_t mode);
static inline int lpc17_phyinit(struct lpc17_driver_s *priv);
#else
#  define lpc17_phyinit(priv)
#endif

/* EMAC Initialization functions */

static inline void lpc17_txdescinit(struct lpc17_driver_s *priv);
static inline void lpc17_rxdescinit(struct lpc17_driver_s *priv);
static void lpc17_macmode(uint8_t mode);
static void lpc17_ethreset(struct lpc17_driver_s *priv);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc17_printreg
 *
 * Description:
 *   Print the contents of an LPC17xx register operation
 *
 ****************************************************************************/

#ifdef CONFIG_LPC17_NET_REGDEBUG
static void lpc17_printreg(uint32_t addr, uint32_t val, bool iswrite)
{
  ninfo("%08x%s%08x\n", addr, iswrite ? "<-" : "->", val);
}
#endif

/****************************************************************************
 * Name: lpc17_checkreg
 *
 * Description:
 *   Get the contents of an LPC17xx register
 *
 ****************************************************************************/

#ifdef CONFIG_LPC17_NET_REGDEBUG
static void lpc17_checkreg(uint32_t addr, uint32_t val, bool iswrite)
{
  static uint32_t prevaddr = 0;
  static uint32_t preval = 0;
  static uint32_t count = 0;
  static bool     prevwrite = false;

  /* Is this the same value that we read from/wrote to the same register last time?
   * Are we polling the register?  If so, suppress the output.
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

              lpc17_printreg(prevaddr, preval, prevwrite);
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

      /* Show the new regisgter access */

      lpc17_printreg(addr, val, iswrite);
    }
}
#endif

/****************************************************************************
 * Name: lpc17_getreg
 *
 * Description:
 *   Get the contents of an LPC17xx register
 *
 ****************************************************************************/

#ifdef CONFIG_LPC17_NET_REGDEBUG
static uint32_t lpc17_getreg(uint32_t addr)
{
  /* Read the value from the register */

  uint32_t val = getreg32(addr);

  /* Check if we need to print this value */

  lpc17_checkreg(addr, val, false);
  return val;
}
#endif

/****************************************************************************
 * Name: lpc17_putreg
 *
 * Description:
 *   Set the contents of an LPC17xx register to a value
 *
 ****************************************************************************/

#ifdef CONFIG_LPC17_NET_REGDEBUG
static void lpc17_putreg(uint32_t val, uint32_t addr)
{
  /* Check if we need to print this value */

  lpc17_checkreg(addr, val, true);

  /* Write the value */

  putreg32(val, addr);
}
#endif

/****************************************************************************
 * Function: lpc17_txdesc
 *
 * Description:
 *   Check if a free TX descriptor is available.
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

static int lpc17_txdesc(struct lpc17_driver_s *priv)
{
  unsigned int prodidx;
  unsigned int considx;

  /* Get the next producer index */

  prodidx = lpc17_getreg(LPC17_ETH_TXPRODIDX) & ETH_TXPRODIDX_MASK;
  if (++prodidx >= CONFIG_LPC17_ETH_NTXDESC)
    {
      /* Wrap back to index zero */

      prodidx = 0;
    }

  /* If the next producer index would overrun the consumer index, then there
   * are no available Tx descriptors.
   */

  considx = lpc17_getreg(LPC17_ETH_TXCONSIDX) & ETH_TXCONSIDX_MASK;
  return prodidx != considx ? OK : -EAGAIN;
}

/****************************************************************************
 * Function: lpc17_transmit
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

static int lpc17_transmit(struct lpc17_driver_s *priv)
{
  uint32_t *txdesc;
  void     *txbuffer;
  unsigned int prodidx;

  /* Verify that the hardware is ready to send another packet.  If we get
   * here, then we are committed to sending a packet; Higher level logic
   * must have assured that there is no transmission in progress.
   */

  DEBUGASSERT(lpc17_txdesc(priv) == OK);

  /* Increment statistics and dump the packet *if so configured) */

  NETDEV_TXPACKETS(&priv->lp_dev);
  lpc17_dumppacket("Transmit packet",
                   priv->lp_dev.d_buf, priv->lp_dev.d_len);

  /* Get the current producer index */

  prodidx = lpc17_getreg(LPC17_ETH_TXPRODIDX) & ETH_TXPRODIDX_MASK;

  /* Get the packet address from the descriptor and set the descriptor control
   * fields.
   */

  txdesc   = (uint32_t *)(LPC17_TXDESC_BASE + (prodidx << 3));
  txbuffer = (void *)*txdesc++;
  *txdesc  = TXDESC_CONTROL_INT | TXDESC_CONTROL_LAST | TXDESC_CONTROL_CRC |
             (priv->lp_dev.d_len - 1);

  /* Copy the packet data into the Tx buffer assignd to this descriptor.  It
   * should fit because each packet buffer is the MTU size and breaking up
   * largerTCP messasges is handled by higher level logic.  The hardware
   * does, however, support breaking up larger messages into many fragments,
   * however, that capability is not exploited here.
   *
   * This would be a great performance improvement:  Remove the buffer from
   * the lp_dev structure and replace it a pointer directly into the EMAC
   * DMA memory.  This could eliminate the following, costly memcpy.
   */

  DEBUGASSERT(priv->lp_dev.d_len <= LPC17_MAXPACKET_SIZE);
  memcpy(txbuffer, priv->lp_dev.d_buf, priv->lp_dev.d_len);

  /* Bump the producer index, making the packet available for transmission. */

  if (++prodidx >= CONFIG_LPC17_ETH_NTXDESC)
    {
      /* Wrap back to index zero */

      prodidx = 0;
    }

  lpc17_putreg(prodidx, LPC17_ETH_TXPRODIDX);

  /* Enable Tx interrupts */

  priv->lp_inten |= ETH_TXINTS;
  lpc17_putreg(priv->lp_inten, LPC17_ETH_INTEN);

  /* Setup the TX timeout watchdog (perhaps restarting the timer) */

  (void)wd_start(priv->lp_txtimeout, LPC17_TXTIMEOUT, lpc17_txtimeout_expiry,
                 1, (uint32_t)priv);
  return OK;
}

/****************************************************************************
 * Function: lpc17_txpoll
 *
 * Description:
 *   The transmitter is available, check if the network layer has any
 *   outgoing packets ready to send.  This is a callback from devif_poll().
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

static int lpc17_txpoll(struct net_driver_s *dev)
{
  struct lpc17_driver_s *priv = (struct lpc17_driver_s *)dev->d_private;
  int ret = OK;

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  if (priv->lp_dev.d_len > 0)
    {
      /* Look up the destination MAC address and add it to the Ethernet
       * header.
       */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
      if (IFF_IS_IPv4(priv->lp_dev.d_flags))
#endif
        {
          arp_out(&priv->lp_dev);
        }
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
      else
#endif
        {
          neighbor_out(&priv->lp_dev);
        }
#endif /* CONFIG_NET_IPv6 */

      if (!devif_loopback(&priv->lp_dev))
        {
          /* Send this packet.  In this context, we know that there is space for
           * at least one more packet in the descriptor list.
           */

          lpc17_transmit(priv);

          /* Check if there is room in the device to hold another packet. If not,
           * return any non-zero value to terminate the poll.
           */

          ret = lpc17_txdesc(priv);
        }
    }

  /* If zero is returned, the polling will continue until all connections have
   * been examined.
   */

  return ret;
}

/****************************************************************************
 * Function: lpc17_response
 *
 * Description:
 *   While processing an RxDone event, higher logic decides to send a packet,
 *   possibly a response to the incoming packet (but probably not, in reality).
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

static void lpc17_response(struct lpc17_driver_s *priv)
{
  int ret;

  /* Check if there is room in the device to hold another packet. */

  ret = lpc17_txdesc(priv);
  if (ret == OK)
    {
      /* Yes.. queue the packet now. */

      lpc17_transmit(priv);
    }
  else
    {
      /* No.. mark the Tx as pending and halt further RX interrupts that
       * could generate more TX activity.
       */

      DEBUGASSERT((priv->lp_inten & ETH_INT_TXDONE) != 0);

      priv->lp_txpending = true;
      priv->lp_inten    &= ~ETH_RXINTS;
      lpc17_putreg(priv->lp_inten, LPC17_ETH_INTEN);
    }
}

/****************************************************************************
 * Function: lpc17_rxdone_work
 *
 * Description:
 *   Perform Rx interrupt handling logic outside of the interrupt handler (on
 *   the work queue thread).
 *
 * Input Parameters:
 *   arg - The reference to the driver structure (case to void*)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void lpc17_rxdone_work(FAR void *arg)
{
  FAR struct lpc17_driver_s *priv = (FAR struct lpc17_driver_s *)arg;
  irqstate_t flags;
  uint32_t *rxstat;
  bool fragment;
  unsigned int prodidx;
  unsigned int considx;
  unsigned int pktlen;

  DEBUGASSERT(priv);

  /* Perform pending RX work.  RX interrupts were disabled prior to
   * scheduling this work to prevent work queue overruns.
   */

  net_lock();

  /* Get the current producer and consumer indices */

  considx = lpc17_getreg(LPC17_ETH_RXCONSIDX) & ETH_RXCONSIDX_MASK;
  prodidx = lpc17_getreg(LPC17_ETH_RXPRODIDX) & ETH_RXPRODIDX_MASK;

  /* Loop while there are incoming packets to be processed, that is, while
   * the producer index is not equal to the consumer index.
   */

  fragment = false;
  while (considx != prodidx)
    {
      /* Update statistics */

      NETDEV_RXPACKETS(&priv->lp_dev);

      /* Get the Rx status and packet length (-4+1) */

      rxstat   = (uint32_t *)(LPC17_RXSTAT_BASE + (considx << 3));
      pktlen   = (*rxstat & RXSTAT_INFO_RXSIZE_MASK) - 3;

      /* Check for errors.  NOTE:  The DMA engine reports bogus length errors,
       * making this a pretty useless (as well as annoying) check.
       */

      if ((*rxstat & RXSTAT_INFO_ERROR) != 0)
        {
          nerr("ERROR: considx: %08x prodidx: %08x rxstat: %08x\n",
               considx, prodidx, *rxstat);
          NETDEV_RXERRORS(&priv->lp_dev);
        }

      /* If the pktlen is greater then the buffer, then we cannot accept
       * the packet.  Also, since the DMA packet buffers are set up to
       * be the same size as our max packet size, any fragments also
       * imply that the packet is too big.
       */

      /* else */ if (pktlen > CONFIG_NET_ETH_PKTSIZE + CONFIG_NET_GUARDSIZE)
        {
          nwarn("WARNING: Too big. considx: %08x prodidx: %08x pktlen: %d rxstat: %08x\n",
                considx, prodidx, pktlen, *rxstat);
          NETDEV_RXERRORS(&priv->lp_dev);
        }
      else if ((*rxstat & RXSTAT_INFO_LASTFLAG) == 0)
        {
          ninfo("Fragment. considx: %08x prodidx: %08x pktlen: %d rxstat: %08x\n",
                considx, prodidx, pktlen, *rxstat);
          NETDEV_RXFRAGMENTS(&priv->lp_dev);
          fragment = true;
        }
      else if (fragment)
        {
          ninfo("Last fragment. considx: %08x prodidx: %08x pktlen: %d rxstat: %08x\n",
                considx, prodidx, pktlen, *rxstat);
          NETDEV_RXFRAGMENTS(&priv->lp_dev);
          fragment = false;
        }
      else
        {
          uint32_t *rxdesc;
          void     *rxbuffer;

          /* Get the Rx buffer address from the Rx descriptor */

          rxdesc   = (uint32_t *)(LPC17_RXDESC_BASE + (considx << 3));
          rxbuffer = (void *)*rxdesc;

          /* Copy the data data from the EMAC DMA RAM to priv->lp_dev.d_buf.
           * Set amount of data in priv->lp_dev.d_len
           *
           * Here would be a great performance improvement:  Remove the
           * buffer from the lp_dev structure and replace it with a pointer
           * directly into the EMAC DMA memory.  This could eliminate the
           * following, costly memcpy.
           */

          memcpy(priv->lp_dev.d_buf, rxbuffer, pktlen);
          priv->lp_dev.d_len = pktlen;

          lpc17_dumppacket("Received packet",
                           priv->lp_dev.d_buf, priv->lp_dev.d_len);

#ifdef CONFIG_NET_PKT
          /* When packet sockets are enabled, feed the frame into the packet
           * tap.
           */

           pkt_input(&priv->lp_dev);
#endif

          /* We only accept IP packets of the configured type and ARP packets */

#ifdef CONFIG_NET_IPv4
          if (BUF->type == HTONS(ETHTYPE_IP))
            {
              ninfo("IPv4 frame\n");
              NETDEV_RXIPV4(&priv->lp_dev);

              /* Handle ARP on input then give the IPv4 packet to the
               * network layer
               */

              arp_ipin(&priv->lp_dev);
              ipv4_input(&priv->lp_dev);

              /* If the above function invocation resulted in data that
               * should be sent out on the network, the field  d_len will
               * set to a value > 0.
               */

              if (priv->lp_dev.d_len > 0)
                {
                  /* Update the Ethernet header with the correct MAC address */

#ifdef CONFIG_NET_IPv6
                  if (IFF_IS_IPv4(priv->lp_dev.d_flags))
#endif
                    {
                      arp_out(&priv->lp_dev);
                    }
#ifdef CONFIG_NET_IPv6
                  else
                    {
                      neighbor_out(&priv->lp_dev);
                    }
#endif

                  /* And send the packet */

                  lpc17_response(priv);
                }
            }
          else
#endif
#ifdef CONFIG_NET_IPv6
          if (BUF->type == HTONS(ETHTYPE_IP6))
            {
              ninfo("Iv6 frame\n");
              NETDEV_RXIPV6(&priv->lp_dev);

              /* Give the IPv6 packet to the network layer */

              ipv6_input(&priv->lp_dev);

              /* If the above function invocation resulted in data that
               * should be sent out on the network, the field  d_len will
               * set to a value > 0.
               */

              if (priv->lp_dev.d_len > 0)
                {
                  /* Update the Ethernet header with the correct MAC address */

#ifdef CONFIG_NET_IPv4
                  if (IFF_IS_IPv4(priv->lp_dev.d_flags))
                    {
                      arp_out(&priv->lp_dev);
                    }
                  else
#endif
#ifdef CONFIG_NET_IPv6
                    {
                      neighbor_out(&priv->lp_dev);
                    }
#endif

                  /* And send the packet */

                  lpc17_response(priv);
                }
            }
          else
#endif
#ifdef CONFIG_NET_ARP
          if (BUF->type == htons(ETHTYPE_ARP))
            {
              NETDEV_RXARP(&priv->lp_dev);
              arp_arpin(&priv->lp_dev);

              /* If the above function invocation resulted in data that
               * should be sent out on the network, the field  d_len will
               * set to a value > 0.
               */

              if (priv->lp_dev.d_len > 0)
                {
                  lpc17_response(priv);
                }
            }
          else
#endif
            {
              /* Unrecognized... drop it. */

              NETDEV_RXDROPPED(&priv->lp_dev);
            }
        }

      /* Bump up the consumer index and resample the producer index (which
       * might also have gotten bumped up by the hardware).
       */

      if (++considx >= CONFIG_LPC17_ETH_NRXDESC)
        {
          /* Wrap back to index zero */

          considx = 0;
        }

      lpc17_putreg(considx, LPC17_ETH_RXCONSIDX);
      prodidx = lpc17_getreg(LPC17_ETH_RXPRODIDX) & ETH_RXPRODIDX_MASK;
    }

  net_unlock();

  /* Re-enable RX interrupts (this must be atomic).  Skip this step if the
   * lp-txpending TX underrun state is in effect.
   */

  flags = enter_critical_section();
  if (!priv->lp_txpending)
    {
      priv->lp_inten |= ETH_RXINTS;
      lpc17_putreg(priv->lp_inten, LPC17_ETH_INTEN);
    }

  leave_critical_section(flags);
}


/****************************************************************************
 * Function: lpc17_txdone_work
 *
 * Description:
 *   Perform Tx interrupt handling logic outside of the interrupt handler (on
 *   the work queue thread).
 *
 * Input Parameters:
 *   arg - The reference to the driver structure (case to void*)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lpc17_txdone_work(FAR void *arg)
{
  FAR struct lpc17_driver_s *priv = (FAR struct lpc17_driver_s *)arg;

  /* Verify that the hardware is ready to send another packet.  Since a Tx
   * just completed, this must be the case.
   */

  DEBUGASSERT(priv);
  DEBUGASSERT(lpc17_txdesc(priv) == OK);

  /* Check if there is a pending Tx transfer that was scheduled by Rx handling
   * while the Tx logic was busy.  If so, processing that pending Tx now.
   */

  net_lock();
  if (priv->lp_txpending)
    {
      /* Clear the pending condition, send the packet, and restore Rx interrupts */

      priv->lp_txpending = false;

      lpc17_transmit(priv);

      priv->lp_inten |= ETH_RXINTS;
      lpc17_putreg(priv->lp_inten, LPC17_ETH_INTEN);
    }

  /* Otherwise poll the network layer for new XMIT data */

  else
    {
      (void)devif_poll(&priv->lp_dev, lpc17_txpoll);
    }

  net_unlock();
}

/****************************************************************************
 * Function: lpc17_interrupt
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

static int lpc17_interrupt(int irq, void *context, FAR void *arg)
{
  register struct lpc17_driver_s *priv;
  uint32_t status;

#if CONFIG_LPC17_NINTERFACES > 1
# error "A mechanism to associate and interface with an IRQ is needed"
#else
  priv = &g_ethdrvr[0];
#endif

  /* Get the interrupt status (zero means no interrupts pending). */

  status = lpc17_getreg(LPC17_ETH_INTST);
  if (status != 0)
    {
      /* Clear all pending interrupts */

      lpc17_putreg(status, LPC17_ETH_INTCLR);

      /* Handle each pending interrupt **************************************/
      /* Check for Wake-Up on Lan *******************************************/

#ifdef CONFIG_LPC17_ETH_WOL
      if ((status & ETH_INT_WKUP) != 0)
        {
#         warning "Missing logic"
        }
      else
#endif
      /* Fatal Errors *******************************************************/
      /* RX OVERRUN -- Fatal overrun error in the receive queue. The fatal
       * interrupt should be resolved by a Rx soft-reset. The bit is not
       * set when there is a nonfatal overrun error.
       *
       * TX UNDERRUN -- Interrupt set on a fatal underrun error in the
       * transmit queue. The fatal interrupt should be resolved by a Tx
       * soft-reset. The bit is not set when there is a nonfatal underrun
       * error.
       */

      if ((status & (ETH_INT_RXOVR | ETH_INT_TXUNR)) != 0)
        {
          if ((status & ETH_INT_RXOVR) != 0)
            {
              nerr("ERROR: RX Overrun. status: %08x\n", status);
              NETDEV_RXERRORS(&priv->lp_dev);
            }

          if ((status & ETH_INT_TXUNR) != 0)
            {
              nerr("ERROR: TX Underrun. status: %08x\n", status);
              NETDEV_TXERRORS(&priv->lp_dev);
            }

          /* ifup() will reset the EMAC and bring it back up */

           (void)lpc17_ifup(&priv->lp_dev);
        }
      else
        {
          /* Check for receive events ***************************************/
          /* RX ERROR -- Triggered on receive errors: AlignmentError,
           * RangeError, LengthError, SymbolError, CRCError or NoDescriptor
           * or Overrun.  NOTE:  (1) We will still need to call lpc17_rxdone_process
           * on RX errors to bump the considx over the bad packet.  (2) The
           * DMA engine reports bogus length errors, making this a pretty
           * useless (as well as annoying) check anyway.
           */

          if ((status & ETH_INT_RXERR) != 0)
            {
              nerr("ERROR: RX ERROR: status: %08x\n", status);
              NETDEV_RXERRORS(&priv->lp_dev);
            }

          /* RX FINISHED -- Triggered when all receive descriptors have
           * been processed i.e. on the transition to the situation
           * where ProduceIndex == ConsumeIndex.
           *
           * Treated as INT_RX_DONE if ProduceIndex != ConsumeIndex so the
           * packets are processed anyway.
           *
           * RX DONE -- Triggered when a receive descriptor has been
           * processed while the Interrupt bit in the Control field of
           * the descriptor was set.
           */

          if ((status & ETH_INT_RXFIN) != 0 || (status & ETH_INT_RXDONE) != 0)
            {
              /* We have received at least one new incoming packet. */
              /* Disable further TX interrupts for now.  TX interrupts will
               * be re-enabled after the work has been processed.
               */

              priv->lp_inten &= ~ETH_RXINTS;
              lpc17_putreg(priv->lp_inten, LPC17_ETH_INTEN);

              /* Schedule RX-related work to be performed on the work thread,
               * perhaps cancelling any pending RX work.
               */

              work_queue(ETHWORK, &priv->lp_rxwork, (worker_t)lpc17_rxdone_work,
                         priv, 0);
            }

          /* Check for Tx events ********************************************/
          /* TX ERROR -- Triggered on transmit errors: LateCollision,
           * ExcessiveCollision and ExcessiveDefer, NoDescriptor or Underrun.
           * NOTE: We will still need to call lpc17_txdone_process() in order to
           * clean up after the failed transmit.
           */

          if ((status & ETH_INT_TXERR) != 0)
            {
              nerr("ERROR: TX ERROR: status: %08x\n", status);
              NETDEV_TXERRORS(&priv->lp_dev);
            }

#if 0
          /* TX FINISHED -- Triggered when all transmit descriptors have
           * been processed i.e. on the transition to the situation
           * where ProduceIndex == ConsumeIndex.
           */

          if ((status & ETH_INT_TXFIN) != 0)
            {
            }
#endif

          /* TX DONE -- Triggered when a descriptor has been transmitted
           * while the Interrupt bit in the Control field of the
           * descriptor was set.
           */

          if ((status & ETH_INT_TXDONE) != 0)
            {
              NETDEV_TXDONE(&priv->lp_dev);

              /* A packet transmission just completed */
              /* Cancel the pending Tx timeout */

              wd_cancel(priv->lp_txtimeout);

              /* Disable further Tx interrupts.  Tx interrupts may be
               * re-enabled again depending upon the actions of
               * lpc17_txdone_process()
               */

              priv->lp_inten &= ~ETH_TXINTS;
              lpc17_putreg(priv->lp_inten, LPC17_ETH_INTEN);

              /* Cancel any pending TX done work (to prevent overruns and also
               * to avoid race conditions with the TX timeout work)
               */

              work_cancel(ETHWORK, &priv->lp_txwork);

              /* Schedule TX-related work to be performed on the work thread,
               * perhaps cancelling any pending TX work.
               */

              work_queue(ETHWORK, &priv->lp_txwork, (worker_t)lpc17_txdone_work,
                         priv, 0);
            }
        }
    }

  /* Clear the pending interrupt */

#if 0 /* Apparently not necessary */
# if CONFIG_LPC17_NINTERFACES > 1
  lpc17_clrpend(priv->irq);
# else
  lpc17_clrpend(LPC17_IRQ_ETH);
# endif
#endif

  return OK;
}

/****************************************************************************
 * Function: lpc17_txtimeout_work
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

static void lpc17_txtimeout_work(FAR void *arg)
{
  FAR struct lpc17_driver_s *priv = (FAR struct lpc17_driver_s *)arg;

  /* Increment statistics and dump debug info */

  net_lock();
  NETDEV_TXTIMEOUTS(&priv->lp_dev);
  if (priv->lp_ifup)
    {
      /* Then reset the hardware. ifup() will reset the interface, then bring
       * it back up.
       */

      (void)lpc17_ifup(&priv->lp_dev);

      /* Then poll the network layer for new XMIT data */

      (void)devif_poll(&priv->lp_dev, lpc17_txpoll);
    }

  net_unlock();
}

/****************************************************************************
 * Function: lpc17_txtimeout_expiry
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

static void lpc17_txtimeout_expiry(int argc, uint32_t arg, ...)
{
  struct lpc17_driver_s *priv = (struct lpc17_driver_s *)arg;

  /* Disable further Tx interrupts.  Tx interrupts may be re-enabled again
   * depending upon the actions of lpc17_poll_process()
   */

  priv->lp_inten &= ~ETH_TXINTS;
  lpc17_putreg(priv->lp_inten, LPC17_ETH_INTEN);

  /* Is the single TX work structure available?  If not, then there is
   * pending TX work to be done this must be a false alarm TX timeout.
   */

  if (work_available(&priv->lp_txwork))
    {
      /* Schedule to perform the interrupt processing on the worker thread. */

      work_queue(ETHWORK, &priv->lp_txwork, lpc17_txtimeout_work, priv, 0);
    }
}

/****************************************************************************
 * Function: lpc17_poll_work
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
 *   Ethernet interrupts are disabled
 *
 ****************************************************************************/

static void lpc17_poll_work(FAR void *arg)
{
  FAR struct lpc17_driver_s *priv = (FAR struct lpc17_driver_s *)arg;
  unsigned int prodidx;
  unsigned int considx;

  /* Check if there is room in the send another TX packet.  We cannot perform
   * the TX poll if he are unable to accept another packet for transmission.
   */

  net_lock();
  if (lpc17_txdesc(priv) == OK)
    {
      /* If so, update TCP timing states and poll the network layer for new
       * XMIT data. Hmmm.. might be bug here.  Does this mean if there is a
       * transmit in progress, we will missing TCP time state updates?
       */

      (void)devif_timer(&priv->lp_dev, lpc17_txpoll);
    }

  /* Simulate a fake receive to relaunch the data exchanges when a receive
   * interrupt has been lost and all the receive buffers are used.
   */

  /* Get the current producer and consumer indices */

  considx = lpc17_getreg(LPC17_ETH_RXCONSIDX) & ETH_RXCONSIDX_MASK;
  prodidx = lpc17_getreg(LPC17_ETH_RXPRODIDX) & ETH_RXPRODIDX_MASK;

  if (considx != prodidx)
    {
      work_queue(ETHWORK, &priv->lp_rxwork, (worker_t)lpc17_rxdone_work,
                 priv, 0);
    }

  /* Setup the watchdog poll timer again */

  (void)wd_start(priv->lp_txpoll, LPC17_WDDELAY, lpc17_poll_expiry,
                1, priv);
  net_unlock();
}

/****************************************************************************
 * Function: lpc17_poll_expiry
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

static void lpc17_poll_expiry(int argc, uint32_t arg, ...)
{
  FAR struct lpc17_driver_s *priv = (FAR struct lpc17_driver_s *)arg;

  DEBUGASSERT(arg);

  /* Schedule to perform the interrupt processing on the worker thread. */

  work_queue(ETHWORK, &priv->lp_pollwork, lpc17_poll_work, priv, 0);
}

/****************************************************************************
 * Function: lpc17_ipv6multicast
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
static void lpc17_ipv6multicast(FAR struct lpc17_driver_s *priv)
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

  dev    = &priv->lp_dev;
  tmp16  = dev->d_ipv6addr[6];
  mac[2] = 0xff;
  mac[3] = tmp16 >> 8;

  tmp16  = dev->d_ipv6addr[7];
  mac[4] = tmp16 & 0xff;
  mac[5] = tmp16 >> 8;

  ninfo("IPv6 Multicast: %02x:%02x:%02x:%02x:%02x:%02x\n",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  (void)lpc17_addmac(dev, mac);

#ifdef CONFIG_NET_ICMPv6_AUTOCONF
  /* Add the IPv6 all link-local nodes Ethernet address.  This is the
   * address that we expect to receive ICMPv6 Router Advertisement
   * packets.
   */

  (void)lpc17_addmac(dev, g_ipv6_ethallnodes.ether_addr_octet);

#endif /* CONFIG_NET_ICMPv6_AUTOCONF */
#ifdef CONFIG_NET_ICMPv6_ROUTER
  /* Add the IPv6 all link-local routers Ethernet address.  This is the
   * address that we expect to receive ICMPv6 Router Solicitation
   * packets.
   */

  (void)lpc17_addmac(dev, g_ipv6_ethallrouters.ether_addr_octet);

#endif /* CONFIG_NET_ICMPv6_ROUTER */
}
#endif /* CONFIG_NET_ICMPv6 */

/****************************************************************************
 * Function: lpc17_ifup
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

static int lpc17_ifup(struct net_driver_s *dev)
{
  struct lpc17_driver_s *priv = (struct lpc17_driver_s *)dev->d_private;
  uint32_t regval;
  int ret;

  ninfo("Bringing up: %d.%d.%d.%d\n",
        dev->d_ipaddr & 0xff, (dev->d_ipaddr >> 8) & 0xff,
        (dev->d_ipaddr >> 16) & 0xff, dev->d_ipaddr >> 24);

  /* Reset the Ethernet controller (again) */

  lpc17_ethreset(priv);

  /* Initialize the PHY and wait for the link to be established */

  ret = lpc17_phyinit(priv);
  if (ret != 0)
    {
      nerr("ERROR: lpc17_phyinit failed: %d\n", ret);
      return ret;
    }

  /* Configure the MAC station address */

  regval = (uint32_t)priv->lp_dev.d_mac.ether.ether_addr_octet[5] << 8 |
           (uint32_t)priv->lp_dev.d_mac.ether.ether_addr_octet[4];
  lpc17_putreg(regval, LPC17_ETH_SA0);

  regval = (uint32_t)priv->lp_dev.d_mac.ether.ether_addr_octet[3] << 8 |
           (uint32_t)priv->lp_dev.d_mac.ether.ether_addr_octet[2];
  lpc17_putreg(regval, LPC17_ETH_SA1);

  regval = (uint32_t)priv->lp_dev.d_mac.ether.ether_addr_octet[1] << 8 |
           (uint32_t)priv->lp_dev.d_mac.ether.ether_addr_octet[0];
  lpc17_putreg(regval, LPC17_ETH_SA2);

#ifdef CONFIG_NET_ICMPv6
  /* Set up the IPv6 multicast address */

  lpc17_ipv6multicast(priv);
#endif

  /* Initialize Ethernet interface for the PHY setup */

  lpc17_macmode(priv->lp_mode);

  /* Initialize EMAC DMA memory -- descriptors, status, packet buffers, etc. */

  lpc17_txdescinit(priv);
  lpc17_rxdescinit(priv);

  /* Configure to pass all received frames */

  regval = lpc17_getreg(LPC17_ETH_MAC1);
  regval |= ETH_MAC1_PARF;
  lpc17_putreg(regval, LPC17_ETH_MAC1);

  /* Set up RX filter and configure to accept broadcast addresses, multicast
   * addresses, and perfect station address matches.  We should also accept
   * perfect matches and, most likely, broadcast (for example, for ARP requests).
   * Other RX filter options will only be enabled if so selected.  NOTE: There
   * is a selection CONFIG_NET_BROADCAST, but this enables receipt of UDP
   * broadcast packets inside of the stack.
   */

  regval = ETH_RXFLCTRL_PERFEN | ETH_RXFLCTRL_BCASTEN;
#ifdef CONFIG_LPC17_MULTICAST
  regval |= (ETH_RXFLCTRL_MCASTEN | ETH_RXFLCTRL_UCASTEN);
#endif
#ifdef CONFIG_LPC17_ETH_HASH
  regval |= (ETH_RXFLCTRL_MCASTHASHEN | ETH_RXFLCTRL_UCASTHASHEN);
#endif
  lpc17_putreg(regval, LPC17_ETH_RXFLCTRL);

  /* Clear any pending interrupts (shouldn't be any) */

  lpc17_putreg(0xffffffff, LPC17_ETH_INTCLR);

  /* Configure interrupts.  The Ethernet interrupt was attached during one-time
   * initialization, so we only need to set the interrupt priority, configure
   * interrupts, and enable them.
   */

  /* Set the interrupt to the highest priority */

#ifdef CONFIG_ARCH_IRQPRIO
#if CONFIG_LPC17_NINTERFACES > 1
  (void)up_prioritize_irq(priv->irq, CONFIG_LPC17_ETH_PRIORITY);
#else
  (void)up_prioritize_irq(LPC17_IRQ_ETH, CONFIG_LPC17_ETH_PRIORITY);
#endif
#endif

  /* Enable Ethernet interrupts.  The way we do this depends on whether or
   * not Wakeup on Lan (WoL) has been configured.
   */

#ifdef CONFIG_LPC17_ETH_WOL
  /* Configure WoL: Clear all receive filter WoLs and enable the perfect
   * match WoL interrupt.  We will wait until the Wake-up to finish
   * bringing things up.
   */

  lpc17_putreg(0xffffffff, LPC17_ETH_RXFLWOLCLR);
  lpc17_putreg(ETH_RXFLCTRL_RXFILEN, LPC17_ETH_RXFLCTRL);

  priv->lp_inten = ETH_INT_WKUP;
  lpc17_putreg(ETH_INT_WKUP, LPC17_ETH_INTEN);
#else
  /* Otherwise, enable all Rx interrupts.  Tx interrupts, SOFTINT and WoL are
   * excluded.  Tx interrupts will not be enabled until there is data to be
   * sent.
   */

  priv->lp_inten = ETH_RXINTS;
  lpc17_putreg(ETH_RXINTS, LPC17_ETH_INTEN);
#endif

  /* Enable Rx. "Enabling of the receive function is located in two places.
   * The receive DMA manager needs to be enabled and the receive data path
   * of the MAC needs to be enabled. To prevent overflow in the receive
   * DMA engine the receive DMA engine should be enabled by setting the
   * RxEnable bit in the Command register before enabling the receive data
   * path in the MAC by setting the RECEIVE ENABLE bit in the MAC1 register."
   */

  regval  = lpc17_getreg(LPC17_ETH_CMD);
  regval |= ETH_CMD_RXEN;
  lpc17_putreg(regval, LPC17_ETH_CMD);

  regval  = lpc17_getreg(LPC17_ETH_MAC1);
  regval |= ETH_MAC1_RE;
  lpc17_putreg(regval, LPC17_ETH_MAC1);

  /* Enable Tx */

  regval  = lpc17_getreg(LPC17_ETH_CMD);
  regval |= ETH_CMD_TXEN;
  lpc17_putreg(regval, LPC17_ETH_CMD);

  /* Set and activate a timer process */

  (void)wd_start(priv->lp_txpoll, LPC17_WDDELAY, lpc17_poll_expiry, 1,
                (uint32_t)priv);

  /* Finally, make the interface up and enable the Ethernet interrupt at
   * the interrupt controller
   */

  priv->lp_ifup = true;
#if CONFIG_LPC17_NINTERFACES > 1
  up_enable_irq(priv->irq);
#else
  up_enable_irq(LPC17_IRQ_ETH);
#endif
  return OK;
}

/****************************************************************************
 * Function: lpc17_ifdown
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

static int lpc17_ifdown(struct net_driver_s *dev)
{
  struct lpc17_driver_s *priv = (struct lpc17_driver_s *)dev->d_private;
  irqstate_t flags;

  /* Disable the Ethernet interrupt */

  flags = enter_critical_section();
  up_disable_irq(LPC17_IRQ_ETH);

  /* Cancel the TX poll timer and TX timeout timers */

  wd_cancel(priv->lp_txpoll);
  wd_cancel(priv->lp_txtimeout);

  /* Reset the device and mark it as down. */

  lpc17_ethreset(priv);
  priv->lp_ifup = false;
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Function: lpc17_txavail_work
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

static void lpc17_txavail_work(FAR void *arg)
{
  FAR struct lpc17_driver_s *priv = (FAR struct lpc17_driver_s *)arg;

  /* Ignore the notification if the interface is not yet up */

  net_lock();
  if (priv->lp_ifup)
    {
      /* Check if there is room in the hardware to hold another outgoing packet. */

      if (lpc17_txdesc(priv) == OK)
        {
          /* If so, then poll the network layer for new XMIT data */

          (void)devif_poll(&priv->lp_dev, lpc17_txpoll);
        }
    }

  net_unlock();
}

/****************************************************************************
 * Function: lpc17_txavail
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

static int lpc17_txavail(struct net_driver_s *dev)
{
  FAR struct lpc17_driver_s *priv = (FAR struct lpc17_driver_s *)dev->d_private;

  /* Is our single poll work structure available?  It may not be if there
   * are pending polling actions and we will have to ignore the Tx
   * availability action (which is okay because all poll actions have,
   * ultimately, the same effect.
   */

  if (work_available(&priv->lp_pollwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      work_queue(ETHWORK, &priv->lp_pollwork, lpc17_txavail_work, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Function: lpc17_calcethcrc
 *
 * Description:
 *   Function to calculate the CRC used by LPC17 to check an Ethernet frame
 *
 *   Algorithm adapted from LPC17xx sample code that contains this notice:
 *
 *     Software that is described herein is for illustrative purposes only
 *     which provides customers with programming information regarding the
 *     products. This software is supplied "AS IS" without any warranties.
 *     NXP Semiconductors assumes no responsibility or liability for the
 *     use of the software, conveys no license or title under any patent,
 *     copyright, or mask work right to the product. NXP Semiconductors
 *     reserves the right to make changes in the software without
 *     notification. NXP Semiconductors also make no representation or
 *     warranty that such application will be suitable for the specified
 *     use without further testing or modification.
 *
 * Input Parameters:
 *   data   - the data to be checked
 *   length - length of the data
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#if defined(CONFIG_NET_IGMP) || defined(CONFIG_NET_ICMPv6)
static uint32_t lpc17_calcethcrc(const uint8_t *data, size_t length)
{
  char byte;
  int crc;
  int q0;
  int q1;
  int q2;
  int q3;
  int i;
  int j;

  crc = 0xffffffff;
  for (i = 0; i < length; i++)
    {
      byte = *data++;
      for (j = 0; j < 2; j++)
        {
          if (((crc >> 28) ^ (byte >> 3)) & 0x00000001)
            {
              q3 = 0x04c11db7;
            }
          else
            {
              q3 = 0x00000000;
            }

          if (((crc >> 29) ^ (byte >> 2)) & 0x00000001)
            {
              q2 = 0x09823b6e;
            }
          else
            {
              q2 = 0x00000000;
            }

          if (((crc >> 30) ^ (byte >> 1)) & 0x00000001)
            {
              q1 = 0x130476dc;
            }
          else
            {
              q1 = 0x00000000;
            }

          if (((crc >> 31) ^ (byte >> 0)) & 0x00000001)
            {
              q0 = 0x2608EDB8;
            }
          else
            {
              q0 = 0x00000000;
            }

          crc = (crc << 4) ^ q3 ^ q2 ^ q1 ^ q0;
          byte >>= 4;
        }
    }

  return crc;
}
#endif /* CONFIG_NET_IGMP || CONFIG_NET_ICMPv6 */

/****************************************************************************
 * Function: lpc17_addmac
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

#if defined(CONFIG_NET_IGMP) || defined(CONFIG_NET_ICMPv6)
static int lpc17_addmac(struct net_driver_s *dev, const uint8_t *mac)
{
  uintptr_t regaddr;
  uint32_t regval;
  uint32_t crc;
  unsigned int ndx;

  ninfo("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  /* Hash function:
   *
   * The standard Ethernet cyclic redundancy check (CRC) function is
   * calculated from the 6 byte MAC address.  Bits [28:23] out of the 32-bit
   * CRC result are taken to form the hash. The 6-bit hash is used to access
   * the hash table: it is used as an index in the 64-bit HashFilter register
   * that has been programmed with accept values. If the selected accept value
   * is 1, the frame is accepted.
   */

  crc = lpc17_calcethcrc(mac, 6);
  ndx = (crc >> 23) & 0x3f;

  /* Add the MAC address to the hardware multicast hash table */

  if (ndx > 31)
    {
      regaddr = LPC17_ETH_HASHFLH; /* Hash filter table MSBs register */
      ndx -= 32;
    }
  else
    {
      regaddr = LPC17_ETH_HASHFLL;  /* Hash filter table LSBs register */
    }

  regval = lpc17_getreg(regaddr);
  regval |= 1 << ndx;
  lpc17_putreg(regval, regaddr);

  /* Enabled multicast address filtering in the RxFilterControl register:
   *
   *   AcceptUnicastHashEn: When set to ’1’, unicast frames that pass the
   *     imperfect hash filter are accepted.
   *   AcceptMulticastHashEn When set to ’1’, multicast frames that pass the
   *     imperfect hash filter are accepted.
   */

  regval = lpc17_getreg(LPC17_ETH_RXFLCTRL);
  regval &= ~ETH_RXFLCTRL_UCASTHASHEN;
  regval |= ETH_RXFLCTRL_MCASTHASHEN;
  lpc17_putreg(regval, LPC17_ETH_RXFLCTRL);

  return OK;
}
#endif /* CONFIG_NET_IGMP || CONFIG_NET_ICMPv6 */

/****************************************************************************
 * Function: lpc17_rmmac
 *
 * Description:
 *   NuttX Callback: Remove the specified MAC address from the hardware multicast
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

#ifdef CONFIG_NET_IGMP
static int lpc17_rmmac(struct net_driver_s *dev, const uint8_t *mac)
{
  uintptr_t regaddr1;
  uintptr_t regaddr2;
  uint32_t regval;
  uint32_t crc;
  unsigned int ndx;

  ninfo("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  /* Hash function:
   *
   * The standard Ethernet cyclic redundancy check (CRC) function is
   * calculated from the 6 byte MAC address.  Bits [28:23] out of the 32-bit
   * CRC result are taken to form the hash. The 6-bit hash is used to access
   * the hash table: it is used as an index in the 64-bit HashFilter register
   * that has been programmed with accept values. If the selected accept value
   * is 1, the frame is accepted.
   */

  crc = lpc17_calcethcrc(mac, 6);
  ndx = (crc >> 23) & 0x3f;

  /* Remove the MAC address to the hardware multicast hash table */

  if (ndx > 31)
    {
      regaddr1 = LPC17_ETH_HASHFLH; /* Hash filter table MSBs register */
      regaddr2 = LPC17_ETH_HASHFLL; /* Hash filter table LSBs register */
      ndx     -= 32;
    }
  else
    {
      regaddr1 = LPC17_ETH_HASHFLL;  /* Hash filter table LSBs register */
      regaddr2 = LPC17_ETH_HASHFLH;  /* Hash filter table MSBs register */
    }

  regval  = lpc17_getreg(regaddr1);
  regval &= ~(1 << ndx);
  lpc17_putreg(regval, regaddr1);

  /* If there are no longer addresses being filtered , disable multicast
   * filtering.
   */

  if (regval == 0 && lpc17_getreg(regaddr2) == 0)
    {
      /*   AcceptUnicastHashEn: When set to ’1’, unicast frames that pass the
       *     imperfect hash filter are accepted.
       *   AcceptMulticastHashEn When set to ’1’, multicast frames that pass the
       *     imperfect hash filter are accepted.
       */

      regval = lpc17_getreg(LPC17_ETH_RXFLCTRL);
      regval &= ~(ETH_RXFLCTRL_UCASTHASHEN | ETH_RXFLCTRL_MCASTHASHEN);
      lpc17_putreg(regval, LPC17_ETH_RXFLCTRL);
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: lpc17_showpins
 *
 * Description:
 *   Dump GPIO registers
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

#if defined(CONFIG_LPC17_NET_REGDEBUG) && defined(CONFIG_DEBUG_GPIO_INFO)
static void lpc17_showpins(void)
{
  lpc17_dumpgpio(GPIO_PORT1 | GPIO_PIN0, "P1[1-15]");
  lpc17_dumpgpio(GPIO_PORT1 | GPIO_PIN16, "P1[16-31]");
}
#endif

/****************************************************************************
 * Name: lpc17_showmii
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

#if defined(CONFIG_LPC17_NET_REGDEBUG) && defined(LPC17_HAVE_PHY)
static void lpc17_showmii(uint8_t phyaddr, const char *msg)
{
  ninfo("PHY " LPC17_PHYNAME ": %s\n", msg);
  ninfo("  MCR:       %04x\n", lpc17_phyread(phyaddr, MII_MCR));
  ninfo("  MSR:       %04x\n", lpc17_phyread(phyaddr, MII_MSR));
  ninfo("  ADVERTISE: %04x\n", lpc17_phyread(phyaddr, MII_ADVERTISE));
  ninfo("  LPA:       %04x\n", lpc17_phyread(phyaddr, MII_LPA));
  ninfo("  EXPANSION: %04x\n", lpc17_phyread(phyaddr, MII_EXPANSION));
#ifdef CONFIG_ETH0_PHY_KS8721
  ninfo("  10BTCR:    %04x\n", lpc17_phyread(phyaddr, MII_KS8721_10BTCR));
#endif
}
#endif

/****************************************************************************
 * Function: lpc17_phywrite
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

#ifdef LPC17_HAVE_PHY
static void lpc17_phywrite(uint8_t phyaddr, uint8_t regaddr, uint16_t phydata)
{
  uint32_t regval;

  /* Set PHY address and PHY register address */

  regval = ((uint32_t)phyaddr << ETH_MADR_PHYADDR_SHIFT) |
           ((uint32_t)regaddr << ETH_MADR_REGADDR_SHIFT);
  lpc17_putreg(regval, LPC17_ETH_MADR);

  /* Set up to write */

  lpc17_putreg(ETH_MCMD_WRITE, LPC17_ETH_MCMD);

  /* Write the register data to the PHY */

  lpc17_putreg((uint32_t)phydata, LPC17_ETH_MWTD);

  /* Wait for the PHY command to complete */

  while ((lpc17_getreg(LPC17_ETH_MIND) & ETH_MIND_BUSY) != 0);
}
#endif

/****************************************************************************
 * Function: lpc17_phyread
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

#ifdef LPC17_HAVE_PHY
static uint16_t lpc17_phyread(uint8_t phyaddr, uint8_t regaddr)
{
  uint32_t regval;

  lpc17_putreg(0, LPC17_ETH_MCMD);

  /* Set PHY address and PHY register address */

  regval = ((uint32_t)phyaddr << ETH_MADR_PHYADDR_SHIFT) |
           ((uint32_t)regaddr << ETH_MADR_REGADDR_SHIFT);
  lpc17_putreg(regval, LPC17_ETH_MADR);

  /* Set up to read */

  lpc17_putreg(ETH_MCMD_READ, LPC17_ETH_MCMD);

  /* Wait for the PHY command to complete */

  while ((lpc17_getreg(LPC17_ETH_MIND) & (ETH_MIND_BUSY | ETH_MIND_NVALID)) != 0);
  lpc17_putreg(0, LPC17_ETH_MCMD);

  /* Return the PHY register data */

  return (uint16_t)(lpc17_getreg(LPC17_ETH_MRDD) & ETH_MRDD_MASK);
}
#endif

/****************************************************************************
 * Function: lpc17_phyreset
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

#ifdef LPC17_HAVE_PHY
static inline int lpc17_phyreset(uint8_t phyaddr)
{
  int32_t timeout;
  uint16_t phyreg;

  /* Reset the PHY.  Needs a minimal 50uS delay after reset. */

  lpc17_phywrite(phyaddr, MII_MCR, MII_MCR_RESET);

  /* Wait for a minimum of 50uS no matter what */

  up_udelay(50);

  /* The MCR reset bit is self-clearing.  Wait for it to be clear indicating
   * that the reset is complete.
   */

  for (timeout = MII_BIG_TIMEOUT; timeout > 0; timeout--)
    {
      phyreg = lpc17_phyread(phyaddr, MII_MCR);
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
 * Function: lpc17_phyautoneg
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
 *   The adverisement regiser has already been configured.
 *
 ****************************************************************************/

#if defined(LPC17_HAVE_PHY) && defined(CONFIG_LPC17_PHY_AUTONEG)
static inline int lpc17_phyautoneg(uint8_t phyaddr)
{
  int32_t timeout;
  uint16_t phyreg;

  /* Start auto-negotiation */

  lpc17_phywrite(phyaddr, MII_MCR, MII_MCR_ANENABLE | MII_MCR_ANRESTART);

  /* Wait for autonegotiation to complete */

  for (timeout = MII_BIG_TIMEOUT; timeout > 0; timeout--)
    {
      /* Check if auto-negotiation has completed */

      phyreg = lpc17_phyread(phyaddr, MII_MSR);
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
 * Function: lpc17_phymode
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

#ifdef LPC17_HAVE_PHY
static int lpc17_phymode(uint8_t phyaddr, uint8_t mode)
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
  if ((mode & LPC17_SPEED_MASK) ==  LPC17_SPEED_100)
    {
      phyreg = MII_MCR_SPEED100;
    }

  if ((mode & LPC17_DUPLEX_MASK) == LPC17_DUPLEX_FULL)
    {
      phyreg |= MII_MCR_FULLDPLX;
    }

  lpc17_phywrite(phyaddr, MII_MCR, phyreg);

  /* Then wait for the link to be established */

  for (timeout = MII_BIG_TIMEOUT; timeout > 0; timeout--)
    {
      /* REVISIT:  This should not depend explicity on the board configuration.
       * Rather, there should be some additional configuration option to
       * suppress this DP83848C-specific behavior.
       */

#if defined(CONFIG_ETH0_PHY_DP83848C) && !defined(CONFIG_ARCH_BOARD_MBED)
      phyreg = lpc17_phyread(phyaddr, MII_DP83848C_STS);
      if ((phyreg & 0x0001) != 0)
        {
          /* Yes.. return success */

          return OK;
        }
#else
      phyreg = lpc17_phyread(phyaddr, MII_MSR);
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
 * Function: lpc17_phyinit
 *
 * Description:
 *   Initialize the PHY
 *
 * Input Parameters:
 *   priv - Pointer to EMAC device driver structure
 *
 * Returned Value:
 *   None directly.  As a side-effect, it will initialize priv->lp_phyaddr
 *   and priv->lp_phymode.
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef LPC17_HAVE_PHY
static inline int lpc17_phyinit(struct lpc17_driver_s *priv)
{
  unsigned int phyaddr;
  uint16_t phyreg;
  uint32_t regval;
  int ret;

  /* MII configuration: host clocked divided per board.h, no suppress
   * preamble, no scan increment.
   */

  lpc17_putreg(ETH_MCFG_CLKSEL_DIV, LPC17_ETH_MCFG);
  lpc17_putreg(0, LPC17_ETH_MCMD);

  /* Enter RMII mode and select 100 MBPS support */

  lpc17_putreg(ETH_CMD_RMII, LPC17_ETH_CMD);
  lpc17_putreg(ETH_SUPP_SPEED, LPC17_ETH_SUPP);

  /* Find PHY Address.  Because the controller has a pull-up and the
   * PHY has pull-down resistors on RXD lines some times the PHY
   * latches different at different addresses.
   */

  for (phyaddr = 1; phyaddr < 32; phyaddr++)
    {
      /* Check if we can see the selected device ID at this
       * PHY address.
       */

      phyreg = (unsigned int)lpc17_phyread(phyaddr, MII_PHYID1);
      ninfo("Addr: %d PHY ID1: %04x\n", phyaddr, phyreg);

      /* Compare OUI bits 3-18 */

      if (phyreg == LPC17_PHYID1)
        {
          phyreg = lpc17_phyread(phyaddr, MII_PHYID2);
          ninfo("Addr: %d PHY ID2: %04x\n", phyaddr, phyreg);

          /* Compare OUI bits 19-24 and the 6-bit model number (ignoring the
           * 4-bit revision number).
           */

          if ((phyreg & 0xfff0) == (LPC17_PHYID2 & 0xfff0))
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

  priv->lp_phyaddr = phyaddr;

  /* Reset the PHY */

  ret = lpc17_phyreset(phyaddr);
  if (ret < 0)
    {
      return ret;
    }
  lpc17_showmii(phyaddr, "After reset");

  /* Check for preamble suppression support */

  phyreg = lpc17_phyread(phyaddr, MII_MSR);
  if ((phyreg & MII_MSR_MFRAMESUPPRESS) != 0)
    {
      /* The PHY supports preamble suppression */

      regval  = lpc17_getreg(LPC17_ETH_MCFG);
      regval |= ETH_MCFG_SUPPRE;
      lpc17_putreg(regval, LPC17_ETH_MCFG);
    }

  /* Are we configured to do auto-negotiation? */

#ifdef CONFIG_LPC17_PHY_AUTONEG
  /* Setup the Auto-negotiation advertisement: 100 or 10, and HD or FD */

  lpc17_phywrite(phyaddr, MII_ADVERTISE,
                 (MII_ADVERTISE_100BASETXFULL | MII_ADVERTISE_100BASETXHALF |
                  MII_ADVERTISE_10BASETXFULL  | MII_ADVERTISE_10BASETXHALF  |
                  MII_ADVERTISE_CSMA));

  /* Then perform the auto-negotiation */

  ret = lpc17_phyautoneg(phyaddr);
  if (ret < 0)
    {
      return ret;
    }
#else
  /* Set up the fixed PHY configuration */

  ret = lpc17_phymode(phyaddr, LPC17_MODE_DEFLT);
  if (ret < 0)
    {
      return ret;
    }
#endif

  /* The link is established */

  lpc17_showmii(phyaddr, "After link established");

  /* Check configuration */

#if defined(CONFIG_ETH0_PHY_KS8721)
  phyreg = lpc17_phyread(phyaddr, MII_KS8721_10BTCR);

  switch (phyreg & KS8721_10BTCR_MODE_MASK)
    {
      case KS8721_10BTCR_MODE_10BTHD:  /* 10BASE-T half duplex */
        priv->lp_mode = LPC17_10BASET_HD;
        lpc17_putreg(0, LPC17_ETH_SUPP);
        break;

      case KS8721_10BTCR_MODE_100BTHD: /* 100BASE-T half duplex */
        priv->lp_mode = LPC17_100BASET_HD;
        break;

      case KS8721_10BTCR_MODE_10BTFD: /* 10BASE-T full duplex */
        priv->lp_mode = LPC17_10BASET_FD;
        lpc17_putreg(0, LPC17_ETH_SUPP);
        break;

      case KS8721_10BTCR_MODE_100BTFD: /* 100BASE-T full duplex */
        priv->lp_mode = LPC17_100BASET_FD;
        break;

      default:
        nerr("ERROR: Unrecognized mode: %04x\n", phyreg);
        return -ENODEV;
    }

#elif defined(CONFIG_ETH0_PHY_KSZ8041)
  phyreg = lpc17_phyread(phyaddr, MII_KSZ8041_PHYCTRL2);

  switch (phyreg & MII_PHYCTRL2_MODE_MASK)
    {
      case MII_PHYCTRL2_MODE_10HDX:  /* 10BASE-T half duplex */
        priv->lp_mode = LPC17_10BASET_HD;
        lpc17_putreg(0, LPC17_ETH_SUPP);
        break;

      case MII_PHYCTRL2_MODE_100HDX: /* 100BASE-T half duplex */
        priv->lp_mode = LPC17_100BASET_HD;
        break;

      case MII_PHYCTRL2_MODE_10FDX: /* 10BASE-T full duplex */
        priv->lp_mode = LPC17_10BASET_FD;
        lpc17_putreg(0, LPC17_ETH_SUPP);
        break;

      case MII_PHYCTRL2_MODE_100FDX: /* 100BASE-T full duplex */
        priv->lp_mode = LPC17_100BASET_FD;
        break;

      default:
        nerr("ERROR: Unrecognized mode: %04x\n", phyreg);
        return -ENODEV;
    }

#elif defined(CONFIG_ETH0_PHY_DP83848C)
  phyreg = lpc17_phyread(phyaddr, MII_DP83848C_STS);

  /* Configure for full/half duplex mode and speed */

  switch (phyreg & 0x0006)
    {
      case 0x0000:
        priv->lp_mode = LPC17_100BASET_HD;
        break;

      case 0x0002:
        priv->lp_mode = LPC17_10BASET_HD;
        break;

      case 0x0004:
        priv->lp_mode = LPC17_100BASET_FD;
        break;

      case 0x0006:
        priv->lp_mode = LPC17_10BASET_FD;
        break;

      default:
        nerr("ERROR: Unrecognized mode: %04x\n", phyreg);
        return -ENODEV;
    }

#elif defined(CONFIG_ETH0_PHY_LAN8720)
  {
    uint16_t advertise;
    uint16_t lpa;

    up_udelay(500);
    advertise = lpc17_phyread(phyaddr, MII_ADVERTISE);
    lpa       = lpc17_phyread(phyaddr, MII_LPA);

    /* Check for 100BASETX full duplex */

    if ((advertise & MII_ADVERTISE_100BASETXFULL) != 0 &&
        (lpa & MII_LPA_100BASETXFULL) != 0)
      {
        priv->lp_mode = LPC17_100BASET_FD;
      }

    /* Check for 100BASETX half duplex */

    else if ((advertise & MII_ADVERTISE_100BASETXHALF) != 0 &&
        (lpa & MII_LPA_100BASETXHALF) != 0)
      {
        priv->lp_mode = LPC17_100BASET_HD;
      }

    /* Check for 10BASETX full duplex */

    else if ((advertise & MII_ADVERTISE_10BASETXFULL) != 0 &&
        (lpa & MII_LPA_10BASETXFULL) != 0)
      {
        priv->lp_mode = LPC17_10BASET_FD;
      }

    /* Check for 10BASETX half duplex */

    else if ((advertise & MII_ADVERTISE_10BASETXHALF) != 0 &&
        (lpa & MII_LPA_10BASETXHALF) != 0)
      {
        priv->lp_mode = LPC17_10BASET_HD;
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
        (priv->lp_mode & LPC17_SPEED_MASK) ==  LPC17_SPEED_100 ? 100 : 10,
        (priv->lp_mode & LPC17_DUPLEX_MASK) == LPC17_DUPLEX_FULL ?"full" : "half");

  /* Disable auto-configuration.  Set the fixed speed/duplex mode.
   * (probably more than little redundant).
   *
   * REVISIT: Revisit the following CONFIG_PHY_CEMENT_DISABLE work-around.
   * It is should not needed if CONFIG_LPC17_PHY_AUTONEG is defined and is known
   * cause a problem for at least one PHY (DP83848I PHY).  It might be
   * safe just to remove this elided coded for all PHYs.
   */

#ifndef CONFIG_PHY_CEMENT_DISABLE
  ret = lpc17_phymode(phyaddr, priv->lp_mode);
#endif
  lpc17_showmii(phyaddr, "After final configuration");
  return ret;
}
#else
static inline int lpc17_phyinit(struct lpc17_driver_s *priv)
{
  priv->lp_mode = LPC17_MODE_DEFLT;
  return OK;
}
#endif

/****************************************************************************
 * Function: lpc17_txdescinit
 *
 * Description:
 *   Initialize the EMAC Tx descriptor table
 *
 * Input Parameters:
 *   priv - Pointer to EMAC device driver structure
 *
 * Returned Value:
 *   None directory.
 *   As a side-effect, it will initialize priv->lp_phyaddr and
 *   priv->lp_phymode.
 *
 * Assumptions:
 *
 ****************************************************************************/

static inline void lpc17_txdescinit(struct lpc17_driver_s *priv)
{
  uint32_t *txdesc;
  uint32_t *txstat;
  uint32_t pktaddr;
  int i;

  /* Configure Tx descriptor and status tables */

  lpc17_putreg(LPC17_TXDESC_BASE, LPC17_ETH_TXDESC);
  lpc17_putreg(LPC17_TXSTAT_BASE, LPC17_ETH_TXSTAT);
  lpc17_putreg(CONFIG_LPC17_ETH_NTXDESC-1, LPC17_ETH_TXDESCRNO);

  /* Initialize Tx descriptors and link to packet buffers */

  txdesc  = (uint32_t *)LPC17_TXDESC_BASE;
  pktaddr = LPC17_TXBUFFER_BASE;

  for (i = 0; i < CONFIG_LPC17_ETH_NTXDESC; i++)
    {
      *txdesc++ = pktaddr;
      *txdesc++ = (TXDESC_CONTROL_INT | (LPC17_MAXPACKET_SIZE - 1));
      pktaddr  += LPC17_MAXPACKET_SIZE;
    }

  /* Initialize Tx status */

  txstat  = (uint32_t *)LPC17_TXSTAT_BASE;
  for (i = 0; i < CONFIG_LPC17_ETH_NTXDESC; i++)
    {
      *txstat++ = 0;
    }

  /* Point to first Tx descriptor */

  lpc17_putreg(0, LPC17_ETH_TXPRODIDX);
}

/****************************************************************************
 * Function: lpc17_rxdescinit
 *
 * Description:
 *   Initialize the EMAC Rx descriptor table
 *
 * Input Parameters:
 *   priv - Pointer to EMAC device driver structure
 *
 * Returned Value:
 *   None directory.
 *   As a side-effect, it will initialize priv->lp_phyaddr and
 *   priv->lp_phymode.
 *
 * Assumptions:
 *
 ****************************************************************************/

static inline void lpc17_rxdescinit(struct lpc17_driver_s *priv)
{
  uint32_t *rxdesc;
  uint32_t *rxstat;
  uint32_t pktaddr;
  int i;

  /* Configure Rx descriptor and status tables */

  lpc17_putreg(LPC17_RXDESC_BASE, LPC17_ETH_RXDESC);
  lpc17_putreg(LPC17_RXSTAT_BASE, LPC17_ETH_RXSTAT);
  lpc17_putreg(CONFIG_LPC17_ETH_NRXDESC-1, LPC17_ETH_RXDESCNO);

  /* Initialize Rx descriptors and link to packet buffers */

  rxdesc  = (uint32_t *)LPC17_RXDESC_BASE;
  pktaddr = LPC17_RXBUFFER_BASE;

  for (i = 0; i < CONFIG_LPC17_ETH_NRXDESC; i++)
    {
      *rxdesc++ = pktaddr;
      *rxdesc++ = (RXDESC_CONTROL_INT | (LPC17_MAXPACKET_SIZE - 1));
      pktaddr  += LPC17_MAXPACKET_SIZE;
    }

  /* Initialize Rx status */

  rxstat  = (uint32_t *)LPC17_RXSTAT_BASE;
  for (i = 0; i < CONFIG_LPC17_ETH_NRXDESC; i++)
    {
      *rxstat++ = 0;
      *rxstat++ = 0;
    }

  /* Point to first Rx descriptor */

  lpc17_putreg(0, LPC17_ETH_RXCONSIDX);
}

/****************************************************************************
 * Function: lpc17_macmode
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

#ifdef LPC17_HAVE_PHY
static void lpc17_macmode(uint8_t mode)
{
  uint32_t regval;

  /* Set up for full or half duplex operation */

  if ((mode & LPC17_DUPLEX_MASK) == LPC17_DUPLEX_FULL)
    {
      /* Set the back-to-back inter-packet gap */

      lpc17_putreg(21, LPC17_ETH_IPGT);

      /* Set MAC to operate in full duplex mode with CRC and Pad enabled */

      regval = lpc17_getreg(LPC17_ETH_MAC2);
      regval |= (ETH_MAC2_FD | ETH_MAC2_CRCEN | ETH_MAC2_PADCRCEN);
      lpc17_putreg(regval, LPC17_ETH_MAC2);

      /* Select full duplex operation for ethernet controller */

      regval = lpc17_getreg(LPC17_ETH_CMD);
      regval |= (ETH_CMD_FD | ETH_CMD_RMII | ETH_CMD_PRFRAME);
      lpc17_putreg(regval, LPC17_ETH_CMD);
    }
  else
    {
      /* Set the back-to-back inter-packet gap */

      lpc17_putreg(18, LPC17_ETH_IPGT);

      /* Set MAC to operate in half duplex mode with CRC and Pad enabled */

      regval = lpc17_getreg(LPC17_ETH_MAC2);
      regval &= ~ETH_MAC2_FD;
      regval |= (ETH_MAC2_CRCEN | ETH_MAC2_PADCRCEN);
      lpc17_putreg(regval, LPC17_ETH_MAC2);

      /* Select half duplex operation for ethernet controller */

      regval = lpc17_getreg(LPC17_ETH_CMD);
      regval &= ~ETH_CMD_FD;
      regval |= (ETH_CMD_RMII | ETH_CMD_PRFRAME);
      lpc17_putreg(regval, LPC17_ETH_CMD);
    }

  /* This is currently done in lpc17_phyinit().  That doesn't
   * seem like the right place. It should be done here.
   */

#if 0
  regval = lpc17_getreg(LPC17_ETH_SUPP);
  if ((mode & LPC17_SPEED_MASK) == LPC17_SPEED_100)
    {
      regval |= ETH_SUPP_SPEED;
    }
  else
    {
      regval &= ~ETH_SUPP_SPEED;
    }
  lpc17_putreg(regval, LPC17_ETH_SUPP);
#endif
}
#endif

/****************************************************************************
 * Function: lpc17_ethreset
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

static void lpc17_ethreset(struct lpc17_driver_s *priv)
{
  irqstate_t flags;

  /* Reset the MAC */

  flags = enter_critical_section();

  /* Put the MAC into the reset state */

  lpc17_putreg((ETH_MAC1_TXRST    | ETH_MAC1_MCSTXRST | ETH_MAC1_RXRST |
                ETH_MAC1_MCSRXRST | ETH_MAC1_SIMRST   | ETH_MAC1_SOFTRST),
               LPC17_ETH_MAC1);

  /* Disable RX/RX, clear modes, reset all control registers */

  lpc17_putreg((ETH_CMD_REGRST | ETH_CMD_TXRST | ETH_CMD_RXRST),
               LPC17_ETH_CMD);

  /* Take the MAC out of the reset state */

  up_udelay(50);
  lpc17_putreg(0, LPC17_ETH_MAC1);

  /* The RMII bit must be set on initialization (I'm not sure this needs
   * to be done here but... oh well).
   */

  lpc17_putreg(ETH_CMD_RMII, LPC17_ETH_CMD);

  /* Set other misc configuration-related registers to default values */

  lpc17_putreg(0, LPC17_ETH_MAC2);
  lpc17_putreg(0, LPC17_ETH_SUPP);
  lpc17_putreg(0, LPC17_ETH_TEST);

  lpc17_putreg(18, LPC17_ETH_IPGR);
  lpc17_putreg(((15 << ETH_CLRT_RMAX_SHIFT) | (55 << ETH_CLRT_COLWIN_SHIFT)),
               LPC17_ETH_CLRT);

  /* Set the Maximum Frame size register. "This field resets to the value
   * 0x0600, which represents a maximum receive frame of 1536 octets. An
   * untagged maximum size Ethernet frame is 1518 octets. A tagged frame adds
   * four octets for a total of 1522 octets. If a shorter maximum length
   * restriction is desired, program this 16-bit field."
   */

  lpc17_putreg(LPC17_MAXPACKET_SIZE, LPC17_ETH_MAXF);

  /* Disable all Ethernet controller interrupts */

  lpc17_putreg(0, LPC17_ETH_INTEN);

  /* Clear any pending interrupts (shouldn't be any) */

  lpc17_putreg(0xffffffff, LPC17_ETH_INTCLR);
  leave_critical_section(flags);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: lpc17_ethinitialize
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

#if CONFIG_LPC17_NINTERFACES > 1
int lpc17_ethinitialize(int intf)
#else
static inline int lpc17_ethinitialize(int intf)
#endif
{
  struct lpc17_driver_s *priv;
  uint8_t *pktbuf;
  uint32_t regval;
  int ret;
  int i;

  DEBUGASSERT(intf < CONFIG_LPC17_NINTERFACES);
  priv = &g_ethdrvr[intf];

  /* Turn on the ethernet MAC clock */

  regval  = lpc17_getreg(LPC17_SYSCON_PCONP);
  regval |= SYSCON_PCONP_PCENET;
  lpc17_putreg(regval, LPC17_SYSCON_PCONP);

  /* Configure all GPIO pins needed by ENET */

  for (i = 0; i < GPIO_NENET_PINS; i++)
    {
      (void)lpc17_configgpio(g_enetpins[i]);
    }

  lpc17_showpins();

  /* Select the packet buffer */

  pktbuf = &g_pktbuf[PKTBUF_SIZE * intf];

  /* Initialize the driver structure */

  memset(priv, 0, sizeof(struct lpc17_driver_s));
  priv->lp_dev.d_buf     = pktbuf;        /* Single packet buffer */
  priv->lp_dev.d_ifup    = lpc17_ifup;    /* I/F down callback */
  priv->lp_dev.d_ifdown  = lpc17_ifdown;  /* I/F up (new IP address) callback */
  priv->lp_dev.d_txavail = lpc17_txavail; /* New TX data callback */
#ifdef CONFIG_NET_IGMP
  priv->lp_dev.d_addmac  = lpc17_addmac;  /* Add multicast MAC address */
  priv->lp_dev.d_rmmac   = lpc17_rmmac;   /* Remove multicast MAC address */
#endif
  priv->lp_dev.d_private = (void *)priv;  /* Used to recover private state from dev */

#if CONFIG_LPC17_NINTERFACES > 1
# error "A mechanism to associate base address an IRQ with an interface is needed"
  priv->lp_base          = ??;            /* Ethernet controller base address */
  priv->lp_irq           = ??;            /* Ethernet controller IRQ number */
#endif

  /* Create a watchdog for timing polling for and timing of transmissions */

  priv->lp_txpoll        = wd_create();   /* Create periodic poll timer */
  priv->lp_txtimeout     = wd_create();   /* Create TX timeout timer */

  /* Reset the Ethernet controller and leave in the ifdown statue.  The
   * Ethernet controller will be properly re-initialized each time
   * lpc17_ifup() is called.
   */

  lpc17_ifdown(&priv->lp_dev);

  /* Attach the IRQ to the driver */

#if CONFIG_LPC17_NINTERFACES > 1
  ret = irq_attach(priv->irq, lpc17_interrupt, NULL);
#else
  ret = irq_attach(LPC17_IRQ_ETH, lpc17_interrupt, NULL);
#endif
  if (ret != 0)
    {
      /* We could not attach the ISR to the interrupt */

      return -EAGAIN;
    }

  /* Register the device with the OS so that socket IOCTLs can be performed */

  (void)netdev_register(&priv->lp_dev, NET_LL_ETHERNET);
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

#if CONFIG_LPC17_NINTERFACES == 1
void up_netinitialize(void)
{
  (void)lpc17_ethinitialize(0);
}
#endif
#endif /* LPC17_NETHCONTROLLERS > 0 */
#endif /* CONFIG_NET && CONFIG_LPC17_ETHERNET */
