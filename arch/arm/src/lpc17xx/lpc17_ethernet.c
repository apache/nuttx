/****************************************************************************
 * arch/arm/src/lpc17xx/lpc17_ethernet.c
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
#include <wdog.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include <net/uip/uip.h>
#include <net/uip/uip-arp.h>
#include <net/uip/uip-arch.h>

#include "chip.h"
#include "up_arch.h"
#include "lpc17_syscon.h"
#include "lpc17_ethernet.h"
#include "lpc17_internal.h"

#include <arch/board/board.h>

/* Does this chip have and ethernet controller? */

#if LPC17_NETHCONTROLLERS > 0

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* CONFIG_LPC17_NINTERFACES determines the number of physical interfaces
 * that will be supported.
 */

#if !defined(CONFIG_LPC17_NINTERFACES) || CONFIG_LPC17_NINTERFACES > LPC17_NETHCONTROLLERS
# define CONFIG_LPC17_NINTERFACES LPC17_NETHCONTROLLERS
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

/* TX poll deley = 1 seconds. CLK_TCK is the number of clock ticks per second */

#define LPC17_WDDELAY   (1*CLK_TCK)
#define LPC17_POLLHSEC  (1*2)

/* TX timeout = 1 minute */

#define LPC17_TXTIMEOUT (60*CLK_TCK)

/* This is a helper pointer for accessing the contents of the Ethernet header */

#define BUF ((struct uip_eth_hdr *)priv->lp_dev.d_buf)

/* This is the number of ethernet GPIO pins that must be configured */

#define GPIO_NENET_PINS 12

/* Register debug */

#ifndef CONFIG_DEBUG
#  undef  CONFIG_LPC17_ENET_REGDEBUG
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The lpc17_driver_s encapsulates all state information for a single hardware
 * interface
 */

struct lpc17_driver_s
{
  bool    lp_bifup;            /* true:ifup false:ifdown */
  WDOG_ID lp_txpoll;           /* TX poll timer */
  WDOG_ID lp_txtimeout;        /* TX timeout timer */

  /* This holds the information visible to uIP/NuttX */

  struct uip_driver_s lp_dev;  /* Interface understood by uIP */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Array of ethernet driver status structures */

static struct lpc17_driver_s g_ethdrvr[CONFIG_LPC17_NINTERFACES];

/* ENET pins are on P1[0,1,4,6,8,9,10,14,15] + MDC on P1[16] or P2[8] and
 * MDIO on P1[17] or P2[9].  The board.h file will define GPIO_ENET_MDC and
 * PGIO_ENET_MDIO to selec which pin setting to use.
 *
 * On older Rev '-' devices, P1[6] ENET-TX_CLK would also have be to configured.
 */

static uint16_t g_enetpins[GPIO_NENET_PINS] =
{
  GPIO_ENET_TXD0, GPIO_ENET_TXD1, GPIO_ENET_TXEN,   GPIO_ENET_CRS, GPIO_ENET_RXD0,
  GPIO_ENET_RXD1, GPIO_ENET_RXER, GPIO_ENET_REFCLK, GPIO_ENET_MDC, GPIO_ENET_MDIO
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register operations ********************************************************/

#ifdef CONFIG_LPC17_ENET_REGDEBUG
static void lpc17_printreg(uint32_t addr, uint32_t val, bool iswrite);
static void lpc17_checkreg(uint32_t addr, uint32_t val, bool iswrite);
static uint32_t lpc17_getreg(uint32_t addr);
static void lpc17_putreg(uint32_t val, uint32_t addr);
static void lpc17_showpins(void);
#else
# define lpc17_getreg(addr)     getreg32(addr)
# define lpc17_putreg(val,addr) putreg32(val,addr)
# define lpc17_showpins()
#endif

/* Common TX logic */

static int  lpc17_transmit(FAR struct lpc17_driver_s *priv);
static int  lpc17_uiptxpoll(struct uip_driver_s *dev);

/* Interrupt handling */

static void lpc17_receive(FAR struct lpc17_driver_s *priv);
static void lpc17_txdone(FAR struct lpc17_driver_s *priv);
static int  lpc17_interrupt(int irq, FAR void *context);

/* Watchdog timer expirations */

static void lpc17_polltimer(int argc, uint32_t arg, ...);
static void lpc17_txtimeout(int argc, uint32_t arg, ...);

/* NuttX callback functions */

static int lpc17_ifup(struct uip_driver_s *dev);
static int lpc17_ifdown(struct uip_driver_s *dev);
static int lpc17_txavail(struct uip_driver_s *dev);
#ifdef CONFIG_NET_IGMP
static int lpc17_addmac(struct uip_driver_s *dev, FAR const uint8_t *mac);
static int lpc17_rmmac(struct uip_driver_s *dev, FAR const uint8_t *mac);
#endif

/* Initialization functions */

static void lpc17_phywrite(uint8_t phyaddr, uint8_t regaddr, uint16_t phydata);
static uint16_t lpc17_phyread(uint8_t phyaddr, uint8_t regaddr);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/*******************************************************************************
 * Name: lpc17_printreg
 *
 * Description:
 *   Print the contents of an LPC17xx register operation
 *
 *******************************************************************************/

#ifdef CONFIG_LPC17_ENET_REGDEBUG
static void lpc17_printreg(uint32_t addr, uint32_t val, bool iswrite)
{
  lldbg("%08x%s%08x\n", addr, iswrite ? "<-" : "->", val);
}
#endif

/*******************************************************************************
 * Name: lpc17_checkreg
 *
 * Description:
 *   Get the contents of an LPC17xx register
 *
 *******************************************************************************/

#ifdef CONFIG_LPC17_ENET_REGDEBUG
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

              lldbg("[repeats %d more times]\n", count);
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

/*******************************************************************************
 * Name: lpc17_getreg
 *
 * Description:
 *   Get the contents of an LPC17xx register
 *
 *******************************************************************************/

#ifdef CONFIG_LPC17_ENET_REGDEBUG
static uint32_t lpc17_getreg(uint32_t addr)
{
  /* Read the value from the register */

  uint32_t val = getreg32(addr);

  /* Check if we need to print this value */

  lpc17_checkreg(addr, val, false);
  return val;
}
#endif

/*******************************************************************************
 * Name: lpc17_putreg
 *
 * Description:
 *   Set the contents of an LPC17xx register to a value
 *
 *******************************************************************************/

#ifdef CONFIG_LPC17_ENET_REGDEBUG
static void lpc17_putreg(uint32_t val, uint32_t addr)
{
  /* Check if we need to print this value */

  lpc17_checkreg(addr, val, true);

  /* Write the value */

  putreg32(val, addr);
}
#endif

/*******************************************************************************
 * Name: lpc17_showpins
 *
 * Description:
 *   Dump GPIO register
 *
 *******************************************************************************/

#ifdef CONFIG_LPC17_ENET_REGDEBUG
static void lpc17_showpins(void)
{
  lpc17_dumpgpio(GPIO_PORT0|GPIO_PIN0, "P0[1-15]");
  lpc17_dumpgpio(GPIO_PORT0|GPIO_PIN16, "P0[16-31]");
}
#endif

/****************************************************************************
 * Function: lpc17_transmit
 *
 * Description:
 *   Start hardware transmission.  Called either from the txdone interrupt
 *   handling or from watchdog based polling.
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *
 ****************************************************************************/

static int lpc17_transmit(FAR struct lpc17_driver_s *priv)
{
  /* Verify that the hardware is ready to send another packet.  If we get
   * here, then we are committed to sending a packet; Higher level logic
   * must have assured that there is not transmission in progress.
   */

  /* Increment statistics */

  /* Disable Ethernet interrupts */

  /* Send the packet: address=priv->lp_dev.d_buf, length=priv->lp_dev.d_len */

  /* Restore Ethernet interrupts */

  /* Setup the TX timeout watchdog (perhaps restarting the timer) */

  (void)wd_start(priv->lp_txtimeout, LPC17_TXTIMEOUT, lpc17_txtimeout, 1, (uint32_t)priv);
  return OK;
}

/****************************************************************************
 * Function: lpc17_uiptxpoll
 *
 * Description:
 *   The transmitter is available, check if uIP has any outgoing packets ready
 *   to send.  This is a callback from uip_poll().  uip_poll() may be called:
 *
 *   1. When the preceding TX packet send is complete,
 *   2. When the preceding TX packet send timesout and the interface is reset
 *   3. During normal TX polling
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *
 ****************************************************************************/

static int lpc17_uiptxpoll(struct uip_driver_s *dev)
{
  FAR struct lpc17_driver_s *priv = (FAR struct lpc17_driver_s *)dev->d_private;

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  if (priv->lp_dev.d_len > 0)
    {
      uip_arp_out(&priv->lp_dev);
      lpc17_transmit(priv);

      /* Check if there is room in the device to hold another packet. If not,
       * return a non-zero value to terminate the poll.
       */
    }

  /* If zero is returned, the polling will continue until all connections have
   * been examined.
   */

  return 0;
}

/****************************************************************************
 * Function: lpc17_receive
 *
 * Description:
 *   An interrupt was received indicating the availability of a new RX packet
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void lpc17_receive(FAR struct lpc17_driver_s *priv)
{
  do
    {
      /* Check for errors and update statistics */

      /* Check if the packet is a valid size for the uIP buffer configuration */

      /* Copy the data data from the hardware to priv->lp_dev.d_buf.  Set
       * amount of data in priv->lp_dev.d_len
       */

      /* We only accept IP packets of the configured type and ARP packets */

#ifdef CONFIG_NET_IPv6
      if (BUF->type == HTONS(UIP_ETHTYPE_IP6))
#else
      if (BUF->type == HTONS(UIP_ETHTYPE_IP))
#endif
        {
          uip_arp_ipin();
          uip_input(&priv->lp_dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, the field  d_len will set to a value > 0.
           */

          if (priv->lp_dev.d_len > 0)
           {
             uip_arp_out(&priv->lp_dev);
             lpc17_transmit(priv);
           }
        }
      else if (BUF->type == htons(UIP_ETHTYPE_ARP))
        {
          uip_arp_arpin(&priv->lp_dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, the field  d_len will set to a value > 0.
           */

          if (priv->lp_dev.d_len > 0)
            {
              lpc17_transmit(priv);
            }
        }
    }
  while (1); /* While there are more packets to be processed */
}

/****************************************************************************
 * Function: lpc17_txdone
 *
 * Description:
 *   An interrupt was received indicating that the last TX packet(s) is done
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void lpc17_txdone(FAR struct lpc17_driver_s *priv)
{
  /* Check for errors and update statistics */

  /* If no further xmits are pending, then cancel the TX timeout */

  wd_cancel(priv->lp_txtimeout);

  /* Then poll uIP for new XMIT data */

  (void)uip_poll(&priv->lp_dev, lpc17_uiptxpoll);
}

/****************************************************************************
 * Function: lpc17_interrupt
 *
 * Description:
 *   Hardware interrupt handler
 *
 * Parameters:
 *   irq     - Number of the IRQ that generated the interrupt
 *   context - Interrupt register state save info (architecture-specific)
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *
 ****************************************************************************/

static int lpc17_interrupt(int irq, FAR void *context)
{
  register FAR struct lpc17_driver_s *priv = &g_ethdrvr[0];

  /* Disable Ethernet interrupts */

  /* Get and clear interrupt status bits */

  /* Handle interrupts according to status bit settings */

  /* Check if we received an incoming packet, if so, call lpc17_receive() */

  lpc17_receive(priv);

  /* Check is a packet transmission just completed.  If so, call lpc17_txdone */

  lpc17_txdone(priv);

  /* Enable Ethernet interrupts (perhaps excluding the TX done interrupt if 
   * there are no pending transmissions.
   */

  return OK;
}

/****************************************************************************
 * Function: lpc17_txtimeout
 *
 * Description:
 *   Our TX watchdog timed out.  Called from the timer interrupt handler.
 *   The last TX never completed.  Reset the hardware and start again.
 *
 * Parameters:
 *   argc - The number of available arguments
 *   arg  - The first argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void lpc17_txtimeout(int argc, uint32_t arg, ...)
{
  FAR struct lpc17_driver_s *priv = (FAR struct lpc17_driver_s *)arg;

  /* Increment statistics and dump debug info */

  /* Then reset the hardware */

  /* Then poll uIP for new XMIT data */

  (void)uip_poll(&priv->lp_dev, lpc17_uiptxpoll);
}

/****************************************************************************
 * Function: lpc17_polltimer
 *
 * Description:
 *   Periodic timer handler.  Called from the timer interrupt handler.
 *
 * Parameters:
 *   argc - The number of available arguments
 *   arg  - The first argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void lpc17_polltimer(int argc, uint32_t arg, ...)
{
  FAR struct lpc17_driver_s *priv = (FAR struct lpc17_driver_s *)arg;

  /* Check if there is room in the send another TX packet.  We cannot perform
   * the TX poll if he are unable to accept another packet for transmission.
   */

  /* If so, update TCP timing states and poll uIP for new XMIT data. Hmmm..
   * might be bug here.  Does this mean if there is a transmit in progress,
   * we will missing TCP time state updates?
   */

  (void)uip_timer(&priv->lp_dev, lpc17_uiptxpoll, LPC17_POLLHSEC);

  /* Setup the watchdog poll timer again */

  (void)wd_start(priv->lp_txpoll, LPC17_WDDELAY, lpc17_polltimer, 1, arg);
}

/****************************************************************************
 * Function: lpc17_ifup
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

static int lpc17_ifup(struct uip_driver_s *dev)
{
  FAR struct lpc17_driver_s *priv = (FAR struct lpc17_driver_s *)dev->d_private;

  ndbg("Bringing up: %d.%d.%d.%d\n",
       dev->d_ipaddr & 0xff, (dev->d_ipaddr >> 8) & 0xff,
       (dev->d_ipaddr >> 16) & 0xff, dev->d_ipaddr >> 24 );

  /* Initilize Ethernet interface */

  /* Set and activate a timer process */

  (void)wd_start(priv->lp_txpoll, LPC17_WDDELAY, lpc17_polltimer, 1, (uint32_t)priv);

  /* Enable the Ethernet interrupt */

  priv->lp_bifup = true;
  up_enable_irq(LPC17_IRQ_ETH);
  return OK;
}

/****************************************************************************
 * Function: lpc17_ifdown
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

static int lpc17_ifdown(struct uip_driver_s *dev)
{
  FAR struct lpc17_driver_s *priv = (FAR struct lpc17_driver_s *)dev->d_private;
  irqstate_t flags;

  /* Disable the Ethernet interrupt */

  flags = irqsave();
  up_disable_irq(LPC17_IRQ_ETH);

  /* Cancel the TX poll timer and TX timeout timers */

  wd_cancel(priv->lp_txpoll);
  wd_cancel(priv->lp_txtimeout);

  /* Reset the device */

  priv->lp_bifup = false;
  irqrestore(flags);
  return OK;
}

/****************************************************************************
 * Function: lpc17_txavail
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

static int lpc17_txavail(struct uip_driver_s *dev)
{
  FAR struct lpc17_driver_s *priv = (FAR struct lpc17_driver_s *)dev->d_private;
  irqstate_t flags;

  flags = irqsave();

  /* Ignore the notification if the interface is not yet up */

  if (priv->lp_bifup)
    {

      /* Check if there is room in the hardware to hold another outgoing packet. */

      /* If so, then poll uIP for new XMIT data */

      (void)uip_poll(&priv->lp_dev, lpc17_uiptxpoll);
    }

  irqrestore(flags);
  return OK;
}

/****************************************************************************
 * Function: lpc17_addmac
 *
 * Description:
 *   NuttX Callback: Add the specified MAC address to the hardware multicast
 *   address filtering
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be added 
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IGMP
static int lpc17_addmac(struct uip_driver_s *dev, FAR const uint8_t *mac)
{
  FAR struct lpc17_driver_s *priv = (FAR struct lpc17_driver_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

  return OK;
}
#endif

/****************************************************************************
 * Function: lpc17_rmmac
 *
 * Description:
 *   NuttX Callback: Remove the specified MAC address from the hardware multicast
 *   address filtering
 *
 * Parameters:
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
static int lpc17_rmmac(struct uip_driver_s *dev, FAR const uint8_t *mac)
{
  FAR struct lpc17_driver_s *priv = (FAR struct lpc17_driver_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

  return OK;
}
#endif

/****************************************************************************
 * Function: lpc17_phywrite
 *
 * Description:
 *   Write a value to an MII PHY register
 *
 * Parameters:
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

/****************************************************************************
 * Function: lpc17_phywrite
 *
 * Description:
 *   Read a value from an MII PHY register
 *
 * Parameters:
 *   phyaddr - The device address where the PHY was discovered
 *   regaddr - The address of the PHY register to be written
 *
 * Returned Value:
 *   Data read from the PHY register
 *
 * Assumptions:
 *
 ****************************************************************************/

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

  while ((lpc17_getreg(LPC17_ETH_MIND) & (ETH_MIND_BUSY|ETH_MIND_NVALID)) != 0);
  lpc17_putreg(0, LPC17_ETH_MCMD);

  /* Return the PHY register data */

  return (uint16_t)lpc17_getreg(LPC17_ETH_MRDD);
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
 * Parameters:
 *   intf - Selects the interface to be initialized.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

#if LPC17_NETHCONTROLLERS > 1
int lpc17_ethinitialize(int intf)
#else
static inline int lpc17_ethinitialize(int intf)
#endif
{
  struct lpc17_driver_s *priv = &g_ethdrvr[intf];
  uint32_t regval;
  int i;

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

  /* Attach the IRQ to the driver */

  if (irq_attach(LPC17_IRQ_ETH, lpc17_interrupt))
    {
      /* We could not attach the ISR to the the interrupt */

      return -EAGAIN;
    }

  /* Initialize the driver structure */

  memset(g_ethdrvr, 0, CONFIG_LPC17_NINTERFACES*sizeof(struct lpc17_driver_s));
  priv->lp_dev.d_ifup    = lpc17_ifup;     /* I/F down callback */
  priv->lp_dev.d_ifdown  = lpc17_ifdown;   /* I/F up (new IP address) callback */
  priv->lp_dev.d_txavail = lpc17_txavail;  /* New TX data callback */
#ifdef CONFIG_NET_IGMP
  priv->lp_dev.d_addmac  = lpc17_addmac;   /* Add multicast MAC address */
  priv->lp_dev.d_rmmac   = lpc17_rmmac;    /* Remove multicast MAC address */
#endif
  priv->lp_dev.d_private = (void*)g_ethdrvr; /* Used to recover private state from dev */

  /* Create a watchdog for timing polling for and timing of transmisstions */

  priv->lp_txpoll       = wd_create();   /* Create periodic poll timer */
  priv->lp_txtimeout    = wd_create();   /* Create TX timeout timer */

  /* Read the MAC address from the hardware into priv->lp_dev.d_mac.ether_addr_octet */

  /* Register the device with the OS so that socket IOCTLs can be performed */

  (void)netdev_register(&priv->lp_dev);
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

#if LPC17_NETHCONTROLLERS == 1
void up_netinitialize(void)
{
  (void)lpc17_ethinitialize(0);
}
#endif
#endif /* LPC17_NETHCONTROLLERS > 0 */
#endif /* CONFIG_NET && CONFIG_LPC17_ETHERNET */
