/****************************************************************************
 * arch/arm/src/lm3s/lm3s_ethernet.c
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
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
#if defined(CONFIG_NET) && defined(CONFIG_LM3S_ETHERNET)

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

#include "up_arch.h"
#include "chip.h"
#include "lm3s_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

#ifdef CONFIG_ARCH_CHIP_LM3S6918
#  define LM3S_NINTERFACES 1
#else
#  error "No Ethernet support for this LM3S chip"
#endif

/* TX poll deley = 1 seconds. CLK_TCK is the number of clock ticks per second */

#define LM3S_WDDELAY   (1*CLK_TCK)
#define LM3S_POLLHSEC  (1*2)

/* TX timeout = 1 minute */

#define LM3S_TXTIMEOUT (60*CLK_TCK)

/* This is a helper pointer for accessing the contents of the Ethernet header */

#define BUF ((struct uip_eth_hdr *)priv->ld_dev.d_buf)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The lm3s_driver_s encapsulates all state information for a single hardware
 * interface
 */

struct lm3s_driver_s
{
  /* The following fields would only be necessary on chips that support
   * multiple Ethernet controllers.
   */

#if LM3S_NINTERFACES > 1
  uint32  ld_base;             /* Ethernet controller base address */
  int     ld-irq;              /* Ethernet controller IRQ */
#endif

  boolean ld_bifup;            /* TRUE:ifup FALSE:ifdown */
  WDOG_ID ld_txpoll;           /* TX poll timer */
  WDOG_ID ld_txtimeout;        /* TX timeout timer */

  /* This holds the information visible to uIP/NuttX */

  struct uip_driver_s ld_dev;  /* Interface understood by uIP */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct lm3s_driver_s g_lm3sdev[LM3S_NINTERFACES];

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Miscellaneous low level helpers */

#if LM3S_NINTERFACES > 1
static uint32 lm3s_ethin(struct lm3s_driver_s *priv, int offset);
static void   lm3s_ethout(struct lm3s_driver_s *priv, int offset, uint32 value);
#else
static inline uint32 lm3s_ethin(struct lm3s_driver_s *priv, int offset);
static inline void lm3s_ethout(struct lm3s_driver_s *priv, int offset, uint32 value);
#endif
static void lm3s_ethreset(struct lm3s_driver_s *priv);
static void lm3s_phywrite(struct lm3s_driver_s *priv, int regaddr, uint16 value);
static uint16 lm3s_phyread(struct lm3s_driver_s *priv, int regaddr);

/* Common TX logic */

static int  lm3s_transmit(struct lm3s_driver_s *priv);
static int  lm3s_uiptxpoll(struct uip_driver_s *dev);

/* Interrupt handling */

static void lm3s_receive(struct lm3s_driver_s *priv);
static void lm3s_txdone(struct lm3s_driver_s *priv);
static int  lm3s_interrupt(int irq, FAR void *context);

/* Watchdog timer expirations */

static void lm3s_polltimer(int argc, uint32 arg, ...);
static void lm3s_txtimeout(int argc, uint32 arg, ...);

/* NuttX callback functions */

static int lm3s_ifup(struct uip_driver_s *dev);
static int lm3s_ifdown(struct uip_driver_s *dev);
static int lm3s_txavail(struct uip_driver_s *dev);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: lm3s_ethin
 *
 * Description:
 *   Read a register from the Ethernet module
 *
 * Parameters:
 *   priv   - Reference to the driver state structure
 *   offset - Byte offset of the register from the ethernet base address
 *
 * Returned Value:
 *   Register value
 *
 ****************************************************************************/

#if LM3S_NINTERFACES > 1
static uint32 lm3s_ethin(struct lm3s_driver_s *priv, int offset)
{
  return getreg32(priv->ld_base + offset);
}
#else
static inline uint32 lm3s_ethin(struct lm3s_driver_s *priv, int offset)
{
  return getreg32(LM3S_ETHCON_BASE + offset);
}
#endif

/****************************************************************************
 * Function: lm3s_ethout
 *
 * Description:
 *   Write a register to the Ethernet module
 *
 * Parameters:
 *   priv   - Reference to the driver state structure
 *   offset - Byte offset of the register from the ethernet base address
 *   value  - The value to write the the Ethernet register
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if LM3S_NINTERFACES > 1
static void lm3s_ethout(struct lm3s_driver_s *priv, int offset, uint32 value)
{
  putreg32(value, priv->ld_base + offset);
}
#else
static inline void lm3s_ethout(struct lm3s_driver_s *priv, int offset, uint32 value)
{
  putreg32(value, LM3S_ETHCON_BASE + offset);
}
#endif

/****************************************************************************
 * Function: lm3s_ethreset
 *
 * Description:
 *   Configure and reset the Ethernet module, leaving it in a disabled state.
 *
 * Parameters:
 *   priv   - Reference to the driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *
 ****************************************************************************/

static void lm3s_ethreset(struct lm3s_driver_s *priv)
{
  irqstate_t flags;
  uint32 regval;
  volatile uint32 delay;
  uint32 div;

#if LM3S_NINTERFACES > 1
#  error "If multiple interfaces are supported, this function would have to be redesigned"
#endif

  /* Make sure that clocking is enabled for the Ethernet (and PHY) peripherals */

  flags   = irqsave();
  regval  = getreg32(LM3S_SYSCON_RCGC2);
  regval |= (SYSCON_RCGC2_EMAC0|SYSCON_RCGC2_EPHY0);
  putreg32(regval, LM3S_SYSCON_RCGC2);
  nvdbg("RCGC2: %08x\n", regval);

  /* Put the Ethernet controller into the reset state */

  regval = getreg32(LM3S_SYSCON_SRCR2);
  regval |= (SYSCON_SRCR2_EMAC0|SYSCON_SRCR2_EPHY0);
  putreg32(regval, LM3S_SYSCON_SRCR2);

  /* Wait just a bit */

  for (delay = 0; delay < 16; delay++);

  /* Then take the Ethernet controller out of the reset state */

  regval &= ~(SYSCON_SRCR2_EMAC0|SYSCON_SRCR2_EPHY0);
  putreg32(regval, LM3S_SYSCON_SRCR2);
  nvdbg("SRCR2: %08x\n", regval);

  /* Enable Port F for Ethernet LEDs: LED0=Bit 3; LED1=Bit 2 */

#ifdef CONFIG_LM3S_ETHLEDS
  /* Configure the pins for the peripheral function */

  lm3s_configgpio(GPIO_ETHPHY_LED0 | GPIO_STRENGTH_2MA | GPIO_PADTYPE_STD);
  lm3s_configgpio(GPIO_ETHPHY_LED1 | GPIO_STRENGTH_2MA | GPIO_PADTYPE_STD);
#endif

  /* Disable all Ethernet controller interrupts */

  regval = lm3s_ethin(priv, LM3S_MAC_IM_OFFSET);
  regval &= ~MAC_IM_ALLINTS;
  lm3s_ethout(priv, LM3S_MAC_IM_OFFSET, regval);

  /* Clear any pending interrupts (shouldn't be any) */

  regval = lm3s_ethin(priv, LM3S_MAC_RIS_OFFSET);
  lm3s_ethout(priv, LM3S_MAC_IACK_OFFSET, regval);

  /* Set the management clock divider register for access to the PHY
   * register set. The MDC clock is divided down from the system clock per:
   *
   *   MCLK_FREQUENCY = SYSCLK_FREQUENCY / (2 * (div + 1))
   *   div = (SYSCLK_FREQUENCY / 2 / MCLK_FREQUENCY) - 1
   *
   * Where MCLK_FREQUENCY is 2,500,000.  We will add 1 to assure the max
   * MCLK_FREQUENCY is not exceeded.
   */

  div = SYSCLK_FREQUENCY / 2 / 2500000;
  lm3s_ethout(priv, LM3S_MAC_MDV_OFFSET, div);
  nvdbg("MDV:   %08x\n", div);
  irqrestore(flags);
}

/****************************************************************************
 * Function: lm3s_phywrite
 *
 * Description:
 *   Write a 16-bit word to a PHY register
 *
 * Parameters:
 *   priv    - Reference to the driver state structure
 *   regaddr - Address of the PHY register to write
 *   value   - The value to write to the register
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lm3s_phywrite(struct lm3s_driver_s *priv, int regaddr, uint16 value)
{
# warning "Missing Logic"
}

/****************************************************************************
 * Function: lm3s_phywrite
 *
 * Description:
 *   Write a 16-bit word to a PHY register
 *
 * Parameters:
 *   priv    - Reference to the driver state structure
 *   regaddr - Address of the PHY register to write
 *   value   - The value to write to the register
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static uint16 lm3s_phyread(struct lm3s_driver_s *priv, int regaddr)
{
# warning "Missing Logic"
  return 0;
}

/****************************************************************************
 * Function: lm3s_transmit
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
 ****************************************************************************/

static int lm3s_transmit(struct lm3s_driver_s *priv)
{
  /* Verify that the hardware is ready to send another packet */
#warning "Missing logic"
  /* Increment statistics */

  /* Disable Ethernet interrupts */

  /* Send the packet: address=priv->ld_dev.d_buf, length=priv->ld_dev.d_len */

  /* Restore Ethernet interrupts */

  /* Setup the TX timeout watchdog (perhaps restarting the timer) */

  (void)wd_start(priv->ld_txtimeout, LM3S_TXTIMEOUT, lm3s_txtimeout, 1, (uint32)priv);
  return OK;
}

/****************************************************************************
 * Function: lm3s_uiptxpoll
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

static int lm3s_uiptxpoll(struct uip_driver_s *dev)
{
  struct lm3s_driver_s *priv = (struct lm3s_driver_s *)dev->d_private;

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  if (priv->ld_dev.d_len > 0)
    {
      uip_arp_out(&priv->ld_dev);
      lm3s_transmit(priv);

      /* Check if there is room in the device to hold another packet. If not,
       * return a non-zero value to terminate the poll.
       */
#warning "Missing logic"
    }

  /* If zero is returned, the polling will continue until all connections have
   * been examined.
   */

  return 0;
}

/****************************************************************************
 * Function: lm3s_receive
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

static void lm3s_receive(struct lm3s_driver_s *priv)
{
  do
    {
      /* Check for errors and update statistics */
#warning "Missing logic"

      /* Check if the packet is a valid size for the uIP buffer configuration */

      /* Copy the data data from the hardware to priv->ld_dev.d_buf.  Set
       * amount of data in priv->ld_dev.d_len
       */

      /* We only accept IP packets of the configured type and ARP packets */

#ifdef CONFIG_NET_IPv6
      if (BUF->type == HTONS(UIP_ETHTYPE_IP6))
#else
      if (BUF->type == HTONS(UIP_ETHTYPE_IP))
#endif
        {
          uip_arp_ipin();
          uip_input(&priv->ld_dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, the field  d_len will set to a value > 0.
           */

          if (priv->ld_dev.d_len > 0)
            {
              uip_arp_out(&priv->ld_dev);
              lm3s_transmit(priv);
            }
        }
      else if (BUF->type == htons(UIP_ETHTYPE_ARP))
        {
          uip_arp_arpin(&priv->ld_dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, the field  d_len will set to a value > 0.
           */

           if (priv->ld_dev.d_len > 0)
             {
               lm3s_transmit(priv);
             }
        }
    }
  while ( /* FIX ME */ TRUE /* FIX ME */); /* While there are more packets to be processed */
}

/****************************************************************************
 * Function: lm3s_txdone
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

static void lm3s_txdone(struct lm3s_driver_s *priv)
{
  /* Check for errors and update statistics */
#warning "Missing logic"

  /* If no further xmits are pending, then cancel the TX timeout */

  wd_cancel(priv->ld_txtimeout);

  /* Then poll uIP for new XMIT data */

  (void)uip_poll(&priv->ld_dev, lm3s_uiptxpoll);
}

/****************************************************************************
 * Function: lm3s_interrupt
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

static int lm3s_interrupt(int irq, FAR void *context)
{
  register struct lm3s_driver_s *priv;

#if LM3S_NINTERFACES > 1
# error "A mechanism to associate and interface with an IRQ is needed"
#else
  priv = &g_lm3sdev[0];
#endif

  /* Disable Ethernet interrupts */
#warning "Missing logic"

  /* Get and clear interrupt status bits */

  /* Handle interrupts according to status bit settings */

  /* Check if we received an incoming packet, if so, call lm3s_receive() */

  lm3s_receive(priv);

  /* Check is a packet transmission just completed.  If so, call lm3s_txdone */

  lm3s_txdone(priv);

  /* Enable Ethernet interrupts (perhaps excluding the TX done interrupt if 
   * there are no pending transmissions.
   */

  return OK;
}

/****************************************************************************
 * Function: lm3s_txtimeout
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

static void lm3s_txtimeout(int argc, uint32 arg, ...)
{
  struct lm3s_driver_s *priv = (struct lm3s_driver_s *)arg;

  /* Increment statistics and dump debug info */
#warning "Missing logic"

  /* Then reset the hardware */

  /* Then poll uIP for new XMIT data */

  (void)uip_poll(&priv->ld_dev, lm3s_uiptxpoll);
}

/****************************************************************************
 * Function: lm3s_polltimer
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

static void lm3s_polltimer(int argc, uint32 arg, ...)
{
  struct lm3s_driver_s *priv = (struct lm3s_driver_s *)arg;

  /* Check if there is room in the send another Tx packet.  */
#warning "Missing logic"

  /* If so, update TCP timing states and poll uIP for new XMIT data */

  (void)uip_timer(&priv->ld_dev, lm3s_uiptxpoll, LM3S_POLLHSEC);

  /* Setup the watchdog poll timer again */

  (void)wd_start(priv->ld_txpoll, LM3S_WDDELAY, lm3s_polltimer, 1, arg);
}

/****************************************************************************
 * Function: lm3s_ifup
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

static int lm3s_ifup(struct uip_driver_s *dev)
{
  struct lm3s_driver_s *priv = (struct lm3s_driver_s *)dev->d_private;
  irqstate_t flags;
  uint32 regval;
  uint16 phyreg;

  ndbg("Bringing up: %d.%d.%d.%d\n",
       dev->d_ipaddr & 0xff, (dev->d_ipaddr >> 8) & 0xff,
       (dev->d_ipaddr >> 16) & 0xff, dev->d_ipaddr >> 24 );

  /* Enable and reset the Ethernet controller */

  flags = irqsave();
  lm3s_ethreset(priv);

  /* Then configure the Ethernet Controller for normal operation 
   *
   * Setup the transmit control register (Full duplex, TX CRC Auto Generation,
   * TX Padding Enabled).
   */

# warning "These should be configurable items"
  regval  = lm3s_ethin(priv, LM3S_MAC_TCTL_OFFSET);
  regval |= (MAC_TCTL_DUPLEX|MAC_TCTL_CRC|MAC_TCTL_PADEN);
  lm3s_ethout(priv, LM3S_MAC_TCTL_OFFSET, regval);
  nvdbg("TCTL:  %08x\n", regval);

  /* Setup the receive control register (Disable multicast frames, disable
   * promiscuous mode, disable bad CRC rejection). >>> These should be configurable items <<<
   */

# warning "These should be configurable items"
  regval  = lm3s_ethin(priv, LM3S_MAC_RCTL_OFFSET);
  regval &= ~(MAC_RCTL_BADCRC | MAC_RCTL_PRMS | MAC_RCTL_AMUL);
  lm3s_ethout(priv, LM3S_MAC_RCTL_OFFSET, regval);
  nvdbg("RCTL:  %08x\n", regval);

  /* Setup the time stamp configuration register */

#ifndef CONFIG_ARCH_CHIP_LM3S6918
  regval = lm3s_ethin(priv, LM3S_MAC_TS_OFFSET);
#ifdef CONFIG_LM3S_TIMESTAMP
  regval |= MAC_TS_EN;
#else
  regval &= ~(MAC_TS_EN);
#endif
  lm3s_ethout(priv, LM3S_MAC_TS_OFFSET, regval);
  nvdbg("TS:    %08x\n", regval);
#endif

  /* Wait for the link to come up */

# warning "This should use a semaphore and an interrupt"

  ndbg("Waiting for link\n");
  do
    {
      phyreg = lm3s_phyread(priv, MII_MSR);
    }
  while ((phyreg & MII_MSR_LINKSTATUS) == 0);
  ndbg("Link established\n");

  /* Reset the receive FIFO */

  regval  = lm3s_ethin(priv, LM3S_MAC_RCTL_OFFSET);
  regval |= MAC_RCTL_RSTFIFO;
  lm3s_ethout(priv, LM3S_MAC_RCTL_OFFSET, regval);

  /* Enable the Ethernet receiver */

  regval = lm3s_ethin(priv, LM3S_MAC_RCTL_OFFSET);
  regval  |= MAC_RCTL_RXEN;
  lm3s_ethout(priv, LM3S_MAC_RCTL_OFFSET, regval);

  /* Enable the Ethernet transmitter */

  regval  = lm3s_ethin(priv, LM3S_MAC_RCTL_OFFSET);
  regval |= MAC_TCTL_TXEN;
  lm3s_ethout(priv, LM3S_MAC_TCTL_OFFSET, regval);

  /* Reset the receive FIFO (again) */

  regval  = lm3s_ethin(priv, LM3S_MAC_RCTL_OFFSET);
  regval |= MAC_RCTL_RSTFIFO;
  lm3s_ethout(priv, LM3S_MAC_RCTL_OFFSET, regval);

  /* Enable the Ethernet interrupt */

#if LM3S_NINTERFACES > 1
  up_enable_irq(priv->irq);
#else
  up_enable_irq(LM3S_IRQ_ETHCON);
#endif

  /* Enable the Ethernet RX packet receipt interrupt */

  regval = lm3s_ethin(priv, LM3S_MAC_IM_OFFSET);
  regval |= MAC_IM_RXINTM;
  lm3s_ethout(priv, LM3S_MAC_IM_OFFSET, regval);

  /* Program the hardware with it's MAC address (for filtering) */

  regval = (uint32)priv->ld_dev.d_mac.ether_addr_octet[3] << 24 |
           (uint32)priv->ld_dev.d_mac.ether_addr_octet[2] << 16 |
           (uint32)priv->ld_dev.d_mac.ether_addr_octet[1] << 8 |
           (uint32)priv->ld_dev.d_mac.ether_addr_octet[0];
  lm3s_ethout(priv, LM3S_MAC_IA0_OFFSET, regval);

  regval = (uint32)priv->ld_dev.d_mac.ether_addr_octet[5] << 8 |
           (uint32)priv->ld_dev.d_mac.ether_addr_octet[4];
  lm3s_ethout(priv, LM3S_MAC_IA1_OFFSET, regval);

  /* Set and activate a timer process */

  (void)wd_start(priv->ld_txpoll, LM3S_WDDELAY, lm3s_polltimer, 1, (uint32)priv);

  priv->ld_bifup = TRUE;
  irqrestore(flags);
  return OK;
}

/****************************************************************************
 * Function: lm3s_ifdown
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

static int lm3s_ifdown(struct uip_driver_s *dev)
{
  struct lm3s_driver_s *priv = (struct lm3s_driver_s *)dev->d_private;
  irqstate_t flags;

  /* Disable the Ethernet interrupt */

  flags = irqsave();
#if LM3S_NINTERFACES > 1
  up_disable_irq(priv->irq);
#else
  up_disable_irq(LM3S_IRQ_ETHCON);
#endif

  /* Cancel the TX poll timer and TX timeout timers */

  wd_cancel(priv->ld_txpoll);
  wd_cancel(priv->ld_txtimeout);

  /* Reset the device */

  priv->ld_bifup = FALSE;
  irqrestore(flags);
  return OK;
}

/****************************************************************************
 * Function: lm3s_txavail
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

static int lm3s_txavail(struct uip_driver_s *dev)
{
  struct lm3s_driver_s *priv = (struct lm3s_driver_s *)dev->d_private;
  irqstate_t flags;

  flags = irqsave();

  /* Ignore the notification if the interface is not yet up */

  if (priv->ld_bifup)
    {
      /* Check if there is room in the hardware to hold another outgoing packet. */
#warning "Missing logic"

      /* If so, then poll uIP for new XMIT data */

      (void)uip_poll(&priv->ld_dev, lm3s_uiptxpoll);
    }

  irqrestore(flags);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: lm3s_initialize
 *
 * Description:
 *   Initialize the Ethernet driver for one interface
 *
 * Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

/* Initialize the Ethernet controller and driver */

#if LM3S_NINTERFACES > 1
int lm3s_initialize(int intf)
#else
static inline int lm3s_initialize(int intf)
#endif
{
  struct lm3s_driver_s *priv = &g_lm3sdev[intf];
  int ret;

  /* Check if the Ethernet module is present */

#if LM3S_NINTERFACES > 1
# error "This debug check only works with one interface"
#else
  DEBUGASSERT((getreg32(LM3S_SYSCON_DC4) & (SYSCON_DC4_EMAC0|SYSCON_DC4_EPHY0)) == (SYSCON_DC4_EMAC0|SYSCON_DC4_EPHY0));
#endif
  DEBUGASSERT((unsigned)intf < LM3S_NINTERFACES);

  /* Initialize the driver structure */

  memset(priv, 0, sizeof(struct lm3s_driver_s));
  priv->ld_dev.d_ifup    = lm3s_ifup;     /* I/F down callback */
  priv->ld_dev.d_ifdown  = lm3s_ifdown;   /* I/F up (new IP address) callback */
  priv->ld_dev.d_txavail = lm3s_txavail;  /* New TX data callback */
  priv->ld_dev.d_private = (void*)&g_lm3sdev[0]; /* Used to recover private state from dev */

  /* Create a watchdog for timing polling for and timing of transmisstions */

#if LM3S_NINTERFACES > 1
# error "A mechanism to associate base address an IRQ with an interface is needed"
  priv->ld_base          = ??;            /* Ethernet controller base address */
  priv->ld_irq           = ??;            /* Ethernet controller IRQ number */
#endif
  priv->ld_txpoll        = wd_create();   /* Create periodic poll timer */
  priv->ld_txtimeout     = wd_create();   /* Create TX timeout timer */

  /* If the board can provide us with a MAC address, get the address
   * from the board now.  The MAC will not be applied until lm3s_ifup()
   * is caleld (and the MAC can be overwritten with a netdev ioctl call). 
   */

#ifdef CONFIG_LM3S_BOARDMAC
   lm3s_ethernetmac(&priv->ld_dev.d_mac);
#endif

  /* Perform minimal, one-time initialization -- just reset the controller and
   * leave it disabled.  The Ethernet controller will be reset and properly
   * re-initialized each time lm3s_ifup() is called.
   */

  lm3s_ethreset(priv);

  /* Attach the IRQ to the driver */

#if LM3S_NINTERFACES > 1
  ret = irq_attach(priv->irq, lm3s_interrupt);
#else
  ret = irq_attach(LM3S_IRQ_ETHCON, lm3s_interrupt);
#endif
  if (ret != 0)
    {
      /* We could not attach the ISR to the IRQ */

      return -EAGAIN;
    }

  /* Register the device with the OS so that socket IOCTLs can be performed */

  (void)netdev_register(&priv->ld_dev);
  return OK;
}


/************************************************************************************
 * Name: up_netinitialize
 *
 * Description:
 *   Initialize the first network interface
 *
 ************************************************************************************/

void up_netinitialize(void)
{
  (void)lm3s_initialize(0);
}

#endif /* CONFIG_NET && CONFIG_LM3S_ETHERNET */

