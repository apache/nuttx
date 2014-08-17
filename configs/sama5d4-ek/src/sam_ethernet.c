/************************************************************************************
 * configs/sama5d4-ek/src/sam_ethernet.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/* Force verbose debug on in this file only to support unit-level testing. */

#ifdef CONFIG_NETDEV_PHY_DEBUG
#  undef  CONFIG_DEBUG_VERBOSE
#  define CONFIG_DEBUG_VERBOSE 1
#  undef  CONFIG_DEBUG_NET
#  define CONFIG_DEBUG_NET 1
#endif

#include <string.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "sam_pio.h"
#include "sam_ethernet.h"

#include "sama5d4-ek.h"

#ifdef HAVE_NETWORK

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#ifndef CONFIG_SAMA5_EMAC0
#  undef CONFIG_SAMA5_EMAC0_ISETH0
#endif

#ifdef CONFIG_SAMA5_EMAC0_ISETH0
#  define SAMA5_EMAC0_DEVNAME "eth0"
#  define SAMA5_EMAC1_DEVNAME "eth1"
#else
#  define SAMA5_EMAC0_DEVNAME "eth1"
#  define SAMA5_EMAC1_DEVNAME "eth0"
#endif

/* Debug ********************************************************************/
/* Extra, in-depth debug output that is only available if
 * CONFIG_NETDEV_PHY_DEBUG us defined.
 */

#ifdef CONFIG_NETDEV_PHY_DEBUG
#  define phydbg    dbg
#  define phylldbg  lldbg
#else
#  define phydbg(x...)
#  define phylldbg(x...)
#endif

/************************************************************************************
 * Private Data
 ************************************************************************************/

#ifdef CONFIG_SAMA5_PIOE_IRQ
#ifdef CONFIG_SAMA5_EMAC0
static xcpt_t g_emac0_handler;
#endif
#ifdef CONFIG_SAMA5_EMAC1
static xcpt_t g_emac1_handler;
#endif
#endif

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: sam_netinitialize
 *
 * Description:
 *   Configure board resources to support networking.
 *
 ************************************************************************************/

void weak_function sam_netinitialize(void)
{
#ifdef CONFIG_SAMA5_EMAC0
  phydbg("Configuring %08x\n", PIO_INT_ETH0);
  sam_configpio(PIO_INT_ETH0);
#endif

#ifdef CONFIG_SAMA5_EMAC1
  phydbg("Configuring %08x\n", PIO_INT_ETH1);
  sam_configpio(PIO_INT_ETH1);
#endif
}

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
 *      PHY interrupt.
 *   b. When the PHY interrupt occurs, work should be scheduled on the
 *      worker thread (or perhaps a dedicated application thread).
 *   c. That worker thread should use the SIOCGMIIPHY, SIOCGMIIREG,
 *      and SIOCSMIIREG ioctl calls** to communicate with the PHY,
 *      determine what network event took place (Link Up/Down?), and
 *      take the appropriate actions.
 *
 *    * This is an OS internal interface and should not be used from
 *      application space.
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
 *
 * Returned Value:
 *   The previous PHY interrupt handler address is returned.  This allows you
 *   to temporarily replace an interrupt handler, then restore the original
 *   interrupt handler.  NULL is returned if there is was not handler in
 *   place when the call was made.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_PIOE_IRQ
xcpt_t arch_phy_irq(FAR const char *intf, xcpt_t handler)
{
  irqstate_t flags;
  xcpt_t *phandler;
  xcpt_t oldhandler;
  pio_pinset_t pinset;
  int irq;

  DEBUGASSERT(intf);

  nvdbg("%s: handler=%p\n", intf, handler);
#ifdef CONFIG_SAMA5_EMAC0
  phydbg("EMAC0: devname=%s\n", SAMA5_EMAC0_DEVNAME);
#endif
#ifdef CONFIG_SAMA5_EMAC0
  phydbg("EMAC1: devname=%s\n", SAMA5_EMAC1_DEVNAME);
#endif

#ifdef CONFIG_SAMA5_EMAC0
  if (strcmp(intf, SAMA5_EMAC0_DEVNAME) == 0)
    {
      phydbg("Select EMAC0\n");
      phandler = &g_emac0_handler;
      pinset   = PIO_INT_ETH0;
      irq      = IRQ_INT_ETH0;
    }
  else
#endif
#ifdef CONFIG_SAMA5_EMAC1
  if (strcmp(intf, SAMA5_EMAC1_DEVNAME) == 0)
    {
      phydbg("Select EMAC1\n");
      phandler = &g_emac1_handler;
      pinset   = PIO_INT_ETH1;
      irq      = IRQ_INT_ETH1;
    }
  else
#endif
    {
      ndbg("Unsupported interface: %s\n", intf);
      return NULL;
    }

  /* Disable interrupts until we are done.  This guarantees that the
   * following operations are atomic.
   */

  flags = irqsave();

  /* Get the old button interrupt handler and save the new one */

  oldhandler = *phandler;
  *phandler  = handler;

  /* Configure the interrupt */

  if (handler)
    {
      phydbg("Configure pin: %08x\n", pinset);
      sam_pioirq(pinset);

      phydbg("Enable IRQ: %d\n", irq);
      (void)irq_attach(irq, handler);
      sam_pioirqenable(irq);
    }
  else
    {
      phydbg("Disable IRQ: %d\n", irq);
      (void)irq_detach(irq);
      sam_pioirqdisable(irq);
    }

  /* Return the old button handler (so that it can be restored) */

  irqrestore(flags);
  return oldhandler;
}
#endif /* CONFIG_SAMA5_PIOE_IRQ */

#endif /* HAVE_NETWORK */
