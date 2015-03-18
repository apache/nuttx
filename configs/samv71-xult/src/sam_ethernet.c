/************************************************************************************
 * configs/samv71-xult/src/sam_ethernet.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/mtd/mtd.h>

#include "sam_gpio.h"
#include "sam_twihs.h"
#include "sam_ethernet.h"

#include "samv71-xult.h"

#ifdef HAVE_NETWORK

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#define SAMV7_EMAC0_DEVNAME "eth0"

#define AT24XX_MACADDR_OFFSET 0x9a

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

#ifdef CONFIG_SAMV7_PIOA_IRQ
static xcpt_t g_emac0_handler;
#endif

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Name: sam_emac0_phy_enable
 ************************************************************************************/

#ifdef CONFIG_SAMV7_PIOA_IRQ
static void sam_emac0_phy_enable(bool enable)
{
  phydbg("IRQ%d: enable=%d\n", IRQ_EMAC0_INT, enable);
  if (enable)
    {
      sam_pioirqenable(IRQ_EMAC0_INT);
    }
  else
    {
      sam_pioirqdisable(IRQ_EMAC0_INT);
    }
}
#endif

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
  /* Configure the PHY interrupt GPIO */

  phydbg("Configuring %08x\n", GPIO_EMAC0_INT);
  sam_configgpio(GPIO_EMAC0_INT);

  /* Configure the PHY SIGDET input */

  sam_configgpio(GPIO_EMAC0_SIGDET);

  /* Configure PHY /RESET output */

  sam_configgpio(GPIO_EMAC0_RESET);
}

/************************************************************************************
 * Name: sam_emac0_setmac
 *
 * Description:
 *   Read the Ethernet MAC address from the AT24 FLASH and configure the Ethernet
 *   driver with that address.
 *
 ************************************************************************************/

#ifdef HAVE_MACADDR
int sam_emac0_setmac(void)
{
  struct i2c_dev_s *i2c;
  struct mtd_dev_s *at24;
  uint8_t mac[6];
  ssize_t nread;
  int ret;

  /* Get an instance of the TWI0 interface */

  i2c = up_i2cinitialize(0);
  if (!i2c)
    {
      ndbg("ERROR: Failed to initialize TWI0\n");
      return -ENODEV;
    }

  /* Initialize the AT24 driver */

  at24 = at24c_initialize(i2c);
  if (!at24)
    {
      ndbg("ERROR: Failed to initialize the AT24 driver\n");
      (void)up_i2cuninitialize(i2c);
      return -ENODEV;
    }

  /* Configure the AT24 to access the extended memory region */

  ret = at24->ioctl(at24, MTDIOC_EXTENDED, 1);
  if (ret < 0)
    {
      ndbg("ERROR: AT24 ioctl(MTDIOC_EXTENDED) failed: %d\n", ret);
      (void)up_i2cuninitialize(i2c);
      return ret;
    }

  /* Read the MAC address */

  nread = at24->read(at24, AT24XX_MACADDR_OFFSET, 6, mac);
  if (nread < 6)
    {
      ndbg("ERROR: AT24 read(AT24XX_MACADDR_OFFSET) failed: ld\n", (long)nread);
      (void)up_i2cuninitialize(i2c);
      return (int)nread;
    }

  /* Put the AT24 back in normal memory access mode */

  ret = at24->ioctl(at24, MTDIOC_EXTENDED, 0);
  if (ret < 0)
    {
      ndbg("ERROR: AT24 ioctl(MTDIOC_EXTENDED) failed: %d\n", ret);
    }

  /* Release the I2C instance.
   * REVISIT:  Need an interface to release the AT24 instance too
   */

  ret = up_i2cuninitialize(i2c);
  if (ret < 0)
    {
      ndbg("ERROR: Failed to release the I2C interface: %d\n", ret);
    }

  nvdbg("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  /* Now configure the EMAC driver to use this MAC address */

  ret = sam_emac_setmacaddr(EMAC0_INTF, mac);
  if (ret < 0)
    {
      ndbg("ERROR: Failed to set MAC address: %d\n", ret);
    }

  return ret;
}
#else
#  define sam_emac0_setmac()
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
 *   d. It should then interact the the PHY to clear any pending
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
 *   enable  - A function pointer that be unsed to enable or disable the
 *             PHY interrupt.
 *
 * Returned Value:
 *   The previous PHY interrupt handler address is returned.  This allows you
 *   to temporarily replace an interrupt handler, then restore the original
 *   interrupt handler.  NULL is returned if there is was not handler in
 *   place when the call was made.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMV7_PIOA_IRQ
xcpt_t arch_phy_irq(FAR const char *intf, xcpt_t handler, phy_enable_t *enable)
{
  irqstate_t flags;
  xcpt_t *phandler;
  xcpt_t oldhandler;
  pio_pinset_t pinset;
  phy_enable_t enabler;
  int irq;

  DEBUGASSERT(intf);

  nvdbg("%s: handler=%p\n", intf, handler);
  phydbg("EMAC0: devname=%s\n", SAMV7_EMAC0_DEVNAME);

  if (strcmp(intf, SAMV7_EMAC0_DEVNAME) == 0)
    {
      phydbg("Select EMAC0\n");
      phandler = &g_emac0_handler;
      pinset   = GPIO_EMAC0_INT;
      irq      = IRQ_EMAC0_INT;
      enabler  = sam_emac0_phy_enable;
    }
  else
    {
      ndbg("Unsupported interface: %s\n", intf);
      return NULL;
    }

  /* Disable interrupts until we are done.  This guarantees that the
   * following operations are atomic.
   */

  flags = irqsave();

  /* Get the old interrupt handler and save the new one */

  oldhandler = *phandler;
  *phandler  = handler;

  /* Configure the interrupt */

  if (handler)
    {
      phydbg("Configure pin: %08x\n", pinset);
      sam_pioirq(pinset);

      phydbg("Attach IRQ%d\n", irq);
      (void)irq_attach(irq, handler);
    }
  else
    {
      phydbg("Detach IRQ%d\n", irq);
      (void)irq_detach(irq);
      enabler = NULL;
    }

  /* Return with the interrupt disabled in either case */

  sam_pioirqdisable(irq);

  /* Return the enabling function pointer */

  if (enable)
    {
      *enable = enabler;
    }

  /* Return the old handler (so that it can be restored) */

  irqrestore(flags);
  return oldhandler;
}
#endif /* CONFIG_SAMV7_PIOA_IRQ */

#endif /* HAVE_NETWORK */
