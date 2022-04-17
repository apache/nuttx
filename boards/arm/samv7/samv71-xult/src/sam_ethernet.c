/****************************************************************************
 * boards/arm/samv7/samv71-xult/src/sam_ethernet.c
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

/* Force verbose debug on in this file only to support unit-level testing. */

#ifdef CONFIG_NETDEV_PHY_DEBUG
#  undef  CONFIG_DEBUG_INFO
#  define CONFIG_DEBUG_INFO 1
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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SAMV7_EMAC0_DEVNAME "eth0"

#define AT24XX_MACADDR_OFFSET 0x9a

/* Debug ********************************************************************/

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

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_emac0_phy_enable
 ****************************************************************************/

#ifdef CONFIG_SAMV7_GPIOA_IRQ
static void sam_emac0_phy_enable(bool enable)
{
  phyinfo("IRQ%d: enable=%d\n", IRQ_EMAC0_INT, enable);
  if (enable)
    {
      sam_gpioirqenable(IRQ_EMAC0_INT);
    }
  else
    {
      sam_gpioirqdisable(IRQ_EMAC0_INT);
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_netinitialize
 *
 * Description:
 *   Configure board resources to support networking.
 *
 ****************************************************************************/

void weak_function sam_netinitialize(void)
{
  /* Configure the PHY interrupt GPIO */

  phyinfo("Configuring %08x\n", GPIO_EMAC0_INT);
  sam_configgpio(GPIO_EMAC0_INT);

  /* Configure the PHY SIGDET input */

  sam_configgpio(GPIO_EMAC0_SIGDET);

  /* Configure PHY /RESET output */

  sam_configgpio(GPIO_EMAC0_RESET);
}

/****************************************************************************
 * Name: sam_emac0_setmac
 *
 * Description:
 *   Read the Ethernet MAC address from the AT24 FLASH and configure the
 *   Ethernet driver with that address.
 *
 ****************************************************************************/

#ifdef HAVE_MACADDR
int sam_emac0_setmac(void)
{
  struct i2c_master_s *i2c;
  struct mtd_dev_s *at24;
  uint8_t mac[6];
  ssize_t nread;
  int ret;

  /* Get an instance of the TWI0 interface */

  i2c = sam_i2cbus_initialize(0);
  if (!i2c)
    {
      nerr("ERROR: Failed to initialize TWI0\n");
      return -ENODEV;
    }

  /* Initialize the AT24 driver */

  at24 = at24c_initialize(i2c);
  if (!at24)
    {
      nerr("ERROR: Failed to initialize the AT24 driver\n");
      sam_i2cbus_uninitialize(i2c);
      return -ENODEV;
    }

  /* Configure the AT24 to access the extended memory region */

  ret = at24->ioctl(at24, MTDIOC_EXTENDED, 1);
  if (ret < 0)
    {
      nerr("ERROR: AT24 ioctl(MTDIOC_EXTENDED) failed: %d\n", ret);
      sam_i2cbus_uninitialize(i2c);
      return ret;
    }

  /* Read the MAC address */

  nread = at24->read(at24, AT24XX_MACADDR_OFFSET, 6, mac);
  if (nread < 6)
    {
      nerr("ERROR: AT24 read(AT24XX_MACADDR_OFFSET) failed: %zd\n",
           nread);
      sam_i2cbus_uninitialize(i2c);
      return (int)nread;
    }

  /* Put the AT24 back in normal memory access mode */

  ret = at24->ioctl(at24, MTDIOC_EXTENDED, 0);
  if (ret < 0)
    {
      nerr("ERROR: AT24 ioctl(MTDIOC_EXTENDED) failed: %d\n", ret);
    }

  /* Release the I2C instance.
   * REVISIT:  Need an interface to release the AT24 instance too
   */

  ret = sam_i2cbus_uninitialize(i2c);
  if (ret < 0)
    {
      nerr("ERROR: Failed to release the I2C interface: %d\n", ret);
    }

  ninfo("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  /* Now configure the EMAC driver to use this MAC address */

  ret = sam_emac_setmacaddr(EMAC0_INTF, mac);
  if (ret < 0)
    {
      nerr("ERROR: Failed to set MAC address: %d\n", ret);
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

#ifdef CONFIG_SAMV7_GPIOA_IRQ
int arch_phy_irq(const char *intf, xcpt_t handler, void *arg,
                 phy_enable_t *enable)
{
  irqstate_t flags;
  gpio_pinset_t pinset;
  phy_enable_t enabler;
  int irq;

  DEBUGASSERT(intf);

  ninfo("%s: handler=%p\n", intf, handler);
  phyinfo("EMAC0: devname=%s\n", SAMV7_EMAC0_DEVNAME);

  if (strcmp(intf, SAMV7_EMAC0_DEVNAME) == 0)
    {
      phyinfo("Select EMAC0\n");

      pinset   = GPIO_EMAC0_INT;
      irq      = IRQ_EMAC0_INT;
      enabler  = sam_emac0_phy_enable;
    }
  else
    {
      nerr("ERROR: Unsupported interface: %s\n", intf);
      return -EINVAL;
    }

  /* Disable interrupts until we are done.  This guarantees that the
   * following operations are atomic.
   */

  flags = enter_critical_section();

  /* Configure the interrupt */

  if (handler)
    {
      phyinfo("Configure pin: %08x\n", pinset);
      sam_gpioirq(pinset);

      phyinfo("Attach IRQ%d\n", irq);
      irq_attach(irq, handler, arg);
    }
  else
    {
      phyinfo("Detach IRQ%d\n", irq);
      irq_detach(irq);
      enabler = NULL;
    }

  /* Return with the interrupt disabled in either case */

  sam_gpioirqdisable(irq);

  /* Return the enabling function pointer */

  if (enable)
    {
      *enable = enabler;
    }

  /* Return the old handler (so that it can be restored) */

  leave_critical_section(flags);
  return OK;
}
#endif /* CONFIG_SAMV7_GPIOA_IRQ */

#endif /* HAVE_NETWORK */
