/****************************************************************************
 * boards/arm/stm32/stm32f4discovery/src/stm32_ethernet.c
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

#include "stm32_gpio.h"
#include "stm32_eth.h"

#include "stm32f4discovery.h"

#if defined(CONFIG_STM32F4DISBB) && defined(CONFIG_STM32_ETHMAC)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define STM32_ETHMAC_DEVNAME "eth0"

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
 * Private Data
 ****************************************************************************/

#ifdef HAVE_NETMONITOR
static xcpt_t g_ethmac_handler;
static void  *g_ethmac_arg;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_emac0_phy_enable
 ****************************************************************************/

#ifdef HAVE_NETMONITOR
static void stm32_emac0_phy_enable(bool enable)
{
  phyinfo("enable=%d\n", enable);
  if (enable && g_ethmac_handler != NULL)
    {
      /* Attach and enable GPIO interrupt (and event) on the falling edge */

      stm32_gpiosetevent(GPIO_EMAC_NINT, false, true, true,
                         g_ethmac_handler, g_ethmac_arg);
    }
  else
    {
      /* Detach and disable GPIO interrupt */

      stm32_gpiosetevent(GPIO_EMAC_NINT, false, false, false,
                         NULL, NULL);
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_netinitialize
 *
 * Description:
 *   Configure board resources to support networking.
 *
 ****************************************************************************/

void weak_function stm32_netinitialize(void)
{
#ifdef HAVE_NETMONITOR
  /* Configure the PHY interrupt GPIO */

  phyinfo("Configuring %08x\n", GPIO_EMAC_NINT);
  stm32_configgpio(GPIO_EMAC_NINT);
#endif

  /* Configure PHY /RESET output */

  stm32_configgpio(GPIO_EMAC_NRST);
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

#ifdef HAVE_NETMONITOR
int arch_phy_irq(const char *intf, xcpt_t handler, void *arg,
                 phy_enable_t *enable)
{
  phy_enable_t enabler;
  irqstate_t flags;

  ninfo("%s: handler=%p\n", intf, handler);
  phyinfo("ETHMAC: devname=%s\n", STM32_ETHMAC_DEVNAME);

  DEBUGASSERT(intf);

  flags = enter_critical_section();

  if (strcmp(intf, STM32_ETHMAC_DEVNAME) == 0)
    {
      phyinfo("Select ETHMAC\n");
      g_ethmac_handler = handler;
      g_ethmac_arg     = arg;
      enabler          = stm32_emac0_phy_enable;
    }
  else
    {
      nerr("ERROR: Unsupported interface: %s\n", intf);
      enabler          = NULL;
    }

  if (enable)
    {
      *enable = enabler;
    }

  leave_critical_section(flags);
  return OK;
}
#endif

#endif /* CONFIG_STM32F4DISBB && CONFIG_STM32_ETHMAC */
