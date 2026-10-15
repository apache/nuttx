/****************************************************************************
 * boards/arm/imxrt/ddcnc-ec300/src/imxrt_ethernet.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <string.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/debug.h>
#include <nuttx/irq.h>
#include <nuttx/wdog.h>

#include "ddcnc-ec300.h"

#if defined(CONFIG_IMXRT_ENET) && defined(CONFIG_ARCH_PHY_INTERRUPT)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IMXRT_ENET_DEVNAME "eth0"

/* The LAN8720A interrupt output is not connected to the MCU, so the PHY
 * interrupt is emulated: while it is enabled, the handler is called
 * periodically and the client (usually the NETINIT monitor thread) reads
 * the PHY status registers to detect link changes.
 */

#define PHY_POLL_TICKS MSEC2TICK(CONFIG_DDCNC_EC300_PHY_POLL_MSEC)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static xcpt_t g_phy_handler;
static void *g_phy_arg;
static struct wdog_s g_phy_wdog;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_enet_phy_poll
 *
 * Description:
 *   Watchdog timer callback that emulates the PHY interrupt.
 *
 ****************************************************************************/

static void imxrt_enet_phy_poll(wdparm_t arg)
{
  xcpt_t handler = g_phy_handler;

  if (handler != NULL)
    {
      handler(0, NULL, g_phy_arg);
    }
}

/****************************************************************************
 * Name: imxrt_enet_phy_enable
 ****************************************************************************/

static void imxrt_enet_phy_enable(bool enable)
{
  if (enable && g_phy_handler != NULL)
    {
      wd_start(&g_phy_wdog, PHY_POLL_TICKS, imxrt_enet_phy_poll, 0);
    }
  else
    {
      wd_cancel(&g_phy_wdog);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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
 *   The EC300 has no PHY interrupt line, so the interrupt is emulated by
 *   calling the handler every CONFIG_DDCNC_EC300_PHY_POLL_MSEC
 *   milliseconds while it is enabled.
 *
 * Input Parameters:
 *   intf    - Identifies the network interface.  For example "eth0".
 *   handler - The client interrupt handler to be invoked when the PHY
 *             asserts an interrupt.  A value of NULL can be passed in order
 *             to detach and disable the PHY interrupt.
 *   arg     - The argument that will accompany the interrupt
 *   enable  - A function pointer that be used to enable or disable the
 *             PHY interrupt.
 *
 * Returned Value:
 *   Zero (OK) returned on success; a negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int arch_phy_irq(const char *intf, xcpt_t handler, void *arg,
                 phy_enable_t *enable)
{
  irqstate_t flags;

  DEBUGASSERT(intf);

  ninfo("%s: handler=%p\n", intf, handler);

  if (strcmp(intf, IMXRT_ENET_DEVNAME) != 0)
    {
      nerr("ERROR: Unsupported interface: %s\n", intf);
      return -EINVAL;
    }

  flags = enter_critical_section();

  /* Return with the (emulated) interrupt disabled in either case */

  wd_cancel(&g_phy_wdog);

  g_phy_handler = handler;
  g_phy_arg     = arg;

  if (enable)
    {
      *enable = handler != NULL ? imxrt_enet_phy_enable : NULL;
    }

  leave_critical_section(flags);
  return OK;
}

#endif /* CONFIG_IMXRT_ENET && CONFIG_ARCH_PHY_INTERRUPT */
