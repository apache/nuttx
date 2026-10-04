/****************************************************************************
 * boards/mips/pic32mz/ev49n51a/src/pic32mz_ethernet.c
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

#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <nuttx/debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "ev49n51a.h"

#ifdef CONFIG_EV49N51A_PHY_INTERRUPT

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ev49n51a_phy_enable
 ****************************************************************************/

static void ev49n51a_phy_enable(bool enable)
{
  if (enable)
    {
      pic32mz_gpioirqenable(GPIO_PHY_NINT);
    }
  else
    {
      pic32mz_gpioirqdisable(GPIO_PHY_NINT);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arch_phy_irq
 *
 * Description:
 *   Attach (or detach, if handler is NULL) the PHY interrupt handler.  The
 *   interrupt is always disabled on return; the caller enables it through
 *   the returned enable function.  See include/nuttx/arch.h.
 *
 ****************************************************************************/

int arch_phy_irq(const char *intf, xcpt_t handler, void *arg,
                 phy_enable_t *enable)
{
  irqstate_t flags;
  int ret;

  DEBUGASSERT(intf != NULL);

  if (strcmp(intf, "eth0") != 0)
    {
      nerr("ERROR: Unsupported interface: %s\n", intf);
      return -ENODEV;
    }

  flags = enter_critical_section();

  pic32mz_gpioirqdisable(GPIO_PHY_NINT);
  ret = pic32mz_gpioattach(GPIO_PHY_NINT, handler, arg);

  if (enable != NULL)
    {
      *enable = handler != NULL ? ev49n51a_phy_enable : NULL;
    }

  leave_critical_section(flags);
  return ret;
}

#endif /* CONFIG_EV49N51A_PHY_INTERRUPT */
