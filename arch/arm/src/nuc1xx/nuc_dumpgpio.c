/****************************************************************************
 * arch/arm/src/nuc1xx/nuc_dumpgpio.c
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

#include <sys/types.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>

#include "arm_internal.h"
#include "chip.h"
#include "nuc_gpio.h"

#ifdef CONFIG_DEBUG_GPIO_INFO

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Port letters for prettier debug output */

static const char g_portchar[NUC_GPIO_NPORTS] =
{
#if NUC_GPIO_NPORTS > 9
#  error "Additional support required for this number of GPIOs"
#elif NUC_GPIO_NPORTS > 8
  'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I'
#elif NUC_GPIO_NPORTS > 7
  'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H'
#elif NUC_GPIO_NPORTS > 6
  'A', 'B', 'C', 'D', 'E', 'F', 'G'
#elif NUC_GPIO_NPORTS > 5
  'A', 'B', 'C', 'D', 'E', 'F'
#elif NUC_GPIO_NPORTS > 4
  'A', 'B', 'C', 'D', 'E'
#elif NUC_GPIO_NPORTS > 3
  'A', 'B', 'C', 'D'
#elif NUC_GPIO_NPORTS > 2
  'A', 'B', 'C'
#elif NUC_GPIO_NPORTS > 1
  'A', 'B'
#elif NUC_GPIO_NPORTS > 0
  'A'
#else
#  error "Bad number of GPIOs"
#endif
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  nuc_dumpgpio
 *
 * Description:
 *   Dump all GPIO registers associated with the provided pin description
 *   along with a descriptive message.
 *
 ****************************************************************************/

void nuc_dumpgpio(gpio_cfgset_t pinset, const char *msg)
{
  irqstate_t flags;
  uintptr_t base;
  int port;

  /* Decode the port and pin.  Use the port number to get the GPIO base
   * address.
   */

  port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  DEBUGASSERT((unsigned)port <= NUC_GPIO_PORTE);
  base = NUC_GPIO_CTRL_BASE(port);

  /* The following requires exclusive access to the GPIO registers */

  flags = enter_critical_section();

  gpioinfo("GPIO%c pinset: %08x base: %08x -- %s\n",
           g_portchar[port], pinset, base, msg);
  gpioinfo("  PMD: %08x  OFFD: %08x  DOUT: %08x DMASK: %08x\n",
           getreg32(base + NUC_GPIO_PMD_OFFSET),
           getreg32(base + NUC_GPIO_OFFD_OFFSET),
           getreg32(base + NUC_GPIO_DOUT_OFFSET),
           getreg32(base + NUC_GPIO_DMASK_OFFSET));
  gpioinfo("  PIN: %08x  DBEN: %08x   IMD: %08x   IEN: %08x\n",
           getreg32(base + NUC_GPIO_PIN_OFFSET),
           getreg32(base + NUC_GPIO_DBEN_OFFSET),
           getreg32(base + NUC_GPIO_IMD_OFFSET),
           getreg32(base + NUC_GPIO_IEN_OFFSET));
  gpioinfo(" ISRC: %08x\n",
           getreg32(base + NUC_GPIO_ISRC_OFFSET));

  leave_critical_section(flags);
}

#endif /* CONFIG_DEBUG_GPIO_INFO */
