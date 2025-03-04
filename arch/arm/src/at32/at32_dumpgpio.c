/****************************************************************************
 * arch/arm/src/at32/at32_dumpgpio.c
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

/* Output debug info even if debug output is not selected. */

#undef  CONFIG_DEBUG_INFO
#define CONFIG_DEBUG_INFO 1

#include <sys/types.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>

#include "arm_internal.h"
#include "chip.h"
#include "at32_gpio.h"
#include "at32_rcc.h"

#ifdef CONFIG_DEBUG_FEATURES

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Port letters for prettier debug output */

static const char g_portchar[AT32_NGPIO_PORTS] =
{
#if AT32_NGPIO_PORTS > 8
#  error "Additional support required for this number of GPIOs"
#elif AT32_NGPIO_PORTS > 7
  'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H'
#elif AT32_NGPIO_PORTS > 6
  'A', 'B', 'C', 'D', 'E', 'F', 'G'
#elif AT32_NGPIO_PORTS > 5
  'A', 'B', 'C', 'D', 'E', 'F'
#elif AT32_NGPIO_PORTS > 4
  'A', 'B', 'C', 'D', 'E'
#elif AT32_NGPIO_PORTS > 3
  'A', 'B', 'C', 'D'
#elif AT32_NGPIO_PORTS > 2
  'A', 'B', 'C'
#elif AT32_NGPIO_PORTS > 1
  'A', 'B'
#elif AT32_NGPIO_PORTS > 0
  'A'
#else
#  error "Bad number of GPIOs"
#endif
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  at32_dumpgpio
 *
 * Description:
 *   Dump all GPIO registers associated with the provided base address
 *
 ****************************************************************************/

int at32_dumpgpio(uint32_t pinset, const char *msg)
{
  irqstate_t   flags;
  uint32_t     base;
  unsigned int port;

  /* Get the base address associated with the GPIO port */

  port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  base = g_gpiobase[port];

  /* The following requires exclusive access to the GPIO registers */

  flags = enter_critical_section();

#if defined(CONFIG_AT32_AT32F43XX)
  DEBUGASSERT(port < AT32_NGPIO_PORTS);

  _info("GPIO%c pinset: %08x base: %08x -- %s\n",
        g_portchar[port], pinset, base, msg);

  if ((getreg32(AT32_CRM_AHBEN1) & CRM_AHB1EN1_GPIOEN(port)) != 0)
    {
      _info(" MODE: %08x OTYPE: %04x     OSPEED: %08x PUPDR: %08x\n",
            getreg32(base + AT32_GPIO_CFGR_OFFSET),
            getreg32(base + AT32_GPIO_OMODER_OFFSET),
            getreg32(base + AT32_GPIO_ODRVR_OFFSET),
            getreg32(base + AT32_GPIO_PULL_OFFSET));
      _info("  IDR: %04x       ODR: %04x       BSRR: %08x  LCKR: %04x\n",
            getreg32(base + AT32_GPIO_IDT_OFFSET),
            getreg32(base + AT32_GPIO_ODT_OFFSET),
            getreg32(base + AT32_GPIO_SCR_OFFSET),
            getreg32(base + AT32_GPIO_WPR_OFFSET));
      _info(" AFRH: %08x  AFRL: %08x\n",
            getreg32(base + AT32_GPIO_MUXH_OFFSET),
            getreg32(base + AT32_GPIO_MUXL_OFFSET));
    }
  else
    {
      _info("  GPIO%c not enabled: AHB1ENR: %08x\n",
            g_portchar[port], getreg32(AT32_CRM_AHBEN1));
    }

#else
# error "Unsupported AT32 chip"
#endif
  leave_critical_section(flags);
  return OK;
}

#endif /* CONFIG_DEBUG_FEATURES */
