/****************************************************************************
 * arch/arm/src/kl/kl_dumpgpio.c
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
#include "kl_gpio.h"

#ifdef CONFIG_DEBUG_FEATURES

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Port letters for prettier debug output */

static const char g_portchar[KL_GPIO_NPORTS] =
{
#if KL_GPIO_NPORTS > 9
#  error "Additional support required for this number of GPIOs"
#elif KL_GPIO_NPORTS > 8
  'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I'
#elif KL_GPIO_NPORTS > 7
  'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H'
#elif KL_GPIO_NPORTS > 6
  'A', 'B', 'C', 'D', 'E', 'F', 'G'
#elif KL_GPIO_NPORTS > 5
  'A', 'B', 'C', 'D', 'E', 'F'
#elif KL_GPIO_NPORTS > 4
  'A', 'B', 'C', 'D', 'E'
#elif KL_GPIO_NPORTS > 3
  'A', 'B', 'C', 'D'
#elif KL_GPIO_NPORTS > 2
  'A', 'B', 'C'
#elif KL_GPIO_NPORTS > 1
  'A', 'B'
#elif KL_GPIO_NPORTS > 0
  'A'
#else
#  error "Bad number of GPIOs"
#endif
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  kl_dumpgpio
 *
 * Description:
 *   Dump all GPIO registers associated with the provided pin description
 *   along with a descriptive message.
 *
 ****************************************************************************/

void kl_dumpgpio(gpio_cfgset_t pinset, const char *msg)
{
  irqstate_t flags;
  uintptr_t base;
  int port;

  /* Decode the port and pin.  Use the port number to get the GPIO base
   * address.
   */

  port = (pinset & _PIN_PORT_MASK) >> _PIN_PORT_SHIFT;
  DEBUGASSERT((unsigned)port < KL_GPIO_NPORTS);
  base = KL_GPIO_BASE(port);

  /* The following requires exclusive access to the GPIO registers */

  flags = enter_critical_section();

  _info("GPIO%c pinset: %08x base: %08x -- %s\n",
       g_portchar[port], pinset, base, msg);
  _info("  PDOR: %08x  PDIR: %08x  PDDR: %08x\n",
       getreg32(base + KL_GPIO_PDOR_OFFSET),
       getreg32(base + KL_GPIO_PDIR_OFFSET),
       getreg32(base + KL_GPIO_PDDR_OFFSET));

  leave_critical_section(flags);
}

#endif /* CONFIG_DEBUG_FEATURES */
