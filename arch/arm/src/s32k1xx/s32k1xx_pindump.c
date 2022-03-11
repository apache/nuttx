/****************************************************************************
 * arch/arm/src/s32k1xx/s32k1xx_pindump.c
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
#include "hardware/s32k1xx_gpio.h"
#include "hardware/s32k1xx_port.h"
#include "s32k1xx_pin.h"

#ifdef CONFIG_DEBUG_GPIO_INFO

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Port letters for prettier debug output */

static const char g_portchar[S32K1XX_NPORTS] =
{
#if S32K1XX_NPORTS > 9
#  error "Additional support required for this number of GPIOs"
#elif S32K1XX_NPORTS > 8
  'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I'
#elif S32K1XX_NPORTS > 7
  'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H'
#elif S32K1XX_NPORTS > 6
  'A', 'B', 'C', 'D', 'E', 'F', 'G'
#elif S32K1XX_NPORTS > 5
  'A', 'B', 'C', 'D', 'E', 'F'
#elif S32K1XX_NPORTS > 4
  'A', 'B', 'C', 'D', 'E'
#elif S32K1XX_NPORTS > 3
  'A', 'B', 'C', 'D'
#elif S32K1XX_NPORTS > 2
  'A', 'B', 'C'
#elif S32K1XX_NPORTS > 1
  'A', 'B'
#elif S32K1XX_NPORTS > 0
  'A'
#else
#  error "Bad number of GPIOs"
#endif
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  s32k1xx_pindump
 *
 * Description:
 *   Dump all GPIO registers associated with the provided pin description
 *   along with a descriptive message.
 *
 ****************************************************************************/

void s32k1xx_pindump(uint32_t pinset, const char *msg)
{
  irqstate_t flags;
  uintptr_t base;
  int port;

  /* Decode the port and pin.  Use the port number to get the GPIO base
   * address.
   */

  port = (pinset & _PIN_PORT_MASK) >> _PIN_PORT_SHIFT;
  DEBUGASSERT((unsigned)port < S32K1XX_NPORTS);
  base = S32K1XX_GPIO_BASE(port);

  /* The following requires exclusive access to the GPIO registers */

  flags = enter_critical_section();

  gpioinfo("GPIO%c pinset: %08x base: %08x -- %s\n",
           g_portchar[port], pinset, base, msg);
  gpioinfo("  PDOR: %08x  PDIR: %08x  PDDR: %08x\n",
           getreg32(base + S32K1XX_GPIO_PDOR_OFFSET),
           getreg32(base + S32K1XX_GPIO_PDIR_OFFSET),
           getreg32(base + S32K1XX_GPIO_PDDR_OFFSET));

  leave_critical_section(flags);
}

#endif /* CONFIG_DEBUG_GPIO_INFO */
