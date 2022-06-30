/****************************************************************************
 * arch/arm/src/stm32wb/stm32wb_dumpgpio.c
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
#include <debug.h>

#include <nuttx/irq.h>

#include "arm_internal.h"
#include "chip.h"
#include "stm32wb_gpio.h"
#include "stm32wb_rcc.h"

#ifdef CONFIG_DEBUG_FEATURES

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Port letters for prettier debug output */

static const char g_portchar[STM32WB_NPORTS] =
{
  'A', 'B', 'C',
#if defined(CONFIG_STM32WB_GPIO_HAVE_PORTD)
  'D',
#endif
#if defined(CONFIG_STM32WB_GPIO_HAVE_PORTE)
  'E',
#endif
  'H'
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  stm32wb_dumpgpio
 *
 * Description:
 *   Dump all GPIO registers associated with the provided base address
 *
 ****************************************************************************/

int stm32wb_dumpgpio(uint32_t pinset, const char *msg)
{
  irqstate_t   flags;
  uint32_t     base;
  unsigned int port;

  /* Get the base address associated with the GPIO port */

  port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  base = g_gpiobase[port];

  /* The following requires exclusive access to the GPIO registers */

  flags = enter_critical_section();

  DEBUGASSERT(port < STM32WB_NPORTS);

  _info("GPIO%c pinset: %08x base: %08x -- %s\n",
        g_portchar[port], pinset, base, msg);

  if ((getreg32(STM32WB_RCC_AHB2ENR) & RCC_AHB2ENR_GPIOEN(port)) != 0)
    {
      _info(" MODE: %08x OTYPE: %04x     OSPEED: %08x PUPDR: %08x\n",
            getreg32(base + STM32WB_GPIO_MODER_OFFSET),
            getreg32(base + STM32WB_GPIO_OTYPER_OFFSET),
            getreg32(base + STM32WB_GPIO_OSPEED_OFFSET),
            getreg32(base + STM32WB_GPIO_PUPDR_OFFSET));
      _info("  IDR: %04x       ODR: %04x       BSRR: %08x  LCKR: %04x\n",
            getreg32(base + STM32WB_GPIO_IDR_OFFSET),
            getreg32(base + STM32WB_GPIO_ODR_OFFSET),
            getreg32(base + STM32WB_GPIO_BSRR_OFFSET),
            getreg32(base + STM32WB_GPIO_LCKR_OFFSET));
      _info(" AFRH: %08x  AFRL: %08x\n",
            getreg32(base + STM32WB_GPIO_AFRH_OFFSET),
            getreg32(base + STM32WB_GPIO_AFRL_OFFSET));
    }
  else
    {
      _info("  GPIO%c not enabled: AHB2ENR: %08x\n",
            g_portchar[port], getreg32(STM32WB_RCC_AHB2ENR));
    }

  leave_critical_section(flags);
  return OK;
}

#endif /* CONFIG_DEBUG_FEATURES */
