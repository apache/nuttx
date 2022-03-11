/****************************************************************************
 * arch/arm/src/stm32/stm32_dumpgpio.c
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
#include "stm32_gpio.h"
#include "stm32_rcc.h"

#ifdef CONFIG_DEBUG_FEATURES

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Port letters for prettier debug output */

static const char g_portchar[STM32_NGPIO_PORTS] =
{
#if STM32_NGPIO_PORTS > 11
#  error "Additional support required for this number of GPIOs"
#elif STM32_NGPIO_PORTS > 10
  'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K'
#elif STM32_NGPIO_PORTS > 9
  'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J'
#elif STM32_NGPIO_PORTS > 8
  'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I'
#elif STM32_NGPIO_PORTS > 7
  'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H'
#elif STM32_NGPIO_PORTS > 6
  'A', 'B', 'C', 'D', 'E', 'F', 'G'
#elif STM32_NGPIO_PORTS > 5
  'A', 'B', 'C', 'D', 'E', 'F'
#elif STM32_NGPIO_PORTS > 4
  'A', 'B', 'C', 'D', 'E'
#elif STM32_NGPIO_PORTS > 3
  'A', 'B', 'C', 'D'
#elif STM32_NGPIO_PORTS > 2
  'A', 'B', 'C'
#elif STM32_NGPIO_PORTS > 1
  'A', 'B'
#elif STM32_NGPIO_PORTS > 0
  'A'
#else
#  error "Bad number of GPIOs"
#endif
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  stm32_dumpgpio
 *
 * Description:
 *   Dump all GPIO registers associated with the provided base address
 *
 ****************************************************************************/

int stm32_dumpgpio(uint32_t pinset, const char *msg)
{
  irqstate_t   flags;
  uint32_t     base;
  unsigned int port;

  /* Get the base address associated with the GPIO port */

  port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  base = g_gpiobase[port];

  /* The following requires exclusive access to the GPIO registers */

  flags = enter_critical_section();

#if defined(CONFIG_STM32_STM32F10XX)
  _info("GPIO%c pinset: %08x base: %08x -- %s\n",
       g_portchar[port], pinset, base, msg);

  if ((getreg32(STM32_RCC_APB2ENR) & RCC_APB2ENR_IOPEN(port)) != 0)
    {
      _info("  CR: %08x %08x IDR: %04x ODR: %04x LCKR: %04x\n",
           getreg32(base + STM32_GPIO_CRH_OFFSET),
           getreg32(base + STM32_GPIO_CRL_OFFSET),
           getreg32(base + STM32_GPIO_IDR_OFFSET),
           getreg32(base + STM32_GPIO_ODR_OFFSET),
           getreg32(base + STM32_GPIO_LCKR_OFFSET));
      _info("  EVCR: %02x MAPR: %08x CR: %04x %04x %04x %04x\n",
           getreg32(STM32_AFIO_EVCR), getreg32(STM32_AFIO_MAPR),
           getreg32(STM32_AFIO_EXTICR1),
           getreg32(STM32_AFIO_EXTICR2),
           getreg32(STM32_AFIO_EXTICR3),
           getreg32(STM32_AFIO_EXTICR4));
    }
  else
    {
      _info("  GPIO%c not enabled: APB2ENR: %08x\n",
          g_portchar[port], getreg32(STM32_RCC_APB2ENR));
    }

#elif defined(CONFIG_STM32_STM32L15XX)
  DEBUGASSERT(port < STM32_NGPIO_PORTS);

  _info("GPIO%c pinset: %08x base: %08x -- %s\n",
        g_portchar[port], pinset, base, msg);

  if ((getreg32(STM32_RCC_AHBENR) & RCC_AHBENR_GPIOEN(port)) != 0)
    {
      _info(" MODE: %08x OTYPE: %04x     OSPEED: %08x PUPDR: %08x\n",
           getreg32(base + STM32_GPIO_MODER_OFFSET),
           getreg32(base + STM32_GPIO_OTYPER_OFFSET),
           getreg32(base + STM32_GPIO_OSPEED_OFFSET),
           getreg32(base + STM32_GPIO_PUPDR_OFFSET));
      _info("  IDR: %04x       ODR: %04x       BSRR: %08x  LCKR: %04x\n",
           getreg32(base + STM32_GPIO_IDR_OFFSET),
           getreg32(base + STM32_GPIO_ODR_OFFSET),
           getreg32(base + STM32_GPIO_BSRR_OFFSET),
           getreg32(base + STM32_GPIO_LCKR_OFFSET));
      _info(" AFRH: %08x  AFRL: %08x\n",
           getreg32(base + STM32_GPIO_AFRH_OFFSET),
           getreg32(base + STM32_GPIO_AFRL_OFFSET));
    }
  else
    {
      _info("  GPIO%c not enabled: AHBENR: %08x\n",
            g_portchar[port], getreg32(STM32_RCC_AHBENR));
    }

#elif defined(CONFIG_STM32_STM32F30XX) || defined(CONFIG_STM32_STM32F37XX) || \
      defined(CONFIG_STM32_STM32F33XX)
  DEBUGASSERT(port < STM32_NGPIO_PORTS);

  _info("GPIO%c pinset: %08x base: %08x -- %s\n",
        g_portchar[port], pinset, base, msg);

  /* GPIOs are always enabled */

  _info(" MODE: %08x OTYPE: %04x     OSPEED: %08x PUPDR: %08x\n",
        getreg32(base + STM32_GPIO_MODER_OFFSET),
        getreg32(base + STM32_GPIO_OTYPER_OFFSET),
        getreg32(base + STM32_GPIO_OSPEED_OFFSET),
        getreg32(base + STM32_GPIO_PUPDR_OFFSET));
  _info("  IDR: %04x       ODR: %04x       BSRR: %08x  LCKR: %04x\n",
        getreg32(base + STM32_GPIO_IDR_OFFSET),
        getreg32(base + STM32_GPIO_ODR_OFFSET),
        getreg32(base + STM32_GPIO_BSRR_OFFSET),
        getreg32(base + STM32_GPIO_LCKR_OFFSET));
  _info(" AFRH: %08x  AFRL: %08x   BRR: %04x\n",
        getreg32(base + STM32_GPIO_AFRH_OFFSET),
        getreg32(base + STM32_GPIO_AFRL_OFFSET),
        getreg32(base + STM32_GPIO_BRR_OFFSET));

#elif defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F4XXX)
  DEBUGASSERT(port < STM32_NGPIO_PORTS);

  _info("GPIO%c pinset: %08x base: %08x -- %s\n",
        g_portchar[port], pinset, base, msg);

  if ((getreg32(STM32_RCC_AHB1ENR) & RCC_AHB1ENR_GPIOEN(port)) != 0)
    {
      _info(" MODE: %08x OTYPE: %04x     OSPEED: %08x PUPDR: %08x\n",
            getreg32(base + STM32_GPIO_MODER_OFFSET),
            getreg32(base + STM32_GPIO_OTYPER_OFFSET),
            getreg32(base + STM32_GPIO_OSPEED_OFFSET),
            getreg32(base + STM32_GPIO_PUPDR_OFFSET));
      _info("  IDR: %04x       ODR: %04x       BSRR: %08x  LCKR: %04x\n",
            getreg32(base + STM32_GPIO_IDR_OFFSET),
            getreg32(base + STM32_GPIO_ODR_OFFSET),
            getreg32(base + STM32_GPIO_BSRR_OFFSET),
            getreg32(base + STM32_GPIO_LCKR_OFFSET));
      _info(" AFRH: %08x  AFRL: %08x\n",
            getreg32(base + STM32_GPIO_AFRH_OFFSET),
            getreg32(base + STM32_GPIO_AFRL_OFFSET));
    }
  else
    {
      _info("  GPIO%c not enabled: AHB1ENR: %08x\n",
            g_portchar[port], getreg32(STM32_RCC_AHB1ENR));
    }

#elif defined(CONFIG_STM32_STM32G4XXX)
  DEBUGASSERT(port < STM32_NGPIO_PORTS);

  _info("GPIO%c pinset: %08x base: %08x -- %s\n",
        g_portchar[port], pinset, base, msg);

  if ((getreg32(STM32_RCC_AHB2ENR) & RCC_AHB2ENR_GPIOEN(port)) != 0)
    {
      _info(" MODE: %08x OTYPE: %04x     OSPEED: %08x PUPDR: %08x\n",
            getreg32(base + STM32_GPIO_MODER_OFFSET),
            getreg32(base + STM32_GPIO_OTYPER_OFFSET),
            getreg32(base + STM32_GPIO_OSPEED_OFFSET),
            getreg32(base + STM32_GPIO_PUPDR_OFFSET));
      _info("  IDR: %04x       ODR: %04x       BSRR: %08x  LCKR: %04x\n",
            getreg32(base + STM32_GPIO_IDR_OFFSET),
            getreg32(base + STM32_GPIO_ODR_OFFSET),
            getreg32(base + STM32_GPIO_BSRR_OFFSET),
            getreg32(base + STM32_GPIO_LCKR_OFFSET));
      _info(" AFRH: %08x  AFRL: %08x   BRR: %04x\n",
            getreg32(base + STM32_GPIO_AFRH_OFFSET),
            getreg32(base + STM32_GPIO_AFRL_OFFSET),
            getreg32(base + STM32_GPIO_BRR_OFFSET));
    }
  else
    {
      _info("  GPIO%c not enabled: AHB2ENR: %08x\n",
            g_portchar[port], getreg32(STM32_RCC_AHB2ENR));
    }

#else
# error "Unsupported STM32 chip"
#endif
  leave_critical_section(flags);
  return OK;
}

#endif /* CONFIG_DEBUG_FEATURES */
