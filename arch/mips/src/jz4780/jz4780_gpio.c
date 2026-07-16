/****************************************************************************
 * arch/mips/src/jz4780/jz4780_gpio.c
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

#include <stdint.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/spinlock.h>
#include <arch/board/board.h>

#include "mips_internal.h"

#include "jz4780_gpio.h"

#if CHIP_NPORTS > 0

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static spinlock_t g_configgpio_lock = SP_UNLOCKED;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* This table is used to map a port number to a port base address. */

const uintptr_t g_gpiobase[CHIP_NPORTS] =
{
  JZ4780_BASE_GPIOA
#if CHIP_NPORTS > 1
  , JZ4780_BASE_GPIOB
#endif
#if CHIP_NPORTS > 2
  , JZ4780_BASE_GPIOC
#endif
#if CHIP_NPORTS > 3
  , JZ4780_BASE_GPIOD
#endif
#if CHIP_NPORTS > 4
  , JZ4780_BASE_GPIOE
#endif
#if CHIP_NPORTS > 5
  , JZ4780_BASE_GPIOF
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: Inline PIN set field extractors
 ****************************************************************************/

static inline unsigned int jz4780_portno(pinset_t pinset)
{
  return ((pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT);
}

static inline unsigned int jz4780_pinno(pinset_t pinset)
{
  return ((pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: jz4780_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin
 *
 * Returned Value:
 *   OK on success; negated errno on failure.
 *
 ****************************************************************************/

int jz4780_configgpio(pinset_t p)
{
  unsigned int port = jz4780_portno(p);
  unsigned int pin  = jz4780_pinno(p);
  uint32_t     mask = (1 << pin);
  uintptr_t    base;
  irqstate_t   flags;

  /* Verify that the port number is within range */

  if (port < CHIP_NPORTS)
    {
      /* Get the base address of the ports */

      base = g_gpiobase[port];

      flags = spin_lock_irqsave(&g_configgpio_lock);

      putreg32(mask, base + ((MODE_BIT_PXT0 & p) ? GPIO_PXT0S : GPIO_PXT0C));
      putreg32(mask, base + ((MODE_BIT_PXT1 & p) ? GPIO_PXT1S : GPIO_PXT1C));
      putreg32(mask, base + ((MODE_BIT_MSK & p) ? GPIO_MSKS : GPIO_MSKC));
      putreg32(mask, base + ((MODE_BIT_INT & p) ? GPIO_INTS : GPIO_INTC));

      putreg32(mask, base + ((GPIO_PULL_EN & p) ? GPIO_PENS : GPIO_PENC));

      spin_unlock_irqrestore(&g_configgpio_lock, flags);

      return OK;
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: jz4780_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void jz4780_gpiowrite(pinset_t pinset, bool value)
{
  unsigned int port = jz4780_portno(pinset);
  unsigned int pin  = jz4780_pinno(pinset);
  uintptr_t    base;

  /* Verify that the port number is within range */

  if (port < CHIP_NPORTS)
    {
      /* Get the base address of the ports */

      base = g_gpiobase[port];

      /* Set or clear the output */

      if (value)
        {
          putreg32(1 << pin, base + GPIO_PXT0S);
        }
      else
        {
          putreg32(1 << pin, base + GPIO_PXT0C);
        }
    }
}

/****************************************************************************
 * Name: jz4780_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool jz4780_gpioread(pinset_t pinset)
{
  unsigned int port = jz4780_portno(pinset);
  unsigned int pin  = jz4780_pinno(pinset);
  uintptr_t    base;

  /* Verify that the port number is within range */

  if (port < CHIP_NPORTS)
    {
      /* Get the base address of the ports */

      base = g_gpiobase[port];

      /* Get and return the input value */

      return (getreg32(base + GPIO_PXPIN) & (1 << pin)) != 0;
    }

  return false;
}

#endif /* CHIP_NPORTS > 0 */
