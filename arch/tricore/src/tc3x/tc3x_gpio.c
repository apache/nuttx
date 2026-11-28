/****************************************************************************
 * arch/tricore/src/tc3x/tc3x_gpio.c
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
#include <stdint.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/spinlock.h>

#include "tricore_internal.h"
#include "aurix/tricore_gpio.h"
#include "tc3x_wdt.h"
#include "hardware/tc3x_port.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static spinlock_t g_tc3x_gpio_lock = SP_UNLOCKED;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void tc3x_gpio_modify(uintptr_t reg, uint32_t clear, uint32_t set)
{
  putreg32((getreg32(reg) & ~clear) | set, reg);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tricore_gpio_config
 *
 * Description:
 *   Configure a port pin as described by a pin set.  The pad driver is
 *   protected by the CPU ENDINIT.
 *
 ****************************************************************************/

int tricore_gpio_config(uint32_t pinset)
{
  uintptr_t port = TC3X_PORT_BASE(GPIO_PORT_OF(pinset));
  int pin = GPIO_PIN_OF(pinset);
  irqstate_t flags;
  uint32_t pc;
  uint32_t pd;
  int shift;

  if (GPIO_IS_OUTPUT(pinset))
    {
      if (GPIO_ALT_OF(pinset) > TC3X_PORT_PC_ALT_MAX)
        {
          return -EINVAL;
        }

      pc = TC3X_PORT_PC_OUTPUT | GPIO_ALT_OF(pinset);
      if ((pinset & GPIO_OPENDRAIN) != 0)
        {
          pc |= TC3X_PORT_PC_OPENDRAIN;
        }
    }
  else
    {
      pc = GPIO_ALT_OF(pinset) & TC3X_PORT_PC_PULL_MASK;
    }

  pd = (GPIO_PADLEVEL_OF(pinset) << TC3X_PORT_PDR_PL_SHIFT) |
       GPIO_PADDRV_OF(pinset);
  if (pd > TC3X_PORT_PDR_MASK)
    {
      return -EINVAL;
    }

  flags = spin_lock_irqsave(&g_tc3x_gpio_lock);

  if (GPIO_IS_OUTPUT(pinset) && GPIO_ALT_OF(pinset) == 0)
    {
      tricore_gpio_write(pinset, (pinset & GPIO_OUTPUT_SET) != 0);
    }

  shift = TC3X_PORT_PDR_SHIFT(pin);
  tc3x_wdt_cpu_endinit(false);
  tc3x_gpio_modify(TC3X_PORT_PDR(port, pin), TC3X_PORT_PDR_MASK << shift,
                   pd << shift);
  tc3x_wdt_cpu_endinit(true);

  shift = TC3X_PORT_IOCR_SHIFT(pin);
  tc3x_gpio_modify(TC3X_PORT_IOCR(port, pin), TC3X_PORT_IOCR_MASK << shift,
                   (pc << TC3X_PORT_PC_SHIFT) << shift);

  spin_unlock_irqrestore(&g_tc3x_gpio_lock, flags);
  return OK;
}

/****************************************************************************
 * Name: tricore_gpio_write
 ****************************************************************************/

void tricore_gpio_write(uint32_t pinset, bool value)
{
  int pin = GPIO_PIN_OF(pinset);

  putreg32(value ? TC3X_PORT_OMR_SET(pin) : TC3X_PORT_OMR_CLR(pin),
           TC3X_PORT_BASE(GPIO_PORT_OF(pinset)) + TC3X_PORT_OMR_OFFSET);
}

/****************************************************************************
 * Name: tricore_gpio_read
 ****************************************************************************/

bool tricore_gpio_read(uint32_t pinset)
{
  uintptr_t in = TC3X_PORT_BASE(GPIO_PORT_OF(pinset)) + TC3X_PORT_IN_OFFSET;

  return (getreg32(in) & (1 << GPIO_PIN_OF(pinset))) != 0;
}
