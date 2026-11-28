/****************************************************************************
 * arch/tricore/src/tc4x/tc4x_gpio.c
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
#include "hardware/tc4x_port.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static spinlock_t g_tc4x_gpio_lock = SP_UNLOCKED;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tc4x_gpio_unprotect
 *
 * Description:
 *   The pad configuration of a port can only be written while the
 *   protection of its access group is not in the RUN state.  Leave RUN for
 *   CONFIG if needed and return true if it has to be restored.
 *
 ****************************************************************************/

static bool tc4x_gpio_unprotect(uintptr_t prote)
{
  if ((getreg32(prote) & TC4X_PORT_PROT_STATE_MASK) != TC4X_PORT_PROT_RUN)
    {
      return false;
    }

  putreg32(TC4X_PORT_PROT_CONFIG | TC4X_PORT_PROT_SWEN, prote);
  return true;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tricore_gpio_config
 *
 * Description:
 *   Configure a port pin as described by a pin set.  Direction, mode, pad
 *   driver and level are all in the DRVCFG register of the pin.
 *
 ****************************************************************************/

int tricore_gpio_config(uint32_t pinset)
{
  uintptr_t port = TC4X_PORT_BASE(GPIO_PORT_OF(pinset));
  uintptr_t drvcfg = TC4X_PORT_DRVCFG(port, GPIO_PIN_OF(pinset));
  irqstate_t flags;
  uint32_t regval;
  bool restore;

  regval = (GPIO_ALT_OF(pinset) << TC4X_PORT_DRVCFG_MODE_SHIFT) |
           (GPIO_PADDRV_OF(pinset) << TC4X_PORT_DRVCFG_PD_SHIFT) |
           (GPIO_PADLEVEL_OF(pinset) << TC4X_PORT_DRVCFG_PL_SHIFT);

  if (GPIO_IS_OUTPUT(pinset))
    {
      regval |= TC4X_PORT_DRVCFG_DIR;
      if ((pinset & GPIO_OPENDRAIN) != 0)
        {
          regval |= TC4X_PORT_DRVCFG_OD;
        }
    }

  flags = spin_lock_irqsave(&g_tc4x_gpio_lock);

  if (GPIO_IS_OUTPUT(pinset) && GPIO_ALT_OF(pinset) == 0)
    {
      tricore_gpio_write(pinset, (pinset & GPIO_OUTPUT_SET) != 0);
    }

  restore = tc4x_gpio_unprotect(TC4X_PORT_PROTE(port));
  putreg32((getreg32(drvcfg) & ~(TC4X_PORT_DRVCFG_MODE_MASK |
                                 TC4X_PORT_DRVCFG_PAD_MASK)) | regval,
           drvcfg);
  if (restore)
    {
      putreg32(TC4X_PORT_PROT_RUN | TC4X_PORT_PROT_SWEN,
               TC4X_PORT_PROTE(port));
    }

  spin_unlock_irqrestore(&g_tc4x_gpio_lock, flags);
  return OK;
}

/****************************************************************************
 * Name: tricore_gpio_write
 ****************************************************************************/

void tricore_gpio_write(uint32_t pinset, bool value)
{
  int pin = GPIO_PIN_OF(pinset);

  putreg32(value ? TC4X_PORT_OMR_SET(pin) : TC4X_PORT_OMR_CLR(pin),
           TC4X_PORT_BASE(GPIO_PORT_OF(pinset)) + TC4X_PORT_OMR_OFFSET);
}

/****************************************************************************
 * Name: tricore_gpio_read
 ****************************************************************************/

bool tricore_gpio_read(uint32_t pinset)
{
  uintptr_t in = TC4X_PORT_BASE(GPIO_PORT_OF(pinset)) + TC4X_PORT_IN_OFFSET;

  return (getreg32(in) & (1 << GPIO_PIN_OF(pinset))) != 0;
}
