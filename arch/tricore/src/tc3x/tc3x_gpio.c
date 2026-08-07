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

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/spinlock.h>

#include "tricore_gpio.h"
#include "tricore_internal.h"
#include "hardware/tc3x_port.h"

static spinlock_t g_gpio_lock = SP_UNLOCKED;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uintptr_t tc3x_port_base(uint32_t port)
{
  return AURIX_PORT_ADDR(port);
}

static uint32_t tc3x_build_padcfg(gpio_pinset_t pinset)
{
  uint32_t mode = GPIO_GET_MODE(pinset);
  uint32_t funcalt = GPIO_GET_FUNCALT(pinset);
  uint32_t cfg;

  if (mode == GPIO_INPUT)
    {
      cfg = funcalt & 0x07;
    }
  else
    {
      cfg = PORT_PC_OUTPUT | (funcalt & 0x07);

      if (GPIO_IS_OPENDRAIN(pinset))
        {
          cfg |= 0x08;
        }
    }

  return cfg;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: aurix_config_gpio
 *
 * Description:
 *   Configure a GPIO pin on TC3x.
 *
 ****************************************************************************/

int aurix_config_gpio(gpio_pinset_t pinset)
{
  irqstate_t flags;
  uint32_t port = GPIO_GET_PORT(pinset);
  uint32_t pin  = GPIO_GET_PIN(pinset);
  uintptr_t base = tc3x_port_base(port);
  uint32_t cfg;
  uint32_t iocr_off;
  uint32_t iocr_shift;
  uint32_t iocr_mask;
  uint32_t pdr_off;
  uint32_t pdr_shift;
  uint32_t pdr_mask;
  uint32_t pdr_val;

  if (pin > 15)
    {
      return -EINVAL;
    }

  cfg = tc3x_build_padcfg(pinset);

  iocr_off   = PORT_IOCR_REG(pin);
  iocr_shift = PORT_IOCR_PC_SHIFT(pin);
  iocr_mask  = PORT_IOCR_PC_MASK(pin);

  pdr_off    = PORT_PDR_REG(pin);
  pdr_shift  = PORT_PDR_SHIFT(pin);
  pdr_mask   = PORT_PDR_MASK(pin);
  pdr_val    = PORT_PDR_VAL(GPIO_GET_PADDRV(pinset) & 0x03,
                             GPIO_GET_PADLEVEL(pinset) & 0x03);

  flags = spin_lock_irqsave(&g_gpio_lock);

  if (GPIO_GET_MODE(pinset) != GPIO_INPUT)
    {
      if (GPIO_IS_INIT_HIGH(pinset))
        {
          putreg32(PORT_OMR_SET(pin), base + PORT_OMR_OFFSET);
        }
      else
        {
          putreg32(PORT_OMR_CLR(pin), base + PORT_OMR_OFFSET);
        }
    }

  aurix_cpu_endinit_enable(false);
  modreg32(pdr_val << pdr_shift, pdr_mask, base + pdr_off);
  aurix_cpu_endinit_enable(true);

  if (GPIO_IS_PERIPH_OWN_PAD(pinset))
    {
      aurix_safety_endinit_enable(false);
      modreg32(BIT(pin), BIT(pin), base + PORT_PCSR_OFFSET);
      aurix_safety_endinit_enable(true);
    }

  modreg32(cfg << iocr_shift, iocr_mask, base + iocr_off);

  spin_unlock_irqrestore(&g_gpio_lock, flags);

  return OK;
}
