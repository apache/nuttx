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

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/spinlock.h>

#include "tricore_gpio.h"
#include "tricore_internal.h"
#include "hardware/tc4x_port.h"

#ifndef CONFIG_TC4X_GPIO_ACCESS_GROUP
#  define CONFIG_TC4X_GPIO_ACCESS_GROUP 0
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AURIX_PROT_STATE_RUN    0
#define AURIX_PROT_STATE_CONFIG 1
#define AURIX_PROT_SWEN        BIT(3)

static spinlock_t g_gpio_lock = SP_UNLOCKED;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

static inline uintptr_t tc4x_port_base(uint32_t port)
{
  return AURIX_PORT_ADDR(port);
}

static void tc4x_prot_set_config(uintptr_t prot_addr)
{
  uint32_t val = getreg32(prot_addr);

  if ((val & 0x07) == AURIX_PROT_STATE_RUN)
    {
      putreg32(AURIX_PROT_STATE_CONFIG | AURIX_PROT_SWEN, prot_addr);
    }
}

static void tc4x_prot_set_run(uintptr_t prot_addr)
{
  putreg32(AURIX_PROT_STATE_RUN | AURIX_PROT_SWEN, prot_addr);
}

static uint32_t tc4x_build_drvcfg(gpio_pinset_t pinset)
{
  uint32_t drvcfg = 0;
  uint32_t mode = GPIO_GET_MODE(pinset);
  uint32_t funcalt = GPIO_GET_FUNCALT(pinset);

  if (mode != GPIO_INPUT)
    {
      drvcfg |= PORT_DRVCFG_DIR;
    }

  if (GPIO_IS_OPENDRAIN(pinset))
    {
      drvcfg |= PORT_DRVCFG_OD;
    }

  drvcfg |= (funcalt << PORT_DRVCFG_MODE_SHIFT) & PORT_DRVCFG_MODE_MASK;

  drvcfg |= (GPIO_GET_PADDRV(pinset) << PORT_DRVCFG_PD_SHIFT)
             & PORT_DRVCFG_PD_MASK;

  drvcfg |= (GPIO_GET_PADLEVEL(pinset) << PORT_DRVCFG_PL_SHIFT)
             & PORT_DRVCFG_PL_MASK;

  return drvcfg;
}

int aurix_config_gpio(gpio_pinset_t pinset)
{
  irqstate_t flags;
  uint32_t port = GPIO_GET_PORT(pinset);
  uint32_t pin  = GPIO_GET_PIN(pinset);
  uintptr_t base = tc4x_port_base(port);
  uintptr_t prot;
  uint32_t drvcfg;

  if (pin > 15)
    {
      return -EINVAL;
    }

  drvcfg = tc4x_build_drvcfg(pinset);

  flags = spin_lock_irqsave(&g_gpio_lock);

  if (GPIO_GET_MODE(pinset) == GPIO_OUTPUT)
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

  if (GPIO_IS_PERIPH_OWN_PAD(pinset))
    {
      uintptr_t protse = base + PORT_PROTSE_OFFSET;
      tc4x_prot_set_config(protse);
      putreg32(BIT(pin), base + PORT_PCSRSEL_OFFSET);
      tc4x_prot_set_run(protse);
    }

  prot = base + PORT_ACCGRP_PROTE(CONFIG_TC4X_GPIO_ACCESS_GROUP);
  tc4x_prot_set_config(prot);

  putreg32(drvcfg, base + PORT_PADCFG_DRVCFG(pin));
  tc4x_prot_set_run(prot);

  spin_unlock_irqrestore(&g_gpio_lock, flags);

  return OK;
}
