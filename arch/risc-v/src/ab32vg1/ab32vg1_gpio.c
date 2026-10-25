/****************************************************************************
 * arch/risc-v/src/ab32vg1/ab32vg1_gpio.c
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
#include <arch/irq.h>

#include "riscv_internal.h"
#include "chip.h"
#include "ab32vg1_gpio.h"
#include "hardware/ab32vg1_gpio.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uintptr_t g_portbase[] =
{
  AB32VG1_GPIOA_BASE,
  AB32VG1_GPIOB_BASE,
  AB32VG1_GPIOE_BASE,
  AB32VG1_GPIOF_BASE,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uintptr_t ab32vg1_portbase(gpio_pinset_t pinset)
{
  return g_portbase[(pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT];
}

static inline uint32_t ab32vg1_pinmask(gpio_pinset_t pinset)
{
  return 1 << ((pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ab32vg1_configgpio
 ****************************************************************************/

int ab32vg1_configgpio(gpio_pinset_t cfgset)
{
  uintptr_t base;
  uint32_t mask;
  irqstate_t flags;

  if (((cfgset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT) >= 4)
    {
      return -EINVAL;
    }

  base = ab32vg1_portbase(cfgset);
  mask = ab32vg1_pinmask(cfgset);

  flags = up_irq_save();

  /* Digital GPIO function */

  modifyreg32(base + AB32VG1_GPIO_FEN_OFFSET, mask, 0);
  modifyreg32(base + AB32VG1_GPIO_DE_OFFSET, 0, mask);

  /* Pull resistors */

  modifyreg32(base + AB32VG1_GPIO_PU_OFFSET, mask, 0);
  modifyreg32(base + AB32VG1_GPIO_PD_OFFSET, mask, 0);
  modifyreg32(base + AB32VG1_GPIO_PU200K_OFFSET, mask, 0);
  modifyreg32(base + AB32VG1_GPIO_PD200K_OFFSET, mask, 0);
  modifyreg32(base + AB32VG1_GPIO_PU300_OFFSET, mask, 0);
  modifyreg32(base + AB32VG1_GPIO_PD300_OFFSET, mask, 0);

  switch (cfgset & GPIO_PULL_MASK)
    {
      case GPIO_PULLUP:
        modifyreg32(base + AB32VG1_GPIO_PU_OFFSET, 0, mask);
        break;

      case GPIO_PULLDOWN:
        modifyreg32(base + AB32VG1_GPIO_PD_OFFSET, 0, mask);
        break;

      default:
        break;
    }

  /* Direction; set the output level before driving the pin */

  if (cfgset & GPIO_OUTPUT)
    {
      putreg32(mask, base + ((cfgset & GPIO_VALUE_ONE) ?
                             AB32VG1_GPIO_SET_OFFSET :
                             AB32VG1_GPIO_CLR_OFFSET));
      modifyreg32(base + AB32VG1_GPIO_DIR_OFFSET, mask, 0);
    }
  else
    {
      modifyreg32(base + AB32VG1_GPIO_DIR_OFFSET, 0, mask);
    }

  up_irq_restore(flags);
  return OK;
}

/****************************************************************************
 * Name: ab32vg1_gpiowrite
 ****************************************************************************/

void ab32vg1_gpiowrite(gpio_pinset_t pinset, bool value)
{
  putreg32(ab32vg1_pinmask(pinset), ab32vg1_portbase(pinset) +
           (value ? AB32VG1_GPIO_SET_OFFSET : AB32VG1_GPIO_CLR_OFFSET));
}

/****************************************************************************
 * Name: ab32vg1_gpioread
 ****************************************************************************/

bool ab32vg1_gpioread(gpio_pinset_t pinset)
{
  return (getreg32(ab32vg1_portbase(pinset) + AB32VG1_GPIO_DATA_OFFSET) &
          ab32vg1_pinmask(pinset)) != 0;
}
