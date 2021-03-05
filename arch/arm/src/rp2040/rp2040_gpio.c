/****************************************************************************
 * arch/arm/src/rp2040/rp2040_gpio.c
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
#include <debug.h>

#include <nuttx/arch.h>

#include <arch/board/board.h>

#include "arm_arch.h"

#include "chip.h"
#include "rp2040_gpio.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void rp2040_gpio_set_function(uint32_t gpio, uint32_t func)
{
  modbits_reg32(RP2040_PADS_BANK0_GPIO_IE,
                RP2040_PADS_BANK0_GPIO_IE | RP2040_PADS_BANK0_GPIO_OD,
                RP2040_PADS_BANK0_GPIO(gpio));

  putreg32(func & RP2040_IO_BANK0_GPIO_CTRL_FUNCSEL_MASK,
           RP2040_IO_BANK0_GPIO_CTRL(gpio));
}

void rp2040_gpio_set_pulls(uint32_t gpio, int up, int down)
{
  modbits_reg32((up   ? RP2040_PADS_BANK0_GPIO_PUE : 0) |
                (down ? RP2040_PADS_BANK0_GPIO_PDE : 0),
                RP2040_PADS_BANK0_GPIO_PUE | RP2040_PADS_BANK0_GPIO_PDE,
                RP2040_PADS_BANK0_GPIO(gpio));
}

void rp2040_gpio_init(uint32_t gpio)
{
  rp2040_gpio_setdir(gpio, false);
  rp2040_gpio_put(gpio, false);
  rp2040_gpio_set_function(gpio, RP2040_GPIO_FUNC_SIO);
}
