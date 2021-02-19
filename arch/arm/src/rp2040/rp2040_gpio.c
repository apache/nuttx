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

#include "hardware/rp2040_pads_bank0.h"
#include "hardware/rp2040_io_bank0.h"
#include "hardware/rp2040_sio.h"

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
