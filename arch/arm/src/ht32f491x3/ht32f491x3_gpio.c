/****************************************************************************
 * arch/arm/src/ht32f491x3/ht32f491x3_gpio.c
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

#include "arm_internal.h"

#include "ht32f491x3_gpio.h"

#include "hardware/ht32f491x3_gpio.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void ht32f491x3_gpioconfig(uintptr_t base, unsigned int pin,
                           unsigned int mode, bool opendrain,
                           unsigned int drive, unsigned int pull,
                           unsigned int af)
{
  modifyreg32(base + HT32_GPIO_CFGR_OFFSET,
              HT32_GPIO_MODE_MASK(pin),
              HT32_GPIO_MODE_VALUE(pin, mode));

  modifyreg32(base + HT32_GPIO_OMODE_OFFSET,
              HT32_GPIO_PIN(pin),
              opendrain ? HT32_GPIO_PIN(pin) : 0);

  modifyreg32(base + HT32_GPIO_ODRVR_OFFSET,
              HT32_GPIO_ODRVR_MASK(pin),
              HT32_GPIO_ODRVR_VALUE(pin, drive));

  modifyreg32(base + HT32_GPIO_PULL_OFFSET,
              HT32_GPIO_PULL_MASK(pin),
              HT32_GPIO_PULL_VALUE(pin, pull));

  modifyreg32(base + HT32_GPIO_MUX_OFFSET(pin),
              HT32_GPIO_MUX_MASK(pin),
              mode == HT32_GPIO_MODE_ALTFN ?
              HT32_GPIO_MUX_VALUE(pin, af) : 0);
}

void ht32f491x3_gpiowrite(uintptr_t base, unsigned int pin, bool value)
{
  putreg32(HT32_GPIO_PIN(pin),
           base + (value ? HT32_GPIO_SCR_OFFSET : HT32_GPIO_CLR_OFFSET));
}

bool ht32f491x3_gpioread(uintptr_t base, unsigned int pin)
{
  return (getreg32(base + HT32_GPIO_IDT_OFFSET) & HT32_GPIO_PIN(pin)) != 0;
}
