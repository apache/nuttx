/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_gpio.c
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

#include <stdint.h>

#include "riscv_arch.h"

#include "hardware/mpfs_gpio.h"
#include "mpfs_gpio.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

static uintptr_t g_gpio_base[] =
{
  MPFS_GPIO0_LO_BASE,  /* Bank-0 Normal GPIO */
  MPFS_GPIO1_LO_BASE,  /* Bank-1 Normal GPIO */
  MPFS_GPIO2_LO_BASE   /* Bank-2 Fabric only */
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpfs_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 * Returned Value:
 *   OK on success
 *   ERROR on invalid port.
 *
 ****************************************************************************/

int mpfs_configgpio(gpio_pinset_t cfgset)
{
  uintptr_t baseaddr;
  uint32_t cfg = 0;
  uint8_t pin = (cfgset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  uint8_t bank = (cfgset & GPIO_BANK_MASK) >> GPIO_BANK_SHIFT;
  uint8_t irq_mode = (cfgset & GPIO_IRQ_MASK) >> GPIO_IRQ_SHIFT;

  if (bank == 3)
    {
      return ERROR;
    }

  /* REVISIT: limit the gpios as
   * bank0  0 - 13
   * bank1  0 - 23
   * bank2  0 - 31
   */

  baseaddr = g_gpio_base[bank] + (pin * sizeof(uint32_t));

  if (cfgset & GPIO_INPUT)
    {
      cfg |= GPIO_CONFIG_EN_IN;
    }

  if (cfgset & GPIO_OUTPUT)
    {
      cfg |= GPIO_CONFIG_EN_OUT;
    }

  if (cfgset & GPIO_BUFFER_ENABLE)
    {
      cfg |= GPIO_CONFIG_EN_OE_BUF;
    }

  if (cfgset & GPIO_IRQ_ENABLE)
    {
      cfg |= GPIO_CONFIG_EN_INT;

      /* Clear irq bit */

      putreg32(1 << pin, baseaddr + MPFS_GPIO_INTR_OFFSET);
    }

  /* set irq mode bits */

  irq_mode &= 7;
  cfg |= irq_mode << 5;

  putreg32(cfg, baseaddr);

  return OK;
}

/****************************************************************************
 * Name: mpfs_gpio_deinit
 *
 * Description:
 *   Deinit a GPIO (Set GPIO to input state)
 *
 * Returned Value:
 *   OK on success
 *   ERROR on invalid port.
 *
 ****************************************************************************/

/* REVISIT: */

int mpfs_gpio_deinit(uint8_t pin)
{
  mpfs_configgpio(pin);
  return OK;
}

/****************************************************************************
 * Name: mpfs_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void mpfs_gpiowrite(gpio_pinset_t pinset, bool value)
{
  uint8_t pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  uint8_t bank = (pinset & GPIO_BANK_MASK) >> GPIO_BANK_SHIFT;
  uintptr_t baseaddr = g_gpio_base[bank];

  if (bank == 3)
    {
      return;
    }

  if (value)
    {
      putreg32((1 << pin), baseaddr + MPFS_GPIO_SET_BITS_OFFSET);
    }
  else
    {
      putreg32((1 << pin), baseaddr + MPFS_GPIO_CLEAR_BITS_OFFSET);
    }
}

/****************************************************************************
 * Name: mpfs_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool mpfs_gpioread(gpio_pinset_t pinset)
{
  uint8_t pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  uint8_t bank = (pinset & GPIO_BANK_MASK) >> GPIO_BANK_SHIFT;
  uintptr_t baseaddr = g_gpio_base[bank];

  if (bank == 3)
    {
      return 0;
    }

  return ((getreg32(baseaddr + MPFS_GPIO_GPIN_OFFSET) & (1 << pin)) ? 1 : 0);
}

