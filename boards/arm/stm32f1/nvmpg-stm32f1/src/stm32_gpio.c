/****************************************************************************
 * boards/arm/stm32f1/nvmpg-stm32f1/src/stm32_gpio.c
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

/* The ten pendant keys exported as /dev/gpio0 .. /dev/gpio9 (inputs, true
 * means released).  PA0..PA3 and PC0..PC3 share EXTI lines, so the keys
 * cannot all be IRQ buttons and the application polls them instead.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <assert.h>

#include <nuttx/ioexpander/gpio.h>

#include <arch/board/board.h>

#include "chip.h"
#include "stm32.h"
#include "nvmpg-stm32f1.h"

#if defined(CONFIG_DEV_GPIO) && !defined(CONFIG_GPIO_LOWER_HALF)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32gpio_dev_s
{
  struct gpio_dev_s gpio;
  uint8_t id;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int gpin_read(FAR struct gpio_dev_s *dev, FAR bool *value);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct gpio_operations_s gpin_ops =
{
  .go_read   = gpin_read,
  .go_write  = NULL,
  .go_attach = NULL,
  .go_enable = NULL,
};

static const uint32_t g_keys[NVMPG_NKEYS] =
{
  GPIO_KEY0, GPIO_KEY1, GPIO_KEY2, GPIO_KEY3, GPIO_KEY4,
  GPIO_KEY5, GPIO_KEY6, GPIO_KEY7, GPIO_KEY8, GPIO_KEY9,
};

static struct stm32gpio_dev_s g_gpin[NVMPG_NKEYS];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int gpin_read(FAR struct gpio_dev_s *dev, FAR bool *value)
{
  FAR struct stm32gpio_dev_s *stm32gpio = (FAR struct stm32gpio_dev_s *)dev;

  DEBUGASSERT(stm32gpio != NULL && value != NULL);
  DEBUGASSERT(stm32gpio->id < NVMPG_NKEYS);

  *value = stm32_gpioread(g_keys[stm32gpio->id]);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_gpio_initialize
 ****************************************************************************/

int stm32_gpio_initialize(void)
{
  int i;

  for (i = 0; i < NVMPG_NKEYS; i++)
    {
      stm32_configgpio(g_keys[i]);

      g_gpin[i].gpio.gp_pintype = GPIO_INPUT_PIN;
      g_gpin[i].gpio.gp_ops     = &gpin_ops;
      g_gpin[i].id              = i;
      gpio_pin_register(&g_gpin[i].gpio, i);
    }

  return OK;
}

#endif /* CONFIG_DEV_GPIO && !CONFIG_GPIO_LOWER_HALF */
