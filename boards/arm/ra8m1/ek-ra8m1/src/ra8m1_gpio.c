/****************************************************************************
 * boards/arm/ra8m1/ek-ra8m1/src/ra8m1_gpio.c
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
#include <assert.h>

#include <nuttx/ioexpander/gpio.h>

#include <arch/board/board.h>

#include "ra_gpio.h"
#include "ek_ra8m1.h"

#ifdef CONFIG_DEV_GPIO

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ra8m1_gpio_dev_s
{
  struct gpio_dev_s gpio;
  uint8_t id;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int gpin_read(FAR struct gpio_dev_s *dev, FAR bool *value);
static int gpout_read(FAR struct gpio_dev_s *dev, FAR bool *value);
static int gpout_write(FAR struct gpio_dev_s *dev, bool value);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct gpio_operations_s g_gpin_ops =
{
  .go_read   = gpin_read,
  .go_write  = NULL,
  .go_attach = NULL,
  .go_enable = NULL,
};

static const struct gpio_operations_s g_gpout_ops =
{
  .go_read   = gpout_read,
  .go_write  = gpout_write,
  .go_attach = NULL,
  .go_enable = NULL,
};

/* The Arduino Uno shield header's D2-D5 (inputs) and D6-D13 (outputs), for
 * apps that use the generic GPIO expander interface (e.g.
 * apps/examples/gpio, apps/examples/timer_gpio).  See GPIO_ARDUINO_Dn in
 * board.h for the pin mapping this comes from.
 */

static const gpio_pinset_t g_gpioinputs[BOARD_NGPIOIN] =
{
  GPIO_ARDUINO_D2, GPIO_ARDUINO_D3, GPIO_ARDUINO_D4, GPIO_ARDUINO_D5
};

static const gpio_pinset_t g_gpiooutputs[BOARD_NGPIOOUT] =
{
  GPIO_ARDUINO_D6, GPIO_ARDUINO_D7, GPIO_ARDUINO_D8, GPIO_ARDUINO_D9,
  GPIO_ARDUINO_D10, GPIO_ARDUINO_D11, GPIO_ARDUINO_D12, GPIO_ARDUINO_D13
};

static struct ra8m1_gpio_dev_s g_gpin[BOARD_NGPIOIN];
static struct ra8m1_gpio_dev_s g_gpout[BOARD_NGPIOOUT];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int gpin_read(FAR struct gpio_dev_s *dev, FAR bool *value)
{
  FAR struct ra8m1_gpio_dev_s *ra8m1gpio =
    (FAR struct ra8m1_gpio_dev_s *)dev;

  DEBUGASSERT(ra8m1gpio != NULL && value != NULL);
  DEBUGASSERT(ra8m1gpio->id < BOARD_NGPIOIN);

  *value = ra_gpioread(g_gpioinputs[ra8m1gpio->id]);
  return OK;
}

static int gpout_read(FAR struct gpio_dev_s *dev, FAR bool *value)
{
  FAR struct ra8m1_gpio_dev_s *ra8m1gpio =
    (FAR struct ra8m1_gpio_dev_s *)dev;

  DEBUGASSERT(ra8m1gpio != NULL && value != NULL);
  DEBUGASSERT(ra8m1gpio->id < BOARD_NGPIOOUT);

  *value = ra_gpioread(g_gpiooutputs[ra8m1gpio->id]);
  return OK;
}

static int gpout_write(FAR struct gpio_dev_s *dev, bool value)
{
  FAR struct ra8m1_gpio_dev_s *ra8m1gpio =
    (FAR struct ra8m1_gpio_dev_s *)dev;

  DEBUGASSERT(ra8m1gpio != NULL);
  DEBUGASSERT(ra8m1gpio->id < BOARD_NGPIOOUT);

  ra_gpiowrite(g_gpiooutputs[ra8m1gpio->id], value);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra8m1_gpio_initialize
 *
 * Description:
 *   Register the Arduino shield header's D2-D5 (inputs) and D6-D13
 *   (outputs) with the generic GPIO expander driver, as /dev/gpio0-3 and
 *   /dev/gpio4-11 respectively.
 *
 ****************************************************************************/

int ra8m1_gpio_initialize(void)
{
  int i;
  int pincount = 0;

  for (i = 0; i < BOARD_NGPIOIN; i++)
    {
      g_gpin[i].gpio.gp_pintype = GPIO_INPUT_PIN;
      g_gpin[i].gpio.gp_ops     = &g_gpin_ops;
      g_gpin[i].id              = i;
      gpio_pin_register(&g_gpin[i].gpio, pincount);

      ra_configgpio(g_gpioinputs[i]);

      pincount++;
    }

  for (i = 0; i < BOARD_NGPIOOUT; i++)
    {
      g_gpout[i].gpio.gp_pintype = GPIO_OUTPUT_PIN;
      g_gpout[i].gpio.gp_ops     = &g_gpout_ops;
      g_gpout[i].id              = i;
      gpio_pin_register(&g_gpout[i].gpio, pincount);

      ra_gpiowrite(g_gpiooutputs[i], false);
      ra_configgpio(g_gpiooutputs[i]);

      pincount++;
    }

  return OK;
}

#endif /* CONFIG_DEV_GPIO */
