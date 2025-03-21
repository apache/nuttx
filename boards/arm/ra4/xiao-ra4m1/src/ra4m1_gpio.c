/****************************************************************************
 * boards/arm/ra4/xiao-ra4m1/src/ra4m1_gpio.c
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
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <arch/board/board.h>
#include <nuttx/ioexpander/gpio.h>

#include "ra_gpio.h"
#include "xiao-ra4m1.h"

#ifdef CONFIG_DEV_GPIO

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ragpio_dev_s
{
  struct gpio_dev_s gpio;
  uint8_t           id;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#if BOARD_NGPIOIN > 0
static int gpin_read(struct gpio_dev_s *dev, bool *value);
#endif
#if BOARD_NGPIOOUT > 0
static int gpout_read(struct gpio_dev_s *dev, bool *value);
static int gpout_write(struct gpio_dev_s *dev, bool value);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if BOARD_NGPIOIN > 0
static const struct gpio_operations_s gpin_ops =
{
  .go_read   = gpin_read,
  .go_write  = NULL,
  .go_attach = NULL,
  .go_enable = NULL,
};
#endif

#if BOARD_NGPIOOUT > 0
static const struct gpio_operations_s gpout_ops =
{
  .go_read   = gpout_read,
  .go_write  = gpout_write,
  .go_attach = NULL,
  .go_enable = NULL,
};
#endif

#if BOARD_NGPIOIN > 0
/* This array maps the GPIO pins used as INPUT */

static const gpio_pinset_t g_gpioinputs[BOARD_NGPIOIN] =
{
  GPIO_IN1,
};

static struct ragpio_dev_s g_gpin[BOARD_NGPIOIN];
#endif

#if BOARD_NGPIOOUT > 0
/* This array maps the GPIO pins used as OUTPUT */

static const gpio_pinset_t g_gpiooutputs[BOARD_NGPIOOUT] =
{
  GPIO_OUT1
};

static struct ragpio_dev_s g_gpout[BOARD_NGPIOOUT];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if BOARD_NGPIOIN > 0

static int gpin_read(struct gpio_dev_s *dev, bool *value)
{
  struct ragpio_dev_s *ragpio = (struct ragpio_dev_s *)dev;

  DEBUGASSERT(ragpio != NULL && value != NULL);
  DEBUGASSERT(ragpio->id < BOARD_NGPIOIN);
  gpioinfo("Reading...");

  *value = ra_gpioread(g_gpioinputs[ragpio->id]);
  return OK;
}

#endif

#if BOARD_NGPIOOUT > 0

static int gpout_read(struct gpio_dev_s *dev, bool *value)
{
  struct ragpio_dev_s *ragpio = (struct ragpio_dev_s *)dev;

  DEBUGASSERT(ragpio != NULL && value != NULL);
  DEBUGASSERT(ragpio->id < BOARD_NGPIOOUT);
  gpioinfo("Reading...");

  *value = ra_gpioread(g_gpiooutputs[ragpio->id]);
  return OK;
}

static int gpout_write(struct gpio_dev_s *dev, bool value)
{
  struct ragpio_dev_s *ragpio = (struct ragpio_dev_s *)dev;

  DEBUGASSERT(ragpio != NULL);
  DEBUGASSERT(ragpio->id < BOARD_NGPIOOUT);
  gpioinfo("Writing %d", (int)value);

  ra_gpiowrite(g_gpiooutputs[ragpio->id], value);
  return OK;
}

#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_gpio_initialize
 *
 * Description:
 *   Initialize GPIO drivers for use with /apps/examples/gpio
 *
 ****************************************************************************/

int ra_gpio_initialize(void)
{
  int pincount = 0;
  int i;
  int ret = 0;

#if BOARD_NGPIOIN > 0
  for (i = 0; i < BOARD_NGPIOIN; i++, pincount++)
    {
      /* Setup and register the GPIO pin */

      g_gpin[i].gpio.gp_pintype = GPIO_INPUT_PIN;
      g_gpin[i].gpio.gp_ops     = &gpin_ops;
      g_gpin[i].id              = i;
      ret = gpio_pin_register(&g_gpin[i].gpio, pincount);
      if (ret < 0)
        {
          gpioerr("GPIOIN(%d): gpio_pin_register failed: %d", i, ret);
          return ret;
        }

      /* Configure the pin that will be used as input */

      ra_configgpio(g_gpioinputs[i]);
    }
#endif

#if BOARD_NGPIOOUT > 0
  for (i = 0; i < BOARD_NGPIOOUT; i++, pincount++)
    {
      /* Setup and register the GPIO pin */

      g_gpout[i].gpio.gp_pintype = GPIO_OUTPUT_PIN;
      g_gpout[i].gpio.gp_ops     = &gpout_ops;
      g_gpout[i].id              = i;
      ret = gpio_pin_register(&g_gpout[i].gpio, pincount);
      if (ret < 0)
        {
          gpioerr("GPIOOUT(%d): gpio_pin_register failed: %d", i, ret);
          return ret;
        }

      /* Configure the pin that will be used as output */

      ra_configgpio(g_gpiooutputs[i]);
    }
#endif

  return ret;
}
#endif
