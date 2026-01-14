/****************************************************************************
 * boards/arm/nrf52/nrf52840-dk/src/nrf52_gpio.c
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
#include <debug.h>

#include <nuttx/ioexpander/gpio.h>

#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "nrf52840-dk.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct nrf52gpio_dev_s
{
  struct gpio_dev_s gpio;
  uint8_t           id;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int gpin_read(struct gpio_dev_s *dev, bool *value);
static int gpout_read(struct gpio_dev_s *dev, bool *value);
static int gpout_write(struct gpio_dev_s *dev, bool value);

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

static const struct gpio_operations_s gpout_ops =
{
  .go_read   = gpout_read,
  .go_write  = gpout_write,
  .go_attach = NULL,
  .go_enable = NULL,
};

/* Buttons as inputs */

static const uint32_t g_gpioinputs[NUM_BUTTONS] =
{
  GPIO_BUTTON1,
  GPIO_BUTTON2,
  GPIO_BUTTON3,
  GPIO_BUTTON4
};

static struct nrf52gpio_dev_s g_gpin[NUM_BUTTONS];

/* Leds as outputs */

static const uint32_t g_gpiooutputs[BOARD_NLEDS] =
{
#if 0 < BOARD_NLEDS
  GPIO_LED1,
#endif
#if 1 < BOARD_NLEDS
  GPIO_LED2,
#endif
#if 2 < BOARD_NLEDS
  GPIO_LED3,
#endif
#if 3 < BOARD_NLEDS
  GPIO_LED4,
#endif
};

static struct nrf52gpio_dev_s g_gpout[BOARD_NLEDS];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int gpin_read(struct gpio_dev_s *dev, bool *value)
{
  struct nrf52gpio_dev_s *io = (struct nrf52gpio_dev_s *)dev;

  *value = nrf52_gpio_read(g_gpioinputs[io->id]);
  return OK;
}

static int gpout_read(struct gpio_dev_s *dev, bool *value)
{
  struct nrf52gpio_dev_s *io = (struct nrf52gpio_dev_s *)dev;

  *value = nrf52_gpio_read(g_gpiooutputs[io->id]);
  return OK;
}

static int gpout_write(struct gpio_dev_s *dev, bool value)
{
  struct nrf52gpio_dev_s *io = (struct nrf52gpio_dev_s *)dev;

  nrf52_gpio_write(g_gpiooutputs[io->id], value);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_gpioleds_initialize
 *
 * Description:
 *   Initialize GPIO devices with board LEDS and buttons.
 *
 ****************************************************************************/

int nrf52_gpioleds_initialize(void)
{
  int i;
  int pincount = 0;

  /* Inputs */

  for (i = 0; i < NUM_BUTTONS; i++)
    {
      /* Setup and register the GPIO pin */

      g_gpin[i].gpio.gp_pintype = GPIO_INPUT_PIN;
      g_gpin[i].gpio.gp_ops     = &gpin_ops;
      g_gpin[i].id              = i;
      gpio_pin_register(&g_gpin[i].gpio, pincount);

      /* Configure the pin that will be used as input */

      nrf52_gpio_config(g_gpioinputs[i]);

      pincount++;
    }

  /* Outputs */

  for (i = 0; i < BOARD_NLEDS; i++)
    {
      /* Setup and register the GPIO pin */

      g_gpout[i].gpio.gp_pintype = GPIO_OUTPUT_PIN;
      g_gpout[i].gpio.gp_ops     = &gpout_ops;
      g_gpout[i].id              = i;
      gpio_pin_register(&g_gpout[i].gpio, pincount);

      /* Configure the pin that will be used as input */

      nrf52_gpio_config(g_gpiooutputs[i]);

      pincount++;
    }

  return OK;
}
