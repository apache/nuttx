/****************************************************************************
 * boards/arm/nrf53/nrf5340-dk/src/nrf53_gpio.c
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

#include "nrf53_gpio.h"

#include <arch/board/board.h>
#include <assert.h>
#include <debug.h>
#include <nuttx/clock.h>
#include <nuttx/config.h>
#include <nuttx/ioexpander/gpio.h>
#include <nuttx/wdog.h>
#include <stdbool.h>

#include "chip.h"
#include "nrf5340-dk.h"
#include "nrf53_gpiote.h"

#if defined(CONFIG_DEV_GPIO) && !defined(CONFIG_GPIO_LOWER_HALF)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct nrf53gpio_dev_s
{
  struct gpio_dev_s gpio;
  uint8_t id;
};

struct nrf53gpint_dev_s
{
  struct nrf53gpio_dev_s nrf53gpio;
  pin_interrupt_t callback;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int gpin_read(struct gpio_dev_s *dev, bool *value);
static int gpout_read(struct gpio_dev_s *dev, bool *value);
static int gpout_write(struct gpio_dev_s *dev, bool value);
static int gpint_read(struct gpio_dev_s *dev, bool *value);
static int gpint_attach(struct gpio_dev_s *dev, pin_interrupt_t callback);
static int gpint_enable(struct gpio_dev_s *dev, bool enable);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct gpio_operations_s gpin_ops =
{
    .go_read = gpin_read,
    .go_write = NULL,
    .go_attach = NULL,
    .go_enable = NULL,
};

static const struct gpio_operations_s gpout_ops =
{
    .go_read = gpout_read,
    .go_write = gpout_write,
    .go_attach = NULL,
    .go_enable = NULL,
};

static const struct gpio_operations_s gpint_ops =
{
    .go_read = gpint_read,
    .go_write = NULL,
    .go_attach = gpint_attach,
    .go_enable = gpint_enable,
};

#if BOARD_NGPIOIN > 0
/* This array maps the GPIO pins used as INPUT */

static const uint32_t g_gpioinputs[BOARD_NGPIOIN] =
{
    GPIO_IN1,
    GPIO_IN2,
};

static struct nrf53gpio_dev_s g_gpin[BOARD_NGPIOIN];
#endif

#if BOARD_NGPIOOUT > 0
/* This array maps the GPIO pins used as OUTPUT */

static const uint32_t g_gpiooutputs[BOARD_NGPIOOUT] =
{
    GPIO_OUT1,
    GPIO_OUT2,
    GPIO_OUT3,
    GPIO_OUT4,
};

static struct nrf53gpio_dev_s g_gpout[BOARD_NGPIOOUT];
#endif /* BOARD_NGPIOOUT > 0 */

#if BOARD_NGPIOINT > 0
/* This array maps the GPIO pins used as INTERRUPT INPUTS */

static const uint32_t g_gpiointinputs[BOARD_NGPIOINT] =
{
    GPIO_INT1,
    GPIO_INT2,
};

static struct nrf53gpint_dev_s g_gpint[BOARD_NGPIOINT];
#endif /* BOARD_NGPIOINT > 0 */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int nrf53gpio_interrupt(int irq, void *context, void *arg)
{
  struct nrf53gpint_dev_s *nrf53gpint = (struct nrf53gpint_dev_s *)arg;

  DEBUGASSERT(nrf53gpint != NULL && nrf53gpint->callback != NULL);
  gpioinfo("Interrupt! callback=%p\n", nrf53gpint->callback);

  nrf53gpint->callback(&nrf53gpint->nrf53gpio.gpio,
                        nrf53gpint->nrf53gpio.id);
  return OK;
}

static int gpin_read(struct gpio_dev_s *dev, bool *value)
{
  struct nrf53gpio_dev_s *nrf53gpio = (struct nrf53gpio_dev_s *)dev;

  DEBUGASSERT(nrf53gpio != NULL && value != NULL);
  DEBUGASSERT(nrf53gpio->id < BOARD_NGPIOIN);
  gpioinfo("Reading...\n");

  *value = nrf53_gpio_read(g_gpioinputs[nrf53gpio->id]);
  return OK;
}

static int gpout_read(struct gpio_dev_s *dev, bool *value)
{
  struct nrf53gpio_dev_s *nrf53gpio = (struct nrf53gpio_dev_s *)dev;

  DEBUGASSERT(nrf53gpio != NULL && value != NULL);
  DEBUGASSERT(nrf53gpio->id < BOARD_NGPIOOUT);
  gpioinfo("Reading...\n");

  *value = nrf53_gpio_read(g_gpiooutputs[nrf53gpio->id]);
  return OK;
}

static int gpout_write(struct gpio_dev_s *dev, bool value)
{
  struct nrf53gpio_dev_s *nrf53gpio = (struct nrf53gpio_dev_s *)dev;

  DEBUGASSERT(nrf53gpio != NULL);
  DEBUGASSERT(nrf53gpio->id < BOARD_NGPIOOUT);
  gpioinfo("Writing %d\n", (int)value);

  nrf53_gpio_write(g_gpiooutputs[nrf53gpio->id], value);
  return OK;
}

static int gpint_read(struct gpio_dev_s *dev, bool *value)
{
  struct nrf53gpint_dev_s *nrf53gpint = (struct nrf53gpint_dev_s *)dev;

  DEBUGASSERT(nrf53gpint != NULL && value != NULL);
  DEBUGASSERT(nrf53gpint->nrf53gpio.id < BOARD_NGPIOINT);
  gpioinfo("Reading int pin...\n");

  *value = nrf53_gpio_read(g_gpiointinputs[nrf53gpint->nrf53gpio.id]);
  return OK;
}

static int gpint_attach(struct gpio_dev_s *dev, pin_interrupt_t callback)
{
  struct nrf53gpint_dev_s *nrf53gpint = (struct nrf53gpint_dev_s *)dev;

  gpioinfo("Attaching the callback\n");

  /* Make sure the interrupt is disabled */

  nrf53_gpiote_set_event(g_gpiointinputs[nrf53gpint->nrf53gpio.id], false,
                         false, NULL, NULL);

  gpioinfo("Attach %p\n", callback);
  nrf53gpint->callback = callback;
  return OK;
}

static int gpint_enable(struct gpio_dev_s *dev, bool enable)
{
  struct nrf53gpint_dev_s *nrf53gpint = (struct nrf53gpint_dev_s *)dev;

  if (enable)
    {
      if (nrf53gpint->callback != NULL)
        {
          gpioinfo("Enabling the interrupt\n");

          /* Configure the interrupt for rising edge */

          nrf53_gpiote_set_event(g_gpiointinputs[nrf53gpint->nrf53gpio.id],
                                true,
                                false,
                                nrf53gpio_interrupt,
                                &g_gpint[nrf53gpint->nrf53gpio.id]);
        }
    }
  else
    {
    gpioinfo("Disable the interrupt\n");
    nrf53_gpiote_set_event(g_gpiointinputs[nrf53gpint->nrf53gpio.id], false,
                           false, NULL, NULL);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf53_gpio_initialize
 *
 * Description:
 *   Initialize GPIO drivers for use with /apps/examples/gpio
 *
 ****************************************************************************/

int nrf53_gpio_initialize(void)
{
  int i = 0;
  int pincount = 0;

#if BOARD_NGPIOIN > 0
  for (i = 0; i < BOARD_NGPIOIN; i++)
    {
      /* Setup and register the GPIO pin */

      g_gpin[i].gpio.gp_pintype = GPIO_INPUT_PIN;
      g_gpin[i].gpio.gp_ops = &gpin_ops;
      g_gpin[i].id = i;
      gpio_pin_register(&g_gpin[i].gpio, pincount);

      /* Configure the pin that will be used as input */

      nrf53_gpio_config(g_gpioinputs[i]);

      pincount++;
    }
#endif

#if BOARD_NGPIOOUT > 0
  for (i = 0; i < BOARD_NGPIOOUT; i++)
    {
      /* Setup and register the GPIO pin */

      g_gpout[i].gpio.gp_pintype = GPIO_OUTPUT_PIN;
      g_gpout[i].gpio.gp_ops = &gpout_ops;
      g_gpout[i].id = i;
      gpio_pin_register(&g_gpout[i].gpio, pincount);

      /* Configure the pin that will be used as output */

      nrf53_gpio_write(g_gpiooutputs[i], 0);
      nrf53_gpio_config(g_gpiooutputs[i]);

      pincount++;
    }
#endif

#if BOARD_NGPIOINT > 0
  for (i = 0; i < BOARD_NGPIOINT; i++)
    {
      /* Setup and register the GPIO pin */

      g_gpint[i].nrf53gpio.gpio.gp_pintype = GPIO_INTERRUPT_PIN;
      g_gpint[i].nrf53gpio.gpio.gp_ops = &gpint_ops;
      g_gpint[i].nrf53gpio.id = i;
      gpio_pin_register(&g_gpint[i].nrf53gpio.gpio, pincount);

      /* Configure the pin that will be used as interrupt input */

      nrf53_gpio_config(g_gpiointinputs[i]);

      pincount++;
    }
#endif

  return 0;
}

#endif /* CONFIG_DEV_GPIO && !CONFIG_GPIO_LOWER_HALF */
