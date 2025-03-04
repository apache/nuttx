/****************************************************************************
 * boards/arm/at32/at32f437-mini/src/at32_gpio.c
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

#include <nuttx/clock.h>
#include <nuttx/wdog.h>
#include <nuttx/ioexpander/gpio.h>

#include <arch/board/board.h>

#include "chip.h"
#include "at32_gpio.h"
#include "at32f437-mini.h"

#if defined(CONFIG_DEV_GPIO) && !defined(CONFIG_GPIO_LOWER_HALF)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct at32gpio_dev_s
{
  struct gpio_dev_s gpio;
  uint8_t id;
};

struct at32gpint_dev_s
{
  struct at32gpio_dev_s at32gpio;
  pin_interrupt_t callback;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int gpin_read(struct gpio_dev_s *dev, bool *value);
static int gpout_read(struct gpio_dev_s *dev, bool *value);
static int gpout_write(struct gpio_dev_s *dev, bool value);
static int gpint_read(struct gpio_dev_s *dev, bool *value);
static int gpint_attach(struct gpio_dev_s *dev,
                        pin_interrupt_t callback);
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

/* This array maps the GPIO pins used as INPUT */

#if (BOARD_NGPIOIN > 0)
static const uint32_t g_gpioinputs[BOARD_NGPIOIN] =
    {
#if 0
      GPIO_IN1,
#endif
};

static struct at32gpio_dev_s g_gpin[BOARD_NGPIOIN];

#endif

/* This array maps the GPIO pins used as OUTPUT */

static const uint32_t g_gpiooutputs[BOARD_NGPIOOUT] =
    {
      GPIO_OUT1,
};

static struct at32gpio_dev_s g_gpout[BOARD_NGPIOOUT];

/* This array maps the GPIO pins used as INTERRUPT INPUTS */

static const uint32_t g_gpiointinputs[BOARD_NGPIOINT] =
    {
#if 0
      GPIO_INT1,
#endif
};

static struct at32gpint_dev_s g_gpint[BOARD_NGPIOINT];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int at32gpio_interrupt(int irq, void *context, void *arg)
{
  struct at32gpint_dev_s *at32gpint =
      (struct at32gpint_dev_s *)arg;

  DEBUGASSERT(at32gpint != NULL && at32gpint->callback != NULL);
  gpioinfo("Interrupt! callback=%p\n", at32gpint->callback);

  at32gpint->callback(&at32gpint->at32gpio.gpio,
                      at32gpint->at32gpio.id);
  return OK;
}

static int gpin_read(struct gpio_dev_s *dev, bool *value)
{
  struct at32gpio_dev_s *at32gpio =
      (struct at32gpio_dev_s *)dev;

  DEBUGASSERT(at32gpio != NULL && value != NULL);
  DEBUGASSERT(at32gpio->id < BOARD_NGPIOIN);
  gpioinfo("Reading...\n");

  *value = at32_gpioread(g_gpioinputs[at32gpio->id]);
  return OK;
}

static int gpout_read(struct gpio_dev_s *dev, bool *value)
{
  struct at32gpio_dev_s *at32gpio =
      (struct at32gpio_dev_s *)dev;

  DEBUGASSERT(at32gpio != NULL && value != NULL);
  DEBUGASSERT(at32gpio->id < BOARD_NGPIOOUT);
  gpioinfo("Reading...\n");

  *value = at32_gpioread(g_gpiooutputs[at32gpio->id]);
  return OK;
}

static int gpout_write(struct gpio_dev_s *dev, bool value)
{
  struct at32gpio_dev_s *at32gpio =
      (struct at32gpio_dev_s *)dev;

  DEBUGASSERT(at32gpio != NULL);
  DEBUGASSERT(at32gpio->id < BOARD_NGPIOOUT);
  gpioinfo("Writing %d\n", (int)value);

  at32_gpiowrite(g_gpiooutputs[at32gpio->id], value);
  return OK;
}

static int gpint_read(struct gpio_dev_s *dev, bool *value)
{
  struct at32gpint_dev_s *at32gpint =
      (struct at32gpint_dev_s *)dev;

  DEBUGASSERT(at32gpint != NULL && value != NULL);
  DEBUGASSERT(at32gpint->at32gpio.id < BOARD_NGPIOINT);
  gpioinfo("Reading int pin...\n");

  *value = at32_gpioread(g_gpiointinputs[at32gpint->at32gpio.id]);
  return OK;
}

static int gpint_attach(struct gpio_dev_s *dev,
                        pin_interrupt_t callback)
{
  struct at32gpint_dev_s *at32gpint =
      (struct at32gpint_dev_s *)dev;

  gpioinfo("Attaching the callback\n");

  /* Make sure the interrupt is disabled */

  at32_gpiosetevent(g_gpiointinputs[at32gpint->at32gpio.id], false,
                    false, false, NULL, NULL);

  gpioinfo("Attach %p\n", callback);
  at32gpint->callback = callback;
  return OK;
}

static int gpint_enable(struct gpio_dev_s *dev, bool enable)
{
  struct at32gpint_dev_s *at32gpint =
      (struct at32gpint_dev_s *)dev;

  if (enable)
    {
      if (at32gpint->callback != NULL)
        {
          gpioinfo("Enabling the interrupt\n");

          /* Configure the interrupt for falling edge */

          at32_gpiosetevent(g_gpiointinputs[at32gpint->at32gpio.id],
                            false, true, false, at32gpio_interrupt,
                            &g_gpint[at32gpint->at32gpio.id]);
        }
    }
  else
    {
      gpioinfo("Disable the interrupt\n");
      at32_gpiosetevent(g_gpiointinputs[at32gpint->at32gpio.id],
                        false, false, false, NULL, NULL);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: at32_gpio_initialize
 *
 * Description:
 *   Initialize GPIO drivers for use with /apps/examples/gpio
 *
 ****************************************************************************/

int at32_gpio_initialize(void)
{
  int i;
  int pincount = 0;

#if (BOARD_NGPIOIN > 0)
  for (i = 0; i < BOARD_NGPIOIN; i++)
    {
      /* Setup and register the GPIO pin */

      g_gpin[i].gpio.gp_pintype = GPIO_INPUT_PIN;
      g_gpin[i].gpio.gp_ops = &gpin_ops;
      g_gpin[i].id = i;
      gpio_pin_register(&g_gpin[i].gpio, pincount);

      /* Configure the pin that will be used as input */

      at32_configgpio(g_gpioinputs[i]);

      pincount++;
    }
#endif

#if (BOARD_NGPIOOUT > 0)
  for (i = 0; i < BOARD_NGPIOOUT; i++)
    {
      /* Setup and register the GPIO pin */

      g_gpout[i].gpio.gp_pintype = GPIO_OUTPUT_PIN;
      g_gpout[i].gpio.gp_ops = &gpout_ops;
      g_gpout[i].id = i;
      gpio_pin_register(&g_gpout[i].gpio, pincount);

      /* Configure the pin that will be used as output */

      at32_gpiowrite(g_gpiooutputs[i], 0);
      at32_configgpio(g_gpiooutputs[i]);

      pincount++;
    }
#endif

#if (BOARD_NGPIOINT > 0)
  for (i = 0; i < BOARD_NGPIOINT; i++)
    {
      /* Setup and register the GPIO pin */

      g_gpint[i].at32gpio.gpio.gp_pintype = GPIO_INTERRUPT_PIN;
      g_gpint[i].at32gpio.gpio.gp_ops = &gpint_ops;
      g_gpint[i].at32gpio.id = i;
      gpio_pin_register(&g_gpint[i].at32gpio.gpio, pincount);

      /* Configure the pin that will be used as interrupt input */

      at32_configgpio(g_gpiointinputs[i]);

      pincount++;
    }
#endif

  return 0;
}
#endif /* CONFIG_DEV_GPIO && !CONFIG_GPIO_LOWER_HALF */
