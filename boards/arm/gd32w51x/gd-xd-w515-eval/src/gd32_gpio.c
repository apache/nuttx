/****************************************************************************
 * boards/arm/gd32w51x/gd-xd-w515-eval/src/gd32_gpio.c
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

#include <nuttx/irq.h>
#include <nuttx/ioexpander/gpio.h>

#include <arch/board/board.h>

#include "gd32w51x_gpio.h"
#include "gd32w51x_exti.h"
#include "gd-xd-w515-eval.h"

#if defined(CONFIG_DEV_GPIO) && !defined(CONFIG_GPIO_LOWER_HALF)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct gd32gpio_dev_s
{
  struct gpio_dev_s gpio;
  uint8_t id;
};

struct gd32gpint_dev_s
{
  struct gd32gpio_dev_s gd32gpio;
  pin_interrupt_t callback;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int gpout_read(struct gpio_dev_s *dev, bool *value);
static int gpout_write(struct gpio_dev_s *dev, bool value);
static int gpint_read(struct gpio_dev_s *dev, bool *value);
static int gpint_attach(struct gpio_dev_s *dev, pin_interrupt_t callback);
static int gpint_enable(struct gpio_dev_s *dev, bool enable);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct gpio_operations_s gpout_ops =
{
  .go_read   = gpout_read,
  .go_write  = gpout_write,
};

static const struct gpio_operations_s gpint_ops =
{
  .go_read   = gpint_read,
  .go_attach = gpint_attach,
  .go_enable = gpint_enable,
};

/* Outputs: the "Heater" high-power port */

static const uint32_t g_gpiooutputs[BOARD_NGPIOOUT] =
{
  GPIO_HEATER,
};

static struct gd32gpio_dev_s g_gpout[BOARD_NGPIOOUT];

/* Interrupt inputs: the USB power detection, the IRQ of the fingerprint
 * sensor and, when its driver does not use it, the IRQ of the battery
 * charger
 */

static const uint32_t g_gpiointinputs[BOARD_NGPIOINT] =
{
  GPIO_USB_DETECT,
  GPIO_FP_IRQ,
#ifndef CONFIG_GD30BC2416
  GPIO_CHARGER_IRQ,
#endif
};

static struct gd32gpint_dev_s g_gpint[BOARD_NGPIOINT];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int gpout_read(struct gpio_dev_s *dev, bool *value)
{
  struct gd32gpio_dev_s *gd32gpio = (struct gd32gpio_dev_s *)dev;

  DEBUGASSERT(gd32gpio->id < BOARD_NGPIOOUT && value != NULL);

  *value = gd32_gpio_read(g_gpiooutputs[gd32gpio->id]);
  return OK;
}

static int gpout_write(struct gpio_dev_s *dev, bool value)
{
  struct gd32gpio_dev_s *gd32gpio = (struct gd32gpio_dev_s *)dev;

  DEBUGASSERT(gd32gpio->id < BOARD_NGPIOOUT);

  gd32_gpio_write(g_gpiooutputs[gd32gpio->id], value);
  return OK;
}

static int gd32gpio_interrupt(int irq, void *context, void *arg)
{
  struct gd32gpint_dev_s *gd32gpint = (struct gd32gpint_dev_s *)arg;

  if (gd32gpint->callback != NULL)
    {
      gd32gpint->callback(&gd32gpint->gd32gpio.gpio,
                          gd32gpint->gd32gpio.id);
    }

  return OK;
}

static int gpint_read(struct gpio_dev_s *dev, bool *value)
{
  struct gd32gpint_dev_s *gd32gpint = (struct gd32gpint_dev_s *)dev;

  DEBUGASSERT(gd32gpint->gd32gpio.id < BOARD_NGPIOINT && value != NULL);

  *value = gd32_gpio_read(g_gpiointinputs[gd32gpint->gd32gpio.id]);
  return OK;
}

static int gpint_attach(struct gpio_dev_s *dev, pin_interrupt_t callback)
{
  struct gd32gpint_dev_s *gd32gpint = (struct gd32gpint_dev_s *)dev;

  /* Disable the interrupt while the callback changes */

  gd32_gpio_setevent(g_gpiointinputs[gd32gpint->gd32gpio.id], false, false,
                     false, NULL, NULL);
  gd32gpint->callback = callback;
  return OK;
}

static int gpint_enable(struct gpio_dev_s *dev, bool enable)
{
  struct gd32gpint_dev_s *gd32gpint = (struct gd32gpint_dev_s *)dev;
  uint32_t pinset = g_gpiointinputs[gd32gpint->gd32gpio.id];

  if (enable && gd32gpint->callback != NULL)
    {
      return gd32_gpio_setevent(pinset, true, true, false,
                                gd32gpio_interrupt, gd32gpint);
    }

  return gd32_gpio_setevent(pinset, false, false, false, NULL, NULL);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_gpio_initialize
 *
 * Description:
 *   Register the board GPIOs: /dev/gpio0 is the heater output, /dev/gpio1
 *   the USB power detection, /dev/gpio2 the IRQ of the fingerprint sensor
 *   and /dev/gpio3 the IRQ of the battery charger if its driver is not
 *   enabled (both edge interrupts).
 *
 ****************************************************************************/

int gd32_gpio_initialize(void)
{
  int pincount = 0;
  int i;

  for (i = 0; i < BOARD_NGPIOOUT; i++)
    {
      g_gpout[i].gpio.gp_pintype = GPIO_OUTPUT_PIN;
      g_gpout[i].gpio.gp_ops     = &gpout_ops;
      g_gpout[i].id              = i;

      gd32_gpio_config(g_gpiooutputs[i]);
      gpio_pin_register(&g_gpout[i].gpio, pincount++);
    }

  for (i = 0; i < BOARD_NGPIOINT; i++)
    {
      g_gpint[i].gd32gpio.gpio.gp_pintype = GPIO_INTERRUPT_BOTH_PIN;
      g_gpint[i].gd32gpio.gpio.gp_ops     = &gpint_ops;
      g_gpint[i].gd32gpio.id              = i;

      gd32_gpio_config(g_gpiointinputs[i]);
      gpio_pin_register(&g_gpint[i].gd32gpio.gpio, pincount++);
    }

  return OK;
}

#endif /* CONFIG_DEV_GPIO && !CONFIG_GPIO_LOWER_HALF */
