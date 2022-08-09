/****************************************************************************
 * boards/arm/samv7/same70-qmtech/src/sam_gpio.c
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

#include <nuttx/ioexpander/gpio.h>

#include "sam_gpio.h"

#include "same70-qmtech.h"

#ifdef CONFIG_DEV_GPIO

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct samgpio_dev_s
{
  struct gpio_dev_s gpio;
  uint8_t           id;
};

struct samgpint_dev_s
{
  struct samgpio_dev_s samgpio;
  pin_interrupt_t      callback;
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
#if BOARD_NGPIOINT > 0
static int gpint_read(struct gpio_dev_s *dev, bool *value);
static int gpint_attach(struct gpio_dev_s *dev,
                        pin_interrupt_t callback);
static int gpint_enable(struct gpio_dev_s *dev, bool enable);
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

#if BOARD_NGPIOINT > 0
static const struct gpio_operations_s gpint_ops =
{
  .go_read   = gpint_read,
  .go_write  = NULL,
  .go_attach = gpint_attach,
  .go_enable = gpint_enable,
};
#endif

#if BOARD_NGPIOIN > 0
/* This array maps the GPIO pins used as INPUT */

static const gpio_pinset_t g_gpioinputs[BOARD_NGPIOIN] =
{
  GPIO_IN1,
};

static struct samgpio_dev_s g_gpin[BOARD_NGPIOIN];
#endif

#if BOARD_NGPIOOUT > 0
/* This array maps the GPIO pins used as OUTPUT */

static const gpio_pinset_t g_gpiooutputs[BOARD_NGPIOOUT] =
{
  GPIO_OUT1
};

static struct samgpio_dev_s g_gpout[BOARD_NGPIOOUT];
#endif

#if BOARD_NGPIOINT > 0
/* This array maps the GPIO pins used as INTERRUPT INPUTS */

static const gpio_pinset_t g_gpiointinputs[BOARD_NGPIOINT] =
{
  GPIO_INT1
};

static const int g_gpiointirqs[BOARD_NGPIOINT] =
{
  GPIO_IRQ_INT1
};

static struct samgpint_dev_s g_gpint[BOARD_NGPIOINT];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if BOARD_NGPIOIN > 0

static int gpin_read(struct gpio_dev_s *dev, bool *value)
{
  struct samgpio_dev_s *samgpio = (struct samgpio_dev_s *)dev;

  DEBUGASSERT(samgpio != NULL && value != NULL);
  DEBUGASSERT(samgpio->id < BOARD_NGPIOIN);
  gpioinfo("Reading...");

  *value = sam_gpioread(g_gpioinputs[samgpio->id]);
  return OK;
}

#endif

#if BOARD_NGPIOOUT > 0

static int gpout_read(struct gpio_dev_s *dev, bool *value)
{
  struct samgpio_dev_s *samgpio = (struct samgpio_dev_s *)dev;

  DEBUGASSERT(samgpio != NULL && value != NULL);
  DEBUGASSERT(samgpio->id < BOARD_NGPIOOUT);
  gpioinfo("Reading...");

  *value = sam_gpioread(g_gpiooutputs[samgpio->id]);
  return OK;
}

static int gpout_write(struct gpio_dev_s *dev, bool value)
{
  struct samgpio_dev_s *samgpio = (struct samgpio_dev_s *)dev;

  DEBUGASSERT(samgpio != NULL);
  DEBUGASSERT(samgpio->id < BOARD_NGPIOOUT);
  gpioinfo("Writing %d", (int)value);

  sam_gpiowrite(g_gpiooutputs[samgpio->id], value);
  return OK;
}

#endif

#if BOARD_NGPIOINT > 0

static int samgpio_interrupt(int irq, void *context, void *arg)
{
  struct samgpint_dev_s *samgpint = (struct samgpint_dev_s *)arg;

  DEBUGASSERT(samgpint != NULL && samgpint->callback != NULL);
  gpioinfo("Interrupt callback=%p", samgpint->callback);

  samgpint->callback(&samgpint->samgpio.gpio, samgpint->samgpio.id);
  return OK;
}

static int gpint_read(struct gpio_dev_s *dev, bool *value)
{
  struct samgpint_dev_s *samgpint = (struct samgpint_dev_s *)dev;

  DEBUGASSERT(samgpint != NULL && value != NULL);
  DEBUGASSERT(samgpint->samgpio.id < BOARD_NGPIOINT);
  gpioinfo("Reading int pin...");

  *value = sam_gpioread(g_gpiointinputs[samgpint->samgpio.id]);
  return OK;
}

static int gpint_attach(struct gpio_dev_s *dev,
                        pin_interrupt_t callback)
{
  struct samgpint_dev_s *samgpint = (struct samgpint_dev_s *)dev;
  irqstate_t flags;

  gpioinfo("Attaching the callback");

  flags = enter_critical_section();

  sam_gpioirq(g_gpiointinputs[samgpint->samgpio.id]);

  /* Make sure the interrupt is disabled */

  irq_attach(g_gpiointirqs[samgpint->samgpio.id], samgpio_interrupt,
             &g_gpint[samgpint->samgpio.id]);

  leave_critical_section(flags);
  gpioinfo("Attach %p", callback);
  samgpint->callback = callback;
  return OK;
}

static int gpint_enable(struct gpio_dev_s *dev, bool enable)
{
  struct samgpint_dev_s *samgpint = (struct samgpint_dev_s *)dev;
  irqstate_t flags;

  flags = enter_critical_section();
  if (enable)
    {
      if (samgpint->callback != NULL)
        {
          gpioinfo("Enabling the interrupt");
          sam_gpioirqenable(g_gpiointirqs[samgpint->samgpio.id]);
        }
    }
  else
    {
      sam_gpioirqdisable(g_gpiointirqs[samgpint->samgpio.id]);
      gpioinfo("Disable the interrupt");
    }

  leave_critical_section(flags);
  return OK;
}

#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_gpio_initialize
 *
 * Description:
 *   Initialize GPIO drivers for use with /apps/examples/gpio
 *
 ****************************************************************************/

int sam_gpio_initialize(void)
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

      sam_configgpio(g_gpioinputs[i]);
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

      sam_configgpio(g_gpiooutputs[i]);
    }
#endif

#if BOARD_NGPIOINT > 0
  for (i = 0; i < BOARD_NGPIOINT; i++, pincount++)
    {
      /* Setup and register the GPIO pin */

      g_gpint[i].samgpio.gpio.gp_pintype = GPIO_INTERRUPT_PIN;
      g_gpint[i].samgpio.gpio.gp_ops     = &gpint_ops;
      g_gpint[i].samgpio.id              = i;
      ret = gpio_pin_register(&g_gpint[i].samgpio.gpio, pincount);
      if (ret < 0)
        {
          gpioerr("GPIOINT(%d): gpio_pin_register failed: %d", i, ret);
          return ret;
        }

      /* Configure the pin that will be used as interrupt input */

      sam_configgpio(g_gpiointinputs[i]);
    }
#endif

  return ret;
}
#endif
