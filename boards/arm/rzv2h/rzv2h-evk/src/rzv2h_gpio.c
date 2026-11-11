/****************************************************************************
 * boards/arm/rzv2h/rzv2h-evk/src/rzv2h_gpio.c
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
#include <nuttx/debug.h>
#include <errno.h>

#include <arch/board/board.h>
#include <nuttx/ioexpander/gpio.h>

#include "rzv2h_gpio.h"
#include "hardware/rzv2h_pinmap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_RZV2H_GPIO

struct rzv2hgpio_dev_s
{
  struct gpio_dev_s gpio;
  uint8_t id;
};

struct rzv2hgpint_dev_s
{
  struct rzv2hgpio_dev_s rzv2hgpio;
  pin_interrupt_t callback;
};

/****************************************************************************
 * Private Functions Prototypes
 ****************************************************************************/

#if BOARD_NGPIOIN > 0
static int gpin_read(struct gpio_dev_s *dev, bool *value);
static int gpin_attach(struct gpio_dev_s *dev, pin_interrupt_t callback);
static int gpin_enable(struct gpio_dev_s *dev, bool enable);

#endif
#if BOARD_NGPIOOUT > 0
static int gpout_read(struct gpio_dev_s *dev, bool *value);
static int gpout_write(struct gpio_dev_s *dev, bool value);

#endif
static int gpio_setpintype(struct gpio_dev_s *dev,
                           enum gpio_pintype_e gpio_pintype);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if BOARD_NGPIOIN > 0
static const struct gpio_operations_s gpin_ops =
{
  .go_read      = gpin_read,   .go_write        = NULL,
  .go_attach    = gpin_attach,
  .go_enable    = gpin_enable, .go_setpintype   = gpio_setpintype,
};
#endif

#if BOARD_NGPIOOUT > 0
static const struct gpio_operations_s gpout_ops =
{
  .go_read      = gpout_read, .go_write            = gpout_write,
  .go_attach    = NULL,
  .go_enable    = NULL,       .go_setpintype       = gpio_setpintype,
};
#endif

#if BOARD_NGPIOIN > 0

/* This array maps the GPIO pins used as INPUT */

static const gpio_pinset_t      g_gpioinputs[BOARD_NGPIOIN] =
{
  BOARD_P8_2_BUTTON
};

static struct rzv2hgpio_dev_s   g_gpin[BOARD_NGPIOIN];
#endif

#if BOARD_NGPIOOUT > 0

/* This array maps the GPIO pins used as OUTPUT */

static const gpio_pinset_t      g_gpiooutputs[BOARD_NGPIOOUT] =
{
  BOARD_P0_0_GPIO, BOARD_P0_1_GPIO
};

static struct rzv2hgpio_dev_s   g_gpout[BOARD_NGPIOOUT];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if BOARD_NGPIOIN > 0

/****************************************************************************
 * Name: gpin_read
 *
 * Description:
 *   Read the current logic state of an input GPIO pin and store the result
 *   in the caller-provided boolean output value.
 ****************************************************************************/

static int gpin_read(struct gpio_dev_s *dev, bool *value)
{
  struct rzv2hgpio_dev_s *rzv2hgpio = (struct rzv2hgpio_dev_s *)dev;

  DEBUGASSERT(rzv2hgpio != NULL && value != NULL);
  DEBUGASSERT(rzv2hgpio->id < BOARD_NGPIOIN);
  gpioinfo("Reading...");

  *value = rzv2h_gpioread(g_gpioinputs[rzv2hgpio->id]);

  return OK;
}

/****************************************************************************
 * Name: gpin_attach
 *
 * Description:
 *   Register an interrupt callback for a GPIO input pin. The callback is
 *   stored for later use when the interrupt is enabled.
 *   Refer to drivers/ioexpander/gpio.c for the reference implementation.
 ****************************************************************************/

static int gpin_attach(struct gpio_dev_s *dev, pin_interrupt_t callback)
{
  struct rzv2hgpint_dev_s *rzv2hgpint = (struct rzv2hgpint_dev_s *)dev;

  gpioinfo("Attaching the callback\n");

  /* Make sure the interrupt is disabled */

  /* Assign Callback function */

  gpioinfo("Attach %p\n", callback);
  rzv2hgpint->callback = callback;
  return OK;
}

/****************************************************************************
 * Name: gpin_enable
 *
 * Description:
 *   Enable or disable the interrupt handling for an input GPIO pin.
 *   The current implementation only tracks the requested state.
 ****************************************************************************/

static int gpin_enable(struct gpio_dev_s *dev, bool enable)
{
  struct rzv2hgpint_dev_s *rzv2hgpint = (struct rzv2hgpint_dev_s *)dev;

  if (enable && rzv2hgpint->callback != NULL)
    {
      gpioinfo("Enabling the interrupt\n");
    }
  else
    {
      gpioinfo("Disable the interrupt\n");
    }

  return OK;
}

#endif

#if BOARD_NGPIOOUT > 0

/****************************************************************************
 * Name: gpout_read
 *
 * Description:
 *   Read the current logic state of an output GPIO pin and return it through
 *   the caller-provided boolean output value.
 ****************************************************************************/

static int gpout_read(struct gpio_dev_s *dev, bool *value)
{
  struct rzv2hgpio_dev_s *rzv2hgpio = (struct rzv2hgpio_dev_s *)dev;

  DEBUGASSERT(rzv2hgpio != NULL && value != NULL);
  DEBUGASSERT(rzv2hgpio->id < BOARD_NGPIOOUT);
  gpioinfo("Reading...");

  *value = rzv2h_gpioread(g_gpiooutputs[rzv2hgpio->id]);

  return OK;
}

/****************************************************************************
 * Name: gpout_write
 *
 * Description:
 *   Drive an output GPIO pin to the requested logic state.
 ****************************************************************************/

static int gpout_write(struct gpio_dev_s *dev, bool value)
{
  struct rzv2hgpio_dev_s *rzv2hgpio = (struct rzv2hgpio_dev_s *)dev;

  DEBUGASSERT(rzv2hgpio != NULL);
  DEBUGASSERT(rzv2hgpio->id < BOARD_NGPIOOUT);
  gpioinfo("Writing %d", (int)value);

  return rzv2h_gpiowrite(g_gpiooutputs[rzv2hgpio->id], value);
}

#endif

/****************************************************************************
 * Name: gpio_setpintype
 *
 * Description:
 *   Set the pin type for a GPIO device. This implementation is currently
 *   unsupported and returns success without changing hardware state.
 ****************************************************************************/

static int gpio_setpintype(struct gpio_dev_s *dev,
                           enum gpio_pintype_e gpio_pintype)
{
  struct rzv2hgpio_dev_s *rzv2hgpio = (struct rzv2hgpio_dev_s *)dev;

  UNUSED(rzv2hgpio);
  UNUSED(gpio_pintype);

  gpioinfo("setpintype is not supported. \n");

  return 0;
}

/****************************************************************************
 * Name: rzv2h_gpio_initialize
 *
 * Description:
 *   Initialize GPIO drivers for use with /apps/examples/gpio
 ****************************************************************************/

int rzv2h_gpio_initialize(void)
{
  int   pincount = 0;
  int   i;
  int   ret = 0;

#if BOARD_NGPIOIN > 0
  for (i = 0; i < BOARD_NGPIOIN; i++, pincount++)
    {
      /* Setup and register the GPIO pin */

      g_gpin[i].gpio.gp_pintype = GPIO_INPUT_PIN;
      g_gpin[i].gpio.gp_ops     = &gpin_ops;
      g_gpin[i].id              = i;
      ret                       =
        gpio_pin_register(&g_gpin[i].gpio, pincount);
      if (ret < 0)
        {
          gpioerr("GPIOIN(%d): gpio_pin_register failed: %d", i, ret);
          return ret;
        }

      /* Configure the pin that will be used as input */

      ret = rzv2h_configgpio(g_gpioinputs[i]);
      if (ret < 0)
        {
          gpioerr("GPIOIN(%d): rzv2h_configgpio failed: %d", i, ret);
          return ret;
        }
    }
#endif

#if BOARD_NGPIOOUT > 0
  for (i = 0; i < BOARD_NGPIOOUT; i++, pincount++)
    {
      /* Setup and register the GPIO pin */

      g_gpout[i].gpio.gp_pintype    = GPIO_OUTPUT_PIN;
      g_gpout[i].gpio.gp_ops        = &gpout_ops;
      g_gpout[i].id                 = i;
      ret                           = gpio_pin_register(&g_gpout[i].gpio,
                                                        pincount);
      if (ret < 0)
        {
          gpioerr("GPIOOUT(%d): gpio_pin_register failed: %d", i, ret);
          return ret;
        }

      /* Configure the pin that will be used as output */

      ret = rzv2h_configgpio(g_gpiooutputs[i]);
      if (ret < 0)
        {
          gpioerr("GPIOOUT(%d): rzv2h_configgpio failed: %d", i, ret);
          return ret;
        }
    }
#endif

  return ret;
}

#endif  /* CONFIG_RZV2H_GPIO */
