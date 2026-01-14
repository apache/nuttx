/****************************************************************************
 * boards/arm64/bcm2711/raspberrypi-4b/src/rpi4b_gpio.c
 *
 * Author: Matteo Golin <matteo.golin@gmail.com>
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

#include <arch/irq.h>
#include <assert.h>
#include <debug.h>
#include <nuttx/irq.h>
#include <sys/types.h>
#include <syslog.h>

#include <nuttx/ioexpander/gpio.h>

#include <arch/board/board.h>

#include "bcm2711_gpio.h"
#include "chip.h"

#if defined(CONFIG_DEV_GPIO) && !defined(CONFIG_GPIO_LOWER_HALF)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Input pins */

#define GPIO_IN1 16

/* Output pins */

#define GPIO_OUT1 26

/* Interrupt pins */

/* TODO: why can't you select interrupt event type??? */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* GPIO device on the BCM2711 */

struct bcm2711_gpio_dev_s
{
  struct gpio_dev_s gpio; /* Underlying GPIO device */
  uint8_t id;             /* The index of the pin in its list. */
};

/* GPIO device with interrupt capabilities on the BCM2711 */

struct bcm2711_gpioint_dev_s
{
  struct bcm2711_gpio_dev_s bcm2711_gpio; /* BCM2711 GPIO device */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#if BOARD_NGPIOOUT > 0
static int gpout_read(struct gpio_dev_s *dev, bool *value);
static int gpout_write(struct gpio_dev_s *dev, bool value);
#endif

#if BOARD_NGPIOIN > 0
static int gpin_read(struct gpio_dev_s *dev, bool *value);
#endif

#if BOARD_NGPIOINT > 0
static int gpint_read(struct gpio_dev_s *dev, bool *value);
static int gpint_attach(struct gpio_dev_s *dev, pin_interrupt_t callback);
static int gpint_enable(struct gpio_dev_s *dev, bool enable);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if BOARD_NGPIOOUT > 0

/* GPIO operations for output pins. */

static const struct gpio_operations_s gpout_ops =
{
    .go_read = gpout_read,
    .go_write = gpout_write,
    .go_attach = NULL,
    .go_enable = NULL,
};

/* This array maps the GPIO pins used as OUTPUT */

static const uint32_t g_gpiooutputs[BOARD_NGPIOOUT] =
{
    GPIO_OUT1,
};

/* GPIO output pin devices */

static struct bcm2711_gpio_dev_s g_gpout[BOARD_NGPIOOUT];

#endif /* BOARD_NGPIOOUT > 0 */

#if BOARD_NGPIOIN > 0

/* GPIO operations for input pins. */

static const struct gpio_operations_s gpin_ops =
{
    .go_read = gpin_read,
    .go_write = NULL,
    .go_attach = NULL,
    .go_enable = NULL,
};

/* This array maps the GPIO pins used as INTERRUPT INPUTS */

static const uint32_t g_gpioinputs[BOARD_NGPIOIN] =
{
    GPIO_IN1,
};

/* GPIO input pin devices */

static struct bcm2711_gpio_dev_s g_gpin[BOARD_NGPIOIN];

#endif /* BOARD_NGPIOIN > 0 */

#if BOARD_NGPIOINT > 0

#warn "Missing functionality for interrupt GPIO pins."

/* GPIO operations for interrupt pins. */

static const struct gpio_operations_s gpint_ops =
{
    .go_read = gpint_read,
    .go_write = NULL,
    .go_attach = gpint_attach,
    .go_enable = gpint_enable,
};

/* This array maps the GPIO pins used as INTERRUPT INPUTS */

static const uint32_t g_gpiointinputs[BOARD_NGPIOINT] =
{
    GPIO_IRQPIN1,
};

/* GPIO interrupt pin devices */

static struct bcm2711_gpioint_dev_s g_gpint[BOARD_NGPIOINT];

#endif /* BOARD_NGPIOINT > 0 */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if BOARD_NGPIOOUT > 0

/****************************************************************************
 * Name: gpout_read
 *
 * Description:
 *     Read the output pin's current status (0 or 1).
 *
 * Input parameters:
 *     dev - The GPIO device structure.
 *     value - A pointer to the location to store the pin status.
 ****************************************************************************/

static int gpout_read(struct gpio_dev_s *dev, bool *value)
{
  struct bcm2711_gpio_dev_s *bcm2711gpio =
        (struct bcm2711_gpio_dev_s *)(dev);

  DEBUGASSERT(bcm2711gpio != NULL);
  DEBUGASSERT(value != NULL);
  DEBUGASSERT(bcm2711gpio->id < BOARD_NGPIOOUT);

  *value = bcm2711_gpio_pin_get(g_gpiooutputs[bcm2711gpio->id]);

  return 0;
}

/****************************************************************************
 * Name: gpout_write
 *
 * Description:
 *     Write a value to a GPIO output pin.
 *
 * Input parameters:
 *     dev - The GPIO device struct of the pin to write to.
 *     value - The value to write to the pin.
 ****************************************************************************/

static int gpout_write(struct gpio_dev_s *dev, bool value)
{
  struct bcm2711_gpio_dev_s *bcm2711gpio =
        (struct bcm2711_gpio_dev_s *)(dev);

  DEBUGASSERT(bcm2711gpio != NULL);
  DEBUGASSERT(bcm2711gpio->id < BOARD_NGPIOOUT);

  gpioinfo("Writing %u to pin %u\n", value, g_gpiooutputs[bcm2711gpio->id]);
  bcm2711_gpio_pin_set(g_gpiooutputs[bcm2711gpio->id], value);

  return 0;
}

#endif /* BOARD_NGPIOOUT > 0 */

#if BOARD_NGPIOIN > 0

/****************************************************************************
 * Name: gpin_read
 *
 * Description:
 *     Read the input pin's current status (0 or 1).
 *
 * Input parameters:
 *     dev - The GPIO device structure.
 *     value - A pointer to the location to store the pin status.
 ****************************************************************************/

static int gpin_read(struct gpio_dev_s *dev, bool *value)
{
  struct bcm2711_gpio_dev_s *bcm2711gpio =
        (struct bcm2711_gpio_dev_s *)(dev);

  DEBUGASSERT(bcm2711gpio != NULL);
  DEBUGASSERT(value != NULL);
  DEBUGASSERT(bcm2711gpio->id < BOARD_NGPIOIN);

  *value = bcm2711_gpio_pin_get(g_gpioinputs[bcm2711gpio->id]);

  return 0;
}

#endif /* BOARD_NGPIOIN > 0 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bcm2711_dev_gpio_init
 ****************************************************************************/

int bcm2711_dev_gpio_init(void)
{
  int i;
  int ret = OK;

  /* Register output pins. */

#if BOARD_NGPIOOUT > 0
  for (i = 0; i < BOARD_NGPIOOUT; i++)
    {
      /* Setup and register the GPIO pin */

      g_gpout[i].gpio.gp_pintype = GPIO_OUTPUT_PIN;
      g_gpout[i].gpio.gp_ops = &gpout_ops;
      g_gpout[i].id = i;
      ret = gpio_pin_register(&g_gpout[i].gpio, g_gpiooutputs[i]);
      if (ret < 0)
        {
          gpioerr("Failed to register output pin %d (BCM2711 #%u): %d\n", i,
                  g_gpiooutputs[i], ret);
          return ret;
        }

      /* Configure the pins that will be used as output.
       * They will start low and have no pull-up or pull-down resistors.
       */

      bcm2711_gpio_set_pulls(g_gpiooutputs[i], false, false);
      bcm2711_gpio_set_func(g_gpiooutputs[i], BCM_GPIO_OUTPUT);
      bcm2711_gpio_pin_set(g_gpiooutputs[i], false);
    }
#endif

  /* Register input pins. */

#if BOARD_NGPIOIN > 0
  for (i = 0; i < BOARD_NGPIOIN; i++)
    {
      /* Setup and register the GPIO pin */

      g_gpin[i].gpio.gp_pintype = GPIO_INPUT_PIN;
      g_gpin[i].gpio.gp_ops = &gpin_ops;
      g_gpin[i].id = i;
      ret = gpio_pin_register(&g_gpin[i].gpio, g_gpioinputs[i]);
      if (ret < 0)
        {
          gpioerr("Failed to register input pin %d (BCM2711 #%u): %d\n", i,
                  g_gpioinputs[i], ret);
          return ret;
        }

      /* Configure the pins that will be used as INPUT.
       * They will have pull-up resistors.
       */

      bcm2711_gpio_set_func(g_gpioinputs[i], BCM_GPIO_INPUT);
      bcm2711_gpio_set_pulls(g_gpioinputs[i], true, false);

      /* TODO: pull-up or pull-down should be configurable per pin */
    }
#endif

  return OK;
}

#endif /* defined(CONFIG_DEV_GPIO) && !defined(CONFIG_GPIO_LOWER_HALF) */
