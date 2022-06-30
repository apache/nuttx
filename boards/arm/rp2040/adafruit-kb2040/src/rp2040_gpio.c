/****************************************************************************
 * boards/arm/rp2040/adafruit-kb2040/src/rp2040_gpio.c
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

#include <sys/types.h>
#include <syslog.h>
#include <nuttx/irq.h>
#include <arch/irq.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/ioexpander/gpio.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "rp2040_gpio.h"

#if defined(CONFIG_DEV_GPIO) && !defined(CONFIG_GPIO_LOWER_HALF)

/* Output pins. GPIO25 is onboard LED any other outputs could be used.
 */

#define GPIO_OUT1     25

/* Input pins.
 */

#define GPIO_IN1      6

/* Interrupt pins.
 */

#define GPIO_IRQPIN1  11

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rp2040gpio_dev_s
{
  struct gpio_dev_s gpio;
  uint8_t id;
};

struct rp2040gpint_dev_s
{
  struct rp2040gpio_dev_s rp2040gpio;
  pin_interrupt_t callback;
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
static int gpint_attach(struct gpio_dev_s *dev,
                        pin_interrupt_t callback);
static int gpint_enable(struct gpio_dev_s *dev, bool enable);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if BOARD_NGPIOOUT > 0
static const struct gpio_operations_s gpout_ops =
{
  .go_read   = gpout_read,
  .go_write  = gpout_write,
  .go_attach = NULL,
  .go_enable = NULL,
};

/* This array maps the GPIO pins used as OUTPUT */

static const uint32_t g_gpiooutputs[BOARD_NGPIOOUT] =
{
  GPIO_OUT1
};

static struct rp2040gpio_dev_s g_gpout[BOARD_NGPIOOUT];
#endif

#if BOARD_NGPIOIN > 0
static const struct gpio_operations_s gpin_ops =
{
  .go_read   = gpin_read,
  .go_write  = NULL,
  .go_attach = NULL,
  .go_enable = NULL,
};

/* This array maps the GPIO pins used as INTERRUPT INPUTS */

static const uint32_t g_gpioinputs[BOARD_NGPIOIN] =
{
  GPIO_IN1
};

static struct rp2040gpio_dev_s g_gpin[BOARD_NGPIOIN];
#endif

#if BOARD_NGPIOINT > 0
static const struct gpio_operations_s gpint_ops =
{
  .go_read   = gpint_read,
  .go_write  = NULL,
  .go_attach = gpint_attach,
  .go_enable = gpint_enable,
};

/* This array maps the GPIO pins used as INTERRUPT INPUTS */

static const uint32_t g_gpiointinputs[BOARD_NGPIOINT] =
{
  GPIO_IRQPIN1,
};

static struct rp2040gpint_dev_s g_gpint[BOARD_NGPIOINT];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gpout_read
 ****************************************************************************/

#if BOARD_NGPIOOUT > 0
static int gpout_read(struct gpio_dev_s *dev, bool *value)
{
  struct rp2040gpio_dev_s *rp2040gpio =
    (struct rp2040gpio_dev_s *)dev;

  DEBUGASSERT(rp2040gpio != NULL && value != NULL);
  DEBUGASSERT(rp2040gpio->id < BOARD_NGPIOOUT);
  gpioinfo("Reading...\n");

  *value = rp2040_gpio_get(g_gpiooutputs[rp2040gpio->id]);
  return OK;
}

/****************************************************************************
 * Name: gpout_write
 ****************************************************************************/

static int gpout_write(struct gpio_dev_s *dev, bool value)
{
  struct rp2040gpio_dev_s *rp2040gpio =
    (struct rp2040gpio_dev_s *)dev;

  DEBUGASSERT(rp2040gpio != NULL);
  DEBUGASSERT(rp2040gpio->id < BOARD_NGPIOOUT);
  gpioinfo("Writing %d\n", (int)value);

  rp2040_gpio_put(g_gpiooutputs[rp2040gpio->id], value);
  return OK;
}
#endif

/****************************************************************************
 * Name: gpin_read
 ****************************************************************************/

#if BOARD_NGPIOIN > 0
static int gpin_read(struct gpio_dev_s *dev, bool *value)
{
  struct rp2040gpio_dev_s *rp2040gpio =
    (struct rp2040gpio_dev_s *)dev;

  DEBUGASSERT(rp2040gpio != NULL && value != NULL);
  DEBUGASSERT(rp2040gpio->id < BOARD_NGPIOIN);
  gpioinfo("Reading... pin %d\n", (int)g_gpioinputs[rp2040gpio->id]);

  *value = rp2040_gpio_get(g_gpioinputs[rp2040gpio->id]);
  return OK;
}
#endif

/****************************************************************************
 * Name: rp2040gpio_interrupt
 ****************************************************************************/

#if BOARD_NGPIOINT > 0
static int rp2040gpio_interrupt(int irq, void *context, void *arg)
{
  struct rp2040gpint_dev_s *rp2040gpint =
    (struct rp2040gpint_dev_s *)arg;

  DEBUGASSERT(rp2040gpint != NULL && rp2040gpint->callback != NULL);
  gpioinfo("Interrupt! callback=%p\n", rp2040gpint->callback);

  rp2040gpint->callback(&rp2040gpint->rp2040gpio.gpio,
                       rp2040gpint->rp2040gpio.id);
  return OK;
}

/****************************************************************************
 * Name: gpint_read
 ****************************************************************************/

static int gpint_read(struct gpio_dev_s *dev, bool *value)
{
  struct rp2040gpint_dev_s *rp2040gpint =
    (struct rp2040gpint_dev_s *)dev;

  DEBUGASSERT(rp2040gpint != NULL && value != NULL);
  DEBUGASSERT(rp2040gpint->rp2040gpio.id < BOARD_NGPIOINT);
  gpioinfo("Reading int pin...\n");

  *value = rp2040_gpio_get(g_gpiointinputs[rp2040gpint->rp2040gpio.id]);
  return OK;
}

/****************************************************************************
 * Name: gpint_attach
 ****************************************************************************/

static int gpint_attach(struct gpio_dev_s *dev,
                        pin_interrupt_t callback)
{
  struct rp2040gpint_dev_s *rp2040gpint =
    (struct rp2040gpint_dev_s *)dev;
  int irq = g_gpiointinputs[rp2040gpint->rp2040gpio.id];
  int ret;

  gpioinfo("Attaching the callback\n");

  /* Make sure the interrupt is disabled */

  rp2040_gpio_disable_irq(irq);
  ret = rp2040_gpio_irq_attach(irq,
                               RP2040_GPIO_INTR_EDGE_LOW,
                               rp2040gpio_interrupt,
                               &g_gpint[rp2040gpint->rp2040gpio.id]);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: gpint_attach() failed: %d\n", ret);
      return ret;
    }

  gpioinfo("Attach %p\n", callback);
  rp2040gpint->callback = callback;
  return OK;
}

/****************************************************************************
 * Name: gpint_enable
 ****************************************************************************/

static int gpint_enable(struct gpio_dev_s *dev, bool enable)
{
  struct rp2040gpint_dev_s *rp2040gpint =
    (struct rp2040gpint_dev_s *)dev;
  int irq = g_gpiointinputs[rp2040gpint->rp2040gpio.id];

  if (enable)
    {
      if (rp2040gpint->callback != NULL)
        {
          gpioinfo("Enabling the interrupt\n");

          /* Configure the interrupt for rising edge */

          rp2040_gpio_enable_irq(irq);
        }
    }
  else
    {
      gpioinfo("Disable the interrupt\n");
      rp2040_gpio_disable_irq(irq);
    }

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp2040_dev_gpio_init
 ****************************************************************************/

int rp2040_dev_gpio_init(void)
{
  int i;
  int pincount = 0;

#if BOARD_NGPIOOUT > 0
  for (i = 0; i < BOARD_NGPIOOUT; i++)
    {
      /* Setup and register the GPIO pin */

      g_gpout[i].gpio.gp_pintype = GPIO_OUTPUT_PIN;
      g_gpout[i].gpio.gp_ops     = &gpout_ops;
      g_gpout[i].id              = i;
      gpio_pin_register(&g_gpout[i].gpio, g_gpiooutputs[i]);

      /* Configure the pins that will be used as output */

      rp2040_gpio_init(g_gpiooutputs[i]);
      rp2040_gpio_setdir(g_gpiooutputs[i], true);
      rp2040_gpio_put(g_gpiooutputs[i], false);

      pincount++;
    }
#endif

  pincount = 0;

#if BOARD_NGPIOIN > 0
  for (i = 0; i < BOARD_NGPIOIN; i++)
    {
      /* Setup and register the GPIO pin */

      g_gpin[i].gpio.gp_pintype = GPIO_INPUT_PIN;
      g_gpin[i].gpio.gp_ops     = &gpin_ops;
      g_gpin[i].id              = i;
      gpio_pin_register(&g_gpin[i].gpio, g_gpioinputs[i]);

      /* Configure the pins that will be used as INPUT */

      rp2040_gpio_init(g_gpioinputs[i]);

      pincount++;
    }
#endif

  pincount = 0;

#if BOARD_NGPIOINT > 0
  for (i = 0; i < BOARD_NGPIOINT; i++)
    {
      /* Setup and register the GPIO pin */

      g_gpint[i].rp2040gpio.gpio.gp_pintype = GPIO_INTERRUPT_PIN;
      g_gpint[i].rp2040gpio.gpio.gp_ops     = &gpint_ops;
      g_gpint[i].rp2040gpio.id              = i;
      gpio_pin_register(&g_gpint[i].rp2040gpio.gpio, g_gpiointinputs[i]);

      /* Configure the pins that will be used as interrupt input */

      rp2040_gpio_init(g_gpiointinputs[i]);

      /* pull-up = false : pull-down = true */

      rp2040_gpio_set_pulls(g_gpiointinputs[i], false, true);

      pincount++;
    }
#endif

  return OK;
}
#endif /* CONFIG_DEV_GPIO && !CONFIG_GPIO_LOWER_HALF */
