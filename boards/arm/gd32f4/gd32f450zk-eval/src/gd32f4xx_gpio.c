/****************************************************************************
 * boards/arm/gd32f4/gd32f450zk-eval/src/gd32f4xx_gpio.c
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

#include "gd32f4xx.h"
#include "gd32f450z_eval.h"

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

irqstate_t flags;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int gpio_setpintype(struct gpio_dev_s *dev,
                           enum gpio_pintype_e gp_pintype);

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
  .go_setpintype = gpio_setpintype,
};

/* This array maps the GPIO pins used as INPUT */

static const uint32_t g_gpioinputs[BOARD_NGPIOIN] =
{
  GPIO_IN1,
};

static struct gd32gpio_dev_s g_gpin[BOARD_NGPIOIN];
#endif

#if BOARD_NGPIOOUT > 0

static const struct gpio_operations_s gpout_ops =
{
  .go_read   = gpout_read,
  .go_write  = gpout_write,
  .go_attach = NULL,
  .go_enable = NULL,
  .go_setpintype = gpio_setpintype,
};

/* This array maps the GPIO pins used as OUTPUT */

static const uint32_t g_gpiooutputs[BOARD_NGPIOOUT] =
{
  GPIO_OUT1,
};

static struct gd32gpio_dev_s g_gpout[BOARD_NGPIOOUT];
#endif

#if BOARD_NGPIOINT > 0

static const struct gpio_operations_s gpint_ops =
{
  .go_read   = gpint_read,
  .go_write  = NULL,
  .go_attach = gpint_attach,
  .go_enable = gpint_enable,
  .go_setpintype = gpio_setpintype,
};

/* This array maps the GPIO pins used as INTERRUPT INPUTS */

static const uint32_t g_gpiointinputs[BOARD_NGPIOINT] =
{
  GPIO_INT1,
};

static struct gd32gpint_dev_s g_gpint[BOARD_NGPIOINT];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gpio_setpintype
 *
 * Description:
 *   set gpio pintype.
 *
 ****************************************************************************/

static int gpio_setpintype(struct gpio_dev_s *dev,
                           enum gpio_pintype_e gpio_pintype)
{
  struct gd32gpint_dev_s *gd32gpint = (struct gd32gpint_dev_s *)dev;

  UNUSED(gd32gpint);
  UNUSED(gpio_pintype);

  gpioinfo("setpintype is not supported. \n");

  return 0;
}

#if BOARD_NGPIOIN > 0
static int gpin_read(struct gpio_dev_s *dev, bool *value)
{
  struct gd32gpio_dev_s *gd32gpio = (struct gd32gpio_dev_s *)dev;

  DEBUGASSERT(gd32gpio != NULL && value != NULL);
  DEBUGASSERT(gd32gpio->id < BOARD_NGPIOIN);
  gpioinfo("Reading...\n");

  *value = gd32_gpio_read(g_gpioinputs[gd32gpio->id]);
  return OK;
}
#endif

#if BOARD_NGPIOOUT > 0
static int gpout_read(struct gpio_dev_s *dev, bool *value)
{
  struct gd32gpio_dev_s *gd32gpio = (struct gd32gpio_dev_s *)dev;

  DEBUGASSERT(gd32gpio != NULL && value != NULL);
  DEBUGASSERT(gd32gpio->id < BOARD_NGPIOOUT);
  gpioinfo("Reading...\n");

  *value = gd32_gpio_read(g_gpiooutputs[gd32gpio->id]);
  return OK;
}

static int gpout_write(struct gpio_dev_s *dev, bool value)
{
  struct gd32gpio_dev_s *gd32gpio = (struct gd32gpio_dev_s *)dev;

  DEBUGASSERT(gd32gpio != NULL);
  DEBUGASSERT(gd32gpio->id < BOARD_NGPIOOUT);
  gpioinfo("Writing %d\n", (int)value);

  gd32_gpio_write(g_gpiooutputs[gd32gpio->id], value);
  return OK;
}
#endif

#if BOARD_NGPIOINT > 0

static int gd32gpio_interrupt(int irq, void *context, void *arg)
{
  struct gd32gpint_dev_s *gd32gpint = (struct gd32gpint_dev_s *)arg;

  DEBUGASSERT(gd32gpint != NULL && gd32gpint->callback != NULL);
  gpioinfo("Interrupt! callback=%p\n", gd32gpint->callback);

  gd32gpint->callback(&gd32gpint->gd32gpio.gpio, gd32gpint->gd32gpio.id);
  return OK;
}

static int gpint_read(struct gpio_dev_s *dev, bool *value)
{
  struct gd32gpint_dev_s *gd32gpint = (struct gd32gpint_dev_s *)dev;

  DEBUGASSERT(gd32gpint != NULL && value != NULL);
  DEBUGASSERT(gd32gpint->gd32gpio.id < BOARD_NGPIOINT);
  gpioinfo("Reading int pin...\n");

  *value = gd32_gpio_read(g_gpiointinputs[gd32gpint->gd32gpio.id]);
  return OK;
}

static int gpint_attach(struct gpio_dev_s *dev, pin_interrupt_t callback)
{
  struct gd32gpint_dev_s *gd32gpint = (struct gd32gpint_dev_s *)dev;
  int ret;
  uint8_t gpio_irq;
  uint8_t gpio_irqnum;

  gpioinfo("Attaching the callback\n");

  flags = enter_critical_section();

  /* Make sure the interrupt is disabled */

  ret = gd32_exti_gpioirq_init(g_gpiointinputs[gd32gpint->gd32gpio.id],
                               EXTI_INTERRUPT, EXTI_TRIG_RISING, &gpio_irq);

  if (ret < 0)
    {
      leave_critical_section(flags);
      return ret;
    }

  gd32_gpio_exti_irqnum_get(g_gpiointinputs[gd32gpint->gd32gpio.id],
                            &gpio_irqnum);

  /* Attach and disable the interrupt */

  gd32_exti_gpio_irq_attach(gpio_irq, gd32gpio_interrupt,
                            &g_gpint[gd32gpint->gd32gpio.id]);

  /* Disable and the interrupt */

  up_disable_irq(gpio_irqnum);

  leave_critical_section(flags);
  gpioinfo("Attach %p\n", callback);
  gd32gpint->callback = callback;
  return OK;
}

static int gpint_enable(struct gpio_dev_s *dev, bool enable)
{
  struct gd32gpint_dev_s *gd32gpint = (struct gd32gpint_dev_s *)dev;
  int ret;
  uint8_t gpio_irqnum;

  flags = enter_critical_section();

  /* Get gpio irq numbers */

  ret = gd32_gpio_exti_irqnum_get(g_gpiointinputs[gd32gpint->gd32gpio.id],
                                  &gpio_irqnum);
  if (ret < 0)
    {
      leave_critical_section(flags);
      return ret;
    }

  if (enable)
    {
      if (gd32gpint->callback != NULL)
        {
          gpioinfo("Enabling the interrupt\n");

          /* Enable the interrupt */

          up_disable_irq(gpio_irqnum);
        }
    }
  else
    {
      up_disable_irq(gpio_irqnum);
      gpioinfo("Disable the interrupt\n");
    }

  leave_critical_section(flags);
  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_gpio_initialize
 *
 * Description:
 *   Initialize GPIO drivers for use with /apps/examples/gpio
 *
 ****************************************************************************/

int gd32_gpio_initialize(void)
{
  int pincount = 0;
  int i;

#if BOARD_NGPIOIN > 0
  for (i = 0; i < BOARD_NGPIOIN; i++)
    {
      /* Setup and register the GPIO pin */

      g_gpin[i].gpio.gp_pintype = GPIO_INPUT_PIN;
      g_gpin[i].gpio.gp_ops     = &gpin_ops;
      g_gpin[i].id              = i;

      gpio_pin_register(&g_gpin[i].gpio, pincount);

      /* Configure the pin that will be used as input */

      gd32_gpio_config(g_gpioinputs[i]);

      pincount++;
    }
#endif

#if BOARD_NGPIOOUT > 0
  for (i = 0; i < BOARD_NGPIOOUT; i++)
    {
      /* Setup and register the GPIO pin */

      g_gpout[i].gpio.gp_pintype = GPIO_OUTPUT_PIN;
      g_gpout[i].gpio.gp_ops     = &gpout_ops;
      g_gpout[i].id              = i;

      gpio_pin_register(&g_gpout[i].gpio, pincount);

      /* Configure the pin that will be used as output */

      gd32_gpio_write(g_gpiooutputs[i], 0);
      gd32_gpio_config(g_gpiooutputs[i]);

      pincount++;
    }
#endif

#if BOARD_NGPIOINT > 0
  for (i = 0; i < BOARD_NGPIOINT; i++)
    {
      /* Setup and register the GPIO pin */

      g_gpint[i].gd32gpio.gpio.gp_pintype = GPIO_INTERRUPT_PIN;
      g_gpint[i].gd32gpio.gpio.gp_ops     = &gpint_ops;
      g_gpint[i].gd32gpio.id              = i;
      (void)gpio_pin_register(&g_gpint[i].gd32gpio.gpio, pincount);

      /* Configure the pin that will be used as interrupt input */

      gd32_gpio_config(g_gpiointinputs[i]);

      pincount++;
    }
#endif

  return 0;
}
#endif /* CONFIG_DEV_GPIO && !CONFIG_GPIO_LOWER_HALF */
