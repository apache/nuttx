/****************************************************************************
 * boards/arm/stm32f7/nucleo-144/src/stm32_gpio.c
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
#include "stm32_gpio.h"
#include "nucleo-144.h"

#if defined(CONFIG_DEV_GPIO) && !defined(CONFIG_GPIO_LOWER_HALF)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32gpio_dev_s
{
  struct gpio_dev_s gpio;
  uint8_t id;
};

struct stm32gpint_dev_s
{
  struct stm32gpio_dev_s stm32gpio;
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

static const struct gpio_operations_s gpint_ops =
{
  .go_read   = gpint_read,
  .go_write  = NULL,
  .go_attach = gpint_attach,
  .go_enable = gpint_enable,
};

#if BOARD_NGPIOIN > 0
/* This array maps the GPIO pins used as INPUT */

static const uint32_t g_gpioinputs[BOARD_NGPIOIN] =
{
  GPIO_IN1,
  GPIO_IN2,
  GPIO_IN3,
  GPIO_IN4,
};

static struct stm32gpio_dev_s g_gpin[BOARD_NGPIOIN];
#endif

#if BOARD_NGPIOOUT
/* This array maps the GPIO pins used as OUTPUT */

static const uint32_t g_gpiooutputs[BOARD_NGPIOOUT] =
{
  GPIO_LD1,
  GPIO_LD2,
  GPIO_LD3,
  GPIO_OUT1,
  GPIO_OUT2,
  GPIO_OUT3,
  GPIO_OUT4,
  GPIO_OUT5,
#if !defined(CONFIG_STM32F7_TIM1_CH1NOUT)
  GPIO_OUT6,
#endif
#if !defined(CONFIG_STM32F7_TIM1_CH2NOUT)  
  GPIO_OUT7,
#endif
};

static struct stm32gpio_dev_s g_gpout[BOARD_NGPIOOUT];
#endif

#if BOARD_NGPIOINT > 0
/* This array maps the GPIO pins used as INTERRUPT INPUTS */

static const uint32_t g_gpiointinputs[BOARD_NGPIOINT] =
{
  GPIO_INT1,
};

static struct stm32gpint_dev_s g_gpint[BOARD_NGPIOINT];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int stm32gpio_interrupt(int irq, void *context, void *arg)
{
  struct stm32gpint_dev_s *stm32gpint =
    (struct stm32gpint_dev_s *)arg;

  DEBUGASSERT(stm32gpint != NULL && stm32gpint->callback != NULL);
  gpioinfo("Interrupt! callback=%p\n", stm32gpint->callback);

  stm32gpint->callback(&stm32gpint->stm32gpio.gpio,
                       stm32gpint->stm32gpio.id);
  return OK;
}

static int gpin_read(struct gpio_dev_s *dev, bool *value)
{
  struct stm32gpio_dev_s *stm32gpio =
    (struct stm32gpio_dev_s *)dev;

  DEBUGASSERT(stm32gpio != NULL && value != NULL);
  DEBUGASSERT(stm32gpio->id < BOARD_NGPIOIN);
  gpioinfo("Reading...\n");

  *value = stm32_gpioread(g_gpioinputs[stm32gpio->id]);
  return OK;
}

static int gpout_read(struct gpio_dev_s *dev, bool *value)
{
  struct stm32gpio_dev_s *stm32gpio =
    (struct stm32gpio_dev_s *)dev;

  DEBUGASSERT(stm32gpio != NULL && value != NULL);
  DEBUGASSERT(stm32gpio->id < BOARD_NGPIOOUT);
  gpioinfo("Reading...\n");

  *value = stm32_gpioread(g_gpiooutputs[stm32gpio->id]);
  return OK;
}

static int gpout_write(struct gpio_dev_s *dev, bool value)
{
  struct stm32gpio_dev_s *stm32gpio =
    (struct stm32gpio_dev_s *)dev;

  DEBUGASSERT(stm32gpio != NULL);
  DEBUGASSERT(stm32gpio->id < BOARD_NGPIOOUT);
  gpioinfo("Writing %d\n", (int)value);

  stm32_gpiowrite(g_gpiooutputs[stm32gpio->id], value);
  return OK;
}

static int gpint_read(struct gpio_dev_s *dev, bool *value)
{
  struct stm32gpint_dev_s *stm32gpint =
    (struct stm32gpint_dev_s *)dev;

  DEBUGASSERT(stm32gpint != NULL && value != NULL);
  DEBUGASSERT(stm32gpint->stm32gpio.id < BOARD_NGPIOINT);
  gpioinfo("Reading int pin...\n");

  *value = stm32_gpioread(g_gpiointinputs[stm32gpint->stm32gpio.id]);
  return OK;
}

static int gpint_attach(struct gpio_dev_s *dev,
                        pin_interrupt_t callback)
{
  struct stm32gpint_dev_s *stm32gpint =
    (struct stm32gpint_dev_s *)dev;

  gpioinfo("Attaching the callback\n");

  /* Make sure the interrupt is disabled */

  stm32_gpiosetevent(g_gpiointinputs[stm32gpint->stm32gpio.id], false,
                     false, false, NULL, NULL);

  gpioinfo("Attach %p\n", callback);
  stm32gpint->callback = callback;
  return OK;
}

static int gpint_enable(struct gpio_dev_s *dev, bool enable)
{
  struct stm32gpint_dev_s *stm32gpint =
    (struct stm32gpint_dev_s *)dev;

  if (enable)
    {
      if (stm32gpint->callback != NULL)
        {
          gpioinfo("Enabling the interrupt\n");

          /* Configure the interrupt for rising edge */

          stm32_gpiosetevent(g_gpiointinputs[stm32gpint->stm32gpio.id],
                             true, false, false, stm32gpio_interrupt,
                             &g_gpint[stm32gpint->stm32gpio.id]);
        }
    }
  else
    {
      gpioinfo("Disable the interrupt\n");
      stm32_gpiosetevent(g_gpiointinputs[stm32gpint->stm32gpio.id],
                         false, false, false, NULL, NULL);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_gpio_initialize
 *
 * Description:
 *   Initialize GPIO drivers for use with /apps/examples/gpio
 *
 ****************************************************************************/

int stm32_gpio_initialize(void)
{
  int i;
  int pincount = 0;

#if BOARD_NGPIOIN > 0
  for (i = 0; i < BOARD_NGPIOIN; i++)
    {
      /* Setup and register the GPIO pin */

      g_gpin[i].gpio.gp_pintype = GPIO_INPUT_PIN;
      g_gpin[i].gpio.gp_ops     = &gpin_ops;
      g_gpin[i].id              = i;
      gpio_pin_register(&g_gpin[i].gpio, pincount);

      /* Configure the pin that will be used as input */

      stm32_configgpio(g_gpioinputs[i]);

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

      stm32_gpiowrite(g_gpiooutputs[i], 0);
      stm32_configgpio(g_gpiooutputs[i]);

      pincount++;
    }
#endif

#if BOARD_NGPIOINT > 0
  for (i = 0; i < BOARD_NGPIOINT; i++)
    {
      /* Setup and register the GPIO pin */

      g_gpint[i].stm32gpio.gpio.gp_pintype = GPIO_INTERRUPT_PIN;
      g_gpint[i].stm32gpio.gpio.gp_ops     = &gpint_ops;
      g_gpint[i].stm32gpio.id              = i;
      gpio_pin_register(&g_gpint[i].stm32gpio.gpio, pincount);

      /* Configure the pin that will be used as interrupt input */

      stm32_configgpio(g_gpiointinputs[i]);

      pincount++;
    }
#endif

  return 0;
}
#endif /* CONFIG_DEV_GPIO && !CONFIG_GPIO_LOWER_HALF */
