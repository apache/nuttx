/****************************************************************************
 * boards/arm/tlsr82/tlsr8278adk80d/src/tlsr8278_gpio.c
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
#include <errno.h>
#include <debug.h>

#include <nuttx/clock.h>
#include <nuttx/wdog.h>
#include <nuttx/ioexpander/gpio.h>

#include <arch/board/board.h>

#include "chip.h"
#include "tlsr82_gpio.h"
#include "tlsr8278adk80d.h"

#if defined(CONFIG_DEV_GPIO) && !defined(CONFIG_GPIO_LOWER_HALF)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct tlsr82gpio_dev_s
{
  struct gpio_dev_s         gpio;         /* GPIO Device */
  pin_interrupt_t           callback;     /* Interrupt callback */
  uint8_t                   id;           /* ID */
  gpio_cfg_t                pinset;       /* The pin set */
  const enum gpio_pintype_e init_pintype; /* The pin type */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int tlsr82_go_read(struct gpio_dev_s *dev, bool *value);
static int tlsr82_go_write(struct gpio_dev_s *dev, bool value);
static int tlsr82_go_attach(struct gpio_dev_s *dev,
                            pin_interrupt_t callback);
static int tlsr82_go_enable(struct gpio_dev_s *dev, bool enable);
static int tlsr82_go_setpintype(struct gpio_dev_s *dev,
                                enum gpio_pintype_e pintype);
static int tlsr82_go_interrupt(int irq, void *context, void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct gpio_operations_s gpio_ops =
{
  .go_read       = tlsr82_go_read,
  .go_write      = tlsr82_go_write,
  .go_attach     = tlsr82_go_attach,
  .go_enable     = tlsr82_go_enable,
  .go_setpintype = tlsr82_go_setpintype,
};

static struct tlsr82gpio_dev_s g_gpdevs[BOARD_NGPIO] =
{
  {
    .pinset = GPIO_PIN_PD6,
    .init_pintype = GPIO_INPUT_PIN_PULLDOWN,
  },
  {
    .pinset = GPIO_PIN_PD0,
    .init_pintype = GPIO_OUTPUT_PIN,
  },
  {
    .pinset = GPIO_PIN_PD1,
    .init_pintype = GPIO_OUTPUT_PIN,
  },
  {
    .pinset = GPIO_PIN_PB3,
    .init_pintype = GPIO_INTERRUPT_FALLING_PIN,
  }
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int tlsr82_go_interrupt(int irq, void *context, void *arg)
{
  struct tlsr82gpio_dev_s *tlsr82gpio = (struct tlsr82gpio_dev_s *)arg;

  DEBUGASSERT(tlsr82gpio != NULL && tlsr82gpio->callback != NULL);
  gpioinfo("Interrupt! callback=%p\n", tlsr82gpio->callback);

  tlsr82gpio->callback(&tlsr82gpio->gpio, tlsr82gpio->id);
  return OK;
}

static int tlsr82_go_read(struct gpio_dev_s *dev, bool *value)
{
  struct tlsr82gpio_dev_s *tlsr82gpio = (struct tlsr82gpio_dev_s *)dev;

  DEBUGASSERT(tlsr82gpio != NULL && value != NULL);
  DEBUGASSERT(tlsr82gpio->id < BOARD_NGPIO);
  gpioinfo("Reading, pinset=0x%08lx\n", tlsr82gpio->pinset);

  *value = tlsr82_gpioread(tlsr82gpio->pinset);
  return OK;
}

static int tlsr82_go_write(struct gpio_dev_s *dev, bool value)
{
  struct tlsr82gpio_dev_s *tlsr82gpio = (struct tlsr82gpio_dev_s *)dev;

  DEBUGASSERT(tlsr82gpio != NULL);
  DEBUGASSERT(tlsr82gpio->id < BOARD_NGPIO);
  gpioinfo("Writing %d\n, pinset=0x%08lx\n", (int)value, tlsr82gpio->pinset);

  tlsr82_gpiowrite(tlsr82gpio->pinset, value);
  return OK;
}

static int tlsr82_go_attach(struct gpio_dev_s *dev,
                            pin_interrupt_t callback)
{
  struct tlsr82gpio_dev_s *tlsr82gpio = (struct tlsr82gpio_dev_s *)dev;

  gpioinfo("Attaching the callback\n");

  /* Make sure the interrupt is disabled */

  tlsr82_gpioirqconfig(tlsr82gpio->pinset, NULL, NULL);

  gpioinfo("Attach %p\n", callback);
  tlsr82gpio->callback = callback;
  return OK;
}

static int tlsr82_go_enable(struct gpio_dev_s *dev, bool enable)
{
  struct tlsr82gpio_dev_s *tlsr82gpio = (struct tlsr82gpio_dev_s *)dev;

  if (enable)
    {
      if (tlsr82gpio->callback != NULL)
        {
          gpioinfo("Enabling the interrupt\n");

          /* Configure the interrupt for rising edge */

          tlsr82_gpioirqconfig(tlsr82gpio->pinset, tlsr82_go_interrupt,
                               tlsr82gpio);
        }
      else
        {
          gpiowarn("GPIO interrupt callback is NULL\n");
        }
    }
  else
    {
      gpioinfo("Disable the interrupt\n");
      tlsr82_gpioirqconfig(tlsr82gpio->pinset, NULL, NULL);
    }

  return OK;
}

static int tlsr82_go_setpintype(struct gpio_dev_s *dev,
                                enum gpio_pintype_e pintype)
{
  int ret = OK;
  gpio_cfg_t cfg;
  gpio_cfg_t irq;
  struct tlsr82gpio_dev_s *tlsr82gpio = (struct tlsr82gpio_dev_s *)dev;

  cfg = GPIO_CFG2PIN(tlsr82gpio->pinset);
  irq = tlsr82gpio->pinset & GPIO_IRQ_MASK;

  switch (pintype)
    {
      case GPIO_INPUT_PIN:
        {
          cfg |= GPIO_AF_INPUT | GPIO_PUPD_NONE;
          tlsr82_gpioconfig(cfg);
        }
        break;

      case GPIO_INPUT_PIN_PULLUP:
        {
          cfg |= GPIO_AF_INPUT | GPIO_PUPD_PU10K;
          tlsr82_gpioconfig(cfg);
        }
        break;

      case GPIO_INPUT_PIN_PULLDOWN:
        {
          cfg |= GPIO_AF_INPUT | GPIO_PUPD_PD100K;
          tlsr82_gpioconfig(cfg);
        }
        break;

      case GPIO_OUTPUT_PIN:
        {
          cfg |= GPIO_AF_OUTPUT | GPIO_DS_HIGH;
          tlsr82_gpioconfig(cfg);
        }
        break;

      case GPIO_INTERRUPT_PIN:
      case GPIO_INTERRUPT_HIGH_PIN:
      case GPIO_INTERRUPT_RISING_PIN:
        {
          cfg |= GPIO_AF_INPUT | GPIO_PUPD_PD100K | GPIO_POL_RISE;
          if (irq == GPIO_IRQ_DISABLE)
            {
              /* If do not specify the interrupt type, default normal */

              cfg |= GPIO_IRQ_NORMAL;
            }
          else
            {
              cfg |= irq;
            }

          tlsr82_gpioirqconfig(cfg, NULL, NULL);
        }
        break;

      case GPIO_INTERRUPT_LOW_PIN:
      case GPIO_INTERRUPT_FALLING_PIN:
        {
          cfg |= GPIO_AF_INPUT | GPIO_PUPD_PU10K | GPIO_POL_FALL;
          if (irq == GPIO_IRQ_DISABLE)
            {
              /* If do not specify the interrupt type, default normal */

              cfg |= GPIO_IRQ_NORMAL;
            }
          else
            {
              cfg |= irq;
            }

          tlsr82_gpioirqconfig(cfg, NULL, NULL);
        }
        break;

      case GPIO_OUTPUT_PIN_OPENDRAIN:
      case GPIO_INTERRUPT_BOTH_PIN:
      default:
        {
          gpioerr("Not support pin type, pintype=%d\n", (int)pintype);
          ret = -ENOTSUP;
          goto errout;
        }
    }

  /* Assign back the config information */

  tlsr82gpio->pinset = cfg;

  gpioinfo("pinset=0x%08lx\n", cfg);

errout:
  return ret;
}

/****************************************************************************
 * Name: tlsr82_gpio_initialize
 *
 * Description:
 *   Initialize GPIO drivers for use with /apps/examples/gpio
 *
 ****************************************************************************/

int tlsr82_gpio_initialize(void)
{
  int i;
  int ret = OK;
  struct tlsr82gpio_dev_s *tlsr82gpio;

  tlsr82_gpioirqinitialize();

  for (i = 0; i < BOARD_NGPIO; i++)
    {
      tlsr82gpio = &g_gpdevs[i];
      tlsr82gpio->gpio.gp_pintype = tlsr82gpio->init_pintype;
      tlsr82gpio->gpio.gp_ops     = &gpio_ops;
      tlsr82gpio->id              = i;

      ret = tlsr82_go_setpintype(&tlsr82gpio->gpio,
                                 tlsr82gpio->init_pintype);
      if (ret < 0)
        {
          goto out;
        }

      gpio_pin_register(&tlsr82gpio->gpio, i);
    }

out:
  return ret;
}

#endif /* CONFIG_DEV_GPIO && !CONFIG_GPIO_LOWER_HALF */
