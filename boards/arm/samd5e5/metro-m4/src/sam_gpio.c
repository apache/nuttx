/****************************************************************************
 * boards/arm/samd5e5/metro-m4/src/sam_gpio.c
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

#include <nuttx/irq.h>

#include "irq/irq.h"
#include "chip.h"
#include <arch/board/board.h>

#include "sam_config.h"
#include "metro-m4.h"
#include "sam_port.h"
#include "sam_eic.h"

#if defined(CONFIG_DEV_GPIO) && !defined(CONFIG_GPIO_LOWER_HALF)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct samgpio_dev_s
{
  struct gpio_dev_s gpio;
  uint8_t id;
};

struct samgpint_dev_s
{
  struct samgpio_dev_s samgpio;
  pin_interrupt_t callback;
};

irqstate_t flags;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#if BOARD_NGPIOIN > 0
static int gpin_read(struct gpio_dev_s *dev, bool *value);
#endif
static int gpout_read(struct gpio_dev_s *dev, bool *value);
static int gpout_write(struct gpio_dev_s *dev, bool value);

static int gpint_read(struct gpio_dev_s *dev, bool *value);
static int gpint_attach(struct gpio_dev_s *dev,
                        pin_interrupt_t callback);
static int gpint_enable(struct gpio_dev_s *dev, bool enable);

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

static const int g_extint[1] = /* SAM_IRQ_EXTINT0 offset! */
{
  4, /* PORT_D9 - PA20 - EXTINT 4 */
};

#if BOARD_NGPIOIN > 0
/* This array maps the GPIO pins used as INPUT */

static const uint32_t g_gpioinputs[BOARD_NGPIOIN] =
{
  PORT_D10,
};

static struct samgpio_dev_s g_gpin[BOARD_NGPIOIN];
#endif

#if BOARD_NGPIOOUT
/* This array maps the GPIO pins used as OUTPUT */

static const uint32_t g_gpiooutputs[BOARD_NGPIOOUT] =
{
  PORT_D8,
};

static struct samgpio_dev_s g_gpout[BOARD_NGPIOOUT];
#endif

#if BOARD_NGPIOINT > 0
/* This array maps the GPIO pins used as INTERRUPT INPUTS */

static const uint32_t g_gpiointinputs[BOARD_NGPIOINT] =
{
  PORT_D9
};

static struct samgpint_dev_s g_gpint[BOARD_NGPIOINT];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int samgpio_interrupt(int irq, void *context, void *arg)
{
  struct samgpint_dev_s *samgpint = (struct samgpint_dev_s *)arg;

  DEBUGASSERT(samgpint != NULL && samgpint->callback != NULL);
  gpioinfo("Interrupt! callback=%p\n", samgpint->callback);

  samgpint->callback(&samgpint->samgpio.gpio, samgpint->samgpio.id);
  return OK;
}
#if BOARD_NGPIOIN > 0

static int gpin_read(struct gpio_dev_s *dev, bool *value)
{
  struct samgpio_dev_s *samgpio = (struct samgpio_dev_s *)dev;

  DEBUGASSERT(samgpio != NULL && value != NULL);
  DEBUGASSERT(samgpio->id < BOARD_NGPIOIN);
  gpioinfo("Reading...\n");

  *value = sam_portread(g_gpioinputs[samgpio->id]);
  return OK;
}
#endif

static int gpout_read(struct gpio_dev_s *dev, bool *value)
{
  struct samgpio_dev_s *samgpio = (struct samgpio_dev_s *)dev;

  DEBUGASSERT(samgpio != NULL && value != NULL);
  DEBUGASSERT(samgpio->id < BOARD_NGPIOOUT);
  gpioinfo("Reading...\n");

  *value = sam_portread(g_gpiooutputs[samgpio->id]);
  return OK;
}

static int gpout_write(struct gpio_dev_s *dev, bool value)
{
  struct samgpio_dev_s *samgpio = (struct samgpio_dev_s *)dev;

  DEBUGASSERT(samgpio != NULL);
  DEBUGASSERT(samgpio->id < BOARD_NGPIOOUT);
  gpioinfo("Writing %d\n", (int)value);

  sam_portwrite(g_gpiooutputs[samgpio->id], value);
  return OK;
}

static int gpint_read(struct gpio_dev_s *dev, bool *value)
{
  struct samgpint_dev_s *samgpint = (struct samgpint_dev_s *)dev;

  DEBUGASSERT(samgpint != NULL && value != NULL);
  DEBUGASSERT(samgpint->samgpio.id < BOARD_NGPIOINT);
  gpioinfo("Reading int pin...\n");

  *value = sam_portread(g_gpiointinputs[samgpint->samgpio.id]);
  return OK;
}

static int gpint_attach(struct gpio_dev_s *dev,
                        pin_interrupt_t callback)
{
  struct samgpint_dev_s *samgpint = (struct samgpint_dev_s *)dev;

  gpioinfo("Attaching the callback\n");

  flags = enter_critical_section();

  /* Make sure the interrupt is disabled */

  int ret = irq_attach(SAM_IRQ_EXTINT0 + g_extint[samgpint->samgpio.id],
                      samgpio_interrupt, &g_gpint[samgpint->samgpio.id]);
  if (ret == OK)
    {
      /* Configure the interrupt edge sensitivity
       * in CONFIGn register of the EIC
       */

      sam_eic_configure(g_extint[samgpint->samgpio.id],
                        g_gpiointinputs[samgpint->samgpio.id]);
    }

  leave_critical_section(flags);
  gpioinfo("Attach %p\n", callback);
  samgpint->callback = callback;
  return OK;
}

static int gpint_enable(struct gpio_dev_s *dev, bool enable)
{
  struct samgpint_dev_s *samgpint = (struct samgpint_dev_s *)dev;
  flags = enter_critical_section();
  if (enable)
    {
      if (samgpint->callback != NULL)
        {
          gpioinfo("Enabling the interrupt\n");
          up_enable_irq(SAM_IRQ_EXTINT0 + g_extint[samgpint->samgpio.id]);
        }
    }
  else
    {
      up_disable_irq(SAM_IRQ_EXTINT0 + g_extint[samgpint->samgpio.id]);
      gpioinfo("Disable the interrupt\n");
    }

  leave_critical_section(flags);
  return OK;
}

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

      sam_portconfig(g_gpioinputs[i]);

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

      sam_portconfig(g_gpiooutputs[i]);
      pincount++;
    }
#endif

#if BOARD_NGPIOINT > 0
  for (i = 0; i < BOARD_NGPIOINT; i++)
    {
      /* Setup and register the GPIO pin */

      g_gpint[i].samgpio.gpio.gp_pintype = GPIO_INTERRUPT_PIN;
      g_gpint[i].samgpio.gpio.gp_ops     = &gpint_ops;
      g_gpint[i].samgpio.id              = i;
      gpio_pin_register(&g_gpint[i].samgpio.gpio, pincount);

      /* Configure the pin that will be used as interrupt input */

      sam_portconfig(g_gpiointinputs[i]);

      pincount++;
    }
#endif

  return 0;
}
#endif /* CONFIG_DEV_GPIO && !CONFIG_GPIO_LOWER_HALF */
