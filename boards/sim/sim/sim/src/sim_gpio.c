/****************************************************************************
 * boards/sim/sim/sim/src/sim_gpio.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author:  Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

#include "sim.h"

#if defined(CONFIG_EXAMPLES_GPIO) && !defined(CONFIG_GPIO_LOWER_HALF)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct simgpio_dev_s
{
  struct gpio_dev_s gpio;
  uint8_t id;
  bool value;
};

struct simgpint_dev_s
{
  struct simgpio_dev_s simgpio;
  struct wdog_s wdog;
  pin_interrupt_t callback;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int gpin_read(FAR struct gpio_dev_s *dev, FAR bool *value);
static int gpout_write(FAR struct gpio_dev_s *dev, bool value);
static int gpint_attach(FAR struct gpio_dev_s *dev,
                        pin_interrupt_t callback);
static int gpint_enable(FAR struct gpio_dev_s *dev, bool enable);

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
  .go_read   = gpin_read,
  .go_write  = gpout_write,
  .go_attach = NULL,
  .go_enable = NULL,
};

static const struct gpio_operations_s gpint_ops =
{
  .go_read   = gpin_read,
  .go_write  = NULL,
  .go_attach = gpint_attach,
  .go_enable = gpint_enable,
};

static struct simgpio_dev_s g_gpin =
{
  .gpio =
  {
    .gp_pintype = GPIO_INPUT_PIN,
    .gp_ops     = &gpin_ops,
  },
  .id = 0,
};

static struct simgpio_dev_s g_gpout =
{
  .gpio =
  {
    .gp_pintype = GPIO_OUTPUT_PIN,
    .gp_ops     = &gpout_ops,
  },
  .id = 1,
};

static struct simgpint_dev_s g_gpint =
{
  .simgpio =
  {
    .gpio =
    {
      .gp_pintype = GPIO_INTERRUPT_PIN,
      .gp_ops     = &gpint_ops,
    },
    .id = 2,
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int sim_interrupt(wdparm_t arg)
{
  FAR struct simgpint_dev_s *simgpint = (FAR struct simgpint_dev_s *)arg;

  DEBUGASSERT(simgpint != NULL && simgpint->callback != NULL);
  gpioinfo("Interrupt! callback=%p\n", simgpint->callback);

  simgpint->callback(&simgpint->simgpio.gpio, simgpint->simgpio.id);
  return OK;
}

static int gpin_read(FAR struct gpio_dev_s *dev, FAR bool *value)
{
  FAR struct simgpio_dev_s *simgpio = (FAR struct simgpio_dev_s *)dev;

  DEBUGASSERT(simgpio != NULL && value != NULL);
  gpioinfo("Reading %d (next=%d)\n",
          (int)simgpio->value, (int)!simgpio->value);

  *value = simgpio->value;
  simgpio->value = !simgpio->value;
  return OK;
}

static int gpout_write(FAR struct gpio_dev_s *dev, bool value)
{
  FAR struct simgpio_dev_s *simgpio = (FAR struct simgpio_dev_s *)dev;

  DEBUGASSERT(simgpio != NULL);
  gpioinfo("Writing %d\n", (int)value);

  simgpio->value = value;
  return OK;
}

static int gpint_attach(FAR struct gpio_dev_s *dev,
                        pin_interrupt_t callback)
{
  FAR struct simgpint_dev_s *simgpint = (FAR struct simgpint_dev_s *)dev;

  gpioinfo("Cancel 1 second timer\n");
  wd_cancel(&simgpint->wdog);

  gpioinfo("Attach %p\n", callback);
  simgpint->callback = callback;
  return OK;
}

static int gpint_enable(FAR struct gpio_dev_s *dev, bool enable)
{
  FAR struct simgpint_dev_s *simgpint = (FAR struct simgpint_dev_s *)dev;

  if (enable)
    {
      if (simgpint->callback != NULL)
        {
          gpioinfo("Start 1 second timer\n");
          wd_start(&simgpint->wdog, SEC2TICK(1),
                   sim_interrupt, (wdparm_t)dev);
        }
    }
  else
    {
       gpioinfo("Cancel 1 second timer\n");
      wd_cancel(&simgpint->wdog);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sim_gpio_initialize
 *
 * Description:
 *   Initialize GPIO drivers for use with /apps/examples/gpio
 *
 ****************************************************************************/

int sim_gpio_initialize(void)
{
  gpio_pin_register(&g_gpin.gpio, g_gpin.id);
  gpio_pin_register(&g_gpout.gpio, g_gpout.id);
  gpio_pin_register(&g_gpint.simgpio.gpio, g_gpint.simgpio.id);
  return 0;
}
#endif /* CONFIG_EXAMPLES_GPIO && !CONFIG_GPIO_LOWER_HALF */
