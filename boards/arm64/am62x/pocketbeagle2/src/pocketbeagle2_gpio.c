/****************************************************************************
 * boards/arm64/am62x/pocketbeagle2/src/pocketbeagle2_gpio.c
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include <nuttx/ioexpander/gpio.h>

#include <arch/board/board.h>

#include "am62x_gpio.h"
#include "pocketbeagle2.h"

#if defined(CONFIG_DEV_GPIO) && !defined(CONFIG_GPIO_LOWER_HALF)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PB2_GPIO_OUT_FLAGS       (AM62X_GPIO_OUTPUT | AM62X_GPIO_PULL_NONE)
#define PB2_LED_PADCFG           (AM62X_PADCFG_PULL_DISABLE | \
                                  AM62X_PADCFG_MUXMODE(7))
#define PB2_PADCFG_GPIO0_3       0x000c
#define PB2_PADCFG_GPIO0_4       0x0010
#define PB2_PADCFG_GPIO0_5       0x0014
#define PB2_PADCFG_GPIO0_6       0x0018

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct pocketbeagle2_gpio_dev_s
{
  struct gpio_dev_s gpio;
  uint8_t id;
};

struct pocketbeagle2_gpio_pin_s
{
  gpio_pinset_t pinset;
  uintptr_t padcfg;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int gpout_read(struct gpio_dev_s *dev, bool *value);
static int gpout_write(struct gpio_dev_s *dev, bool value);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct gpio_operations_s gpout_ops =
{
  .go_read = gpout_read,
  .go_write = gpout_write,
  .go_attach = NULL,
  .go_enable = NULL,
};

static const struct pocketbeagle2_gpio_pin_s g_gpiooutputs[BOARD_NGPIOOUT] =
{
  { AM62X_GPIO_PIN(0, 3) | PB2_GPIO_OUT_FLAGS, PB2_PADCFG_GPIO0_3 },
  { AM62X_GPIO_PIN(0, 4) | PB2_GPIO_OUT_FLAGS, PB2_PADCFG_GPIO0_4 },
  { AM62X_GPIO_PIN(0, 5) | PB2_GPIO_OUT_FLAGS, PB2_PADCFG_GPIO0_5 },
  { AM62X_GPIO_PIN(0, 6) | PB2_GPIO_OUT_FLAGS, PB2_PADCFG_GPIO0_6 },
};

static struct pocketbeagle2_gpio_dev_s g_gpout[BOARD_NGPIOOUT];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int gpout_read(struct gpio_dev_s *dev, bool *value)
{
  struct pocketbeagle2_gpio_dev_s *pb2gpio =
    (struct pocketbeagle2_gpio_dev_s *)dev;

  DEBUGASSERT(pb2gpio != NULL);
  DEBUGASSERT(value != NULL);
  DEBUGASSERT(pb2gpio->id < BOARD_NGPIOOUT);

  *value = am62x_gpiooutread(g_gpiooutputs[pb2gpio->id].pinset);
  return OK;
}

static int gpout_write(struct gpio_dev_s *dev, bool value)
{
  struct pocketbeagle2_gpio_dev_s *pb2gpio =
    (struct pocketbeagle2_gpio_dev_s *)dev;

  DEBUGASSERT(pb2gpio != NULL);
  DEBUGASSERT(pb2gpio->id < BOARD_NGPIOOUT);

  am62x_gpiowrite(g_gpiooutputs[pb2gpio->id].pinset, value);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int pocketbeagle2_gpio_initialize(void)
{
  int i;
  int ret;

  for (i = 0; i < BOARD_NGPIOOUT; i++)
    {
      ret = am62x_pinmux_configure(g_gpiooutputs[i].padcfg, PB2_LED_PADCFG);
      if (ret < 0)
        {
          return ret;
        }

      ret = am62x_configgpio(g_gpiooutputs[i].pinset);
      if (ret < 0)
        {
          return ret;
        }

      g_gpout[i].gpio.gp_pintype = GPIO_OUTPUT_PIN;
      g_gpout[i].gpio.gp_ops = &gpout_ops;
      g_gpout[i].id = i;

      ret = gpio_pin_register(&g_gpout[i].gpio,
                              AM62X_GPIO_LINE(g_gpiooutputs[i].pinset));
      if (ret < 0)
        {
          return ret;
        }
    }

  return OK;
}

#endif /* defined(CONFIG_DEV_GPIO) && !defined(CONFIG_GPIO_LOWER_HALF) */
