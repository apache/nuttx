/****************************************************************************
 * boards/arm/cxd56xx/spresense/src/cxd56_gpioif.c
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

#include <stdio.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "cxd56_pinconfig.h"
#include "cxd56_gpio.h"
#include "cxd56_gpioint.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_gpio_config
 ****************************************************************************/

int board_gpio_config(uint32_t pin, int mode, bool input, bool drive,
                      int pull)
{
  uint32_t pinconf;

  pinconf = PINCONF_SET_PIN(pin);
  pinconf |= (input) ? PINCONF_INPUT_ENABLE : PINCONF_INPUT_DISABLE;
  pinconf |= (drive) ? PINCONF_DRIVE_HIGH : PINCONF_DRIVE_NORMAL;

  switch (pull)
    {
      case PIN_PULLUP:
        pinconf |= PINCONF_PULLUP;
        break;
      case PIN_PULLDOWN:
        pinconf |= PINCONF_PULLDOWN;
        break;
      case PIN_BUSKEEPER:
        pinconf |= PINCONF_BUSKEEPER;
        break;
      default:
      case PIN_FLOAT:
        pinconf |= PINCONF_FLOAT;
        break;
    }

  pinconf |= PINCONF_SET_MODE(mode);

  return cxd56_pin_config(pinconf);
}

/****************************************************************************
 * Name: board_gpio_status
 ****************************************************************************/

int board_gpio_status(uint32_t pin, bool *input, bool *output, bool *drive,
                      int *pull)
{
  int ret;
  cxd56_pin_status_t pstat;
  cxd56_gpio_status_t gstat;

  DEBUGASSERT(input);
  DEBUGASSERT(output);
  DEBUGASSERT(drive);
  DEBUGASSERT(pull);

  ret = cxd56_pin_status(pin, &pstat);
  if (ret < 0)
    {
      return ret;
    }

  *input = PINCONF_INPUT_ENABLED(pstat.input_en);
  *drive = PINCONF_IS_DRIVE_HIGH(pstat.drive);
  if (PINCONF_IS_FLOAT(pstat.pull))
    {
      *pull = PIN_FLOAT;
    }
  else if (PINCONF_IS_PULLUP(pstat.pull))
    {
      *pull = PIN_PULLUP;
    }
  else if (PINCONF_IS_PULLDOWN(pstat.pull))
    {
      *pull = PIN_PULLDOWN;
    }
  else
    {
      *pull = PIN_BUSKEEPER;
    }

  cxd56_gpio_status(pin, &gstat);
  *output = gstat.output_en;

  return pstat.mode;
}

/****************************************************************************
 * Name: board_gpio_write
 ****************************************************************************/

void board_gpio_write(uint32_t pin, int value)
{
  if (value < 0)
    {
      cxd56_gpio_write_hiz(pin);
    }
  else
    {
      cxd56_gpio_write(pin, (value > 0));
    }

  return;
}

/****************************************************************************
 * Name: board_gpio_read
 ****************************************************************************/

int board_gpio_read(uint32_t pin)
{
  return (int)cxd56_gpio_read(pin);
}

/****************************************************************************
 * Name: board_gpio_intconfig
 ****************************************************************************/

int board_gpio_intconfig(uint32_t pin, int mode, bool filter, xcpt_t isr)
{
#ifdef CONFIG_CXD56_GPIO_IRQ
  int ret = OK;
  uint32_t gpiocfg = 0;

  switch (mode)
    {
      case INT_HIGH_LEVEL:
        gpiocfg = GPIOINT_LEVEL_HIGH;
        break;
      case INT_LOW_LEVEL:
        gpiocfg = GPIOINT_LEVEL_LOW;
        break;
      case INT_RISING_EDGE:
        gpiocfg = GPIOINT_PSEUDO_EDGE_RISE;
        break;
      case INT_FALLING_EDGE:
        gpiocfg = GPIOINT_PSEUDO_EDGE_FALL;
        break;
      case INT_BOTH_EDGE:
        gpiocfg = GPIOINT_PSEUDO_EDGE_BOTH;
        break;
      default:
        if (isr)
          {
            return -EINVAL;
          }
        break;
    }

  if (filter)
    {
      gpiocfg |= GPIOINT_NOISE_FILTER_ENABLE;
    }

  ret = cxd56_gpioint_config(pin, gpiocfg, isr, (void *)pin);

  return ret;
#else
  return -ENOTSUP;
#endif
}

/****************************************************************************
 * Name: board_gpio_intstatus
 ****************************************************************************/

int board_gpio_intstatus(uint32_t pin,
                         int *mode,
                         bool *filter,
                         bool *enabled)
{
#ifdef CONFIG_CXD56_GPIO_IRQ
  int ret;
  cxd56_gpioint_status_t stat;

  DEBUGASSERT(mode);
  DEBUGASSERT(filter);
  DEBUGASSERT(enabled);

  ret = cxd56_gpioint_status(pin, &stat);
  if (ret < 0)
    {
      return ret;
    }

  switch (stat.polarity)
    {
      case GPIOINT_LEVEL_HIGH:
        *mode = INT_HIGH_LEVEL;
        break;
      case GPIOINT_LEVEL_LOW:
        *mode = INT_LOW_LEVEL;
        break;
      case GPIOINT_EDGE_RISE:
        *mode = INT_RISING_EDGE;
        break;
      case GPIOINT_EDGE_FALL:
        *mode = INT_FALLING_EDGE;
        break;
      case GPIOINT_EDGE_BOTH:
        *mode = INT_BOTH_EDGE;
        break;
      default:
        *mode = 0;
        break;
    }

  *filter = stat.filter;
  *enabled = stat.enable;

  return stat.irq;
#else
  return -ENOTSUP;
#endif
}

/****************************************************************************
 * Name: board_gpio_int
 ****************************************************************************/

int board_gpio_int(uint32_t pin, bool enable)
{
#ifdef CONFIG_CXD56_GPIO_IRQ
  int irq = cxd56_gpioint_irq(pin);

  if (irq > 0)
    {
      if (enable)
        {
          cxd56_gpioint_enable(pin);
        }
      else
        {
          cxd56_gpioint_disable(pin);
        }
    }

  return irq;
#else
  return -ENOTSUP;
#endif
}
