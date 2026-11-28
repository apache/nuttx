/****************************************************************************
 * arch/tricore/src/common/aurix/tricore_gpio_lower.c
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
#include <stdint.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/ioexpander/gpio.h>

#include "tricore_gpio.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct tricore_gpio_dev_s
{
  struct gpio_dev_s gpio;  /* Upper half, must be first */
  uint32_t pinset;         /* Current pin set */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int tricore_gpio_dev_read(FAR struct gpio_dev_s *dev,
                                 FAR bool *value);
static int tricore_gpio_dev_write(FAR struct gpio_dev_s *dev, bool value);
static int tricore_gpio_dev_setpintype(FAR struct gpio_dev_s *dev,
                                       enum gpio_pintype_e pintype);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct gpio_operations_s g_tricore_gpio_ops =
{
  .go_read       = tricore_gpio_dev_read,
  .go_write      = tricore_gpio_dev_write,
  .go_setpintype = tricore_gpio_dev_setpintype,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tricore_gpio_pintype
 *
 * Description:
 *   Return the GPIO pin type of a pin set.
 *
 ****************************************************************************/

static enum gpio_pintype_e tricore_gpio_pintype(uint32_t pinset)
{
  if (GPIO_IS_OUTPUT(pinset))
    {
      return (pinset & GPIO_OPENDRAIN) != 0 ? GPIO_OUTPUT_PIN_OPENDRAIN :
                                              GPIO_OUTPUT_PIN;
    }

  switch (pinset & GPIO_ALT_MASK)
    {
      case GPIO_PULLUP:
        return GPIO_INPUT_PIN_PULLUP;

      case GPIO_PULLDOWN:
        return GPIO_INPUT_PIN_PULLDOWN;

      default:
        return GPIO_INPUT_PIN;
    }
}

static int tricore_gpio_dev_read(FAR struct gpio_dev_s *dev,
                                 FAR bool *value)
{
  FAR struct tricore_gpio_dev_s *priv = (FAR struct tricore_gpio_dev_s *)dev;

  *value = tricore_gpio_read(priv->pinset);
  return OK;
}

static int tricore_gpio_dev_write(FAR struct gpio_dev_s *dev, bool value)
{
  FAR struct tricore_gpio_dev_s *priv = (FAR struct tricore_gpio_dev_s *)dev;

  tricore_gpio_write(priv->pinset, value);
  return OK;
}

/****************************************************************************
 * Name: tricore_gpio_dev_setpintype
 *
 * Description:
 *   Switch the pin between the GPIO input and output types.  The pad
 *   settings of the registered pin set are kept.  Interrupt types are not
 *   supported.
 *
 ****************************************************************************/

static int tricore_gpio_dev_setpintype(FAR struct gpio_dev_s *dev,
                                       enum gpio_pintype_e pintype)
{
  FAR struct tricore_gpio_dev_s *priv = (FAR struct tricore_gpio_dev_s *)dev;
  uint32_t pinset;
  int ret;

  pinset = priv->pinset & (GPIO_PORT_MASK | GPIO_PIN_MASK |
                           GPIO_PADDRV_MASK | GPIO_PADLEVEL_MASK);

  switch (pintype)
    {
      case GPIO_INPUT_PIN:
        pinset |= GPIO_INPUT | GPIO_FLOAT;
        break;

      case GPIO_INPUT_PIN_PULLUP:
        pinset |= GPIO_INPUT | GPIO_PULLUP;
        break;

      case GPIO_INPUT_PIN_PULLDOWN:
        pinset |= GPIO_INPUT | GPIO_PULLDOWN;
        break;

      case GPIO_OUTPUT_PIN:
        pinset |= GPIO_OUTPUT;
        break;

      case GPIO_OUTPUT_PIN_OPENDRAIN:
        pinset |= GPIO_OUTPUT | GPIO_OPENDRAIN;
        break;

      default:
        return -EINVAL;
    }

  /* Keep the current output level */

  if (tricore_gpio_read(priv->pinset))
    {
      pinset |= GPIO_OUTPUT_SET;
    }

  ret = tricore_gpio_config(pinset);
  if (ret < 0)
    {
      return ret;
    }

  priv->pinset = pinset;
  dev->gp_pintype = pintype;
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tricore_gpio_register
 *
 * Description:
 *   Configure the pin of a GPIO pin set and register it as the GPIO device
 *   /dev/gpio<minor>.
 *
 ****************************************************************************/

int tricore_gpio_register(int minor, uint32_t pinset)
{
  FAR struct tricore_gpio_dev_s *priv;
  int ret;

  if (GPIO_ALT_OF(pinset) != 0 && GPIO_IS_OUTPUT(pinset))
    {
      return -EINVAL;
    }

  priv = kmm_zalloc(sizeof(struct tricore_gpio_dev_s));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  ret = tricore_gpio_config(pinset);
  if (ret < 0)
    {
      kmm_free(priv);
      return ret;
    }

  priv->pinset = pinset;
  priv->gpio.gp_ops = &g_tricore_gpio_ops;
  priv->gpio.gp_pintype = tricore_gpio_pintype(pinset);

  ret = gpio_pin_register(&priv->gpio, minor);
  if (ret < 0)
    {
      kmm_free(priv);
    }

  return ret;
}
