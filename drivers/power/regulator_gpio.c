/****************************************************************************
 * drivers/power/regulator_gpio.c
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

#include <errno.h>

#include <nuttx/ioexpander/ioexpander.h>
#include <nuttx/kmalloc.h>
#include <nuttx/power/regulator.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private
 ****************************************************************************/

struct regulator_gpio_priv
{
  FAR struct ioexpander_dev_s *iodev;
  FAR struct regulator_dev_s  *rdev;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int regulator_gpio_enable(FAR struct regulator_dev_s *rdev);
static int regulator_gpio_disable(FAR struct regulator_dev_s *rdev);
static int regulator_gpio_is_enabled(FAR struct regulator_dev_s *rdev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct regulator_ops_s g_regulator_gpio_ops =
{
  .enable      = regulator_gpio_enable,
  .disable     = regulator_gpio_disable,
  .is_enabled  = regulator_gpio_is_enabled,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int regulator_gpio_enable(FAR struct regulator_dev_s *rdev)
{
  FAR struct regulator_gpio_priv *priv = rdev->priv;

  return IOEXP_WRITEPIN(priv->iodev, rdev->desc->enable_reg,
                        !!rdev->desc->enable_mask);
}

static int regulator_gpio_disable(FAR struct regulator_dev_s *rdev)
{
  FAR struct regulator_gpio_priv *priv = rdev->priv;

  return IOEXP_WRITEPIN(priv->iodev, rdev->desc->enable_reg,
                        !rdev->desc->enable_mask);
}

static int regulator_gpio_is_enabled(FAR struct regulator_dev_s *rdev)
{
  FAR struct regulator_gpio_priv *priv = rdev->priv;
  bool val;

  if (!IOEXP_READPIN(priv->iodev, rdev->desc->enable_reg, &val))
    {
      return val ^ !rdev->desc->enable_mask;
    }
  else
    {
      return -EINVAL;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: regulator_gpio_init
 *
 * Description:
 *
 * Input Parameters:
 *
 *   iodev  - The ioexpander dev pointer.
 *   desc   - The regulator desc pointer, must contain follow section
 *            name          - The regulator name.
 *            enable_reg    - The regulator gpio pin number.
 *            enable_mask   -
 *                            true : enable is high, disable is low
 *                            false: enable is low,  disable is high
 *
 * Returned Value:
 *
 ****************************************************************************/

int regulator_gpio_init(FAR struct ioexpander_dev_s *iodev,
                        FAR const struct regulator_desc_s *desc)
{
  FAR struct regulator_gpio_priv *priv;

  if (!iodev || !desc)
    {
      return -EINVAL;
    }

  priv = kmm_zalloc(sizeof(struct regulator_gpio_priv));
  if (!priv)
    {
      return -ENOMEM;
    }

  priv->iodev = iodev;
  priv->rdev = regulator_register(desc, &g_regulator_gpio_ops,
                                  priv);
  if (!priv->rdev)
    {
      kmm_free(priv);
      return -EINVAL;
    }

  return 0;
}
