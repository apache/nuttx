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
  struct ioexpander_dev_s *iodev;
  struct regulator_dev    *rdev;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int regulator_gpio_enable(struct regulator_dev *rdev);
static int regulator_gpio_disable(struct regulator_dev *rdev);
static int regulator_gpio_is_enabled(struct regulator_dev *rdev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct regulator_ops g_regulator_gpio_ops =
{
  .enable      = regulator_gpio_enable,
  .disable     = regulator_gpio_disable,
  .is_enabled  = regulator_gpio_is_enabled,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int regulator_gpio_enable(struct regulator_dev *rdev)
{
  struct regulator_gpio_priv *priv = rdev->priv;

  return IOEXP_WRITEPIN(priv->iodev, rdev->desc->enable_reg,
                        !!rdev->desc->enable_mask);
}

static int regulator_gpio_disable(struct regulator_dev *rdev)
{
  struct regulator_gpio_priv *priv = rdev->priv;

  return IOEXP_WRITEPIN(priv->iodev, rdev->desc->enable_reg,
                        !rdev->desc->enable_mask);
}

static int regulator_gpio_is_enabled(struct regulator_dev *rdev)
{
  struct regulator_gpio_priv *priv = rdev->priv;
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

int regulator_gpio_init(struct ioexpander_dev_s *iodev,
                        const struct regulator_desc *desc)
{
  struct regulator_gpio_priv *priv;
  int i;

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
