/****************************************************************************
 * drivers/power/relay/relay_gpio.c
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

#include <assert.h>
#include <errno.h>

#include <nuttx/ioexpander/ioexpander.h>
#include <nuttx/kmalloc.h>
#include <nuttx/power/relay.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct relay_gpio_priv
{
  struct relay_dev_s           dev;
  FAR struct ioexpander_dev_s *iodev;
  uint8_t                      iopin;
  bool                         ioinvert;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int relay_gpio_set(FAR struct relay_dev_s *dev, bool enable);
static int relay_gpio_get(FAR struct relay_dev_s *dev, FAR bool *enable);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct relay_ops_s g_relay_gpio_ops =
{
  relay_gpio_set,  /* set */
  relay_gpio_get,  /* get */
  NULL,            /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int relay_gpio_set(FAR struct relay_dev_s *dev, bool enable)
{
  FAR struct relay_gpio_priv *priv = (FAR struct relay_gpio_priv *)dev;

  return IOEXP_WRITEPIN(priv->iodev, priv->iopin, enable ^ priv->ioinvert);
}

static int relay_gpio_get(FAR struct relay_dev_s *dev, FAR bool *enable)
{
  FAR struct relay_gpio_priv *priv = (FAR struct relay_gpio_priv *)dev;
  bool val;
  int ret;

  ret = IOEXP_READPIN(priv->iodev, priv->iopin, &val);
  if (ret < 0)
    {
      return ret;
    }

  *enable = val ^ priv->ioinvert;
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: relay_gpio_register
 *
 * Description:
 *   Register the relay device based on the ioexpander device
 *
 * Input Parameters:
 *   iodev     - The ioexpander dev pointer.
 *   iopin     - The relay gpio pin number.
 *   ioinvert  - true : enable (relay close) is low , disable (relay open)
 *                      is high.
 *               false: enable (rekat close) is high, disable (relay open)
 *                      is low.
 *   devname   - The relay device name.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int relay_gpio_register(FAR struct ioexpander_dev_s *iodev, uint8_t iopin,
                        bool ioinvert, FAR const char *devname)
{
  FAR struct relay_gpio_priv *priv;
  int ret;

  DEBUGASSERT(iodev != NULL && devname != NULL);

  priv = kmm_zalloc(sizeof(struct relay_gpio_priv));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  priv->iodev    = iodev;
  priv->iopin    = iopin;
  priv->ioinvert = ioinvert;
  priv->dev.ops  = &g_relay_gpio_ops;

  ret = relay_register(&priv->dev, devname);
  if (ret < 0)
    {
      goto errout;
    }

  return ret;

errout:
  kmm_free(priv);
  return ret;
}
