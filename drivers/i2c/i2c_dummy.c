/****************************************************************************
 * drivers/i2c/i2c_dummy.c
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

#include <assert.h>
#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/i2c/i2c_dummy.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct i2c_dummy_dev_s
{
  struct i2c_master_s i2c;
  mutex_t lock;
  FAR struct i2c_dummy_device_s * const *devices;
  int ndevices;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int i2c_dummy_setup(FAR struct i2c_master_s *dev);
static int i2c_dummy_shutdown(FAR struct i2c_master_s *dev);
static uint8_t i2c_dummy_status(FAR struct i2c_master_s *dev);
#ifdef CONFIG_I2C_RESET
static int i2c_dummy_reset(FAR struct i2c_master_s *dev);
#endif
static FAR struct i2c_dummy_device_s *
  i2c_dummy_find(FAR struct i2c_dummy_dev_s *priv, uint16_t addr);
static int i2c_dummy_transfer(FAR struct i2c_master_s *dev,
                              FAR struct i2c_msg_s *msgs, int count);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct i2c_ops_s g_i2c_dummy_ops =
{
  i2c_dummy_transfer,
#ifdef CONFIG_I2C_RESET
  i2c_dummy_reset,
#endif
  i2c_dummy_setup,
  i2c_dummy_shutdown,
  i2c_dummy_status
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int i2c_dummy_setup(FAR struct i2c_master_s *dev)
{
  return OK;
}

static int i2c_dummy_shutdown(FAR struct i2c_master_s *dev)
{
  return OK;
}

static uint8_t i2c_dummy_status(FAR struct i2c_master_s *dev)
{
  return I2C_STATUS_PRESENT;
}

#ifdef CONFIG_I2C_RESET
static int i2c_dummy_reset(FAR struct i2c_master_s *dev)
{
  FAR struct i2c_dummy_dev_s *priv = (FAR struct i2c_dummy_dev_s *)dev;
  FAR struct i2c_dummy_device_s *dummy;
  int i;

  nxmutex_lock(&priv->lock);
  for (i = 0; i < priv->ndevices; i++)
    {
      dummy = priv->devices[i];
      if (dummy->reset != NULL)
        {
          dummy->reset(dummy);
        }
    }

  nxmutex_unlock(&priv->lock);

  return OK;
}
#endif

static FAR struct i2c_dummy_device_s *
i2c_dummy_find(FAR struct i2c_dummy_dev_s *priv, uint16_t addr)
{
  FAR struct i2c_dummy_device_s *dummy;
  int i;

  for (i = 0; i < priv->ndevices; i++)
    {
      dummy = priv->devices[i];
      if (dummy->addr == addr)
        {
          return dummy;
        }
    }

  return NULL;
}

static int i2c_dummy_transfer(FAR struct i2c_master_s *dev,
                             FAR struct i2c_msg_s *msgs, int count)
{
  FAR struct i2c_dummy_dev_s *priv = (FAR struct i2c_dummy_dev_s *)dev;
  FAR struct i2c_dummy_device_s *dummy;
  int ret = OK;
  int i;

  DEBUGASSERT(priv != NULL);

  if (msgs == NULL || count <= 0)
    {
      return -EINVAL;
    }

  nxmutex_lock(&priv->lock);

  for (i = 0; i < count; i++)
    {
      dummy = i2c_dummy_find(priv, msgs[i].addr);
      if (dummy == NULL)
        {
          ret = -ENXIO;
          break;
        }

      if (dummy->transfer == NULL)
        {
          ret = -ENOSYS;
          break;
        }

      ret = dummy->transfer(dummy, &msgs[i]);
      if (ret < 0)
        {
          break;
        }
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct i2c_master_s *
i2c_dummy_initialize(FAR struct i2c_dummy_device_s * const *devices,
                     int ndevices)
{
  FAR struct i2c_dummy_dev_s *priv;

  if (ndevices < 0 || (devices == NULL && ndevices > 0))
    {
      return NULL;
    }

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      return NULL;
    }

  priv->i2c.ops = &g_i2c_dummy_ops;
  priv->devices = devices;
  priv->ndevices = ndevices;
  nxmutex_init(&priv->lock);

  return &priv->i2c;
}
