/****************************************************************************
 * drivers/i2c/i2c_bitbang.c
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
#include <debug.h>

#include <nuttx/spinlock.h>
#include <nuttx/semaphore.h>
#include <nuttx/kmalloc.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/i2c/i2c_bitbang.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct i2c_bitbang_dev_s
{
  struct i2c_master_s i2c;
  struct i2c_bitbang_lower_dev_s *lower;

#ifndef CONFIG_I2C_BITBANG_NO_DELAY
  int32_t delay;
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int i2c_bitbang_transfer(FAR struct i2c_master_s *dev,
                                FAR struct i2c_msg_s *msgs, int count);

static int i2c_bitbang_set_scl(FAR struct i2c_bitbang_dev_s *dev,
                               bool high, bool nodelay);
static void i2c_bitbang_set_sda(FAR struct i2c_bitbang_dev_s *dev,
                                bool high);

static int i2c_bitbang_wait_ack(FAR struct i2c_bitbang_dev_s *dev);
static void i2c_bitbang_send(FAR struct i2c_bitbang_dev_s *dev,
                             uint8_t data);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct i2c_ops_s g_i2c_ops =
{
  .transfer = i2c_bitbang_transfer
};

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

inline static bool i2c_bitbang_get_sda(FAR struct i2c_bitbang_dev_s *dev)
{
  return dev->lower->ops->get_sda(dev->lower);
}

#ifdef CONFIG_I2C_BITBANG_CLOCK_STRETCHING
inline static bool i2c_bitbang_get_scl(FAR struct i2c_bitbang_dev_s *dev)
{
  return dev->lower->ops->get_scl(dev->lower);
}
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: i2c_bitbang_transfer
 ****************************************************************************/

static int i2c_bitbang_transfer(FAR struct i2c_master_s *dev,
                                FAR struct i2c_msg_s *msgs, int count)
{
  FAR struct i2c_bitbang_dev_s *priv = (FAR struct i2c_bitbang_dev_s *)dev;
  int ret = OK;
  int i;
  irqstate_t flags;

  /* Lock to enforce timings */

  flags = spin_lock_irqsave(NULL);

  for (i = 0; i < count; i++)
    {
      uint8_t addr;
      FAR struct i2c_msg_s *msg = &msgs[i];

#ifndef CONFIG_I2C_BITBANG_NO_DELAY
      /* Compute delay from frequency */

      priv->delay = (USEC_PER_SEC / (2 * msg->frequency)) -
                    CONFIG_I2C_BITBANG_GPIO_OVERHEAD;

      if (priv->delay < 0)
        {
          priv->delay = 0;
        }
#endif

      /* If this is the start of transfer or we're changing sending direction
       * from last transfer, send START
       */

      if (i == 0 ||
          (msgs[i - 1].flags & I2C_M_READ) != (msgs[i].flags & I2C_M_READ))
        {
          /* Send start bit */

          i2c_bitbang_set_scl(priv, true, false);
          i2c_bitbang_set_sda(priv, false);
          i2c_bitbang_set_scl(priv, false, false);

          /* Send the address */

          addr = (msg->flags & I2C_M_READ ? I2C_READADDR8(msg->addr) :
                                            I2C_WRITEADDR8(msg->addr));

          i2c_bitbang_send(priv, addr);

          /* Wait for ACK */

          ret = i2c_bitbang_wait_ack(priv);

          if (ret < 0)
            {
              goto out;
            }
        }

      i2c_bitbang_set_scl(priv, false, false);

      if (msg->flags & I2C_M_READ)
        {
          int j;
          int k;

          for (j = 0; j < msg->length; j++)
            {
              uint8_t data = 0;

              i2c_bitbang_set_sda(priv, true);

              msg->buffer[j] = 0;

              for (k = 0; k < 8; k++)
                {
                  i2c_bitbang_set_scl(priv, true, false);
                  data |= (i2c_bitbang_get_sda(priv) & 1) << (7 - k);
                  i2c_bitbang_set_scl(priv, false, false);
                }

              msg->buffer[j] = data;

              if (j < msg->length - 1)
                {
                  /* Send ACK */

                  i2c_bitbang_set_sda(priv, false);
                  i2c_bitbang_set_scl(priv, true, false);
                  i2c_bitbang_set_scl(priv, false, false);
                }
              else
                {
                  /* On the last byte send NAK */

                  i2c_bitbang_set_sda(priv, true);
                  i2c_bitbang_set_scl(priv, true, false);
                  i2c_bitbang_set_scl(priv, false, false);
                }
            }
        }
      else
        {
          int j;

          for (j = 0; j < msg->length; j++)
            {
              /* Send the data */

              i2c_bitbang_send(priv, msg->buffer[j]);

              ret = i2c_bitbang_wait_ack(priv);

              if (ret < 0)
                {
                  goto out;
                }

              i2c_bitbang_set_scl(priv, false, false);
            }
        }

      if (!(msg->flags & I2C_M_NOSTOP))
        {
          /* Send stop */

          i2c_bitbang_set_sda(priv, false);
          i2c_bitbang_set_scl(priv, true, true);
          i2c_bitbang_set_sda(priv, true);
        }
    }

out:

  /* Ensure lines are released */

  i2c_bitbang_set_scl(priv, true, false);
  i2c_bitbang_set_sda(priv, true);

  spin_unlock_irqrestore(NULL, flags);

  return ret;
}

/****************************************************************************
 * Name: i2c_bitbang_wait_ack
 ****************************************************************************/

static int i2c_bitbang_wait_ack(FAR struct i2c_bitbang_dev_s *priv)
{
  int ret = OK;
  int i;

  /* Wait for ACK */

  i2c_bitbang_set_sda(priv, true);
  i2c_bitbang_set_scl(priv, true, true);

  for (i = 0; i2c_bitbang_get_sda(priv) &&
              i < CONFIG_I2C_BITBANG_TIMEOUT; i++)
    {
      up_udelay(1);
    }

  if (i == CONFIG_I2C_BITBANG_TIMEOUT)
    {
      ret = -EIO;
    }
#ifndef CONFIG_I2C_BITBANG_NO_DELAY
  else
    {
      int remaining = priv->delay - i;

      if (remaining > 0)
        {
          up_udelay(remaining);
        }
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: i2c_bitbang_send
 ****************************************************************************/

static void i2c_bitbang_send(FAR struct i2c_bitbang_dev_s *priv,
                             uint8_t data)
{
  uint8_t bit = 0b10000000;

  while (bit)
    {
      i2c_bitbang_set_sda(priv, !!(data & bit));
      i2c_bitbang_set_scl(priv, true, false);
      i2c_bitbang_set_scl(priv, false, false);

      bit >>= 1;
    }
}

/****************************************************************************
 * Name: i2c_bitbang_set_sda
 ****************************************************************************/

static void i2c_bitbang_set_sda(FAR struct i2c_bitbang_dev_s *dev, bool high)
{
  dev->lower->ops->set_sda(dev->lower, high);
}

/****************************************************************************
 * Name: i2c_bitbang_set_scl
 ****************************************************************************/

static int i2c_bitbang_set_scl(FAR struct i2c_bitbang_dev_s *dev, bool high,
                               bool nodelay)
{
  dev->lower->ops->set_scl(dev->lower, high);

#ifndef CONFIG_I2C_BITBANG_NO_DELAY
  if (!nodelay && dev->delay)
    {
      up_udelay(dev->delay);
    }
#endif

#ifdef CONFIG_I2C_BITBANG_CLOCK_STRETCHING
  /* Allow for clock stretching */

  if (high)
    {
      int i;

      for (i = 0; !i2c_bitbang_get_scl(dev) &&
                  i < CONFIG_I2C_BITBANG_TIMEOUT; i++)
        {
          up_udelay(1);

          if (i == CONFIG_I2C_BITBANG_TIMEOUT)
            {
              return -ETIMEDOUT;
            }
        }
    }
#endif

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: i2c_bitbang_initialize
 *
 * Description:
 *   Initialize a bitbang I2C device instance
 *
 * Input Parameters:
 *   lower  - Lower half of driver
 *
 * Returned Value:
 *   Pointer to a the I2C instance
 *
 ****************************************************************************/

FAR struct i2c_master_s *i2c_bitbang_initialize(
    FAR struct i2c_bitbang_lower_dev_s *lower)
{
  FAR struct i2c_bitbang_dev_s *dev;

  DEBUGASSERT(lower && lower->ops);

  dev = (FAR struct i2c_bitbang_dev_s *)kmm_zalloc(sizeof(*dev));

  if (!dev)
    {
      return NULL;
    }

  dev->i2c.ops = &g_i2c_ops;
  dev->lower = lower;
  dev->lower->ops->initialize(dev->lower);

  return &dev->i2c;
}

