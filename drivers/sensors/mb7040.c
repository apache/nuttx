/****************************************************************************
 * drivers/sensors/mb7040.c
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
#include <stdlib.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/mb7040.h>
#include <nuttx/random.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_MB7040)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_MB7040_I2C_FREQUENCY
#  define CONFIG_MB7040_I2C_FREQUENCY 400000
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct mb7040_dev_s
{
  FAR struct i2c_master_s *i2c;  /* I2C interface */
  uint8_t                  addr; /* I2C address */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C Helpers */

static int mb7040_measurerange(FAR struct mb7040_dev_s *priv);
static int mb7040_readrange(FAR struct mb7040_dev_s *priv,
                            FAR uint16_t *range);
static int mb7040_changeaddr(FAR struct mb7040_dev_s *priv, uint8_t addr);

/* Character Driver Methods */

static ssize_t mb7040_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen);
static ssize_t mb7040_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen);
static int     mb7040_ioctl(FAR struct file *filep, int cmd,
                            unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_fops =
{
  NULL,            /* open */
  NULL,            /* close */
  mb7040_read,     /* read */
  mb7040_write,    /* write */
  NULL,            /* seek */
  mb7040_ioctl,    /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mb7040_measurerange
 *
 * Description:
 *   Command the device to measure the range.
 *
 ****************************************************************************/

static int mb7040_measurerange(FAR struct mb7040_dev_s *priv)
{
  struct i2c_config_s config;
  uint8_t regaddr;
  int ret;

  sninfo("addr: %02x\n", regaddr);

  /* Set up the I2C configuration */

  config.frequency = CONFIG_MB7040_I2C_FREQUENCY;
  config.address   = priv->addr;
  config.addrlen   = 7;

  /* Write the register address */

  regaddr = MB7040_RANGE_REG;

  ret = i2c_write(priv->i2c, &config, &regaddr, sizeof(regaddr));
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: mb7040_readrange
 *
 * Description:
 *   Read the range last measured by the device. The units are centimeters.
 *
 ****************************************************************************/

static int mb7040_readrange(FAR struct mb7040_dev_s *priv,
                            FAR uint16_t *range)
{
  struct i2c_config_s config;
  uint8_t buffer[2];
  int ret;

  /* Set up the I2C configuration */

  config.frequency = CONFIG_MB7040_I2C_FREQUENCY;
  config.address   = priv->addr;
  config.addrlen   = 7;

  /* Read two bytes */

  ret = i2c_read(priv->i2c, &config, buffer, sizeof(buffer));
  if (ret < 0)
    {
      snerr("ERROR: i2c_read failed: %d\n", ret);
      return ret;
    }

  *range = (uint16_t)buffer[0] << 8 | (uint16_t)buffer[1];
  sninfo("range: %04x ret: %d\n", *range, ret);
  return ret;
}

/****************************************************************************
 * Name: mb7040_changeaddr
 *
 * Description:
 *   Change the device's I2C address.
 *
 ****************************************************************************/

static int mb7040_changeaddr(FAR struct mb7040_dev_s *priv, uint8_t addr)
{
  struct i2c_config_s config;
  uint8_t buffer[3];
  int ret;

  sninfo("new addr: %02x\n", addr);

  /* Sanity check */

  DEBUGASSERT((addr & 1) == 0);
  DEBUGASSERT(addr != 0x00 && addr != 0x50 && addr != 0xa4 && addr != 0xaa);

  /* Set up a 3-byte message to send */

  buffer[0] = MB7040_ADDRUNLOCK1_REG;
  buffer[1] = MB7040_ADDRUNLOCK2_REG;
  buffer[2] = addr;

  /* Set up the I2C configuration */

  config.frequency = CONFIG_MB7040_I2C_FREQUENCY;
  config.address   = priv->addr;
  config.addrlen   = 7;

  /* Write the register address followed by the data (no RESTART) */

  ret = i2c_write(priv->i2c, &config, buffer, sizeof(buffer));
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
      return ret;
    }

  priv->addr = addr;
  return ret;
}

/****************************************************************************
 * Name: mb7040_read
 *
 * Description:
 *   A dummy read method.
 *
 ****************************************************************************/

static ssize_t mb7040_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
  return 0;
}

/****************************************************************************
 * Name: mb7040_write
 *
 * Description:
 *   A dummy write method.
 *
 ****************************************************************************/

static ssize_t mb7040_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: mb7040_ioctl
 *
 * Description:
 *   The standard ioctl method.
 *
 ****************************************************************************/

static int mb7040_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct mb7040_dev_s *priv  = inode->i_private;
  int                      ret   = OK;

  /* Handle ioctl commands */

  switch (cmd)
    {
      /* Command the device to measure the range. Arg: None. */

      case SNIOC_MEASURE:
        DEBUGASSERT(arg == 0);
        ret = mb7040_measurerange(priv);
        break;

      /* Read the range last measured by the device. Arg: int32_t* pointer. */

      case SNIOC_RANGE:
        {
          FAR int32_t *ptr   = (FAR int32_t *)((uintptr_t)arg);
          uint16_t     range = 0;
          DEBUGASSERT(ptr != NULL);
          ret = mb7040_readrange(priv, &range);
          if (ret == OK)
            {
              *ptr = (int32_t)range;

              /* Feed sensor data to entropy pool */

              add_sensor_randomness(range);
            }

          sninfo("range: %04x ret: %d\n", *ptr, ret);
        }
        break;

      /* Change the device's I2C address. Arg: uint8_t value. */

      case SNIOC_CHANGEADDR:
        ret = mb7040_changeaddr(priv, (uint8_t)arg);
        sninfo("new addr: %02x ret: %d\n", *(uint8_t *)arg, ret);
        break;

      /* Unrecognized commands */

      default:
        snerr("ERROR: Unrecognized cmd: %d arg: %ld\n", cmd, arg);
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mb7040_register
 *
 * Description:
 *   Register the MB7040 character device as 'devpath'.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register, e.g., "/dev/sonar0".
 *   i2c     - An I2C driver instance.
 *   addr    - The I2C address of the MB7040.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int mb7040_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                    uint8_t addr)
{
  FAR struct mb7040_dev_s *priv;
  int ret;

  /* Sanity check */

  DEBUGASSERT(i2c != NULL);

  /* Initialize the device's structure */

  priv = (FAR struct mb7040_dev_s *)kmm_malloc(sizeof(*priv));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c  = i2c;
  priv->addr = addr;

  /* Register the character driver */

  ret = register_driver(devpath, &g_fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  return ret;
}

#endif /* CONFIG_I2C && CONFIG_SENSORS_MB7040 */
