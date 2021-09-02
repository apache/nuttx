/****************************************************************************
 * drivers/sensors/bh1750fvi.c
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

/* Character driver for the Rohm Ambient Light Sensor BH1750FVI */

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
#include <nuttx/sensors/bh1750fvi.h>
#include <nuttx/random.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_BH1750FVI)

/****************************************************************************
 * Pre-process Definitions
 ****************************************************************************/

#ifndef CONFIG_BH1750FVI_I2C_FREQUENCY
#  define CONFIG_BH1750FVI_I2C_FREQUENCY 400000
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct bh1750fvi_dev_s
{
  FAR struct i2c_master_s *i2c; /* I2C interface */
  uint8_t addr;                 /* I2C address */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C Helpers */

static int     bh1750fvi_read16(FAR struct bh1750fvi_dev_s *priv,
                                FAR uint16_t *regval);
static int     bh1750fvi_write8(FAR struct bh1750fvi_dev_s *priv,
                                uint8_t regval);

/* Character driver methods */

static int     bh1750fvi_open(FAR struct file *filep);
static int     bh1750fvi_close(FAR struct file *filep);
static ssize_t bh1750fvi_read(FAR struct file *filep, FAR char *buffer,
                              size_t buflen);
static ssize_t bh1750fvi_write(FAR struct file *filep,
                               FAR const char *buffer, size_t buflen);
static int     bh1750fvi_ioctl(FAR struct file *filep, int cmd,
                               unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_bh1750fvi_fops =
{
  NULL,             /* open */
  NULL,             /* close */
  bh1750fvi_read,   /* read */
  bh1750fvi_write,  /* write */
  NULL,             /* seek */
  bh1750fvi_ioctl,  /* ioctl */
  NULL              /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL            /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bh1750fvi_read16
 *
 * Description:
 *   Read 16-bit register
 *
 ****************************************************************************/

static int bh1750fvi_read16(FAR struct bh1750fvi_dev_s *priv,
                            FAR uint16_t *regval)
{
  struct i2c_config_s config;
  uint8_t buffer[2];
  int ret = -1;

  /* Set up the I2C configuration */

  config.frequency = CONFIG_BH1750FVI_I2C_FREQUENCY;
  config.address   = priv->addr;
  config.addrlen   = 7;

  /* Read 16-bits from the device */

  ret = i2c_read(priv->i2c, &config, buffer, 2);
  if (ret < 0)
    {
      snerr ("i2c_read failed: %d\n", ret);
      return ret;
    }

  /* Copy the content of the buffer to the location of the uint16_t pointer */

  *regval = (uint16_t)((buffer[0] << 8) | (buffer[1]));

  sninfo("value: %08x ret: %d\n", *regval, ret);
  return OK;
}

/****************************************************************************
 * Name: bh1750fvi_write8
 *
 * Description:
 *   Write from an 8-bit register
 *
 ****************************************************************************/

static int bh1750fvi_write8(FAR struct bh1750fvi_dev_s *priv, uint8_t regval)
{
  struct i2c_config_s config;
  int ret;

  sninfo("value: %02x\n", regval);

  /* Set up the I2C configuration */

  config.frequency = CONFIG_BH1750FVI_I2C_FREQUENCY;
  config.address   = priv->addr;
  config.addrlen   = 7;

  /* Write 8 bits to device */

  ret = i2c_write(priv->i2c, &config, &regval, 1);
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: bh1750fvi_read
 ****************************************************************************/

static ssize_t bh1750fvi_read(FAR struct file *filep, FAR char *buffer,
                              size_t buflen)
{
  int ret;
  FAR struct inode         *inode;
  FAR struct bh1750fvi_dev_s *priv;
  uint16_t lux = 0;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct bh1750fvi_dev_s *)inode->i_private;

  /* Check if the user is reading the right size */

  if (buflen != 2)
    {
      snerr("ERROR: You need to read 2 bytes from this sensor!\n");
      return -EINVAL;
    }

  ret = bh1750fvi_read16(priv, &lux);
  if (ret < 0)
    {
      snerr("ERROR: Error reading light sensor!\n");
      return ret;
    }

  buffer[0] = lux & 0xff;
  buffer[1] = (lux & 0xff00) >> 8;

  add_sensor_randomness(lux);

  return buflen;
}

/****************************************************************************
 * Name: bh1750fvi_write
 ****************************************************************************/

static ssize_t bh1750fvi_write(FAR struct file *filep,
                               FAR const char *buffer, size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: bh1750fvi_ioctl
 ****************************************************************************/

static int bh1750fvi_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct bh1750fvi_dev_s *priv = inode->i_private;
  int ret = OK;

  switch (cmd)
    {
      /* Set device to Continuously H-Resolution Mode */

      case SNIOC_CHRM:
        {
          ret = bh1750fvi_write8(priv, BH1750FVI_CONTINUOUS_HRM);
          if (ret < 0)
            {
              snerr("ERROR:");
              snerr(" Cannot change to Continuously H-Resolution Mode!\n");
            }
        }
        break;

      /* Set device to Continuously H-Resolution Mode2 */

      case SNIOC_CHRM2:
        {
          ret = bh1750fvi_write8(priv, BH1750FVI_CONTINUOUS_HRM2);
          if (ret < 0)
            {
              snerr("ERROR:");
              snerr(" Cannot change to Continuously H-Resolution Mode2!\n");
            }
        }
        break;

      /* Set device to Continuously L-Resolution Mode */

      case SNIOC_CLRM:
        {
          ret = bh1750fvi_write8(priv, BH1750FVI_CONTINUOUS_LRM);
          if (ret < 0)
            {
              snerr("ERROR:");
              snerr(" Cannot change to Continuously L-Resolution Mode!\n");
            }
        }
        break;

      /* Set device to One Time H-Resolution Mode */

      case SNIOC_OTHRM:
        {
          ret = bh1750fvi_write8(priv, BH1750FVI_ONETIME_HRM);
          if (ret < 0)
            {
              snerr("ERROR: Cannot change to One Time H-Resolution Mode!\n");
            }
        }
        break;

      /* Set device to One Time H-Resolution Mode 2 */

      case SNIOC_OTHRM2:
        {
          ret = bh1750fvi_write8(priv, BH1750FVI_ONETIME_HRM2);
          if (ret < 0)
            {
              snerr("ERROR:");
              snerr(" Cannot change to One Time H-Resolution Mode2!\n");
            }
        }
        break;

      /* Set device to One Time L-Resolution Mode */

      case SNIOC_OTLRM:
        {
          ret = bh1750fvi_write8(priv, BH1750FVI_ONETIME_LRM);
          if (ret < 0)
            {
              snerr("ERROR: Cannot change to One Time L-Resolution Mode!\n");
            }
        }
        break;

      /* Change the Measurement Time */

      case SNIOC_CHMEATIME:
        {
          FAR uint8_t *ptr = (FAR uint8_t *)((uintptr_t)arg);
          uint8_t reg;
          DEBUGASSERT(ptr != NULL);

          reg = BH1750FVI_MEASURE_TIMEH | ((*ptr & 0xe0) >> 5);

          ret = bh1750fvi_write8(priv, reg);
          if (ret < 0)
            {
              snerr("ERROR: Cannot Change Measure Time at MEASURE_TIMEH!\n");
            }

          reg = BH1750FVI_MEASURE_TIMEL | (*ptr & 0x1f);

          ret = bh1750fvi_write8(priv, reg);
          if (ret < 0)
            {
              snerr("ERROR: Cannot Change Measure Time at MEASURE_TIMEL!\n");
            }
        }
        break;

      default:
        snerr("ERROR: Unrecognized cmd: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bh1750fvi_register
 *
 * Description:
 *   Register the BH1750FVI character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/light0"
 *   i2c - An instance of the I2C interface to use to communicate with
 *         BH1750FVI
 *   addr - The I2C address of the BH1750FVI.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int bh1750fvi_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                       uint8_t addr)
{
  int ret;

  /* Sanity check */

  DEBUGASSERT(i2c != NULL);

  /* Initialize the BH1750FVI device structure */

  FAR struct bh1750fvi_dev_s *priv =
    (FAR struct bh1750fvi_dev_s *)kmm_malloc(sizeof(struct bh1750fvi_dev_s));

  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c  = i2c;
  priv->addr = addr;

  /* Power on the device */

  ret = bh1750fvi_write8(priv, BH1750FVI_POWERON);
  if (ret < 0)
    {
      snerr("ERROR: Failed to power-on the BH1750FVI!\n");
      return ret;
    }

  /* Set Continuously H-Resolution Mode */

  ret = bh1750fvi_write8(priv, BH1750FVI_CONTINUOUS_HRM);
  if (ret < 0)
    {
      snerr("ERROR: Failed to enable the Continuously H-Resolution Mode!\n");
      return ret;
    }

  /* Register the character driver */

  ret = register_driver(devpath, &g_bh1750fvi_fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  return ret;
}

#endif /* CONFIG_I2C && CONFIG_SENSORS_BH1750FVI */
