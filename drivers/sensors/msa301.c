/****************************************************************************
 * drivers/sensors/msa301.c
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
#include <debug.h>
#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/mutex.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/msa301.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_MSA301)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct msa301_dev_s
{
  FAR struct i2c_master_s *      i2c;  /* I2C interface */
  uint8_t                        addr; /* I2C address */
  msa301_range_t                 range;
  FAR const struct msa301_ops_s *ops;
  mutex_t                        lock;
  struct msa301_sensor_data_s    sensor_data; /* Sensor data container     */
};

struct msa301_ops_s
{
  CODE int (*config)(FAR struct msa301_dev_s *priv);
  CODE int (*start)(FAR struct msa301_dev_s *priv);
  CODE int (*stop)(FAR struct msa301_dev_s *priv);
  CODE int (*sensor_read)(FAR struct msa301_dev_s *priv);
};

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#  ifndef CONFIG_MSA301_I2C_FREQUENCY
#    define CONFIG_MSA301_I2C_FREQUENCY 400000
#  endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C Helpers */

static int msa301_readreg8(FAR struct msa301_dev_s *priv,
                           uint8_t regaddr, FAR uint8_t *regval);
static int msa301_writereg8(FAR struct msa301_dev_s *priv,
                            uint8_t regaddr, uint8_t regval);
static int msa301_readreg(FAR struct msa301_dev_s *priv,
                          uint8_t regaddr, FAR uint8_t *regval, uint8_t len);

/* Accelerometer Operations */

static int msa301_sensor_config(FAR struct msa301_dev_s *priv);
static int msa301_sensor_start(FAR struct msa301_dev_s *priv);
static int msa301_sensor_stop(FAR struct msa301_dev_s *priv);
static int msa301_sensor_read(FAR struct msa301_dev_s *priv);

/* Character Driver Methods */

static int     msa301_open(FAR struct file *filep);
static int     msa301_close(FAR struct file *filep);
static ssize_t msa301_read(FAR struct file *filep,
                           FAR char *buffer, size_t buflen);
static ssize_t msa301_write(FAR struct file *filep,
                            FAR const char *buffer, size_t buflen);
static int     msa301_ioctl(FAR struct file *filep, int cmd,
                            unsigned long arg);

/* Common Register Function */

static int msa301_register(FAR const char *devpath,
                           FAR struct i2c_master_s *i2c, uint8_t addr,
                           FAR const struct msa301_ops_s *ops);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_fops =
{
  msa301_open,     /* open */
  msa301_close,    /* close */
  msa301_read,     /* read */
  msa301_write,    /* write */
  NULL,            /* seek */
  msa301_ioctl,    /* ioctl */
};

static const struct msa301_ops_s g_msa301_sensor_ops =
{
  msa301_sensor_config,
  msa301_sensor_start,
  msa301_sensor_stop,
  msa301_sensor_read,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int msa301_readreg(FAR struct msa301_dev_s *priv,
                          uint8_t regaddr, FAR uint8_t *regval, uint8_t len)
{
  struct i2c_config_s config;
  int                 ret;

  /* Sanity check */

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(regval != NULL);

  /* Set up the I2C configuration */

  config.frequency = CONFIG_MSA301_I2C_FREQUENCY;
  config.address   = priv->addr;
  config.addrlen   = 7;

  /* Write the register address */

  ret = i2c_write(priv->i2c, &config, &regaddr, sizeof(regaddr));
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
      return ret;
    }

  /* Restart and read 8 bits from the register */

  ret = i2c_read(priv->i2c, &config, regval, len);
  if (ret < 0)
    {
      snerr("ERROR: i2c_read failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: msa301_readreg8
 *
 * Description:
 *   Read from an 8-bit register.
 *
 ****************************************************************************/

static int msa301_readreg8(FAR struct msa301_dev_s *priv,
                           uint8_t regaddr, FAR uint8_t *regval)
{
  struct i2c_config_s config;
  int                 ret;

  /* Sanity check */

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(regval != NULL);

  /* Set up the I2C configuration */

  config.frequency = CONFIG_MSA301_I2C_FREQUENCY;
  config.address   = priv->addr;
  config.addrlen   = 7;

  /* Write the register address */

  ret = i2c_write(priv->i2c, &config, &regaddr, sizeof(regaddr));
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
      return ret;
    }

  /* Restart and read 8 bits from the register */

  ret = i2c_read(priv->i2c, &config, regval, sizeof(*regval));
  if (ret < 0)
    {
      snerr("ERROR: i2c_read failed: %d\n", ret);
      return ret;
    }

  sninfo("addr: %02x value: %02x\n", regaddr, *regval);

  return OK;
}

/****************************************************************************
 * Name: msa301_writereg8
 *
 * Description:
 *   Write to an 8-bit register.
 *
 ****************************************************************************/

static int msa301_writereg8(FAR struct msa301_dev_s *priv,
                            uint8_t regaddr, uint8_t regval)
{
  struct i2c_config_s config;
  uint8_t             buffer[2];
  int                 ret;

  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  /* Set up a 2-byte message to send */

  buffer[0] = regaddr;
  buffer[1] = regval;

  /* Set up the I2C configuration */

  config.frequency = CONFIG_MSA301_I2C_FREQUENCY;
  config.address   = priv->addr;
  config.addrlen   = 7;

  /* Write the register address followed by the data (no RESTART) */

  ret = i2c_write(priv->i2c, &config, buffer, sizeof(buffer));
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
      return ret;
    }

  sninfo("addr: %02x value: %02x\n", regaddr, regval);

  return OK;
}

static int msa301_set_range(FAR struct msa301_dev_s *priv,
                            msa301_range_t range)
{
  uint8_t ctl;

  if (range > MSA301_RANGE_16_G)
    {
      return -1;
    }

  msa301_readreg8(priv, MSA301_REG_RESRANGE, &ctl);
  ctl &= ~(MSA301_CTL_RANGE_MASK);
  ctl |= (range << MSA301_CTL_RANGE_SHIFT);
  msa301_writereg8(priv, MSA301_REG_RESRANGE, ctl);

  priv->range = range;

  return OK;
}

static int msa301_set_rate(FAR struct msa301_dev_s *priv, msa301_rate_t rate)
{
  uint8_t ctl;

  msa301_readreg8(priv, MSA301_REG_ODR, &ctl);
  ctl &= ~(MSA301_CTL_RATE_MASK);
  ctl |= (rate << MSA301_CTL_RATE_SHIFT);
  msa301_writereg8(priv, MSA301_REG_ODR, ctl);

  return OK;
}

static int msa301_set_powermode(FAR struct msa301_dev_s *priv,
                                msa301_powermode_t mode)
{
  uint8_t ctl;

  msa301_readreg8(priv, MSA301_REG_POWERMODE, &ctl);
  ctl &= ~(MSA301_CTL_POWERMODE_MASK);
  ctl |= (mode << MSA301_CTL_POWERMODE_SHIFT);
  msa301_writereg8(priv, MSA301_REG_POWERMODE, ctl);

  return OK;
}

static int msa301_set_resolution(FAR struct msa301_dev_s *priv,
                                 msa301_resolution_t resolution)
{
  uint8_t ctl;

  msa301_readreg8(priv, MSA301_REG_RESRANGE, &ctl);
  ctl &= ~(MSA301_CTL_RESOLUTION_MASK);
  ctl |= (resolution << MSA301_CTL_RESOLUTION_SHIFT);
  msa301_writereg8(priv, MSA301_REG_RESRANGE, ctl);

  return OK;
}

static int msa301_set_axis(FAR struct msa301_dev_s *priv, uint8_t enable)
{
  uint8_t ctl;

  msa301_readreg8(priv, MSA301_REG_ODR, &ctl);
  ctl &= ~(MSA301_CTL_AXIS_MASK);
  if (enable)
    {
      ctl |= (MSA301_ENABLE_AXIS << MSA301_CTL_AXIS_SHIFT);
    }
  else
    {
      ctl |= (MSA301_DISABLE_AXIS << MSA301_CTL_AXIS_SHIFT);
    }

  msa301_writereg8(priv, MSA301_REG_ODR, ctl);

  return OK;
}

static int msa301_sensor_config(FAR struct msa301_dev_s *priv)
{
  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  msa301_set_resolution(priv, MSA301_RESOLUTION_14);
  msa301_set_rate(priv, MSA301_RATE_500_HZ);
  msa301_set_range(priv, MSA301_RANGE_4_G);
  msa301_set_powermode(priv, MSA301_SUSPENDMODE);

  return OK;
}

/****************************************************************************
 * Name: msa301_sensor_start
 *
 * Description:
 *   Start the accelerometer.
 *
 ****************************************************************************/

static int msa301_sensor_start(FAR struct msa301_dev_s *priv)
{
  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  /* Power normal */

  msa301_set_powermode(priv, MSA301_NORMALMODE);

  /* Enable the accelerometer */

  msa301_set_axis(priv, 1);

  up_mdelay(5);

  sninfo("Starting....");

  return OK;
}

/****************************************************************************
 * Name: msa301_sensor_stop
 *
 * Description:
 *   Stop the accelerometer.
 *
 ****************************************************************************/

static int msa301_sensor_stop(FAR struct msa301_dev_s *priv)
{
  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  /* Disable the accelerometer */

  msa301_set_axis(priv, 0);

  /* Power suspend */

  msa301_set_powermode(priv, MSA301_SUSPENDMODE);

  sninfo("Stopping....");

  return OK;
}

/****************************************************************************
 * Name: msa301_sensor_read
 *
 * Description:
 *   Read the sensor.
 *   A sensor in a steady state on a horizontal surface will
 *   measure 0 g on both the X-axis and Y-axis, whereas the Z-axis will
 *   measure 1 g. The X- and Y-axis have an offset
 *   of 40 mg/LSB
 *
 ****************************************************************************/

static int msa301_sensor_read(FAR struct msa301_dev_s *priv)
{
  uint8_t xyz_value[6];
  float scale = 1;

  DEBUGASSERT(priv != NULL);

  if (msa301_readreg(priv, MSA301_REG_OUT_X_L, xyz_value, 6) < 0)
    {
      return -EIO;
    }

  priv->sensor_data.x_data = xyz_value[1] << 8 | xyz_value[0];
  priv->sensor_data.y_data = xyz_value[3] << 8 | xyz_value[2];
  priv->sensor_data.z_data = xyz_value[5] << 8 | xyz_value[4];

  /* 14 bit resolution */

  priv->sensor_data.x_data >>= 2;
  priv->sensor_data.y_data >>= 2;
  priv->sensor_data.z_data >>= 2;

  if (priv->range == MSA301_RANGE_2_G)
    {
      scale = 4096;
    }
  else if (priv->range == MSA301_RANGE_4_G)
    {
      scale = 2048;
    }
  else if (priv->range == MSA301_RANGE_8_G)
    {
      scale = 1024;
    }
  else if (priv->range == MSA301_RANGE_16_G)
    {
      scale = 512;
    }

  priv->sensor_data.x_acc = (float)priv->sensor_data.x_data / scale;
  priv->sensor_data.y_acc = (float)priv->sensor_data.y_data / scale;
  priv->sensor_data.z_acc = (float)priv->sensor_data.z_data / scale;

  return OK;
}

/****************************************************************************
 * Name: msa301_open
 *
 * Description:
 *   This method is called when the device is opened.
 *
 ****************************************************************************/

static int msa301_open(FAR struct file *filep)
{
  FAR struct inode *       inode;
  FAR struct msa301_dev_s *priv;

  DEBUGASSERT(filep != NULL);
  inode = filep->f_inode;

  DEBUGASSERT(inode != NULL);
  priv = (FAR struct msa301_dev_s *)inode->i_private;

  DEBUGASSERT(priv != NULL);

  nxmutex_lock(&priv->lock);

  priv->ops->start(priv);

  return OK;
}

/****************************************************************************
 * Name: msa301_close
 *
 * Description:
 *   This method is called when the device is closed.
 *
 ****************************************************************************/

static int msa301_close(FAR struct file *filep)
{
  FAR struct inode *       inode;
  FAR struct msa301_dev_s *priv;

  DEBUGASSERT(filep != NULL);
  inode = filep->f_inode;

  DEBUGASSERT(inode != NULL);
  priv = (FAR struct msa301_dev_s *)inode->i_private;

  DEBUGASSERT(priv != NULL);

  priv->ops->stop(priv);

  nxmutex_unlock(&priv->lock);

  return OK;
}

/****************************************************************************
 * Name: msa301_read
 *
 * Description:
 *   The standard read method.
 *
 ****************************************************************************/

static ssize_t msa301_read(FAR struct file *filep,
                           FAR char *buffer, size_t buflen)
{
  FAR struct inode *       inode;
  FAR struct msa301_dev_s *priv;
  int datalen = sizeof(struct msa301_sensor_data_s);

  /* Sanity check */

  DEBUGASSERT(filep != NULL);
  inode = filep->f_inode;

  DEBUGASSERT(inode != NULL);
  priv = (FAR struct msa301_dev_s *)inode->i_private;

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(buffer != NULL);

  if (buflen < sizeof(struct msa301_sensor_data_s))
    {
      return -ENOMEM;
    }

  if (priv->ops->sensor_read(priv) < 0)
    {
      return -EIO;
    }

  memcpy(buffer, &priv->sensor_data, datalen);

  return datalen;
}

/****************************************************************************
 * Name: msa301_write
 *
 * Description:
 *   A dummy write method.
 *
 ****************************************************************************/

static ssize_t msa301_write(FAR struct file *filep,
                            FAR const char *buffer, size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: msa301_ioctl
 *
 * Description:
 *   The standard ioctl method.
 *
 ****************************************************************************/

static int msa301_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *       inode;
  FAR struct msa301_dev_s *priv;
  int                      ret;

  /* Sanity check */

  DEBUGASSERT(filep != NULL);
  inode = filep->f_inode;

  DEBUGASSERT(inode != NULL);
  priv = (FAR struct msa301_dev_s *)inode->i_private;

  DEBUGASSERT(priv != NULL);

  /* Handle ioctl commands */

  switch (cmd)
    {
      case SNIOC_MSA301_START:
        ret = priv->ops->start(priv);
        break;

      case SNIOC_MSA301_STOP:
        ret = priv->ops->stop(priv);
        break;

      case SNIOC_MSA301_SET_RANGE:
        ret = msa301_set_range(priv, arg);
        break;

      case SNIOC_MSA301_SET_RATE:
        ret = msa301_set_rate(priv, arg);
        break;

        /* Unrecognized commands */

      default:
        snerr("ERROR: Unrecognized cmd: %d arg: %lu\n", cmd, arg);
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: msa301_register
 *
 * Description:
 *   Register the msa301 accelerometer, gyroscope device as 'devpath'.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register, e.g.
 *             "/dev/msa301".
 *   i2c     - An I2C driver instance.
 *   addr    - The I2C address of the msa301 accelerometer, gyroscope or
 *             magnetometer.
 *   ops     - The device operations structure.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int msa301_register(FAR const char *devpath,
                           FAR struct i2c_master_s *i2c, uint8_t addr,
                           FAR const struct msa301_ops_s *ops)
{
  FAR struct msa301_dev_s *priv;
  int ret;
  uint8_t id = 0;

  /* Sanity check */

  DEBUGASSERT(devpath != NULL);
  DEBUGASSERT(i2c != NULL);

  /* Initialize the device's structure */

  priv = (FAR struct msa301_dev_s *)kmm_malloc(sizeof(*priv));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c  = i2c;
  priv->addr = addr;
  priv->ops  = ops;

  /* ID check */

  msa301_readreg8(priv, MSA301_REG_PARTID, &id);
  if (id != 0x13)
    {
      snerr("ERROR: Failed to read msa301 id\n");
      kmm_free(priv);
      return -EIO;
    }

  /* Configure the device */

  ret = priv->ops->config(priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to configure device: %d\n", ret);
      kmm_free(priv);
      return ret;
    }

  nxmutex_init(&priv->lock);

  /* Register the character driver */

  ret = register_driver(devpath, &g_fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      nxmutex_destroy(&priv->lock);
      kmm_free(priv);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: msa301_sensor_register
 *
 * Description:
 *   Register the msa301 accelerometer character device as 'devpath'.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register,
 *             e.g. "/dev/msa301".
 *   i2c     - An I2C driver instance.
 *   addr    - The I2C address of the msa301 accelerometer.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int msa301_sensor_register(FAR const char *         devpath,
                           FAR struct i2c_master_s *i2c)
{
  return msa301_register(devpath, i2c, MSA301_ACCEL_ADDR0,
                         &g_msa301_sensor_ops);
}

#endif /* CONFIG_I2C && CONFIG_SENSORS_MSA301 */
