/****************************************************************************
 * drivers/sensors/lsm9ds1.c
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

#include "lsm9ds1_base.h"

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_LSM9DS1)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Character Driver Methods */

static ssize_t lsm9ds1_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen);
static ssize_t lsm9ds1_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen);
static int     lsm9ds1_ioctl(FAR struct file *filep, int cmd,
                             unsigned long arg);

/* Common Register Function */

static int lsm9ds1_register(FAR const char *devpath,
                            FAR struct i2c_master_s *i2c, uint8_t addr,
                            FAR const struct lsm9ds1_ops_s *ops,
                            uint8_t datareg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_fops =
{
  NULL,            /* open */
  NULL,            /* close */
  lsm9ds1_read,    /* read */
  lsm9ds1_write,   /* write */
  NULL,            /* seek */
  lsm9ds1_ioctl,   /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lsm9ds1_read
 *
 * Description:
 *   The standard read method.
 *
 ****************************************************************************/

static ssize_t lsm9ds1_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen)
{
  FAR struct inode         *inode;
  FAR struct lsm9ds1_dev_s *priv;
  int                       ret;
  size_t                    i;
  size_t                    j;
  size_t                    samplesize;
  size_t                    nsamples;
  uint16_t                  data;
  FAR int16_t              *ptr;
  uint8_t                   regaddr;
  uint8_t                   lo;
  uint8_t                   hi;
  uint32_t                  merge = 0;

  /* Sanity check */

  inode = filep->f_inode;

  priv = inode->i_private;

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(priv->datareg == LSM9DS1_OUT_X_L_G ||
              priv->datareg == LSM9DS1_OUT_X_L_XL ||
              priv->datareg == LSM9DS1_OUT_X_L_M);
  DEBUGASSERT(buffer != NULL);

  samplesize = 3 * sizeof(*ptr);
  nsamples   = buflen / samplesize;
  ptr        = (FAR int16_t *)buffer;

  /* Get the requested number of samples */

  for (i = 0; i < nsamples; i++)
    {
      /* Reset the register address to the X low byte register */

      regaddr = priv->datareg;

      /* Read the X, Y and Z data */

      for (j = 0; j < 3; j++)
        {
          /* Read the low byte */

          ret = lsm9ds1_readreg8(priv, regaddr, &lo);
          if (ret < 0)
            {
              snerr("ERROR: lsm9ds1_readreg8 failed: %d\n", ret);
              return (ssize_t)ret;
            }

          regaddr++;

          /* Read the high byte */

          ret = lsm9ds1_readreg8(priv, regaddr, &hi);
          if (ret < 0)
            {
              snerr("ERROR: lsm9ds1_readreg8 failed: %d\n", ret);
              return (ssize_t)ret;
            }

          regaddr++;

          /* The data is 16 bits in two's complement representation */

          data = ((uint16_t)hi << 8) | (uint16_t)lo;

          /* Collect entropy */

          merge += data ^ (merge >> 16);

          /* The value is positive */

          if (data < 0x8000)
            {
              ptr[j] = (int16_t)data;
            }

          /* The value is negative, so find its absolute value by taking the
           * two's complement
           */

          else if (data > 0x8000)
            {
              data = ~data + 1;
              ptr[j] = -(int16_t)data;
            }

          /* The value is negative and can't be represented as a positive
           * int16_t value
           */

          else
            {
              ptr[j] = (int16_t)(-32768);
            }
        }
    }

  /* Feed sensor data to entropy pool */

  add_sensor_randomness(merge);

  return nsamples * samplesize;
}

/****************************************************************************
 * Name: lsm9ds1_write
 *
 * Description:
 *   A dummy write method.
 *
 ****************************************************************************/

static ssize_t lsm9ds1_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: lsm9ds1_ioctl
 *
 * Description:
 *   The standard ioctl method.
 *
 ****************************************************************************/

static int lsm9ds1_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode         *inode;
  FAR struct lsm9ds1_dev_s *priv;
  int                       ret;

  /* Sanity check */

  inode = filep->f_inode;

  priv = inode->i_private;

  DEBUGASSERT(priv != NULL);

  /* Handle ioctl commands */

  switch (cmd)
    {
      /* Start converting. Arg: None. */

      case SNIOC_START:
        ret = priv->ops->start(priv);
        break;

      /* Stop converting. Arg: None. */

      case SNIOC_STOP:
        ret = priv->ops->stop(priv);
        break;

      /* Set the sample rate. Arg: uint32_t value. */

      case SNIOC_SETSAMPLERATE:
        ret = priv->ops->setsamplerate(priv, (uint32_t)arg);
        sninfo("sample rate: %08" PRId32 " ret: %d\n",
               (uint32_t)arg, ret);
        break;

      /* Set the full-scale range. Arg: uint32_t value. */

      case SNIOC_SETFULLSCALE:
        ret = priv->ops->setfullscale(priv, (uint32_t)arg);
        sninfo("full-scale range: %08" PRId32 " ret: %d\n",
               (uint32_t)arg, ret);
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
 * Name: lsm9ds1_register
 *
 * Description:
 *   Register the LSM9DS1 accelerometer, gyroscope or magnetometer character
 *   device as 'devpath'.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register, e.g., "/dev/accel0",
 *             "/dev/gyro0" or "/dev/mag0".
 *   i2c     - An I2C driver instance.
 *   addr    - The I2C address of the LSM9DS1 accelerometer, gyroscope or
 *             magnetometer.
 *   ops     - The device operations structure.
 *   datareg - The register address of the low byte of the X-coordinate data.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int lsm9ds1_register(FAR const char *devpath,
                            FAR struct i2c_master_s *i2c, uint8_t addr,
                            FAR const struct lsm9ds1_ops_s *ops,
                            uint8_t datareg)
{
  FAR struct lsm9ds1_dev_s *priv;
  int ret;

  /* Sanity check */

  DEBUGASSERT(devpath != NULL);
  DEBUGASSERT(i2c != NULL);
  DEBUGASSERT(datareg == LSM9DS1_OUT_X_L_XL ||
              datareg == LSM9DS1_OUT_X_L_G ||
              datareg == LSM9DS1_OUT_X_L_M);

  /* Initialize the device's structure */

  priv = kmm_malloc(sizeof(*priv));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c        = i2c;
  priv->addr       = addr;
  priv->ops        = ops;
  priv->samplerate = 0;
  priv->datareg    = datareg;

  /* Configure the device */

  ret = priv->ops->config(priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to configure device: %d\n", ret);
      kmm_free(priv);
      return ret;
    }

  /* Register the character driver */

  ret = register_driver(devpath, &g_fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lsm9ds1accel_register
 *
 * Description:
 *   Register the LSM9DS1 accelerometer character device as 'devpath'.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register, e.g., "/dev/accel0".
 *   i2c     - An I2C driver instance.
 *   addr    - The I2C address of the LSM9DS1 accelerometer.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int lsm9ds1accel_register(FAR const char *devpath,
                          FAR struct i2c_master_s *i2c,
                          uint8_t addr)
{
  /* Sanity check */

  DEBUGASSERT(addr == LSM9DS1ACCEL_ADDR0 || addr == LSM9DS1ACCEL_ADDR1);

  return lsm9ds1_register(devpath, i2c, addr, &g_lsm9ds1accel_ops,
                          LSM9DS1_OUT_X_L_XL);
}

/****************************************************************************
 * Name: lsm9ds1gyro_register
 *
 * Description:
 *   Register the LSM9DS1 gyroscope character device as 'devpath'.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register, e.g., "/dev/gyro0".
 *   i2c     - An I2C driver instance.
 *   addr    - The I2C address of the LSM9DS1 gyroscope.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int lsm9ds1gyro_register(FAR const char *devpath,
                         FAR struct i2c_master_s *i2c,
                         uint8_t addr)
{
  /* Sanity check */

  DEBUGASSERT(addr == LSM9DS1GYRO_ADDR0 || addr == LSM9DS1GYRO_ADDR1);

  return lsm9ds1_register(devpath, i2c, addr, &g_lsm9ds1gyro_ops,
                          LSM9DS1_OUT_X_L_G);
}

/****************************************************************************
 * Name: lsm9ds1mag_register
 *
 * Description:
 *   Register the LSM9DS1 magnetometer character device as 'devpath'.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register, e.g., "/dev/mag0".
 *   i2c     - An I2C driver instance.
 *   addr    - The I2C address of the LSM9DS1 magnetometer.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int lsm9ds1mag_register(FAR const char *devpath,
                        FAR struct i2c_master_s *i2c,
                        uint8_t addr)
{
  /* Sanity check */

  DEBUGASSERT(addr == LSM9DS1MAG_ADDR0 || addr == LSM9DS1MAG_ADDR1);

  return lsm9ds1_register(devpath, i2c, addr, &g_lsm9ds1mag_ops,
                          LSM9DS1_OUT_X_L_M);
}
#endif /* CONFIG_I2C && CONFIG_SENSORS_LSM9DS1 */
