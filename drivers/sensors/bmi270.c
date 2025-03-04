/****************************************************************************
 * drivers/sensors/bmi270.c
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

#include <stdlib.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/sensors/bmi270.h>
#ifdef CONFIG_SENSORS_BMI270_I2C
#  include <nuttx/i2c/i2c_master.h>
#else
#  include <nuttx/spi/spi.h>
#endif

#include "bmi270_base.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Character driver methods */

static int     bmi270_open(FAR struct file *filep);
static int     bmi270_close(FAR struct file *filep);
static ssize_t bmi270_read(FAR struct file *filep, FAR char *buffer,
                            size_t len);
static int     bmi270_ioctl(FAR struct file *filep, int cmd,
                            unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This the vtable that supports the character driver interface */

static const struct file_operations g_bmi270fops =
{
  bmi270_open,     /* open */
  bmi270_close,    /* close */
  bmi270_read,     /* read */
  NULL,            /* write */
  NULL,            /* seek */
  bmi270_ioctl,    /* ioctl */
};

/****************************************************************************
 * Name: bmi270_open
 *
 * Description:
 *   Standard character driver open method.
 *
 ****************************************************************************/

static int bmi270_open(FAR struct file *filep)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct bmi270_dev_s *priv  = inode->i_private;
  int                      ret   = OK;

  /* Initialization sequence */

  ret = bmi270_init_seq(priv);
  if (ret != 0)
    {
      return ret;
    }

  /* Set normal mode */

  bmi270_set_normal_imu(priv);

  return OK;
}

/****************************************************************************
 * Name: bmi270_close
 *
 * Description:
 *   Standard character driver close method.
 *
 ****************************************************************************/

static int bmi270_close(FAR struct file *filep)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct bmi270_dev_s *priv  = inode->i_private;

  /* Disable acquisition of acc and gyro */

  bmi270_putreg8(priv, BMI270_PWR_CTRL, 0);
  up_mdelay(30);

  return OK;
}

/****************************************************************************
 * Name: bmi270_read
 *
 * Description:
 *   Standard character driver read method.
 *
 ****************************************************************************/

static ssize_t bmi270_read(FAR struct file *filep, FAR char *buffer,
                           size_t len)
{
  FAR struct inode           *inode = filep->f_inode;
  FAR struct bmi270_dev_s    *priv  = inode->i_private;
  FAR struct accel_gyro_st_s *p     = (FAR struct accel_gyro_st_s *)buffer;

  if (len < sizeof(struct accel_gyro_st_s))
    {
      snerr("Expected buffer size is %d\n", sizeof(struct accel_gyro_st_s));
      return 0;
    }

  bmi270_getregs(priv, BMI270_DATA_8, (FAR uint8_t *)p, 15);

  /* Adjust sensing time into 24 bit */

  p->sensor_time >>= 8;

  return len;
}

/****************************************************************************
 * Name: bmi270_ioctl
 *
 * Description:
 *   Standard character driver ioctl method.
 *
 ****************************************************************************/

static int bmi270_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  return -ENOTTY;
}

/****************************************************************************
 * Name: bmi270_register
 *
 * Description:
 *   Register the BMI270 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/imu0"
 *   dev     - An instance of the SPI or I2C interface to use to communicate
 *             with BMI270
 *   addr - (I2C only) The I2C address
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_BMI270_I2C
int bmi270_register(FAR const char *devpath, FAR struct i2c_master_s *dev,
                    uint8_t addr)
#else /* CONFIG_SENSORS_BMI270_SPI */
int bmi270_register(FAR const char *devpath, FAR struct spi_dev_s *dev)
#endif
{
  FAR struct bmi270_dev_s *priv;
  int ret;

  priv = kmm_malloc(sizeof(struct bmi270_dev_s));
  if (!priv)
    {
      snerr("Failed to allocate instance\n");
      return -ENOMEM;
    }

#ifdef CONFIG_SENSORS_BMI270_I2C
  priv->i2c  = dev;
  priv->addr = addr;
  priv->freq = BMI270_I2C_FREQ;
#else /* CONFIG_SENSORS_BMI270_SPI */
  priv->spi = dev;

  /* BMI270 detects communication bus is SPI by rising edge of CS. */

  bmi270_getreg8(priv, 0x00);
  bmi270_getreg8(priv, 0x00);
  up_udelay(200);
#endif

  ret = bmi270_checkid(priv);
  if (ret < 0)
    {
      snerr("Wrong Device ID!\n");
      kmm_free(priv);
      return ret;
    }

  /* Register driver */

  ret = register_driver(devpath, &g_bmi270fops, 0666, priv);
  if (ret < 0)
    {
      snerr("Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  sninfo("BMI270 driver loaded successfully!\n");
  return OK;
}
