/****************************************************************************
 * drivers/sensors/bmi088.c
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

#include <nuttx/semaphore.h>

#include "bmi088_base.h"

#if defined(CONFIG_SENSORS_BMI088)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

static uint8_t acc_range      = BMI088_ACC_RANGE_6G;
static uint8_t gyro_range     = BMI088_GYRO_RANGE_2000DPS;
static uint8_t gyro_bandwidth = BMI088_GYRO_BANDWIDTH_2000HZ_532HZ;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Character driver methods */

static int     bmi088_acc_open(FAR struct file *filep);
static int     bmi088_gyro_open(FAR struct file *filep);
static int     bmi088_acc_close(FAR struct file *filep);
static int     bmi088_gyro_close(FAR struct file *filep);
static ssize_t bmi088_acc_read(FAR struct file *filep, FAR char *buffer,
                            size_t len);
static ssize_t bmi088_gyro_read(FAR struct file *filep, FAR char *buffer,
                            size_t len);
static int     bmi088_acc_ioctl(FAR struct file *filep, int cmd,
                            unsigned long arg);
static int     bmi088_gyro_ioctl(FAR struct file *filep, int cmd,
                            unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This the vtable that supports the character driver interface */

static const struct file_operations g_bmi088_acc_fops =
{
  bmi088_acc_open,     /* open */
  bmi088_acc_close,    /* close */
  bmi088_acc_read,     /* read */
  NULL,                /* write */
  NULL,                /* seek */
  bmi088_acc_ioctl,    /* ioctl */
};

/* This the vtable that supports the character driver interface */

static const struct file_operations g_bmi088_gyro_fops =
{
  bmi088_gyro_open,     /* open */
  bmi088_gyro_close,    /* close */
  bmi088_gyro_read,     /* read */
  NULL,                 /* write */
  NULL,                 /* seek */
  bmi088_gyro_ioctl,    /* ioctl */
};

/****************************************************************************
 * Name: bmi088_acc_open
 *
 * Description:
 *   Standard character driver open method.
 *
 ****************************************************************************/

static int bmi088_acc_open(FAR struct file *filep)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct bmi088_dev_s *priv  = inode->i_private;

  up_mdelay(1);
  bmi088_put_acc_reg8(priv, BMI088_ACC_PWR_CTRL ,
                            BMI088_ACC_PWR_CTRL_ACC_ENABLE);
  up_mdelay(5);
  bmi088_put_acc_reg8(priv, BMI088_ACC_RANGE    , acc_range);
  up_mdelay(50);
  bmi088_put_acc_reg8(priv, BMI088_ACC_PWR_CONF ,
                            BMI088_ACC_PWR_CONF_ACTIVE_MODE);

  bmi088_put_acc_reg8(priv, BMI088_INT1_IO_CONF , 0x08);
  bmi088_put_acc_reg8(priv, BMI088_INT1_INT2_MAP_DATA, 0x04);

  return OK;
}

/****************************************************************************
 * Name: bmi088_gyro_open
 *
 * Description:
 *   Standard character driver open method.
 *
 ****************************************************************************/

static int bmi088_gyro_open(FAR struct file *filep)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct bmi088_dev_s *priv  = inode->i_private;

  /* emable and config acc */

  bmi088_put_gyro_reg8(priv, BMI088_GYRO_LPM1     , BMI088_GYRO_PM_NORMAL);
  bmi088_put_gyro_reg8(priv, BMI088_GYRO_RANGE    , gyro_range);
  bmi088_put_gyro_reg8(priv, BMI088_GYRO_BANDWIDTH, gyro_bandwidth);

  bmi088_put_gyro_reg8(priv, BMI088_INT3_INT4_IO_CONF, 0x00);
  bmi088_put_gyro_reg8(priv, BMI088_INT3_INT4_IO_MAP , 0x01);
  bmi088_put_gyro_reg8(priv, BMI088_GYRO_INT_CTRL    , 0x80);

  return OK;
}

/****************************************************************************
 * Name: bmi088_acc_close
 *
 * Description:
 *   Standard character driver close method.
 *
 ****************************************************************************/

static int bmi088_acc_close(FAR struct file *filep)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct bmi088_dev_s *priv  = inode->i_private;

  /* Set suspend mode to each sensors. */

  bmi088_put_acc_reg8(priv, BMI088_ACC_INT_STAT_1, 0x00);

  bmi088_put_acc_reg8(priv, BMI088_ACC_PWR_CONF ,
                            BMI088_ACC_PWR_CONF_SUSPEND_MODE);
  up_mdelay(5);
  bmi088_put_acc_reg8(priv, BMI088_ACC_PWR_CTRL ,
                            BMI088_ACC_PWR_CTRL_ACC_DISABLE);
  up_mdelay(5);

  return OK;
}

/****************************************************************************
 * Name: bmi088_gyro_close
 *
 * Description:
 *   Standard character driver close method.
 *
 ****************************************************************************/

static int bmi088_gyro_close(FAR struct file *filep)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct bmi088_dev_s *priv  = inode->i_private;

  /* Set suspend mode to each sensors. */

  bmi088_put_gyro_reg8(priv, BMI088_GYRO_LPM1, BMI088_GYRO_PM_SUSPEND);
  up_mdelay(30);

  return OK;
}

/****************************************************************************
 * Name: bmi088_acc_read
 *
 * Description:
 *   Standard character driver read method.
 *
 ****************************************************************************/

static ssize_t bmi088_acc_read(FAR struct file *filep, FAR char *buffer,
                           size_t len)
{
  FAR struct inode         *inode = filep->f_inode;
  FAR struct bmi088_dev_s  *priv  = inode->i_private;
  FAR struct acc_gyro_st_s *p = (FAR struct acc_gyro_st_s *)buffer;

  if (len < sizeof(struct acc_gyro_st_s))
    {
      snerr("Expected buffer size is %zu\n", sizeof(struct acc_gyro_st_s));
      return 0;
    }

  /* read and caculate acc */

  bmi088_get_acc_regs(priv, BMI088_ACC_X_LSB, (uint8_t *)&p->acc_source, 6);
  p->accel.x = p->acc_source.x / 32768.0 * ((1 << (acc_range)) * 3.0);
  p->accel.y = p->acc_source.y / 32768.0 * ((1 << (acc_range)) * 3.0);
  p->accel.z = p->acc_source.z / 32768.0 * ((1 << (acc_range)) * 3.0);

  return len;
}

/****************************************************************************
 * Name: bmi088_acc_read
 *
 * Description:
 *   Standard character driver read method.
 *
 ****************************************************************************/

static ssize_t bmi088_gyro_read(FAR struct file *filep, FAR char *buffer,
                           size_t len)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct bmi088_dev_s *priv  = inode->i_private;
  FAR struct acc_gyro_st_s *p = (FAR struct acc_gyro_st_s *)buffer;

  if (len < sizeof(struct gyro_t))
    {
      snerr("Expected buffer size is %zu\n", sizeof(struct acc_gyro_st_s));
      return 0;
    }

  bmi088_get_gyro_regs(priv, BMI088_GYRO_X_LSB,
                            (uint8_t *)&p->gyro_source, 6);
  p->gyro.x = p->gyro_source.x / 32768.0 * (2000.0 / (1 << (gyro_range)));
  p->gyro.y = p->gyro_source.y / 32768.0 * (2000.0 / (1 << (gyro_range)));
  p->gyro.z = p->gyro_source.z / 32768.0 * (2000.0 / (1 << (gyro_range)));

  return len;
}

/****************************************************************************
 * Name: bmi088_acc_ioctl
 *
 * Description:
 *   Standard character driver ioctl method.
 *
 ****************************************************************************/

static int bmi088_acc_ioctl(FAR struct file *filep, int cmd,
                                                    unsigned long arg)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct bmi088_dev_s *priv  = inode->i_private;
  int ret = OK;

  switch (cmd)
    {
      case SNIOC_ACC_ENABLE:
        break;
      case SNIOC_ACC_DISABLE:
        break;
      default:
        snerr("Unrecognized cmd: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: bmi088_gyro_ioctl
 *
 * Description:
 *   Standard character driver ioctl method.
 *
 ****************************************************************************/

static int bmi088_gyro_ioctl(FAR struct file *filep, int cmd,
                                                     unsigned long arg)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct bmi088_dev_s *priv  = inode->i_private;
  int ret = OK;

  switch (cmd)
    {
      case SNIOC_GYRO_ENABLE:
        break;
      case SNIOC_GYRO_DISABLE:
        break;
      default:
        snerr("Unrecognized cmd: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bmi088_acc_register
 *
 * Description:
 *   Register the BMI088 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/press0"
 *   dev     - An instance of the SPI interface to use to communicate with
 *             BMI088
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_BMI088_I2C
int bmi088_acc_register(FAR const char *devpath,
                        FAR struct i2c_master_s *dev)
#else /* CONFIG_SENSORS_BMI088_SPI */
int bmi088_acc_register(FAR const char *devpath, FAR struct spi_dev_s *dev)
#endif
{
  FAR struct bmi088_dev_s *priv;
  int ret;

  priv = (FAR struct bmi088_dev_s *)kmm_malloc(sizeof(struct bmi088_dev_s));
  if (!priv)
    {
      snerr("Failed to allocate instance\n");
      return -ENOMEM;
    }

#ifdef CONFIG_SENSORS_BMI088_I2C
  priv->i2c = dev;
  priv->addr = BMI088_I2C_ACC_ADDR;
  priv->freq = BMI088_I2C_FREQ;

#else /* CONFIG_SENSORS_BMI088_SPI */
  priv->spi = dev;
#endif

  ret = bmi088_get_acc_reg8(priv, BMI088_ACC_CHIP_ID);
  if (ret != 0x1e)
    {
      snerr("Wrong Device ID!\n");
      kmm_free(priv);
      return ret;
    }

  ret = register_driver(devpath, &g_bmi088_acc_fops, 0666, priv);
  if (ret < 0)
    {
      snerr("Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  sninfo("BMI088 driver loaded successfully!\n");
  return OK;
}

/****************************************************************************
 * Name: bmi088_gyroregister
 *
 * Description:
 *   Register the BMI088 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/press0"
 *   dev     - An instance of the SPI interface to use to communicate with
 *             BMI088
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_BMI088_I2C
int bmi088_gyro_register(FAR const char *devpath,
                         FAR struct i2c_master_s *dev)
#else /* CONFIG_SENSORS_BMI088_SPI */
int bmi088_gyro_register(FAR const char *devpath, FAR struct spi_dev_s *dev)
#endif
{
  FAR struct bmi088_dev_s *priv;
  int ret;

  priv = (FAR struct bmi088_dev_s *)kmm_malloc(sizeof(struct bmi088_dev_s));
  if (!priv)
    {
      snerr("Failed to allocate instance\n");
      return -ENOMEM;
    }

#ifdef CONFIG_SENSORS_BMI088_I2C
  priv->i2c = dev;
  priv->addr = BMI088_I2C_GY_ADDR;
  priv->freq = BMI088_I2C_FREQ;

#else /* CONFIG_SENSORS_BMI088_SPI */
  priv->spi = dev;

#endif

  ret = bmi088_get_gyro_reg8(priv, BMI088_GYRO_CHIP_ID);
  if (ret != 0x0f)
    {
      snerr("Wrong Device ID!\n");
      kmm_free(priv);
      return ret;
    }

  ret = register_driver(devpath, &g_bmi088_gyro_fops, 0666, priv);
  if (ret < 0)
    {
      snerr("Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  sninfo("BMI088 driver loaded successfully!\n");
  return OK;
}

#endif /* CONFIG_SENSORS_BMI088 */
