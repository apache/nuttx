/****************************************************************************
 * drivers/sensors/adxl367.c
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

#include <errno.h>
#include <string.h>
#include <sys/param.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/signal.h>
#include <nuttx/sensors/adxl367.h>
#include <nuttx/sensors/sensor.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ADXL367_I2C_FREQUENCY      100000
#define ADXL367_DEVID_AD           0x00
#define ADXL367_DEVID_AD_VALUE     0xad
#define ADXL367_DEVID_MST_VALUE    0x1d
#define ADXL367_PARTID_VALUE       0xf7
#define ADXL367_STATUS             0x0b
#define ADXL367_STATUS_DATA_READY  (1 << 0)
#define ADXL367_XDATA_H            0x0e
#define ADXL367_SOFT_RESET         0x1f
#define ADXL367_SOFT_RESET_VALUE   0x52
#define ADXL367_FILTER_CTL         0x2c
#define ADXL367_POWER_CTL          0x2d
#define ADXL367_POWER_MEASURE      2
#define ADXL367_TEMP_CTL           0x3d
#define ADXL367_TEMP_ENABLE        (1 << 0)
#define ADXL367_DEFAULT_ODR        3

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct adxl367_dev_s
{
  struct sensor_lowerhalf_s lower;
  FAR struct i2c_master_s *i2c;
  mutex_t lock;
  uint8_t addr;
  uint8_t odr;
  bool enabled;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int adxl367_activate(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep, bool enable);
static int adxl367_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                FAR struct file *filep,
                                FAR uint32_t *period_us);
static int adxl367_fetch(FAR struct sensor_lowerhalf_s *lower,
                         FAR struct file *filep, FAR char *buffer,
                         size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_adxl367_ops =
{
  .activate     = adxl367_activate,
  .set_interval = adxl367_set_interval,
  .fetch        = adxl367_fetch,
};

static const uint32_t g_adxl367_periods[] =
{
  80000, 40000, 20000, 10000, 5000, 2500
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: adxl367_getregs
 ****************************************************************************/

static int adxl367_getregs(FAR struct adxl367_dev_s *priv, uint8_t reg,
                           FAR uint8_t *buffer, int len)
{
  struct i2c_msg_s msg[2] =
  {
    {
      .frequency = ADXL367_I2C_FREQUENCY,
      .addr      = priv->addr,
      .flags     = I2C_M_NOSTOP,
      .buffer    = &reg,
      .length    = 1,
    },
    {
      .frequency = ADXL367_I2C_FREQUENCY,
      .addr      = priv->addr,
      .flags     = I2C_M_READ,
      .buffer    = buffer,
      .length    = len,
    }
  };

  int ret = I2C_TRANSFER(priv->i2c, msg, 2);

  return ret < 0 ? ret : OK;
}

/****************************************************************************
 * Name: adxl367_putreg
 ****************************************************************************/

static int adxl367_putreg(FAR struct adxl367_dev_s *priv, uint8_t reg,
                          uint8_t value)
{
  uint8_t buffer[2];
  struct i2c_config_s config =
  {
    .frequency = ADXL367_I2C_FREQUENCY,
    .address   = priv->addr,
    .addrlen   = 7,
  };

  int ret;

  buffer[0] = reg;
  buffer[1] = value;
  ret = i2c_write(priv->i2c, &config, buffer, sizeof(buffer));

  return ret < 0 ? ret : OK;
}

/****************************************************************************
 * Name: adxl367_set_power
 ****************************************************************************/

static int adxl367_set_power(FAR struct adxl367_dev_s *priv, bool enable)
{
  int ret;

  ret = adxl367_putreg(priv, ADXL367_POWER_CTL,
                       enable ? ADXL367_POWER_MEASURE : 0);
  if (ret == OK)
    {
      priv->enabled = enable;
    }

  return ret;
}

/****************************************************************************
 * Name: adxl367_activate
 ****************************************************************************/

static int adxl367_activate(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep, bool enable)
{
  FAR struct adxl367_dev_s *priv = (FAR struct adxl367_dev_s *)lower;
  int ret;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = adxl367_set_power(priv, enable);
  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: adxl367_set_interval
 ****************************************************************************/

static int adxl367_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                FAR struct file *filep,
                                FAR uint32_t *period_us)
{
  FAR struct adxl367_dev_s *priv = (FAR struct adxl367_dev_s *)lower;
  bool enabled;
  uint8_t odr = 0;
  int ret;
  int restore;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  enabled = priv->enabled;

  while (odr < nitems(g_adxl367_periods) - 1 &&
         *period_us < g_adxl367_periods[odr])
    {
      odr++;
    }

  if (odr == priv->odr)
    {
      *period_us = g_adxl367_periods[odr];
      goto out;
    }

  /* FILTER_CTL must only be changed in standby. Keep the range at +/-2 g
   * and disable the nonstandard high speed I2C mode.
   */

  ret = adxl367_set_power(priv, false);
  if (ret < 0)
    {
      goto out;
    }

  ret = adxl367_putreg(priv, ADXL367_FILTER_CTL, odr);
  if (ret == OK)
    {
      priv->odr = odr;
      *period_us = g_adxl367_periods[odr];
    }

  if (enabled)
    {
      restore = adxl367_set_power(priv, true);
      if (ret == OK)
        {
          ret = restore;
        }
    }

out:
  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: adxl367_data
 ****************************************************************************/

static int16_t adxl367_data(FAR const uint8_t *data)
{
  return (int16_t)((data[0] << 8) | data[1]) / 4;
}

/****************************************************************************
 * Name: adxl367_fetch
 ****************************************************************************/

static int adxl367_fetch(FAR struct sensor_lowerhalf_s *lower,
                         FAR struct file *filep, FAR char *buffer,
                         size_t buflen)
{
  FAR struct adxl367_dev_s *priv = (FAR struct adxl367_dev_s *)lower;
  struct sensor_accel sample;
  uint8_t data[8];
  uint8_t status;
  int ret;

  if (buflen < sizeof(sample))
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = adxl367_getregs(priv, ADXL367_STATUS, &status, 1);
  if (ret < 0)
    {
      goto out;
    }

  if (!(status & ADXL367_STATUS_DATA_READY))
    {
      ret = -EAGAIN;
      goto out;
    }

  ret = adxl367_getregs(priv, ADXL367_XDATA_H, data, sizeof(data));
  if (ret < 0)
    {
      goto out;
    }

  sample.timestamp = sensor_get_timestamp();
  sample.x = sensor_data_muli(sensor_data_ftof(9.80665f / 4000),
                              adxl367_data(&data[0]));
  sample.y = sensor_data_muli(sensor_data_ftof(9.80665f / 4000),
                              adxl367_data(&data[2]));
  sample.z = sensor_data_muli(sensor_data_ftof(9.80665f / 4000),
                              adxl367_data(&data[4]));

  /* Typical uncalibrated temperature: 165 LSB at 25 C, 54 LSB/C. */

  sample.temperature =
    sensor_data_add(sensor_data_ftof(25.0f),
                    sensor_data_divi(
                      sensor_data_itof(adxl367_data(&data[6]) - 165),
                      54));
  memcpy(buffer, &sample, sizeof(sample));
  ret = sizeof(sample);

out:
  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: adxl367_register
 ****************************************************************************/

int adxl367_register(int devno, FAR struct i2c_master_s *i2c, uint8_t addr)
{
  FAR struct adxl367_dev_s *priv;
  uint8_t id[3];
  int ret;

  if (i2c == NULL || (addr != 0x1d && addr != 0x53))
    {
      return -EINVAL;
    }

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  priv->i2c        = i2c;
  priv->addr       = addr;
  priv->odr        = ADXL367_DEFAULT_ODR;
  priv->lower.ops  = &g_adxl367_ops;
  priv->lower.type = SENSOR_TYPE_ACCELEROMETER;
  nxmutex_init(&priv->lock);

  /* Allow power-up to standby before the first bus access. */

  nxsig_usleep(10000);
  ret = adxl367_getregs(priv, ADXL367_DEVID_AD, id, sizeof(id));
  if (ret < 0)
    {
      goto errout;
    }

  if (id[0] != ADXL367_DEVID_AD_VALUE ||
      id[1] != ADXL367_DEVID_MST_VALUE ||
      id[2] != ADXL367_PARTID_VALUE)
    {
      ret = -ENODEV;
      goto errout;
    }

  ret = adxl367_putreg(priv, ADXL367_SOFT_RESET, ADXL367_SOFT_RESET_VALUE);
  if (ret < 0)
    {
      goto errout;
    }

  nxsig_usleep(10000);
  ret = adxl367_putreg(priv, ADXL367_FILTER_CTL, priv->odr);
  if (ret < 0)
    {
      goto errout;
    }

  ret = adxl367_putreg(priv, ADXL367_TEMP_CTL, ADXL367_TEMP_ENABLE);
  if (ret < 0)
    {
      goto errout;
    }

  ret = sensor_register(&priv->lower, devno);
  if (ret < 0)
    {
      goto errout;
    }

  return OK;

errout:
  nxmutex_destroy(&priv->lock);
  kmm_free(priv);
  return ret;
}
