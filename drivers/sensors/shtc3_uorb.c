/****************************************************************************
 * drivers/sensors/shtc3_uorb.c
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

#include <stdio.h>
#include <string.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/sensors/shtc3.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_SHTC3)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_SENSORS_USE_B16
#  error fixed-point data type not supported yet
#endif

#define SHTC3_CMD_WAKEUP  0x3517
#define SHTC3_CMD_SLEEP   0xb098
#define SHTC3_CMD_MEASURE 0x7866

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct shtc3_sensor_s
{
  struct sensor_lowerhalf_s lower;
  bool enabled;
  FAR struct shtc3_dev_s *dev;
};

struct shtc3_dev_s
{
  struct shtc3_sensor_s temp;
  struct shtc3_sensor_s humi;
  FAR struct i2c_master_s *i2c;
  uint8_t addr;
  mutex_t devlock;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int shtc3_activate(FAR struct sensor_lowerhalf_s *lower,
                          FAR struct file *filep, bool enable);
static int shtc3_fetch(FAR struct sensor_lowerhalf_s *lower,
                       FAR struct file *filep, FAR char *buffer,
                       size_t buflen);
static int shtc3_get_info(FAR struct sensor_lowerhalf_s *lower,
                          FAR struct file *filep,
                          FAR struct sensor_device_info_s *info);
static int shtc3_set_interval(FAR struct sensor_lowerhalf_s *lower,
                              FAR struct file *filep,
                              FAR uint32_t *period_us);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_sensor_ops =
{
  .activate     = shtc3_activate,
  .fetch        = shtc3_fetch,
  .set_interval = shtc3_set_interval,
  .get_info     = shtc3_get_info,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: shtc3_write_cmd
 ****************************************************************************/

static int shtc3_write_cmd(FAR struct shtc3_dev_s *priv, uint16_t cmd)
{
  struct i2c_msg_s msg;
  uint8_t buf[2];

  buf[0] = cmd >> 8;
  buf[1] = cmd & 0xff;

  msg.frequency = CONFIG_SHTC3_I2C_FREQUENCY;
  msg.addr      = priv->addr;
  msg.flags     = 0;
  msg.buffer    = buf;
  msg.length    = 2;

  return I2C_TRANSFER(priv->i2c, &msg, 1);
}

/****************************************************************************
 * Name: shtc3_read_data
 ****************************************************************************/

static int shtc3_read_data(FAR struct shtc3_dev_s *priv, FAR uint8_t *data)
{
  struct i2c_msg_s msg;

  msg.frequency = CONFIG_SHTC3_I2C_FREQUENCY;
  msg.addr      = priv->addr;
  msg.flags     = I2C_M_READ;
  msg.buffer    = data;
  msg.length    = 6;

  return I2C_TRANSFER(priv->i2c, &msg, 1);
}

/****************************************************************************
 * Name: shtc3_crc
 ****************************************************************************/

static uint8_t shtc3_crc(uint16_t word)
{
  static const uint8_t crc_table[16] =
  {
    0x00, 0x31, 0x62, 0x53, 0xc4, 0xf5, 0xa6, 0x97,
    0xb9, 0x88, 0xdb, 0xea, 0x7d, 0x4c, 0x1f, 0x2e
  };

  uint8_t crc = 0xff;
  crc ^= word >> 8;
  crc = (crc << 4) ^ crc_table[crc >> 4];
  crc = (crc << 4) ^ crc_table[crc >> 4];
  crc ^= word & 0xff;
  crc = (crc << 4) ^ crc_table[crc >> 4];
  crc = (crc << 4) ^ crc_table[crc >> 4];

  return crc;
}

/****************************************************************************
 * Name: shtc3_activate
 ****************************************************************************/

static int shtc3_activate(FAR struct sensor_lowerhalf_s *lower,
                          FAR struct file *filep, bool enable)
{
  FAR struct shtc3_sensor_s *sensor = (FAR struct shtc3_sensor_s *)lower;
  sensor->enabled = enable;
  return OK;
}

/****************************************************************************
 * Name: shtc3_set_interval
 ****************************************************************************/

static int shtc3_set_interval(FAR struct sensor_lowerhalf_s *lower,
                              FAR struct file *filep,
                              FAR uint32_t *period_us)
{
  return OK;
}

/****************************************************************************
 * Name: shtc3_get_info
 ****************************************************************************/

static int shtc3_get_info(FAR struct sensor_lowerhalf_s *lower,
                          FAR struct file *filep,
                          FAR struct sensor_device_info_s *info)
{
  FAR struct shtc3_sensor_s *sensor = (FAR struct shtc3_sensor_s *)lower;

  memset(info, 0, sizeof(struct sensor_device_info_s));
  info->version   = 1;
  info->power     = 0.43f; /* mA */
  info->min_delay = 15000; /* 15 ms measurement time */
  strlcpy(info->vendor, "Sensirion", sizeof(info->vendor));

  if (sensor->lower.type == SENSOR_TYPE_AMBIENT_TEMPERATURE)
    {
      info->max_range  = 125.0f;
      info->resolution = 0.01f;
      strlcpy(info->name, "shtc3_temp", sizeof(info->name));
    }
  else
    {
      info->max_range  = 100.0f;
      info->resolution = 0.01f;
      strlcpy(info->name, "shtc3_humi", sizeof(info->name));
    }

  return OK;
}

/****************************************************************************
 * Name: shtc3_fetch
 ****************************************************************************/

static int shtc3_fetch(FAR struct sensor_lowerhalf_s *lower,
                       FAR struct file *filep, FAR char *buffer,
                       size_t buflen)
{
  FAR struct shtc3_sensor_s *sensor = (FAR struct shtc3_sensor_s *)lower;
  FAR struct shtc3_dev_s *priv = sensor->dev;
  FAR struct sensor_event_s *event = (FAR struct sensor_event_s *)buffer;
  uint8_t data[6];
  uint16_t raw;
  int ret;

  if (buflen != sizeof(struct sensor_event_s))
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  ret = shtc3_write_cmd(priv, SHTC3_CMD_WAKEUP);
  if (ret < 0)
    {
      goto errout;
    }

  nxsched_usleep(240);

  ret = shtc3_write_cmd(priv, SHTC3_CMD_MEASURE);
  if (ret < 0)
    {
      goto errout;
    }

  nxsched_usleep(13000);

  ret = shtc3_read_data(priv, data);
  if (ret < 0)
    {
      goto errout;
    }

  shtc3_write_cmd(priv, SHTC3_CMD_SLEEP);

  event->timestamp = sensor_get_timestamp();

  if (sensor->lower.type == SENSOR_TYPE_AMBIENT_TEMPERATURE)
    {
      raw = (data[0] << 8) | data[1];
      if (shtc3_crc(raw) != data[2])
        {
          ret = -EIO;
          goto errout;
        }

      event->temperature = -45.0f + 175.0f * (float)raw / 65535.0f;
    }
  else
    {
      raw = (data[3] << 8) | data[4];
      if (shtc3_crc(raw) != data[5])
        {
          ret = -EIO;
          goto errout;
        }

      event->humidity = 100.0f * (float)raw / 65535.0f;
    }

  ret = sizeof(struct sensor_event_s);

errout:
  nxmutex_unlock(&priv->devlock);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: shtc3_register_uorb
 ****************************************************************************/

int shtc3_register_uorb(int devno, FAR struct i2c_master_s *i2c,
                        uint8_t addr)
{
  FAR struct shtc3_dev_s *priv;
  int ret;

  priv = kmm_zalloc(sizeof(struct shtc3_dev_s));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  priv->i2c  = i2c;
  priv->addr = addr;
  nxmutex_init(&priv->devlock);

  /* Register Temperature lower half */

  priv->temp.dev          = priv;
  priv->temp.lower.type   = SENSOR_TYPE_AMBIENT_TEMPERATURE;
  priv->temp.lower.ops    = &g_sensor_ops;
  ret = sensor_register(&priv->temp.lower, devno);
  if (ret < 0)
    {
      goto temp_err;
    }

  /* Register Humidity lower half */

  priv->humi.dev          = priv;
  priv->humi.lower.type   = SENSOR_TYPE_RELATIVE_HUMIDITY;
  priv->humi.lower.ops    = &g_sensor_ops;
  ret = sensor_register(&priv->humi.lower, devno);
  if (ret < 0)
    {
      goto humi_err;
    }

  return OK;

humi_err:
  sensor_unregister(&priv->temp.lower, devno);
temp_err:
  nxmutex_destroy(&priv->devlock);
  kmm_free(priv);
  return ret;
}

#endif /* CONFIG_I2C && CONFIG_SENSORS_SHTC3 */
