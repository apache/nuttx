/****************************************************************************
 * drivers/sensors/lsm9ds1_uorb.c
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

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <limits.h>
#include <fcntl.h>
#include <inttypes.h>
#include <sys/param.h>

#include <nuttx/mutex.h>
#include <nuttx/signal.h>
#include <nuttx/compiler.h>
#include <nuttx/nuttx.h>
#include <nuttx/kthread.h>

#include <nuttx/sensors/sensor.h>
#include <nuttx/sensors/ioctl.h>

#include "lsm9ds1_base.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CONSTANTS_ONE_G 9.8f

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum lsm9ds1_idx_e
{
  LSM9DS1_ACCEL_IDX = 0,
  LSM9DS1_GYRO_IDX,
  LSM9DS1_MAG_IDX,
  LSM9DS1_MAX_IDX
};

struct lsm9ds1_sensor_s
{
  struct sensor_lowerhalf_s  lower;
  uint64_t                   last_update;
  float                      scale;
  FAR void                  *dev;
#ifdef CONFIG_SENSORS_LSM9DS1_POLL
  bool                       enabled;
  uint32_t                   interval;
#endif
  struct lsm9ds1_dev_s       base;
};

struct lsm9ds1_sensor_dev_s
{
  struct lsm9ds1_sensor_s priv[3];
  mutex_t                 lock;
#ifdef CONFIG_SENSORS_LSM9DS1_POLL
  sem_t                   run;
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Sensor methods */

static int lsm9ds1_activate(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep,
                           bool enable);
static int lsm9ds1_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                FAR struct file *filep,
                                FAR uint32_t *period_us);
#ifndef CONFIG_SENSORS_LSM9DS1_POLL
static int lsm9ds1_fetch(FAR struct sensor_lowerhalf_s *lower,
                         FAR struct file *filep,
                         FAR char *buffer, size_t buflen);
#endif
static int lsm9ds1_control(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep,
                           int cmd, unsigned long arg);

/* Helpers */

static int lsm9ds1_mag_scale(FAR struct lsm9ds1_sensor_s *priv,
                             uint8_t scale);
static int lsm9ds1_accel_scale(FAR struct lsm9ds1_sensor_s *priv,
                               uint8_t scale);
static int lsm9ds1_gyro_scale(FAR struct lsm9ds1_sensor_s *priv,
                              uint8_t scale);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_sensor_ops =
{
  NULL,                 /* open */
  NULL,                 /* close */
  .activate     = lsm9ds1_activate,
  .set_interval = lsm9ds1_set_interval,
  NULL,                 /* batch */
#ifdef CONFIG_SENSORS_LSM9DS1_POLL
  NULL,                 /* fetch */
#else
  .fetch        = lsm9ds1_fetch,
#endif
  NULL,                 /* flush */
  NULL,                 /* selftest */
  NULL,                 /* set_calibvalue */
  NULL,                 /* calibrate */
  NULL,                 /* get_info */
  .control      = lsm9ds1_control
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lsm9ds1_activate
 ****************************************************************************/

static int lsm9ds1_activate(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep, bool enable)
{
#ifdef CONFIG_SENSORS_LSM9DS1_POLL
  FAR struct lsm9ds1_sensor_s     *priv         = NULL;
  FAR struct lsm9ds1_sensor_dev_s *dev          = NULL;
  bool                             start_thread = false;
  int                              ret          = OK;

  priv = container_of(lower, struct lsm9ds1_sensor_s, lower);
  dev = priv->dev;

  if (enable)
    {
      if (!priv->enabled)
        {
          start_thread      = true;
          priv->last_update = sensor_get_timestamp();
        }

      ret = priv->base.ops->start(&priv->base);
    }
  else
    {
      ret = priv->base.ops->stop(&priv->base);
    }

  priv->enabled = enable;

  if (start_thread)
    {
      /* Wake up the thread */

      nxsem_post(&dev->run);
    }

  return ret;
#else
  return OK;
#endif
}

/****************************************************************************
 * Name: lsm9ds1_set_interval
 ****************************************************************************/

static int lsm9ds1_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                FAR struct file *filep,
                                FAR uint32_t *interval)
{
#ifdef CONFIG_SENSORS_LSM9DS1_POLL
  FAR struct lsm9ds1_sensor_s *priv = NULL;

  priv = container_of(lower, struct lsm9ds1_sensor_s, lower);

  priv->interval = *interval;
#endif

  return OK;
}

/****************************************************************************
 * Name: lsm9ds1_data
 ****************************************************************************/

static int16_t lsm9ds1_data(int16_t data)
{
  /* The value is positive */

  if (data < 0x8000)
    {
      data = data;
    }

  /* The value is negative, so find its absolute value by taking the
   * two's complement
   */

  else if (data > 0x8000)
    {
      data = -(~data + 1);
    }

  /* The value is negative and can't be represented as a positive
   * int16_t value
   */

  else
    {
      data = -32768;
    }

  return data;
}

#ifndef CONFIG_SENSORS_LSM9DS1_POLL
/****************************************************************************
 * Name: lsm9ds1_fetch
 ****************************************************************************/

static int lsm9ds1_fetch(FAR struct sensor_lowerhalf_s *lower,
                         FAR struct file *filep, FAR char *buffer,
                         size_t buflen)
{
  FAR struct lsm9ds1_sensor_s *priv = NULL;
  int16_t                      data[3];
  int                          ret  = OK;

  priv = container_of(lower, struct lsm9ds1_sensor_s, lower);

  switch (lower->type)
    {
      case SENSOR_TYPE_ACCELEROMETER:
        {
          struct sensor_accel accel;

          ret = lsm9ds1_readreg(&priv->base, LSM9DS1_OUT_X_L_XL,
                                (FAR uint8_t *)data, 6);

          accel.timestamp = sensor_get_timestamp();
          accel.x         = (int16_t)lsm9ds1_data(data[0]) * priv->scale;
          accel.y         = (int16_t)lsm9ds1_data(data[1]) * priv->scale;
          accel.z         = (int16_t)lsm9ds1_data(data[2]) * priv->scale;

          memcpy(buffer, &accel, sizeof(accel));

          break;
        }

      case SENSOR_TYPE_GYROSCOPE:
        {
          struct sensor_gyro gyro;

          ret = lsm9ds1_readreg(&priv->base, LSM9DS1_OUT_X_L_G,
                                (FAR uint8_t *)data, 6);

          gyro.timestamp = sensor_get_timestamp();
          gyro.x         = (int16_t)lsm9ds1_data(data[0]) * priv->scale;
          gyro.y         = (int16_t)lsm9ds1_data(data[1]) * priv->scale;
          gyro.z         = (int16_t)lsm9ds1_data(data[2]) * priv->scale;

          memcpy(buffer, &gyro, sizeof(gyro));

          break;
        }

      case SENSOR_TYPE_MAGNETIC_FIELD:
        {
          struct sensor_mag mag;

          ret = lsm9ds1_readreg(&priv->base, LSM9DS1_OUT_X_L_M,
                                (FAR uint8_t *)data, 6);

          mag.timestamp = sensor_get_timestamp();
          mag.x         = (int16_t)lsm9ds1_data(data[0]) * priv->scale;
          mag.y         = (int16_t)lsm9ds1_data(data[1]) * priv->scale;
          mag.z         = (int16_t)lsm9ds1_data(data[2]) * priv->scale;

          memcpy(buffer, &mag, sizeof(mag));

          break;
        }

      default:
        {
          ret = -EINVAL;
          break;
        }
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: lsm9ds1_cotrol
 ****************************************************************************/

static int lsm9ds1_control(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep, int cmd,
                           unsigned long arg)
{
  FAR struct lsm9ds1_sensor_s *priv = NULL;
  int                          ret  = OK;

  priv = container_of(lower, struct lsm9ds1_sensor_s, lower);

  switch (cmd)
    {
        /* Set full scale command */

      case SNIOC_SET_SCALE_XL:
        {
          if (priv->lower.type == SENSOR_TYPE_GYROSCOPE)
            {
              ret = lsm9ds1_gyro_scale(priv, arg);
            }
          else if (priv->lower.type == SENSOR_TYPE_ACCELEROMETER)
            {
              ret = lsm9ds1_accel_scale(priv, arg);
            }
          else if (priv->lower.type == SENSOR_TYPE_MAGNETIC_FIELD)
            {
              ret = lsm9ds1_mag_scale(priv, arg);
            }

          break;
        }

      default:
        {
          snerr("ERROR: Unrecognized cmd: %d\n", cmd);
          ret = -ENOTTY;
          break;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: lsm9ds1_mag_scale
 ****************************************************************************/

static int lsm9ds1_mag_scale(FAR struct lsm9ds1_sensor_s *priv,
                             uint8_t scale)
{
  int ret = OK;

  ret = priv->base.ops->setfullscale(&priv->base, scale);
  if (ret < 0)
    {
      return ret;
    }

  if (scale < lsm9ds1_midpoint(4, 8))
    {
      priv->scale = 8.f / 65536.f;
    }
  else if (scale < lsm9ds1_midpoint(8, 12))
    {
      priv->scale = 16.f / 65536.f;
    }
  else if (scale < lsm9ds1_midpoint(12, 16))
    {
      priv->scale = 24.f / 65536.f;
    }
  else
    {
      priv->scale = 32.f / 65536.f;
    }

  return ret;
}

/****************************************************************************
 * Name: lsm9ds1_accel_scale
 ****************************************************************************/

static int lsm9ds1_accel_scale(FAR struct lsm9ds1_sensor_s *priv,
                               uint8_t scale)
{
  int ret = OK;

  ret = priv->base.ops->setfullscale(&priv->base, scale);
  if (ret < 0)
    {
      return ret;
    }

  if (scale < lsm9ds1_midpoint(2, 4))
    {
      priv->scale = CONSTANTS_ONE_G / 16384.f;
    }
  else if (scale < lsm9ds1_midpoint(4, 8))
    {
      priv->scale = CONSTANTS_ONE_G / 8192.f;
    }
  else if (scale < lsm9ds1_midpoint(8, 16))
    {
      priv->scale = CONSTANTS_ONE_G / 4096.f;
    }
  else
    {
      priv->scale = CONSTANTS_ONE_G / 2048.f;
    }

  return ret;
}

/****************************************************************************
 * Name: lsm9ds1_gyro_scale
 ****************************************************************************/

static int lsm9ds1_gyro_scale(FAR struct lsm9ds1_sensor_s *priv,
                              uint8_t scale)
{
  int ret = OK;

  ret = priv->base.ops->setfullscale(&priv->base, scale);
  if (ret < 0)
    {
      return ret;
    }

  if (scale < lsm9ds1_midpoint(245, 500))
    {
      priv->scale = (M_PI / 180.0f) * 245.f / 32768.f;
    }
  else if (scale < lsm9ds1_midpoint(500, 2000))
    {
      priv->scale = (M_PI / 180.0f) * 500.f / 32768.f;
    }
  else
    {
      priv->scale = (M_PI / 180.0f) * 2000.f / 32768.f;
    }

  return ret;
}

#ifdef CONFIG_SENSORS_LSM9DS1_POLL
/****************************************************************************
 * Name: lsm9ds1_accel_data
 *
 * Description: get and push accel data from struct sensor_data_s
 *
 * Parameter:
 *   priv  - Internal private lower half driver instance
 *   buf  - Point to data
 *
 * Return:
 *   OK - on success
 *
 ****************************************************************************/

static void lsm9ds1_accel_data(FAR struct lsm9ds1_sensor_s *priv,
                               FAR int16_t *buf)
{
  FAR struct sensor_lowerhalf_s *lower = &priv->lower;
  struct sensor_accel accel;
  uint64_t now                         = sensor_get_timestamp();

  if (!priv->enabled || now - priv->last_update < priv->interval)
    {
      return;
    }

  priv->last_update = now;

  accel.timestamp   = now;
  accel.x           = (int16_t)lsm9ds1_data(buf[0]) * priv->scale;
  accel.y           = (int16_t)lsm9ds1_data(buf[1]) * priv->scale;
  accel.z           = (int16_t)lsm9ds1_data(buf[2]) * priv->scale;
  accel.temperature = 0;

  lower->push_event(lower->priv, &accel, sizeof(accel));
}

/****************************************************************************
 * Name: lsm9ds1_gyro_data
 *
 * Description: get and push gyro data from struct sensor_data_s
 *
 * Parameter:
 *   priv  - Internal private lower half driver instance
 *   buf  - Point to data
 *
 * Return:
 *   OK - on success
 *
 ****************************************************************************/

static void lsm9ds1_gyro_data(FAR struct lsm9ds1_sensor_s *priv,
                              FAR int16_t *buf)
{
  FAR struct sensor_lowerhalf_s *lower = &priv->lower;
  struct sensor_gyro             gyro;
  uint64_t                       now   = sensor_get_timestamp();

  if (!priv->enabled || now - priv->last_update < priv->interval)
    {
      return;
    }

  priv->last_update = now;

  gyro.timestamp   = now;
  gyro.x           = (int16_t)lsm9ds1_data(buf[0]) * priv->scale;
  gyro.y           = (int16_t)lsm9ds1_data(buf[1]) * priv->scale;
  gyro.z           = (int16_t)lsm9ds1_data(buf[2]) * priv->scale;
  gyro.temperature = 0;

  lower->push_event(lower->priv, &gyro, sizeof(gyro));
}

/****************************************************************************
 * Name: lsm9ds1_mag_data
 *
 * Description: get and push magnetometer data from struct sensor_data_s
 *
 * Parameter:
 *   priv  - Internal private lower half driver instance
 *   buf  - Point to data
 *
 * Return:
 *   OK - on success
 *
 ****************************************************************************/

static void lsm9ds1_mag_data(FAR struct lsm9ds1_sensor_s *priv,
                             FAR int16_t *buf)
{
  FAR struct sensor_lowerhalf_s *lower = &priv->lower;
  struct sensor_mag              mag;
  uint64_t                       now   = sensor_get_timestamp();

  if (!priv->enabled || now - priv->last_update < priv->interval)
    {
      return;
    }

  priv->last_update = now;

  mag.timestamp   = now;
  mag.x           = (int16_t)lsm9ds1_data(buf[0]) * priv->scale;
  mag.y           = (int16_t)lsm9ds1_data(buf[1]) * priv->scale;
  mag.z           = (int16_t)lsm9ds1_data(buf[2]) * priv->scale;
  mag.temperature = 0;

  lower->push_event(lower->priv, &mag, sizeof(mag));
}

/****************************************************************************
 * Name: lsm9ds1_thread
 *
 * Description: Thread for performing interval measurement cycle and data
 *              read.
 *
 * Parameter:
 *   argc - Number opf arguments
 *   argv - Pointer to argument list
 *
 ****************************************************************************/

static int lsm9ds1_thread(int argc, FAR char **argv)
{
  FAR struct lsm9ds1_sensor_dev_s *dev
      = (FAR struct lsm9ds1_sensor_dev_s *)((uintptr_t)strtoul(argv[1], NULL,
                                                               16));
  FAR struct lsm9ds1_sensor_s *accel = &dev->priv[LSM9DS1_ACCEL_IDX];
  FAR struct lsm9ds1_sensor_s *gyro  = &dev->priv[LSM9DS1_GYRO_IDX];
  FAR struct lsm9ds1_sensor_s *mag   = &dev->priv[LSM9DS1_MAG_IDX];
  unsigned long                min_interval;
  int16_t                      adata[3];
  int16_t                      gdata[3];
  int16_t                      mdata[3];
  int                          ret;

  while (true)
    {
      if ((!accel->enabled) && (!gyro->enabled) && (!mag->enabled))
        {
          /* Waiting to be woken up */

          ret = nxsem_wait(&dev->run);
          if (ret < 0)
            {
              continue;
            }
        }

      /* Read accel */

      if (accel->enabled)
        {
          ret = lsm9ds1_readreg(&accel->base,
                                LSM9DS1_OUT_X_L_XL, (FAR uint8_t *)adata, 6);
          lsm9ds1_accel_data(accel, adata);
        }

      /* Read gyro */

      if (gyro->enabled)
        {
          ret = lsm9ds1_readreg(&gyro->base,
                                LSM9DS1_OUT_X_L_G, (FAR uint8_t *)gdata, 6);
          lsm9ds1_gyro_data(gyro, gdata);
        }

      /* Read mag */

      if (mag->enabled)
        {
          ret = lsm9ds1_readreg(&mag->base,
                                LSM9DS1_OUT_X_L_M, (FAR uint8_t *)mdata, 6);
          lsm9ds1_mag_data(mag, mdata);
        }

      /* Sleeping thread before fetching the next sensor data */

      min_interval = MIN(accel->interval, gyro->interval);
      min_interval = MIN(min_interval, mag->interval);
      nxsig_usleep(min_interval);
    }

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lsm9ds1_register_uorb
 *
 * Description:
 *   Register the LSM9DS1 IMU as sensor device
 *
 * Input Parameters:
 *   devno   - Instance number for driver
 *   config  - configuratio
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int lsm9ds1_register_uorb(int devno, FAR struct lsm9ds1_config_s *config)
{
  FAR struct lsm9ds1_sensor_dev_s *dev = NULL;
  FAR struct lsm9ds1_sensor_s     *tmp = NULL;
#ifdef CONFIG_SENSORS_LSM9DS1_POLL
  FAR char                        *argv[2];
  char                             arg1[32];
#endif
  int                              ret = OK;

  /* Initialize the device structure. */

  dev = (FAR struct lsm9ds1_sensor_dev_s *)kmm_malloc(sizeof(*dev));
  if (dev == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  memset(dev, 0, sizeof(*dev));
  nxmutex_init(&dev->lock);
#ifdef CONFIG_SENSORS_LSM9DS1_POLL
  nxsem_init(&dev->run, 0, 0);
#endif

  /* Accelerometer register */

  tmp                = &dev->priv[LSM9DS1_ACCEL_IDX];
  tmp->dev           = dev;
  tmp->base.ops      = &g_lsm9ds1accel_ops;
  tmp->base.i2c      = config->i2c;
  tmp->base.addr     = config->addr_acc;
  tmp->lower.ops     = &g_sensor_ops;
  tmp->lower.type    = SENSOR_TYPE_ACCELEROMETER;
  tmp->lower.nbuffer = 1;
#ifdef CONFIG_SENSORS_LSM9DS1_POLL
  tmp->enabled       = false;
  tmp->interval      = CONFIG_SENSORS_LSM9DS1_POLL_INTERVAL;
#endif

  ret = sensor_register(&tmp->lower, devno);
  if (ret < 0)
    {
      snerr("sensor_register failed: %d\n", ret);
      goto gyro_err;
    }

  lsm9ds1_accel_scale(tmp, 2);

  /* Gyroscope register */

  tmp                = &dev->priv[LSM9DS1_GYRO_IDX];
  tmp->dev           = dev;
  tmp->base.ops      = &g_lsm9ds1gyro_ops;
  tmp->base.i2c      = config->i2c;
  tmp->base.addr     = config->addr_gyro;
  tmp->lower.ops     = &g_sensor_ops;
  tmp->lower.type    = SENSOR_TYPE_GYROSCOPE;
  tmp->lower.nbuffer = 1;
#ifdef CONFIG_SENSORS_LSM9DS1_POLL
  tmp->enabled       = false;
  tmp->interval      = CONFIG_SENSORS_LSM9DS1_POLL_INTERVAL;
#endif

  ret = sensor_register(&tmp->lower, devno);
  if (ret < 0)
    {
      snerr("sensor_register failed: %d\n", ret);
      goto gyro_err;
    }

  lsm9ds1_gyro_scale(tmp, 245);

  /* Magnetic register */

  tmp                = &dev->priv[LSM9DS1_MAG_IDX];
  tmp->dev           = dev;
  tmp->base.ops      = &g_lsm9ds1mag_ops;
  tmp->base.i2c      = config->i2c;
  tmp->base.addr     = config->addr_mag;
  tmp->lower.ops     = &g_sensor_ops;
  tmp->lower.type    = SENSOR_TYPE_MAGNETIC_FIELD;
  tmp->lower.nbuffer = 1;
#ifdef CONFIG_SENSORS_LSM9DS1_POLL
  tmp->enabled       = false;
  tmp->interval      = CONFIG_SENSORS_LSM9DS1_POLL_INTERVAL;
#endif

  ret = sensor_register(&tmp->lower, devno);
  if (ret < 0)
    {
      snerr("sensor_register failed: %d\n", ret);
      goto mag_err;
    }

  lsm9ds1_mag_scale(tmp, 4);

#ifdef CONFIG_SENSORS_LSM9DS1_POLL
  /* Create thread for polling sensor data */

  snprintf(arg1, 16, "%p", dev);
  argv[0] = arg1;
  argv[1] = NULL;

  ret = kthread_create("lsm9ds1_thread", SCHED_PRIORITY_DEFAULT,
                       CONFIG_SENSORS_LSM9DS1_THREAD_STACKSIZE,
                       lsm9ds1_thread,
                       argv);
  if (ret < 0)
    {
      goto thr_err;
    }
#endif

  return ret;

#ifdef CONFIG_SENSORS_LSM9DS1_POLL
thr_err:
#endif
  sensor_unregister(&dev->priv[LSM9DS1_MAG_IDX].lower, devno);
mag_err:
  sensor_unregister(&dev->priv[LSM9DS1_GYRO_IDX].lower, devno);
gyro_err:
  sensor_unregister(&dev->priv[LSM9DS1_ACCEL_IDX].lower, devno);

  kmm_free(dev);

  return ret;
}
