/****************************************************************************
 * drivers/sensors/bmi270_uorb.c
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

#include "bmi270_base.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CONSTANTS_ONE_G 9.8f

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum bmi270_idx_e
{
  BMI270_ACCEL_IDX = 0,
  BMI270_GYRO_IDX,
  BMI270_MAX_IDX
};

struct bmi270_sensor_s
{
  struct sensor_lowerhalf_s  lower;
  uint64_t                   last_update;
  float                      scale;
  FAR void                  *dev;
  bool                       enabled;
#ifdef CONFIG_SENSORS_BMI270_POLL
  unsigned long              interval;
#endif
  struct bmi270_dev_s        base;
};

struct bmi270_sensor_dev_s
{
  struct bmi270_sensor_s priv[BMI270_MAX_IDX];
  mutex_t                lock;
#ifdef CONFIG_SENSORS_BMI270_POLL
  sem_t                  run;
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Sensor methods */

static int bmi270_activate(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep,
                           bool enable);
static int bmi270_set_interval(FAR struct sensor_lowerhalf_s *lower,
                               FAR struct file *filep,
                               FAR unsigned long *period_us);
#ifndef CONFIG_SENSORS_BMI270_POLL
static int bmi270_fetch(FAR struct sensor_lowerhalf_s *lower,
                        FAR struct file *filep,
                        FAR char *buffer, size_t buflen);
#endif
static int bmi270_control(FAR struct sensor_lowerhalf_s *lower,
                          FAR struct file *filep,
                          int cmd, unsigned long arg);

/* Helpers */

static int bmi270_accel_scale(FAR struct bmi270_sensor_s *priv,
                              uint8_t scale);
static int bmi270_gyro_scale(FAR struct bmi270_sensor_s *priv,
                             uint16_t scale);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_sensor_ops =
{
  NULL,                 /* open */
  NULL,                 /* close */
  bmi270_activate,
  bmi270_set_interval,
  NULL,                 /* batch */
#ifdef CONFIG_SENSORS_BMI270_POLL
  NULL,                 /* fetch */
#else
  bmi270_fetch,
#endif
  NULL,                 /* selftest */
  NULL,                 /* set_calibvalue */
  NULL,                 /* calibrate */
  bmi270_control
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bmi270_activate
 ****************************************************************************/

static int bmi270_activate(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep, bool enable)
{
  FAR struct bmi270_sensor_s     *priv  = NULL;
  FAR struct bmi270_sensor_dev_s *dev   = NULL;
  bool                            start = false;
  bool                            stop  = false;
  int                             ret   = OK;
  int                             tmp   = 0;

  priv = (FAR struct bmi270_sensor_s *)lower;
  dev = priv->dev;

  nxmutex_lock(&dev->lock);

  tmp = (dev->priv[BMI270_ACCEL_IDX].enabled +
         dev->priv[BMI270_GYRO_IDX].enabled);

  if (enable && tmp == 0)
    {
      /* One time start */

      start = true;
    }
  else if (!enable && tmp == 1)
    {
      /* One time stop */

      stop = true;
    }

  priv->enabled = enable;

  nxmutex_unlock(&dev->lock);

  if (start)
    {
      /* Set normal mode */

      bmi270_set_normal_imu(&priv->base);

#ifdef CONFIG_SENSORS_BMI270_POLL
      priv->last_update = sensor_get_timestamp();

      /* Wake up the thread */

      nxsem_post(&dev->run);
#endif
    }

  else if (stop)
    {
      /* Disable acquisition of acc and gyro */

      bmi270_putreg8(&priv->base, BMI270_PWR_CTRL, 0);
      up_mdelay(30);
    }

  return ret;
}

/****************************************************************************
 * Name: bmi270_set_interval
 ****************************************************************************/

static int bmi270_set_interval(FAR struct sensor_lowerhalf_s *lower,
                               FAR struct file *filep,
                               FAR unsigned long *interval)
{
#ifdef CONFIG_SENSORS_BMI270_POLL
  FAR struct bmi270_sensor_s *priv = NULL;

  priv = (FAR struct bmi270_sensor_s *)lower;

  priv->interval = *interval;
#endif

  return OK;
}

#ifndef CONFIG_SENSORS_BMI270_POLL
/****************************************************************************
 * Name: bmi270_set_interval
 ****************************************************************************/

static int bmi270_fetch(FAR struct sensor_lowerhalf_s *lower,
                        FAR struct file *filep, FAR char *buffer,
                        size_t buflen)
{
  FAR struct bmi270_sensor_s *priv = NULL;
  int16_t                     data[3];
  int                         ret  = OK;

  priv = (FAR struct bmi270_sensor_s *)lower;

  switch (lower->type)
    {
      case SENSOR_TYPE_ACCELEROMETER:
        {
          struct sensor_accel accel;

          bmi270_getregs(&priv->base, BMI270_DATA_8,
                         (FAR uint8_t *)data, 6);

          accel.timestamp = sensor_get_timestamp();
          accel.x         = data[0] * priv->scale;
          accel.y         = data[1] * priv->scale;
          accel.z         = data[2] * priv->scale;

          memcpy(buffer, &accel, sizeof(accel));
          ret = sizeof(accel);

          break;
        }

      case SENSOR_TYPE_GYROSCOPE:
        {
          struct sensor_gyro gyro;

          bmi270_getregs(&priv->base, BMI270_DATA_14,
                         (FAR uint8_t *)data, 6);

          gyro.timestamp = sensor_get_timestamp();
          gyro.x         = data[0] * priv->scale;
          gyro.y         = data[1] * priv->scale;
          gyro.z         = data[2] * priv->scale;

          memcpy(buffer, &gyro, sizeof(gyro));
          ret = sizeof(gyro);

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
 * Name: bmi270_cotrol
 ****************************************************************************/

static int bmi270_control(FAR struct sensor_lowerhalf_s *lower,
                          FAR struct file *filep, int cmd,
                          unsigned long arg)
{
  FAR struct bmi270_sensor_s *priv = NULL;
  int                          ret  = OK;

  priv = (FAR struct bmi270_sensor_s *)lower;

  switch (cmd)
    {
      /* Set full scale command */

      case SNIOC_SET_SCALE_XL:
        {
          if (priv->lower.type == SENSOR_TYPE_GYROSCOPE)
            {
              ret = bmi270_gyro_scale(priv, arg);
            }
          else if (priv->lower.type == SENSOR_TYPE_ACCELEROMETER)
            {
              ret = bmi270_accel_scale(priv, arg);
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
 * Name: bmi270_midpoint
 *
 * Description:
 *   Find the midpoint between two numbers.
 *
 ****************************************************************************/

static uint32_t bmi270_midpoint(uint32_t a, uint32_t b)
{
  return (uint32_t)(((uint64_t)a +
                     (uint64_t)b + (uint64_t)1) / (uint64_t)2);
}

/****************************************************************************
 * Name: bmi270_accel_scale
 ****************************************************************************/

static int bmi270_accel_scale(FAR struct bmi270_sensor_s *priv,
                              uint8_t scale)
{
  int ret = OK;

  if (scale < bmi270_midpoint(2, 4))
    {
      bmi270_putreg8(&priv->base, BMI270_ACC_RANGE, ACCEL_RANGE_2G);
      priv->scale = CONSTANTS_ONE_G / 16384.f;
    }
  else if (scale < bmi270_midpoint(4, 8))
    {
      bmi270_putreg8(&priv->base, BMI270_ACC_RANGE, ACCEL_RANGE_4G);
      priv->scale = CONSTANTS_ONE_G / 8192.f;
    }
  else if (scale < bmi270_midpoint(8, 16))
    {
      bmi270_putreg8(&priv->base, BMI270_ACC_RANGE, ACCEL_RANGE_8G);
      priv->scale = CONSTANTS_ONE_G / 4096.f;
    }
  else
    {
      bmi270_putreg8(&priv->base, BMI270_ACC_RANGE, ACCEL_RANGE_16G);
      priv->scale = CONSTANTS_ONE_G / 2048.f;
    }

  return ret;
}

/****************************************************************************
 * Name: bmi270_gyro_scale
 ****************************************************************************/

static int bmi270_gyro_scale(FAR struct bmi270_sensor_s *priv,
                             uint16_t scale)
{
  int ret = OK;

  if (scale < bmi270_midpoint(125, 250))
    {
      bmi270_putreg8(&priv->base, BMI270_GYR_RANGE, GYRO_RANGE_125);
      priv->scale = (M_PI / 180.0f) * 125.f / 32768.f;
    }
  else if (scale < bmi270_midpoint(250, 500))
    {
      bmi270_putreg8(&priv->base, BMI270_GYR_RANGE, GYRO_RANGE_250);
      priv->scale = (M_PI / 180.0f) * 250.f / 32768.f;
    }
  else if (scale < bmi270_midpoint(500, 1000))
    {
      bmi270_putreg8(&priv->base, BMI270_GYR_RANGE, GYRO_RANGE_500);
      priv->scale = (M_PI / 180.0f) * 500.f / 32768.f;
    }
  else if (scale < bmi270_midpoint(1000, 2000))
    {
      bmi270_putreg8(&priv->base, BMI270_GYR_RANGE, GYRO_RANGE_1000);
      priv->scale = (M_PI / 180.0f) * 1000.f / 32768.f;
    }
  else
    {
      bmi270_putreg8(&priv->base, BMI270_GYR_RANGE, GYRO_RANGE_2000);
      priv->scale = (M_PI / 180.0f) * 2000.f / 32768.f;
    }

  return ret;
}

#ifdef CONFIG_SENSORS_BMI270_POLL
/****************************************************************************
 * Name: bmi270_accel_data
 *
 * Description:
 *   Get and push accel data from struct sensor_data_s
 *
 * Parameter:
 *   priv  - Internal private lower half driver instance
 *   buf  - Point to data
 *
 * Return:
 *   OK - on success
 *
 ****************************************************************************/

static void bmi270_accel_data(FAR struct bmi270_sensor_s *priv,
                              FAR int16_t *buf)
{
  FAR struct sensor_lowerhalf_s *lower = &priv->lower;
  struct sensor_accel            accel;
  uint64_t                       now   = sensor_get_timestamp();

  if (!priv->enabled || now - priv->last_update < priv->interval)
    {
      return;
    }

  priv->last_update = now;

  accel.timestamp   = now;
  accel.x           = buf[0] * priv->scale;
  accel.y           = buf[1] * priv->scale;
  accel.z           = buf[2] * priv->scale;
  accel.temperature = 0;

  lower->push_event(lower->priv, &accel, sizeof(accel));
}

/****************************************************************************
 * Name: bmi270_gyro_data
 *
 * Description:
 *   Get and push gyro data from struct sensor_data_s
 *
 * Parameter:
 *   priv  - Internal private lower half driver instance
 *   buf  - Point to data
 *
 * Return:
 *   OK - on success
 *
 ****************************************************************************/

static void bmi270_gyro_data(FAR struct bmi270_sensor_s *priv,
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
  gyro.x           = buf[0] * priv->scale;
  gyro.y           = buf[1] * priv->scale;
  gyro.z           = buf[2] * priv->scale;
  gyro.temperature = 0;

  lower->push_event(lower->priv, &gyro, sizeof(gyro));
}

/****************************************************************************
 * Name: bmi270_thread
 *
 * Description:
 *   Thread for performing interval measurement cycle and data read.
 *
 * Parameter:
 *   argc - Number opf arguments
 *   argv - Pointer to argument list
 *
 ****************************************************************************/

static int bmi270_thread(int argc, FAR char **argv)
{
  FAR struct bmi270_sensor_dev_s *dev
      = (FAR struct bmi270_sensor_dev_s *)((uintptr_t)strtoul(argv[1], NULL,
                                                               16));
  FAR struct bmi270_sensor_s *accel = &dev->priv[BMI270_ACCEL_IDX];
  FAR struct bmi270_sensor_s *gyro  = &dev->priv[BMI270_GYRO_IDX];
  unsigned long               min_interval;
  int16_t                     data[6];
  int                         ret;

  while (true)
    {
      if ((!accel->enabled) && (!gyro->enabled))
        {
          /* Waiting to be woken up */

          ret = nxsem_wait(&dev->run);
          if (ret < 0)
            {
              continue;
            }
        }

      /* Get data */

      bmi270_getregs(&gyro->base, BMI270_DATA_8, (uint8_t *)data, 12);

      /* Read accel */

      if (accel->enabled)
        {
          bmi270_accel_data(accel, data);
        }

      /* Read gyro */

      if (gyro->enabled)
        {
          bmi270_gyro_data(gyro, &data[3]);
        }

      /* Sleeping thread before fetching the next sensor data */

      min_interval = MIN(accel->interval, gyro->interval);
      nxsig_usleep(min_interval);
    }

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bmi270_register_uorb
 *
 * Description:
 *   Register the BMI270 IMU as sensor device
 *
 * Input Parameters:
 *   devno   - Instance number for driver
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_BMI270_I2C
int bmi270_register_uorb(int devno, FAR struct i2c_master_s *i2c,
                         uint8_t addr)
#else /* CONFIG_SENSORS_BMI270_SPI */
int bmi270_register_uorb(int devno, FAR struct spi_dev_s *spi)
#endif
{
  FAR struct bmi270_sensor_dev_s *dev = NULL;
  FAR struct bmi270_sensor_s     *tmp = NULL;
#ifdef CONFIG_SENSORS_BMI270_POLL
  FAR char                       *argv[2];
  char                            arg1[32];
#endif
  int                             ret = OK;

  /* Initialize the device structure. */

  dev = (FAR struct bmi270_sensor_dev_s *)kmm_malloc(sizeof(*dev));
  if (dev == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  memset(dev, 0, sizeof(*dev));
  nxmutex_init(&dev->lock);
#ifdef CONFIG_SENSORS_BMI270_POLL
  nxsem_init(&dev->run, 0, 0);
#endif

  /* Accelerometer register */

  tmp                = &dev->priv[BMI270_ACCEL_IDX];
  tmp->dev           = dev;
#ifdef CONFIG_SENSORS_BMI270_I2C
  tmp->base.i2c      = i2c;
  tmp->base.addr     = addr;
#else
  tmp->base.spi      = spi;
#endif
  tmp->lower.ops     = &g_sensor_ops;
  tmp->lower.type    = SENSOR_TYPE_ACCELEROMETER;
  tmp->lower.nbuffer = 1;
#ifdef CONFIG_SENSORS_BMI270_POLL
  tmp->enabled       = false;
  tmp->interval      = CONFIG_SENSORS_BMI270_POLL_INTERVAL;
#endif

  ret = sensor_register(&tmp->lower, devno);
  if (ret < 0)
    {
      snerr("sensor_register failed: %d\n", ret);
      goto gyro_err;
    }

  /* Gyroscope register */

  tmp                = &dev->priv[BMI270_GYRO_IDX];
  tmp->dev           = dev;
#ifdef CONFIG_SENSORS_BMI270_I2C
  tmp->base.i2c      = i2c;
  tmp->base.addr     = addr;
#else
  tmp->base.spi      = spi;
#endif
  tmp->lower.ops     = &g_sensor_ops;
  tmp->lower.type    = SENSOR_TYPE_GYROSCOPE;
  tmp->lower.nbuffer = 1;
#ifdef CONFIG_SENSORS_BMI270_POLL
  tmp->enabled       = false;
  tmp->interval      = CONFIG_SENSORS_BMI270_POLL_INTERVAL;
#endif

  ret = sensor_register(&tmp->lower, devno);
  if (ret < 0)
    {
      snerr("sensor_register failed: %d\n", ret);
      goto gyro_err;
    }

#ifdef CONFIG_SENSORS_BMI270_SPI
  /* BMI270 detects communication bus is SPI by rising edge of CS. */

  bmi270_getreg8(&tmp->base, 0x00);
  bmi270_getreg8(&tmp->base, 0x00);
  up_udelay(200);
#endif

  /* Initialization sequence */

  ret = bmi270_init_seq(&tmp->base);
  if (ret != 0)
    {
      return ret;
    }

  /* Set default scale */

  bmi270_accel_scale(&dev->priv[BMI270_ACCEL_IDX], 2);
  bmi270_gyro_scale(&dev->priv[BMI270_GYRO_IDX], 2000);

#ifdef CONFIG_SENSORS_BMI270_POLL
  /* Create thread for polling sensor data */

  snprintf(arg1, 16, "%p", dev);
  argv[0] = arg1;
  argv[1] = NULL;

  ret = kthread_create("bmi270_thread", SCHED_PRIORITY_DEFAULT,
                       CONFIG_SENSORS_BMI270_THREAD_STACKSIZE,
                       bmi270_thread,
                       argv);
  if (ret < 0)
    {
      goto thr_err;
    }
#endif

  return ret;

#ifdef CONFIG_SENSORS_BMI270_POLL
thr_err:
#endif
#ifdef AUX_MAG_SUPPORTED
  sensor_unregister(&dev->priv[BMI270_MAG_IDX].lower, devno);
mag_err:
#endif
  sensor_unregister(&dev->priv[BMI270_GYRO_IDX].lower, devno);
gyro_err:
  sensor_unregister(&dev->priv[BMI270_ACCEL_IDX].lower, devno);

  kmm_free(dev);

  return ret;
}
