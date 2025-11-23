/****************************************************************************
 * drivers/sensors/qmi8658_uorb.c
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
#include <nuttx/mutex.h>
#include <nuttx/signal.h>
#include <nuttx/compiler.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/sensors/qmi8658.h>
#include <debug.h>

#include "qmi8658_base.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Scale factors are defined in nuttx/sensors/qmi8658.h */

/* Sensor indices */

enum qmi8658_idx_e
{
  QMI8658_ACCEL_IDX = 0,
  QMI8658_GYRO_IDX,
  QMI8658_MAX_IDX
};

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct qmi8658_sensor_s
{
  struct sensor_lowerhalf_s lower;
#ifdef CONFIG_SENSORS_QMI8658_POLL
  struct work_s work;
  uint64_t last_update;
  uint32_t interval;
#endif
  float scale;
  FAR struct qmi8658_dev_s *dev;
  bool enabled;
};

struct qmi8658_uorb_dev_s
{
  struct qmi8658_dev_s base;
  struct qmi8658_sensor_s priv[QMI8658_MAX_IDX];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Sensor methods */

static int qmi8658_activate(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep, bool enable);
#ifdef CONFIG_SENSORS_QMI8658_POLL
static int qmi8658_set_interval(FAR struct sensor_lowerhalf_s *lower,
                               FAR struct file *filep,
                               FAR uint32_t *period_us);
#else
static int qmi8658_fetch(FAR struct sensor_lowerhalf_s *lower,
                        FAR struct file *filep,
                        FAR char *buffer, size_t buflen);
#endif

#ifdef CONFIG_SENSORS_QMI8658_POLL
static void qmi8658_accel_worker(FAR void *arg);
static void qmi8658_gyro_worker(FAR void *arg);
#endif

static int qmi8658_read_imu(FAR struct qmi8658_dev_s *dev,
                            FAR struct sensor_accel *accel,
                            FAR struct sensor_gyro *gyro);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_sensor_ops =
{
  NULL,                   /* open */
  NULL,                   /* close */
  qmi8658_activate,       /* activate */
#ifdef CONFIG_SENSORS_QMI8658_POLL
  qmi8658_set_interval,   /* set_interval */
#else
  NULL,                   /* set_interval */
#endif
  NULL,                   /* batch */
#ifdef CONFIG_SENSORS_QMI8658_POLL
  NULL,                   /* fetch */
#else
  qmi8658_fetch,          /* fetch */
#endif
  NULL,                   /* flush */
  NULL,                   /* selftest */
  NULL,                   /* set_calibvalue */
  NULL,                   /* calibrate */
  NULL,                   /* get_info */
  NULL,                   /* control */
};

/****************************************************************************
 * Private Function
 ****************************************************************************/

/****************************************************************************
 * Name: qmi8658_read_imu
 *
 * Description:
 *   Read accelerometer and/or gyroscope data from the QMI8658 sensor.
 *   This function reads raw sensor data and applies appropriate scaling
 *   factors to convert to physical units. Temperature data is also
 *   read for sensor compensation.
 *
 * Input Parameters:
 *   dev   - Pointer to the QMI8658 device structure
 *   accel - Pointer to accelerometer data structure (NULL if not needed)
 *   gyro  - Pointer to gyroscope data structure (NULL if not needed)
 *
 * Returned Value:
 *   OK on success; negated errno on failure
 *
 *   -EINVAL - Invalid device pointer
 *   -EIO    - I2C communication failure
 *
 * Assumptions:
 *   The device must be properly initialized before calling this function.
 *   Either accel or gyro (or both) must be non-NULL.
 *
 ****************************************************************************/

static int qmi8658_read_imu(FAR struct qmi8658_dev_s *dev,
                             FAR struct sensor_accel *accel,
                             FAR struct sensor_gyro *gyro)
{
  struct qmi8658_scale_factors_s scale_factors;
  int16_t accel_data[3];
  int16_t gyro_data[3];
  int16_t temperature;
  int ret;

  if (!dev)
    {
      return -EINVAL;
    }

  ret = qmi8658_get_scale_factors(dev, &scale_factors);
  if (ret < 0)
    {
      return ret;
    }

  /* Read temperature data (shared by both accel and gyro) */

  ret = qmi8658_read_temp(dev, &temperature);
  if (ret < 0)
    {
      return ret;
    }

  if (accel)
    {
      /* Read accelerometer data */

      ret = qmi8658_read_accel(dev, accel_data);
      if (ret < 0)
        {
          return ret;
        }

      accel->x = (float)accel_data[0] / scale_factors.acc_scale;
      accel->y = (float)accel_data[1] / scale_factors.acc_scale;
      accel->z = (float)accel_data[2] / scale_factors.acc_scale;
      accel->temperature = (float)temperature / scale_factors.temp_scale;
      accel->timestamp = sensor_get_timestamp();
    }

  if (gyro)
    {
      /* Read gyroscope data */

      ret = qmi8658_read_gyro(dev, gyro_data);
      if (ret < 0)
        {
          return ret;
        }

      gyro->x = (float)gyro_data[0] / scale_factors.gyro_scale;
      gyro->y = (float)gyro_data[1] / scale_factors.gyro_scale;
      gyro->z = (float)gyro_data[2] / scale_factors.gyro_scale;
      gyro->temperature = (float)temperature / scale_factors.temp_scale;
      gyro->timestamp = sensor_get_timestamp();
    }

  return OK;
}

/****************************************************************************
 * Name: qmi8658_activate
 *
 * Description:
 *   Enable or disable the sensor and manage polling work if enabled.
 *   When enabling a sensor in polling mode, this function starts a
 *   periodic work queue job to read sensor data at the configured
 *   interval. When disabling, it cancels any pending work.
 *
 * Input Parameters:
 *   lower - Pointer to the sensor lower-half structure
 *   filep - Pointer to the file structure (unused)
 *   enable - true to enable sensor, false to disable
 *
 * Returned Value:
 *   OK on success; negated errno on failure
 *
 *   -EINVAL - Invalid pointer parameters
 *   -EIO    - Work queue operation failed
 *
 * Assumptions:
 *   This function is called from the sensor framework when applications
 *   open/close the sensor device.
 *
 *   In polling mode, the work queue must be available for scheduling
 *   periodic sensor reads.
 *
 ****************************************************************************/

static int qmi8658_activate(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep, bool enable)
{
  FAR struct qmi8658_sensor_s *priv = (FAR struct qmi8658_sensor_s *)lower;
  FAR struct qmi8658_uorb_dev_s *dev =
    (FAR struct qmi8658_uorb_dev_s *)priv->dev;
  int ret = OK;

  if (!priv || !dev)
    {
      return -EINVAL;
    }

  int sensor_idx = priv - &dev->priv[0];

  priv->enabled = enable;

#ifdef CONFIG_SENSORS_QMI8658_POLL
  if (enable)
    {
      if (priv->interval > 0)
        {
          uint32_t delay = priv->interval / USEC_PER_TICK;
          if (sensor_idx == QMI8658_ACCEL_IDX)
            {
              ret = work_queue(HPWORK, &priv->work, qmi8658_accel_worker,
                               priv, delay);
            }
          else if (sensor_idx == QMI8658_GYRO_IDX)
            {
              ret = work_queue(HPWORK, &priv->work, qmi8658_gyro_worker,
                               priv, delay);
            }
        }
    }
  else
    {
      work_cancel(HPWORK, &priv->work);
    }
#endif

  return ret;
}

#ifdef CONFIG_SENSORS_QMI8658_POLL
/****************************************************************************
 * Name: qmi8658_set_interval
 *
 * Description:
 *   Set the polling interval for sensor data acquisition.
 *   This function updates the interval at which sensor data will be
 *   read when polling mode is enabled. The interval is specified
 *   in microseconds.
 *
 * Input Parameters:
 *   lower     - Pointer to the sensor lower-half structure
 *   filep     - Pointer to the file structure (unused)
 *   period_us - Pointer to the polling period in microseconds
 *
 * Returned Value:
 *   OK on success; negated errno on failure
 *
 *   -EINVAL - Invalid pointer parameters
 *
 * Assumptions:
 *   This function is called from the sensor framework when applications
 *   set the sensor polling interval via IOCTL or similar interface.
 *
 *   The new interval takes effect immediately for the next polling cycle.
 *
 *   interval_us should be greater than 0 for meaningful operation.
 *
 *   CONFIG_SENSORS_QMI8658_POLL must be enabled for this function
 *   to be available.
 *
 ****************************************************************************/

static int qmi8658_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                FAR struct file *filep,
                                FAR uint32_t *period_us)
{
  FAR struct qmi8658_sensor_s *priv = (FAR struct qmi8658_sensor_s *)lower;

  if (!priv || !period_us)
    {
      return -EINVAL;
    }

  priv->interval = *period_us;

  return OK;
}
#endif

#ifndef CONFIG_SENSORS_QMI8658_POLL
/****************************************************************************
 * Name: qmi8658_fetch
 *
 * Description:
 *   Fetch sensor data when polling mode is disabled.
 *   This function reads accelerometer or gyroscope data from the
 *   QMI8658 sensor and returns it to the caller. The function
 *   determines which sensor data to read based on the sensor index.
 *
 * Input Parameters:
 *   lower  - Pointer to the sensor lower-half structure
 *   filep  - Pointer to the file structure (unused)
 *   buffer - Buffer to store the sensor data
 *   buflen - Size of the buffer in bytes
 *
 * Returned Value:
 *   Number of bytes read on success; negated errno on failure
 *
 *   -EINVAL - Invalid pointer parameters or insufficient buffer size
 *   -EIO    - I2C communication failure
 *
 *   For accelerometer: sizeof(struct sensor_accel) bytes
 *   For gyroscope: sizeof(struct sensor_gyro) bytes
 *
 * Assumptions:
 *   This function is called when polling mode is not enabled.
 *   The application is responsible for providing a sufficiently
 *   large buffer to hold the sensor data structure.
 *
 *   The function detects the sensor type based on the sensor index
 *   and reads the appropriate data (accelerometer or gyroscope).
 *
 *   This is a blocking read operation that communicates with the
 *   hardware via I2C.
 *
 *   CONFIG_SENSORS_QMI8658_POLL must be disabled for this function
 *   to be available.
 *
 ****************************************************************************/

static int qmi8658_fetch(FAR struct sensor_lowerhalf_s *lower,
                        FAR struct file *filep,
                        FAR char *buffer, size_t buflen)
{
  FAR struct qmi8658_sensor_s *priv = (FAR struct qmi8658_sensor_s *)lower;
  FAR struct qmi8658_uorb_dev_s *dev =
    (FAR struct qmi8658_uorb_dev_s *)priv->dev;
  int ret;

  if (!priv || !dev || !buffer)
    {
      return -EINVAL;
    }

  int sensor_idx = priv - &dev->priv[0];

  if (sensor_idx == QMI8658_ACCEL_IDX)
    {
      struct sensor_accel accel;
      ret = qmi8658_read_imu(&dev->base, &accel, NULL);
      if (ret < 0)
        {
          return ret;
        }

      if (buflen < sizeof(accel))
        {
          return -EINVAL;
        }

      memcpy(buffer, &accel, sizeof(accel));
      return sizeof(accel);
    }
  else if (sensor_idx == QMI8658_GYRO_IDX)
    {
      struct sensor_gyro gyro;
      ret = qmi8658_read_imu(&dev->base, NULL, &gyro);
      if (ret < 0)
        {
          return ret;
        }

      if (buflen < sizeof(gyro))
        {
          return -EINVAL;
        }

      memcpy(buffer, &gyro, sizeof(gyro));
      return sizeof(gyro);
    }

  return -EINVAL;
}
#endif

#ifdef CONFIG_SENSORS_QMI8658_POLL
/****************************************************************************
 * Name: qmi8658_accel_worker
 *
 * Description:
 *   Worker function for accelerometer data polling. This function is
 *   scheduled by the work queue to periodically read accelerometer
 *   data from the QMI8658 sensor and push it to the sensor framework.
 *
 * Input Parameters:
 *   arg - Pointer to the qmi8658_sensor_s structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The sensor is enabled and has a valid polling interval.
 *   The work queue is available for rescheduling.
 *
 ****************************************************************************/

static void qmi8658_accel_worker(FAR void *arg)
{
  FAR struct qmi8658_sensor_s *priv = (FAR struct qmi8658_sensor_s *)arg;
  FAR struct qmi8658_uorb_dev_s *dev =
    (FAR struct qmi8658_uorb_dev_s *)priv->dev;
  struct sensor_accel accel;
  int ret;

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(dev != NULL);

  if (priv->enabled && priv->interval > 0)
    {
      uint32_t delay = priv->interval / USEC_PER_TICK;
      work_queue(HPWORK, &priv->work, qmi8658_accel_worker, priv, delay);
    }

  ret = qmi8658_read_imu(&dev->base, &accel, NULL);
  if (ret < 0)
    {
      return;
    }

  priv->last_update = accel.timestamp;

  if (priv->lower.push_event && priv->lower.priv)
    {
      priv->lower.push_event(priv->lower.priv, &accel, sizeof(accel));
    }
}
#endif

#ifdef CONFIG_SENSORS_QMI8658_POLL
/****************************************************************************
 * Name: qmi8658_gyro_worker
 *
 * Description:
 *   Worker function for gyroscope data polling. This function is
 *   scheduled by the work queue to periodically read gyroscope
 *   data from the QMI8658 sensor and push it to the sensor framework.
 *
 * Input Parameters:
 *   arg - Pointer to the qmi8658_sensor_s structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The sensor is enabled and has a valid polling interval.
 *   The work queue is available for rescheduling.
 *
 ****************************************************************************/

static void qmi8658_gyro_worker(FAR void *arg)
{
  FAR struct qmi8658_sensor_s *priv = (FAR struct qmi8658_sensor_s *)arg;
  FAR struct qmi8658_uorb_dev_s *dev =
    (FAR struct qmi8658_uorb_dev_s *)priv->dev;
  struct sensor_gyro gyro;
  int ret;

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(dev != NULL);

  if (priv->enabled && priv->interval > 0)
    {
      uint32_t delay = priv->interval / USEC_PER_TICK;
      work_queue(HPWORK, &priv->work, qmi8658_gyro_worker, priv, delay);
    }

  ret = qmi8658_read_imu(&dev->base, NULL, &gyro);
  if (ret < 0)
    {
      return;
    }

  priv->last_update = gyro.timestamp;

  if (priv->lower.push_event && priv->lower.priv)
    {
      priv->lower.push_event(priv->lower.priv, &gyro, sizeof(gyro));
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: qmi8658_uorb_register
 *
 * Description:
 *   Register the QMI8658 IMU sensor device with the NuttX sensor
 *   framework. This function initializes the device, sets up the
 *   accelerometer and gyroscope sensors, and registers them with
 *   the sensor subsystem.
 *
 * Input Parameters:
 *   devno - Device number for sensor registration
 *   i2c   - Pointer to the I2C master interface
 *   addr  - I2C slave address of the QMI8658 device
 *
 * Returned Value:
 *   OK on success; negated errno on failure
 *
 *   -EINVAL - Invalid I2C pointer
 *   -ENOMEM - Memory allocation failure
 *   -EIO    - Device initialization or registration failure
 *
 * Assumptions:
 *   The I2C bus is properly configured and available.
 *   The QMI8658 device is connected and powered.
 *
 ****************************************************************************/

int qmi8658_uorb_register(int devno, FAR struct i2c_master_s *i2c,
                          uint8_t addr)
{
  FAR struct qmi8658_uorb_dev_s *dev;
  struct sensor_lowerhalf_s *lower;
  struct qmi8658_scale_factors_s scale_factors;
  int ret = OK;
  int i;

  if (!i2c)
    {
      return -EINVAL;
    }

  dev = (FAR struct qmi8658_uorb_dev_s *)
        kmm_zalloc(sizeof(struct qmi8658_uorb_dev_s));
  if (!dev)
    {
      return -ENOMEM;
    }

  dev->base.i2c = i2c;
  dev->base.addr = addr;
  dev->base.freq = CONFIG_QMI8658_I2C_FREQUENCY;

  for (i = 0; i < QMI8658_MAX_IDX; i++)
    {
      FAR struct qmi8658_sensor_s *sensor = &dev->priv[i];

      sensor->dev = &dev->base;
      sensor->enabled = false;
#ifdef CONFIG_SENSORS_QMI8658_POLL
      sensor->interval = CONFIG_SENSORS_QMI8658_POLL_INTERVAL;
      sensor->last_update = 0;

      memset(&sensor->work, 0, sizeof(sensor->work));
#endif

      lower = &sensor->lower;
      lower->type = (i == QMI8658_ACCEL_IDX) ? SENSOR_TYPE_ACCELEROMETER :
                                              SENSOR_TYPE_GYROSCOPE;
      lower->nbuffer = 2;
      lower->ops = &g_sensor_ops;

      ret = qmi8658_get_scale_factors(&dev->base, &scale_factors);
      if (ret < 0)
        {
          snerr("Failed to get scale factors: %d\n", ret);
          goto errout;
        }

      sensor->scale = (i == QMI8658_ACCEL_IDX) ? scale_factors.acc_scale :
                                                  scale_factors.gyro_scale;
    }

  ret = qmi8658_initialize(&dev->base);
  if (ret < 0)
    {
      snerr("Failed to initialize QMI8658: %d\n", ret);
      goto errout;
    }

  for (i = 0; i < QMI8658_MAX_IDX; i++)
    {
      FAR struct qmi8658_sensor_s *sensor = &dev->priv[i];
      FAR const char *devname;

      devname = (i == QMI8658_ACCEL_IDX) ? "qmi8658_accel" : "qmi8658_gyro";

      ret = sensor_register(&sensor->lower, devno);
      if (ret < 0)
        {
          snerr("Failed to register %s: %d\n", devname, ret);
          goto errout;
        }
    }

  return ret;

errout:
  for (i = 0; i < QMI8658_MAX_IDX; i++)
    {
      if (dev->priv[i].lower.type != 0)
        {
          sensor_unregister(&dev->priv[i].lower, devno);
        }
    }

  kmm_free(dev);

  return ret;
}
