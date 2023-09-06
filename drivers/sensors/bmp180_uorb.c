/****************************************************************************
 * drivers/sensors/bmp180_uorb.c
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

/* Character driver for the Freescale BMP1801 Barometer Sensor */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "bmp180_base.h"

#include <nuttx/wqueue.h>
#include <nuttx/sensors/sensor.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_BMP180)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BMP180_MIN_INTERVAL 30000

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

struct bmp180_dev_uorb_s
{
  /* sensor_lowerhalf_s must be in the first line. */

  struct sensor_lowerhalf_s lower; /* Lower half sensor driver. */
  struct work_s work;              /* Interrupt handler worker. */
  unsigned long interval;          /* Sensor acquisition interval. */
  struct bmp180_dev_s dev;
};
/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void bmp180_worker(FAR void *arg);
static int bmp180_set_interval(FAR struct sensor_lowerhalf_s *lower,
                               FAR struct file *filep,
                               FAR unsigned long *period_us);
static int bmp180_activate(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep,
                           bool enable);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_bmp180_ops =
{
  .activate     = bmp180_activate,     /* Enable/disable sensor. */
  .set_interval = bmp180_set_interval, /* Set output data period. */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bmp180_set_interval
 *
 * Description:
 *   Set the sensor output data period in microseconds for a given sensor.
 *   If *period_us > max_delay it will be truncated to max_delay and if
 *   *period_us < min_delay it will be replaced by min_delay.
 *
 * Input Parameters:
 *   lower     - The instance of lower half sensor driver.
 *   filep     - The pointer of file, represents each user using the sensor.
 *   period_us - The time between report data, in us. It may by overwrite
 *               by lower half driver.
 *
 * Returned Value:
 *   Return OK(0) if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int bmp180_set_interval(FAR struct sensor_lowerhalf_s *lower,
                               FAR struct file *filep,
                               FAR unsigned long *period_us)
{
  FAR struct bmp180_dev_uorb_s *priv = (FAR struct bmp180_dev_uorb_s *)lower;

  /* minimum interval 4.5ms + 25.5ms */

  if (*period_us < BMP180_MIN_INTERVAL)
    {
      priv->interval = BMP180_MIN_INTERVAL;
      *period_us = priv->interval;
    }
  else
    {
      priv->interval = *period_us;
    }

  return OK;
}

/****************************************************************************
 * Name: bmp180_activate
 *
 * Description:
 *   Enable or disable sensor device. when enable sensor, sensor will
 *   work in  current mode(if not set, use default mode). when disable
 *   sensor, it will disable sense path and stop convert.
 *
 * Input Parameters:
 *   lower  - The instance of lower half sensor driver.
 *   filep  - The pointer of file, represents each user using the sensor.
 *   enable - true(enable) and false(disable).
 *
 * Returned Value:
 *   Return OK(0)  if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int bmp180_activate(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep, bool enable)
{
  FAR struct bmp180_dev_uorb_s *priv = (FAR struct bmp180_dev_uorb_s *)lower;

  /* Set accel output data rate. */

  if (enable)
    {
      work_queue(HPWORK, &priv->work,
                 bmp180_worker, priv,
                 priv->interval / USEC_PER_TICK);
    }
  else
    {
      /* Set suspend mode to sensors. */

      work_cancel(HPWORK, &priv->work);
    }

  return OK;
}

/****************************************************************************
 * Name: bmp180_worker
 *
 * Description:
 *   Task the worker with retrieving the latest sensor data. We should not do
 *   this in a interrupt since it might take too long. Also we cannot lock
 *   the I2C bus from within an interrupt.
 *
 * Input Parameters:
 *   arg    - Device struct.
 *
 * Returned Value:
 *   none.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static void bmp180_worker(FAR void *arg)
{
  FAR struct bmp180_dev_uorb_s *priv = arg;
  struct sensor_baro baro;

  DEBUGASSERT(priv != NULL);

  work_queue(HPWORK, &priv->work,
             bmp180_worker, priv,
             priv->interval / USEC_PER_TICK);

  baro.pressure = bmp180_getpressure(&priv->dev, &baro.temperature) / 100.0f;
  baro.timestamp = sensor_get_timestamp();

  priv->lower.push_event(priv->lower.priv, &baro, sizeof(baro));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bmp180_register
 *
 * Description:
 *   Register the BMP180 character device as 'devpath'
 *
 * Input Parameters:
 *   devno   - Sensor device number.
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             BMP180
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int bmp180_register_uorb(int devno, FAR struct i2c_master_s *i2c)
{
  FAR struct bmp180_dev_uorb_s *priv;
  int ret;

  /* Initialize the BMP180 device structure */

  priv = (FAR struct bmp180_dev_uorb_s *)
         kmm_zalloc(sizeof(struct bmp180_dev_uorb_s));

  if (!priv)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->dev.i2c = i2c;
  priv->dev.addr = BMP180_ADDR;
  priv->dev.freq = BMP180_FREQ;
  priv->lower.ops = &g_bmp180_ops;
  priv->lower.type = SENSOR_TYPE_BAROMETER;
  priv->lower.nbuffer = 1;
  priv->interval = BMP180_MIN_INTERVAL;

  /* Check Device ID */

  ret = bmp180_checkid(&priv->dev);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
      return ret;
    }

  /* Read the coefficient value */

  bmp180_updatecaldata(&priv->dev);

  /* Register the character driver */

  ret = sensor_register(&priv->lower, devno);

  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  sninfo("BMP180 driver loaded successfully!\n");
  return ret;
}

#endif /* CONFIG_I2C && CONFIG_SENSORS_BMP180 */
