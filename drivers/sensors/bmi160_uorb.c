/****************************************************************************
 * drivers/sensors/bmi160_uorb.c
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

#include "bmi160_base.h"
#include <sys/param.h>
#include <nuttx/wqueue.h>
#include <nuttx/signal.h>
#include <nuttx/sensors/sensor.h>

#if defined(CONFIG_SENSORS_BMI160_UORB)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BMI160_DEFAULT_INTERVAL 10000  /* Default conversion interval. */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Sensor ODR */

struct bmi160_odr_s
{
  uint8_t regval;    /* the data of register */
  uint32_t odr;      /* the unit is us */
};

/* Device struct */

struct bmi160_dev_uorb_s
{
  /* sensor_lowerhalf_s must be in the first line. */

  struct sensor_lowerhalf_s lower;      /* Lower half sensor driver. */

  struct work_s work;                   /* Interrupt handler worker. */
  uint32_t interval;                    /* Sensor acquisition interval. */

  struct bmi160_dev_s dev;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Sensor handle functions */

static void bmi160_accel_enable(FAR struct bmi160_dev_uorb_s *priv,
                                bool enable);
static void bmi160_gyro_enable(FAR struct bmi160_dev_uorb_s *priv,
                               bool enable);

/* Sensor ops functions */

static int bmi160_set_accel_interval(FAR struct sensor_lowerhalf_s *lower,
                                     FAR struct file *filep,
                                     FAR uint32_t *period_us);
static int bmi160_set_gyro_interval(FAR struct sensor_lowerhalf_s *lower,
                                    FAR struct file *filep,
                                    FAR uint32_t *period_us);
static int bmi160_accel_activate(FAR struct sensor_lowerhalf_s *lower,
                                 FAR struct file *filep,
                                 bool enable);
static int bmi160_gyro_activate(FAR struct sensor_lowerhalf_s *lower,
                                FAR struct file *filep,
                                bool enable);

/* Sensor poll functions */

static void bmi160_accel_worker(FAR void *arg);
static void bmi160_gyro_worker(FAR void *arg);
static int bmi160_findodr(uint32_t time,
                          FAR const struct bmi160_odr_s *odr_s,
                          int len);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_bmi160_accel_ops =
{
  .activate     = bmi160_accel_activate,      /* Enable/disable sensor. */
  .set_interval = bmi160_set_accel_interval,  /* Set output data period. */
};

static const struct sensor_ops_s g_bmi160_gyro_ops =
{
  .activate     = bmi160_gyro_activate,      /* Enable/disable sensor. */
  .set_interval = bmi160_set_gyro_interval,  /* Set output data period. */
};

static const struct bmi160_odr_s g_bmi160_gyro_odr[] =
{
  { GYRO_ODR_25HZ,  40000 }, /* Sampling interval is 40ms. */
  { GYRO_ODR_50HZ,  20000 }, /* Sampling interval is 20ms. */
  { GYRO_ODR_100HZ, 10000 }, /* Sampling interval is 10ms. */
  { GYRO_ODR_200HZ,  5000 }, /* Sampling interval is 5ms. */
  { GYRO_ODR_400HZ,  2500 }, /* Sampling interval is 2.5ms. */
  { GYRO_ODR_800HZ,  1250 }, /* Sampling interval is 1.25ms. */
  { GYRO_ODR_1600HZ,  625 }, /* Sampling interval is 0.625ms. */
  { GYRO_ODR_3200HZ,  312 }, /* Sampling interval is 0.3125ms. */
};

static const struct bmi160_odr_s g_bmi160_accel_odr[] =
{
  { BMI160_ACCEL_ODR_0_78HZ, 1282000 }, /* Sampling interval is 1282.0ms. */
  { BMI160_ACCEL_ODR_1_56HZ,  641000 }, /* Sampling interval is 641.0ms. */
  { BMI160_ACCEL_ODR_3_12HZ,  320500 }, /* Sampling interval is 320.5ms. */
  { BMI160_ACCEL_ODR_6_25HZ,  160000 }, /* Sampling interval is 160.0ms. */
  { BMI160_ACCEL_ODR_12_5HZ,   80000 }, /* Sampling interval is 80.0ms. */
  { BMI160_ACCEL_ODR_25HZ,     40000 }, /* Sampling interval is 40.0ms. */
  { BMI160_ACCEL_ODR_50HZ,     20000 }, /* Sampling interval is 20.0ms. */
  { BMI160_ACCEL_ODR_100HZ,    10000 }, /* Sampling interval is 10.0ms. */
  { BMI160_ACCEL_ODR_200HZ,     5000 }, /* Sampling interval is 5.0ms. */
  { BMI160_ACCEL_ODR_400HZ,     2500 }, /* Sampling interval is 2.5ms. */
  { BMI160_ACCEL_ODR_800HZ,     1250 }, /* Sampling interval is 1.25ms. */
  { BMI160_ACCEL_ODR_1600HZ,     625 }, /* Sampling interval is 0.625ms. */
};

/****************************************************************************
 * Name: bmi160_findodr
 *
 * Description:
 *   Find the period that matches best.
 *
 * Input Parameters:
 *   time  - Desired interval.
 *   odr_s - Array of sensor output data rate.
 *   len   - Array length.
 *
 * Returned Value:
 *   Index of the best fit ODR.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int bmi160_findodr(uint32_t time,
                          FAR const struct bmi160_odr_s *odr_s,
                          int len)
{
  int i;

  for (i = 0; i < len; i++)
    {
      if (time == odr_s[i].odr)
        {
          return i;
        }
    }

  return i - 1;
}

/****************************************************************************
 * Name: bmi160_accel_enable
 *
 * Description:
 *   Enable or disable sensor device. when enable sensor, sensor will
 *   work in  current mode(if not set, use default mode). when disable
 *   sensor, it will disable sense path and stop convert.
 *
 * Input Parameters:
 *   priv   - The instance of lower half sensor driver
 *   enable - true(enable) and false(disable)
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static void bmi160_accel_enable(FAR struct bmi160_dev_uorb_s *priv,
                                bool enable)
{
  int idx;

  if (enable)
    {
      /* Set accel as normal mode. */

      bmi160_putreg8(&priv->dev, BMI160_CMD, ACCEL_PM_NORMAL);
      nxsig_usleep(30000);

      idx = bmi160_findodr(priv->interval, g_bmi160_accel_odr,
                           nitems(g_bmi160_accel_odr));
      bmi160_putreg8(&priv->dev, BMI160_ACCEL_CONFIG,
                     ACCEL_NORMAL_AVG4 | g_bmi160_accel_odr[idx].regval);

      work_queue(HPWORK, &priv->work,
                 bmi160_accel_worker, priv,
                 priv->interval / USEC_PER_TICK);
    }
  else
    {
      /* Set suspend mode to sensors. */

      work_cancel(HPWORK, &priv->work);
      bmi160_putreg8(&priv->dev, BMI160_CMD, ACCEL_PM_SUSPEND);
    }
}

/****************************************************************************
 * Name: bmi160_gyro_enable
 *
 * Description:
 *   Enable or disable sensor device. when enable sensor, sensor will
 *   work in  current mode(if not set, use default mode). when disable
 *   sensor, it will disable sense path and stop convert.
 *
 * Input Parameters:
 *   priv   - The instance of lower half sensor driver
 *   enable - true(enable) and false(disable)
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static void bmi160_gyro_enable(FAR struct bmi160_dev_uorb_s *priv,
                               bool enable)
{
  int idx;

  if (enable)
    {
      /* Set gyro as normal mode. */

      bmi160_putreg8(&priv->dev, BMI160_CMD, GYRO_PM_NORMAL);
      nxsig_usleep(30000);

      idx = bmi160_findodr(priv->interval, g_bmi160_gyro_odr,
                           nitems(g_bmi160_gyro_odr));
      bmi160_putreg8(&priv->dev, BMI160_GYRO_CONFIG,
                    GYRO_NORMAL_MODE | g_bmi160_gyro_odr[idx].regval);

      work_queue(HPWORK, &priv->work,
                 bmi160_gyro_worker, priv,
                 priv->interval / USEC_PER_TICK);
    }
  else
    {
      work_cancel(HPWORK, &priv->work);

      /* Set suspend mode to sensors. */

      bmi160_putreg8(&priv->dev, BMI160_CMD, GYRO_PM_SUSPEND);
    }
}

/****************************************************************************
 * Name: bmi160_set_accel_interval
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
 *                by lower half driver.
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int bmi160_set_accel_interval(FAR struct sensor_lowerhalf_s *lower,
                                     FAR struct file *filep,
                                     FAR uint32_t *period_us)
{
  FAR struct bmi160_dev_uorb_s *priv = (FAR struct bmi160_dev_uorb_s *)lower;
  int num;

  /* Sanity check. */

  if (NULL == priv || NULL == period_us)
    {
      return -EINVAL;
    }

  num = bmi160_findodr(*period_us, g_bmi160_accel_odr,
                       nitems(g_bmi160_accel_odr));
  bmi160_putreg8(&priv->dev, BMI160_ACCEL_CONFIG,
                 ACCEL_NORMAL_AVG4 | g_bmi160_accel_odr[num].regval);

  priv->interval = g_bmi160_accel_odr[num].odr;
  *period_us = priv->interval;
  return OK;
}

/****************************************************************************
 * Name: bmi160_set_gyro_interval
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
 *                by lower half driver.
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int bmi160_set_gyro_interval(FAR struct sensor_lowerhalf_s *lower,
                                    FAR struct file *filep,
                                    FAR uint32_t *period_us)
{
  FAR struct bmi160_dev_uorb_s *priv = (FAR struct bmi160_dev_uorb_s *)lower;
  int num;

  /* Sanity check. */

  if (NULL == priv || NULL == period_us)
    {
      return -EINVAL;
    }

  num = bmi160_findodr(*period_us, g_bmi160_gyro_odr,
                       nitems(g_bmi160_gyro_odr));
  bmi160_putreg8(&priv->dev, BMI160_GYRO_CONFIG,
                 GYRO_NORMAL_MODE | g_bmi160_gyro_odr[num].regval);

  priv->interval = g_bmi160_gyro_odr[num].odr;
  *period_us = priv->interval;
  return OK;
}

/****************************************************************************
 * Name: bmi160_gyro_activate
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
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int bmi160_gyro_activate(FAR struct sensor_lowerhalf_s *lower,
                                FAR struct file *filep,
                                bool enable)
{
  FAR struct bmi160_dev_uorb_s *priv = (FAR struct bmi160_dev_uorb_s *)lower;

  bmi160_gyro_enable(priv, enable);

  return OK;
}

/****************************************************************************
 * Name: bmi160_accel_activate
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
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int bmi160_accel_activate(FAR struct sensor_lowerhalf_s *lower,
                                 FAR struct file *filep,
                                 bool enable)
{
  FAR struct bmi160_dev_uorb_s *priv = (FAR struct bmi160_dev_uorb_s *)lower;

  bmi160_accel_enable(priv, enable);

  return OK;
}

/* Sensor poll functions */

/****************************************************************************
 * Name: bmi160_accel_worker
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

static void bmi160_accel_worker(FAR void *arg)
{
  FAR struct bmi160_dev_uorb_s *priv = arg;
  struct sensor_accel accel;
  struct accel_t p;
  uint32_t time;

  DEBUGASSERT(priv != NULL);

  work_queue(HPWORK, &priv->work,
             bmi160_accel_worker, priv,
             priv->interval / USEC_PER_TICK);

  bmi160_getregs(&priv->dev, BMI160_DATA_14, (FAR uint8_t *)&p, 6);
  accel.x = p.x;
  accel.y = p.y;
  accel.z = p.z;

  bmi160_getregs(&priv->dev, BMI160_SENSORTIME_0, (FAR uint8_t *)&time, 3);

  /* Adjust sensing time into 24 bit */

  time >>= 8;
  accel.timestamp = time;

  priv->lower.push_event(priv->lower.priv, &accel, sizeof(accel));
}

/****************************************************************************
 * Name: bmi160_gyro_worker
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

static void bmi160_gyro_worker(FAR void *arg)
{
  FAR struct bmi160_dev_uorb_s *priv = arg;
  struct sensor_gyro gyro;
  struct gyro_t p;
  uint32_t time;

  DEBUGASSERT(priv != NULL);

  work_queue(HPWORK, &priv->work,
             bmi160_gyro_worker, priv,
             priv->interval / USEC_PER_TICK);

  bmi160_getregs(&priv->dev, BMI160_DATA_8, (FAR uint8_t *)&p, 6);
  gyro.x = p.x;
  gyro.y = p.y;
  gyro.z = p.z;

  bmi160_getregs(&priv->dev, BMI160_SENSORTIME_0, (FAR uint8_t *)&time, 3);

  /* Adjust sensing time into 24 bit */

  time >>= 8;
  gyro.timestamp = time;

  priv->lower.push_event(priv->lower.priv, &gyro, sizeof(gyro));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bmi160_register_accel
 *
 * Description:
 *   Register the BMI160 accel sensor.
 *
 * Input Parameters:
 *   devno   - Sensor device number.
 *   config  - Interrupt fuctions.
 *
 * Returned Value:
 *   Description of the value returned by this function (if any),
 *   including an enumeration of all possible error values.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_BMI160_I2C
static int bmi160_register_accel(int devno,
                                 FAR struct i2c_master_s *dev)
#else /* CONFIG_BMI160_SPI */
static int bmi160_register_accel(int devno,
                                 FAR struct spi_dev_s *dev)
#endif
{
  FAR struct bmi160_dev_uorb_s *priv;
  int ret;

  /* Sanity check */

  DEBUGASSERT(dev != NULL);

  /* Initialize the STK31850 device structure */

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  /* config accelerometer */

#ifdef CONFIG_SENSORS_BMI160_I2C
  priv->dev.i2c  = dev;
  priv->dev.addr = BMI160_I2C_ADDR;
  priv->dev.freq = BMI160_I2C_FREQ;

#else /* CONFIG_SENSORS_BMI160_SPI */
  priv->dev.spi = dev;

  /* BMI160 detects communication bus is SPI by rising edge of CS. */

  bmi160_getreg8(&priv->dev, 0x7f);
  bmi160_getreg8(&priv->dev, 0x7f); /* workaround: fail to switch SPI, run twice */
  nxsig_usleep(200);

#endif

  priv->lower.ops = &g_bmi160_accel_ops;
  priv->lower.type = SENSOR_TYPE_ACCELEROMETER;
  priv->lower.uncalibrated = true;
  priv->interval = BMI160_DEFAULT_INTERVAL;
  priv->lower.nbuffer = 1;

  /* Read and verify the deviceid */

  ret = bmi160_checkid(&priv->dev);
  if (ret < 0)
    {
      snerr("Wrong Device ID!\n");
      kmm_free(priv);
      return ret;
    }

  /* set sensor power mode */

  bmi160_putreg8(&priv->dev, BMI160_PMU_TRIGGER, 0);

  /* Register the character driver */

  ret = sensor_register(&priv->lower, devno);
  if (ret < 0)
    {
      snerr("Failed to register accel driver: %d\n", ret);
      kmm_free(priv);
    }

  return ret;
}

/****************************************************************************
 * Name: bmi160_register_gyro
 *
 * Description:
 *   Register the BMI160 gyro sensor.
 *
 * Input Parameters:
 *   devno   - Sensor device number.
 *   config  - Interrupt fuctions.
 *
 * Returned Value:
 *   Description of the value returned by this function (if any),
 *   including an enumeration of all possible error values.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_BMI160_I2C
static int bmi160_register_gyro(int devno,
                                FAR struct i2c_master_s *dev)
#else /* CONFIG_BMI160_SPI */
static int bmi160_register_gyro(int devno,
                                FAR struct spi_dev_s *dev)
#endif
{
  FAR struct bmi160_dev_uorb_s *priv;
  int ret ;

  /* Sanity check */

  DEBUGASSERT(dev != NULL);

  /* Initialize the device structure */

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  /* config gyroscope */

#ifdef CONFIG_SENSORS_BMI160_I2C
  priv->dev.i2c  = dev;
  priv->dev.addr = BMI160_I2C_ADDR;
  priv->dev.freq = BMI160_I2C_FREQ;

#else /* CONFIG_SENSORS_BMI160_SPI */
  priv->dev.spi = dev;
#endif

  priv->lower.ops = &g_bmi160_gyro_ops;
  priv->lower.type = SENSOR_TYPE_GYROSCOPE;
  priv->lower.uncalibrated = true;
  priv->interval = BMI160_DEFAULT_INTERVAL;
  priv->lower.nbuffer = 1;

  /* Read and verify the deviceid */

  ret = bmi160_checkid(&priv->dev);
  if (ret < 0)
    {
      snerr("Wrong Device ID!\n");
      kmm_free(priv);
      return ret;
    }

  /* set sensor power mode */

  bmi160_putreg8(&priv->dev, BMI160_PMU_TRIGGER, 0);

  /* Register the character driver */

  ret = sensor_register(&priv->lower, devno);
  if (ret < 0)
    {
      snerr("Failed to register gyro driver: %d\n", ret);
      kmm_free(priv);
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bmi160_register
 *
 * Description:
 *   Register the BMI160 accel and gyro sensor.
 *
 * Input Parameters:
 *   devno   - Sensor device number.
 *   dev     - An instance of the SPI or I2C interface to use to communicate
 *             with BMI160
 *
 * Returned Value:
 *   Description of the value returned by this function (if any),
 *   including an enumeration of all possible error values.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_BMI160_I2C
int bmi160_register_uorb(int devno, FAR struct i2c_master_s *dev)
#else /* CONFIG_BMI160_SPI */
int bmi160_register_uorb(int devno, FAR struct spi_dev_s *dev)
#endif
{
  int ret;

  ret = bmi160_register_accel(devno, dev);
  DEBUGASSERT(ret >= 0);

  ret = bmi160_register_gyro(devno, dev);
  DEBUGASSERT(ret >= 0);

  sninfo("BMI160 driver loaded successfully!\n");
  return ret;
}

#endif /* CONFIG_SENSORS_BMI160_UORB */
