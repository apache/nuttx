/****************************************************************************
 * drivers/sensors/bmi088_uorb.c
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

#include "bmi088_base.h"
#include <sys/param.h>
#include <nuttx/wqueue.h>
#include <nuttx/signal.h>
#include <nuttx/sensors/sensor.h>

#if defined(CONFIG_SENSORS_BMI088_UORB)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BMI088_DEFAULT_INTERVAL 10000  /* Default conversion interval. */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Sensor ODR */

struct bmi088_odr_s
{
  uint8_t regval;    /* the data of register */
  uint32_t odr;      /* the unit is us */
};

/* Device struct */

struct bmi088_dev_uorb_s
{
  /* sensor_lowerhalf_s must be in the first line. */

  struct work_s work;                   /* Interrupt handler worker. */
  uint32_t interval;                    /* Sensor acquisition interval. */

  struct bmi088_dev_s dev;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Sensor handle functions */

static void bmi088_accel_enable(FAR struct bmi088_dev_uorb_s *priv,
                                bool enable);
static void bmi088_gyro_enable(FAR struct bmi088_dev_uorb_s *priv,
                               bool enable);

/* Sensor ops functions */

static int bmi088_set_accel_interval(FAR struct sensor_lowerhalf_s *lower,
                                     FAR struct file *filep,
                                     FAR uint32_t *period_us);
static int bmi088_set_gyro_interval(FAR struct sensor_lowerhalf_s *lower,
                                    FAR struct file *filep,
                                    FAR uint32_t *period_us);
static int bmi088_accel_activate(FAR struct sensor_lowerhalf_s *lower,
                                 FAR struct file *filep,
                                 bool enable);
static int bmi088_gyro_activate(FAR struct sensor_lowerhalf_s *lower,
                                FAR struct file *filep,
                                bool enable);

/* Sensor poll functions */

static void bmi088_accel_worker(FAR void *arg);
static void bmi088_gyro_worker(FAR void *arg);
static int bmi088_findodr(uint32_t time,
                          FAR const struct bmi088_odr_s *odr_s,
                          int len);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_bmi088_accel_ops =
{
  .activate     = bmi088_accel_activate,      /* Enable/disable sensor. */
  .set_interval = bmi088_set_accel_interval,  /* Set output data period. */
};

static const struct sensor_ops_s g_bmi088_gyro_ops =
{
  .activate     = bmi088_gyro_activate,      /* Enable/disable sensor. */
  .set_interval = bmi088_set_gyro_interval,  /* Set output data period. */
};

static const struct bmi088_odr_s g_bmi088_gyro_odr[] =
{
  { BMI088_GYRO_BANDWIDTH_100HZ_12HZ,   10000}, /* Sampling interval is 10ms. */
  { BMI088_GYRO_BANDWIDTH_100HZ_32HZ,   10000}, /* Sampling interval is 10ms. */
  { BMI088_GYRO_BANDWIDTH_200HZ_23HZ,   5000},  /* Sampling interval is 5ms. */
  { BMI088_GYRO_BANDWIDTH_200HZ_64HZ,   5000},  /* Sampling interval is 10ms. */
  { BMI088_GYRO_BANDWIDTH_400HZ_47HZ,   2500},  /* Sampling interval is 5ms. */
  { BMI088_GYRO_BANDWIDTH_1000HZ_116HZ, 1000},  /* Sampling interval is 2.5ms. */
  { BMI088_GYRO_BANDWIDTH_2000HZ_230HZ, 500},   /* Sampling interval is 1ms. */
  { BMI088_GYRO_BANDWIDTH_2000HZ_532HZ, 500},   /* Sampling interval is 0.5ms. */
};

static const struct bmi088_odr_s g_bmi088_accel_odr[] =
{
  { BMI088_ACC_CONF_ODR_12_5,   80000 }, /* Sampling interval is 80.0ms. */
  { BMI088_ACC_CONF_ODR_25,     40000 }, /* Sampling interval is 40.0ms. */
  { BMI088_ACC_CONF_ODR_50,     20000 }, /* Sampling interval is 20.0ms. */
  { BMI088_ACC_CONF_ODR_100,    10000 }, /* Sampling interval is 10.0ms. */
  { BMI088_ACC_CONF_ODR_200,     5000 }, /* Sampling interval is 5.0ms. */
  { BMI088_ACC_CONF_ODR_400,     2500 }, /* Sampling interval is 2.5ms. */
  { BMI088_ACC_CONF_ODR_800,     1250 }, /* Sampling interval is 1.25ms. */
  { BMI088_ACC_CONF_ODR_1600,     625 }, /* Sampling interval is 0.625ms. */
};

/****************************************************************************
 * Name: bmi088_findodr
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

static int bmi088_findodr(uint32_t time,
                          FAR const struct bmi088_odr_s *odr_s,
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
 * Name: bmi088_accel_enable
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

static void bmi088_accel_enable(FAR struct bmi088_dev_uorb_s *priv,
                                bool enable)
{
  int idx;

  if (enable)
    {
      /* Set accel as normal mode. */

      nxsig_usleep(1000);
      bmi088_putreg8(&priv->dev_acc, BMI088_ACC_PWR_CTRL,
                                BMI088_ACC_PWR_CTRL_ACC_ENABLE);
      nxsig_usleep(50000);

      idx = bmi088_findodr(priv->interval, g_bmi088_accel_odr,
                           nitems(g_bmi088_accel_odr));
      bmi088_putreg8(&priv->dev, BMI088_ACC_CONF,
                                 BMI088_ACC_CONF_BWP_NORMAL |
                                 g_bmi088_accel_odr[idx].regval);

      work_queue(HPWORK, &priv->work,
                 bmi088_accel_worker, priv,
                 priv->interval / USEC_PER_TICK);
    }
  else
    {
      /* Set suspend mode to sensors. */

      work_cancel(HPWORK, &priv->work);
      bmi088_putreg8(&priv->dev, BMI088_ACC_PWR_CTRL,
                                 BMI088_ACC_PWR_CONF_SUSPEND_MODE);
    }
}

/****************************************************************************
 * Name: bmi088_gyro_enable
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

static void bmi088_gyro_enable(FAR struct bmi088_dev_uorb_s *priv,
                               bool enable)
{
  int idx;

  if (enable)
    {
      /* Set gyro as normal mode. */

      bmi088_putreg8(&priv->dev, BMI088_GYRO_LPM1,
                                 BMI088_GYRO_LPM1_NORMAL_MODE);
      nxsig_usleep(30000);

      idx = bmi088_findodr(priv->interval, g_bmi088_gyro_odr,
                           nitems(g_bmi088_gyro_odr));
      bmi088_putreg8(&priv->dev, BMI088_GYRO_BANDWIDTH,
                                g_bmi088_gyro_odr[idx].regval);

      work_queue(HPWORK, &priv->work,
                 bmi088_gyro_worker, priv,
                 priv->interval / USEC_PER_TICK);
    }
  else
    {
      work_cancel(HPWORK, &priv->work);

      /* Set suspend mode to sensors. */

      bmi088_putreg8(&priv->dev, BMI088_GYRO_LPM1,
                                 BMI088_GYRO_LPM1_SUSPEND_MODE);
    }
}

/****************************************************************************
 * Name: bmi088_set_accel_interval
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

static int bmi088_set_accel_interval(FAR struct sensor_lowerhalf_s *lower,
                                     FAR struct file *filep,
                                     FAR uint32_t *period_us)
{
  FAR struct bmi088_dev_uorb_s *priv = (FAR struct bmi088_dev_uorb_s *)lower;
  int num;

  /* Sanity check. */

  if (NULL == priv || NULL == period_us)
    {
      return -EINVAL;
    }

  num = bmi088_findodr(*period_us, g_bmi088_accel_odr,
                       nitems(g_bmi088_accel_odr));
  bmi088_putreg8(&priv->dev, BMI088_ACC_CONF,
                 BMI088_ACC_CONF_BWP_NORMAL |
                 g_bmi088_accel_odr[num].regval);

  priv->interval = g_bmi088_accel_odr[num].odr;
  *period_us = priv->interval;
  return OK;
}

/****************************************************************************
 * Name: bmi088_set_gyro_interval
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

static int bmi088_set_gyro_interval(FAR struct sensor_lowerhalf_s *lower,
                                    FAR struct file *filep,
                                    FAR uint32_t *period_us)
{
  FAR struct bmi088_dev_uorb_s *priv = (FAR struct bmi088_dev_uorb_s *)lower;
  int num;

  /* Sanity check. */

  if (NULL == priv || NULL == period_us)
    {
      return -EINVAL;
    }

  num = bmi088_findodr(*period_us, g_bmi088_gyro_odr,
                       nitems(g_bmi088_gyro_odr));
  bmi088_putreg8(&priv->dev, BMI088_GYRO_BANDWIDTH,
                              g_bmi088_gyro_odr[num].regval);

  priv->interval = g_bmi088_gyro_odr[num].odr;
  *period_us = priv->interval;
  return OK;
}

/****************************************************************************
 * Name: bmi088_gyro_activate
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

static int bmi088_gyro_activate(FAR struct sensor_lowerhalf_s *lower,
                                FAR struct file *filep,
                                bool enable)
{
  FAR struct bmi088_dev_uorb_s *priv = (FAR struct bmi088_dev_uorb_s *)lower;

  bmi088_gyro_enable(priv, enable);

  return OK;
}

/****************************************************************************
 * Name: bmi088_accel_activate
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

static int bmi088_accel_activate(FAR struct sensor_lowerhalf_s *lower,
                                 FAR struct file *filep,
                                 bool enable)
{
  FAR struct bmi088_dev_uorb_s *priv = (FAR struct bmi088_dev_uorb_s *)lower;

  bmi088_accel_enable(priv, enable);

  return OK;
}

/* Sensor poll functions */

/****************************************************************************
 * Name: bmi088_accel_worker
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

static void bmi088_accel_worker(FAR void *arg)
{
  FAR struct bmi088_dev_uorb_s *priv = arg;
  struct sensor_accel accel;
  struct accel_t p;
  uint32_t time;

  DEBUGASSERT(priv != NULL);

  work_queue(HPWORK, &priv->work,
             bmi088_accel_worker, priv,
             priv->interval / USEC_PER_TICK);

  bmi088_getregs(&priv->dev, BMI088_ACC_X_LSB, (FAR uint8_t *)&p, 6);
  accel.x = p.x;
  accel.y = p.y;
  accel.z = p.z;

  bmi088_getregs(&priv->dev, BMI088_SENSORTIME_0, (FAR uint8_t *)&time, 3);

  /* Adjust sensing time into 24 bit */

  time >>= 8;
  accel.timestamp = time;

  priv->lower.push_event(priv->lower.priv, &accel, sizeof(accel));
}

/****************************************************************************
 * Name: bmi088_gyro_worker
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

static void bmi088_gyro_worker(FAR void *arg)
{
  FAR struct bmi088_dev_uorb_s *priv = arg;
  struct sensor_gyro gyro;
  struct gyro_t p;
  uint32_t time;

  DEBUGASSERT(priv != NULL);

  work_queue(HPWORK, &priv->work,
             bmi088_gyro_worker, priv,
             priv->interval / USEC_PER_TICK);

  bmi088_getregs(&priv->dev, BMI088_RATE_X_LSB, (FAR uint8_t *)&p, 6);
  gyro.x = p.x;
  gyro.y = p.y;
  gyro.z = p.z;

  bmi088_getregs(&priv->dev, BMI088_SENSORTIME_0, (FAR uint8_t *)&time, 3);

  /* Adjust sensing time into 24 bit */

  time >>= 8;
  gyro.timestamp = time;

  priv->lower.push_event(priv->lower.priv, &gyro, sizeof(gyro));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bmi088_register_accel
 *
 * Description:
 *   Register the BMI088 accel sensor.
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

#ifdef CONFIG_SENSORS_BMI088_I2C
static int bmi088_register_accel(int devno,
                                 FAR struct i2c_master_s *dev)
#else /* CONFIG_BMI088_SPI */
static int bmi088_register_accel(int devno,
                                 FAR struct spi_dev_s *dev)
#endif
{
  FAR struct bmi088_dev_uorb_s *priv
  int ret;

  /* Sanity check */

  DEBUGASSERT(dev != NULL);

  /* Initialize the BMI088 device structure */

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  /* config accelerometer */

#ifdef CONFIG_SENSORS_BMI088_I2C
  priv->dev_acc.i2c  = priv_acc;
  priv->dev_acc.addr = BMI088_I2C_ACC_ADDR;
  priv->dev_acc.freq = BMI088_I2C_FREQ;

#else /* CONFIG_SENSORS_BMI088_SPI */
  priv->dev.spi = dev;

  /* BMI088 detects communication bus is SPI by rising edge of CS. */

  bmi088_getreg8(&priv->dev_acc , BMI088_ACC_CHIP_ID);
  nxsig_usleep(200);

#endif

  priv->lower.lower  = &g_bmi088_accel_ops;
  priv->lower.type = SENSOR_TYPE_ACCELEROMETER;
  priv->lower.uncalibrated = true;
  priv->interval = BMI088_DEFAULT_INTERVAL;
  priv->lower.nbuffer = 1;

  /* Read and verify the deviceid */

  ret = bmi088_acc_checkid(&priv->dev_acc);
  if (ret < 0)
    {
      snerr("Wrong Device ID!\n");
      kmm_free(priv);
      return ret;
    }

  /* set sensor power mode */

  bmi088_putreg8(&priv->dev, BMI088_ACC_CONF, BMI088_ACC_CONF_BWP_NORMAL);

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
 * Name: bmi088_register_gyro
 *
 * Description:
 *   Register the BMI088 gyro sensor.
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

#ifdef CONFIG_SENSORS_BMI088_I2C
static int bmi088_register_gyro(int devno,
                                FAR struct i2c_master_s *dev)
#else /* CONFIG_BMI088_SPI */
static int bmi088_register_gyro(int devno,
                                FAR struct spi_dev_s *dev)
#endif
{
  FAR struct bmi088_dev_uorb_s *priv;
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

#ifdef CONFIG_SENSORS_BMI088_I2C
  priv->dev.i2c  = dev;
  priv->dev.addr = BMI088_I2C_GY_ADDR;
  priv->dev.freq = BMI088_I2C_FREQ;

#else /* CONFIG_SENSORS_BMI088_SPI */
  priv->dev.spi = dev;
#endif

  priv->lower.ops = &g_bmi088_gyro_ops;
  priv->lower.type = SENSOR_TYPE_GYROSCOPE;
  priv->lower.uncalibrated = true;
  priv->interval = BMI088_DEFAULT_INTERVAL;
  priv->lower.nbuffer = 1;

  /* Read and verify the deviceid */

  ret = bmi088_gyro_checkid(&priv->dev);
  if (ret < 0)
    {
      snerr("Wrong Device ID!\n");
      kmm_free(priv);
      return ret;
    }

  /* set sensor power mode */

  bmi088_putreg8(&priv->dev, BMI088_GYRO_LPM1, BMI088_GYRO_LPM1_NORMAL_MODE);

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
 * Name: bmi088_register
 *
 * Description:
 *   Register the BMI088 accel and gyro sensor.
 *
 * Input Parameters:
 *   devno   - Sensor device number.
 *   dev     - An instance of the SPI or I2C interface to use to communicate
 *             with BMI088
 *
 * Returned Value:
 *   Description of the value returned by this function (if any),
 *   including an enumeration of all possible error values.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_BMI088_I2C
int bmi088_register_uorb(int devno, FAR struct i2c_master_s *dev)
#else /* CONFIG_BMI088_SPI */
int bmi088_register_uorb(int devno, FAR struct spi_dev_s *dev)
#endif
{
  int ret;

  ret = bmi088_register_accel(devno, dev);
  DEBUGASSERT(ret >= 0);

  ret = bmi088_register_gyro(devno, dev);
  DEBUGASSERT(ret >= 0);

  sninfo("BMI088 driver loaded successfully!\n");
  return ret;
}

#endif /* CONFIG_SENSORS_BMI088_UORB */
