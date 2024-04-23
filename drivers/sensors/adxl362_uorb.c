/****************************************************************************
 * drivers/sensors/adxl362_uorb.c
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

#include <debug.h>
#include <stdio.h>
#include <string.h>

#include <sys/param.h>

#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/signal.h>
#include <nuttx/sensors/adxl362.h>

#include <nuttx/sensors/sensor.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ADXL362_SPI_FREQUENCY         1000000
#define ADXL362_SPI_MODE              SPIDEV_MODE0

#define ADXL362_WRITE                 0x0a
#define ADXL362_READ                  0x0b
#define ADXL362_FIFO                  0x0d

#define ADXL362_DEVID_AD              0x00
#  define ADXL362_DEVID_AD_VALUE      0xad
#define ADXL362_DEVID_MST             0x01
#  define ADXL362_DEVID_MST_VALUE     0x1d
#define ADXL362_PARTID                0x02
#  define ADXL362_PARTID_VALUE        0xf2
#define ADXL362_REVID                 0x03
#define ADXL362_XDATA                 0x08
#define ADXL362_YDATA                 0x09
#define ADXL362_ZDATA                 0x0a
#define ADXL362_STATUS                0x0b
#define ADXL362_FIFO_L                0x0c
#define ADXL362_FIFO_H                0x0d
#define ADXL362_XDATA_L               0x0e
#define ADXL362_XDATA_H               0x0f
#define ADXL362_YDATA_L               0x10
#define ADXL362_YDATA_H               0x11
#define ADXL362_ZDATA_L               0x12
#define ADXL362_ZDATA_H               0x13
#define ADXL362_TMEP_L                0x14
#define ADXL362_TEMP_H                0x15
#define ADXL362_SOFT_RESET            0x1f
#  define ADXL362_SOFT_RESET_VALUE    0x52
#define ADXL362_THRESH_ACT_L          0x20
#define ADXL362_THRESH_ACT_H          0x21
#define ADXL362_TIME_ACT              0x22
#define ADXL362_THRESH_INACT_L        0x23
#define ADXL362_THRESH_INACT_H        0x24
#define ADXL362_TIME_INACT_L          0x25
#define ADXL362_TIME_INACT_H          0x26
#define ADXL362_ACT_INACT_CTL         0x27
#define ADXL362_FIFO_CONTROL          0x28
#define ADXL362_FIFO_SAMPLES          0x29
#define ADXL362_INTMAP1               0x2a
#define ADXL362_INTMAP2               0x2b
#define ADXL362_FILTER_CTL            0x2c
#  define ADXL362_FILTER_CTL_DEFAULT  0x13
#define ADXL362_POWER_CTL             0x2d
#   define ADXL362_LOW_NOISE          (0 << 4)
#   define ADXL362_ULTRALOW_NOISE     (2 << 4)
#   define ADXL362_POWER_MODE_STANDBY 0
#   define ADXL362_POWER_MODE_MEASURE 2
#define ADXL362_SELF_TEST             0x2e

#define CONSTANTS_ONE_G               9.8f
#define ADXL362_TEMP_SCALE            0.065f

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct adxl362_sensor_s
{
  struct sensor_lowerhalf_s  lower;
  FAR struct spi_dev_s      *spi;
  int                        devno;
  float                      scale;
#ifdef CONFIG_SENSORS_ADXL362_POLL
  bool                       enabled;
  unsigned long              interval;
  sem_t                      run;
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static uint8_t adxl362_getreg8(FAR struct adxl362_sensor_s *priv,
                              uint8_t regaddr);
static void adxl362_putreg8(FAR struct adxl362_sensor_s *priv,
                            uint8_t regaddr, uint8_t regval);
static void adxl362_getregs(FAR struct adxl362_sensor_s *priv,
                            uint8_t regaddr, FAR uint8_t *regval, int len);
static int16_t adxl362_data(FAR uint8_t *data);
static int adxl362_checkid(FAR struct adxl362_sensor_s *priv);
static void adxl362_start(FAR struct adxl362_sensor_s *priv);
static void adxl362_stop(FAR struct adxl362_sensor_s *priv);

/* Sensor ops functions */

static int adxl362_activate(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep,
                            bool enable);
static int adxl362_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                FAR struct file *filep,
                                FAR unsigned long *period_us);
#ifndef CONFIG_SENSORS_ADXL362_POLL
static int adxl362_fetch(FAR struct sensor_lowerhalf_s *lower,
                         FAR struct file *filep,
                         FAR char *buffer, size_t buflen);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_adxl362_accel_ops =
{
  NULL,                 /* open */
  NULL,                 /* close */
  adxl362_activate,
  adxl362_set_interval,
  NULL,                 /* batch */
#ifdef CONFIG_SENSORS_ADXL362_POLL
  NULL,                 /* fetch */
#else
  adxl362_fetch,
#endif
  NULL,                 /* selftest */
  NULL,                 /* set_calibvalue */
  NULL,                 /* calibrate */
  NULL,                 /* control */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: adxl362_getreg8
 *
 * Description:
 *   Read from an 8-bit ADXL362 register
 *
 ****************************************************************************/

static uint8_t adxl362_getreg8(FAR struct adxl362_sensor_s *priv,
                               uint8_t regaddr)
{
  uint8_t regval = 0;

  SPI_LOCK(priv->spi, true);

  SPI_SETFREQUENCY(priv->spi, ADXL362_SPI_FREQUENCY);
  SPI_SETMODE(priv->spi, ADXL362_SPI_MODE);

  /* Select the ADXL362 */

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER(priv->devno), true);

  /* Send register to read and get the next byte */

  SPI_SEND(priv->spi, ADXL362_READ);
  SPI_SEND(priv->spi, regaddr);
  SPI_RECVBLOCK(priv->spi, &regval, 1);

  /* Deselect the ADXL362 */

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER(priv->devno), false);

  /* Unlock bus */

  SPI_LOCK(priv->spi, false);

  return regval;
}

/****************************************************************************
 * Name: adxl362_putreg8
 *
 * Description:
 *   Write a value to an 8-bit ADXL362 register
 *
 ****************************************************************************/

static void adxl362_putreg8(FAR struct adxl362_sensor_s *priv,
                            uint8_t regaddr, uint8_t regval)
{
  SPI_LOCK(priv->spi, true);

  SPI_SETFREQUENCY(priv->spi, ADXL362_SPI_FREQUENCY);
  SPI_SETMODE(priv->spi, ADXL362_SPI_MODE);

  /* Select the ADXL362 */

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER(priv->devno), true);

  /* Send register address and set the value */

  SPI_SEND(priv->spi, ADXL362_WRITE);
  SPI_SEND(priv->spi, regaddr);
  SPI_SEND(priv->spi, regval);

  /* Deselect the ADXL362 */

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER(priv->devno), false);

  /* Unlock bus */

  SPI_LOCK(priv->spi, false);
}

/****************************************************************************
 * Name: adxl362_getregs
 *
 * Description:
 *   Read bytes from specified regaddr
 *
 ****************************************************************************/

static void adxl362_getregs(FAR struct adxl362_sensor_s *priv,
                            uint8_t regaddr, FAR uint8_t *regval, int len)
{
  /* If SPI bus is shared then lock and configure it */

  SPI_LOCK(priv->spi, true);

  SPI_SETFREQUENCY(priv->spi, ADXL362_SPI_FREQUENCY);
  SPI_SETMODE(priv->spi, ADXL362_SPI_MODE);

  /* Select the ADXL362 */

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER(priv->devno), true);

  /* Send register to read and get the next 2 bytes */

  SPI_SEND(priv->spi, ADXL362_READ);
  SPI_SEND(priv->spi, regaddr);
  SPI_RECVBLOCK(priv->spi, regval, len);

  /* Deselect the ADXL362 */

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER(priv->devno), false);

  /* Unlock bus */

  SPI_LOCK(priv->spi, false);
}

/****************************************************************************
 * Name: adxl362_data
 ****************************************************************************/

static int16_t adxl362_data(FAR uint8_t *data)
{
  return (int16_t)(data[0] | (data[1]) << 8);
}

/****************************************************************************
 * Name: adxl362_checkid
 *
 * Description:
 *   Read and verify the ADXL362 chip ID
 *
 ****************************************************************************/

static int adxl362_checkid(FAR struct adxl362_sensor_s *priv)
{
  uint8_t id = 0;

  id = adxl362_getreg8(priv, ADXL362_DEVID_AD);
  if (id != ADXL362_DEVID_AD_VALUE)
    {
      snerr("Wrong AD! %02x\n", id);
      return -ENODEV;
    }

  id = adxl362_getreg8(priv, ADXL362_DEVID_MST);
  if (id != ADXL362_DEVID_MST_VALUE)
    {
      snerr("Wrong MST! %02x\n", id);
      return -ENODEV;
    }

  id = adxl362_getreg8(priv, ADXL362_PARTID);
  if (id != ADXL362_PARTID_VALUE)
    {
      snerr("Wrong PARTID! %02x\n", id);
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: adxl362_reset
 *
 * Description:
 *   Soft reset
 *
 ****************************************************************************/

static void adxl362_reset(FAR struct adxl362_sensor_s *priv)
{
  adxl362_putreg8(priv, ADXL362_SOFT_RESET, ADXL362_SOFT_RESET_VALUE);
  up_mdelay(5);
}

/****************************************************************************
 * Name: adxl362_start
 ****************************************************************************/

static void adxl362_start(FAR struct adxl362_sensor_s *priv)
{
  adxl362_putreg8(priv, ADXL362_POWER_CTL, ADXL362_POWER_MODE_MEASURE);

  /* Wait for sensor ready - otherwise the first measuremet is garbage */

  up_mdelay(5);
}

/****************************************************************************
 * Name: adxl362_stop
 ****************************************************************************/

static void adxl362_stop(FAR struct adxl362_sensor_s *priv)
{
  adxl362_putreg8(priv, ADXL362_POWER_CTL, 0);
}

/****************************************************************************
 * Name: adxl362_activate
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

static int adxl362_activate(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep,
                            bool enable)
{
  FAR struct adxl362_sensor_s *priv = (FAR struct adxl362_sensor_s *)lower;
#ifdef CONFIG_SENSORS_ADXL362_POLL
  bool start_thread = false;
#endif

  if (enable)
    {
#ifdef CONFIG_SENSORS_ADXL362_POLL
      if (!priv->enabled)
        {
          start_thread = true;
        }
#endif

      adxl362_start(priv);
    }
  else
    {
      adxl362_stop(priv);
    }

#ifdef CONFIG_SENSORS_ADXL362_POLL
  priv->enabled = enable;

  if (start_thread)
    {
      /* Wake up the thread */

      nxsem_post(&priv->run);
    }
#endif

  return OK;
}

/****************************************************************************
 * Name: adxl362_set_interval
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

static int adxl362_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                FAR struct file *filep,
                                FAR unsigned long *period_us)
{
#ifdef CONFIG_SENSORS_ADXL362_POLL
  FAR struct adxl362_sensor_s *priv = (FAR struct adxl362_sensor_s *)lower;

  priv->interval = *period_us;
#endif

  return OK;
}

#ifndef CONFIG_SENSORS_ADXL362_POLL
/****************************************************************************
 * Name: adxl362_set_interval
 ****************************************************************************/

static int adxl362_fetch(FAR struct sensor_lowerhalf_s *lower,
                         FAR struct file *filep, FAR char *buffer,
                         size_t buflen)
{
  FAR struct adxl362_sensor_s *priv = (FAR struct adxl362_sensor_s *)lower;
  struct sensor_accel          accel;
  uint8_t                      data[8];

  /* Wait for data ready */

  while (!(adxl362_getreg8(priv, ADXL362_STATUS) & 0x01));

  /* Get data */

  adxl362_getregs(priv, ADXL362_XDATA_L, (FAR uint8_t *)data, 8);

  accel.timestamp   = sensor_get_timestamp();
  accel.x           = (float)adxl362_data(&data[0]) * priv->scale;
  accel.y           = (float)adxl362_data(&data[2]) * priv->scale;
  accel.z           = (float)adxl362_data(&data[4]) * priv->scale;
  accel.temperature = (float)adxl362_data(&data[6]) * ADXL362_TEMP_SCALE;

  memcpy(buffer, &accel, sizeof(accel));

  return sizeof(accel);
}
#endif

#ifdef CONFIG_SENSORS_ADXL362_POLL
/****************************************************************************
 * Name: adxl362_thread
 *
 * Description: Thread for performing interval measurement cycle and data
 *              read.
 *
 * Parameter:
 *   argc - Number opf arguments
 *   argv - Pointer to argument list
 *
 ****************************************************************************/

static int adxl362_thread(int argc, FAR char **argv)
{
  FAR struct adxl362_sensor_s *priv
      = (FAR struct adxl362_sensor_s *)((uintptr_t)strtoul(argv[1], NULL,
                                                           16));
  struct sensor_accel accel;
  uint8_t             data[8];
  int                 ret;

  while (true)
    {
      if ((!priv->enabled))
        {
          /* Waiting to be woken up */

          ret = nxsem_wait(&priv->run);
          if (ret < 0)
            {
              continue;
            }
        }

      /* Read accel */

      if (priv->enabled)
        {
          /* Wait for data ready */

          while (!(adxl362_getreg8(priv, ADXL362_STATUS) & 0x01));

          adxl362_getregs(priv, ADXL362_XDATA_L, (FAR uint8_t *)data, 8);

          accel.timestamp   = sensor_get_timestamp();
          accel.x           = (float)adxl362_data(&data[0]) * priv->scale;
          accel.y           = (float)adxl362_data(&data[2]) * priv->scale;
          accel.z           = (float)adxl362_data(&data[4]) * priv->scale;
          accel.temperature =
            (float)adxl362_data(&data[6]) * ADXL362_TEMP_SCALE;

          priv->lower.push_event(priv->lower.priv, &accel, sizeof(accel));
        }

      /* Sleeping thread before fetching the next sensor data */

      nxsig_usleep(priv->interval);
    }

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: adxl362_register
 *
 * Description:
 *   Register the ADXL362 character device as 'devpath'
 *
 * Input Parameters:
 *   devno   - Sensor device number.
 *   dev     - An instance of the SPI or I2C interface to use to communicate
 *             with ADXL362
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int adxl362_register(int devno, FAR struct spi_dev_s *spi)
{
  FAR struct adxl362_sensor_s *priv;
  int                          ret;
#ifdef CONFIG_SENSORS_ADXL362_POLL
  FAR char                    *argv[2];
  char                         arg1[32];
#endif

  /* Sanity check */

  DEBUGASSERT(spi != NULL);

  /* Initialize the ADXL362 device structure */

  priv = kmm_zalloc(sizeof(struct adxl362_sensor_s));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  priv->spi           = spi;
  priv->lower.ops     = &g_adxl362_accel_ops;
  priv->lower.type    = SENSOR_TYPE_ACCELEROMETER;
  priv->lower.nbuffer = 1;
  priv->scale         = (CONSTANTS_ONE_G / 1000.0f);
  priv->devno         = devno;
#ifdef CONFIG_SENSORS_ADXL362_POLL
  priv->enabled       = false;
  priv->interval      = CONFIG_SENSORS_ADXL362_POLL_INTERVAL;

  nxsem_init(&priv->run, 0, 0);
#endif

  /* Read and verify the deviceid */

  ret = adxl362_checkid(priv);
  if (ret < 0)
    {
      snerr("Wrong Device ID!\n");
      kmm_free(priv);
      return ret;
    }

  /* Soft reset */

  adxl362_reset(priv);

  /* Register the character driver */

  ret = sensor_register(&priv->lower, devno);
  if (ret < 0)
    {
      snerr("Failed to register accel driver: %d\n", ret);
      kmm_free(priv);
    }

#ifdef CONFIG_SENSORS_ADXL362_POLL
  /* Create thread for polling sensor data */

  snprintf(arg1, 16, "%p", priv);
  argv[0] = arg1;
  argv[1] = NULL;

  ret = kthread_create("adxl362_thread", SCHED_PRIORITY_DEFAULT,
                       CONFIG_SENSORS_ADXL362_THREAD_STACKSIZE,
                       adxl362_thread,
                       argv);
  if (ret < 0)
    {
      kmm_free(priv);
    }
#endif

  return ret;
}
