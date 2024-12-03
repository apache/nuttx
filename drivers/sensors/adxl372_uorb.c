/****************************************************************************
 * drivers/sensors/adxl372_uorb.c
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

#include <debug.h>
#include <stdio.h>
#include <string.h>
#include <sys/param.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/signal.h>
#include <nuttx/sensors/adxl372.h>

#include <nuttx/sensors/sensor.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CONSTANTS_ONE_G 9.8f

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct adxl372_sensor_s
{
  struct sensor_lowerhalf_s  lower;
  FAR struct spi_dev_s      *spi;
  float                      scale;
  int                        devno;
#ifdef CONFIG_SENSORS_ADXL372_POLL
  bool                       enabled;
  uint32_t                   interval;
  sem_t                      run;
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static uint8_t adxl372_getreg8(FAR struct adxl372_sensor_s *priv,
                              uint8_t regaddr);
static void adxl372_putreg8(FAR struct adxl372_sensor_s *priv,
                            uint8_t regaddr, uint8_t regval);
static void adxl372_getregs(FAR struct adxl372_sensor_s *priv,
                            uint8_t regaddr, FAR uint8_t *regval, int len);
static int16_t adxl372_data(FAR uint8_t *data);
static int adxl372_checkid(FAR struct adxl372_sensor_s *priv);
static void adxl372_start(FAR struct adxl372_sensor_s *priv);
static void adxl372_stop(FAR struct adxl372_sensor_s *priv);

/* Sensor ops functions */

static int adxl372_activate(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep,
                            bool enable);
static int adxl372_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                FAR struct file *filep,
                                FAR uint32_t *period_us);
#ifndef CONFIG_SENSORS_ADXL372_POLL
static int adxl372_fetch(FAR struct sensor_lowerhalf_s *lower,
                         FAR struct file *filep,
                         FAR char *buffer, size_t buflen);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_adxl372_accel_ops =
{
  NULL,                 /* open */
  NULL,                 /* close */
  adxl372_activate,
  adxl372_set_interval,
  NULL,                 /* batch */
#ifdef CONFIG_SENSORS_ADXL372_POLL
  NULL,                 /* fetch */
#else
  adxl372_fetch,
#endif
  NULL,                 /* flush */
  NULL,                 /* selftest */
  NULL,                 /* set_calibvalue */
  NULL,                 /* calibrate */
  NULL                  /* get_info */
  NULL                  /* control */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: adxl372_getreg8
 *
 * Description:
 *   Read from an 8-bit ADXL372 register
 *
 ****************************************************************************/

static uint8_t adxl372_getreg8(FAR struct adxl372_sensor_s *priv,
                               uint8_t regaddr)
{
  uint8_t regval = 0;

  SPI_LOCK(priv->spi, true);

  SPI_SETFREQUENCY(priv->spi, ADXL372_SPI_FREQUENCY);
  SPI_SETMODE(priv->spi, ADXL372_SPI_MODE);

  /* Select the ADXL372 */

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER(priv->devno), true);

  /* Send register to read and get the next byte */

  SPI_SEND(priv->spi, (regaddr << 1) | ADXL372_READ);
  SPI_RECVBLOCK(priv->spi, &regval, 1);

  /* Deselect the ADXL372 */

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER(priv->devno), false);

  /* Unlock bus */

  SPI_LOCK(priv->spi, false);

  return regval;
}

/****************************************************************************
 * Name: adxl372_putreg8
 *
 * Description:
 *   Write a value to an 8-bit ADXL372 register
 *
 ****************************************************************************/

static void adxl372_putreg8(FAR struct adxl372_sensor_s *priv,
                            uint8_t regaddr, uint8_t regval)
{
  SPI_LOCK(priv->spi, true);

  SPI_SETFREQUENCY(priv->spi, ADXL372_SPI_FREQUENCY);
  SPI_SETMODE(priv->spi, ADXL372_SPI_MODE);

  /* Select the ADXL372 */

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER(priv->devno), true);

  /* Send register address and set the value */

  SPI_SEND(priv->spi, (regaddr << 1) | ADXL372_WRITE);
  SPI_SEND(priv->spi, regval);

  /* Deselect the ADXL372 */

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER(priv->devno), false);

  /* Unlock bus */

  SPI_LOCK(priv->spi, false);
}

/****************************************************************************
 * Name: adxl372_getregs
 *
 * Description:
 *   Read bytes from specified regaddr
 *
 ****************************************************************************/

static void adxl372_getregs(FAR struct adxl372_sensor_s *priv,
                            uint8_t regaddr, FAR uint8_t *regval, int len)
{
  /* If SPI bus is shared then lock and configure it */

  SPI_LOCK(priv->spi, true);

  SPI_SETFREQUENCY(priv->spi, ADXL372_SPI_FREQUENCY);
  SPI_SETMODE(priv->spi, ADXL372_SPI_MODE);

  /* Select the ADXL372 */

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER(priv->devno), true);

  /* Send register to read and get the next 2 bytes */

  SPI_SEND(priv->spi, (regaddr << 1) | ADXL372_READ);
  SPI_RECVBLOCK(priv->spi, regval, len);

  /* Deselect the ADXL372 */

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER(priv->devno), false);

  /* Unlock bus */

  SPI_LOCK(priv->spi, false);
}

/****************************************************************************
 * Name: adxl372_data
 ****************************************************************************/

static int16_t adxl372_data(FAR uint8_t *data)
{
  return ((int16_t)(((data[0] << 8) | (data[1] & 0xf0)))) >> 4;
}

/****************************************************************************
 * Name: adxl372_checkid
 *
 * Description:
 *   Read and verify the ADXL372 chip ID
 *
 ****************************************************************************/

static int adxl372_checkid(FAR struct adxl372_sensor_s *priv)
{
  uint8_t id = 0;

  id = adxl372_getreg8(priv, ADXL372_DEVID_AD);
  if (id != ADXL372_DEVID_AD_VALUE)
    {
      snerr("Wrong AD! %02x\n", id);
      return -ENODEV;
    }

  id = adxl372_getreg8(priv, ADXL372_DEVID_MST);
  if (id != ADXL372_DEVID_MST_VALUE)
    {
      snerr("Wrong MST! %02x\n", id);
      return -ENODEV;
    }

  id = adxl372_getreg8(priv, ADXL372_PARTID);
  if (id != ADXL372_PARTID_VALUE)
    {
      snerr("Wrong PARTID! %02x\n", id);
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: adxl372_start
 ****************************************************************************/

static void adxl372_start(FAR struct adxl372_sensor_s *priv)
{
  adxl372_putreg8(priv, ADXL372_POWER_CTL,
                  ADXL372_POWER_HPF_DISABLE |
                  ADXL372_POWER_MODE_MEASURE);
}

/****************************************************************************
 * Name: adxl372_stop
 ****************************************************************************/

static void adxl372_stop(FAR struct adxl372_sensor_s *priv)
{
  adxl372_putreg8(priv, ADXL372_POWER_CTL, 0);
}

/****************************************************************************
 * Name: adxl372_reset
 ****************************************************************************/

static void adxl372_reset(FAR struct adxl372_sensor_s *priv)
{
  int wdcnt = 10;

  /* Set stanby mode */

  adxl372_putreg8(priv, ADXL372_POWER_CTL, 0);

  /* Wait for boot to finish (15 ms error timeout) */

  up_mdelay(5);
  while (wdcnt > 0 && (0 != adxl372_getreg8(priv, ADXL372_RESET)))
    {
      up_mdelay(1);
      wdcnt--;
    }

  /* Reset ADXL372 Accelerometer. Write only. Begin a boot. */

  adxl372_putreg8(priv, ADXL372_RESET_VALUE, ADXL372_RESET);

  /* Wait for boot to finish (15 ms error timeout) */

  up_mdelay(5);
  wdcnt = 10;
  while (wdcnt > 0 && (0 != adxl372_getreg8(priv, ADXL372_RESET)))
    {
      up_mdelay(1);
      wdcnt--;
    }
}

/****************************************************************************
 * Name: adxl372_activate
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

static int adxl372_activate(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep,
                            bool enable)
{
  FAR struct adxl372_sensor_s *priv = (FAR struct adxl372_sensor_s *)lower;
#ifdef CONFIG_SENSORS_ADXL372_POLL
  bool start_thread = false;
#endif

  if (enable)
    {
#ifdef CONFIG_SENSORS_ADXL372_POLL
      if (!priv->enabled)
        {
          start_thread = true;
        }
#endif

      adxl372_start(priv);
    }
  else
    {
      adxl372_stop(priv);
    }

#ifdef CONFIG_SENSORS_ADXL372_POLL
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
 * Name: adxl372_set_interval
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

static int adxl372_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                FAR struct file *filep,
                                FAR uint32_t *period_us)
{
#ifdef CONFIG_SENSORS_ADXL372_POLL
  FAR struct adxl372_sensor_s *priv = (FAR struct adxl372_sensor_s *)lower;

  priv->interval = *period_us;
#endif

  return OK;
}

#ifndef CONFIG_SENSORS_ADXL372_POLL
/****************************************************************************
 * Name: adxl372_fetch
 ****************************************************************************/

static int adxl372_fetch(FAR struct sensor_lowerhalf_s *lower,
                         FAR struct file *filep, FAR char *buffer,
                         size_t buflen)
{
  FAR struct adxl372_sensor_s *priv = (FAR struct adxl372_sensor_s *)lower;
  struct sensor_accel          accel;
  uint8_t                      data[6];

  /* Wait for data ready */

  while (!(adxl372_getreg8(priv, ADXL372_STATUS) & 0x01));

  /* Get data */

  adxl372_getregs(priv, ADXL372_XDATA_H, (FAR uint8_t *)data, 6);

  accel.timestamp   = sensor_get_timestamp();
  accel.x           = (float)adxl372_data(&data[0]) * priv->scale;
  accel.y           = (float)adxl372_data(&data[2]) * priv->scale;
  accel.z           = (float)adxl372_data(&data[4]) * priv->scale;
  accel.temperature = 0;

  memcpy(buffer, &accel, sizeof(accel));

  return sizeof(accel);
}
#endif

#ifdef CONFIG_SENSORS_ADXL372_POLL
/****************************************************************************
 * Name: adxl372_thread
 *
 * Description: Thread for performing interval measurement cycle and data
 *              read.
 *
 * Parameter:
 *   argc - Number opf arguments
 *   argv - Pointer to argument list
 *
 ****************************************************************************/

static int adxl372_thread(int argc, FAR char **argv)
{
  FAR struct adxl372_sensor_s *priv
      = (FAR struct adxl372_sensor_s *)((uintptr_t)strtoul(argv[1], NULL,
                                                           16));
  struct sensor_accel accel;
  uint8_t             data[6];
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

          while (!(adxl372_getreg8(priv, ADXL372_STATUS) & 0x01));

          adxl372_getregs(priv, ADXL372_XDATA_H, (FAR uint8_t *)data, 6);

          accel.timestamp   = sensor_get_timestamp();
          accel.x           = (float)adxl372_data(&data[0]) * priv->scale;
          accel.y           = (float)adxl372_data(&data[2]) * priv->scale;
          accel.z           = (float)adxl372_data(&data[4]) * priv->scale;
          accel.temperature = 0;

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
 * Name: adxl372_register
 *
 * Description:
 *   Register the ADXL372 character device as 'devpath'
 *
 * Input Parameters:
 *   devno   - Sensor device number.
 *   dev     - An instance of the SPI or I2C interface to use to communicate
 *             with ADXL372
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int adxl372_register_uorb(int devno, FAR struct spi_dev_s *spi)
{
  FAR struct adxl372_sensor_s *priv;
  int                          ret;
#ifdef CONFIG_SENSORS_ADXL372_POLL
  FAR char                    *argv[2];
  char                         arg1[32];
#endif

  /* Sanity check */

  DEBUGASSERT(spi != NULL);

  /* Initialize the ADXL372 device structure */

  priv = kmm_zalloc(sizeof(struct adxl372_sensor_s));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  priv->spi           = spi;
  priv->lower.ops     = &g_adxl372_accel_ops;
  priv->lower.type    = SENSOR_TYPE_ACCELEROMETER;
  priv->lower.nbuffer = 1;
  priv->scale         = (CONSTANTS_ONE_G / 10.0f);
  priv->devno         = devno;
#ifdef CONFIG_SENSORS_ADXL372_POLL
  priv->enabled       = false;
  priv->interval      = CONFIG_SENSORS_ADXL372_POLL_INTERVAL;

  nxsem_init(&priv->run, 0, 0);
#endif

  /* Read and verify the deviceid */

  ret = adxl372_checkid(priv);
  if (ret < 0)
    {
      snerr("Wrong Device ID!\n");
      kmm_free(priv);
      return ret;
    }

  /* Reset device */

  adxl372_reset(priv);

  /* Register the character driver */

  ret = sensor_register(&priv->lower, devno);
  if (ret < 0)
    {
      snerr("Failed to register accel driver: %d\n", ret);
      kmm_free(priv);
    }

#ifdef CONFIG_SENSORS_ADXL372_POLL
  /* Create thread for polling sensor data */

  snprintf(arg1, 16, "%p", priv);
  argv[0] = arg1;
  argv[1] = NULL;

  ret = kthread_create("adxl372_thread", SCHED_PRIORITY_DEFAULT,
                       CONFIG_SENSORS_ADXL372_THREAD_STACKSIZE,
                       adxl372_thread,
                       argv);
  if (ret < 0)
    {
      kmm_free(priv);
    }
#endif

  return ret;
}
