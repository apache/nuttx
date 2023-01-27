/****************************************************************************
 * drivers/sensors/ltr308.c
 * Character driver for the LTR-308ALS-01 Lite-On ambient light sensor.
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
#include <stdlib.h>
#include <debug.h>

#include <nuttx/nuttx.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/signal.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/sensors/ltr308.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_LTR308)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LTR308_ADDR           0x53
#define DEVID                 0xB1

#define LTR308_CTRL           0x00
#define LTR308_MEAS_RATE      0x04
#define LTR308_ALS_GAIN       0x05
#define LTR308_PART_ID        0x06
#define LTR308_STATUS         0x07
#define LTR308_DATA_0         0x0D

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

struct ltr308_sensor_s
{
  struct sensor_lowerhalf_s lower;  /* Common lower interface */
  uint8_t integration_time;         /* The interval between 2 ALS cycles */
  uint8_t measurement_rate;         /* The interval between 2 data updates */
  uint8_t gain;                     /* Sensor gain */
};

struct ltr308_dev_s
{
  struct ltr308_sensor_s dev;     /* Sensor private data */
  FAR struct i2c_master_s *i2c;   /* I2C interface */
  mutex_t dev_lock;               /* Manages exclusive access to the device */
  sem_t run;                      /* Locks sensor thread */
  bool enabled;                   /* Enable/Disable LTR308 */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int ltr308_activate(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep, bool enabled);
static int ltr308_calibrate(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep,
                            unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_sensor_ops =
{
  NULL,               /* open */
  NULL,               /* close */
  ltr308_activate,    /* activate */
  NULL,               /* set_interval */
  NULL,               /* batch */
  NULL,               /* fetch */
  NULL,               /* selftest */
  NULL,               /* set_calibvalue */
  ltr308_calibrate,   /* calibrate */
  NULL                /* control */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ltr308_set_reg8
 *
 * Description:
 *   Write to an 8-bit LTR308 register
 *
 * Return value:
 *   Zero (OK) on success; a negated errno value on failure
 ****************************************************************************/

static int ltr308_set_reg8(FAR struct ltr308_dev_s *priv, uint8_t regaddr,
                           uint8_t regval)
{
  struct i2c_msg_s msg;
  uint8_t txbuffer[2];
  int ret;

  txbuffer[0]   = regaddr;
  txbuffer[1]   = regval;

  msg.frequency = CONFIG_SENSORS_LTR308_I2C_FREQUENCY;
  msg.addr      = LTR308_ADDR;
  msg.flags     = 0;
  msg.buffer    = txbuffer;
  msg.length    = 2;

  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  if (ret < 0)
    {
      snerr("I2C_TRANSFER failed (err = %d)\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: ltr308_get_reg8
 *
 * Description:
 *   Read from an 8-bit LTR308 register
 *
 * Return value:
 *   Zero (OK) on success; a negated errno value on failure
 ****************************************************************************/

static int ltr308_get_reg8(FAR struct ltr308_dev_s *priv, uint8_t regaddr,
                           FAR uint8_t *regval)
{
  struct i2c_msg_s msg[2];
  int ret;

  msg[0].frequency = CONFIG_SENSORS_LTR308_I2C_FREQUENCY;
  msg[0].addr      = LTR308_ADDR;
  msg[0].flags     = 0;
  msg[0].buffer    = &regaddr;
  msg[0].length    = 1;

  msg[1].frequency = CONFIG_SENSORS_LTR308_I2C_FREQUENCY;
  msg[1].addr      = LTR308_ADDR;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = regval;
  msg[1].length    = 1;

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  if (ret < 0)
    {
      snerr("ERROR: I2C_TRANSFER failed (err = %d)\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: ltr308_get_reg24
 *
 * Description:
 *   Read from 3 8-bit LTR308 registers
 *
 * Return value:
 *   Zero (OK) on success; a negated errno value on failure
 ****************************************************************************/

static int ltr308_get_reg24(FAR struct ltr308_dev_s *priv, uint8_t regaddr,
                            FAR uint32_t *val)
{
  int ret;
  int i;

  *val = 0;
  for (i = 0; i < 3; i++, regaddr++)
    {
      ret = ltr308_get_reg8(priv, regaddr, ((uint8_t *)val) + i);
      if (ret < 0)
        {
          return ret;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: ltr308_checkid
 *
 * Description:
 *   Read and verify the LTR308 chip ID
 *
 * Return value:
 *   Zero (OK) on success; a negated errno value on failure
 ****************************************************************************/

static int ltr308_checkid(FAR struct ltr308_dev_s *priv)
{
  uint8_t devid;
  int ret;

  /* Read device ID */

  ret = ltr308_get_reg8(priv, LTR308_PART_ID, &devid);
  if (ret < 0)
    {
      return ret;
    }

  sninfo("devid: 0x%02x\n", devid);

  return (devid != DEVID) ? -ENODEV : OK;
}

/****************************************************************************
 * Name: ltr308_get_status
 *
 * Description:
 *   Get the status information of LTR308
 *
 * Return value:
 *   Zero (OK) on success; a negated errno value on failure
 ****************************************************************************/

static int ltr308_get_status(FAR struct ltr308_dev_s *priv,
                             FAR bool *power_on,
                             FAR bool *interrupt_pending,
                             FAR bool *data_pending)
{
  uint8_t status;
  uint8_t ret;

  ret = ltr308_get_reg8(priv, LTR308_STATUS, &status);
  if (ret < 0)
    {
      return ret;
    }

  if (power_on != NULL)
    {
      *power_on = status & 0x20;
    }

  if (interrupt_pending != NULL)
    {
      *interrupt_pending = status & 0x10;
    }

  if (data_pending != NULL)
    {
      *data_pending = status & 0x08;
    }

  return ret;
}

/****************************************************************************
 * Name: ltr308_get_lux
 *
 * Description:
 *   Convert raw data to lux
 *   @gain: see ltr308_calibrate()
 *   @integration_time: see ltr308_calibrate()
 *   @ch: result from ltr308_get_data()
 *
 * Return value:
 *   Zero (OK) on success; a negated errno value on failure
 ****************************************************************************/

static int ltr308_get_lux(FAR struct ltr308_dev_s *priv, uint8_t gain,
                          uint8_t integration_time, uint32_t data,
                          FAR float *lux)
{
  float d0;

  /* Determine if sensor is saturated. If so, abandon ship (calculation will
   * not be accurate)
   */

  if (data == 0x000fffff)
    {
      *lux = 0.0;
      return -ENODATA;
    }

  d0 = ((float)data * 0.6);
  switch (gain)
    {
      case 0:
        break;

      case 1:
        d0 /= 3;
        break;

      case 2:
        d0 /= 6;
        break;

      case 3:
        d0 /= 9;
        break;

      case 4:
        d0 /= 18;
        break;

      default:
        d0 = 0.0;
        break;
    }

  switch (integration_time)
    {
      case 0:
        *lux = d0 / 4;
        break;

      case 1:
        *lux = d0 / 2;
        break;

      case 2:
        *lux = d0;
        break;

      case 3:
        *lux = d0 * 2;
        break;

      case 4:
        *lux = d0 * 4;
        break;

      default:
        *lux = 0.0;
        break;
    }

  return 0;
}

/****************************************************************************
 * Name: ltr308_activate
 *
 * Description:
 *   Enable/Disable LTR308 sensor
 *
 * Return value:
 *   Zero (OK) on success; a negated errno value on failure
 ****************************************************************************/

static int ltr308_activate(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep, bool enabled)
{
  FAR struct ltr308_sensor_s *dev = container_of(lower,
                                                 FAR struct ltr308_sensor_s,
                                                 lower);
  FAR struct ltr308_dev_s *priv = container_of(dev,
                                               FAR struct ltr308_dev_s,
                                               dev);
  int ret;

  ret = ltr308_set_reg8(priv, LTR308_CTRL, enabled << 1);
  if (ret < 0)
    {
      snerr("ERROR: Could not enable sensor\n");
      return ret;
    }

  priv->enabled = enabled;
  if (enabled == true)
    {
      /* Wake up the polling thread */

      nxsem_post(&priv->run);
    }

  return OK;
}

/****************************************************************************
 * Name: ltr308_calibrate
 *
 * Description:
 *   Set the integration time, measurement rate and gain of LTR308
 *   @integration_time - the measurement time for each ALS cycle
 *   @measurement_rate - the interval between DATA_REGISTERS update. Must be
 *                       set to be equal or greater than integration time
 *   @gain             - the sensor gain
 *
 * Return value:
 *   Zero (OK) on success; a negated errno value on failure
 ****************************************************************************/

static int ltr308_calibrate(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep,
                            unsigned long arg)
{
  FAR struct ltr308_sensor_s *dev = container_of(lower,
                                                 FAR struct ltr308_sensor_s,
                                                 lower);
  FAR struct ltr308_dev_s *priv = container_of(dev,
                                               FAR struct ltr308_dev_s,
                                               dev);
  FAR struct ltr308_calibval *calibval =
        (FAR struct ltr308_calibval *) arg;
  uint8_t measurement = 0;
  int ret;

  /* Sanity checks */

  if (calibval->integration_time >= 5)
    {
      calibval->integration_time = 0;
    }

  if (calibval->measurement_rate >= 6 || calibval->measurement_rate == 4)
    {
      calibval->measurement_rate = 0;
    }

  if (calibval->gain >= 5)
    {
      calibval->gain = 0;
    }

  /* Do not overwrite if the values have not been modified */

  nxmutex_lock(&priv->dev_lock);

  if (dev->integration_time != calibval->integration_time ||
      dev->measurement_rate != calibval->measurement_rate)
    {
      dev->integration_time = calibval->integration_time;
      dev->measurement_rate = calibval->measurement_rate;

      measurement |= calibval->integration_time << 4;
      measurement |= calibval->measurement_rate;

      ret = ltr308_set_reg8(priv, LTR308_MEAS_RATE, measurement);
      if (ret < 0)
        {
          goto err_out;
        }
    }

  if (dev->gain != calibval->gain)
    {
      dev->gain = calibval->gain;

      ret = ltr308_set_reg8(priv, LTR308_ALS_GAIN, calibval->gain);
      if (ret < 0)
        {
          goto err_out;
        }
    }

  nxmutex_unlock(&priv->dev_lock);
  return OK;

err_out:
  nxmutex_unlock(&priv->dev_lock);
  snerr("ERROR: Failed to calibrate sensor\n");
  return ret;
}

/****************************************************************************
 * Name: ltr308_thread
 *
 * Description:
 *   Thread for performing data readings at fixed intervals, defined by
 *   CONFIG_SENSORS_LTR308_POLL_INTERVAL
 *   @argc - Number of arguments
 *   @argv - Pointer to argument list
 *
 * Return value:
 *   Zero (OK) on success; a negated errno value on failure
 ****************************************************************************/

static int ltr308_thread(int argc, char** argv)
{
  FAR struct ltr308_dev_s *priv = (FAR struct ltr308_dev_s *)
        ((uintptr_t)strtoul(argv[1], NULL, 0));
  struct sensor_light light;
  uint32_t data = 0;
  bool data_pending;
  float lux;
  int ret;

  while (true)
    {
      if (priv->enabled == false)
        {
          /* Wait for the sensor to be enabled */

          nxsem_wait(&priv->run);
        }

      /* Read the data registers and sleep if no change is detected
       * or the sensor is saturated
       */

      ret = ltr308_get_status(priv, NULL, NULL, &data_pending);
      if (ret < 0)
        {
          snerr("ERROR: Failed to read sensor's status\n");
          return ret;
        }

      if (data_pending == false)
        {
          goto thread_sleep;
        }

      ret = ltr308_get_reg24(priv, LTR308_DATA_0, &data);
      if (ret < 0)
        {
          snerr("ERROR: Failed to read data from sensor\n");
          return ret;
        }

      ret = ltr308_get_lux(priv, priv->dev.gain, priv->dev.integration_time,
                           data, &lux);
      if (ret < 0)
        {
          goto thread_sleep;
        }

      light.timestamp = sensor_get_timestamp();
      light.light = lux;
      priv->dev.lower.push_event(priv->dev.lower.priv, &light,
                                 sizeof(struct sensor_light));

thread_sleep:
      nxsig_usleep(CONFIG_SENSORS_LTR308_POLL_INTERVAL);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ltr308_register
 *
 * Description:
 *   Register the LTR308 character device
 *   @devno - The user-specified device number, starting from 0
 *   @i2c   - An instance of the I2C interface to use to communicate with
 *           LTR308
 *
 * Return value:
 *   Zero (OK) on success; a negated errno value on failure
 ****************************************************************************/

int ltr308_register(int devno, FAR struct i2c_master_s *i2c)
{
  FAR struct sensor_lowerhalf_s *lower;
  FAR struct ltr308_dev_s *priv;
  FAR char *argv[2];
  char arg1[32];
  int ret = OK;

  DEBUGASSERT(i2c != NULL);

  /* Initialize the LTR308 device structure */

  priv = (FAR struct ltr308_dev_s *)kmm_zalloc(sizeof(struct ltr308_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance (err = %d)\n", ret);
      return -ENOMEM;
    }

  priv->i2c = i2c;
  priv->enabled = false;
  nxmutex_init(&priv->dev_lock);
  nxsem_init(&priv->run, 0, 0);

  /* Check Device ID */

  ret = ltr308_checkid(priv);
  if (ret < 0)
    {
      snerr("ERROR: Wrong device ID!\n");
      goto err_init;
    }

  /* Register the character driver */

  lower = &priv->dev.lower;
  lower->ops = &g_sensor_ops;
  lower->type = SENSOR_TYPE_LIGHT;

  ret = sensor_register(lower, 0);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver (err = %d)\n", ret);
      goto err_init;
    }

  /* Create thread for polling sensor data */

  snprintf(arg1, 16, "0x%" PRIxPTR, (uintptr_t)priv);
  argv[0] = arg1;
  argv[1] = NULL;
  ret = kthread_create("ltr308_thread", SCHED_PRIORITY_DEFAULT,
                       CONFIG_SENSORS_LTR308_THREAD_STACKSIZE,
                       ltr308_thread, argv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to create poll thread (err = %d)\n", ret);
      goto err_register;
    }

  sninfo("LTR308 driver loaded successfully!\n");
  return OK;

err_register:
  sensor_unregister(lower, 0);
err_init:
  nxsem_destroy(&priv->run);
  nxmutex_destroy(&priv->dev_lock);
  kmm_free(priv);
  return ret;
}

#endif /* CONFIG_I2C && CONFIG_SENSORS_LTR308 */
